# Plan: bus Dynamixel compartido entre las ruedas y el brazo

Acordado el 2026-11-01 (sesión de preguntas, rondas 1–3). Revisado el mismo día tras validarlo contra el driver actual y
la Pi (commit 936a698: el bucle de motores ya corre en un hilo propio); los cambios están marcados con **(rev.)**.
**Este documento es solo el plan: no hay código ni cambios en el robot.** Contexto del brazo: [gripper-plan.md](gripper-plan.md). Tornillo: [tornillo.md](tornillo.md).

## Problema

Las ruedas (IDs 2 y 4), el tornillo (3) y el gripper (5) cuelgan del mismo bus (`/dev/dynamixel`, 57600 baud). El driver
C++ de las ruedas (`ros/wallens_driver`) abre el puerto y no lo suelta. Un nodo del brazo en otro proceso no puede
abrirlo a la vez. No hay un segundo adaptador USB, así que el puerto tiene un solo dueño y el brazo lo usa por él.

## Decisión

El driver C++ sigue siendo el único dueño del puerto y ofrece un **servicio de transacciones** al brazo. `Brazo`
(Python, ya probado con 34 tests) no cambia de lógica: se le da un `BusRos` que cumple la misma interfaz
(`ping`, `leer`, `escribir`) en lugar de `SdkBus`.

**(rev.)** El callback del servicio **no toca el puerto**. Valida la llamada (lista blanca, presupuesto), la mete en una
cola protegida por mutex y espera su resultado con un `promise`/`future` con timeout. El **hilo de motores** (`hilo_motores_`,
tiempo real, núcleo 2) vacía la cola justo después de `ciclo()` y antes de `sleep_until`. Así hay un solo hilo que
habla con el puerto y no hay carrera con el SDK. Costo: la latencia de una llamada del brazo es de hasta un ciclo (~33 ms),
aceptable para el brazo.

Se descartaron: brazo dentro del driver C++ (hilo propio, mutex y reescribir las pruebas, sin ganancia porque el límite es
el cable), nodo de bus aparte (no hay más clientes) y un segundo adaptador (no se tiene).
Si más adelante el brazo se porta a C++, `Bus` sigue siendo la frontera y solo cambia un lado.

## Qué se comprobó en el código

- **(rev.)** `ciclo()` corre en `hilo_motores_` (`wallens_driver.cpp:192`, `:244-246`); `main` solo hace `spin` para
  suscripciones y `publicar_pendiente`. El plan original suponía un executor de un solo hilo sin mutex: **es falso**. Un
  callback de servicio ejecutando bus competiría por `port_` con el hilo de tiempo real. `PortHandler::is_using_` no es
  atómico, así que habría `COMM_PORT_BUSY` o paquetes intercalados, y el error caería sobre las ruedas (`runtime_error` y
  suma a `fallos_`). De ahí la cola.
- Cualquier `Fatal`, o 5 fallos seguidos, apaga el driver y cierra el puerto, ahora desde el hilo de motores
  (`:263-277`). `apagar()` solo conoce las ruedas y hace `join()` del hilo antes de nada (`:200-201`).
- **(rev.)** El executor llegó a bloquearse ~350 ms en la capa DDS con SLAM, Nav2 y el PC conectado (comentario en
  `:188-190`). La respuesta del servicio viaja por ese executor, así que el timeout del cliente no puede ser de 100 ms.
- **(rev.)** `revisar_watchdog` y `revisar_salud` hacen dos lecturas sueltas (~10 ms) una vez por segundo
  (`ciclos_ % loop_hz == 0`, `:453`).
- El watchdog del bus que `Brazo` arma en sus motores es de 200 ms (`watchdog = 10`).

## Presupuesto de bus (corrección de una estimación anterior)

A 57600 baud un byte tarda 0.174 ms (8N1). El tiempo de cable de un paquete de lectura es `(14 + 11 + n) bytes`, más el
tiempo de retorno del motor (~0.5 ms):

| Operación | Bytes | Tiempo |
|---|---|---|
| Lectura suelta de 1–4 bytes | ~27–29 | **~5 ms** (no 2–3 ms, como se dijo en la conversación) |
| Lectura en bloque 126..146 de un motor (21 bytes) | 46 | ~8.5 ms |
| Ciclo de las ruedas hoy (sync write 24 bytes + sync read de 2 motores, 80 bytes) | ~104 | **~19–20 ms de los 33 ms** |

Quedan **~13 ms por ciclo** para el brazo: dos lecturas sueltas, o un bloque de un motor. **(rev.)** Excepción: en el
ciclo en que corre `revisar_watchdog`/`revisar_salud` (una vez por segundo) ya se gastan ~10 ms más, y ahí no se atiende al
brazo. Hoy `Brazo._vigilar()` hace
3 lecturas sueltas por motor, 6 en total (~30 ms), y no cabe. Consecuencias:

1. `Brazo` debe leer **un bloque por motor** (126..146: corriente, velocidad, posición, voltaje, temperatura) y reutilizarlo
   en esa iteración. El error de hardware (70) no necesita lectura aparte: viene en el byte de estado de cada respuesta.
2. Con ruedas y brazo activos a la vez, la lectura de salud del brazo baja a ~10 Hz alternando los dos motores.
3. **(rev.)** Opción a medir más adelante: leer la salud del brazo con un `GroupSyncRead` extra dentro del ciclo (bloque
   126..146 de los IDs 3 y 5, cada 3 ciclos, ~9 ms más en un ciclo de ~20). No se hace sin medir.
4. **Subir el baudrate** (57600 → 1 Mbps) reduce todo ~17 veces y quita el problema de raíz. Es una escritura en EEPROM de
   los 4 motores y del adaptador, y toca también al driver de las ruedas. No se hace sin mediciones; se decide en el paso 6
   con los números reales.

El límite de tiempo por llamada (abajo) se calcula con esta misma cuenta, así que cambia solo si cambia el baudrate.

## El servicio

Paquete nuevo `wallens_interfaces` (ament_cmake + rosidl):

```
# msg/OperacionDxl.msg
uint8 tipo        # 0 = leer, 1 = escribir, 2 = ping
uint8 id
uint16 direccion
uint8 largo       # 1, 2 o 4
uint32 valor      # solo en escrituras

# msg/ResultadoDxl.msg
uint8 estado      # 0 ok, 1 no ejecutada, 2 rechazada (lista blanca), 3 error de comunicación, 4 error del motor
int8 comunicacion # código COMM del SDK
uint8 error_motor # byte de error del paquete de estado (bit 7 = alerta de hardware)
uint32 valor      # lectura (el bloque largo se entrega en `datos`)
uint8[] datos     # lecturas en bloque

# srv/TransaccionDxl.srv
OperacionDxl[] operaciones
---
bool aceptada
string motivo     # por qué se rechazó toda la llamada (presupuesto, demasiadas operaciones, bus apagado)
ResultadoDxl[] resultados
float32 tiempo_bus_ms
```

Semántica:

- Las operaciones se ejecutan **en orden** y se **para en la primera que falla**; las siguientes quedan "no ejecutada".
  Es lo que `Brazo` ya supone (la velocidad 112 va antes que la aceleración 108; el torque off antes de cambiar el modo).
- Máximo ~8 operaciones por llamada. Las lecturas de bloque (`largo` hasta 32) usan `datos`.
- La alerta de hardware (bit 7) se devuelve tal cual; el servicio **no** apaga el driver por ella.
- **Lista blanca** (parámetro YAML `brazo.ids` y `brazo.direcciones`): solo IDs 3 y 5; solo las direcciones que `Brazo`
  usa (11, 38, 48, 52, 64, 70, 98, 102, 108, 112, 116, 126–146, 222). ID (7) y baudrate (8) quedan prohibidos siempre. Una
  dirección fuera de la lista da estado 2 en esa operación y corta la llamada. Un test comprueba que cada registro que
  usa `Brazo` está en la lista.
- **Presupuesto**: antes de ejecutar se estima el tiempo de cable de la llamada (bytes ÷ baudios + retorno). Si pasa de
  `brazo.max_ms` (12 ms por defecto) se rechaza con `PRESUPUESTO` y el cliente la divide. **(rev.)** El hilo de motores
  también respeta ese tope por ciclo al vaciar la cola (si quedan llamadas, esperan al ciclo siguiente) y no atiende al
  brazo en el ciclo de `revisar_watchdog`/`revisar_salud`.
- **Aislamiento de fallos del brazo**: un error de comunicación o de hardware en los IDs 3 o 5 se devuelve al cliente y
  **nunca** lanza `Fatal` ni suma a `max_failures`. El `Fatal` de las ruedas se mantiene como hoy. **(rev.)** La
  ejecución del brazo lleva su propio `try/catch`, fuera del `catch` de `ciclo()`: no toca `fallos_`, `Fatal` ni
  `rearme_hasta_` (la lógica de rearme del watchdog sigue siendo solo de las ruedas); el estado 2/3/4 va en el resultado.
- **(rev.) Cola y apagado**: la cola tiene tamaño máximo (llena = `aceptada=false`, "cola llena"). Al apagar se pone
  `corriendo_=false`, se vacía la cola respondiendo "bus apagado" a cada espera y solo entonces se hace el `join()`, para
  que ningún cliente quede colgado.
- **Diagnóstico**: contador de transacciones, tiempo de bus acumulado y ciclos de las ruedas que se atrasaron mientras el
  servicio estaba activo, publicados en `/bus/diagnostico` (el aviso de `dt > 3 / loop_hz` ya existe).
- **Cuando el driver muere**: `apagar()` no toca los IDs 3 y 5. La red de seguridad es el watchdog del bus de esos motores.
  Qué hace el XM430 al dispararse en modo 5 (soltar el torque o mantener) se mide en la etapa 0.

## Cambios en `wallens_driver` (sin romper las ruedas)

- Código nuevo en `src/bus_brazo.hpp`: una clase que recibe `PortHandler` y `PacketHandler` por constructor, valida la
  lista blanca, estima el presupuesto y ejecuta las operaciones. Sin ROS dentro, para probarla con gtest.
  **(rev.)** Incluye la cola (`encolar` desde cualquier hilo, `atender(presupuesto_ms)` solo desde el hilo de motores,
  `cerrar()` que vacía rechazando), también sin ROS, para probar con gtest las carreras y el apagado.
- En `wallens_driver.cpp` solo se agrega: crear el servicio si `brazo_habilitado` es `true`, la llamada a `atender()` en
  `bucle_motores` tras `ciclo()`, `cerrar()` en `apagar()` antes del `join()`, y el contador de ciclos tardíos.
- **Parámetro `brazo_habilitado`, `false` por defecto.** Apagado, el driver se comporta exactamente como hoy y no existe
  ningún servicio. Es la red de seguridad: si algo falla se apaga sin recompilar.
- Dependencias nuevas: `wallens_interfaces` y `ament_cmake_gtest`.

## Cliente Python: `BusRos`

Mismo contrato que `SdkBus` (`ping`, `leer`, `escribir`), más `leer_bloque(id, addr, n)` para las lecturas de salud.

- **(rev.)** Espera máxima por llamada: **~400 ms** por defecto (parámetro). Los 100 ms originales darían falsos abortos
  duros: el executor llegó a bloquearse ~350 ms por DDS. Como el watchdog del bus de los motores del brazo es de 200 ms,
  hay que subirlo (`Brazo.watchdog`) o que el hilo de motores renueve su keepalive con una lectura de 1 byte por segundo.
  Se decide con la medición del paso 6. Si no hay respuesta, `ErrorBus`, que `Brazo` ya convierte en aborto duro.
- **Sin reintentos en escrituras.** Una lectura puede repetirse una vez (parámetro `reintentos`).
- Si el servicio no existe al arrancar, el nodo espera y rechaza todo con "sin bus".
- Timeout y reintentos son parámetros del nodo.

## Nodo `wallens_brazo` (Python, `rclpy`)

Envuelve a `Brazo` y es el único cliente del servicio.

| Interfaz | Función |
|---|---|
| Acción `Recoger` | baja, cierra, sube. Feedback con la fase; resultado `agarrado` o `sin_agarre` |
| Servicio `Soltar` | abre el gripper (con la meta congelada, ver gripper-plan.md) |
| Servicio `Calibrar` | homing o cero reusado. Explícito, nunca automático al arrancar. `Recoger` lo rechaza sin cero |
| Servicio `Apagar` / `Congelar` | paradas manuales |
| Tópico `/brazo/estado` (5 Hz) | calibrado, fase, posición del tornillo, posición y corriente de la pinza, objeto agarrado |
| Tópico `/brazo/ocupado` | `true` mientras el brazo se mueve o lleva un objeto |

Mientras no esté decidido el método de cero (tornillo roto), el homing por tope queda **desactivado por parámetro**.

### Seguridad

- **`/parada`** → aborto **suave**: congela donde está, con torque, y espera el rearme. No suelta el objeto.
- **Batería crítica o sin `/battery_state`** → aborto **duro**: torque off de ambos motores. El tornillo se sostiene solo.
- **Interbloqueo**: `/brazo/ocupado` hace que `wallens_safety` ponga `/cmd_vel_safe` en 0 mientras sea `true`; el nodo del
  brazo rechaza `Recoger` si las ruedas se mueven (odometría sobre un umbral en el YAML).
- Las pruebas nuevas con motores siguen preferiblemente con el bus directo y las ruedas apagadas (ver abajo).

## Cómo se prueban el brazo y los scripts

Con el driver dueño del puerto, `SdkBus` no puede abrirlo. Los scripts `prueba-brazo.py` y `diagnostico-pinza.py` aceptan
`--bus=sdk|ros`:

- `--bus=sdk` (servicio `wallens` parado, como hoy): para toda prueba nueva con motores. Con las ruedas apagadas hay menos riesgo.
- `--bus=ros` (servicio activo con `brazo_habilitado:=true`, ruedas energizadas a velocidad 0): para verificar la
  integración y medir la latencia real.

## Orden de trabajo

Ningún paso 1–5 mueve un motor. Los pasos 1–5 se pueden hacer con tests; el 6 es el primero que necesita el robot.

1. **Commit del brazo solo**: `scripts/brazo/`, `scripts/prueba-brazo.py`, `scripts/diagnostico-pinza.py`,
   `docs/gripper-plan.md` y este documento, sin los cambios de `wallens_safety`, `README.md` ni `docker/Dockerfile`.
2. **`wallens_interfaces`** con los dos `.msg` y el `.srv`.
3. **`bus_brazo.hpp` + gtest** (puerto falso: lista blanca, orden, parar en el primer fallo, presupuesto, aislamiento de
   fallos, alerta de hardware sin `Fatal`, **(rev.)** cola con varios hilos, cola llena, cierre que no deja clientes
   colgados), el servicio y el vaciado de la cola en el hilo de motores con `brazo_habilitado:=false`, y el contador de
   ciclos tardíos. Los tests actuales de `wallens_safety` y los del brazo deben seguir en verde.
4. **`BusRos` + `leer_bloque`** en Python, con un test contra una transacción simulada, y el cambio en `Brazo` para leer un
   bloque por motor por iteración (con test de regresión: no más de N lecturas por iteración).
5. **Nodo `wallens_brazo`**: acción, servicios, estado, `/parada`, batería, `/brazo/ocupado`, y el ajuste de `wallens_safety`.
6. **Verificación en el robot** (ruedas en el aire, `brazo_habilitado:=true`):
   - sin ciclos tardíos nuevos y con el odómetro igual que antes;
   - latencia real del servicio y tiempo de bus por llamada (`/bus/diagnostico`);
   - etapa 0 de la marca 0xCA11 en el registro 222;
   - comportamiento del XM430 al dispararse el watchdog en modo 5;
   - **con estos números se decide si hay que subir el baudrate**.
7. **Aparte, cuando se repare el tornillo**: método de cero sin golpear topes duros, y las etapas 1, 3, 4 y 5 de gripper-plan.md.

## Criterios de "listo"

- Con `brazo_habilitado:=false` el driver es indistinguible del actual (mismos tests, mismo odómetro, mismos avisos).
- Una operación fuera de la lista blanca nunca llega al bus.
- Un fallo en los IDs 3 o 5 nunca apaga las ruedas.
- Una llamada que no cabe en el presupuesto se rechaza sin tocar el bus.
- Todo cliente recibe un error dentro del timeout configurado (**(rev.)** ~400 ms, no 100 ms) si el servicio no responde.
- **(rev.)** Solo el hilo de motores toca el puerto: ningún callback de ROS llama al SDK.

## Riesgos y puntos abiertos

- **(rev.) Latencia y watchdog**: con la cola, cada llamada espera hasta un ciclo y DDS puede retrasar la respuesta; el
  watchdog de 200 ms del brazo puede dispararse sin que nada falle. Medir en el paso 6.
- **(rev.) Repo y Pi**: en la Pi `~/wallens` no es un repositorio git (los fuentes llegan por copia y `~/ros_ws/src` es un
  enlace simbólico). Antes de compilar el driver nuevo hay que copiar el repo y verificar que coincide.
- **Presupuesto de bus justo** a 57600 (~13 ms libres de 33). La mitigación está arriba (bloques y ~10 Hz); la solución
  real es subir el baudrate, pendiente de medir.
- **Tornillo roto**: bloquea las etapas 1, 3, 4 y 5 y la decisión del método de cero.
- **Marca 0xCA11 y watchdog del XM430 en modo 5**: sin verificar en hardware.
- **Estrategia B del gripper abierto**: sin verificar en el robot (`diagnostico-pinza.py`).
- **Ramas**: hoy hay cambios tuyos sin commit en `wallens_safety`, `README.md` y `docker/Dockerfile`; el paso 5 vuelve a
  tocar `wallens_safety`, así que conviene que esos cambios se commiteen antes.
