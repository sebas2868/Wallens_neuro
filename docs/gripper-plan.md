# Plan de pruebas del brazo (tornillo + gripper)

Acordado el 2026-11-01. Tornillo sin fin = ID 3 (sube y baja), gripper = ID 5. Detalle del tornillo en [tornillo.md](tornillo.md).

## Gripper nuevo (cambiado el 2026-11-03)

- **Recorrido**, medido a mano con el torque apagado (`mirar_pinza.py`, solo lectura): abierto del todo **2866**, dedos
  tocándose **3811**. El viejo era 2794 / 3851. Ya está en `scripts/brazo/brazo.py` (`ABIERTA`, `CERRADA`), con
  `CIERRE_MEDIDO` = 3811 provisional hasta que lo mida G0. `APRIETE_FRUTA` y `SOSTEN_FRUTA` siguen siendo **del viejo**
  hasta repetir la fase G (`scripts/prueba-pinza-frutas.py`).
- **REGLA: antes de mover la pinza, la tenaza va despegada del piso.** Los dedos nuevos llegan más abajo: con la tenaza
  en el tope de abajo rozan el piso, la pinza se frena y el brazo aborta ("los dedos se detuvieron lejos del cierre" o
  "corriente de la pinza > 100: algo la frena") sin nada entre los dedos. `prueba-pinza-frutas.py` sube el tornillo de
  a 300 ticks hasta que digas que pasa una hoja por debajo y guarda cuánto subió (`despegue_ticks`). Ese número es el
  que hay que sumarle a la altura de agarre de la misión (`Config.altura`, hoy 40 ticks sobre el tope de abajo).
- **Más fuerza para abrir:** con 120 (~320 mA) y aborto en 100, el nuevo se frenaba al abrir. La prueba usa
  `--corriente-abrir=200` (aborto en 180). Si la misión también lo necesita, hay que subir `corriente_pinza` y
  `corriente_aborto_p` en `Config`.

## Por qué este plan

Las primeras pruebas combinadas fallaron cuatro veces seguidas por código que nunca se había ejecutado: la aceleración
del perfil escrita antes que la velocidad (error 4), un despegue del tope que se olvidó, un parámetro que tapaba una
función y un homing a 23 rpm que encajó el tornillo en el tope. Ninguna rompió nada, pero fue suerte. Desde ahora el
código se prueba primero contra un bus falso y las pruebas con motores van por etapas.

## Código

- `scripts/brazo/`: módulo sin ROS. `Brazo(bus, cal)` con órdenes simples: `preparar`, `energizar`, `calibrar`, `bajar`,
  `subir`, `abrir`, `cerrar_y_apretar`, `recoger`, `soltar`, `congelar`, `apagar`.
- El módulo no abre el puerto: recibe un `bus` con `ping`, `leer` y `escribir`. Hoy es `SdkBus` (el SDK); en los tests,
  `BusFalso`. El nodo ROS del brazo solo llamará a estas órdenes (una acción `Recoger`, un servicio `Soltar`).
- `scripts/brazo/falso.py`: bus falso con los fallos reales (aceleración mayor que velocidad, EEPROM con torque, watchdog,
  tornillo que se encaja si llega rápido al tope, objeto elástico que puede resbalar). Reloj falso: los tests no esperan.
- Tests: `cd scripts && python3 -m unittest brazo.test_brazo -v` (34 pruebas; se comprobó que reintroducir el bug del
  orden 108/112 y el del despegue las hace fallar).
- `scripts/prueba-brazo.py --etapa=N`: las pruebas con motores. Reemplaza a `test-recoger.py`.

**Regla: no se toca el robot si los tests no pasan.**

## Gripper abierto: congelar la meta (estrategia B)

El gripper tiene juego en los engranajes. Si tras abrir la meta queda en la apertura, el controlador de posición sigue
corrigiendo ese juego y la pinza oscila. **Estrategia acordada:** al terminar de abrir, escribir como meta la posición
actual (`meta = posición real`), con el torque encendido. Así el error es cero y el controlador no empuja. Para cerrar
se escribe una meta nueva (`cerrar_y_apretar()` ya lo hace). Está en `Brazo.abrir()`.

- La pinza queda donde llegó, a lo sumo a 4 ticks de la apertura nominal (la tolerancia de llegada).
- Siguen valiendo la corriente y la velocidad configuradas; solo cambia la meta.
- No es un bloqueo mecánico: si algo empuja los dedos, el controlador los devuelve a esa meta.
- **Pendiente de verificar en el robot:** `scripts/diagnostico-pinza.py` compara la estrategia A (como antes), B, C (B con
  corriente baja de mantenimiento) y D (torque apagado) midiendo el pico a pico, las inversiones de sentido y la corriente.
  Si B no basta, la C es el siguiente paso.
- **2026-11-02:** el usuario vio que con B la pinza abierta sigue oscilando. Se adopta la **C**: tras congelar la meta,
  `abrir()` baja la corriente a `corriente_reposo_p` = 30 (~80 mA); cualquier movimiento nuevo vuelve a escribir su corriente.
  Se verifica en la prueba de la fase G (`scripts/prueba-pinza-frutas.py`, pregunta de G1).
- **2026-11-03:** con C sigue vibrando (medido: 12-16 ticks de pico a pico, ~10 Hz, corriente ~17; suelta, sin torque,
  la pinza queda 15-20 ticks más abierta: es el juego más la tensión de los dedos). La D no lo arregla (D=0: 28 ticks);
  lo que lo quita es bajar la P **solo en reposo**: `abrir()` deja **P120 D300** (Position P/D Gain, registros 84/80,
  RAM) y cualquier movimiento de la pinza vuelve primero a las normales **P800 D4000** (las del motor, con las que se
  probó el agarre). P120 D300: 1 tick (ruido del encoder) en 5 de 5; P200 D300 al límite (2 de 4 oscilaron 6-7 ticks).
  Con el tornillo moviéndose (vel 40 y 100) la pinza no vibra. Repetir la medida: `scripts/diagnostico-ganancias-pinza.py`.
  El driver permite ahora 80-81 y 84-85 en la lista blanca del bus del brazo.

## Cierre medido (2026-11-02)

El limón no llegaba a tocar los dedos antes de `CERRADA - MARGEN_PINZA` (3791). `Brazo.medir_cierre()` cierra en vacío a
`vel_agarre` con corriente 80 (~215 mA) hasta que los dedos se detienen (máximo `CERRADA + 150`) y usa esa posición como
tope de apriete. Un contacto a menos de 15 ticks de ella es "dedos juntos" (sin objeto). Si se detiene a más de 240 ticks
antes de `CERRADA`, aborta (algo entre los dedos). Lo usa la fase G; el nodo del brazo lo adoptará en la fase 4.

## Sostén y rearme (2026-11-02)

- Tras agarrar, `cerrar_fruta(tipo)` baja la corriente a la de sostén (`SOSTEN_FRUTA`: limón 200, naranja 300, las
  mínimas firmes de la fase G): sostener con la de apriete dejó el gripper con sobrecarga (registro 70 = 32).
- `ControlBrazo.ocupado` es "ejecutando una orden". Antes era "cerrojo tomado", y `mantener()` lo toma para vigilar: un
  `/parada` con el brazo quieto lo congelaba y `brazo/ocupado` frenaba las ruedas a ratos. En reposo una petición de
  aborto suave se descarta; una dura (batería) se atiende.
- Orden `rearmar`: saca del congelado abriendo la pinza, solo sin fruta (con fruta, `brazo/soltar` a mano). La usa la
  misión al empezar.
- En reposo el nodo vigila cada 0.25 s con el watchdog del motor a 1 s (antes 0.1 s y 0.5 s).

## Política de abortos

| Anomalía | Tipo | Qué pasa |
|---|---|---|
| Atasco, corriente alta, resbalón, tiempo agotado, no despega del tope | Suave | `congelar()`: las metas pasan a la posición actual y el torque sigue. Espera tu Enter; entonces abre la pinza y apaga |
| Error de hardware, sobretemperatura (>55 °C), voltaje fuera de 10–14.8 V, fallo de comunicación, Ctrl+C | Duro | Torque off de ambos motores (cada uno por separado), se invalida el cero. Suelta el objeto |

El tornillo se sostiene solo con el torque apagado (comprobado a mano el 2026-11-01), así que apagarlo no lo deja caer.
El gripper sí suelta el objeto al apagarse.

## Calibración

- Se golpea el tope inferior una sola vez por energización, a ~20 rpm (`vel_busqueda = 87`), con el torque limitado a
  corriente 120 (~320 mA). A 23 rpm (100) el tornillo se encajó y no se despegaba con corriente 150; 87 está cerca de ese
  límite, así que es lo primero que se baja si vuelve a encajarse (`--busqueda=60`, 40 era el valor lento validado).
- Tras el tope se despega 40 ticks, a la misma velocidad y con corriente 250 (~670 mA). Si no se despega, aborto suave y queda sin calibrar.
- El cero se guarda en `~/.cache/wallens/brazo-cero.json` y vale solo mientras la marca de la RAM del motor (Indirect
  Address 28, dirección 222, valor 0xCA11) siga puesta. Si hubo corte de los 12 V, la marca vuelve a 0 y se hace homing.
- Sin calibrar, `bajar`, `subir` y `recoger` se rechazan.
- **Por verificar en el banco (etapa 0):** que escribir 0xCA11 en esa dirección sea aceptado, que no afecte a nada, y que
  vuelva a 0 al quitar los 12 V. Si el motor la rechaza, el código sigue funcionando pero hace homing en cada arranque.

## Etapas con motores

Cada etapa solo empieza si la anterior pasó. Una persona con la mano en el interruptor o en el conector de la batería
durante todo movimiento. Etapas 1 a 3 con fuente de banco limitada a ~2 A si se tiene; si no, batería con la persona al lado.
Antes de cada una: `sudo systemctl stop wallens` y `source /opt/ros/jazzy/setup.bash`.

| Etapa | Comando | Pasa si |
|---|---|---|
| 0 | a mano, ver arriba | la marca se escribe, se lee y vuelve a 0 sin los 12 V |
| 1 tornillo solo | `python3 scripts/prueba-brazo.py --etapa=1` | homing en menos de 70 s (~35 s esperados a 20 rpm), despegue sin abortar, 3 ciclos subir/bajar con error de llegada < 8 ticks y corriente máx < 250, sin abortos |
| 2 gripper solo | `--etapa=2` | 2 ciclos abrir/cerrar en vacío, el cierre dice "sin contacto". (Antes se puede repetir `test-gripper.py` y `--agarrar`, ya validados) |
| 3 combinado en vacío | `--etapa=3` | `recoger()` devuelve `sin_agarre`, nunca `agarrado` |
| 4 objeto blando (<100 g) | `--etapa=4 --corridas=5` | 5 de 5 corridas `agarrado`, sin resbalar |
| 5 objeto real | `--etapa=5 --corridas=5` | igual que la 4 |

Si una etapa falla: se para, se anota el mensaje, se reproduce en el bus falso (añadiendo el caso a los tests) y se corrige
antes de volver a mover los motores.

## Pendiente antes de la etapa 4

- Medir la holgura entre el gripper y la mesa con el tornillo en el tope inferior y anotarla en `tornillo.md`.
- Elegir el objeto blando (pelota de espuma o fruta de juguete, menos de 100 g).

## Punto abierto: integración con ROS

El bus de los motores es un solo puerto (`/dev/dynamixel`) y el driver C++ de las ruedas lo tiene abierto. Un nodo del
brazo en otro proceso no puede abrirlo a la vez. Opciones: (a) el brazo dentro del driver C++, (b) el driver presta el bus
por un servicio o tópico y el nodo Python usa un `Bus` que habla con él, (c) un segundo adaptador USB. No se decide ahora.

## Incidente 2026-11-01: se rompió el tornillo

En la primera corrida de la etapa 1 (homing a ~20 rpm, `vel_busqueda = 87`, torque limitado a corriente 120) el motor
giró 45836 ticks sin frenarse y el script abortó con "sin tope inferior". Después se comprobó que el tornillo sin fin se
había roto. Antes habían ocurrido un encaje a 23 rpm contra el tope y un despegue con corriente 250. La causa exacta no
está confirmada; los golpes contra el tope duro son el sospechoso principal.

Consecuencias para el plan:
- No repetir la etapa 1 ni ninguna con motores hasta cambiar el tornillo y revisar el acople y la tuerca.
- Replantear el cero: golpear un tope duro en cada energización es lo que más exige al mecanismo. Alternativas a decidir:
  un final de carrera o sensor en el carro, un tope blando o una marca mecánica, o limitar el homing a velocidad muy baja
  y corriente mínima validada a mano.
- Registrar posición y corriente durante el homing para ver lo que pasa en el tope.
