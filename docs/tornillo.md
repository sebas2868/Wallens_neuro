# Tornillo sin fin del gripper (Dynamixel ID 3)

Sube y baja el gripper. XM430-W210, 57600 baudios. Se corre en la Pi con ROS cargado (trae `dynamixel_sdk`):
`source /opt/ros/jazzy/setup.bash && python3 ~/wallens/scripts/<script> ... /dev/dynamixel` (parar el servicio antes si el
driver de las ruedas usa el mismo bus; los IDs son distintos pero conviene no mezclar).

## Calibración del tornillo nuevo (medida el 2026-11-03)

El tornillo se rompió el 2026-11-01 y se cambió por uno reforzado. Medido con `jog-tornillo.py` (paso 20, corriente 90):
abajo con la tecla `b` hasta que el jog dijo "tope"; arriba a ojo, parando en la marca, porque **arriba no hay tope por
corriente** (el carro no frena el motor: la corriente llegó a 57 y el paso terminó sin detectarlo).

Los ticks crecen al bajar (`sentido_abajo: 1`). Posición multivuelta (modo 5, posición por corriente).

| | Ticks | Tornillo viejo (2026-11-01) |
|---|---|---|
| Tope abajo | 25577 (detectado por corriente, 90) | 24245 |
| Tope arriba | -12357 (a ojo) | -13399 |
| **Recorrido** | **37934 = 9.26 vueltas** | 37644 = 9.19 vueltas |
| Límite útil arriba | -10460 (margen del 5 % = 1897 ticks) | -11517 |
| Límite útil abajo | 25577 (sin margen: la base es robusta) | 24245 |

Guardado en `scripts/calibracion-tornillo.json`. **Las posiciones absolutas solo valen hasta el siguiente corte de energía**
(al apagar, el motor pierde la cuenta de vueltas y arranca con un valor dentro de una vuelta). Lo que no cambia es el
**recorrido**; por eso el cero es siempre el tope de abajo y el de arriba se calcula:

    arriba = tope_abajo - recorrido_util      (recorrido_util = abajo_util - arriba_util = 36037 ticks)

Si se cambia o reajusta el mecanismo, el recorrido se vuelve a medir con el jog como arriba.

## Cero al arrancar: homing suave al tope de abajo

`Brazo.calibrar()` (scripts/brazo/brazo.py) baja hasta que el tornillo se detiene con corriente, se despega 40 ticks y
fija `arriba` con la resta de arriba. **Nunca busca el tope de arriba.**

- Velocidad del homing `vel_busqueda = 40` (~9 rpm, la del jog). Antes era 87 (~20 rpm): por encima de 60 el tope encaja
  el tornillo y el despegue lo arranca con corriente 250, en **cada** homing. Es el esfuerzo sospechoso de la rotura. El
  bus falso lo reproduce (`test_homing_a_87_como_antes_si_encajaba_el_tornillo`) y el test
  `test_homing_con_la_configuracion_por_defecto_nunca_encaja_el_tornillo` exige 0 encajes. Peor caso (arranca arriba del
  todo): ~61 s, dentro de `tiempo_homing = 90`.
- **El despegue encajaba el tornillo aun a 40** (robot, 2026-11-03 17:10): tope en 37948 tras bajar 36037 ticks, y al
  despegar con 250 **bajó 227 ticks más** contra la base y quedó atascado. Causa probable: frenado contra la base, el
  perfil del motor sigue "bajando", y la meta nueva con más corriente termina esa bajada empujando. Arreglo
  (`_asentar_en_tope`): torque off y on en el tope (reinicia el perfil en la posición real; el tornillo sin fin se
  sostiene solo y no se pierden las vueltas), 0.3 s quieto, y despegue primero con `corriente_despegue = 150`; 250 solo
  si con 150 no sale. El bus falso modela ese empuje (`INERCIA_S`) y
  `test_sin_reiniciar_el_perfil_el_despegue_con_250_encaja_el_tornillo` lo reproduce. Se desencajó con 150 (subió 133
  ticks) y el homing siguiente salió al primer intento: tope en 38014.
- **La marca de RAM no funciona** (probado en el motor el 2026-11-03): el código escribe `0xCA11` en la dirección 222
  (Indirect Address 28), pero ese registro solo acepta 64-661; el motor responde error 4 "data value is out of range" y
  deja su valor de fábrica, 251. Consecuencia: `calibrar()` nunca reusa el cero y **hace homing cada vez que arranca el
  programa del brazo**. Es el comportamiento seguro y se deja así (decisión del 2026-11-03): un toque suave a la base
  por arranque, nunca una posición vieja.
- El homing lo lanza una persona al iniciar el programa, no el servicio al prender la Pi: si la Pi se reinicia sola, el
  brazo no se mueve sin nadie mirando.

## Scripts

- `scripts/jog-tornillo.py`: mueve el tornillo de a poco con el teclado y detecta los topes por corriente.
- `scripts/prueba-brazo.py --etapa=N`: pruebas por etapas del tornillo y el gripper (ver `docs/gripper-plan.md`).
- Los primeros `test-tornillo.py` y `test-tornillo-ciclos.py` (búsqueda de topes y ciclos a 150 ≈ 34 rpm; el tope de
  fábrica es 265 ≈ 61 rpm) están borrados; siguen en el historial de git.
  Ojo: el registro 48 (Max Position Limit) solo acepta 0-4095; en modo multivuelta se ignora, no escribirlo.

## Pendiente

- Probar `calibrar()` en el robot con la velocidad nueva (homing desde la mitad y desde arriba) y un recorrido completo
  hasta `arriba_util` y de vuelta, mirando que la corriente no suba en ningún punto.
- Revisar si `corriente_retiro = 250` sigue haciendo falta: desde 2026-11-03 se intenta antes con 150 (el log dice si escaló).
- Elegir la velocidad de trabajo (`vel_tornillo = 100`, ~23 rpm; se probó 150) y comprobar que el gripper en su
  posición más baja no toque el suelo ni lleve una fruta.
- La misión todavía va **sin ascensor** (`bajar`/`subir` responden "sin ascensor"): volver a activarlo es otro paso.
