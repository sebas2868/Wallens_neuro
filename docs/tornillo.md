# Tornillo sin fin del gripper (Dynamixel ID 3)

Sube y baja el gripper. XM430-W210, 57600 baudios. Se corre en la Pi con ROS cargado (trae `dynamixel_sdk`):
`source /opt/ros/jazzy/setup.bash && python3 ~/wallens/scripts/<script> ... /dev/dynamixel` (parar el servicio antes si el
driver de las ruedas usa el mismo bus; los IDs son distintos pero conviene no mezclar).

## Calibración (medida el 2026-11-01)

Los ticks decrecen al subir (sentido -1 = sube). Posición multivuelta (modo 5, posición por corriente).

| | Ticks |
|---|---|
| Tope arriba | -13399 (a mano; el tornillo no frena solo, no hay tope por corriente) |
| Tope abajo | 24245 (detectado por corriente, ~56) |
| Recorrido | 37644 ticks = 9.19 vueltas |
| Límite útil arriba | -11517 (margen del 5 % = 1882 ticks) |
| Límite útil abajo | 24245 (el real, sin margen: la base es más robusta) |

Guardado en `scripts/calibracion-tornillo.json`.

## Scripts

- `scripts/test-tornillo.py [--sentido=+1|-1] [--corriente=N] [--arriba=N] [--tope-arriba=N] [--vueltas=N]`: busca topes despacio
  (~9 rpm, ~320 mA) hasta que la posición se detiene con corriente subiendo. Guarda el JSON si completa.
  Ojo: el registro 48 (Max Position Limit) solo acepta 0-4095; en modo multivuelta se ignora, no escribirlo.
- `scripts/test-tornillo-ciclos.py [--ciclos=N] [--velocidad=N] [--aceleracion=N]`: va y viene entre los límites útiles y mide
  tiempo, error de llegada y corriente máxima por tramo. Aborta si la corriente pasa de 250 sostenida, si no avanza o por
  temperatura/voltaje. Por defecto 150 (~34 rpm); el tope de fábrica de velocidad es 265 (~61 rpm).

## Pendiente

Elegir la velocidad de trabajo (se probó 150; se quería subir), cero al arrancar bajando al tope inferior vigilando la corriente
(probar a mano antes de automatizar), comprobar que el gripper en su posición más baja no toque el suelo ni lleve una fruta.
