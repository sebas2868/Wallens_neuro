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

- `scripts/jog-tornillo.py`: mueve el tornillo de a poco con el teclado y detecta los topes por corriente.
- `scripts/prueba-brazo.py --etapa=N`: pruebas por etapas del tornillo y el gripper (ver `docs/gripper-plan.md`).
- Los primeros `test-tornillo.py` y `test-tornillo-ciclos.py` (búsqueda de topes y ciclos a 150 ≈ 34 rpm; el tope de
  fábrica es 265 ≈ 61 rpm) están borrados; siguen en el historial de git.
  Ojo: el registro 48 (Max Position Limit) solo acepta 0-4095; en modo multivuelta se ignora, no escribirlo.

## Pendiente

Elegir la velocidad de trabajo (se probó 150; se quería subir), cero al arrancar bajando al tope inferior vigilando la corriente
(probar a mano antes de automatizar), comprobar que el gripper en su posición más baja no toque el suelo ni lleve una fruta.
