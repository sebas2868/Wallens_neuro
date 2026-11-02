# Cámaras USB

| Nombre | Equipo | Uso | Dispositivo |
|---|---|---|---|
| `cam_frutas` | Logitech C270 (`046d:0825`) | identificar frutas | `/dev/cam_frutas` |
| `cam_garra` | Suyin HD Camera (`1e45:8022`) | verificar si la garra agarró | `/dev/cam_garra` |

Los nombres son fijos aunque cambie el orden de conexión (regla udev por
modelo en `config/99-wallens.rules`; el `index 0` es el flujo de video).
Si se reemplaza una cámara por otro modelo, cambiar `idVendor`/`idProduct`
(`lsusb`) y correr `sudo scripts/02-udev.sh`.

## Uso sin ROS (opción principal)

Son dispositivos V4L2 estándar: sirven directo a OpenCV, ffmpeg, etc.

```python
import cv2
cap = cv2.VideoCapture("/dev/cam_frutas", cv2.CAP_V4L2)
ok, frame = cap.read()
```

```
v4l2-ctl -d /dev/cam_frutas --list-formats-ext   # resoluciones y fps
```

Usar MJPG a 640x480 si las dos cámaras van a la vez: comparten el ancho de
banda del bus USB 2.0 de la Pi.

## Uso con ROS 2 (opcional)

Está instalado `ros-jazzy-v4l2-camera`. No arranca solo:

```
ros2 run v4l2_camera v4l2_camera_node --ros-args \
  -p video_device:=/dev/cam_garra -r __ns:=/cam_garra
ros2 topic hz /cam_garra/image_raw
```

## Prueba de aceptación

`scripts/preflight.py` (lo corre `probar-mision.sh`) enciende cada cámara y comprueba que el detector publica con ella.
