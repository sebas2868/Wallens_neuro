# LiDAR LDRobot STL-19P (LD19)

USB-UART (CP2102), 230400 baud, ~10 Hz. Publica `/scan` (`sensor_msgs/LaserScan`)
con frame `base_laser`.

## Uso

```
ros2 launch ~/wallens/launch/lidar.launch.py
ros2 topic echo /scan --once
```

Lanza el driver y la TF estática `base_link → base_laser` (x = 0.089 m al centro de giro, z = 0.12675 m, yaw −90°; ajustar
en `launch/lidar.launch.py` al montaje real).

## Prueba de aceptación

En la Pi: `~/wallens/scripts/test-lidar.sh` → `OK: /scan a ~10 Hz`.

## Cómo está montado

- **Puerto fijo:** `/dev/lidar` (regla udev en `config/99-wallens.rules`, por
  puerto USB físico: el CP2102 es genérico y su serie `0001` no sirve para
  distinguirlo). Si se cambia de puerto USB, actualizar `ID_PATH` en la regla
  (`udevadm info /dev/ttyUSB0 | grep ID_PATH`) y correr `scripts/02-udev.sh`.
- **Driver:** `github.com/ldrobotSensorTeam/ldlidar_stl_ros2` en
  `~/ros_ws/src`, compilado con `scripts/build-lidar.sh` (desde el PC).
  Requiere `config/ldlidar-pthread.patch` (falta `#include <pthread.h>` con
  GCC 13): `patch -p1 < config/ldlidar-pthread.patch` en la carpeta del driver.
- No ejecutar los scripts con `sudo` que trae el repo del fabricante.

## Problemas conocidos

- `stty` sobre `/dev/lidar` se cuelga: leer con Python no bloqueante.
- "Puerto ocupado": queda un nodo vivo → `pkill ldlidar_stl_ros`.
