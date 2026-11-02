# LiDAR LDRobot STL-19P (LD19)

USB-UART (CP2102), 230400 baud, ~10 Hz. Publica `/scan` (`sensor_msgs/LaserScan`)
con frame `base_laser`.

## Uso

Lo lanza el servicio `wallens` (`launch/robot.launch.py`). Para verlo:

```
ros2 topic hz /scan
ros2 topic echo /scan --once
```

El launch pone también la TF estática `base_link → base_laser` (x = 0.089 m al centro de giro, z = 0.12675 m, yaw −90°; ajustar
en `launch/robot.launch.py` y en `laser_x` de `wallens_safety.yaml` al montaje real).

## Cómo está montado

- **Puerto fijo:** `/dev/lidar` (regla udev en `config/99-wallens.rules`, por
  puerto USB físico: el CP2102 es genérico y su serie `0001` no sirve para
  distinguirlo). Si se cambia de puerto USB, actualizar `ID_PATH` en la regla
  (`udevadm info /dev/ttyUSB0 | grep ID_PATH`) y correr `scripts/02-udev.sh`.
- **Driver:** `github.com/ldrobotSensorTeam/ldlidar_stl_ros2` en
  `~/ros_ws/src`, clonado en versión fija, parcheado (`config/ldlidar-pthread.patch`: falta
  `#include <pthread.h>` con GCC 13) y compilado por `scripts/05-ros-ws.sh`.
- No ejecutar los scripts con `sudo` que trae el repo del fabricante.

## Problemas conocidos

- `stty` sobre `/dev/lidar` se cuelga: leer con Python no bloqueante.
- "Puerto ocupado": queda un nodo vivo → `pkill ldlidar_stl_ros`.
