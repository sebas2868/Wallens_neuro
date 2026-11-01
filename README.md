# Wallens

Raspberry Pi 4 (8 GB) del robot diferencial tipo retroexcavadora que recoge
frutas. Este repo prepara la Pi para que cualquiera se conecte por SSH y use
los sensores y motores con ROS 2.

## Plataforma

Ubuntu Server 24.04 (arm64) + ROS 2 Jazzy. Hostname `wallens`.

## Hardware

| Dispositivo | Interfaz |
|---|---|
| LiDAR LDRobot STL-19P | USB-UART |
| BNO055 (IMU) | I²C |
| INA226 (consumo) | I²C |
| 2 cámaras USB | USB |
| Dynamixel XM430-W210-R | USB-RS485 Peacefair |

## Conectarse

La red `LABORATORIOS` bloquea mDNS (`wallens.local` no funciona) y la IP
cambia. Se busca por MAC de la Pi (`d8:3a:dd:46:9f:2b`):

```
ssh wallens@$(scripts/pi-ip.sh)
```

## Instalar (en orden)

1. Flashear la SD con Raspberry Pi Imager: *Ubuntu Server 24.04 LTS (64-bit)*;
   en ajustes avanzados poner hostname `wallens`, usuario, clave SSH y Wi-Fi.
2. `sudo scripts/00-base.sh <usuario> [otros usuarios]`
3. `sudo scripts/01-ros2.sh` (ROS 2 Jazzy) y `sudo scripts/02-udev.sh` (nombres fijos de puertos).
3b. `sudo scripts/03-i2c.sh` y reiniciar (bus I²C por software, ver [docs/imu.md](docs/imu.md)).
4. Copiar el repo a la Pi (`~/wallens`) y compilar los paquetes de `ros/` en `~/ros_ws` (symlinks en `src/`; `colcon build`).
5. Un paso por dispositivo, ver `docs/`: [LiDAR](docs/lidar.md), [cámaras](docs/camaras.md), [IMU](docs/imu.md).
6. `sudo scripts/04-servicio.sh`: arranque automático de todo, ver [docs/sistema.md](docs/sistema.md).

Cada dispositivo tendrá `docs/<dispositivo>.md` y un `scripts/test-<dispositivo>.sh`.

## Estado

| Parte | Estado |
|---|---|
| LiDAR | listo; TF medida (x 108 mm, z 126.75 mm, yaw -90°), alimenta el freno por obstáculo |
| Cámaras | listas, `test-camaras.sh` OK; sin integrar al servicio |
| IMU BNO055 | driver C++ propio, remap P6 (boca abajo), I²C por software; ver [docs/imu.md](docs/imu.md) |
| INA226 | `wallens_power`: `/battery_state` y corte a 3.4 V/celda |
| Ruedas (IDs 2 y 4) | driver C++, giro de 360° con 1.2° de error, avance de 1 m exacto |
| EKF | ruedas + IMU, `/odometry/filtered` a 30 Hz |
| Freno LiDAR | a 30 cm, verificado con un obstáculo |
| Arranque | servicio systemd con chequeo previo; ver [docs/sistema.md](docs/sistema.md) |
| Tornillo sin fin (ID 3) | recorrido calibrado, ver [docs/tornillo.md](docs/tornillo.md) |
| Gripper (ID 5) | `scripts/test-gripper.py` con apriete progresivo; sin nodo ROS |
| Navegación | rutas de prueba en curso, ver [docs/navegacion.md](docs/navegacion.md); falta SLAM/Nav2 |

Pendiente: IP fija (reserva DHCP por MAC con TI), descubrimiento ROS 2 sin multicast para PC↔Pi, imagen de respaldo de la SD,
parada de emergencia física, nodo ROS del brazo (IDs 3 y 5), envío de imagen, auditoría de seguridad (`docs/sistema.md`).
