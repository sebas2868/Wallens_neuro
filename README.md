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

ROS 2 desde el PC (Docker, Jazzy como la Pi, con RViz y teleop): `scripts/pc-ros.sh` abre una shell;
`scripts/pc-ros.sh ros2 topic list` o `scripts/pc-ros.sh rviz2` corren un comando. Usa el dominio ROS 37, que debe ser el mismo de la Pi
(otro: `ROS_DOMAIN_ID=N scripts/pc-ros.sh`); si no aparecen los tópicos, `ROS_STATIC_PEERS=<ip de la Pi>`.

## Uso diario (desde el PC)

| Comando | Qué hace |
|---|---|
| `scripts/probar.sh [agarre\|acercar\|hasta-agarrar\|completa]` | tests → despliegue → etapa de la misión en la Pi → registro en `registros/` |
| `scripts/desplegar.sh` | tests, rsync a la Pi, compila los paquetes tocados, comprueba md5 y reinicia el servicio activo |
| `scripts/pc-ros.sh [comando]` | ROS 2 en Docker conectado al robot (RViz, `ros2 topic ...`) |

En la Pi corre siempre uno de dos servicios systemd (nunca los dos: `Conflicts=`): `wallens` (robot base, arranca solo)
o `wallens-mision` (robot + brazo + detector; lo arranca `probar-mision.sh`). Shells de la Pi:
`source ~/wallens/scripts/ros-env.sh`. No lanzar launch a mano con un servicio activo: duplica el driver en el bus.

## Instalar (en orden)

1. Flashear la SD con Raspberry Pi Imager: *Ubuntu Server 24.04 LTS (64-bit)*;
   en ajustes avanzados poner hostname `wallens`, usuario, clave SSH y Wi-Fi.
2. `sudo scripts/00-base.sh <usuario> [otros usuarios]`
3. `sudo scripts/01-ros2.sh` (ROS 2 Jazzy) y `sudo scripts/02-udev.sh` (nombres fijos de puertos).
4. `sudo scripts/03-i2c.sh` y reiniciar (bus I²C por software, ver [docs/imu.md](docs/imu.md)).
5. Copiar el repo a `~/wallens` (`scripts/desplegar.sh`), `sudo scripts/05-ros-ws.sh deps` y `scripts/05-ros-ws.sh`.
6. `sudo scripts/04-servicio.sh`: los dos servicios y la regla de sudo, ver [docs/sistema.md](docs/sistema.md).

## Estado

| Parte | Estado |
|---|---|
| Ruedas (IDs 2 y 4) | driver C++ en tiempo real, giro de 360° con 1.2° de error, avance de 1 m exacto; patinan al girar |
| IMU BNO055 | driver C++ propio, remap P6 (boca abajo), I²C por software; ver [docs/imu.md](docs/imu.md) |
| EKF | ruedas + giro del IMU, `/odometry/filtered` a 30 Hz |
| LiDAR | TF medida (x 89 mm, z 126.75 mm, yaw -90°); alimenta el freno; ver [docs/lidar.md](docs/lidar.md) |
| Freno LiDAR | por huella, en C++; también frena mientras el brazo se mueve |
| INA226 | `wallens_power`: `/battery_state` y corte a 3.4 V/celda |
| Gripper (ID 5) | nodo `wallens_brazo` (`scripts/brazo/`) por el bus del driver; apriete por fruta (limón, naranja) |
| Tornillo sin fin (ID 3) | roto: la misión va sin ascensor; ver [docs/tornillo.md](docs/tornillo.md) |
| Cámaras | `detector_frutas` (OpenCV, una cámara a la vez); ver [docs/camaras.md](docs/camaras.md) |
| Misión de frutas | máquina de estados con tests; pruebas reales por etapas; ver [docs/mision-frutas-plan.md](docs/mision-frutas-plan.md) |
| Mapeo | SLAM + Nav2 + explore_lite (`ros2 launch wallens_nav mapeo.launch.py`); ver [docs/navegacion.md](docs/navegacion.md) |

Pendiente: IP fija (reserva DHCP por MAC con TI), imagen de respaldo de la SD, envío de imagen, reimprimir las llantas.

Los scripts de diagnóstico que ya cumplieron (calibraciones iniciales, pruebas del bus I²C, giros sueltos) se borraron
el 2026-11-02; se recuperan con `git log --diff-filter=D --name-only` y `git show <commit>^:<ruta>`.
