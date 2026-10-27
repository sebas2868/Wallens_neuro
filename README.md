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
3b. `sudo scripts/03-i2c.sh` y reiniciar (I²C a 50 kHz).
4. Copiar el repo a la Pi: `scp -r launch scripts wallens@IP:~/wallens/`.
5. Un paso por dispositivo, ver `docs/`: [LiDAR](docs/lidar.md), [cámaras](docs/camaras.md), [IMU](docs/imu.md).

Cada dispositivo tendrá `docs/<dispositivo>.md` y un `scripts/test-<dispositivo>.sh`.

## Estado

| Dispositivo | Estado |
|---|---|
| LiDAR | listo, `test-lidar.sh` OK |
| Cámaras | listas, `test-camaras.sh` OK |
| BNO055 | listo, 30 min estables (re-verificar tras recablear) |
| INA226 | detectado en bus 1 (`0x40`); falta calibración (valor del shunt) y topic |
| USB-RS485 | CH340 reconocido (`/dev/dynamixel`); falta probar con motores |

Pendiente: IP fija (reserva DHCP por MAC con TI), clave SSH en vez de contraseña,
descubrimiento ROS 2 sin multicast para PC↔Pi, imagen de respaldo de la SD.
