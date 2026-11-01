# Sistema del robot: arranque, nodos y seguridad

## Arquitectura

```
/cmd_vel (scripts, Nav2) -> freno_lidar -> /cmd_vel_safe -> wallens_driver -> motores 2 y 4
                                                              |-> /odom (ruedas)
/bno055/imu  (wallens_imu) ---------------------------------> EKF -> /odometry/filtered + TF odom -> base_link
/scan (LD19) -> freno_lidar (y SLAM/Nav2 más adelante)
INA226 -> wallens_power -> /battery_state, /battery_critical -> wallens_driver (frena)
```

| Paquete / archivo | Qué hace |
|---|---|
| `ros/wallens_driver` (C++) | Ruedas IDs 2 y 4 en modo velocidad, watchdog del bus de 200 ms, `/odom`. `publish_tf: false`: la TF la da el EKF |
| `ros/wallens_power` (C++) | INA226 a 2 Hz, aviso a 3.6 V/celda, corte a 3.4 V/celda (5 muestras seguidas) |
| `ros/wallens_imu` (C++) | BNO055, ver [imu.md](imu.md) |
| `ros/wallens_safety` (Python) | `freno_lidar`: anula el avance a menos de 30 cm en ±40° al frente (o atrás al retroceder); suelta a 35 cm; el giro en el sitio no se frena; sin `/scan` en 0.3 s, o con menos de 10 lecturas válidas en el sector (LiDAR tapado), bloquea el avance (`require_scan`); descarta `/cmd_vel` no finito; republica solo 0.25 s tras el último `/cmd_vel`. Publica `/freno_obstaculo` |
| `config/ekf.yaml` | `robot_localization`: velocidad lineal y angular de las ruedas + yaw y giro del IMU (`imu0_relative`), 2D, 30 Hz |
| `launch/robot.launch.py` | Lanza todo; si el driver muere se cierra todo (`on_exit=Shutdown`) y systemd reintenta |

El driver escucha `cmd_vel_safe` (remapeo en `robot.launch.py`): todo lo que publica `/cmd_vel` pasa por el freno, incluidos
los scripts de `scripts/odometria/`. Para saltarlo hay que publicar a `/cmd_vel_safe` a propósito.

## Marcos (TF)

`base_link` está en el punto medio del eje de las ruedas, a la altura del eje (36.25 mm sobre el suelo).

| Hijo | x | y | z | yaw | Origen |
|---|---|---|---|---|---|
| `imu_link` | 0 | 0 | 23.75 mm | 0 | chip a 60 mm del suelo, sobre el eje; remap P6 hace que coincida con `base_link` |
| `base_laser` | 108 mm | 0 | 126.75 mm | -90° | LiDAR a 163 mm del suelo; su cero apunta a la izquierda del robot (medido con un objeto de frente: 90° del LiDAR). No está en espejo |

## Arranque

Servicio `wallens` (`systemd/wallens.service`, se instala con `sudo scripts/04-servicio.sh`, `--quitar` lo desinstala).
`scripts/arranque.sh` corre el chequeo previo y luego `launch/robot.launch.py`. Si el chequeo previo falla, systemd reintenta cada 10 s.

`scripts/chequeo-arranque.py`:
- **previa** (antes de ROS). Crítico, impide arrancar: ruedas 2 y 4 responden, sin error de hardware, temperatura menor de 55 °C y
  voltaje entre 10 y 14.8 V (los motores leen el riel regulado de 12 V). Solo avisa: gripper y tornillo (IDs 3 y 5),
  `/dev/lidar`, IMU (0x28), INA226 (0x40). Reintenta 30 s porque el USB tarda.
- **ros** (25 s después, con los nodos en marcha). Solo avisa: `/scan` >= 5 Hz, `/bno055/imu` >= 20, `/odom` >= 20,
  `/odometry/filtered` >= 20, giroscopio del IMU en 3 y batería sobre 14.4 V.

Comandos: `journalctl -u wallens -f`, `sudo systemctl stop|restart wallens`. Los nodos que se lancen a mano mientras el
servicio corre duplican tópicos (pasó con el IMU y el freno): parar primero el servicio o matar solo el propio proceso por PID.

## CPU

El bus I²C por software espera bit a bit en el kernel. Medido con el servicio completo: núcleo 3 a ~26 % (solo el IMU, fijado con
`taskset -c 3` en `imu.launch.py`), núcleos 0-2 a 15-18 % (`CPUAffinity=0 1 2` en el servicio), Pi al ~19 %, 43 °C, sin throttling.
Con el driver Python del IMU (45 bytes, 50 Hz) el nodo llegaba al 85 % y el EKF dejaba de cumplir su ritmo.
Presupuesto estimado (no medido) para SLAM + Nav2: 1-1.5 núcleos; video JPEG 640x480 a 15 fps ~0.3; H.264 por el codificador
de la Pi 4 casi nada. Medir antes y después de sumarlos y subir la prioridad del driver y el EKF si hace falta.

## Batería y motores: lo aprendido

- Tras reiniciar la Pi el watchdog de los motores queda en 255 y rechaza las escrituras ("data value is out of range", error 4):
  el driver lo limpia (reg. 98 = 0) antes de configurar.
- Con la LiPo casi agotada el voltaje cayó a 12.2 V; `wallens_power` publicó `battery_critical` bajo 13.6 V y el driver
  frenó las ruedas (los giros abortaban por tiempo). Es el comportamiento esperado; cargar antes de las rutas.
- Los motores leen 12 V aunque la batería marque 14-15 V: hay un regulador en medio.

## Pendiente

Parada de emergencia física, cero del gripper al arrancar (bajar el tornillo al tope inferior), SLAM/Nav2, envío de imagen,
auditoría de seguridad/eficiencia/confiabilidad (prompt en la conversación; el informe iría en `docs/auditoria-*.md`).
