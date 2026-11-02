# Sistema del robot: arranque, nodos y seguridad

## Arquitectura

```
/cmd_vel (scripts, Nav2) -> freno_lidar -> /cmd_vel_safe -> wallens_driver -> motores 2 y 4
                                                              |-> /odom (ruedas)
/bno055/imu  (wallens_imu) ---------------------------------> EKF -> /odometry/filtered + TF odom -> base_link
/scan (LD19) -> freno_lidar (y SLAM/Nav2 más adelante)
INA226 -> wallens_power -> /battery_state, /battery_critical -> wallens_driver (frena)
/parada (Bool) -----------------------------------------> wallens_driver (velocidad 0, manda sobre /cmd_vel_safe)
```

| Paquete / archivo | Qué hace |
|---|---|
| `ros/wallens_driver` (C++) | Ruedas IDs 2 y 4 en modo velocidad, watchdog del bus de 200 ms, `/odom`. `publish_tf: false`: la TF la da el EKF. Frena si `/cmd_vel_safe` tiene más de 0.25 s, si es no finito, si `/battery_state` tiene más de 2 s (`battery_timeout`; `<= 0` lo desactiva en el banco sin INA226), con `battery_critical` o con `/parada` |
| `ros/wallens_power` (C++) | INA226 a 2 Hz, aviso a 3.6 V/celda, corte a 3.4 V/celda (5 muestras seguidas) |
| `ros/wallens_imu` (C++) | BNO055, ver [imu.md](imu.md) |
| `ros/wallens_safety` (Python) | `freno_lidar`: con la huella del robot (parámetro `footprint`: chasis redondo de 296 mm centrado en el eje de las ruedas + carro y dedos cerrados detrás; llega a 28.5 cm del centro) anula el avance si hay algo a menos de 15 cm del borde de la huella en el pasillo que barre (o detrás al retroceder, desde la punta del gripper o el borde redondo según la altura lateral; suelta a 20 cm), y anula el giro si en los próximos 0.5 s el gripper golpearía algo o lo acercaría a menos de 3 cm (girar para alejarse sí se permite). Sin huella: cono de ±40° desde el LiDAR a 30 cm y el giro no se frena. Sin `/scan` en 0.3 s bloquea avance y giro; con menos de 10 lecturas válidas en el cono (LiDAR tapado) bloquea el avance (`require_scan`); descarta `/cmd_vel` no finito; republica solo 0.25 s tras el último `/cmd_vel`. Publica `/freno_obstaculo` |
| `config/ekf.yaml` | `robot_localization`: velocidades x, y (siempre 0) y de giro de las ruedas + velocidad de giro del IMU, 2D, 30 Hz. El yaw absoluto del IMU no se usa (si el chip se reconfigura en marcha su rumbo vuelve a 0, #16) |
| `launch/robot.launch.py` | Lanza todo; si el driver muere se cierra todo (`on_exit=Shutdown`) y systemd lo vuelve a lanzar |

`vigilante_lidar` (en `wallens_safety`) reinicia el nodo del LiDAR si `/scan` falta 5 s, con 15 s de espera entre reinicios: el nodo
`ldlidar` no se cae al desenchufar el USB y no se recupera al reconectar (`/dev/lidar` pasa a otro `ttyUSB`). Medido: `/scan` de vuelta
~10 s después de desenchufar; mientras tanto el freno bloquea el avance (0.5 s tras el último `/scan`).

Freno, LiDAR, IMU, EKF y `wallens_power` se lanzan con `respawn=True, respawn_delay=2.0`: si uno muere vuelve solo a los 2 s (el
EKF reinicia su pose en 0 al volver). Tests sin hardware del freno: `python3 -m pytest ros/wallens_safety/test` (no necesita ROS).

El driver escucha `cmd_vel_safe` (remapeo en `robot.launch.py`): todo lo que publica `/cmd_vel` pasa por el freno, incluidos
los scripts de `scripts/odometria/`. Para saltarlo hay que publicar a `/cmd_vel_safe` a propósito.

## Parada por software (`/parada`)

La parada de emergencia física se descartó (no es viable en el chasis; riesgo aceptado). La mitigación por software es el
tópico `/parada` (`std_msgs/Bool`, `transient_local`): el driver lo obedece por encima de `/cmd_vel_safe`. Con `true` las
ruedas quedan a velocidad 0 y no se mueven hasta recibir `false`; el valor queda guardado, así que si el driver se reinicia
con la parada puesta arranca parado. `/battery_critical` y el resto de frenos siguen aplicando.

```bash
# parar
ros2 topic pub --once --qos-durability transient_local /parada std_msgs/msg/Bool "{data: true}"
# reanudar
ros2 topic pub --once --qos-durability transient_local /parada std_msgs/msg/Bool "{data: false}"
```

El driver acepta también publicadores sin `--qos-durability` (volátiles), pero solo `transient_local` queda guardado para un
driver que arranque después. **No sustituye a la seta**: si falla el propio software, la red o el PC desde el que se
publica, no llega. Las capas que quedan son el watchdog del bus (200 ms, si el proceso muere) y, en las pruebas en el suelo,
a ≤ 0.10 m/s con una persona junto al interruptor o conector de la batería. Aún no hay un nodo del brazo; cuando exista
(IDs 3 y 5) debe obedecer `/parada` igual.

## Marcos (TF)

`base_link` está en el punto medio del eje de las ruedas, a la altura del eje (36.25 mm sobre el suelo).

| Hijo | x | y | z | yaw | Origen |
|---|---|---|---|---|---|
| `imu_link` | 0 | 0 | 23.75 mm | 0 | chip a 60 mm del suelo, sobre el eje; remap P6 hace que coincida con `base_link` |
| `base_laser` | 108 mm | 0 | 126.75 mm | -90° | LiDAR a 163 mm del suelo; su cero apunta a la izquierda del robot (medido con un objeto de frente: 90° del LiDAR). No está en espejo |

## Arranque

Servicio `wallens` (`systemd/wallens.service`, se instala con `sudo scripts/04-servicio.sh`, `--quitar` lo desinstala).
`scripts/arranque.sh` corre el chequeo previo y luego `launch/robot.launch.py`. Con `Restart=always` systemd lo vuelve a lanzar cada 10 s si el chequeo previo falla o si el driver se apaga solo
(tirón del USB, caída del riel de 12 V); el driver sale con código 1 en un fallo y 0 con Ctrl+C/SIGINT. El servicio fija
`ROS_DOMAIN_ID=37` (hay que exportar el mismo en el PC y en cada shell desde la que se lancen scripts) y
`ROS_LOG_DIR=/run/wallens-log` (tmpfs: los logs de ROS no escriben en la SD; se borran al parar el servicio).

`scripts/chequeo-arranque.py`:
- **previa** (antes de ROS). Crítico, impide arrancar: ruedas 2 y 4 responden, sin error de hardware, temperatura menor de 55 °C y
  voltaje entre 10 y 14.8 V (los motores leen el riel regulado de 12 V). Solo avisa: gripper y tornillo (IDs 3 y 5),
  `/dev/lidar`, IMU (0x28), INA226 (0x40). Reintenta 30 s porque el USB tarda.
- **ros** (25 s después, con los nodos en marcha). Solo avisa: `/scan` >= 5 Hz, `/bno055/imu` >= 20, `/odom` >= 20,
  `/odometry/filtered` >= 20, giroscopio del IMU en 3 y batería sobre 14.4 V.

Comandos: `journalctl -u wallens -f`, `sudo systemctl stop|restart wallens`. Los nodos que se lancen a mano mientras el
servicio corre duplican tópicos (pasó con el IMU y el freno): parar primero el servicio o matar solo el propio proceso por PID.

**Memoria compartida y SSH:** el servicio corre como `wallens`, el mismo usuario que entra por SSH. Sin `loginctl
enable-linger wallens` (lo hace `04-servicio.sh`), al cerrarse la última sesión SSH `systemd-logind` (`RemoveIPC=yes`) borra
los segmentos de Fast DDS en `/dev/shm` y los nodos dejan de comunicarse: el driver frena por falta de `/battery_state` y el
vigilante reinicia el LiDAR en bucle (2026-11-01, 16:23). Se arregla reiniciando el servicio; con linger no vuelve a pasar.

## Red ROS

`config/ros-red.env` fija `ROS_DOMAIN_ID=37` y `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`: el robot solo descubre nodos de la
propia Pi y de las IP que figuren en `ROS_STATIC_PEERS`. Un equipo del laboratorio en el mismo dominio no ve ni manda
nada al robot salvo que su IP esté en esa lista. Lo leen el servicio (`EnvironmentFile`) y `scripts/ros-env.sh` (para cada
shell de la Pi: `source ~/wallens/scripts/ros-env.sh`). Probado en la Pi (dominio 99): un shell sin `LOCALHOST` sigue
viendo a los nodos con `LOCALHOST` y al revés, así que los scripts lanzados desde la Pi funcionan igual.

PC: la IP del PC va en la Pi, fuera del repo, y hace falta reiniciar el servicio tras cambiarla:

```bash
# en la Pi (IP del PC con `ip -4 addr`)
mkdir -p ~/.config/wallens && echo "ROS_STATIC_PEERS=11.11.41.48" > ~/.config/wallens/peers.env
sudo systemctl restart wallens
# en el PC
ROS_STATIC_PEERS=$(scripts/pi-ip.sh) scripts/pc-ros.sh ros2 topic list
```

El PC tiene `ufw` activo, que bloquea el DDS entrante (medido 2026-11-01: ni el dominio 37 actual llega al PC, con o sin
peers). Para dejar pasar solo a la Pi: `sudo ufw allow from <ip pi> proto udp to any port 16650:16899` (puertos DDS del
dominio 37: 7400 + 250 × 37 en adelante). Sin probar todavía de punta a punta por ese motivo.

## CPU

El bus I²C por software espera bit a bit en el kernel. Medido con el servicio completo: núcleo 3 a ~26 % (solo el IMU, fijado con
`taskset -c 3` en `imu.launch.py`), núcleos 0-2 a 15-18 % (`CPUAffinity=0 1 2` en el servicio), Pi al ~19 %, 43 °C, sin throttling.
Con el driver Python del IMU (45 bytes, 50 Hz) el nodo llegaba al 85 % y el EKF dejaba de cumplir su ritmo.
Presupuesto estimado (no medido) para SLAM + Nav2: 1-1.5 núcleos; video JPEG 640x480 a 15 fps ~0.3; H.264 por el codificador
de la Pi 4 casi nada.

Reparto actual (2026-11-01): **núcleo 2 solo para el driver** con prioridad de tiempo real (`taskset -c 2 chrt -f 50` en
`robot.launch.py`, `LimitRTPRIO=50` en la unidad), núcleo 3 para el IMU, y el resto del servicio y el mapeo en los núcleos 0-1
(`CPUAffinity=0 1`). Motivo: con SLAM + Nav2 en marcha el bucle del driver (lectura de motores cada 33 ms, un solo hilo) se
atrasó hasta 352 ms; pasados los 200 ms salta el watchdog del bus y al tercero en un minuto el driver se apaga (pasó dos veces).
El mapa puede ir lento; las ruedas no.

## Batería y motores: lo aprendido

- Tras reiniciar la Pi el watchdog de los motores queda en 255 y rechaza las escrituras ("data value is out of range", error 4):
  el driver lo limpia (reg. 98 = 0) antes de configurar.
- Con la LiPo casi agotada el voltaje cayó a 12.2 V; `wallens_power` publicó `battery_critical` bajo 13.6 V y el driver
  frenó las ruedas (los giros abortaban por tiempo). Es el comportamiento esperado; cargar antes de las rutas.
- Los motores leen 12 V aunque la batería marque 14-15 V: hay un regulador en medio.

## Pendiente

Cero del gripper al arrancar (bajar el tornillo al tope inferior), SLAM/Nav2, envío de imagen,
auditoría de seguridad/eficiencia/confiabilidad (prompt en la conversación; el informe iría en `docs/auditoria-*.md`).
