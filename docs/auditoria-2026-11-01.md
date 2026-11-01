# Auditoría de Wallens — 2026-11-01

Alcance: código en `master` (cd227a8 + `rutas.py` sin commitear), configuración, launch, systemd, scripts de instalación y de
banco, docs y memoria del proyecto. Solo lectura: no se movió el robot, no se tocaron servicios, EEPROM ni `config.txt`.

**Limitación importante:** la Pi no estaba en la red durante la auditoría (`scripts/pi-ip.sh` no encontró la MAC
`d8:3a:dd:46:9f:2b` en 11.11.41.0/24, y la IP anterior 11.11.41.25 ya responde otro equipo). No se midió nada en vivo
(CPU, temperatura, journal, versiones). Los números de CPU de este informe vienen de `docs/sistema.md` o son estimaciones,
y están marcados así. La sección [Mediciones pendientes](#mediciones-pendientes-en-la-pi) trae los comandos de solo lectura
para completarlo.

Niveles de evidencia: **confirmado** (visto en el código o la configuración), **probable** (deducido del código y del
comportamiento documentado del hardware/ROS, sin probar), **hipótesis** (hace falta medir).

---

## Resumen

1. **Seguridad:** no hay parada de emergencia física (descartada por el equipo: riesgo aceptado); todo lo que detiene las
   ruedas es software (timeouts, watchdog del bus, freno LiDAR, corte por batería). Esas capas están bien pensadas y en
   general fallan hacia "frenar".
2. **Mayor riesgo mecánico:** el tornillo sin fin pierde la posición multivuelta al cortar la energía y arriba no tiene tope
   por corriente; `test-tornillo-ciclos.py` acepta una posición recién encendida y puede mandarlo más allá del tope superior.
   Solución acordada: booleano `calibrado` guardado en la RAM del motor y homing al iniciar el programa.
3. Hay caminos que **saltan el freno LiDAR**: `ros/wallens_driver/launch/driver.launch.py` no remapea `cmd_vel` y lo
   recomienda un docstring; cualquier equipo ROS 2 del laboratorio en el dominio 0 puede publicar `/cmd_vel_safe`.
4. **Protección de batería incompleta:** si `wallens_power` no arranca (INA226 ausente) o se cae, el driver nunca frena por
   batería: el último valor de `/battery_critical` se mantiene para siempre.
5. **Bug en el freno:** `freno_lidar.py` comparte por referencia el `Twist` recibido y lo pone a cero al bloquear; la
   histéresis no funciona y `/freno_obstaculo` alterna true/false (el aborto por bloqueo de `rutas.py` no salta).
6. **El freno no ve** obstáculos de menos de ~16 cm de alto (frutas en el suelo, bordes, desniveles) ni nada a los lados en
   el cono de ±40°; el giro en el sitio no se frena nunca.
7. **Confiabilidad:** un fallo fatal del driver termina con código 0 y `Restart=on-failure` no reinicia (la doc dice que sí).
   `rutas.py` no detecta que `/odometry/filtered` dejó de llegar y seguiría mandando velocidad hasta 40 s.
8. **Instalación desde cero incompleta:** los scripts no instalan `robot_localization` ni `dynamixel_sdk`, el driver del
   LiDAR se clona sin fijar versión y no se compila en Release.
9. **Eficiencia:** con 19 % de CPU medido hay margen para SLAM + Nav2 + video H.264 (estimado: cabe). El cuello real es el
   bus Dynamixel a 57600 baudios: el ciclo de salud ya ocupa casi todo el periodo de 33 ms (estimado) y no hay sitio para
   sumar los IDs 3 y 5 a 30 Hz sin cambiar la forma de leer.
10. **Estado general:** sólido para un prototipo. Los P0 se arreglan con hardware (E-stop) y un homing obligatorio; los P1
    son cambios cortos, casi todos de pocas líneas.

| Área | Nota | Por qué |
|---|---|---|
| Seguridad | 5/10 | Buen diseño de software que falla hacia frenar; sin E-stop físico, freno con zonas ciegas y caminos para saltarlo |
| Confiabilidad | 6/10 | Arranque con chequeo previo y torque verificado al apagar; política de reinicio inconsistente, instalación no reproducible |
| Eficiencia | 7/10 | CPU holgada y el IMU aislado en el núcleo 3; bus Dynamixel lento y builds sin optimizar |

---

## Hallazgos

P0 = puede dañar el robot o a alguien. P1 = falla probable o capa de seguridad que no cumple. P2 = conviene arreglar.
P3 = limpieza. Esfuerzo: S (< 1 h), M (medio día), L (días).

| # | P | Área | Hallazgo | Evidencia | Impacto | Arreglo propuesto | Esf. | Estado |
|---|---|---|---|---|---|---|---|---|
| 1 | P0 (riesgo aceptado) | Seg. | No hay parada de emergencia física | Confirmado: README y `docs/sistema.md` la listan como pendiente; ningún código lee un GPIO de E-stop | Con un fallo de software, un script mal escrito o un comando remoto, la única forma de parar es tirar de la batería | **Decisión del equipo (2026-11-01): la seta no es viable.** Mitigación por software: tópico de parada que el driver respete por encima de todo; pruebas en el suelo a baja velocidad con alguien junto al interruptor de la batería. Ver [Parada sin E-stop](#parada-sin-e-stop-físico) | S | Mitigado: `/parada` (ce3a7d5); sin seta, riesgo aceptado |
| 2 | P0 | Seg. | El tornillo puede pasar el tope superior tras un corte de energía | Confirmado en código: `test-tornillo-ciclos.py:104` solo exige `arriba-200 ≤ pos ≤ abajo+200`; `docs/tornillo.md`: arriba "no hay tope por corriente". Probable (comportamiento del XM430 en modo multivuelta): al encender, la posición vuelve a un valor dentro de una vuelta (0–4095), que **pasa** ese chequeo | Tras apagar y encender, ir a `arriba_util` = -11517 puede ser hasta ~3 vueltas más arriba que el tope real: el gripper se sale del recorrido o se fuerza la mecánica. El aborto por corriente (250 sostenido) puede no saltar si arriba no hay tope duro | **Booleano `calibrado`** que bloquea todo movimiento absoluto mientras sea falso; homing al tope inferior por corriente **al iniciar el programa definitivo** (no al prender el robot). El booleano se guarda en la RAM del motor, no en la Pi. Ver [Calibración del tornillo](#calibración-del-tornillo-booleano-calibrado) | M | Pendiente (booleano `calibrado` y homing: nodo del brazo) |
| 3 | P1 | Seg. | `driver.launch.py` salta el freno LiDAR | Confirmado: `ros/wallens_driver/launch/driver.launch.py` no remapea `cmd_vel`; los docstrings de `avance-1m.py`, `giro-360.py`, `giro-ekf.py`, `giro-imu.py`, `giro180-avance1m.py`, `movimiento-basico.py` e `imu-reposo.py` indican lanzar ese launch, y esos scripts publican `/cmd_vel` | Las pruebas lanzadas así mueven el robot sin freno por obstáculo | Añadir `remappings=[('cmd_vel', 'cmd_vel_safe')]` al launch del paquete (o borrarlo y usar solo `robot.launch.py`), y actualizar los docstrings | S | Resuelto 4353f02 |
| 4 | P1 | Seg. | Sin protección de batería si `wallens_power` falta o muere | Confirmado: `wallens_driver.cpp:96-102` solo reacciona a mensajes; `bateria_critica_` empieza en `false` y no caduca. `wallens_power` lanza excepción y sale si el INA226 no responde al arrancar (`wallens_power.cpp:90`); el launch no lo reinicia ni para el resto; la fase `ros` del chequeo solo avisa | El robot anda con la batería agotándose bajo 3.4 V/celda (daño a la LiPo, caída del riel y reinicios) | El driver debe exigir `/battery_state` reciente (p. ej. < 2 s) además de `battery_critical == false`, con un parámetro `require_battery` para el banco. Diff en [D2](#d2-driver-batería-con-caducidad-y-comandos-no-finitos) | S | Resuelto 20eb434 (falta probar en el robot) |
| 5 | P1 | Seg./Conf. | El freno modifica el comando guardado y rompe la histéresis | Confirmado: `freno_lidar.py:65` hace `out.linear = self.cmd.linear` (los mensajes rclpy asignan submensajes por referencia); `:79` `out.linear.x = 0.0` pone a 0 también `self.cmd`. En el ciclo siguiente sin `/cmd_vel` nuevo, `v == 0` → `set_freno(False)` | `/freno_obstaculo` alterna a la frecuencia de llegada de `/cmd_vel`; la histéresis de 5 cm no se aplica; `rutas.py` reinicia el contador de bloqueo y no aborta a los 1.5 s. La salida a las ruedas sí queda en 0 (no es inseguro, pero la señal es falsa) | Copiar campos, no objetos. Diff en [D1](#d1-freno-copia-del-comando-reloj-monótono-y-valores-no-finitos) | S | Resuelto 2678742; tests 2e69dd3 |
| 6 | P1 | Seg. | Zonas ciegas del freno LiDAR | Confirmado (geometría): LiDAR a 163 mm del suelo, plano horizontal → no ve nada más bajo; sector en cono ±40° desde el LiDAR: a 15 cm cubre solo ±12.6 cm de lado; `v == 0` (giro en el sitio) nunca se frena; lecturas < 6 cm, 0 o NaN se descartan, y si **todas** son inválidas (LiDAR tapado) `distancia_minima` da `None` → no bloquea | Choque con frutas, cajas bajas, pies de mesa a los lados, escalones; el gripper puede golpear al girar; un LiDAR tapado o cegado deja avanzar | (a) Zona rectangular por huella: `0 < x < frente + stop` y `|y| < semiancho + margen` en el marco del robot; (b) en giro, bloquear `w` si hay puntos dentro del radio de barrido + margen; (c) bloquear si el sector tiene menos de N lecturas válidas; (d) para objetos bajos hace falta otro sensor (ToF/ultrasonido frontal o la cámara). Necesito las medidas de la huella ([preguntas](#preguntas-abiertas)) | M | Parcial: (c) 2678742; (a) y (b) 4a259f4 y 256363d (huella real, tests; falta probar en el robot); (d) riesgo aceptado por el usuario (2026-11-01): entorno sin objetos sueltos de menos de 16 cm |
| 7 | P1 | Seg. | Cualquier equipo del laboratorio puede mandar comandos | Confirmado: no hay `ROS_DOMAIN_ID`, `ROS_AUTOMATIC_DISCOVERY_RANGE` ni `ROS_LOCALHOST_ONLY` en el repo ni en `wallens.service`. Hipótesis: si la red deja pasar multicast, un `teleop` de otro grupo en el dominio 0 llega a `/cmd_vel`, y cualquiera puede publicar `/cmd_vel_safe` (sin freno) o `battery_critical: false` | Movimiento no esperado; el freno solo cubre `/cmd_vel` | En el servicio: `Environment=ROS_DOMAIN_ID=<único>` y `Environment=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`; para el PC usar `ROS_STATIC_PEERS` (pendiente ya anotado de descubrimiento sin multicast). Verificar en la Pi qué variables tiene `~/.bashrc` | S | Resuelto 5913c3b (`LOCALHOST` + `ROS_STATIC_PEERS`, instalado); sin probar desde otro PC: el `ufw` del PC bloquea el DDS |
| 8 | P1 | Conf. | Un fallo fatal no reinicia el servicio, aunque la doc dice que sí | Confirmado: en `ciclo()` un `Fatal` o 5 fallos llaman `rclcpp::shutdown()` y `main` devuelve 0 (`wallens_driver.cpp:174-185, 371-382`). Probable: `ros2 launch` termina con 0 cuando lo cierra la acción `Shutdown`. `Restart=on-failure` no reinicia con 0. `robot.launch.py:80` y `docs/sistema.md` dicen "systemd reintenta" | Un tirón del USB o un bajón del riel de 12 V deja el robot muerto hasta reiniciarlo a mano; el journal muestra "Deactivated successfully", lo que confunde | Decidir la política (pregunta 7). Propuesta: `Restart=always`, `RestartSec=10`; los errores de hardware persistentes ya los bloquea la fase `previa` del chequeo (lee el registro 70), así que no hay bucle peligroso. Y que el driver salga con 1 en un `Fatal` (diff [D3](#d3-código-de-salida-y-servicio)) | S | Resuelto 9e0a44f (falta probar en el robot) |
| 9 | P1 | Seg. | `rutas.py` no detecta odometría congelada | Confirmado: `rutas.py:34-43, 83-110` usan el último `d["x"]`/`d["th"]` sin mirar su edad; `TIEMPO_MAX = 40` s por tramo | Si el EKF muere a mitad de un tramo, el robot sigue avanzando (hasta 4.8 m a 0.12 m/s) o girando 40 s. Además Ctrl+C no manda cero (lo cubre el timeout del driver, 1 s en total, ver #12) | Guardar la hora de cada mensaje y abortar si pasa de 0.3 s; `frenar()` también en `KeyboardInterrupt`. Diff en [D4](#d4-rutaspy-odometría-congelada-y-ctrlc) | S | Resuelto 2b6f73c (falta probar en el robot) |
| 10 | P1 | Seg./Conf. | Scripts de banco abren el bus de los motores sin bloqueo y por defecto en `/dev/ttyUSB0` | Confirmado: `test-tornillo.py:19`, `test-tornillo-ciclos.py:12`, `test-gripper.py:11`, `test-avance.py:19`, `identify-dynamixel.py`, `scan-dynamixel.py` usan `/dev/ttyUSB0`; ninguno comprueba si `wallens` está activo. El LiDAR (CP2102) también es un `ttyUSB*` | Con el servicio activo, dos procesos escriben en el mismo bus: paquetes corruptos, el driver cae, el script puede abortar a mitad de un movimiento del gripper. Sin argumento, el script puede abrir el LiDAR | Por defecto `/dev/dynamixel` y abortar si `systemctl is-active --quiet wallens` (o un `flock` sobre `/run/lock/wallens-dynamixel` que tome también el driver). Diff en [D5](#d5-scripts-de-banco-puerto-y-bloqueo) | S | Resuelto 9a65134 |
| 11 | P1 | Conf. | La instalación desde cero no llega a un robot que funcione | Confirmado: ningún script instala `ros-jazzy-robot-localization` ni `ros-jazzy-dynamixel-sdk`; README paso 4 ("copiar el repo… `colcon build`") es manual; `build-lidar.sh` clona `ldlidar_stl_ros2` sin fijar commit; `01-ros2.sh` instala `ros-jazzy-bno055`, que ya no se usa; `00-base.sh` hace `apt-get upgrade` sin fijar versiones | Tras reflashear la SD el servicio no arranca, o arranca con otra versión del driver del LiDAR o del kernel (el i2c-gpio depende del kernel) | `05-ros-ws.sh`: instala dependencias (`rosdep install --from-paths ros`), crea los symlinks, fija el commit del LiDAR y compila con `--cmake-args -DCMAKE_BUILD_TYPE=Release`. Guardar `dpkg -l` de la Pi buena en el repo. Hacer la imagen de respaldo de la SD (ya pendiente) | M | Parcial: `05-ros-ws.sh` y `config/versiones-pi.txt` 7ffcf64; falta la imagen de respaldo de la SD |
| 12 | P2 | Seg. | Hasta ~1 s de movimiento tras perder al que manda | Confirmado: el freno republica el último `/cmd_vel` durante `cmd_timeout = 0.5` s (`freno_lidar.py:61`) y el driver lo sostiene otros 0.5 s (`wallens_driver.yaml: cmd_timeout`) | A 0.15 m/s son ~15 cm más la rampa (~4 cm) después de que el script o Nav2 se cuelgan | Que el freno republique cada mensaje con su hora original y no rellene: publicar solo en `on_cmd` (y en el ciclo solo cuando cambia el estado del freno), o bajar ambos timeouts a 0.25 s. Nav2 publica a 20 Hz, así que 0.25 s es suficiente | S | Resuelto 46d971d (ambos timeouts a 0.25 s) |
| 13 | P2 | Seg. | `scan_timeout` de 1 s es largo | Confirmado: `wallens_safety.yaml: scan_timeout: 1.0`; el LD19 va a ~10 Hz | Si el LiDAR se desconecta, el robot avanza hasta 15 cm a ciegas | 0.3 s (3 vueltas perdidas) | S | Resuelto 46d971d |
| 14 | P2 | Seg. | Comandos con NaN pasan por el freno y el driver | Confirmado: `std::clamp(NaN, …)` devuelve NaN (`wallens_driver.cpp:192`); en Python `NaN != 0.0` y la comparación de distancia es falsa. Probable: en aarch64 `lround(NaN)` da 0, así que hoy frena por casualidad (comportamiento no definido por el estándar) | Un nodo con un bug podría mandar un valor arbitrario si cambia el compilador o la plataforma | Rechazar valores no finitos en los dos nodos ([D1](#d1-freno-copia-del-comando-reloj-monótono-y-valores-no-finitos), [D2](#d2-driver-batería-con-caducidad-y-comandos-no-finitos)) | S | Resuelto 2678742 (freno) y 20eb434 (driver) |
| 15 | P2 | Conf. | El freno mide tiempos con el reloj de pared | Confirmado: `time.time()` en `freno_lidar.py:54,57,60`. La Pi no tiene RTC: el reloj salta cuando sincroniza NTP | Un salto hacia atrás haría "frescos" un comando o un scan viejos durante lo que dure el salto | `time.monotonic()` ([D1](#d1-freno-copia-del-comando-reloj-monótono-y-valores-no-finitos)). Igual en los scripts de `odometria/` | S | Parcial: freno 2678742; los scripts de `odometria/` aún usan `time.time()` |
| 16 | P2 | Conf. | Salto de yaw si el BNO055 se reinicia en marcha | Probable: `ekf.yaml` fusiona el yaw absoluto con `imu0_relative: true` (el cero se fija con el primer mensaje); `wallens_imu` reconfigura el chip si pierde el modo (`wallens_imu.cpp:143-146`) y en IMUPLUS el rumbo vuelve a empezar en 0 | Tras un bajón del IMU la pose del EKF gira de golpe lo que el robot había girado; con Nav2, replanificación o maniobras raras | Fusionar del IMU solo la velocidad angular z (o `imu0_differential: true`) y dejar el rumbo al integrador; o publicar un contador de reconfiguraciones y resetear el EKF (`/set_pose`) | S | Resuelto e1c192c (solo la velocidad de giro del IMU; falta repetir giro-ekf y `cuadrado` en el suelo) |
| 17 | P2 | Efic. | El bus a 57600 baudios va casi lleno | Estimado (sin medir): sync write ≈ 4 ms, sync read de 10 bytes × 2 ≈ 10 ms; cada segundo `revisar_salud()` añade 4 lecturas sueltas ≈ 19 ms → ese ciclo ronda los 33 ms del periodo a 30 Hz, más la latencia del adaptador USB | Ciclos tardíos una vez por segundo (jitter en `/odom`); no hay margen para meter los IDs 3 y 5 en el mismo ciclo | Leer 126–146 (21 bytes: corriente, velocidad, posición, trayectorias, voltaje, temperatura) en el mismo sync read: ≈ 13.5 ms y desaparecen las lecturas sueltas ([D6](#d6-voltaje-y-temperatura-en-el-sync-read)). A medio plazo subir el bus a 1 Mbps (escritura de EEPROM: decisión tuya, con el procedimiento de [plan](#fase-3--antes-de-la-navegación-autónoma)) | S | Parcial: D6 7a81704; el bus a 1 Mbps queda para la Fase 3 |
| 18 | P2 | Conf. | Logs: se pierden al reiniciar y los de ROS van a la SD | Confirmado: `00-base.sh` deja journald en RAM; el servicio no define `ROS_LOG_DIR`, así que `ros2 launch` escribe en `~/.ros/log` en la SD | Tras un corte no queda el journal para saber qué pasó; escrituras continuas en la SD | `ROS_LOG_DIR=/run/wallens-log` (tmpfs) y journald persistente con tope (`Storage=persistent`, `SystemMaxUse=50M`, `SyncIntervalSec=5m`): poco desgaste y los últimos minutos sobreviven a un reinicio limpio. Para cortes duros, protección real = imagen de respaldo + (más adelante) raíz en solo lectura (`overlayroot`) | S | Parcial: `ROS_LOG_DIR` en tmpfs 9e0a44f; falta journald persistente |
| 19 | P2 | Conf. | Nodos que mueren sin que nadie se entere | Confirmado: solo el driver tiene `on_exit=Shutdown()`. Si mueren el freno, el LiDAR, el EKF o la potencia, el resto sigue | Freno muerto = el robot no se mueve (seguro, pero silencioso); potencia muerta = ver #4; EKF muerto = ver #9 | `respawn=True, respawn_delay=2.0` en freno, LiDAR, IMU, EKF y potencia; y que la fase `ros` del chequeo corra cada minuto (o un nodo vigilante) y publique un estado único | S | Parcial: `respawn` 6b66b25 y vigilante del LiDAR 99a4055; falta el chequeo periódico de la fase `ros` |
| 20 | P2 | Conf. | Los scripts del gripper y del tornillo apagan en el orden que el watchdog rechaza | Confirmado: `finally` de `test-tornillo.py:172-174`, `test-tornillo-ciclos.py:134-136` y `test-gripper.py:254-256` escriben torque 0 **antes** de limpiar el watchdog (98) — el mismo problema ya corregido en el driver | Si el watchdog disparó, el torque queda encendido y el script pide desconectar la alimentación | Invertir: primero `98 = 0`, después `64 = 0`, y leer 64 para confirmar (como hace `test-avance.py`) | S | Resuelto 9a65134 |
| 21 | P2 | Conf. | El EKF no aprovecha que el robot no se mueve de lado | Confirmado: `ekf.yaml odom0_config` no fusiona `vy` | Más deriva lateral, sobre todo con patinaje en los giros | Fusionar `vy` de las ruedas (siempre 0) con covarianza baja, como recomienda `robot_localization` para diferenciales; el driver ya publica `twist.linear.y = 0`, solo hay que bajar `twist.covariance[7]` de 1e6 a 1e-3 | S | Resuelto e1c192c |
| 22 | P2 | Seg. | El giro en el sitio no se frena | Confirmado: `freno_lidar.py:68` solo mira `v != 0` | Con el gripper fuera de la huella, un giro barre lo que haya alrededor | Incluido en #6 (b) | — | Resuelto con #6b (4a259f4, 256363d) |
| 23 | P3 | Cód. | Parámetros duplicados | Confirmado: 10 V / 14.8 V / 55 °C en `wallens_driver.yaml`, `chequeo-arranque.py:15` y 3 scripts de banco; aviso 14.4 V en el chequeo y en `wallens_power.yaml`; yaw del LiDAR −90° en `lidar.launch.py` y `wallens_safety.yaml`; frecuencia del IMU en dos YAML; IDs 2/4 en el chequeo y el driver | Cambiar uno y olvidar otro; ya pasó con la z del LiDAR (`docs/lidar.md` dice 0.18 m) | Que el chequeo y los scripts lean `ros/wallens_driver/config/wallens_driver.yaml` y `wallens_power.yaml`; que el freno lea el yaw del TF (`tf2_ros.Buffer`) en vez de un parámetro | S | Pendiente |
| 24 | P3 | Doc. | Documentación desactualizada | Confirmado: `docs/lidar.md` z = 0.18 m (es 0.12675); `chequeo-arranque.py:112` "calib_status sale cada 10 s" (ahora 1 Hz); `imu-ejes.py` usa `/bno055/imu_raw`, que el driver nuevo no publica; `imu-directo.py`, `imu-calibrar.py` y `docs/imu.md` dicen "parar el nodo bno055" (ahora es `wallens_imu` dentro del servicio); `config/bno055.yaml` sin uso; `robot.launch.py:80` y `docs/sistema.md` "systemd reintenta" (#8); README dice "Peacefair (CH340)" y el encargo dice U2D2 | Pruebas que no funcionan o que se corren con el servicio activo | Corregir textos; en `imu-ejes.py` usar `/bno055/imu` (ahora trae aceleración lineal, sin gravedad: para ejes hace falta la aceleración con gravedad → añadir `bno055/imu_raw` al driver o leer el bloque 0x08) | S | Parcial: ba9bda9 (z del LiDAR, calib_status, wallens_imu, reinicios); faltan `imu-ejes.py`, `config/bno055.yaml` y README (adaptador) |
| 25 | P3 | Cód. | Sin tests automáticos | Confirmado: ningún `test/` ni `ament_add_gtest`/`pytest` | Las regresiones (como #5) solo se ven en el suelo | Tests de mesa sin hardware: `distancia_minima` y la lógica del freno con scans sintéticos (pytest); `ticks_entre` y la cinemática del driver separadas en una función libre (gtest) | M | Parcial: pytest del freno 2e69dd3; falta gtest del driver |
| 26 | P3 | Efic. | Paquetes C++ sin optimizar | Probable: los `CMakeLists.txt` no fijan `CMAKE_BUILD_TYPE` y `build-lidar.sh` compila sin `--cmake-args`; sin tipo, CMake compila con `-O0` | Poco impacto hoy (los nodos son ligeros), más con SLAM/Nav2 compilados desde fuente | `colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release` | S | Resuelto 7ffcf64 (`05-ros-ws.sh` compila en Release) |
| 27 | P3 | Conf. | Los scripts de banco reescriben la EEPROM en cada corrida | Confirmado: modo (11), límite de corriente (38) y límites de posición (48/52) en cada ejecución del gripper y del tornillo | Desgaste mínimo; el riesgo es que un script deje un valor distinto del que otro espera | Escribir solo si el valor leído difiere | S | Pendiente |

### Lo que está bien (para no romperlo)

- El watchdog del bus (200 ms) en ruedas y en los scripts del gripper y del tornillo: si el proceso muere o se mata con
  SIGKILL, los motores paran solos.
- El driver verifica el modelo, limpia el watchdog antes de configurar, comprueba la lectura del torque al apagar y trata el
  bit de error de hardware como fatal.
- `wallens_power` declara la batería crítica si deja de leer el INA226 (falla hacia frenar) y filtra caídas bajo carga
  con 5 muestras.
- El IMU descarta bloques corruptos por la norma del cuaternión y se reconfigura solo.
- La rampa de aceleración (`profile_acceleration: 20`, ~0.55 s hasta 0.15 m/s, ~4 cm de frenada) es razonable para el
  freno a 30 cm.

---

## Seguridad: qué pasa en cada fallo

| Evento | Qué pasa hoy | Estado |
|---|---|---|
| Se desconecta el USB de los motores | `txRxPacket` falla, 5 ciclos (~170 ms) → `apagar()` (sus escrituras fallan) → `Shutdown` de todo el launch. Los motores paran por su watchdog (200 ms), con torque encendido sosteniendo. El servicio no se reinicia (#8) | Seguro, no se recupera |
| Se corta el riel de 12 V | Igual que arriba. Al volver, los motores arrancan con torque apagado y sin watchdog; nadie los reconfigura hasta reiniciar el servicio | Seguro, no se recupera |
| Se desconecta el LiDAR | El freno bloquea el avance tras 1 s (#13); el giro sigue permitido | Seguro con retraso |
| LiDAR tapado o cegado (todo 0/NaN) | No bloquea (#6c) | **No seguro** |
| Se desconecta el IMU | El IMU reintenta y reconfigura; el EKF sigue con las ruedas; el chequeo solo avisa | Aceptable |
| El IMU se reinicia en marcha | Salto de yaw en el EKF (#16) | Probable fallo |
| Se desconecta el INA226 en marcha | 5 fallos → batería crítica → ruedas frenan | Seguro |
| INA226 ausente al arrancar o `wallens_power` muere | Nunca hay corte por batería (#4) | **No seguro** |
| La batería se hunde bajo carga | Corte tras 2.5 s seguidos bajo 13.6 V; se levanta a 13.9 V. Torque sigue encendido (ruedas paradas sosteniendo). Funcionó en la prueba de `giro90` | Seguro |
| El EKF deja de publicar | El driver sigue; `rutas.py` sigue mandando hasta 40 s (#9); Nav2 pararía por TF caducado | **No seguro con `rutas.py`** |
| El que manda `/cmd_vel` se cuelga | Hasta ~1 s de movimiento (#12) | Aceptable, mejorable |
| `/cmd_vel` absurdo (enorme, ±inf) | Se recorta a 0.15 m/s y 1 rad/s y luego a 60 rpm por rueda conservando la curva | Seguro |
| `/cmd_vel` con NaN | Hoy probablemente 0 por cómo redondea aarch64 (#14) | Seguro por casualidad |
| Publicar en `/cmd_vel_safe` o con `driver.launch.py` | Sin freno (#3, #7) | **No seguro** |
| Apagado del servicio (SIGINT) | Driver frena, espera la rampa, limpia el watchdog, apaga torque y lo verifica. `TimeoutStopSec=15` alcanza | Seguro |
| SIGKILL o cuelgue del driver | El watchdog para las ruedas en 200 ms; torque queda encendido | Seguro |
| Gripper/tornillo tras un corte | Posición multivuelta perdida (#2); nadie hace homing | **Riesgo mecánico** |

### Parada sin E-stop físico

La seta se descartó (no es viable en el chasis). Riesgo aceptado; mitigaciones sin hardware nuevo:

- Tópico `/parada` (`std_msgs/Bool`, `transient_local`) que el driver de ruedas y el del brazo obedecen por encima de
  `/cmd_vel_safe`: con `true`, velocidad 0 y ninguna escritura de meta hasta recibir `false`. Se publica desde el PC con un
  atajo de teclado, o desde la Pi con `ros2 topic pub --once`. No sustituye a la seta: si el problema es el propio software
  o la red, no llega.
- Pruebas en el suelo a ≤ 0.10 m/s, con una persona junto al interruptor o conector de la batería.
- El watchdog del bus (200 ms) sigue siendo la parada de último recurso si el proceso muere.

### Calibración del tornillo: booleano `calibrado`

Decisión del equipo (2026-11-01): un booleano que diga si el tornillo está calibrado. Si es falso, el robot no mueve el
tornillo a ninguna posición absoluta.

- **Cuándo calibrar: al iniciar el programa definitivo** (o con el primer comando al brazo), no al prender el robot. Si el
  servicio hiciera el homing al arrancar, el gripper se movería cada vez que se reinicia la Pi o vuelve la energía, sin
  nadie avisado. Al iniciar el programa hay alguien presente.
- **Cómo:** bajar despacio hasta el tope inferior por corriente (el único que se detecta solo; `buscar_tope()` de
  `test-tornillo.py`, ~9 rpm, corriente 120). Los límites se guardan relativos a ese tope:
  `arriba_util = abajo - 35763` ticks (de `calibracion-tornillo.json`: 24245 − (−11517)).
- **Dónde guardar el booleano: en la RAM del motor, no en la Pi.** Un archivo en la Pi se equivoca en los dos sentidos: si se
  corta solo el riel de 12 V con la Pi encendida, el archivo dice "calibrado" y la posición ya no vale; si se reinicia la
  Pi con el motor encendido, la posición sigue valiendo. Propuesta: tras el homing, escribir una marca en un registro de RAM
  que no se use (p. ej. Indirect Address 28, dirección 222) y al arrancar leerla: si está, `calibrado = true`; si volvió a
  su valor por defecto, hubo corte y hay que calibrar. **Por verificar en el banco** que ese registro vuelve al valor por
  defecto al quitar los 12 V y que escribirlo no afecta a nada más. Si no funciona, calibrar siempre al iniciar el
  programa.
- **También vuelve a falso** si el motor reporta un error de hardware, si falla la comunicación (puede haber sido un corte)
  o si la posición leída sale del recorrido calibrado.
- `test-tornillo-ciclos.py` debe usar la misma comprobación hasta que exista el nodo del brazo.

---

## Eficiencia

### Datos disponibles

Medido en sesiones anteriores (`docs/sistema.md`): servicio completo con la Pi al ~19 % de CPU, núcleo 3 al ~26 % (solo el
IMU por el I²C por software), núcleos 0–2 al 15–18 %, 43 °C, sin throttling. Hoy no se pudo repetir (Pi fuera de la red).

### Presupuesto estimado para la navegación autónoma (sin medir; confirmar con la Pi)

| Carga | Estimado | Núcleos |
|---|---|---|
| Servicio actual | ~0.6 núcleo (dato medido) | 0–3 |
| `slam_toolbox` en modo online async, LD19 a 10 Hz, resolución 5 cm | 0.2–0.4 | 0–2 |
| Nav2 (controller a 20 Hz, planner a 1 Hz, dos costmaps pequeños, BT) | 0.4–0.8 | 0–2 |
| 1 cámara MJPEG 640×480 a 15 fps → H.264 por hardware (`v4l2h264enc`) → RTP | 0.1–0.2 (decodificar MJPEG es lo que cuesta) | 1–2 |
| 2.ª cámara igual | 0.1–0.2 | 1–2 |
| **Total** | **~1.4–2.2 de 4 núcleos** | |

Conclusión (estimada): cabe, con margen de ~2 núcleos. Memoria: de sobra en 8 GB. El riesgo no es la media sino los picos
(planner, cierre de lazo de SLAM) que retrasan el ciclo del driver o del EKF.

### Qué aislar y cómo

1. Mantener el IMU solo en el núcleo 3.
2. Driver y EKF con prioridad: `prefix='chrt -f 20'` en sus `Node` (necesita `LimitRTPRIO=20` en el servicio) o, más simple,
   `Nice=-5` para ellos. Medir el jitter de `/odom` antes y después (`ros2 topic hz /odom` y la desviación estándar).
3. Video en **otro servicio** systemd con `CPUAffinity=1 2`, `Nice=10`, fuera de DDS: GStreamer `v4l2src ! jpegdec !
   v4l2h264enc ! rtph264pay ! udpsink` o WebRTC. Mandar imágenes crudas por DDS sobre el Wi-Fi del laboratorio no cabe
   (640×480 RGB a 15 fps ≈ 110 Mbit/s); H.264 a 1–2 Mbit/s sí.
4. USB: en la Pi 4 todos los puertos USB 2.0 comparten un mismo hub interno del VL805 (480 Mbit/s) con el adaptador de los
   motores y el LiDAR. Hipótesis: dos cámaras en YUYV reservan casi todo el ancho isócrono y pueden fallar al abrir la
   segunda o meter latencia. Usar MJPEG en las dos (C270 y Suyin lo ofrecen) y medir con `dmesg` al abrirlas.
5. QoS: `/scan` y el IMU con `sensor_data` (best effort, profundidad 5) en todos los consumidores; `/cmd_vel` fiable con
   profundidad 1 (con 10, un consumidor lento procesa comandos viejos en ráfaga).
6. A C++ solo si se mide que hace falta: el freno en Python recorre ~450 puntos dos veces a 10 Hz y su ciclo a 30 Hz; debería
   estar bajo el 5 % de un núcleo. Medir con `top -H -p <pid>` antes de reescribirlo.
7. Bajar de frecuencia: el chequeo de calibración del IMU a 1 Hz está bien; el IMU a 30 Hz es lo justo para el EKF a 30 Hz.
   No subirlo (cada byte cuesta CPU en el kernel).

### Latencia sensor → motor (estimada)

`/scan` (10 Hz, hasta 100 ms de antigüedad) → freno (ciclo de 33 ms) → `/cmd_vel_safe` → driver (ciclo de 33 ms, ~14 ms de
bus) → rampa del motor. Peor caso ≈ 170 ms + ~0.55 s de rampa a 0.15 m/s: 2.5 cm + 4 cm ≈ 6.5 cm de frenada tras detectar.
Con el umbral a 30 cm desde el LiDAR, queda margen si el frente del robot está a menos de ~20 cm del LiDAR (pregunta 3).

---

## Diffs sugeridos

### D1. Freno: copia del comando, reloj monótono y valores no finitos

```diff
--- a/ros/wallens_safety/scripts/freno_lidar.py
+++ b/ros/wallens_safety/scripts/freno_lidar.py
@@ def on_scan(self, m):
-        self.t_scan = time.time()
+        self.t_scan = time.monotonic()
@@ def on_cmd(self, m):
-        self.cmd, self.t_cmd = m, time.time()
+        if not all(math.isfinite(x) for x in (m.linear.x, m.angular.z)):
+            self.get_logger().error("cmd_vel no finito: se descarta", throttle_duration_sec=2.0)
+            return
+        self.cmd, self.t_cmd = m, time.monotonic()
@@ def ciclo(self):
-        ahora = time.time()
+        ahora = time.monotonic()
@@
         out = Twist()
-        out.linear, out.angular = self.cmd.linear, self.cmd.angular
+        # copiar valores: asignar el submensaje lo comparte y el freno pondría a 0 también self.cmd
+        out.linear.x, out.angular.z = self.cmd.linear.x, self.cmd.angular.z
         v = out.linear.x
```

Para #6c, en `distancia_minima` contar las lecturas válidas del sector y que `on_scan` guarde `None` también cuando
hay menos de, por ejemplo, 10: en `ciclo`, `d is None` con `requiere_scan` debe bloquear (hoy deja pasar).

### D2. Driver: batería con caducidad y comandos no finitos

```diff
--- a/ros/wallens_driver/src/wallens_driver.cpp
+++ b/ros/wallens_driver/src/wallens_driver.cpp
@@
+#include "sensor_msgs/msg/battery_state.hpp"
@@ WallensDriver()
+    bateria_timeout_ = declare_parameter("battery_timeout", 2.0);   // s; <= 0 desactiva (banco sin INA226)
+    sub_estado_bateria_ = create_subscription<sensor_msgs::msg::BatteryState>(
+        "battery_state", 10, [this](sensor_msgs::msg::BatteryState::ConstSharedPtr) {
+          ultima_bateria_ = std::chrono::steady_clock::now();
+          hay_bateria_ = true;
+        });
@@ void mandar_velocidades(...)
     double v = 0.0, w = 0.0;
-    if (!bateria_critica_ && hay_cmd_ && std::chrono::duration<double>(ahora - ultimo_cmd_).count() <= cmd_timeout_) {
+    const bool bateria_ok = !bateria_critica_ && (bateria_timeout_ <= 0 ||
+        (hay_bateria_ && std::chrono::duration<double>(ahora - ultima_bateria_).count() <= bateria_timeout_));
+    if (bateria_ok && hay_cmd_ && std::isfinite(cmd_v_) && std::isfinite(cmd_w_) &&
+        std::chrono::duration<double>(ahora - ultimo_cmd_).count() <= cmd_timeout_) {
```

(más los miembros `bateria_timeout_`, `ultima_bateria_`, `hay_bateria_`, `sub_estado_bateria_`, `<sensor_msgs>` en
`package.xml`/`CMakeLists.txt`, y un `RCLCPP_WARN_THROTTLE` cuando `bateria_ok` pasa a falso por caducidad.)

### D3. Código de salida y servicio

```diff
--- a/ros/wallens_driver/src/wallens_driver.cpp
+++ b/ros/wallens_driver/src/wallens_driver.cpp
@@ void ciclo()
     } catch (const Fatal& e) {
       RCLCPP_FATAL(get_logger(), "%s: se apaga el driver", e.what());
+      fallo_fatal_ = true;
       apagar();
@@
       if (++fallos_ >= max_fallos_) {
         RCLCPP_FATAL(...);
+        fallo_fatal_ = true;
         apagar();
@@ public:
+  bool fallo_fatal() const { return fallo_fatal_; }
@@ int main
   if (nodo) nodo->apagar();
   rclcpp::shutdown();
-  return codigo;
+  return (nodo && nodo->fallo_fatal()) ? 1 : codigo;
```

```diff
--- a/systemd/wallens.service
+++ b/systemd/wallens.service
@@ [Service]
-Restart=on-failure
+# El chequeo previo bloquea el arranque si hay error de hardware, así que reintentar siempre no entra en un bucle peligroso
+Restart=always
 RestartSec=10
+Environment=ROS_DOMAIN_ID=37
+Environment=ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
+Environment=ROS_LOG_DIR=/run/wallens-log
+RuntimeDirectory=wallens-log
```

(`ROS_DOMAIN_ID` de ejemplo: elegir uno y ponerlo igual en el PC. Con `LOCALHOST` el PC deja de ver la Pi hasta configurar
`ROS_STATIC_PEERS`; aplicar junto con ese pendiente. `RuntimeDirectory=wallens-log` crea `/run/wallens-log`.)

### D4. `rutas.py`: odometría congelada y Ctrl+C

```diff
--- a/scripts/odometria/rutas.py
+++ b/scripts/odometria/rutas.py
@@
+EDAD_MAX_ODOM = 0.3      # s sin /odometry/filtered: se aborta (el EKF murió)
@@ def odom(m):
-    d.update(x=..., y=..., th=...)
+    d.update(x=..., y=..., th=..., t_odom=time.monotonic())
@@
+def revisar_odom():
+    if time.monotonic() - d["t_odom"] > EDAD_MAX_ODOM:
+        raise Abortar("sin /odometry/filtered reciente")
@@ def avanzar / def girar (dentro del while, antes de enviar)
+        revisar_odom()
@@
-except Abortar as e:
+except (Abortar, KeyboardInterrupt) as e:
     frenar()
```

### D5. Scripts de banco: puerto y bloqueo

```diff
--- a/scripts/test-tornillo.py   (igual en test-tornillo-ciclos.py, test-gripper.py, test-avance.py)
+++ b/scripts/test-tornillo.py
@@
-PUERTO = _args[0] if _args else "/dev/ttyUSB0"
+PUERTO = _args[0] if _args else "/dev/dynamixel"
+if os.system("systemctl is-active --quiet wallens") == 0:
+    sys.exit("El servicio wallens usa el bus de los motores: sudo systemctl stop wallens")
@@ finally:
     try:
-        escribir(64, 1, 0)
-        escribir(98, 1, 0)
+        escribir(98, 1, 0)                      # primero el watchdog: disparado, rechaza la escritura del torque
+        escribir(64, 1, 0)
+        if leer(64, 1):
+            raise Aborto("el torque sigue encendido")
```

### D6. Voltaje y temperatura en el sync read

```diff
--- a/ros/wallens_driver/src/wallens_driver.cpp
+++ b/ros/wallens_driver/src/wallens_driver.cpp
-constexpr uint16_t LARGO_LECTURA = 10;
+constexpr uint16_t LARGO_LECTURA = 21;   // 126..146: corriente, velocidad, posición, trayectorias, voltaje, temperatura
@@ void revisar_salud()
-      const double voltaje = leer2(r->id, DIR_VOLTAJE) / 10.0;
-      const int temp = leer1(r->id, DIR_TEMPERATURA);
+      const double voltaje = sync_lectura_->getData(r->id, DIR_VOLTAJE, 2) / 10.0;
+      const int temp = static_cast<int>(sync_lectura_->getData(r->id, DIR_TEMPERATURA, 1));
```

(`revisar_salud()` se llama después de `leer_y_publicar()`, así que usa los datos del mismo ciclo.)

### D7. Launch del driver solo

```diff
--- a/ros/wallens_driver/launch/driver.launch.py
+++ b/ros/wallens_driver/launch/driver.launch.py
         Node(package='wallens_driver', executable='wallens_driver', name='wallens_driver',
-             output='screen', parameters=[config]),
+             output='screen', parameters=[config],
+             remappings=[('cmd_vel', 'cmd_vel_safe')]),   # nunca saltar el freno: sin freno_lidar no se mueve
```

---

## Plan de implementación

Regla para todas las pruebas: primero con el robot **sobre tacos, ruedas en el aire**, y el gripper sin carga; al suelo solo
cuando pase en el aire. Antes de cada prueba en el suelo: voltaje de la batería en reposo con multímetro.

### Fase 1 — Ya (antes de la próxima prueba en el suelo)

| Cambio | Hallazgos | Cómo probarlo sin riesgo |
|---|---|---|
| D1 (freno) | #5, #14, #15 | Robot en tacos. `ros2 topic pub -r 20 /cmd_vel` con 0.05 m/s y una caja a 27 cm: `/freno_obstaculo` debe quedarse en `true` (`ros2 topic echo`), no alternar; con la caja a 32 cm debe seguir frenado (histéresis); a 36 cm debe soltar. Publicar `nan` en `linear.x`: `/cmd_vel_safe` no debe recibir nada |
| D7 y docstrings | #3 | `ros2 launch wallens_driver driver.launch.py` y `ros2 node info /wallens_driver`: debe suscribirse a `/cmd_vel_safe` |
| D4 (`rutas.py`) | #9 | Tacos. Lanzar `rutas.py avance` y matar el EKF por PID a los 2 s: debe abortar en < 0.5 s y las ruedas parar |
| D5 (scripts de banco) | #10, #20 | Con el servicio activo, el script debe negarse a correr. Sin servicio, provocar el watchdog (Ctrl+Z al script > 200 ms, luego `kill`) y comprobar que el `finally` apaga el torque |
| Parar el uso de `test-tornillo-ciclos.py` hasta tener homing | #2 | — |
| `scan_timeout: 0.3` | #13 | Tacos. Desenchufar el LiDAR con `/cmd_vel` activo: medir en el log el tiempo hasta "sin /scan reciente" |

### Fase 2 — Antes de usar el gripper de forma autónoma

| Cambio | Hallazgos | Cómo probarlo |
|---|---|---|
| Booleano `calibrado` y homing al tope inferior al iniciar el programa; límites relativos a ese tope | #2 | Primero verificar la marca en RAM: escribirla, quitar los 12 V, volver a encender y leerla (debe haber vuelto al valor por defecto). Luego el homing a mano junto al robot, con la mano en la alimentación, corriente 120 y `VUELTAS_MAX` corto. Repetir 5 veces tras apagar y encender el riel: el tope debe repetirse en ±50 ticks. Con `calibrado = false`, un comando de posición debe rechazarse |
| Tópico `/parada` en driver (y brazo) | #1 | Tacos, ruedas girando: `ros2 topic pub --once /parada std_msgs/Bool "{data: true}"` → paran y no aceptan `/cmd_vel` hasta `false` |
| D2 (batería con caducidad) | #4 | Tacos. Matar `wallens_power` por PID: las ruedas deben parar en < 2.5 s. Arrancar sin el INA226 conectado: no deben moverse |
| D3 + `Restart=always` | #8 | Tacos. Desenchufar el USB de los motores: el servicio debe volver solo a los ~10 s de reconectar (`journalctl -u wallens`) |
| `respawn` en nodos no críticos | #19 | Matar cada nodo por PID y ver que vuelve |
| Nodo ROS del brazo (IDs 3 y 5) dentro del driver o con el mismo bloqueo del bus | #10, #17 | Antes, D6, para dejar sitio en el bus |

### Fase 3 — Antes de la navegación autónoma

| Cambio | Hallazgos | Cómo probarlo |
|---|---|---|
| Freno por huella rectangular, barrido en el giro, mínimo de lecturas válidas | #6, #22 | Test de pytest con scans sintéticos (sin robot). Luego tacos con cajas en las esquinas y a los lados |
| Sensor para obstáculos bajos (ToF/ultrasonido frontal o detección con la cámara) | #6d | Objetos de 5–15 cm delante, robot a 0.05 m/s con el E-stop en la mano |
| Aislamiento de red (D3, `ROS_STATIC_PEERS`) | #7 | Desde otro PC en el dominio 0, `ros2 topic pub /cmd_vel`: el robot no debe enterarse |
| EKF: `vy` y yaw del IMU solo como velocidad | #16, #21 | Repetir giro-ekf y `cuadrado` y comparar con cinta; reconfigurar el IMU a mano en marcha (desconectar SDA 1 s) y ver que la pose no salta |
| Script `05-ros-ws.sh`, versiones fijadas, build Release, imagen de la SD | #11, #26 | Flashear una SD nueva y llegar al servicio funcionando solo con los scripts |
| Logs (D3 `ROS_LOG_DIR`, journald persistente con tope) | #18 | `systemctl restart wallens` y ver que `journalctl -b -1` existe tras un reinicio |
| Subir el bus a 1 Mbps (opcional) | #17 | Escribe EEPROM (reg. 8): un motor cada vez, con el torque apagado y probando `ping` a 1 Mbps antes de pasar al siguiente; actualizar la YAML y el chequeo a la vez |
| Medir el presupuesto real con SLAM + Nav2 + video, y prioridad al driver/EKF | Eficiencia | `mpstat -P ALL 5`, `ros2 topic hz /odom` y su jitter, antes y después |

---

## Mediciones pendientes en la Pi

Todo de solo lectura. Con la Pi encendida: `ssh wallens@$(scripts/pi-ip.sh)`.

```bash
# Estado del servicio y por qué terminó la última vez
systemctl status wallens --no-pager; systemctl show wallens -p NRestarts,ExecMainStatus,Result
journalctl -u wallens -b --no-pager | grep -E 'FATAL|ERROR|WARN|Deactivated|exited' | tail -50
# Energía y temperatura (0x0 = nunca hubo bajo voltaje ni throttling)
vcgencmd get_throttled; vcgencmd measure_temp; vcgencmd measure_volts core
# CPU por núcleo y por proceso (10 s)
mpstat -P ALL 5 2; top -b -n 1 -o %CPU | head -25; ps -eLo pid,psr,pcpu,rtprio,ni,comm --sort=-pcpu | head -30
# Frecuencias y latencias de los tópicos (con ROS cargado)
ros2 topic hz /odom & ros2 topic hz /scan & ros2 topic hz /bno055/imu & sleep 10; kill %1 %2 %3
# Red ROS y reloj
env | grep -E '^ROS_|RMW'; grep -E 'ROS_' ~/.bashrc /etc/profile.d/* 2>/dev/null
timedatectl; journalctl -b | grep -iE 'time has been changed|clock' | head
# SD y logs
findmnt -no OPTIONS /; du -sh ~/.ros/log; df -h /
# Actualizaciones automáticas (pueden cargar CPU/disco o cambiar paquetes en marcha)
systemctl is-enabled unattended-upgrades apt-daily.timer apt-daily-upgrade.timer snapd 2>/dev/null
# USB: árbol y velocidades (comparten hub las cámaras, el LiDAR y el adaptador de motores?)
lsusb -t; lsusb | grep -iE '1a86|0403|10c4|046d|1e45'
# Módulo i2c-dev cargado de forma persistente (reproducibilidad)
lsmod | grep i2c; cat /etc/modules /etc/modules-load.d/* 2>/dev/null
# Versiones instaladas (para fijarlas en el repo)
dpkg -l | grep -E 'ros-jazzy-(robot-localization|dynamixel-sdk|rclcpp )'; git -C ~/ros_ws/src/ldlidar_stl_ros2 rev-parse HEAD
```

---

## Preguntas abiertas

1. **Batería en reposo:** voltaje con multímetro, robot apagado 10 min. Por debajo de ~15.2 V (3.8 V/celda) conviene cargar
   antes de repetir `giro90`, `giro180`, `ida-vuelta` y `cuadrado`.
2. **Riel de 12 V:** modelo y corriente máxima del regulador. ¿El INA226 mide la batería antes del regulador y por su shunt
   pasa la corriente de todo (Pi + motores)? ¿De dónde sale el 5 V de la Pi y con qué regulador?
3. **Huella del robot** (para #6 y la frenada): distancia del LiDAR al punto más adelantado y al más atrasado del chasis,
   ancho total, y cuánto sobresale el gripper (y su radio de barrido al girar en el sitio).
4. **Adaptador de los motores:** ¿U2D2 o el Peacefair USB-RS485 (CH340) que dicen el README y la regla udev? Cambia la
   latencia y la regla.
5. **E-stop:** descartada (no es viable); queda como riesgo aceptado con las mitigaciones de software.
6. **Tornillo:** ¿qué hay arriba del recorrido si se pasa (se sale la tuerca, choca, se fuerza)? Decide cuán urgente es #2.
7. **Política ante fallos:** ¿el robot debe reintentar solo tras perder el USB o el riel (propuesta), o quedarse parado hasta
   que alguien lo revise?
8. **Red:** ¿qué `ROS_DOMAIN_ID` usan el PC y los otros grupos del laboratorio? ¿La red deja pasar multicast?
