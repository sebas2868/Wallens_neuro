# Wallens

Robot diferencial con una Raspberry Pi 4 (8 GB) que busca un limón en el suelo con una cámara, lo agarra con la pinza
de atrás y lo deja en el punto que se le pida. Corre ROS 2 Jazzy en la Pi; desde el PC se mira y se manda todo por
ROS 2 dentro de Docker.

Este README sirve para empezar: entender el robot, montar el PC, conectarse y correr la misión. El detalle está en
[docs/](docs/).

## Cómo funciona

1. Mira al frente con `cam_frutas` y encuentra el limón (color y forma con OpenCV).
2. Se acerca hasta ~0.36 m y gira 180°: la pinza está **detrás** del robot.
3. Abre la pinza y retrocede guiándose con `cam_garra`, que mira hacia abajo entre los dedos.
4. Cierra, comprueba que lo tiene y lo lleva al punto de entrega; lo suelta y se aparta.

La lógica es una máquina de estados sin ROS (`ros/wallens_mision/scripts/mision_logica.py`) con sus tests; el nodo
`mision_frutas.py` solo traduce tópicos. Estados, tiempos y fallos de las pruebas reales:
[docs/mision-frutas-plan.md](docs/mision-frutas-plan.md).

```
mision_frutas ── camara ("frutas" | "garra" | "") ──> detector_frutas (abre una sola cámara)
      ▲  frutas/detecciones, garra/fruta  ◄───────────────┘
      ├── /odometry/filtered, brazo/estado, brazo/resultado, /battery_state, /parada
      ├─> /cmd_vel ─> freno_lidar ─> /cmd_vel_safe ─> wallens_driver ─> ruedas (IDs 2 y 4), /odom
      └─> brazo/orden ─> wallens_brazo ─> brazo/transaccion (driver) ─> pinza (ID 5)

/scan (LD19) ─> freno_lidar          /bno055/imu (wallens_imu) + /odom ─> EKF ─> /odometry/filtered
INA226 ─> wallens_power ─> /battery_state, /battery_critical ─> wallens_driver (frena)
```

Todo lo que se publica en `/cmd_vel` (misión, teleop, Nav2, scripts) pasa por el freno del LiDAR antes de llegar a las
ruedas. Arquitectura completa, arranque y marcos TF: [docs/sistema.md](docs/sistema.md).

## Hardware

| Pieza | Conexión | Notas |
|---|---|---|
| Raspberry Pi 4 (8 GB) | — | Ubuntu Server 24.04 arm64, ROS 2 Jazzy, hostname `wallens` |
| Ruedas: 2 Dynamixel XM430-W210-R (IDs 2 y 4) | USB-RS485 Peacefair (`/dev/dynamixel`) | driver C++ en tiempo real |
| Pinza: Dynamixel XM430-W210 (ID 5) | mismo bus | [docs/gripper-plan.md](docs/gripper-plan.md) |
| Tornillo sin fin (ID 3) | mismo bus | cambiado por uno reforzado el 2026-11-03; **cero cada vez que se prende**, ver [Tornillo](#tornillo-cada-vez-que-prendes-el-robot) y [docs/tornillo.md](docs/tornillo.md) |
| LiDAR LDRobot STL-19P (LD19) | USB-UART | `/scan`, alimenta el freno, [docs/lidar.md](docs/lidar.md) |
| IMU BNO055 | I²C por software | [docs/imu.md](docs/imu.md) |
| INA226 (batería LiPo 4S) | I²C | corte a 3.4 V/celda |
| `cam_frutas` (Logitech C270, al frente) y `cam_garra` (entre los dedos) | USB | [docs/camaras.md](docs/camaras.md) |

## El repo

| Carpeta | Qué hay |
|---|---|
| `ros/` | Paquetes ROS 2: `wallens_driver` (ruedas, odometría, bus del brazo), `wallens_imu`, `wallens_power` (batería), `wallens_safety` (freno LiDAR), `wallens_mision` (misión y detector de frutas), `wallens_nav` (SLAM + Nav2 + exploración), `wallens_interfaces` (mensajes del bus) |
| `launch/` | `robot.launch.py` (robot base) y `mision.launch.py` (robot + brazo + detector) |
| `scripts/` | Uso diario desde el PC (`mision.sh`, `probar.sh`, `desplegar.sh`, `pc-ros.sh`, `pi-ip.sh`, `cero-tornillo.sh`), instalación de la Pi (`00`…`05`), nodo del brazo (`brazo/`), visión (`vision/`) |
| `config/` | Red ROS (`ros-red.env`), reglas udev, EKF, versiones de la Pi buena (`versiones-pi.txt`) |
| `systemd/` | Servicios `wallens` y `wallens-mision` y la regla de sudo |
| `docker/` | Imagen de ROS 2 para el PC |
| `docs/` | Detalle de cada parte |
| `registros/` | Registros de las misiones traídos de la Pi (no se sube a git) |

## 1. El PC: ROS 2 en Docker

Hace falta Linux con Docker (el contenedor usa la red del host, `--net=host`). ROS 2 tiene que ser Jazzy, como en la Pi:
Humble y Jazzy no se entienden por DDS.

```bash
scripts/pc-ros.sh                       # abre una shell con ROS 2; si ya está abierto, entra en el mismo contenedor
scripts/pc-ros.sh ros2 topic list       # corre un comando y sale
scripts/pc-ros.sh rviz2                 # RViz (ventanas por X11/XWayland)
```

La primera vez `pc-ros.sh` construye sola la imagen `wallens-pc:jazzy` desde `docker/Dockerfile`. Tras cambiar el
Dockerfile: `RECONSTRUIR=1 scripts/pc-ros.sh`. A mano es lo mismo que:

```bash
docker build -t wallens-pc:jazzy docker/
```

La imagen parte de `ros:jazzy-ros-base` y trae RViz, `teleop_twist_keyboard`, `tf2_tools`, `rqt_graph`, `rqt_tf_tree`,
`slam_toolbox` y Nav2 (map server, bringup y plugins de RViz). El repo se monta en `/wallens` y el contenedor corre con
tu mismo uid, así que lo que se cree ahí queda a tu nombre.

Teleop (**publica `/cmd_vel`: mueve el robot**):

```bash
scripts/pc-ros.sh ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## 2. Conectarse a la Pi

La red `LABORATORIOS` bloquea mDNS (`wallens.local` no funciona) y la IP cambia. `scripts/pi-ip.sh` busca la Pi por su
MAC (`d8:3a:dd:46:9f:2b` wlan0, `d8:3a:dd:46:9f:27` eth0) en la tabla ARP, barre las subredes /24 si no está y recuerda
la última IP en `~/.cache/wallens/pi-ip`. `WALLENS_IP=<ip>` se salta la búsqueda (vale para todos los scripts).

```bash
ssh wallens@$(scripts/pi-ip.sh)
```

En cada shell de la Pi, para tener ROS, el workspace y la red del robot:

```bash
source ~/wallens/scripts/ros-env.sh
```

## 3. Red ROS 2

El robot usa **`ROS_DOMAIN_ID=37`** en la Pi (`config/ros-red.env`) y en el PC (`pc-ros.sh` lo pone por defecto). Otros
grupos del laboratorio usan el dominio 0: con un dominio propio no ven ni mandan nada al robot. Otro dominio en el PC:
`ROS_DOMAIN_ID=N scripts/pc-ros.sh` (tiene que coincidir con el de la Pi).

Además la Pi tiene `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`: solo descubre sus propios nodos y las IP de
`ROS_STATIC_PEERS`. Para que el PC vea los tópicos hay que configurar **los dos lados**:

```bash
# 1) En la Pi: la IP del PC (verla en el PC con `ip -4 addr`), fuera del repo porque cambia
mkdir -p ~/.config/wallens && echo "ROS_STATIC_PEERS=11.11.41.48" > ~/.config/wallens/peers.env
sudo systemctl restart wallens          # o wallens-mision, el que esté activo: leen peers.env al arrancar

# 2) En el PC: la IP de la Pi
ROS_STATIC_PEERS=$(scripts/pi-ip.sh) scripts/pc-ros.sh ros2 topic list
```

Los servicios y `ros-env.sh` leen `peers.env`; una shell de la Pi abierta antes del cambio necesita otro `source`.

**Si no aparecen los tópicos:**

- ¿Está corriendo un servicio en la Pi? `systemctl is-active wallens wallens-mision`
- ¿El dominio es 37 en los dos lados? En la Pi `echo $ROS_DOMAIN_ID` tras el `source`.
- ¿`peers.env` tiene la IP **actual** del PC y se reinició el servicio después?
- ¿El PC arrancó con `ROS_STATIC_PEERS=<ip de la Pi>`? Si el contenedor ya estaba abierto, `pc-ros.sh` entra en él con
  el entorno viejo: cerrarlo y abrirlo otra vez.
- El firewall del PC: con `ufw` activo el DDS entrante no llega. Dejar pasar solo a la Pi:
  `sudo ufw allow from <ip pi> proto udp to any port 16650:16899` (puertos del dominio 37).

**Ojo con otros contenedores:** cualquier contenedor con la red del host y el mismo dominio ve al robot y puede moverlo.
`pc-ros.sh` avisa al arrancar (`AVISO: el contenedor '…' usa la red del host y el dominio ROS 37 …`); pararlo con
`docker stop <nombre>` si no es tuyo.

## Tornillo: cada vez que prendes el robot

Al quedarse sin energía (apagar, cambiar la batería) el motor del tornillo **pierde la cuenta de vueltas**: no sabe a qué
altura está. Antes de usar el ascensor hay que darle el cero:

```bash
scripts/cero-tornillo.sh          # desde el PC; arranca wallens-mision si hace falta y pide Enter antes de mover
```

- **Qué hace:** abre la pinza, **baja** el tornillo despacio (~9 rpm, poca fuerza) hasta que se detiene contra el tope de
  abajo, se despega un poco y **calcula** el de arriba sumando el recorrido medido (36037 ticks útiles, 5 % antes del
  tope real). **El tope de arriba nunca se toca:** ahí el motor no frena y no se puede detectar por corriente.
- **Tarda** hasta ~65 s si el carro estaba arriba del todo.
- **Ctrl+C** mientras baja congela el brazo donde esté.
- **Vale mientras** no se corte la energía ni se reinicie `wallens-mision`: el cero vive en el nodo del brazo. Si se
  reinicia, se repite (la prueba de la caja también lo hace sola al empezar si el brazo no está calibrado).
- Antes de correrlo: nada debajo del gripper y mano en el interruptor.

**No usar `scripts/prueba-brazo.py --etapa=1` para esto:** su homing iba a ~20 rpm, el tope encajaba el tornillo y se
arrancaba con fuerza; así se rompió el anterior (2026-11-01). Ahora el homing va a ~9 rpm y un test exige que nunca encaje.

**Si se cambia o reajusta el mecanismo**, el recorrido se vuelve a medir a mano con `scripts/jog-tornillo.py` (paso 20,
tope de abajo por corriente, tope de arriba a ojo) y se anota en `scripts/calibracion-tornillo.json`. Detalle, números y
por qué no hay marca de "ya calibrado" en el motor: [docs/tornillo.md](docs/tornillo.md).

## 4. Correr la misión del limón

En la Pi corre siempre uno de dos servicios, nunca los dos (`Conflicts=`, comparten el bus de los motores):

- `wallens`: el robot base (driver, batería, IMU, LiDAR, EKF, freno). Arranca solo al encender.
- `wallens-mision`: el robot más el nodo del brazo y el detector (`mision.launch.py`). Lo arrancan los scripts de abajo.

No lanzar launch a mano con un servicio activo: duplica el driver en el bus.

### Rápido: `scripts/mision.sh X Y`

Desde la raíz del repo en el PC. Recoge el limón que está **a menos de 1 m al frente** del robot (entre 0.3 y 1.0 m,
hasta ±0.4 m de lado; la cámara no ve a menos de ~0.3 m) y lo deja en (X, Y), en metros respecto a donde está el robot
al lanzarla: **x al frente, y a la izquierda** (negativo = derecha).

```bash
scripts/mision.sh 0 0                              # lo deja donde arrancó el robot
scripts/mision.sh -0.75 0                          # 0.75 m detrás del inicio (dejar ~1 m libre detrás)
MAS="-p v_max:=0.15" scripts/mision.sh -0.75 0     # cualquier campo de Config (por defecto v_max 0.10 m/s, w_max 0.35 rad/s)
```

Valida el punto en el PC antes de tocar el robot, despliega solo si cambió algo y corre la misión **sin tests, sin
preflight y sin la espera de 5 s**.

### Por etapas: `scripts/probar.sh <etapa>`

Para cuando se cambia código: corre los tests, despliega, ejecuta la etapa en la Pi (`scripts/probar-mision.sh`:
arranca `wallens-mision`, pasa el preflight y lanza la misión) y trae el registro. De la más corta a la completa:

| Etapa | Dónde va el limón | Qué hace |
|---|---|---|
| `agarre` | 33 cm **detrás** del centro de las ruedas, en el eje | calibra si falta cero, baja, abre, retrocede, agarra, mide `x_agarre`/`y_agarre` abajo y sube antes de terminar |
| `acercar` | a menos de 1 m al frente | calibra si falta cero y cierra en vacío; busca con `cam_frutas`, se acerca y gira 180° |
| `hasta-agarrar` | a menos de 1 m al frente | como `acercar` y luego lo agarra |
| `completa` (por defecto) | a menos de 1 m al frente | lo agarra, lo deja 0.75 m detrás de donde arrancó y se aparta |

```bash
scripts/probar.sh agarre
scripts/probar.sh acercar
MAS="-p w_max:=0.45" scripts/probar.sh acercar
```

Si el preflight falla dice qué falta y **no mueve nada**. Una misión que agarra bien guarda la calibración del agarre
en la Pi (`~/.config/wallens/agarre.yaml`) y las siguientes la cargan solas; `X_AGARRE=… Y_AGARRE=…` la fuerzan.

**B4 (2026-11-03), pendiente de validar en el robot:** `usar_tornillo=true` por defecto en la misión. El homing
(`calibrar`) puede tardar ~65 s desde arriba y tiene un límite de 90 s; abre la pinza, así que se empieza sin fruta
agarrada. `subir` tarda ~25 s. Las ruedas esperan el resultado y el estado de brazo libre; un fallo deja el robot
quieto y no ordena soltar. `MAS="-p usar_tornillo:=false"` permite probar el comportamiento anterior sin ascensor.

La líder corre `scripts/probar.sh agarre` **5 veces, una por una**, con la mano en el interruptor y el limón a 33 cm
detrás del eje. Comprueba: calibra solo cuando falta cero, baja antes de abrir, agarra y sube; las ruedas quedan
quietas durante cada orden del brazo. Al finalizar, observa **30 s sosteniendo arriba**, sin sobrecarga ni caída.
Retira la fruta antes de repetir. `probar.sh` despliega el árbol completo: coordinar el árbol compartido antes de
ejecutarlo. Las alturas para baldosa quedaron medidas el 4 de noviembre; falta validar el contacto real.

### Misión automática: pedido, caja o baldosa

La integración del 3 de noviembre añade reconocimiento, mapa de objetos, pregunta del final, entrega en caja o
baldosa y vuelta a esperar otro pedido. La prueba física está pendiente: seguir primero
[las pruebas de mañana, en orden](docs/pruebas-manana.md) y la [actualización del 4 de noviembre](docs/resultados-20261104.md).
La pinza nueva agarra a 3200 ticks y sostiene las tres frutas con 200. Su posición **no distingue vacío de fruta**:
el cierre devuelve `por_verificar`; falta medir la región visual de la fruta subida antes de autorizar transporte.

En la Pi, con `wallens-mision` disponible y las medidas configuradas:

```bash
source ~/wallens/scripts/ros-env.sh
ros2 run wallens_mision mision_frutas.py --ros-args --params-file ~/wallens/config/mision_automatica.yaml
```

Espera `/mision/pedido` (voz o publicación manual); después de recoger y subir pregunta en `/mision/estado`:

```bash
ros2 topic pub /mision/pedido std_msgs/msg/String '{data: limon}' -r 5 --times 10
# Solo después de ver «¿caja o baldosa?»:
ros2 topic pub /mision/final std_msgs/msg/String '{data: caja}' -r 5 --times 10
```

`config/frutas.yaml` contiene referencias y medidas del detector; `config/mision_automatica.yaml`, las de la misión.
Marco 0.072 m, diámetros 0.050/0.070/0.074 m, alturas por fruta y frente de caja 3.3 cm ya están configurados.
Las cajas turquesa se localizan con cámara y láser; se visitan para leer la tarjeta a 0.7 m, con un reintento a 0.6 m.
`agarre_arriba_x/y` siguen **POR MEDIR**: el cierre incierto se sostiene quieto y no autoriza el traslado.
`IR_A_ENTREGA/DEJAR` permanece como plan B, con `esperar_pedido=false` y `automatico=false`.
La navegación no integra Nav2. La cámara sigue siendo propiedad exclusiva de `detector_frutas`, también al grabar
fotos con `scripts/vision/camara.py grabar`.

### Parar y volver al robot normal

- **Ctrl+C** frena (ruedas a 0) y deja la pinza cerrada si no lleva fruta; un segundo Ctrl+C sale sin esperar. El
  registro se trae igual.
- Al terminar, en la Pi: `sudo systemctl start wallens` (o reiniciar la Pi) vuelve al robot base. Al apagarse el torque
  la pinza suelta la fruta.

### Registros

- En el PC: `registros/mision-<fecha>-<etapa>/` con `mision.log`, `preflight.txt` y `cuadros/` (último cuadro anotado
  de cada cámara).
- En la Pi: `~/.cache/wallens/mision-*` y el log del servicio: `journalctl -u wallens-mision` (y `journalctl -u wallens -f`
  para el robot base).

## 5. Instalar la Pi desde cero

En orden; todos son idempotentes. Los scripts se corren en la Pi desde `~/wallens`.

1. Flashear la SD con Raspberry Pi Imager: *Ubuntu Server 24.04 LTS (64-bit)*; en ajustes avanzados poner hostname
   `wallens`, usuario, clave SSH y Wi-Fi.
2. Copiar el repo desde el PC: `scripts/desplegar.sh` (rsync a `~/wallens`). La primera vez su paso de compilación falla
   porque aún no existe `~/ros_ws`; el código ya quedó copiado.
3. `sudo scripts/00-base.sh <usuario> [otros usuarios]`: hostname, repos `noble-updates`, paquetes base, journal en RAM
   (protege la SD), usuarios en los grupos `dialout`, `video` e `i2c`. Reiniciar.
4. `sudo scripts/01-ros2.sh`: ROS 2 Jazzy (`ros-base`, sin GUI), `v4l2_camera` y OpenCV.
5. `sudo scripts/02-udev.sh`: nombres fijos `/dev/lidar`, `/dev/cam_frutas`, `/dev/cam_garra`, `/dev/dynamixel`.
6. `sudo scripts/03-i2c.sh` y reiniciar: bus I²C por software (el hardware falla con el BNO055, ver
   [docs/imu.md](docs/imu.md)). `scripts/03-i2c.sh --diff` muestra el cambio sin escribir.
7. `sudo scripts/05-ros-ws.sh deps` (dependencias apt: Dynamixel SDK, EKF, SLAM, Nav2) y luego `scripts/05-ros-ws.sh`
   sin sudo: clona el driver del LiDAR y `explore_lite` en commits fijos, enlaza `ros/` y compila `~/ros_ws` en Release.
   `scripts/05-ros-ws.sh versiones` imprime lo instalado para comparar con `config/versiones-pi.txt`.
8. `sudo scripts/04-servicio.sh`: instala `wallens` (activo y al arranque), `wallens-mision` y la regla de sudo que deja
   a los scripts arrancarlos sin contraseña. `--quitar` lo deshace.

### Desplegar cambios

```bash
scripts/desplegar.sh              # tests → rsync de lo que cambió → compila los paquetes tocados → md5 → reinicia el servicio activo
scripts/desplegar.sh --sin-tests
```

Si cambia algo de `systemd/`, en la Pi hay que correr otra vez `sudo ~/wallens/scripts/04-servicio.sh`.

## 6. Tests

Ninguno necesita ROS ni el robot. Los dos primeros los corre `desplegar.sh` antes de copiar nada:

```bash
(cd ros/wallens_mision && python3 -m unittest discover -s test)                          # misión, con mundo falso
(cd scripts && python3 -m unittest brazo.test_brazo brazo.test_control brazo.test_jog)   # brazo, con bus falso
(cd scripts && uv run --no-project --with opencv-python-headless --with numpy python -m unittest vision.test_camara)
uv run --no-project --with pytest python -m pytest ros/wallens_safety/test ros/wallens_nav/test   # freno y supervisor
```

`vision.test_camara` necesita OpenCV; con OpenCV instalado basta `python3 -m unittest vision.test_camara` desde
`scripts/`. Los dos últimos, con `pytest` instalado: `python3 -m pytest …`.

## 7. Seguridad

- **No hay parada de emergencia física** (riesgo aceptado). Durante las pruebas: espacio despejado, una persona junto al
  interruptor de la batería.
- `/parada` manda sobre todo y el driver la recuerda hasta recibir `false`:
  ```bash
  ros2 topic pub --once --qos-durability transient_local /parada std_msgs/msg/Bool "{data: true}"    # parar
  ros2 topic pub --once --qos-durability transient_local /parada std_msgs/msg/Bool "{data: false}"   # reanudar
  ```
  No sustituye a una seta: si falla el software o la red, no llega.
- El freno LiDAR anula el avance o el giro si la huella del robot (con la pinza) va a tocar algo, y frena mientras el
  brazo se mueve. El LiDAR está a 16 cm del suelo: **no ve objetos más bajos, ni la fruta**.
- `wallens_power` corta a 3.4 V/celda (`/battery_critical`: el driver frena); la misión necesita ≥ 14.6 V para empezar.
- El driver frena solo si deja de recibir `/cmd_vel_safe` 0.25 s.
- Teleop publica `/cmd_vel` y **mueve el robot**.

Detalle: [docs/sistema.md](docs/sistema.md). Mapeo autónomo (SLAM + Nav2): [docs/navegacion.md](docs/navegacion.md).
Comunicación con la Pi, en una página: [docs/comunicacion-pi4.html](docs/comunicacion-pi4.html).

## Pendiente

IP fija (reserva DHCP por MAC con TI), imagen de respaldo de la SD, envío de imagen, reimprimir las llantas.

Los scripts de diagnóstico que ya cumplieron se borraron el 2026-11-02; se recuperan con
`git log --diff-filter=D --name-only` y `git show <commit>^:<ruta>`.
