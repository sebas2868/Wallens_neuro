# Pruebas de navegación

Rutas cerradas con el EKF (`/odometry/filtered`) que publican a `/cmd_vel` (pasan por el freno LiDAR):
`python3 scripts/odometria/rutas.py <ruta> [corridas]` con ROS cargado y el servicio en marcha. MUEVE el robot: espacio libre
y mano cerca. Rutas: `avance` (1 m adelante y atrás), `ida-vuelta` (2 m, giro 180°, 2 m, giro 180°), `cuadrado` (4 x [1 m + 90°]),
`giro90` (4 giros de 90°), `giro180` (2 giros de 180°). Todas terminan donde empiezan, así que el error se mide con cinta
contra una marca en el suelo. Criterio: menos de 3 cm y menos de 2°, 5 corridas sin corregir entre ellas.
`ESCALA` en el script multiplica la velocidad base (0.10 m/s y 0.3 rad/s) y los anticipos de frenado; hoy 1.2.

Otras: `giro-ekf.py` (360° cerrado con el EKF), `prueba-freno.py` (comprueba el freno con un obstáculo a menos de 30 cm).

## Resultados (2026-11-01)

| Prueba | Resultado según el EKF | Medida real |
|---|---|---|
| Giro 360° con el EKF, 0.3 rad/s | EKF 360.5°, IMU 360.4°, ruedas 360.5° | sin medir en el suelo |
| `avance` a 1.0x | -0.2 cm adelante, -0.2 cm de lado, -0.1° | sin medir |
| `avance` a 1.2x | -0.1 cm adelante, -0.3 cm de lado, -0.1° | sin medir |
| Freno con caja a 27 cm | 0.0 cm de movimiento, `/cmd_vel_safe` siempre en 0 | |
| `giro90`, `giro180` a 1.2x | abortaron por tiempo: la batería bajó de 13.6 V y `battery_critical` frenó el driver | |

## Mapeo autónomo (SLAM + Nav2 + exploración)

Paquete `ros/wallens_nav`. Criterio de diseño: poca CPU y poca energía (LiPo 4S); la velocidad importa poco.

```
/scan ──> slam_toolbox (async) ──> /map, TF map -> odom
             │                        │
             │                  explore_lite ── meta (frontera) ──> bt_navigator (explorar.xml)
             │                                                      ├─ planner_server (NavFn, a demanda)
/odometry/filtered (EKF) ─────────────────────────────────────────> ├─ controller_server (RPP, 10 Hz) ──> /cmd_vel
                                                                    └─ behavior_server (backup, wait)
/cmd_vel ──> freno_lidar (huella) ──> /cmd_vel_safe ──> wallens_driver          supervisor_mision: fin o fallo
```

| Pieza | Elección | Por qué |
|---|---|---|
| SLAM | `slam_toolbox` online asíncrono, 5 cm, 1 de cada 2 scans, solo tras 30 cm o 20°, `/map` cada 5 s, alcance 8 m, cierre de lazo con cadenas de 15 y búsqueda de 2 m | El LD19 va a 10 Hz fijo: se descartan scans en vez de bajar el sensor. A 0.10 m/s procesa ~1 scan cada 3 s en recta. Asíncrono: si va atrasado, tira scans en vez de encolarlos |
| Localización | Sin AMCL: la da el propio SLAM (TF `map -> odom`) | Un nodo menos |
| Controlador | Regulated Pure Pursuit a 10 Hz, 0.10 m/s, sin marcha atrás, giro en el sitio solo si el camino sale a más de 60°, sin giro final (`yaw_goal_tolerance` 6.28) | Cuenta geométrica barata (MPPI o DWB muestrean cientos de trayectorias). Los giros en el sitio patinan y gastan. 10 Hz da 2.5 veces margen sobre el `cmd_timeout` de 0.25 s |
| Planificador | NavFn (A*) y árbol `explorar.xml`: replanifica solo si el camino deja de ser válido (revisa a 1 Hz) | No planifica a ritmo fijo |
| Recuperaciones | Limpiar costmaps, esperar 3 s, retroceder 15 cm a 0.05 m/s; 3 reintentos; sin `Spin` | El giro de recuperación patina y gasta; si no hay camino, explore_lite descarta la frontera |
| Costmaps | Local 2 x 2 m a 4 Hz (publica a 1 Hz); global a 1 Hz (publica a 0.2 Hz), capa estática del `/map` + obstáculos del scan | Pequeños y lentos |
| Huella | Chasis redondo de 296 mm + gripper detrás (133 x 30 mm, se usan 50 de ancho): polígono de `huella_wallens()`; inflación 0.35 m | La misma en el freno y en Nav2; el gripper llega a 28.1 cm del centro y barre al girar |
| Rampa | La del motor (`profile_acceleration`), sin `velocity_smoother` | Un nodo menos; aceleraciones suaves ya en el hardware |
| Seguridad | Nav2 publica `/cmd_vel`, que pasa por `freno_lidar`; sin `collision_monitor` | El freno ya filtra y frena el giro por la huella |
| Exploración | `explore_lite` (m-explore-ros2, commit fijo): fronteras cada 5 s, peso alto a la distancia (fronteras cercanas), sin volver al inicio, descarta una frontera tras 30 s sin acercarse | Trayectorias cortas; al acabar se queda donde está |
| Fin | `supervisor_mision`: termina con exploración completa, batería < 14.6 V de media en 10 s o 20 min; aborta (y publica `/parada`) sin avanzar 20 cm en 2 min, frenado por obstáculo 20 s o sin batería/odometría 3 s. Guarda el mapa en `~/mapas` y sale: el launch se cierra entero | Nunca reintenta: el robot queda quieto y la CPU vuelve al reposo |
| CPU | Núcleo 2 solo para el driver (tiempo real, `chrt -f 50`), 3 para el IMU; el servicio y el mapeo en los núcleos 0-1, el mapeo con `nice 5` | Con SLAM + Nav2 el bucle del driver llegó a atrasarse 350 ms y el watchdog del bus lo apagó (2026-11-01). Mejor un mapa lento que unas ruedas sin control |
| Apagado | Sin RViz en la Pi, sin cámaras, sin `ros2 bag` salvo que se pida | |

Obstáculos bajos: el LiDAR (a 16 cm del suelo) no ve nada más bajo. **Riesgo aceptado** (2026-11-01): mapear solo en un
espacio sin objetos sueltos de menos de 16 cm (o cerrarlo con cajas altas). Un sensor frontal queda para más adelante.

### Cómo lanzar, guardar el mapa y parar

En la Pi, con el servicio `wallens` en marcha y la batería en reposo por encima de ~15.2 V:

```bash
source ~/wallens/scripts/ros-env.sh
python3 ~/wallens/scripts/medicion/medir.py --etiqueta mapeo &        # opcional: CPU y Wh del recorrido (Ctrl+C al final)
ros2 launch wallens_nav mapeo.launch.py                               # MUEVE el robot: explora solo
#   explorar:=false               solo SLAM + Nav2 (destinos a mano desde RViz, o mapear con teleop)
#   resolucion:=0.10 costmap_hz:=2.0 throttle_scans:=3      perfil de menos CPU
```

Lanzarlo dentro de `tmux` o `screen`: si se cae el SSH, el launch muere y el robot se para (sin `/cmd_vel` el driver frena
en 0.25 s), pero se pierde el mapa sin guardar.

- **Guardar el mapa a mano** (el supervisor lo guarda solo al terminar):
  `ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/home/wallens/mapas/sala'}}"`
  (deja `sala.pgm` y `sala.yaml`, formato de `map_server`).
- **Parar la exploración** dejando SLAM y Nav2: `ros2 topic pub --once /explore/resume std_msgs/msg/Bool "{data: false}"`.
- **Parar de forma segura:** `/parada` (manda sobre todo, el driver la recuerda hasta recibir `false`):
  `ros2 topic pub --once --qos-durability transient_local /parada std_msgs/msg/Bool "{data: true}"`; después Ctrl+C
  en el launch. Si algo falla, cortar la batería.
- Tras un aborto del supervisor queda `/parada` en true: revisar el motivo en la salida del launch y publicar `false`.

## Pendiente

Repetir `giro90`, `giro180`, `ida-vuelta` y `cuadrado` con la batería cargada y medir con cinta; 5 corridas de repetibilidad
(ahora con el EKF sin yaw absoluto del IMU). Después, las pruebas del mapeo en el orden de la sección siguiente.
