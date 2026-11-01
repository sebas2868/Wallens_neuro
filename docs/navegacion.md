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

## Pendiente

Repetir `giro90`, `giro180`, `ida-vuelta` y `cuadrado` con la batería cargada y medir con cinta; 5 corridas de repetibilidad;
luego SLAM y Nav2 (posición del LiDAR ya medida, ver [sistema.md](sistema.md)).
