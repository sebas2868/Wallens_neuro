# Plan: misión de recoger una fruta y llevarla a un punto

Propuesto el 2026-11-02, revisado el mismo día con las respuestas del usuario y rehecho tras la primera prueba real. Contexto: [sistema.md](sistema.md), [navegacion.md](navegacion.md), [gripper-plan.md](gripper-plan.md),
[brazo-bus-plan.md](brazo-bus-plan.md), [camaras.md](camaras.md), [tornillo.md](tornillo.md).

## La misión

Se le dice al robot dónde buscar y en qué orden recoger: por ejemplo "las frutas están en x = 1.00 m, y = 1.00 m, en un
radio de 10 cm; primero el limón, luego la naranja, luego la mandarina". Zona, radio, orden y punto de entrega son
**parámetros de cada corrida** (`config/mision.yaml` o argumentos), no valores fijos; coordenadas en el marco `odom`,
origen = donde arrancó. Las frutas están en el suelo. Para cada fruta de la lista:

1. Va hacia esa zona y se detiene antes, donde `cam_frutas` (Logitech C270, al frente) la vea entera.
2. Busca y clasifica la fruta (limón, naranja o mandarina) por **color y forma**.
3. Se acerca a ella y se detiene a una distancia que le deje girar 180° sin tocarla.
4. Gira 180°: el gripper está **detrás** (dedos hasta x = -0.285 m), así que queda mirando a la fruta.
5. Baja el ascensor (tornillo, ID 3) y abre el gripper (ID 5).
6. Con `cam_garra` (apunta a la zona que ocupa una fruta con el gripper cerrado) retrocede y corrige el rumbo hasta que
   la fruta queda en esa zona.
7. Cierra, comprueba con `cam_garra` que la tiene y sube el ascensor.
8. Va al punto de entrega (un punto que se le dé o *home*, el de arranque), baja, suelta y sube.
9. Vuelve a la zona por la siguiente fruta. Las entregadas quedan **una al lado de la otra en línea recta**: la fruta
   `i` se deja en `entrega + i · separación` a lo largo de la dirección de la línea.

Decisiones:
- **Sin Nav2 al principio.** Los traslados (ir a la zona, ir a la entrega) son rectas cerradas con el EKF, como
  `scripts/odometria/rutas.py`: girar hacia el destino y avanzar. El espacio tiene que estar despejado. Nav2 entra después
  **solo** en esos dos estados (ver [Nav2 después](#nav2-después)).
- **Coordenadas en `odom`** con origen en el arranque. Con la deriva medida (1 m exacto, 360° con 0.5°) alcanza para
  recorridos de pocos metros. Con mapa y Nav2 pasan a `map`.
- **Clasificación por color y forma con OpenCV**, sin modelo: barato en la Pi 4.
- El tornillo está roto hoy y el gripper quedó abajo, al ras del suelo, igual que las frutas: **modo sin ascensor**
  (bajar y subir se omiten) y se agarra directo del suelo, sin soporte. Riesgo: la fruta viaja casi tocando el suelo
  (el modelo da ~5 mm de holgura); se acepta para las pruebas, pero el suelo tiene que ser liso y sin cables.
- **Orden de recogida:** primero la que se pide, en el orden de la lista. Si en `BUSCAR` no aparece ese tipo, la misión
  falla (no cambia de fruta por su cuenta) y se registra qué tipos sí vio.
- **Línea de entrega:** la separación tiene que dejar pasar los dedos abiertos (±136 mm) sin tocar la fruta anterior:
  separación ≥ 136 mm + radio de la fruta + margen ≈ **0.25 m**. El robot llega a cada punto de frente a la línea
  (perpendicular a ella) para no pasar por encima de las ya entregadas, que el LiDAR no ve.

## Máquina de estados

`ros/wallens_mision/scripts/mision_logica.py` (sin ROS). Rehecha el 2026-11-02 tras la primera prueba real, más corta:
un estado hace una sola cosa y tiene un tiempo máximo. La fruta elegida vive en `odom` (`objetivo`): `cam_frutas` la
sitúa desde lejos, `cam_garra` la corrige de cerca y entre cuadros la lleva la odometría, así que un cuadro perdido no
para nada. **Una cámara a la vez**: la misión publica en `camara` cuál abre `detector_frutas` (ninguna en los traslados).

```
INICIO ─> IR_A_ZONA ─> BUSCAR ─> ACERCAR ─> GIRAR ─> ABRIR ─> RETROCEDER ─> CERRAR ─> VERIFICAR ─> SALIR
                         ▲                                        │            │          │
                         └────────────── REINTENTO <──────────────┴────────────┴──────────┘  (2 veces; luego FALLO)
SALIR ─> IR_A_ENTREGA ─> DEJAR ─> SOLTAR ─> ALEJAR ─> CERRAR_VACIO ─> SIGUIENTE ─(quedan)─> IR_A_ZONA
                                                                               └─(lista vacía)─> FIN (o VOLVER_HOME)
```

Cualquier estado puede ir a `FALLO`: robot quieto, pide `/parada` y, si lleva fruta, la sostiene. `/parada`, la batería
baja, el brazo congelado y la falta de datos (odometría 0.5 s, brazo 1 s, batería 3 s, la cámara pedida 5 s desde que
se pidió: abrirla tarda 3.4 s `cam_frutas` y 1.4 s `cam_garra`, medido en la Pi) mandan sobre todo.

| Estado | Cámara | Qué hace | Sale cuando | Máx. |
|---|---|---|---|---|
| `INICIO` | la del primer estado | Espera odometría, brazo, batería y el detector, **diciendo qué falta**; batería ≥ 14.6 V; brazo congelado sin fruta → orden `rearmar` (queda en el log); congelado con fruta → fallo | todo bien | 60 s |
| `IR_A_ZONA` | frutas | Traslado (`_ir_a`) a 0.8 m de la zona rodeando las entregadas; gira hacia ella | llega | 60 s |
| `BUSCAR` | frutas | Mira (0.6 s de latencia, mínimo 1 s); barre 0°, ±20°, ±40°. Registra todas las frutas de la zona y elige una dirección de aproximación que no toque a las vecinas | 3 detecciones coherentes de la pedida | 40 s |
| `ACERCAR` | frutas | Si hace falta rodea la fruta a 0.7 m en arcos de ≤ 45°; luego de frente, corrigiendo con la cámara | a 0.40 ± 0.02 m | 90 s |
| `GIRAR` | garra | 180° en el sitio: el gripper queda mirando a la fruta | ± 2° | 20 s |
| `ABRIR` | garra | orden `abrir` | el brazo responde | 20 s |
| `RETROCEDER` | garra | Marcha atrás ≤ 0.03 m/s hacia la estimación, corrigiendo el rumbo. Cada cuadro de `cam_garra` a < 10 cm de la estimación (5 cm ya confirmada) la corrige a medias; los demás se ignoran. Si la estimación está dentro del campo de `cam_garra` y no la ve en 0.6 s: **para** y espera 1.5 s; si sigue sin verla y ya estaba confirmada, sigue con la odometría ≤ 6 cm (pasa bajo la cámara); si no, `REINTENTO` | estimación en la zona de agarre (± 1.5 cm, ± 1.2 cm) con ≥ 2 confirmaciones | 40 s, ≤ 0.30 m |
| `CERRAR` | garra | 0.5 s quieto (el brazo exige ruedas quietas) y `cerrar:<tipo>` | `agarrado`; `sin_agarre` → `REINTENTO` | 20 s |
| `VERIFICAR` | garra | 1.5 s mirando la fruta agarrada: **registra dónde la ve** (`agarre_visto`, sale en el log) | el brazo la sostiene | 10 s |
| `REINTENTO` | frutas | `abrir`, avanza 0.30 m, gira hacia la fruta y vuelve a `BUSCAR` alrededor de la última estimación (radio 15 cm) | | 40 s |
| `SALIR` | — | Avanza 0.3 m en recta (girar ahí barrería a las vecinas) | llega | 15 s |
| `IR_A_ENTREGA` | — | Traslado al punto previo, a 0.5 m de la línea por el lado de la zona, rodeando frutas | llega | 90 s |
| `DEJAR` | — | Gira con el frente lejos de la línea y marcha atrás hasta que la zona de agarre queda sobre su punto | llega | 40 s |
| `SOLTAR` / `ALEJAR` / `CERRAR_VACIO` | — | `soltar`; avanza 0.3 m con la pinza abierta; `cerrar` (si dice `agarrado`, fallo) | | 20/15/20 s |
| `VOLVER_HOME` | — | Si se pidió: traslado a (0, 0) | llega | 90 s |

Cambios frente a la primera versión: sin estados del ascensor (el tornillo está roto; cuando vuelva, `bajar`/`subir`
entran como órdenes dentro de `ABRIR` y `VERIFICAR`), `RODEAR` va dentro de `ACERCAR` y `ORIENTAR` dentro de `DEJAR`,
`ALINEAR` pasa a ser `RETROCEDER` y el reintento ya no es "avanzar 5 cm y repetir" sino volver a buscar con `cam_frutas`.

### Fallos de la primera prueba (2026-11-02) y qué se cambió

`desde=ABRIR hasta=VERIFICAR` con el limón detrás del robot. Reproducidos en el mundo falso antes de arreglarlos
(`test_mision.FallosReales`, `brazo.test_control.TestReposo`).

1. **"cam_garra perdió la fruta más de 1-2 s".** Causa: la "lima" es verde oscura, **V de 17 a 62** en `cam_garra`, y el
   detector exigía `v_min = 50` (y `s_min = 70`). Grabando 20 s con la fruta quieta (`scripts/vision/camara.py`): con los
   umbrales viejos la veía en **2 de 300 cuadros** (una vez como naranja); con `s_min = 60, v_min = 20`, en **298 de 300**,
   siempre limón. El suelo (S ≤ 30) y el dedo negro (H 113-144) siguen fuera. Además la misión ya no depende de cada
   cuadro: estimación en odom, puerta de distancia, parar y esperar, y `REINTENTO` en vez de fallar.
2. **"INICIO: el brazo está congelado".** Causa: un bug del brazo, no de la misión. `ocupado` era "el cerrojo está
   tomado", y `mantener()` lo toma 10 veces por segundo para vigilar. El `/parada` del fallo anterior llegó con el brazo
   quieto, se tomó como "ocupado", pidió un aborto suave y la vigilancia siguiente congeló el brazo. El mismo bug hacía
   que `brazo/ocupado` saliera true a ratos en reposo, y el freno LiDAR **frenaba las ruedas a trozos durante ALINEAR**.
   Ahora `ocupado` = "ejecutando una orden"; en reposo una petición suave se descarta. Política: un fallo deja todo
   quieto (y `/parada` true); una misión nueva publica `/parada` false y, si el brazo quedó congelado **sin fruta**, lo
   rearma con la orden `rearmar` (se registra en el log del brazo y de la misión). Con fruta, no: lo decide una persona.
3. **La misión fallaba por tiempo mientras los nodos subían.** `INICIO` espera 60 s y dice qué falta (odometría, brazo,
   batería, el detector con su cámara); si se agota, el fallo lista lo que no llegó.
4. **Cierre de los dedos.** Verificado en el código: la misión ya no cierra los dedos para medir (en la prueba barrieron
   el limón: `cam_garra` lo vio pasar de x = -0.266 a -0.139 mientras se cerraban); `abrir` usa `CIERRE_MEDIDO = 3890`
   (medido 3890, 3891, 3891 y 3897 en la prueba).
5. **Sobrecarga del gripper** (registro 70 = 32) tras sostener la fruta. Tras agarrar, `cerrar:<tipo>` baja la corriente
   a la mínima que la sostuvo firme en la fase G (`SOSTEN_FRUTA`: limón 200, naranja 300). `preparar()` ya reiniciaba el
   gripper si quedaba con el error.
6. **Geometría del agarre:** ver abajo.
7. **CPU.** Antes corrían los dos detectores todo el tiempo (~19 % de un núcleo cada uno). Ahora hay uno, con una sola
   cámara y solo en los estados que la usan: 0.4 % sin cámara, 15 % con `cam_garra`, 11 % con `cam_frutas` (OpenCV con
   1 hilo: su pool repartía cada operación en 4 núcleos y esperaba girando). El nodo del brazo vigila en reposo a 4 Hz
   (antes 10 Hz) con el watchdog del motor a 1 s (antes 0.5 s) y 4 hilos (antes 6). La misión va a 10 Hz (antes 20).
   **Sin medir en la Pi** para el brazo y la misión: los motores no respondían (12 V apagados) y sin driver no arrancan.

### Geometría del agarre (qué medir y cómo)

Hay tres números que tienen que cuadrar y hoy no cuadran: la punta de los dedos según la huella (`DEDOS_X[0] = -0.285`),
lo que dice `cam_garra` (calibración afín de la fase 0, ±0.6 cm, hecha con la fruta en el suelo) y lo que vio la persona
(limón "3-4 cm detrás de las puntas", que la huella pone en x ≈ -0.32 a -0.36 según se mida al borde o al centro).
`cam_garra` lo vio en **x = -0.265**, y su campo medido termina en -0.32: a -0.35 no lo habría visto. O la huella
tiene la punta más atrás de lo real o la medida se tomó con otra referencia. El centro de la imagen de `cam_garra` cae
en x = -0.229, que es el `x_agarre` = -0.23 de partida.

Lo que importa para agarrar es **dónde ve `cam_garra` la fruta cuando está bien agarrada**, en su propio marco: la misión
retrocede hasta que la estimación (corregida por `cam_garra`) llega ahí, así que un sesgo de la calibración se cancela.
Cómo fijarlo:
1. Correr la prueba hasta `VERIFICAR` (`probar-mision.sh VERIFICAR`). Al final el log dice
   `fruta agarrada vista por cam_garra en x=… y=…: usar como -p x_agarre:=… -p y_agarre:=…`. Si difiere más de 1 cm de lo
   configurado, pasarlo a `Config` (y a `probar-mision.sh`).
2. Con cinta, con el gripper cerrado: x de las puntas de los dedos desde el eje de las ruedas → `DEDOS_X[0]` (huella:
   `scripts/modelo/huella_modelo.py`, `mision_logica.py` y la del freno).
3. Comprobar la afín: limón con el centro en x = -0.25 y -0.30 (y = 0 y ±4 cm) medidos con cinta; el preflight o
   `ros2 topic echo /garra/fruta` dan lo que ve. Si el error pasa de 1.5 cm, repetir la fase 0 de `cam_garra`.

La huella y los márgenes con las vecinas usan `x_agarre` para la fruta llevada: 2-4 cm de diferencia entre marcos solo
cambian esos márgenes (2 cm).

### Pruebas

`cd ros/wallens_mision && python3 -m unittest discover -s test -v` (48 tests, sin ROS, ~0.3 s) y
`cd scripts && python3 -m unittest brazo.test_brazo brazo.test_control brazo.test_jog` (112). El mundo falso
(`test/mundo_falso.py`) integra la cinemática, simula `cam_frutas` (±22°, de 0.30 a 2 m, ruido, tipos que cambian),
`cam_garra` (cuadros perdidos, banda ciega bajo la cámara, sesgo de la calibración), el brazo (congelado, `rearmar`) y
la batería (también tardía), y registra cualquier contacto del chasis, el carro o los dedos con una fruta. 300 misiones
al azar (1 a 3 frutas a 17-24 cm, ruido de 0.5 a 2 cm, hasta 50 % de cuadros perdidos, banda ciega y sesgo de -2 a +3 cm
en `cam_garra`): **0 choques**; 272 de 278 pedidos válidos terminan (14 tras un `REINTENTO`), los otros 6 se rechazan
limpios porque las frutas están demasiado juntas.

**Probar por partes:** `Mision(pedido, desde="ESTADO", hasta="ESTADO")`; en el nodo `-p desde:=… -p hasta:=…`. `ACERCAR` y
`GIRAR` buscan la fruta antes con un `BUSCAR`; los estados de entrega exigen que el brazo ya la lleve.

### Probar en el robot

Por etapas, de la más corta a la completa; cada una prueba una pieza nueva y el launch queda corriendo entre ellas:

```
~/wallens/scripts/probar-mision.sh agarre          # limón 33 cm detrás, en el eje: cam_garra + gripper (~20 s)
X_AGARRE=… Y_AGARRE=… ~/wallens/scripts/probar-mision.sh agarre   # repetir con lo que imprimió VERIFICAR
~/wallens/scripts/probar-mision.sh acercar         # limón 1.2 m al frente: cam_frutas, acercarse y girar 180°
~/wallens/scripts/probar-mision.sh hasta-agarrar   # lo anterior y agarrarlo
~/wallens/scripts/probar-mision.sh                 # completa: lo deja donde arrancó el robot y se aparta
```

Para el servicio `wallens` (pide sudo), lanza `mision.launch.py` con `depuracion:=true` si no corre, pasa el
**preflight** (`scripts/preflight.py`: servicio, una copia de cada nodo, frecuencias, brazo, error de hardware del
gripper, cada cámara 4 s, batería, CPU) y solo entonces lanza la misión. Todo queda en `~/.cache/wallens/mision-<fecha>/`
y el último cuadro de cada cámara, con las manchas, en `~/.cache/wallens/detector/`. El pedido es **relativo al robot**
al lanzar la misión (`relativo:=true`), así que no importa desde cuándo corre el launch.

Desde el PC: `scripts/desplegar.sh` (tests → rsync de lo que cambió → compila solo los paquetes tocados → md5 de fuente
e instalado → dice qué relanzar). Umbrales de color: `scripts/vision/camara.py grabar|evaluar` (graba 20 s y corre el
mismo filtro que el detector sobre los cuadros, con los umbrales que se le den).

### Órdenes del nodo del brazo

`abrir`, `soltar`, `rearmar` (sale del congelado solo sin fruta), `cerrar` (vacío) y `cerrar:<tipo>` (corriente de esa
fruta y luego la de sostén); `bajar`/`subir` responden "sin ascensor". Cada una responde en `brazo/resultado`
`(ok, texto)`; `cerrar*` responde `agarrado` o `sin_agarre`. El brazo solo recibe órdenes con las ruedas quietas (el
mundo falso lo comprueba) y `brazo/ocupado` (lo que frena las ruedas) es true solo mientras ejecuta una.

### Nav2 después

Los traslados (`IR_A_ZONA`, `IR_A_ENTREGA`, `VOLVER_HOME`) pasan todos por `Mision._ir_a(destino, frutas)`. Hoy encola
rectas que rodean las frutas conocidas; con Nav2 ese método manda la meta y el estado espera el resultado. Las frutas
quedan por debajo del LiDAR: habrá que pasarlas a Nav2 como obstáculos (capa de costos) con el mismo radio.

## Clasificación por color y forma

| Fruta | Color (HSV) | Forma | Tamaño típico |
|---|---|---|---|
| Limón | amarillo-verde | alargada: elipse con eje mayor/menor ~1.3–1.5, puntas | 5–6 × 7–8 cm |
| Naranja | naranja | redonda: relación de ejes ~1.0–1.1 | 7–9 cm |
| Mandarina | naranja (a veces más rojiza) | algo achatada: relación ~1.1–1.2 vista de lado | 5–7 cm |

- Color separa el limón de las otras dos. Naranja vs. mandarina sale sobre todo del **tamaño real**, que se calcula con
  la distancia (proyección del punto inferior del contorno al suelo con la altura e inclinación de la cámara) y el ancho en
  píxeles. La forma (ajuste de elipse, circularidad) desempata.
- Rangos HSV y umbrales en un YAML (`config/frutas.yaml`), medidos con fotos de las frutas reales en el lugar real.
  Herramienta: `scripts/vision/calibrar-colores.py` (deslizadores sobre una foto, guarda el YAML).
- Salida de `detector_frutas`: lista de frutas con tipo, confianza, posición en `base_link` y diámetro estimado.
- 640×480 MJPG procesado a 320×240, 5 Hz, núcleos 0-1. Las dos cámaras **por turnos**, nunca a la vez.

## Arquitectura

```
                    camara ("frutas" | "garra" | "")
mision_frutas ─────────────────────────────────────> detector_frutas (abre una sola cámara)
      ▲   frutas/detecciones (tipo, x, y en base_link) / garra/fruta (visible, x, y) ┘
      ├── /odometry/filtered, brazo/estado, brazo/resultado, /battery_state, /parada
      ├─> /cmd_vel ─> freno_lidar ─> driver
      └─> brazo/orden ─> wallens_brazo ─> brazo/transaccion (driver) ─> gripper ID 5
```

Paquete `ros/wallens_mision`: la lógica sin ROS (`scripts/mision_logica.py`) con su mundo falso y nodos delgados que
traducen tópicos (`scripts/mision_frutas.py`, `src/detector_frutas.cpp`). Regla: no se mueve el robot si los tests no
pasan.

Cambios en lo que ya existe:

| Pieza | Cambio | Estado |
|---|---|---|
| `wallens_brazo` | Órdenes sueltas `abrir`, `soltar`, `rearmar`, `cerrar`, `cerrar:<tipo>` | hecho |
| `wallens_brazo` | `brazo/ocupado` solo mientras ejecuta una orden (ni por llevar objeto ni por vigilar) | hecho |
| `wallens_brazo` | Sin tornillo (`usar_tornillo: false`) | hecho |
| `wallens_brazo` | Corriente de apriete y de sostén por tipo de fruta | hecho |
| `freno_lidar` | Huella con los dedos abiertos (±136 mm) cuando `brazo/estado` dice pinza abierta | pendiente |

Ojo: la fruta queda **por debajo del LiDAR** (16 cm): ni el freno ni Nav2 la ven. Por eso `d_parada` = alcance del gripper al
girar (0.285 m) + radio de la fruta + 5 cm ≈ 0.40 m, y se valida con 5 corridas antes de seguir.

## Fases

Cada fase empieza solo si la anterior pasó. Velocidades ≤ 0.10 m/s (`RETROCEDER` ≤ 0.03 m/s) y una persona junto al
interruptor durante todo movimiento.

### Fase 0 — Medidas (sin código)

**Medido (2026-11-02, `scripts/prueba-camaras.py`, 640x480 en MJPG; las dos en YUYV a la vez no caben en el USB):**
- `cam_frutas` (C270, ~55° diagonal): ±22° horizontal (`60 25` y `100 -40` quedaron fuera, como predice), casi a ras
  del suelo y horizontal, horizonte en la fila ~270. Fruta entera desde x ≈ 0.40 m (a 0.40 toca el borde de abajo);
  a 1 m el limón mide ~55 px, así que a 2 m sigue detectable (~28 px). f ≈ 770 px.
- `cam_garra` (70° diagonal, mira hacia abajo entre los dedos): en el suelo, en cm de base_link,
  `x = -12.58 - 0.0019 u - 0.0403 v`, `y = 7.55 - 0.0277 u + 0.0008 v` (error ±0.6 cm con 4 puntos). Ve centros de
  fruta de x -0.32 a -0.18 y |y| ≤ 6.5 cm; +y (izquierda del robot) sale a la izquierda de la imagen.
- Falta: `x_agarre` medido con la fruta agarrada (lo registra `VERIFICAR`, ver «Geometría del agarre»).
