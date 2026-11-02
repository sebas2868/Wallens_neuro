# Plan: misión de recoger una fruta y llevarla a un punto

Propuesto el 2026-11-02, revisado el mismo día con las respuestas del usuario. **Solo plan: no hay código ni cambios en el
robot.** Contexto: [sistema.md](sistema.md), [navegacion.md](navegacion.md), [gripper-plan.md](gripper-plan.md),
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

Implementada en la fase 2: `ros/wallens_mision/scripts/mision_logica.py` (sin ROS). Un estado hace una sola cosa, tiene
una condición de salida, un tiempo máximo y un motivo de fallo. Cualquier estado puede ir a `FALLO`: robot quieto, pide
`/parada` y, si lleva fruta, la sostiene. `/parada`, la batería baja, el brazo congelado y la falta de datos (odometría
0.5 s, brazo 1 s, batería 3 s) mandan sobre todo.

```
INICIO ─> IR_A_ZONA ─> BUSCAR ─> RODEAR ─> ACERCAR ─> GIRAR_180 ─> BAJAR ─> ABRIR ─> ALINEAR ─> CERRAR ─> VERIFICAR
                          │                                                            ▲          │
                          └─(no la ve / frutas muy juntas)─> FALLO       REINTENTO ────┘ (1 vez)  │ (sin agarre)
VERIFICAR ─> SUBIR ─> SALIR ─> IR_A_ENTREGA ─> ORIENTAR ─> DEJAR ─> BAJAR_ENTREGA ─> SOLTAR ─> SUBIR_ENTREGA
SUBIR_ENTREGA ─> ALEJAR ─> CERRAR_VACIO ─> SIGUIENTE ─(quedan)─> IR_A_ZONA
                                                    └─(lista vacía)─> FIN  (o VOLVER_HOME ─> FIN)
```

Los estados del ascensor (`BAJAR`, `SUBIR`, `BAJAR_ENTREGA`, `SUBIR_ENTREGA`) se saltan con `ascensor: false` (hoy).

| Estado | Qué hace | Sale cuando | Máx. |
|---|---|---|---|
| `INICIO` | Espera odometría, brazo y batería; exige batería ≥ 14.6 V, brazo no congelado, calibrado si hay ascensor, sin fruta (o con fruta si se empieza en un estado de entrega) | todo bien | 10 s |
| `IR_A_ZONA` | Rectas hasta un punto a `d_vista` = 0.8 m del centro de la zona (también se aleja si está más cerca), rodeando las frutas ya entregadas; gira hacia la zona | llega | 60 s |
| `BUSCAR` | Mira (0.6 s de espera por latencia, mínimo 1 s); si no ve la pedida, barre 0°, +20°, -20°, +40°, -40°. Registra **todas** las frutas de la zona | 3 detecciones coherentes de la pedida dentro de radio + 15 cm | 40 s |
| `RODEAR` | Si la dirección de aproximación libre no es la actual, rodea la fruta a 0.7 m en arcos de ≤ 45° | en la dirección elegida | 60 s |
| `ACERCAR` | Hacia la fruta corrigiendo con la cámara; de cerca sigue con la odometría | a `d_parada` = 0.40 ± 0.02 m | 40 s |
| `GIRAR_180` | Gira hasta que el gripper apunta a la fruta | ± 2° | 20 s |
| `ABRIR` | orden `abrir` | el brazo responde | 5 s |
| `ALINEAR` | Retrocede ≤ 0.03 m/s y corrige el rumbo con la posición de la fruta en `cam_garra` (o con la odometría mientras no la ve) | 3 cuadros de `cam_garra` en la zona de agarre (± 1.5 cm, ± 1.2 cm) | 30 s, ≤ 0.25 m |
| `CERRAR` | orden `cerrar:<tipo>` (corriente según la fruta) | `agarrado` → `VERIFICAR`; `sin_agarre` → `REINTENTO` | 5 s |
| `VERIFICAR` | `cam_garra` ve la fruta en la zona con la pinza cerrada | 3 cuadros; si no en 3 s → `REINTENTO` | 10 s |
| `REINTENTO` | `abrir`, avanza 5 cm y vuelve a `ALINEAR`. Solo una vez; la segunda es fallo | | 15 s |
| `SALIR` | Avanza 0.3 m en recta: girar ahí mismo barrería a las vecinas con el gripper (29 cm de alcance) | llega | 15 s |
| `IR_A_ENTREGA` | Rectas hasta el punto previo, a 0.5 m de la línea por el lado de la zona, rodeando las frutas que quedan en la zona y las entregadas | llega | 90 s |
| `ORIENTAR` | Gira con el frente lejos de la línea (gripper hacia ella) | ± 2° | 20 s |
| `DEJAR` | Marcha atrás hasta que la zona de agarre queda sobre su punto de la línea | llega | 30 s |
| `SOLTAR` | orden `soltar` | el brazo responde | 5 s |
| `ALEJAR` | Avanza 0.3 m con la pinza abierta (cerrar junto a la fruta la volvería a agarrar) | llega | 15 s |
| `CERRAR_VACIO` | orden `cerrar` | `sin_agarre` (si dice `agarrado`, fallo) | 5 s |
| `SIGUIENTE` | Pasa a la siguiente fruta de la lista | | |
| `VOLVER_HOME` | Si se pidió: rectas a (0, 0) rodeando frutas; si home está en la línea, se queda en el punto previo de la primera | llega | 90 s |

### Lo que salió de los tests (restricciones nuevas)

- **Frutas vecinas.** Con los dedos abiertos (±136 mm) el gripper toca a una vecina en casi cualquier dirección si están
  pegadas. Antes de acercarse, la misión revisa en el modelo de la huella toda la maniobra (acercarse, girar 180°,
  retroceder con los dedos abiertos con ±5° de desvío, salir con la fruta y girar) contra las frutas vistas en `BUSCAR`,
  y elige la dirección libre más parecida a la actual, en pasos de 15°. Si no hay ninguna, falla sin tocar nada.
  En el mundo falso (triángulo de 3 frutas, 40 disposiciones por separación): **12 cm: 6/40; 14 cm: 36/40; desde 15 cm:
  40/40**. Recomendado: **≥ 17 cm entre centros**. Un radio de zona de 10 cm con 3 frutas da ~17 cm si están repartidas.
- **Traslados.** Las rectas rodean las frutas conocidas por un círculo de radio fruta + alcance del gripper (0.287 m) +
  4 cm. El gripper barre 29 cm en cada giro en el sitio de un punto de paso.
- **Línea de entrega.** Tiene que quedar a ≥ ~1.1 m de la zona (punto previo 0.5 m + radio + 15 cm + alcance del
  gripper + fruta + margen); si no, el pedido se rechaza al crearlo.
- **`brazo/ocupado`.** Hoy el nodo del brazo publica `ocupado = moviéndose o con objeto`, y el freno anula las ruedas
  mientras sea true: **con la fruta agarrada el robot no podría ir a la entrega**. Cambio para la fase 4.

### Pruebas

`cd ros/wallens_mision && python3 -m unittest discover -s test -v` (40 tests, sin ROS, ~0.4 s). El mundo falso
(`test/mundo_falso.py`) integra la cinemática, simula `cam_frutas` (campo ±30°, de 0.35 a 2 m, ruido), `cam_garra`, el
brazo y la batería, y registra cualquier contacto del chasis, el carro o los dedos (abiertos o cerrados) con una fruta.
Además se corrieron 1000 misiones al azar (zona, separación de 15 a 24 cm, orden, lado de la línea, ruido de 0.5 a 2 cm):
**0 choques**; todas terminan salvo rechazos limpios con frutas a 15–16 cm o pedidos inválidos. Se comprobó que los tests
fallan si se meten a propósito 11 de 13 errores (los 2 que pasan no cambian el resultado en el mundo falso).

**Probar por partes:** `Mision(pedido, desde="ESTADO", hasta="ESTADO")`. `RODEAR`, `ACERCAR` y `GIRAR_180` buscan la
fruta antes con un `BUSCAR`; los estados de entrega exigen que el brazo ya la lleve. En el nodo serán `--desde/--hasta`.

### Órdenes que el nodo del brazo tiene que aceptar (fase 4)

`bajar`, `subir`, `abrir`, `soltar`, `cerrar` (vacío) y `cerrar:<tipo>` (con la corriente de esa fruta). Cada una
responde `(ok, texto)`; `cerrar*` responde `agarrado` o `sin_agarre`. El brazo solo recibe órdenes con las ruedas
quietas (el mundo falso lo comprueba).

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
cam_frutas ──> detector_frutas ──> /frutas (tipo, posición en base_link, diámetro)  ┐
cam_garra  ──> detector_garra  ──> /fruta/en_garra (error lateral, en_zona)         ├─> mision_frutas (máquina de estados)
/odometry/filtered, brazo/estado, /battery_state, /parada ─────────────────────────┘        ├─> /cmd_vel ─> freno_lidar ─> driver
                                                                                            └─> brazo/* (órdenes)
```

Paquete nuevo `ros/wallens_mision` (Python), con el patrón del brazo y del supervisor:
- **Lógica sin ROS**: `scripts/mision_logica.py` (la máquina, hecha) y `vision.py` (clasificación sobre una imagen).
  Tests con un mundo falso, reloj falso y **fotos reales guardadas** de las frutas (`test/fotos/`). Regla: no se mueve el
  robot si los tests no pasan.
- **Nodos delgados** que traducen tópicos.

Cambios en lo que ya existe:

| Pieza | Cambio | Por qué |
|---|---|---|
| `wallens_brazo` | Órdenes sueltas `bajar`, `subir`, `abrir`, `soltar`, `cerrar`, `cerrar:<tipo>` (hoy solo `recoger` de corrido) | `ALINEAR` va entre abrir y cerrar |
| `wallens_brazo` | `brazo/ocupado` solo mientras ejecuta una orden, no por llevar un objeto | Hoy con la fruta agarrada el freno anula las ruedas y no podría ir a la entrega |
| `wallens_brazo` | Parámetro `sin_ascensor` | Probar con el tornillo roto |
| `wallens_brazo` | Corriente de apriete por tipo de fruta (`cerrar` recibe el tipo) | Una mandarina se aplasta con la fuerza que necesita una naranja |
| `freno_lidar` | Huella con los dedos abiertos (±136 mm) cuando `brazo/estado` dice pinza abierta | `ALINEAR` mueve el robot con la pinza abierta |
| `freno_lidar` | Confirmar que `brazo/ocupado` es `false` entre órdenes | Si no, el freno anula `ALINEAR` |

Ojo: la fruta queda **por debajo del LiDAR** (16 cm): ni el freno ni Nav2 la ven. Por eso `d_parada` = alcance del gripper al
girar (0.285 m) + radio de la fruta + 5 cm ≈ 0.40 m, y se valida con 5 corridas antes de seguir.

## Fases

Cada fase empieza solo si la anterior pasó. Velocidades ≤ 0.10 m/s (`ALINEAR` ≤ 0.03 m/s) y una persona junto al
interruptor durante todo movimiento.

### Fase 0 — Medidas (sin código)

- Pose de las dos cámaras respecto a `base_link` (x, y, z, inclinación) → TF y `sistema.md`.
- Intrínsecas de las dos cámaras con un tablero de ajedrez (`camera_calibration`), YAML en `config/`.
- Zona de agarre en la imagen de `cam_garra`: foto de cada fruta bien puesta entre los dedos cerrados → rectángulo objetivo.
- Fotos de las 3 frutas (varias de cada una, a 0.4–1.5 m, distintas luces) para los tests.

### Fase G — Gripper (en paralelo con la 1; nunca se ha probado con fruta)

Sigue [gripper-plan.md](gripper-plan.md), sin el tornillo. Antes de cada prueba: `sudo systemctl stop wallens` o el
nodo del brazo por el bus del driver (`--bus=ros`), tests del brazo en verde.

| Prueba | Cómo | Pasa si |
|---|---|---|
| G1 abrir/cerrar en vacío | `prueba-brazo.py --etapa=2` | 2 ciclos, el cierre dice "sin contacto" |
| G2 pinza abierta quieta | `diagnostico-pinza.py`, estrategias A–D | se elige la que no oscila (hoy B) |
| G3 cierre sobre cada fruta, a mano | fruta sostenida en la zona de agarre, `cerrar` con corriente baja que se sube por pasos | se anota la corriente mínima que la sostiene y la máxima sin marcarla, por fruta |
| G4 sostener | fruta agarrada 60 s y una sacudida suave a mano | no resbala ni se marca; temperatura < 55 °C |
| G5 soltar | `soltar` con la fruta | cae limpia, la pinza queda abierta y luego cierra |
| G6 orden suelto por ROS | `brazo/abrir`, `brazo/cerrar` desde la terminal | igual que G1 y G3, `ocupado` vuelve a false |

### Fase 1 — Visión de frutas, robot quieto

- `vision.py` + `detector_frutas`. Tests con las fotos de la fase 0.
- **Pasa si:** con cada fruta a 0.5, 1.0 y 1.5 m y ±20°: tipo correcto 19 de 20, error de distancia < 5 cm, de ángulo < 3°;
  sin falsos positivos 2 min sin fruta; < 30 % de un núcleo.

### Fase 2 — Máquina de estados en el mundo falso

- `mision_logica.py` con todos los estados, tiempos y fallos; tests de cada transición y de cada fallo (fruta que no
  aparece, fruta que se pierde, `sin_agarre`, `/parada` a mitad, batería baja, brazo congelado).
- **Pasa si:** los tests pasan. Nada se mueve en esta fase.
- **Estado (2026-11-02): hecha.** 40 tests en verde y 1000 misiones al azar sin choques (detalle en
  [Máquina de estados](#máquina-de-estados)). Pendiente para la fase 3: los valores reales de `x_agarre`, campo de visión
  y distancia mínima de las cámaras (fase 0), y repasar `margen_vecinas`/`tolerancia_eje` con el robot real.

### Fase 3 — Moverse: `IR_A_ZONA`, `BUSCAR`, `ACERCAR`, `GIRAR_180`

- Por partes con `--desde/--hasta`. Fruta en el suelo en la zona dada.
- **Pasa si:** 5 de 5 corridas terminan con la fruta detrás, a `d_parada` ± 4 cm y ± 5° del eje, sin tocarla.

### Fase 4 — `ABRIR` → `VERIFICAR` (sin ascensor)

- `cam_garra` + `detector_garra`; luego alineación con la fruta en el suelo (el gripper ya está abajo).
- **Pasa si:** 5 de 5 agarres de cada fruta desde el final de la fase 3.

### Fase 5 — Entrega

- `IR_A_ENTREGA`, `ORIENTAR`, `SOLTAR`, `CERRAR_VACIO`, `SIGUIENTE`, con *home* y con un punto dado.
- Comprobar que la fruta agarrada no sobresale de la huella cerrada (±35 mm); si sobresale, huella con fruta.
- Dejar 3 frutas en línea (una de cada tipo) y comprobar que ninguna toca a otra ni se movió al dejar la siguiente.
- **Pasa si:** 5 de 5 entregas a menos de 10 cm de su punto, y 3 de 3 líneas sin tocar las frutas ya dejadas.

### Fase 6 — Misión completa (sin ascensor)

- `mision.launch.py` (detectores + misión). Lista de 3 frutas en un orden dado: 5 de 5 misiones sin intervención, y un
  `/parada` a mitad que deja todo quieto.

### Fase 7 — Con ascensor (cuando el tornillo esté arreglado)

- Cero sin golpear topes duros (pendiente del incidente en `gripper-plan.md`), etapas 1–3 del brazo.
- Holgura gripper–suelo con el carro abajo (el modelo da ~5 mm) y altura de agarre según la fruta.
- Quitar `sin_ascensor` y repetir las fases 4–6: navegar con el gripper abajo como hasta ahora, y subir la fruta tras
  agarrarla para no arrastrarla.

### Nav2 después

Cuando haga falta esquivar obstáculos: `IR_A_ZONA` e `IR_A_ENTREGA` mandan una meta a `navigate_to_pose` en vez de la
recta, con un mapa guardado y `slam_toolbox` en modo localización (las coordenadas pasan a `map`). El resto de estados
no cambia. `ACERCAR` y `ALINEAR` siguen siendo control directo: Nav2 no ve la fruta.

## Riesgos

- **CPU:** cámaras por turnos; sin SLAM en la misión.
- **Luz:** el color cambia con la iluminación; calibrar en el lugar real.
- **Naranja vs. mandarina** depende de la estimación de tamaño: si falla, la fase 1 lo muestra con las fotos.
- **Fruta invisible al LiDAR:** el margen de `d_parada` y la fase 3 cubren el riesgo de pisarla o empujarla al girar.

## Preguntas abiertas

Respondidas (2026-11-02): las frutas y el gripper están al ras del suelo; recoge en el orden pedido y deja todas en una
línea; zona y entrega son variables por corrida.

1. Altura e inclinación de `cam_frutas` (se mide en la fase 0).
2. Dirección de la línea de entrega: ¿la damos como ángulo en `odom` o como un segundo punto (inicio y fin de la línea)?
   Propuesta: punto + ángulo, separación 0.25 m.
3. Convención de ejes: x hacia el frente del robot al arrancar, y hacia su izquierda (la de ROS). ¿Vale así?
