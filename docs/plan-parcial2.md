# Plan del Parcial 2: de la voz a la caja o la baldosa

Escrito el 2026-11-03, día antes de la presentación (miércoles 4 de noviembre). Es el plan maestro: dice **todo** lo que
falta, en qué orden y cómo se prueba cada bloque. El detalle del agarre ya está en
[mision-frutas-plan.md](mision-frutas-plan.md); el del sistema, en [sistema.md](sistema.md),
[navegacion.md](navegacion.md), [tornillo.md](tornillo.md) y [camaras.md](camaras.md).

**Prioridad (decidida el 2026-11-03): primero el circuito completo** (agarrar la fruta y dejarla en su caja); la voz y
lo demás después. Primer paso: la **prueba de la caja teleoperada** (sección 6, B0.7), ya escrita.

Regla de trabajo: **un bloque a la vez, probado solo antes de juntarlo con otro.** No se mueve el robot si los tests no
pasan. Cada fallo del robot real se reproduce primero en el mundo falso y después se corrige.

## 1. El enunciado (lo que dijo el profe, no hay documento escrito)

- Hay **3 frutas regadas** por el espacio: limón, naranja y mandarina.
- Hay **3 cajas regadas**, no muy altas, cada una con la **imagen impresa** de una de las frutas.
- El parcial empieza con **alguien que le habla al robot** y dice el nombre de una fruta.
- El robot reconoce qué fruta dijo, **la busca, va hasta ella, se gira y la agarra** con el gripper.
- Decisión de la líder (2026-11-03): una fruta por pedido, en un espacio de ~3 × 3 m con todo al azar.
  Después de agarrar **sube la fruta**, se queda quieto y pregunta **«¿caja o baldosa?»** en `mision/estado`.
  La respuesta llega desde el PC por `mision/final`, sin voz: `caja` o `baldosa`.
- **Caja:** busca la tarjeta de su fruta, llega por el frente bajo, gira y entra de reversa con la fruta arriba;
  suelta adentro. **Baldosa:** busca la de 25 × 25 × 2.5 cm con otra fruta igual en el centro; baja despacio hasta
  contacto, abre de a poco, sube un poco abierta, se aleja y cierra lejos. La fruta siempre viaja arriba.
- Al terminar vuelve a esperar otro pedido de voz. Pregunta y finales integrados en código; validación real pendiente.
  La líder autorizó continuar todos los bloques sin pausas en esta sesión. Pruebas y comandos:
  [pruebas-manana.md](pruebas-manana.md).

Consecuencias para el diseño:

1. Las posiciones **no son fijas**: hay que encontrar frutas y cajas con la cámara. El pedido actual
   (`zona_x`, `zona_y` y la entrega como parámetros) no basta: hace falta un **mapa de objetos** que el robot arme solo.
2. Frutas, baldosa y pared frontal de la caja (7.8 cm) quedan bajo el sensor láser (16 cm); la pared trasera de la caja
   (23.8 cm) sí se ve. La navegación también debe esquivar lo que solo vio la cámara.
3. Mientras busca la caja, las **otras dos frutas siguen en el piso** y tienen colores parecidos a las fotos de las cajas:
   el detector tiene que distinguir "fruta real en el piso" de "foto impresa en una caja".

## 2. Preguntas para el profe (o supuestos si no responde)

| Pregunta | Supuesto mientras tanto | Qué cambia si es distinto |
|---|---|---|
| ¿Se pide **una** fruta o se repite con las tres? | Una por corrida; se puede repetir la misión con otra palabra | Con tres: la máquina de estados ya trae `SIGUIENTE` |
| ¿Quién hace las cajas y de qué tamaño? | Las armamos nosotros (memoria del 31 oct): interior ≥ 15 × 15 cm, pared ≤ 5 cm, foto grande en la pared que mira al centro | Si las pone él: medirlas apenas lleguen y ajustar `Config` |
| ¿Dónde va la foto: en la pared, en el fondo o en un cartel? | En la cara exterior de la pared, a la vista de la cámara del frente | Si va solo en el fondo, la cámara del frente no la ve desde lejos: habría que acercarse a cada caja |
| ¿Hay paredes o límites en el espacio? ¿De qué tamaño es? | Espacio cerrado de ~3 × 3 m, piso liso | Más grande: más puntos de vista al reconocer |
| ¿Puede haber gente u otros objetos en el camino? | No | El freno con el sensor láser cubre lo alto; lo bajo no |
| ¿Se puede repetir la orden de voz si no entiende? | Sí | Si no: confirmar en voz alta antes de arrancar |

## 3. Estado hoy (2026-11-04)

Medidas y pruebas de hoy: [resultados-20261104.md](resultados-20261104.md). Código local; todavía no desplegado.

| Pieza | Estado |
|---|---|
| Base: ruedas, odometría filtrada, unidad de medición inercial, freno con el sensor láser, batería, `/parada` | ✅ medido por la líder: radio 0.0360, separación 0.2647, `k_left=1.000`, escala de giro 0.971; vueltas con error menor de 1°. Se preservan sus cambios |
| Tornillo del ascensor (ID 3) | ✅ **arreglado**; recorrido útil 36037 ticks, abajo 25577, arriba útil -10460 (`scripts/calibracion-tornillo.json`, [tornillo.md](tornillo.md)). B4 integrado en la lógica: homing si falta cero, bajar antes de abrir, subir antes de salir. Pruebas del mundo falso al pie de B4; **pendientes 5 agarres reales** |
| Prueba de la caja teleoperada (`scripts/prueba-caja.sh`) | ✅ datos integrados en B6: 20.5 / 4.7 cm, ~400 ticks. **Revalidar apertura con la pinza nueva** registrada por la otra sesión durante este trabajo |
| Detectar cajas | ✅ marco 0.072 m; C++ corregido para marco fino C270. Tres tarjetas 31/31 a 0.5 m; desde lejos localiza turquesa y usa láser, visita a 0.7 m y lee quieto. Mandarina cercana 495/496, cero tipos equivocados. A 2 m, una toma de mandarina localiza 23/31; ver límites en el reporte |
| Gripper (ID 5): abrir, cerrar por fruta, sostener | ✅ sostén 200 para las tres frutas (pinza nueva, 3 nov 20:16). Altura segura 3200; no acciona dedos sin cero ni junto al piso. Homing y rearme no abren. ⚠️ el cierre es `por_verificar`, nunca prueba agarre por posición |
| Agarre completo del limón (`scripts/mision.sh X Y`) | ⚠️ repetir cinco veces con altura nueva. Medir región de cam_garra ARRIBA para distinguir fruta sostenida de fruta que quedó en el piso; sin esa región la misión no transporta |
| Detector de frutas (`detector_frutas.cpp`) | ✅ referencias y filtro de diámetro; medidas actuales limón 0.050, naranja 0.070, mandarina 0.074 m. Cero falsas frutas en las capturas de tarjetas evaluadas con el binario. Falta ≥95 % con frutas reales a las tres distancias |
| Voz (`PARCIAL2/voz_fruta_pi4/voz.py`, Vosk sin internet, 3 palabras) | ✅ envoltura `voz_pedido.py`, confianza ≥0.7 y rechazo de desconocidos. Fuente/modelo copiados a Pi; micrófono C270 identificado. **Instalación bloqueada por contraseña sudo**, faltan 15 pedidos con ruido |
| Reconocer el espacio y armar el mapa de objetos | ✅ mapa sin ROS, agrupación por clase/tipo y tres cuadros distintos; barrido por modos y exploración limitada. Navegación local rodea rectángulos; no se integró Nav2 |
| Ir a la caja y soltar adentro | ✅ pregunta, caja por parámetro o tarjeta, entrada con distancia robusta y freno existente, apertura parcial y retirada; plan B `IR_A_ENTREGA/DEJAR` conservado |
| Preguntar el final y entregar en baldosa | ✅ ventanas calculadas con dos puntos medidos POR FRUTA, sin promedio ni cero supuesto; cámara trasera elige la fruta central entre candidatas y excluye la transportada. Falta probar contacto y entrega reales |
| Simulación | ✅ mundo falso extendido a cajas, baldosas, distractor igual en piso, ascensor y reconocimiento; tandas de 300 en cada cierre. Geometría y alturas sintéticas no sustituyen la prueba física. MuJoCo no se modificó |
| Cajas físicas | ✅ frente turquesa real 3.3 cm; fondo de fruta arriba 4.5–4.8 cm, holgura 1.2–1.5 cm. Dedos arriba ~4.9 cm (extrapolados). Exige subida completa antes de entrar. No recortar salvo roce observado por la líder |

## 4. Arquitectura final

Se mantiene la estructura actual: **la lógica en Python puro sin ROS** (`mision_logica.py`, probada con el mundo falso),
**nodos delgados** que solo traducen tópicos y **un solo dueño de cada cámara** (`detector_frutas`).

```
 micrófono (C270 o USB)
        │
   voz_pedido.py ──── mision/pedido ("limon" | "naranja" | "mandarina") ───┐
                                                                           ▼
                        camara ("frutas" | "garra" | "cajas" | "")   mision_frutas.py
   detector_frutas.cpp ◄─────────────────────────────────────────────  (envuelve a mision_logica.Mision)
        │  frutas/detecciones  {tipo, x, y, d}   (fruta real en el piso)      │
        │  cajas/detecciones   {tipo, x, y, ancho} (foto impresa en una caja) │
        │  garra/fruta         {visible, x, y, tipo}                          │
        └──────────────────────────────────────────────────────────────────►  │
                                                                             ├─► cmd_vel ─► freno_lidar ─► driver
   odometry/filtered, brazo/estado, brazo/resultado, battery_state, parada ─►├─► brazo/orden ─► nodo del brazo
                                                                             └─► mision/estado (para mostrar en vivo)
```

Archivos (★ nuevo o con cambios):

```
ros/wallens_mision/
├── scripts/
│   ├── voz_pedido.py        ★ nodo: escucha, publica mision/pedido; vuelve a escuchar si no entiende
│   ├── mision_frutas.py     ★ espera mision/pedido para crear la Mision; publica mision/estado
│   ├── mision_logica.py     ★ RECONOCER, mapa de objetos, cajas, ascensor
│   └── mapa_objetos.py      ★ (sin ROS) agrupa detecciones en odom: frutas y cajas con su posición y confianza
├── src/detector_frutas.cpp  ★ mandarina; modo "cajas"; separar fruta real de foto impresa
├── config/frutas.yaml       ★ umbrales medidos en el lugar (hoy están en el código como parámetros)
└── test/
    ├── mundo_falso.py       ★ cajas, frutas al azar, cámara que confunde fotos con frutas
    ├── test_mapa_objetos.py ★
    └── test_mision.py       ★ casos nuevos
launch/mision.launch.py      ★ lanza también voz_pedido
scripts/probar.sh            ★ etapas nuevas: voz, reconocer, caja, completa
```

## 5. Máquina de estados final

```
ESPERAR_PEDIDO ─► INICIO ─► RECONOCER ─► IR_A_FRUTA ─► BUSCAR ─► ACERCAR ─► GIRAR ─► BAJAR_Y_ABRIR ─► RETROCEDER
       ▲              (homing del tornillo,   │  (no vio la pedida)                                         │
       │               pinza cerrada)         └─► EXPLORAR ─► RECONOCER                                     ▼
       │                                                                        VERIFICAR_Y_SUBIR ◄─── CERRAR
       │                                                                                 ▼
       │                                                                          PREGUNTAR_FINAL
       │   caja: SALIR → IR_A_CAJA / BUSCAR_CAJA → ACERCAR_CAJA → GIRAR_CAJA → ENTRAR_A_CAJA → SOLTAR
       │   baldosa: BUSCAR_BALDOSA → IR_A_BALDOSA → GIRAR → ACERCAR_BALDOSA → BAJAR_CONTACTO → SOLTAR_SUAVE → SUBIR_UN_POCO
       └──────── FIN ◄── CERRAR_VACIO ◄── ALEJAR ◄── (ambos finales)
```

| Estado | Cámara | Qué hace | Sale cuando |
|---|---|---|---|
| `ESPERAR_PEDIDO` ★ | — | Quieto, espera `mision/pedido` | llega una fruta válida |
| `INICIO` | la del primer estado | Homing si falta cero (máximo 90 s), SUBIR y luego cerrar en vacío. Sin cero no mueve pinza ni ruedas. Rearme sin abrir; con fruta exige cero vigente | resultado y estado actualizado, libre y calibrado |
| `RECONOCER` ★ | frutas | Gira 360° en pasos de 40° (el campo de la C270 es ±22°), quieto 1 s en cada uno; suma al mapa de objetos todas las frutas y cajas que ve | vuelta completa |
| `EXPLORAR` ★ | frutas | Si la fruta pedida no está en el mapa: va a un punto de vista nuevo (rejilla de 1 m dentro del espacio, el más cercano sin visitar) y vuelve a `RECONOCER`. Máximo 4 puntos | la ve; si no, `FALLO` |
| `IR_A_FRUTA` | frutas | Era `IR_A_ZONA`: la zona sale del mapa de objetos (radio 15 cm), rodeando frutas y **cajas** conocidas | llega a 0.8 m |
| `BUSCAR` … `CERRAR` | frutas / garra | Como hoy; `ABRIR` pasa a `BAJAR_Y_ABRIR` (orden `bajar` y luego `abrir`) | |
| `VERIFICAR_Y_SUBIR` | garra | Conserva `VERIFICAR`: ante `por_verificar` exige tres vistas entre dedos, sube y exige tres vistas nuevas en región ARRIBA distinta de la región del piso | `listo`, brazo libre y región visual calibrada; sin ella falla quieto sosteniendo |
| `PREGUNTAR_FINAL` ★ | — | Quieto y con fruta arriba: publica «¿caja o baldosa?»; espera `mision/final` con entrega del último mensaje a suscriptores nuevos (`transient_local`). Repite ante respuesta inválida; parámetro `final:=caja\|baldosa` para pruebas | respuesta válida; 120 s sin respuesta → `FALLO` sosteniendo la fruta |
| `SALIR` | — | Como hoy | |
| `IR_A_CAJA` ★ | — | Traslado a 0.8 m de su caja (posición del mapa), de frente a la foto, rodeando frutas y cajas | llega |
| `BUSCAR_CAJA` ★ | cajas | Barrido 360° en pasos de 40°, solo modo cajas. Turquesa + pared alta del láser → candidata; elige la más cercana sin revisar. Si no quedan, explora hasta cuatro puntos | candidata o caja pedida conocida |
| `IR_A_CANDIDATA` / `LEER_CAJA` / `REACERCAR_CAJA` | cajas | Va al frente a 0.7 m de la tarjeta; quieto exige tres lecturas. Otra fruta: guarda tipo y continúa. Ilegible 3 s: se acerca 10 cm una vez; segundo fallo la marca sin leer | pedida → acercar; otra/ilegible → buscar siguiente |
| `ACERCAR_CAJA` ★ | cajas | De frente, corrigiendo con la cámara, hasta `d_parada_caja` (lo que barre el gripper al girar + media caja + margen) | llega |
| `GIRAR_CAJA` ★ | — | 180°: el gripper queda hacia la caja | ± 2° |
| `ENTRAR_A_CAJA` ★ | — | Repite SUBIR hasta `listo`, comprueba holgura medida, reversa lenta por láser 20.5 → 4.7 cm y freno modo caja. Ascensor arriba del todo durante entrada | llega, o falla quieto |
| `SOLTAR` / `ALEJAR` / `CERRAR_VACIO` | — | Como hoy | |
| `BUSCAR_BALDOSA` / `IR_A_BALDOSA` / `GIRAR` / `ACERCAR_BALDOSA` ★ | frutas / garra | Identifica la fruta pedida elevada sobre el cuadrado claro; no acepta la igual en el piso. Llega por un lado libre y retrocede con odometría y cámara trasera | posición de entrega libre de la fruta del centro |
| `BAJAR_CONTACTO` / `SOLTAR_SUAVE` / `SUBIR_UN_POCO` ★ | — | Baja a velocidad ≤ 40 y corriente limitada; contacto dentro de ventana de altura. Abre en pasos de 50 hasta ~400 ticks; sube un poco abierta antes de alejarse | contacto y órdenes completadas; sin contacto en la ventana → fallo |
| `FIN` ★ | — | Publica "misión cumplida" en `mision/estado`; vuelve a `ESPERAR_PEDIDO` | |

Se conservan `IR_A_ENTREGA` y `DEJAR` como plan B (ver sección 9). Los estados con ★ ya están implementados;
su aprobación física sigue pendiente. `automatico=false` conserva la misión anterior; el nodo arranca por defecto
esperando pedidos, con `automatico=true` y `reconocer=true`. Los scripts de pruebas anteriores pasan
`esperar_pedido=false`. La líder autorizó seguir todos los bloques sin pausas.

## 6. Bloques de trabajo

Columnas: **Prueba** = cómo se sabe que está listo (sin eso no se pasa al siguiente). **Quién**: lo llena la líder.

### B0. Medidas y cajas físicas (sin código, en paralelo con B1-B3)

| # | Qué | Prueba | Quién |
|---|---|---|---|
| B0.1 | **Construir las 3 cajas**: interior ≥ 15 × 15 cm, pared ≤ 5 cm, foto impresa grande (≥ 12 cm) en la cara que mira hacia afuera, fondo liso | Las 3 hechas y fotografiadas | |
| B0.2 | Medir con el carro **arriba**: altura del fondo de la fruta y de los dedos sobre el piso | Fruta > 1 cm por encima de la pared | |
| B0.3 | Medir si el **chasis o el carro chocan con la pared** al retroceder hasta poner la fruta en el centro de la caja (la fruta va en x ≈ -0.23 y el carro empieza en x = -0.148) | Holgura medida; si choca: caja más ancha o soltar más cerca de la pared | |
| B0.4 | Corriente de sostén de la **mandarina** (fase G del gripper, como limón 200 y naranja 300) | Valor en `SOSTEN_FRUTA` | |
| B0.5 | Fotos con `cam_frutas` en el lugar real: cada fruta a 0.5, 1 y 1.5 m; cada caja a 0.5, 1 y 1.5 m; las tres frutas juntas | Carpeta `registros/fotos-lugar/` | |
| B0.6 | Dar de alta el homing del tornillo: probar a mano `jog-tornillo.py` hasta el tope inferior con la calibración nueva | Baja, detecta el tope por corriente y sube a la altura de viaje sin pasarse | |

| B0.7 | **Prueba de la caja teleoperada** (ver abajo), primero `vacio` y luego `limon` | `resumen.json` con cuánto se movió entre los dos momentos y el LiDAR en cada uno; el limón cae dentro | |

#### B0.7 Prueba de la caja TELEOPERADA (código listo, 2026-11-03)

La maneja una persona desde el PC; el programa solo sube, abre y mide. `scripts/prueba-caja.sh [vacio|limon]`
(despliega, entra a la Pi y al terminar trae el registro a `registros/caja-<modo>-<fecha>/`).

1. Si el tornillo no tiene cero, pregunta y hace el homing. Baja la tenaza. En modo `limon` abre, espera a que pongas el
   limón entre los dedos (Enter) y lo agarra; en `vacio` cierra la pinza.
2. **Momento 1:** mueves el robot con el teclado hasta dejarlo frente a la caja (el gripper está atrás) y das Enter:
   **sube la tenaza al tope**.
3. **Momento 2:** lo acercas más a la caja con la tenaza arriba y das Enter: **abre el gripper** y suelta. Mientras
   tanto el programa pide el **modo caja** al freno LiDAR: la reversa llega hasta 2 cm de la punta de los dedos (no
   15 cm) a ≤ 3 cm/s; el avance y el giro no cambian y, si el programa se cae, vence solo en 1 s. Con
   `CAJA=0.03 scripts/prueba-caja.sh limon` se cambian los 2 cm. Probado con el freno real compilado y un robot
   falso: en el momento 1 frenó a 14.9 cm de la pared, en el momento 2 entró 13 cm más y frenó a 2.0 cm.
4. Lo alejas y Enter: cierra la pinza vacía y baja la tenaza (`q` sale sin tocar el brazo).

**Primera corrida real con el limón (2026-11-03, `registros/caja-limon-20261103-144404`): el limón quedó dentro.**
Momento 1: pared del fondo a 20.5 cm de la punta de los dedos; momento 2: 14.7 cm más de reversa (pasos de 2 cm), pared a
4.7 cm. Abierta del todo, la pinza chocó con las paredes de la caja en 3243 (agarraba en 3815) y el brazo se congeló:
ahora suelta con `soltar:300` (abre 300 ticks desde donde agarra; `SOLTAR=` lo cambia), cierra y baja solo después de
alejarlo, y si arranca congelado o sin cero te deja alejarlo antes de abrir.

**Modo `auto`** (`scripts/prueba-caja.sh auto`): el primer paso hacia lo automático. Agarra el limón; tú lo pones lejos
y de frente a la caja (al menos 25 cm del chasis para girar), y él solo gira 180° y se acerca de reversa con el LiDAR
hasta la distancia del momento 1 (la del último `caja-limon-*`, o `D1=`). Ahí sube la tenaza, pide el modo caja y se
acerca hasta la del momento 2 (`D2=`). La pinza la abres tú de a poco con `o` (pasos de 20, 50, 100 o 200 ticks), y
en `resumen.json` queda `auto` con objetivo contra medido en cada momento, el giro y los ticks que abriste en total.

Primera corrida en el robot (registros/caja-auto-20261103-174956): giró 180.3° (0.5 cm de deriva), el limón cayó dentro
tras abrir **~400 ticks** (8 pasos de 50, de 3820 a 3422, sin tocar la caja: la corriente no subió). Paró antes: momento 1
a 23.7 cm (pedido 20.5) y momento 2 a 5.9 (pedido 4.7). Causa: con el robot quieto, 1 de cada 4 vueltas el mínimo del
LiDAR detrás salía 2-5 cm corto, y el filtro tomaba muestras repetidas (20 Hz sobre un LiDAR de 10). Ahora usa la
mediana de los 5 puntos más cercanos y la mediana de 5 vueltas distintas (`atras_robusta`, también en el CSV).

**Modo `solo`** (`scripts/prueba-caja.sh solo`): todo solo tras poner el limón. Detector de cajas nuevo
(`scripts/vision/cajas.py`, tests con fotos reales en `vision/test_cajas.py`): la caja es turquesa (H ~94, único en el
laboratorio) y la tarjeta se encuentra por su borde; la fruta impresa se clasifica por el color DENTRO de la tarjeta
(limón H ~51 S ~17; naranja H ~23 S ~140; el cartón también da H ~22). Busca girando de 30 en 30° (3 cuadros por parada,
2 de 3 deben ver la caja de la fruta; no gira si algo queda a menos de 33 cm del centro), se centra, avanza de frente en
tramos de 30 cm re-centrándose hasta 37.9 cm del chasis (tras el giro la punta queda 4 cm más allá del momento 1), y
sigue como auto; suelta `soltar:400`, avanza 25 cm, cierra y baja. Si el freno lo para a menos de 1.5 cm del objetivo lo
da por llegado (el freno usa el punto más cercano sin filtrar). Probado en Docker con cámara falsa
(`PRUEBA_CAMARA_FALSA=dir`); falta en el robot.

Teclas: `w`/`s` avanza/retrocede un paso, `a`/`d` gira un paso, `1`-`5` tamaño del paso (0.5 cm/1°, 1 cm/2°,
2 cm/5°, 5 cm/15°, 10 cm/45°), espacio para, `i` lecturas. Cada paso se mide con la odometría y pasa por el freno
LiDAR; si algo lo frena, lo dice.

`resumen.json` guarda en cada momento la pose, el LiDAR (lo más cercano delante del chasis y detrás de la punta de los
dedos), el freno, la posición del tornillo y la pinza, y **cuánto se movió desde el momento anterior**; al salir imprime
cuánto lo moviste entre el momento 1 y el 2. `registro.csv`: lo mismo 10 veces por segundo.

Probado contra un robot falso en ROS Jazzy (homing, agarre, pasos, subir, soltar, registro); **falta en el robot real**.
Lo que sale de aquí (distancia de parada, cuánto entrar con la tenaza arriba, qué ve el LiDAR) es lo que después usa
la misión automática (B6).

### B1. Mandarina en el detector

- Antes: un solo corte de tono, ya reemplazado por referencias de varias señales. Medido el 31 oct: naranja ≈ 25, **mandarina ≈ 35, limón ≈ 37**. Por tono, la mandarina
  cae como limón: **el tono no alcanza**.
- Clasificar con varias señales: tono, saturación, **textura** (la mandarina es rugosa), **diámetro real** (la mandarina
  es la más ancha, el limón el más pequeño: el detector ya estima `d` con la distancia al piso) y forma (el limón es
  alargado). Reusar las reglas de `voz_fruta_pi4/fruta.py`, que ya separaba las tres con las fotos reales.
- Pasar los umbrales a `config/frutas.yaml`.
- **Prueba:** `scripts/vision/camara.py grabar|evaluar` con las fotos de B0.5: las 3 frutas bien clasificadas en ≥ 95 %
  de los cuadros, a 0.5, 1 y 1.5 m, por separado y juntas.

### B2. Voz como nodo

- `voz_pedido.py`: envuelve `voz.py` (Vosk limitado a "limón", "naranja", "mandarina" y "[unk]", confianza ≥ 0.7).
  Publica en `mision/pedido` (`transient_local`, para que la misión lo reciba aunque arranque después).
- Micrófono: el de la C270 (es otro dispositivo de audio, no choca con el video del detector) o uno USB. Comprobar con
  `arecord -l` en la Pi.
- Confirmación: repetir en voz alta lo que entendió (`espeak-ng -v es "voy por la mandarina"`) o, como mínimo, en el log.
  Si no entiende, dice "no entendí" y vuelve a escuchar.
- Instalar Vosk y el modelo (`voz_fruta_pi4/instalar.sh`) en la Pi.
- **Prueba:** en la Pi, `ros2 topic echo /mision/pedido` mientras 3 personas distintas dicen cada fruta 5 veces, con el
  ruido del salón: ≥ 13 de 15 bien y **0 frutas equivocadas** (es mejor "no entendí" que ir por otra).

### B3. El pedido por tópico

- `mision_frutas.py` arranca en `ESPERAR_PEDIDO` y crea la misión al llegar `mision/pedido`. Se mantienen los
  parámetros (`frutas`, `zona_x`…) para las pruebas por etapas.
- Publica `mision/estado` (estado actual + aviso) para verlo en vivo desde el PC durante la presentación.
- **Prueba:** publicar a mano `ros2 topic pub /mision/pedido std_msgs/msg/String "{data: limon}" -r 5 --times 10` y que haga el
  agarre del limón como hoy con `mision.sh`.

### B4. Ascensor en la misión (el tornillo ya está arreglado)

- Implementado: `Config.usar_tornillo=True`, expuesto como parámetro del nodo de misión. En false conserva el
  recorrido anterior sin órdenes al tornillo; los 61 casos históricos lo prueban explícitamente.
- `INICIO` calibra solo si falta cero y espera resultado más estado actualizado. Límite de calibración: 90 s desde
  la orden; espera inicial de datos: 60 s. Con fruta y sin cero falla sin abrir (el homing abre la pinza).
- `ABRIR`: `bajar` → `listo` → `abrir`. `VERIFICAR`: mide abajo → `subir` → `listo` y brazo libre → `SALIR`
  o fin de la etapa. Límite por tramo: 60 s, igual que `brazo.Config.tiempo_max_t`; subida real medida ~25 s.
- Pérdida de cero, rechazo, exceso de tiempo o `/parada` → `FALLO`, ruedas quietas y sin ordenar soltar.
  La subida no modifica la calibración de `cam_garra`. No se modifican las órdenes ni los límites del brazo.
- Cierre reproducible: `cd ros/wallens_mision && WALLENS_CIERRE_B4=1 python3 -m unittest discover -s test`.
  Sin esa variable las 300 misiones se omiten; se corren solamente al cerrar el bloque.
- Resultado local (2026-11-03): antes **61 tests, OK (0.502 s)**; después **79 tests, OK (10.308 s)** con la tanda
  incluida: 61 históricos, 17 nuevos de ascensor y 1 prueba de 300 misiones. **300/300 terminadas, 0 choques,
  0 movimientos con brazo ocupado o fruta abajo**. Sin la tanda: 78 ejecutados, 1 omitido (0.788 s).
  Semillas 0–299: varían posición de la zona, orientación y separación de las tres frutas, fruta pedida, ruido,
  cero vigente y altura inicial; una fruta entregada por misión al punto fijo conocido. Esto valida B4, no cajas,
  baldosa ni reconocimiento. No se ejecutaron motores ni se desplegó; **la aprobación real sigue pendiente**.
- **Prueba:** `scripts/probar.sh agarre` 5 veces con el limón: baja, agarra, sube y la sostiene arriba 30 s sin
  sobrecarga del gripper.

### B5. Mapa de objetos y `RECONOCER` (solo lógica, en el mundo falso)

- `mapa_objetos.py`: recibe detecciones en `base_link` con la pose, las pasa a odom y las agrupa (radio 10 cm para
  frutas, 15 cm para cajas). Cada objeto: tipo, posición media, número de veces visto. Se acepta con ≥ 3 detecciones.
- `RECONOCER` y `EXPLORAR` en `mision_logica.py`. `IR_A_FRUTA` toma la zona del mapa.
- `_ir_a` esquiva también las cajas (como rectángulo con margen, igual que hoy las frutas).
- Mundo falso: frutas y cajas en posiciones al azar dentro de 3 × 3 m, con separación mínima; cámara simulada que a
  veces confunde una foto con una fruta.
- **Prueba:** 300 misiones al azar en el mundo falso: 0 choques con frutas o cajas, ≥ 95 % encuentra la fruta pedida.

### B6. Cajas en la lógica (en el mundo falso)

- `IR_A_CAJA`, `BUSCAR_CAJA`, `ACERCAR_CAJA`, `GIRAR_CAJA`, `ENTRAR_A_CAJA` con sus tiempos máximos.
- El mundo falso registra si la fruta soltada quedó **dentro** del interior de la caja correcta y si el chasis tocó una
  pared (con las medidas de B0.3).
- **Prueba:** 300 misiones completas al azar: ≥ 95 % con la fruta dentro de su caja, 0 en una caja equivocada.

### B7. Cajas en el detector

- Modo `"cajas"` en `detector_frutas.cpp` (es el dueño de la cámara: un nodo aparte no puede abrirla a la vez).
- Separar **foto impresa** de **fruta real**: hoy el detector ya descarta lo que no cuadra con el tamaño esperado de una
  fruta en el piso (`tam_min` / `tam_max` con la distancia proyectada al piso). La foto está más alta que el piso y
  mide ≥ 12 cm: en modo "frutas" se descarta y en modo "cajas" es justo lo que se busca. Se puede reforzar con el
  rectángulo de la caja (contorno de 4 lados alrededor de la mancha).
- Implementado con la esquina negra de la tarjeta, sin clasificar su fruta impresa por color. Posición y rumbo
  salen de la perspectiva del marco y su ancho medido; del plano de la tarjeta se desplaza 16.5 cm hacia el frente.
  Si falta el ancho, no se publica posición. El modo cajas no ejecuta el detector de frutas.
- **Prueba:** con B0.5, cada caja bien identificada a 0.5, 1 y 1.5 m; con las 3 frutas reales en el piso al lado, en
  modo "cajas" no sale ninguna fruta y en modo "frutas" no sale ninguna caja.

### B8. Simulación

**Integración local, 2026-11-03:** se añadieron caja y baldosa al mundo falso. Las tandas de cierre se activan con
`WALLENS_CIERRE_B4=1`, `WALLENS_CIERRE_CAJA=1`, `WALLENS_CIERRE_BALDOSA=1`, `WALLENS_CIERRE_MAPA=1` y
`WALLENS_CIERRE_COMPLETA=1`, respectivamente. Sin estas variables se omiten las tandas, manteniendo las pruebas
rápidas para uso diario. Cajas y baldosas se prueban por separado y también tras reconocimiento y agarre.
Resultado final: **100 tests, OK (34.207 s), cinco tandas de 300: todas completas, cero choques y cero destinos
equivocados en los escenarios probados**. Brazo: **139 tests, OK**. Compilación local Jazzy y adaptador de tópicos
probados sin red; 10 tests de visión anteriores conservados y tres tarjetas reales identificadas en C++.
Detalle de límites, instalación de voz pendiente de sudo y orden de pruebas reales en [pruebas-manana.md](pruebas-manana.md).

La entrega en baldosa tiene órdenes `bajar_hasta_contacto:MIN:MAX` y `subir_corto:TICKS` en el brazo. MIN/MAX son
alturas sobre el cero; se calculan con B0.2. Contacto reutiliza la detección de detención con corriente del homing,
con velocidad ≤40 y corriente ≤120 (90 en la configuración actual). Rebasar la ventana sin contacto no abre la
pinza; `/parada` aborta. `soltar:50` ocho veces, subida corta abierta, avance y cierre lejos. Validado en bus falso;
**primero probarlo aislado en el robot**, con la pinza nueva.

Dos niveles, con propósitos distintos:

| Nivel | Qué es | Para qué | Prioridad |
|---|---|---|---|
| **Mundo falso** (`test/mundo_falso.py`) | Cinemática + cámaras y brazo simulados, sin gráficos, ~0.3 s por corrida | Probar la lógica antes del robot (B5, B6). Es lo que evita romper cosas | **Obligatorio** |
| **MuJoCo** (`CARRO_BRAZO_NEURO/mision`: `crear_escena.py`, `robot_sim.py`, `correr_sim.py`) | Física con el modelo del robot, ya hace 3/3 con posiciones fijas | **Video para la presentación** y respaldo si el robot real falla | Si sobra tiempo |

Para MuJoCo: escena con frutas y cajas al azar; que `correr_sim.py` use la **misma** `mision_logica.Mision` (con
detecciones sacadas de las posiciones reales más ruido, sin cámara simulada) en vez de su `mision.py` viejo; grabar
un video de la misión completa con cada fruta.

### B9. Integración en el robot real (por etapas, de la más corta a la completa)

Cada etapa 3 veces seguidas sin tocar nada antes de pasar a la siguiente:

1. `voz` → solo publica el pedido.
2. `reconocer` → gira, arma el mapa y lo imprime (comparar con cinta: error ≤ 10 cm).
3. `agarre` con cada fruta (limón, naranja, mandarina), con ascensor.
4. `hasta-agarrar` desde lejos, con la fruta pedida entre las otras dos.
5. `caja` → empieza con la fruta ya agarrada y la deja en su caja.
6. **completa** → de la voz a la caja, con frutas y cajas puestas por otra persona.

### B10. Presentación

- Explicación corta (máquina de estados, cómo reconoce frutas y cajas, cómo distingue fruta de foto). Sin acrónimos
  sin explicar.
- `mision/estado` en una pantalla del PC durante la demo (qué está haciendo el robot).
- Video de respaldo: la mejor corrida real completa grabada hoy y, si existe, el de MuJoCo.
- Lista del día (sección 10).

## 7. Orden y dependencias

```
B0 (medidas, cajas) ──────────────┬──────────────► B6 (usa B0.3) ──► B7 (usa B0.5) ──┐
B1 (mandarina) ───────────────────┤                                                  │
B2 (voz) ─► B3 (pedido por tópico)├─► B4 (ascensor) ─► B5 (reconocer) ───────────────┼─► B9 ─► B10
                                  └──────────────────────────────────────────────────┘
B8 (MuJoCo): en paralelo, solo si hay alguien libre
```

Prioridad (2026-11-03): **circuito completo primero**. B0.7 (prueba de la caja) > B4 (ascensor en el agarre) > B6 + B7
(la caja dentro de la misión) > B1 (mandarina) > B5 (reconocer solo; sin él, plan B de la sección 9) > B2 + B3 (voz)
> B8. Sin voz, el pedido se escribe a mano (plan B de la sección 9).

## 8. Cronograma (hoy martes 3 y mañana miércoles 4)

| Cuándo | Código | Robot / físico |
|---|---|---|
| Martes, tarde | B0.7 en el robot, B4 | B0.1 cajas, B0.4, B0.5 fotos, B0.6 homing |
| Martes, tarde-noche | B6 (caja dentro de la misión), B1 | B0.2, B0.3 medidas con las cajas hechas; B9.3-B9.5 |
| Martes, noche | B5, B2 + B3 si queda tiempo | B9.6; **grabar la mejor corrida** |
| **Corte martes 22:00** | Lo que no pase su prueba se queda fuera y entra el plan B | |
| Miércoles, antes de presentar | Nada de código nuevo; solo umbrales de color en el lugar | Batería cargada; 2 corridas completas de ensayo |

## 9. Planes B (de menos a más recortado)

| Si falla | Plan B |
|---|---|
| `RECONOCER` / `EXPLORAR` | Antes de empezar, el robot gira una vez en su sitio y se le marca a mano lo que vio; o se miden con cinta las posiciones y se pasan como parámetros (como hoy `zona_x`, `zona_y`) |
| Detección de cajas | Posición de cada caja medida con cinta → `IR_A_ENTREGA` + `DEJAR` de hoy (ya funciona), con la fruta arriba |
| Pregunta del final | Fijar `final:=caja` o `final:=baldosa` para pruebas; la voz solo elige fruta |
| Baldosa: detección, altura o contacto sin validar | Elegir caja; no bajar a ciegas. Medir B0.2 y aprobar primero la orden aislada `bajar_hasta_contacto` con la líder al interruptor |
| Juez de agarre de pinza nueva | No transportar por un resultado de posición. Calibrar región visual arriba; mientras tanto, prueba manual aislada con líder y fruto sostenido |
| Mandarina | Se demuestra con limón y naranja y se explica qué la confunde (tono parecido al limón) |
| Voz | Publicar `ros2 topic pub /mision/pedido std_msgs/msg/String '{data: limon}' -r 5 --times 10` |
| El robot real ese día | Video de la mejor corrida y simulación en MuJoCo |

## 10. Lista del día de la presentación

- [ ] Batería cargada (≥ 15.2 V en reposo; `INICIO` exige 14.6 V) y una de repuesto si hay.
- [ ] Pi encendida, en la red, servicio `wallens-mision` arriba, `preflight.py` en verde.
- [ ] Homing del tornillo hecho al iniciar el programa.
- [ ] Umbrales de color revisados con la luz del salón (`scripts/vision/camara.py evaluar`).
- [ ] Micrófono probado con la voz del profe si se puede (o de quien vaya a hablar).
- [ ] Piso liso, sin cables; nada suelto de menos de 16 cm aparte de frutas y cajas.
- [ ] Una persona junto al interruptor de la batería; `/parada` lista en una terminal.
- [ ] Videos de respaldo en el PC, sin depender de internet.

## 11. Riesgos conocidos

| Riesgo | Mitigación |
|---|---|
| Mandarina y limón con tono casi igual | Textura + diámetro real + forma (B1) |
| Fotos impresas confundidas con frutas reales | Tamaño contra la distancia al piso + rectángulo de la caja (B7) |
| Frutas y cajas invisibles para el sensor láser y el freno | Todo lo bajo se esquiva con el mapa de objetos de la cámara; velocidad ≤ 0.10 m/s |
| Las ruedas patinan al girar en el sitio | El rumbo sale de la odometría filtrada con la unidad de medición inercial; `BUSCAR_CAJA` corrige de cerca |
| El tornillo pierde la posición al cortar la energía | Homing obligatorio al iniciar (B4, B0.6) |
| Luz del salón distinta a la de las pruebas | Umbrales en `config/frutas.yaml` y revisarlos allá |
| Ruido del salón para la voz | Gramática de 3 palabras, confianza ≥ 0.7, repetir si no entiende, plan B por teclado |
| Batería que baja en medio de la demo | Cargar antes; `INICIO` no arranca por debajo de 14.6 V |
