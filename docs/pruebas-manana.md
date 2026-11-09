# Pruebas del miércoles 4 de noviembre

**Actualización del 4 nov:** leer primero [resultados y comandos corregidos](resultados-20261104.md).
Las cuatro medidas ya están incorporadas; falta distinguir por cámara la fruta subida de una que quedó en el piso.
El cierre nuevo responde `por_verificar`; la posición de los dedos no certifica agarre.

El código de misión está integrado y compilado en ROS 2 Jazzy. Los resultados del mundo falso **no certifican el
robot real**: las distancias de cámara, la fuerza de la pinza nueva y el contacto sobre baldosa se validan abajo.
La líder hace cualquier prueba con movimiento, con la mano en el interruptor. No correr las etapas juntas en un bucle.

## 1. Antes de desplegar: pinza nueva y cuatro medidas

Abierto manual 2866, cerrado manual 3811. La prueba del 3 nov 20:16 confirmó sostén 200 para las tres frutas.
No reutilizar un cierre antiguo de 3890 ni certificar agarre por posición. La apertura parcial de 400 ticks
todavía debe comprobarse dentro de la caja. La altura segura de pinza es 3200 ticks; homing y rearme no abren.

Medidas YA INCORPORADAS (4 nov):

- Dos alturas con sus ticks POR FRUTA en `config/mision_automatica.yaml`; arriba 4.5–4.8 cm. No promediar.
- Marco negro exterior 0.072 m, en `config/frutas.yaml`.
- Diámetros limón 0.050, naranja 0.070, mandarina 0.074 m, en ese mismo archivo.
- Sostén de mandarina 200, también valor por defecto del nodo; limón y naranja usan 200.
- Frente turquesa real 3.3 cm: exigir subida completa antes de entrar de reversa.

Falta medir `agarre_arriba_x/y` con cam_garra, en una región distinta del piso. Sin ella la misión falla quieta
tras subir, sosteniendo la posible fruta; no transporta. El reporte del 4 nov explica la prueba aislada.

## 2. Desplegar y preparar terminales

Desde el PC, con la otra sesión avisada y el árbol compartido revisado (el despliegue copia todo):

```bash
cd /home/ale/NEUROCONTROL/PARCIAL2/Wallens_neuro
scripts/desplegar.sh
ssh -t wallens@11.11.41.26
```

En cada terminal de la Pi:

```bash
source ~/wallens/scripts/ros-env.sh
```

Arrancar el servicio si hace falta. Esto deja el brazo y el único detector disponibles; todavía no lanza la misión:

```bash
sudo systemctl start wallens-mision
ros2 topic echo /brazo/resultado
```

En otra terminal, observar `ros2 topic echo /mision/estado`. El dominio es 37. Para detener una misión además del
interruptor físico: `ros2 topic pub /parada std_msgs/msg/Bool '{data: true}' -r 5 --times 10`.

## 3. Visión, sin mover motores

Sin el proceso `mision_frutas.py` ejecutándose, poner cada caja de frente a la C270 a **0.5, 1 y 1.5 m**. Para cada
posición, en la Pi:

```bash
python3 ~/wallens/scripts/vision/camara.py grabar --camara=cajas --segundos=20
```

El comando imprime la carpeta de fotos; **no abre otra cámara**, pide fotos al detector y copia los cuadros crudos.
El servicio ya arranca con `depuracion:=true`. El frente turquesa REAL mide 3.3 cm y no tapa la tarjeta en las
fotos de hoy. Las cajas deben estar derechas para evitar reflejos. Desde lejos puede publicar `tipo: ""`:
es una candidata turquesa; el criterio de 95 % de lectura corresponde al acercamiento, a unos 0.7 m.

Evaluar la carpeta impresa (sustituir `CARPETA` y el tipo):

```bash
python3 ~/wallens/scripts/vision/camara.py evaluar CARPETA --modo=cajas --tipo=limon
```

Repetir con naranja y mandarina. Este comando usa el **binario C++ real**, sin cámara. Exige al menos 95 % correcto.
Repetir las capturas con `--camara=frutas` y evaluar con `--modo=frutas --tipo=limon`, `naranja`, `mandarina` o
`todas`. Para una escena sin destinos de un modo usar `--tipo=ninguna`: tarjetas solas no deben dar frutas y frutas
solas no deben dar cajas. Probar también las tres frutas al lado de las cajas.

Para baldosa: una fruta al centro y otra igual en el piso; grabar `--camara=baldosas` y evaluar
`--modo=baldosas --tipo=limon`. Quitar la baldosa y repetir con `--tipo=ninguna`. Verificar con cinta las coordenadas
publicadas en `/baldosas/detecciones`; la distancia se calcula por diámetro, no proyectando la fruta al piso.

Las fotos antiguas del repositorio dieron las tres tarjetas correctas. No sustituyen estas nueve distancias ni la
prueba desde la cámara baja. Los umbrales de fruta vienen de las referencias medidas de `voz_fruta_pi4/fruta.py`.

## 4. Agarre y ascensor: cinco intentos con limón

Desde el PC, una vez por intento:

```bash
cd /home/ale/NEUROCONTROL/PARCIAL2/Wallens_neuro
scripts/probar.sh agarre
```

Limón a 33 cm detrás del eje. Primero aprobar la región visual como explica el reporte del 4 nov. Mirar: calibra
si falta cero (hasta ~65 s, límite 90 s), sube ANTES de accionar pinza, baja a 3200, abre, retrocede, cierra y sube.
Las ruedas no se mueven durante órdenes del brazo; un cierre incierto no permite salir sin verificación arriba.
Al terminar sostiene arriba **30 s** sin sobrecarga ni caída. Retirar la fruta antes de repetir.

## 5. Contacto de baldosa: primero solo el brazo

Con las alturas medidas, calcular las órdenes **sin mover nada**, desde la Pi:

```bash
python3 ~/wallens/scripts/parametros-baldosa.py --abajo 0.1 --ticks-abajo 8484 --arriba 4.8 --ticks-arriba 36036
```

Este ejemplo es LIMÓN; las otras ventanas están en el reporte del 4 nov.
Imprime `bajar_hasta_contacto:MIN:MAX` y `subir_corto:TICKS`.
MIN y MAX son alturas en ticks **sobre el cero inferior**, no posiciones absolutas del motor.

Con ruedas quietas, fruta agarrada arriba y baldosa colocada debajo, copiar la primera orden calculada:

```bash
ros2 topic pub --once /brazo/orden std_msgs/msg/String '{data: "bajar_hasta_contacto:MIN:MAX"}'
```

Las órdenes del brazo se publican una sola vez; observar la respuesta antes de otra orden. Debe descender a velocidad
≤40 y corriente limitada a la del homing actual (90), detectar `contacto` dentro de la ventana y dejar de empujar.
Con baldosa retirada debe devolver `sin_contacto` y parar antes del tope inferior. Probar `/parada` a mitad:
debe congelar el movimiento, sin continuar bajando. Si falla cualquiera, no ejecutar el final automático baldosa.

Tras contacto correcto, probar por separado cada `soltar:50` hasta ocho pasos, esperar `listo` tras cada uno;
después la orden `subir_corto:TICKS` calculada. Mirar que no arrastre la fruta, que no toque la central y que los
dedos no se apoyen en la baldosa. El modelo falso no demuestra esa holgura vertical de la pinza nueva.

## 6. Entregas completas y reconocimiento

Primero probar caja con coordenadas medidas. Con fruta agarrada y arriba, usar en la Pi:

```bash
ros2 run wallens_mision mision_frutas.py --ros-args \
  --params-file ~/wallens/config/mision_automatica.yaml \
  -p esperar_pedido:=false -p automatico:=true -p reconocer:=false \
  -p desde:=PREGUNTAR_FINAL -p final:=caja \
  -p caja_x:=X -p caja_y:=Y -p caja_rumbo:=RUMBO
```

X/Y son el **centro del frente bajo** en el marco de odometría; RUMBO es su normal hacia afuera, en radianes.
Consultar `/odometry/filtered` y medir con cinta: esos tres parámetros no se transforman con `relativo`.
Debe acercarse por el frente, girar, entrar de reversa hasta 20.5 cm y después 4.7 cm desde los dedos a la pared alta,
abrir 400 ticks, avanzar y solo entonces cerrar y bajar. Confirmar holgura de la pinza nueva.

Después de aprobar visión y ambas mecánicas, lanzar la misión completa en la Pi:

```bash
ros2 run wallens_mision mision_frutas.py --ros-args \
  --params-file ~/wallens/config/mision_automatica.yaml
```

En otra terminal:

```bash
ros2 topic pub /mision/pedido std_msgs/msg/String '{data: limon}' -r 5 --times 10
```

Debe reconocer con giros de 40° y pausas, recoger la fruta del piso, subir y preguntar **«¿caja o baldosa?»**.
Responder solo después de ver la pregunta:

```bash
ros2 topic pub /mision/final std_msgs/msg/String '{data: caja}' -r 5 --times 10
```

Repetir con `baldosa`. Comprobar que una respuesta inválida repite la pregunta y 120 s sin respuesta producen fallo
sosteniendo la fruta. Al terminar bien debe publicar `ESPERAR_PEDIDO` y aceptar otro pedido; durante una misión no
debe cambiar de fruta por otro mensaje. Cambiar las posiciones entre intentos y comprobar las rutas con objetos en
la recta. No se integró Nav2; el mapa de objetos alimenta la navegación local y el freno láser sigue activo.

## 7. Voz

El paquete fuente, modelo y muestras quedaron copiados en `~/voz_fruta_pi4` de la Pi. El instalador pidió contraseña
de `sudo` y no se pudo completar remotamente sin ella. En una terminal interactiva de la Pi:

```bash
bash ~/voz_fruta_pi4/instalar.sh
arecord -l
source ~/wallens/scripts/ros-env.sh
~/voz_fruta_pi4/venv/bin/python ~/wallens/ros/wallens_mision/scripts/voz_pedido.py \
  --ros-args -p microfono:=plughw:CARD=U0x46d0x825,DEV=0
```

La C270 se detectó como tarjeta `U0x46d0x825`, dispositivo 0; confirmar con `arecord -l` por si cambia.
Hacer **15 pedidos con ruido: al menos 13 correctos, cero frutas equivocadas**. El nodo solo publica cuando la
misión espera pedido. Rechaza confianza inferior a 0.7, frases con dos frutas y palabras desconocidas. La respuesta
del final sigue siendo por tópico, nunca por voz. Para probar solo voz sin mover motores, dejar la misión apagada,
observar `/mision/pedido` y publicar `ESPERAR_PEDIDO` en `/mision/estado` antes de cada frase.

## Evidencia local y límites

Los números siguientes son históricos (cierre del 3 nov). El cierre actualizado está en
[resultados-20261104.md](resultados-20261104.md).

- Misión: 100 tests con las cinco tandas de cierre, `OK` (34.207 s): ascensor 300/300; caja parametrizada 300/300;
  baldosa parametrizada 300/300; reconocimiento 300/300; misión completa 300/300 (150 caja + 150 baldosa).
- Cero choques y cero destinos equivocados en esos escenarios; las misiones completas usan tres sectores libres
  dentro de 3 × 3 m y tarjetas mirando al espacio accesible. No representan todas las disposiciones posibles.
- Brazo: 139 tests, `OK`, incluyendo contacto, ausencia de contacto, aborto, mandarina sin medida y cero encajes; incluye cambios
  concurrentes de la otra sesión. La ventana de altura usada por tests es sintética, no está configurada en el robot.
- Compilación local de `wallens_interfaces` y `wallens_mision`, tarjetas reales 3/3 y prueba del adaptador ROS: `OK`.
  Falta compilar en la Pi al desplegar. `bash scripts/verificar-mision-local.sh` repite la verificación local sin red.
- No se desplegó el árbol Wallens, no se reiniciaron servicios, no se movieron motores ni se hicieron commits/push.
- La prueba física de visión a las tres distancias, la pinza nueva, las alturas y los pedidos con ruido siguen
  pendientes. El programa rechaza los datos imprescindibles que todavía están POR MEDIR.

## Archivos de esta integración

- Misión: `ros/wallens_mision/scripts/mision_logica.py`, `mision_frutas.py`, nuevos `mapa_objetos.py` y
  `voz_pedido.py`; `ros/wallens_mision/CMakeLists.txt`.
- Detector: `ros/wallens_mision/src/detector_frutas.cpp`, nuevo `vision_destinos.hpp`, `config/frutas.yaml` y
  `launch/mision.launch.py`. Prototipo preservado en `scripts/vision/marcador_caja.py`.
- Brazo: cambios puntuales en `scripts/brazo/brazo.py`, `control.py`, `falso.py`, `nodo.py` y `test_control.py`.
  Se preservaron las ediciones concurrentes de pinza nueva y sus tests.
- Pruebas: `ros/wallens_mision/test/mundo_falso.py`, `test_mision.py`, nuevos `test_finales.py`, `test_mapa.py`,
  `test_vision_destinos.cpp` y `comprobar_adaptador_ros.py`; `scripts/verificar-mision-local.sh`.
- Uso: `scripts/vision/camara.py`, comentarios y selección de modo en `scripts/probar-mision.sh`, nuevo
  `scripts/parametros-baldosa.py`, `config/mision_automatica.yaml`, README, plan maestro, plan de misión y esta guía.

No son de esta integración los cambios existentes en ruedas, freno del sensor láser, calibración del tornillo,
`jog.py`, pruebas de pinza ni documentación mecánica de la otra sesión. No se modificó la parte neuronal.
