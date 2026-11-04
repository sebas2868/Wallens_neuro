# Plan: mejorar el reconocimiento de frutas

**Problema (2026-11-04):** en el laboratorio el detector (`ros/wallens_mision/src/detector_frutas.cpp`) casi no ve
las frutas. Se calibró en otro lugar con otra luz. Además ahora tiene que distinguir **limón, naranja y mandarina**
(antes solo limón y naranja).

**Cómo funciona hoy:** un filtro HSV (H 10-75, `s_min` 60, `v_min` 20) arma las manchas; los filtros de forma y tamaño
(circularidad, `tam_min/max`, aspecto, horizonte) descartan lo que no parece fruta, y el tono mediano decide el tipo
(H < `h_corte` = 32 → naranja, si no limón). Detectar y clasificar ocurren en el mismo paso y con el mismo umbral.

**Idea central:** separarlo en dos.

- **A. Detectar**: encontrar todo lo redondo y del tamaño de una fruta en el suelo, sin importar qué fruta es.
  Un filtro de color amplio, sobre todo forma. Así el robot sabe cuántas frutas hay y dónde están.
- **B. Clasificar**: decidir qué fruta es cuando el robot está cerca (al final de ACERCAR o con cam_garra). Ahí la
  fruta ocupa muchos más píxeles y la luz influye menos.

Cada parte va de lo más simple a lo más complicado. Se avanza al paso siguiente solo si el anterior no alcanza.

## 0. Diagnóstico (antes de cambiar nada)

- Correr el detector con `depuracion:=true` en el laboratorio, con las dos cámaras.
- Guardar también la **máscara** y anotar por qué se descarta cada mancha (color, forma, tamaño, horizonte).
- Con eso se sabe si la fruta no pasa el filtro de color o si pasa y la tira otro filtro.
- Mirar cómo es el piso (claro, brillante, beige o de madera): influye en los pasos A2 y A3.

## A. Detectar frutas (redondos)

1. **Fijar la cámara**: apagar el balance de blancos y la exposición automáticos (`v4l2-ctl`) y usar valores fijos
   para el laboratorio, para que el color no cambie cuadro a cuadro.
2. **Máscara amplia de "fruta"**: medir en el laboratorio el HSV de las tres frutas y abrir el rango para que entren
   todas (amarillo-verde a naranja-rojizo). Los valores van en un archivo de parámetros por lugar (casa / laboratorio),
   no fijos en el código.
3. **Revisar los filtros de forma y tamaño**: aflojar `circularidad_min`, `tam_min/max`, `area_min` y `aspecto_max`
   si el diagnóstico muestra que tiran frutas buenas. El tamaño aceptado tiene que cubrir de la mandarina a la naranja.
4. **Compensar la luz**: balance de blancos por software (gray-world o una cartulina blanca de referencia) o CLAHE
   sobre el brillo.
5. **Círculos sin depender del color**: si el color sigue fallando, buscar redondos por bordes (HoughCircles o
   contornos de Canny) y usar el color solo para descartar lo que claramente no es fruta (piso, paredes).
   Cuidado: sin color salen muchos falsos redondos (ruedas, enchufes, sombras).

**Resultado de A:** `frutas/detecciones` publica candidatas con `tipo: "fruta"` (o un tipo provisional) y su posición.

## B. Clasificar qué fruta es

1. **Clasificar cerca**: la misión se acerca a la candidata y clasifica cuando está a menos de ~0.5 m (cam_frutas)
   o con cam_garra. Si no es la fruta pedida, la anota y pasa a la siguiente.
2. **Un rango de tono por fruta**: en vez de un solo `h_corte`, un rango para cada una (limón verde-amarillo, naranja,
   mandarina más rojiza), medido en el laboratorio. Si en HSV no se separan, probar el espacio Lab (a\*, b\*).
3. **Tamaño como último recurso para mandarina contra naranja**: si el color no las separa, usar el diámetro que ya
   estima el detector (la mandarina es más chica y más achatada).
4. **Clasificador ligero**: k-NN o SVM sobre el histograma de color de la mancha recortada, entrenado con fotos del
   laboratorio de las tres frutas.
5. **Red pequeña**: MobileNet o YOLO-nano en TFLite/ONNX en la Pi, solo si nada de lo anterior alcanza (cuidado con
   la CPU: el detector ya corre con un solo hilo de OpenCV).

## C. Validar al agarrarla (¿es una fruta real?)

Después de cerrar la pinza, confirmar con los motores (gripper ID 5, tornillo ID 3) que lo agarrado es una fruta real y
no una falsa (plástico, espuma).

1. **Diámetro al contacto**: la posición del gripper cuando toca la fruta (`contacto` en `scripts/brazo/brazo.py`) da
   el ancho real. Tiene que coincidir con la fruta que se clasificó (sirve también para confirmar mandarina contra naranja).
2. **Firmeza**: cuánto avanza el gripper desde el contacto hasta el tope con la corriente de apriete. Una fruta de
   plástico casi no cede; una de espuma cede de más; una real queda en medio. Medir el rango de cada fruta real.
3. **Peso por el torque del tornillo**: comparar la corriente al subir vacío y al subir con fruta. Es dudoso: el
   tornillo sin fin se sostiene solo y casi toda su corriente es fricción (~400 mA), así que una fruta de 80-250 g
   cambia poco la lectura. Además, hoy la misión agarra al ras del suelo sin subir. Probarlo en banco antes de contar
   con esto. El gripper cierra en horizontal, así que el peso no carga su eje.

Si la validación falla: soltar, marcar esa candidata como descartada y seguir con la siguiente.

## Lo que toca fuera del detector

- `mision_logica.py` hoy busca directamente la fruta pedida. Con A/B tiene que ir hacia candidatas sin tipo,
  clasificarlas de cerca y descartar las que no son la pedida. Esto cambia BUSCAR/ACERCAR y sus pruebas
  (`test/mundo_falso.py`, `test/test_mision.py`).
- La validación de C va en VERIFICAR, que hoy solo mira cam_garra. Necesita que el nodo del brazo publique el
  contacto y la firmeza del último agarre.
- `scripts/vision/camara.py` es el espejo en Python del detector: hay que mantenerlo igual.

## Cómo saber que mejoró

Unas 20-30 fotos del laboratorio, etiquetadas a mano, y un script que cuente cuántas frutas se detectan (A) y cuántas
se clasifican bien (B). Se corre antes y después de cada paso.
