# Actualización del 4 de noviembre: medidas, pinza nueva y búsqueda de cajas

Cambios locales, **sin desplegar ni mover motores**. Se conservaron los ajustes de la líder para ruedas
(0.0360 / 0.2647 / 1.000) y giroscopio (0.971), el homing sin abrir la pinza y el sostén de mandarina 200.
La dirección de la Pi comunicada hoy es `wallens@11.11.41.26`.

## Qué cambió

- Configuración: marco 0.072 m; diámetros limón 0.050, naranja 0.070 y mandarina 0.074 m.
  Se usan las medidas nuevas también en el mapa y en las comprobaciones de geometría.
- Pinza: altura de agarre 3200 ticks. Sin cero o por debajo de esa altura se rechazan abrir, cerrar y soltar.
  Homing y rearme no mueven los dedos. La misión calibra, SUBE y después cierra para empezar.
  Todas las órdenes de movimiento del brazo exigen ruedas quietas.
- `cerrar:<fruta>` limita la corriente al sostén medido (200 para las tres), sin el aumento anterior de 25 %.
  Devuelve `por_verificar`, también en vacío. `brazo/estado.objeto=true` es conservador: **puede llevar fruta**,
  no una certificación de agarre. `recoger` también termina como `por_verificar`.
- La misión exige tres vistas entre los dedos, sube y espera tres vistas nuevas en una región visual ARRIBA
  distinta de la del piso. **`agarre_arriba_x/y` siguen POR MEDIR**. Sin ellas falla quieta y conserva el sostén;
  no entrega una fruta basándose en el juez de posición que falló hoy. Si arriba no se ve la fruta o las regiones
  se superponen, esta forma de verificar no sirve y hay que resolverlo antes de autorizar transporte automático.
- `garra/fruta` conserva sus campos anteriores y añade `candidatas`. La lógica selecciona la fruta subida al
  verificar y la central al acercarse a la baldosa, excluyendo la región de la fruta transportada.
- Caja real: frente 3.3 cm, fruta arriba 4.5–4.8 cm. La misión verifica la holgura y repite SUBIR hasta `listo`
  antes de la reversa. Mantiene 20.5 → 4.7 cm, freno existente y apertura parcial de 400 ticks.
  No se recorta la caja: si la prueba física muestra roce, la líder propone frente de ~2.5 cm.

## Buscar cajas en dos etapas

El detector conserva la lectura de la esquina negra; no clasifica el dibujo impreso por color. El marco fino
se cortaba con el desenfoque y la resta de 25: se añadió una lectura sin desenfoque con resta de 5, manteniendo
los criterios de cuadrilátero, hueco blanco y esquina mordida.

En modo `cajas` publica además candidatos turquesa (`tipo: ""`, columna, ángulo y semiancho angular), con los
umbrales medidos de `cajas.py`: H 84–104, S≥60, V≥70, área≥1200, más ancho que alto. No inventa distancia visual:
la lógica cruza ese ángulo con puntos recientes del láser, exige una pared coherente y obtiene su normal.
El láser ve la pared alta; se desplaza 16.5 cm hacia el frente para guardar la caja en el mapa.

`BUSCAR_CAJA` barre en pasos de 40°, solo modo cajas. Visita la candidata más cercana no revisada, a unos 0.7 m
de la tarjeta. En `LEER_CAJA`, quieto, exige tres lecturas coherentes:

- Pedida: sigue a `ACERCAR_CAJA`, giro y entrada.
- Otra: conserva tipo y revisión; busca la siguiente.
- Sin lectura en 3 s: se acerca 10 cm una sola vez. Si tampoco lee, marca `sin_leer` y continúa.
- Sin candidatas pendientes: explora hasta cuatro puntos y vuelve a barrer. No vuelve a visitar una caja sin leer.

Las posiciones medidas de caja siguen disponibles como plan B. Una candidata sin láser reciente no entra al mapa.
Las cajas deben estar derechas; las fotos inclinadas con reflejo no se corrigen inventando un tipo.

## Alturas por fruta y órdenes de contacto

Fuente: `registros/cajas-distancias-20261104/alturas-frutas-20261104-091955.json`. Se guardan los dos puntos de cada
fruta en `config/mision_automatica.yaml`. No se promedian: el fondo de la fruta depende de cómo la toma la pinza.
Los 40 ticks del modelo viejo ahora son un parámetro de referencia, **no** la altura de agarre.

| Fruta | Punto bajo (ticks; cm) | Punto alto (ticks; cm) | Ventana 2.5±0.5 cm (ticks) | Subida corta de 1 cm |
|---|---|---|---|---|
| Limón | 8484; 0.1 | 36036; 4.8 | 19622–25484 | 5862 |
| Naranja | 5490; 0.1 | 36037; 4.5 | 18681–25623 | 6942 |
| Mandarina | 4992; 0.1 | 36036; 4.7 | 17815–24563 | 6749 |

Para recalcular, por ejemplo limón, sin mover nada:

```bash
python3 ~/wallens/scripts/parametros-baldosa.py --abajo 0.1 --ticks-abajo 8484 --arriba 4.8 --ticks-arriba 36036
```

## Qué probar ahora en el robot

La líder ejecuta cada orden con la mano en el interruptor, mirando `brazo/resultado` antes de la siguiente.
Desplegar solo después de revisar el árbol compartido: `scripts/desplegar.sh` copia todo y puede reiniciar servicios.
Esta sesión no ejecutó ese despliegue.

Primero obtener la observación que falta para el agarre, con misión apagada y detector del servicio disponible.
En la Pi:

```bash
source ~/wallens/scripts/ros-env.sh
ros2 topic pub /camara std_msgs/msg/String '{data: garra}' --qos-durability transient_local -r 5 --times 10
ros2 topic echo /garra/fruta
```

En otra terminal, observar `ros2 topic echo /brazo/resultado`. En una tercera, las siguientes órdenes se ejecutan
**una por una**, esperando su resultado; no pegar toda la secuencia para que corra sin revisión:

```bash
ros2 topic pub --once /brazo/orden std_msgs/msg/String '{data: calibrar}'
ros2 topic pub --once /brazo/orden std_msgs/msg/String '{data: subir}'
ros2 topic pub --once /brazo/orden std_msgs/msg/String '{data: abrir}'
ros2 topic pub --once /brazo/orden std_msgs/msg/String '{data: bajar}'
```

Comprobar que `bajar` deja pasar la hoja bajo los dedos. Colocar el limón en la zona de agarre sin mover ruedas.
Registrar las candidatas de cámara con la fruta suelta en el piso. Después:

```bash
ros2 topic pub --once /brazo/orden std_msgs/msg/String '{data: "cerrar:limon"}'
ros2 topic pub --once /brazo/orden std_msgs/msg/String '{data: subir}'
```

`por_verificar` es la respuesta esperada, no `sin_agarre`. Con la fruta realmente levantada y sostenida 30 s,
registrar las candidatas de cámara. Repetir cinco agarres; comprobar con las otras frutas que la región visual
es válida para ellas antes de configurarla. No aceptar una región que incluya la fruta que quedó en el piso.

Después de aprobar esa verificación:

1. **Contacto aislado** con fruta arriba y baldosa debajo: para limón,
   `ros2 topic pub --once /brazo/orden std_msgs/msg/String '{data: "bajar_hasta_contacto:19622:25484"}'`.
   Contacto → deja de empujar; sin baldosa → `sin_contacto` en el límite 19622, sin llegar al fondo;
   `/parada` a mitad → se detiene.
   Corriente de búsqueda actual 90, velocidad ≤40. Luego ocho `soltar:50`, esperando cada resultado, y
   `subir_corto:5862`; comprobar que no arrastra ni toca la fruta central. Usar las otras ventanas según la fruta.
2. **Caja de frente**, primero con posición medida y las alturas cargadas:

   ```bash
   ros2 run wallens_mision mision_frutas.py --ros-args \
     --params-file ~/wallens/config/mision_automatica.yaml \
     -p esperar_pedido:=false -p reconocer:=false -p desde:=PREGUNTAR_FINAL -p final:=caja \
     -p caja_x:=X -p caja_y:=Y -p caja_rumbo:=RUMBO
   ```

   X/Y son el centro del frente en odometría y RUMBO su normal hacia afuera. Fruta agarrada y arriba antes de lanzar.
   Mirar subida completa antes de reversa, holgura sobre 3.3 cm, distancias y apertura parcial sin tocar paredes.
3. **Visita de cajas**: dos cajas derechas, primero otra fruta más cerca y después la pedida. Debe revisar y saltar
   la equivocada. Tapar una esquina: esperar 3 s, un avance de 10 cm, otros 3 s y descarte; no repetir indefinidamente.
   Empezar con cajas entre 0.5 y 1.5 m; validar aparte el extremo de 2 m donde faltaron cuadros de turquesa.
   Para probar solo este tramo, con la líder confirmando a mano que el limón está sostenido arriba:

   ```bash
   ros2 run wallens_mision mision_frutas.py --ros-args \
     --params-file ~/wallens/config/mision_automatica.yaml \
     -p esperar_pedido:=false -p reconocer:=true -p desde:=PREGUNTAR_FINAL -p final:=caja -p frutas:=limon
   ```

4. **Misión completa** con caja y con baldosa; reordenar objetos, comprobar rutas y siguiente pedido.
5. **Voz**: completar instalador interactivo con sudo y 15 pedidos con ruido (≥13 correctos, cero equivocados),
   como en [pruebas-manana.md](pruebas-manana.md).

## Evidencia local

- Antes: misión 100 pruebas, cinco tandas omitidas por defecto; brazo 139, con un fallo porque el test antiguo
  exigía abrir durante homing. Después: **114 pruebas de misión con seis tandas, OK en 50.433 s; 145 de brazo, OK**.
- **1800 escenarios**: 300 ascensor, 300 caja por parámetro, 300 baldosa, 300 reconocimiento, 300 misión completa
  y 300 visitas a cajas (otra fruta o ilegible primero). Cero choques y destinos equivocados en esos escenarios.
  Las misiones completas ya simulan cierre incierto, región visual arriba sintética y tarjetas legibles solo de cerca.
  Esto no valida la región visual real ni todas las disposiciones posibles del espacio.
- Binario C++ con capturas de hoy: 31/31 de cada tarjeta a 0.5 m; desde más lejos se mantienen como turquesa sin
  tipo. Localiza 364/372 cuadros muestreados entre 0.5 y 2 m: la excepción es mandarina a 2 m (23/31).
  Las ocho carpetas cercanas de `mandarina-diagnostico/pasos*` dan 495/496 lecturas correctas.
  Cero tipos equivocados y cero falsas frutas en las capturas evaluadas; no se retocó el color del dibujo impreso.
- La compilación y evaluación se repiten sin red ni dispositivos con `bash scripts/verificar-mision-local.sh`.
  Cierre: dos paquetes compilados en 16.4 s; 10 pruebas de cámara, salida de dos candidatas del detector y adaptador
  de mensajes con láser del frente: OK. Evaluación de **1237 cuadros**, cero tipos equivocados y cero falsas frutas.
  Las pruebas de voz, contacto, nuevas regiones visuales y holgura siguen siendo físicas.

Archivos: lógica y adaptador de misión, mapa, detector y geometría visual; configuraciones de frutas y misión;
brazo/control/nodo y sus pruebas; mundo falso, pruebas de misión y `test_actualizacion_04.py`; cálculo de ventana,
evaluación de capturas, verificador local, README y planes. No se tocó la parte neuronal ni se hicieron commits/push.
