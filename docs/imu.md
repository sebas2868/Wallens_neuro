# IMU BNO055

I²C por software (i2c-gpio), bus 1 (pines 3 SDA, 5 SCL, 3,3 V, GND), dirección `0x28`,
modo IMUPLUS (acelerómetro + giroscopio). Driver: `ros-jazzy-bno055` (apt).

## Uso

```
ros2 launch ~/wallens/launch/imu.launch.py
ros2 topic echo /bno055/imu --once
```

Topics principales (prefijo `bno055/`): `imu` (orientación + velocidad angular
+ aceleración), `imu_raw`, `mag`, `temp`, `calib_status`. Parámetros en
`config/bno055.yaml` (50 Hz; orientación de montaje en `placement_axis_remap`,
ajustar según cómo quede fijado al robot).

Calibración: tras encender, mover el sensor en ocho y dejarlo quieto en
varias posiciones hasta que `calib_status` muestre todo en 3.

## Prueba de aceptación

En la Pi: `~/wallens/scripts/test-imu.sh` → `OK: /bno055/imu a ~50 Hz`.
Estabilidad: `~/wallens/scripts/estabilidad-imu.sh 30` (30 min, cuenta errores).

## Por qué un bus I²C por software

El BNO055 hace clock-stretching y el I²C hardware de la Pi lo maneja mal, a 100 kHz y también a 50 kHz: de vez en
cuando una lectura larga (el driver lee 45 bytes) llega entera como `0xFF` y sin error de I²C. El driver normaliza
el cuaternión `(-1,-1,-1,-1)` a `(-0.5,-0.5,-0.5,-0.5)`, lo que da picos de una sola muestra con yaw exactamente 90.0°
(medido: ~0.14 % de las lecturas, 13 fallos en 300 s con el hardware; las lecturas de 8 bytes no fallaron).

`sudo scripts/03-i2c.sh` apaga el I²C hardware y crea con `dtoverlay=i2c-gpio` un bus por software sobre los mismos
pines (GPIO 2 y 3, pines 3 y 5), que sigue siendo `/dev/i2c-1`; no cambia el cableado ni los `i2c_bus: 1`.
`scripts/03-i2c.sh --diff` muestra el cambio sin escribir; el script hace copia `config.txt.bak-<fecha>` y escritura
atómica, y deja el overlay al final porque los `dtparam` de la base deben ir antes de cualquier `dtoverlay=`.
Resultado medido: 0 fallos en 7600 ciclos (5 min) y 0 picos en ventanas de 30 s con el driver ROS y `wallens_power`.
`scripts/odometria/imu-directo.py` repite esa medición leyendo el chip sin ROS (parar el nodo `bno055` antes).

`scripts/prueba-i2c-gpio.sh` carga el bus por software solo en ejecución (sin tocar `config.txt`; reiniciar lo deshace).

El reinicio de la Pi resetea el BNO055 (modo CONFIGMODE, calibración a 0 salvo el giroscopio): el driver ROS le fija el
modo al arrancar, así que hay que recalibrar el acelerómetro tras cada reinicio.

## Diagnóstico

```
i2cdetect -y 1          # debe verse 28
i2cget -y 1 0x28 0x00   # debe dar 0xa0
```

Ignorar los buses 20 y 21: son los de HDMI y muestran direcciones falsas.
