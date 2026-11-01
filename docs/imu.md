# IMU BNO055

I²C por software (i2c-gpio), bus 1 (pines 3 SDA, 5 SCL, 3,3 V, GND), dirección `0x28`,
modo IMUPLUS (acelerómetro + giroscopio, sin magnetómetro). Driver: `ros/wallens_imu` (C++, propio).

## Uso

El servicio `wallens` lo lanza con todo lo demás (`launch/robot.launch.py`). A mano:

```
ros2 launch ~/wallens/launch/imu.launch.py
ros2 topic echo /bno055/imu --once
```

Topics (prefijo `bno055/`): `imu` (orientación + velocidad angular + aceleración lineal, 30 Hz) y `calib_status`
(JSON con sys/gyro/accel/mag, 1 Hz). Parámetros en `ros/wallens_imu/config/wallens_imu.yaml`; el montaje boca abajo
es `placement_axis_remap: "P6"` (se midió con `scripts/odometria/imu-ejes.py`, que usaba el driver anterior).

**Por qué un driver propio.** El bus por software gasta CPU del kernel por cada byte (espera bit a bit). El paquete
Python `ros-jazzy-bno055` leía 45 bytes por ciclo a 50 Hz y dejaba su nodo al ~85 % de CPU; el EKF dejó de cumplir su
ritmo. `wallens_imu` lee un solo bloque de 26 bytes (giroscopio, Euler, cuaternión, aceleración lineal) a 30 Hz: ~27 %.
Descarta bloques con el cuaternión sin norma 1 (la lectura corrupta de 0xFF) y reconfigura el chip si pierde el modo.
El lanzamiento lo fija al núcleo 3 (`taskset -c 3`) y el servicio deja el resto en los núcleos 0-2.
`config/bno055.yaml` queda para volver al paquete Python (`ros2 run bno055 bno055 --ros-args --params-file ...`).

Acelerómetro sin calibrar a propósito: en IMUPLUS el yaw sale del giroscopio, que llega solo a 3. El chequeo de
arranque (`scripts/chequeo-arranque.py`) exige solo el giroscopio.

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
`scripts/odometria/imu-directo.py` repite esa medición leyendo el chip sin ROS (parar antes `wallens_imu`, que va dentro del servicio: `sudo systemctl stop wallens`).

`scripts/prueba-i2c-gpio.sh` carga el bus por software solo en ejecución (sin tocar `config.txt`; reiniciar lo deshace).

El reinicio de la Pi resetea el BNO055 (modo CONFIGMODE, calibración a 0 salvo el giroscopio): el driver le fija el
modo al arrancar.

## Diagnóstico

```
i2cdetect -y 1          # debe verse 28
i2cget -y 1 0x28 0x00   # debe dar 0xa0
```

Ignorar los buses 20 y 21: son los de HDMI y muestran direcciones falsas.
