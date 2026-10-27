# IMU BNO055

I²C hardware, bus 1 (pines 3 SDA, 5 SCL, 3,3 V, GND), dirección `0x28`,
modo NDOF (fusión). Driver: `ros-jazzy-bno055` (apt).

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

## Por qué 50 kHz

El BNO055 hace clock-stretching y el I²C hardware de la Pi lo maneja mal a
100 kHz. `sudo scripts/03-i2c.sh` fija `i2c_arm_baudrate=50000` en
`/boot/firmware/config.txt` (reiniciar). Si la prueba de estabilidad falla,
la alternativa es un bus por software con el overlay `i2c-gpio`.
Resultado medido: 30 min a 50 Hz sin errores, no hizo falta.

## Diagnóstico

```
i2cdetect -y 1          # debe verse 28
i2cget -y 1 0x28 0x00   # debe dar 0xa0
```

Ignorar los buses 20 y 21: son los de HDMI y muestran direcciones falsas.
