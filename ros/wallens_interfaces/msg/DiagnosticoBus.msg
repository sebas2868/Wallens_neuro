# Estado del bus Dynamixel compartido (lo publica el driver a 1 Hz, solo con brazo_habilitado)
uint64 transacciones          # llamadas del brazo ejecutadas
uint64 rechazadas             # llamadas rechazadas sin tocar el bus (presupuesto, cola llena, bus apagado...)
uint32 en_cola
float64 tiempo_bus_ms_total   # tiempo de bus gastado por el brazo
float32 tiempo_bus_ms_max     # la llamada más larga
uint64 ciclos_atrasados       # ciclos de las ruedas con más de 1.5 periodos entre lecturas
uint64 ciclos_tardios         # ciclos con más de 3 periodos (acercan el watchdog de 200 ms)
