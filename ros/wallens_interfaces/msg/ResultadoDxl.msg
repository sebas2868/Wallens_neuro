# Resultado de una OperacionDxl
uint8 OK = 0
uint8 NO_EJECUTADA = 1
uint8 RECHAZADA = 2        # fuera de la lista blanca
uint8 ERROR_COMUNICACION = 3
uint8 ERROR_MOTOR = 4

uint8 estado
int16 comunicacion         # código COMM del SDK (0, -1000..-9000: no cabe en int8)
uint8 error_motor          # byte de error del paquete de estado (bit 7 = alerta de hardware)
uint32 valor               # lectura de 1, 2 o 4 bytes
uint8[] datos              # lecturas en bloque (largo > 4)
