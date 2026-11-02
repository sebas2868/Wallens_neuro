# Una operación sobre el bus Dynamixel del brazo (IDs 3 y 5)
uint8 LEER = 0
uint8 ESCRIBIR = 1
uint8 PING = 2

uint8 tipo        # LEER, ESCRIBIR o PING
uint8 id
uint16 direccion
uint8 largo       # 1, 2 o 4; en lecturas en bloque, hasta 32
uint32 valor      # solo en escrituras
