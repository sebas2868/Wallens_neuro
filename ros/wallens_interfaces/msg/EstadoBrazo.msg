# Estado del brazo (lo publica wallens_brazo a 5 Hz)
bool energizado
bool calibrado
bool congelado          # aborto suave: quieto con torque hasta llamar a brazo/soltar
bool objeto             # sostiene un objeto
bool ocupado            # ejecutando una orden
bool parada             # /parada activa
string fase
int32 pos_tornillo
int32 pos_pinza
int16 corriente_tornillo
int16 corriente_pinza
float32 voltaje         # V, del bloque de lectura de la pinza
uint8 temperatura_max   # °C, la mayor de los dos motores
