"""Prueba del adaptador en ROS aislado, sin driver ni motores. Solo verificar-mision-local.sh."""
import math
import time
import rclpy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from mision_frutas import NodoMision

rclpy.init()
n = NodoMision()
try:
    n._paso()
    assert n.mision is None and n.esperar_pedido
    odom=Odometry()
    odom.pose.pose.orientation.w=1.0
    n._al_odom(odom)
    pub = n.create_publisher(String, "mision/pedido", 10)  # como ros2 topic pub, durabilidad volátil
    limite=time.monotonic()+3
    while n.mision is None and time.monotonic()<limite:
        pub.publish(String(data="limon"))
        rclpy.spin_once(n,timeout_sec=0.05)
    assert n.mision is not None and n.mision.p.frutas==["limon"]
    assert n.mision.cfg.automatico and n.mision.cfg.reconocer
    n._al_odom(odom)  # el flujo real sigue llegando después de crear la misión
    scan=LaserScan()
    scan.angle_min=-math.pi/2-0.02
    scan.angle_increment=0.01
    scan.range_min=0.06
    scan.range_max=10.0
    scan.ranges=[0.6]*5
    for i in range(5):
        scan.header.stamp.sec=i+1
        n._al_scan(scan)
        if i<4:
            assert n.mision.lidar_atras is None
    assert abs(n.mision.lidar_atras-0.226)<0.0001
    previo=n.mision.lidar_atras
    scan.ranges=[0.9]*5
    n._al_scan(scan)  # mismo sello: no cuenta como otra vuelta
    assert n.mision.lidar_atras==previo
    # Candidata turquesa + pared alta: el adaptador traduce puntos y la lógica obtiene el frente.
    n.camara='cajas'
    scan.angle_min=math.pi/2-0.1
    scan.ranges=[1.076/math.cos(-.1+i*.01) for i in range(21)]
    for i in range(3):
        scan.header.stamp.sec=10+i
        n._al_scan(scan)
        n._al_objetos('caja',String(data='{"cajas":[{"tipo":"","angulo":0,"semiancho_angular":0.11}]}'))
    candidatas=n.mision.mapa_objetos.buscar('caja')
    assert len(candidatas)==1 and abs(candidatas[0].x-1.0)<.001
    # Dos frutas visibles: la llevada no sustituye a la central de la baldosa.
    n.camara='garra'
    n.mision.estado='ACERCAR_BALDOSA'
    n.mision.destino_baldosa=(-.23,.08)
    n.mision.cfg.agarre_arriba_x,n.mision.cfg.agarre_arriba_y=-.17,0.0
    n.mision.confirmaciones_baldosa,n.mision.t_baldosa_vista=0,-math.inf
    n._al_garra(String(data='{"candidatas":[{"x":-0.17,"y":0},{"x":-0.23,"y":0.08}]}'))
    assert n.mision.confirmaciones_baldosa==1
    n._al_pedido(String(data="naranja"))
    assert n.mision.p.frutas==["limon"]
    n.mision.final=("fin","prueba sin motores")
    n.mision.estado="FIN"
    n._paso()
    assert n.mision is None and not n.salir
    print("Adaptador ROS: pedido, láser del frente + candidata, dos frutas en garra y vuelta a esperar: OK.")
finally:
    n.destroy_node()
    rclpy.shutdown()
