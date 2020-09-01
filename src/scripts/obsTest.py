#!/usr/bin/env python
#Se importan las librerias necesarias junto con los mensajes a utilizar
import rospy, math, roslaunch, time, sys
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped,PoseWithCovariance,Pose
import matplotlib.pyplot as plt
import networkx as nx



def punto2c():
    global posicionActual, g, ruta, pubMot, arrivedP, p, umbralP, kp, kb, ka, empezar, pedal
    rospy.init_node('obs', anonymous=True)
    # Se crea referencia a topico para publicar velocidad de los motores
    pubMot = rospy.Publisher ('InfoObs', Twist, queue_size=10)#twist

    obs=Twist()
    obs.linear.x=6
    obs.linear.y=1
    obs.linear.z=3
    while True:
        pubMot.publish (obs)















# Metodo main, mira si existen parametros para la posicion final deseada y ejecuta el metodo principal
if __name__ == '__main__':
    try:
        # En caso de que se pasen tres parametros tipo numero se ajusta la nueva posicion final deseada

        punto2c()
    except rospy.ROSInterruptException:
        pass
