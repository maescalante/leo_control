#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import rospy, math, roslaunch, time, sys
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TwistStamped,PoseWithCovariance,Pose,Point
import matplotlib.pyplot as plt
import networkx as nx

# Variable global referencia de la figura
fig = None
# Variable global referencia de los ejes de la figura
axs = None
# Arreglo cordenadas trayectoria en x robot
xCord = []
# Arreglo cordenadas trayectoria en y robot
yCord = []
# Variable global referencia de la animacion
ani = None
# Variable tipo Twist() con posicion actual robot
position = Twist()
#Es la variable en donde se almacena la posicion y orientacion actual del obstaculo 1.
lisObs=[]

# Funcion que recibe y actualiza informacion de topicos de obstaculos, numero obstaculo esta guardado en twist.angular.x
def setObst(posicionObstacle):
    if posicionObstacle not in lisObs:
        lisObs.append(posicionObstacle)


# Metodo principal: iniciliza nodo, suscribe a topico posicion, crea figura, crea ejes, llama metodo actualizacion de
# datos animate cada 0.3 segundos, muestra figura y cuando esta se cierre las guarda
def graficar():
    global fig, xCord, axs, ani
    fig = plt.figure()
    axs = fig.add_subplot(111)
    rospy.init_node('graficador', anonymous=True)
    #rospy.Subscriber('pioneerPosition', Twist, setNewPosition)
    rospy.Subscriber ('/controllers/diff_drive/odom', Odometry, setNewPosition) # wheels_odom twist stamped
    rospy.Subscriber ('InfoObs', Twist, setObst)
    ani = animation.FuncAnimation(fig, animate, interval=300)
    plt.show()
    # fig.savefig ('../catkin_ws/src/taller2_4/results/resultsPunto2/figure.png')


# Metodo para actualizar posicion robot en variable position una vez otro nodo publique en el topico
def setNewPosition(odom):
    global position
    pos=odom.pose
    point= pos.pose

    position.linear.x=point.position.x
    position.linear.y=point.position.y
    position.angular.z=point.orientation.z


# Metodo para actualizar los arreglos de cordenadas, es llamado cada 0.3 segundos por meotodo principal
def animate(i):
    global axs, xCord, yCord, position
    xCord.append (position.linear.x)
    # xCord = xCord[-100:]
    yCord.append (position.linear.y)
    # yCord = yCord[-100:]
    axs.clear()
    axs.axes.set_xlim(-40, 40)
    axs.axes.set_ylim(-40, 40)
    for i in lisObs:

        axs.add_artist (plt.Circle ((i.linear.x, i.linear.y), i.linear.z/2, color='r'))

    axs.plot(xCord, yCord)
    plt.title('Posicion en tiempo real de Robot')
    plt.grid()

# Metodo Main, simplemente ejecuta el metodo principal
if __name__ == '__main__':
    try:
        graficar()
    except rospy.ROSInterruptException:
        pass
