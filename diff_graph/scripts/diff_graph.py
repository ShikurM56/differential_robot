#!/usr/bin/env python3

from std_msgs.msg import Float64
import rospy
from std_msgs.msg import String

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np


x_robot = 0
y_robot = 0
x_objeto = 0
y_objeto = 0





def x_robot_callback(data):
    global x_robot
    x_robot = data.data

def y_robot_callback(data):
    global y_robot
    y_robot = data.data
def x_tag_callback(data):
    global x_objeto
    x_objeto = data.data

def y_tag_callback(data):
    global y_objeto
    y_objeto = data.data

def diff_graph():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('plotter', anonymous=True)
    rospy.Subscriber("x_real", Float64, x_robot_callback)
    rospy.Subscriber("y_real", Float64, y_robot_callback)
    rospy.Subscriber("x_target", Float64, x_tag_callback)
    rospy.Subscriber("y_target", Float64, y_tag_callback)

    # Creamos una figura vacía
    fig = plt.figure()

    # Definimos los límites de los ejes x e y
    plt.xlim(-220, 220)
    plt.ylim(-220, 220)

    # Creamos una línea vacía con un marcador circular
    line, = plt.plot([], [], 'ro-')
    line2, = plt.plot([], [], 'go-')

    # Definimos una función que se llamará repetidamente para actualizar la línea
    def update(frame):
        # Generamos posiciones x e y aleatorias para cada actualización

        # Actualizamos la línea con las nuevas posiciones
        line.set_data(int(x_robot), int(y_robot))
        #print ("Recibi en x: ", x_robot)    
        #print ("Recibi en y: ", y_robot)   
        line2.set_data(int(x_objeto), int(y_objeto))

        # Devolvemos la línea actualizada
        return line,

    # Creamos una animación con la función "update", que se actualizará cada 100 milisegundos
    ani = FuncAnimation(fig, update, interval=100)

    # Mostramos el gráfico
    plt.show()


    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    diff_graph()