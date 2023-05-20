#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import String

from tkinter import *
from PIL import Image, ImageTk
import numpy as np
from pyzbar import pyzbar

import math

import cv2
import sys

      

angular_right = 0
angular_left = 0
theta_odom = 0
qr_lectura = "Objeto"
last = 0
last_x_obj = 0
last_y_obj = 0
hxr = 0
hyr = 0
hxd = 0
hyd = 0
hxo = 0
hyo = 0
s = 145 # Region repulsiva
hye = 0
hxe = 0

def theta_callback(data):
    global theta_odom
    theta_odom = data.data
    #print ("Estoy leyendo en theta: ", theta_odom)

def qr_callback(data):
    global qr_lectura 
    qr_lectura = data.data
    #print ("Estoy leyendo el QR: ", qr_lectura)

def toggle():
    btn.config(text=btnVar.get())


def onClosing():
     #arduino.sendData([0,0])
     #arduino.close()
     cap.release()
     print("Cámara desconectada")
     root.destroy()
     

def qrDetection(frame):

    cxr = 0
    cyr = 0
    cxd = 0
    cyd = 0
    cxo = 0
    cyo = 0
    
    isRobot = False
    isObject = False
    isObstacle = False

    barcodes = pyzbar.decode(frame)
    
    for barcode in barcodes:
        (x, y, w, h) = barcode.rect
        barcodeData = barcode.data.decode("utf-8")

        if(barcodeData=="Robot4"):
            isRobot = True
            cxr = x + (w)//2
            cyr = y + (h)//2
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

        if(barcodeData==qr_lectura):
            isObject = True
            cxd = x + (w)//2
            cyd = y + (h)//2
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

        if(barcodeData=="Robot3"):
            isObstacle = True
            cxo = x + (w)//2
            cyo = y + (h)//2
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.circle(frame,(cxo,cyo), s,(255,0,0),1)


    return frame,cxd,cyd,cxr,cyr,cxo,cyo,isRobot,isObject,isObstacle


def callback():
    global last, last_x_obj, last_y_obj, last_x_obstacle, last_y_obstacle, hxr, hyr, hxd, hyd, hxo, hyo, s, hxe, hye


    ret, frame = cap.read()

    if ret:
          
        frame,cxd,cyd,cxr,cyr,cxo,cyo,isRobot,isObject,isObstacle = qrDetection(frame)

        cv2.circle(frame,(cxd,cyd), 5,(0,255,0),-1)
        cv2.circle(frame,(cxr,cyr), 5,(0,0,255),-1)
        cv2.circle(frame,(cxo,cyo), 5,(255,0,0),-1)
          
        if(isRobot):
            hxr = cxr - frame.shape[1]/2
            hyr = frame.shape[0]/2 - cyr

        if (isObject):
            hxd = cxd - frame.shape[1]/2
            hyd = frame.shape[0]/2 - cyd
        
        if (isObstacle):
            hxo = cxo - frame.shape[1]/2
            hyo = frame.shape[0]/2 - cyo

        print (" Coordenadas deseadas: ", hxd, hyd)
        print ("    Coordenadas robot: ", hxr, hyr)
        print ("Coordenadas obstaculo: ", hxo, hyo)

        print ("Error en X: ", abs(hxe))
        print ("Error en Y: ", abs(hye))
        print ("Error acumulado: ", (abs(hxe)))
        if btnVar.get() == 'Start':

            # Calculo de velocidad ATRACTIVA
            a = 0.04085
            phi.set(theta_odom)

            # Errores
            hxe = hxd - hxr
            hye = hyd - hyr

            #pub_y_target.publish(hyd)
            #pub_x_target.publish(hxd)
            #pub_x_real.publish(hx)
            #pub_y_real.publish(hy)
             
            # Metodo Lyvsivob
            he = np.array([[hxe],[hye]])
            K = np.diag([0.0001,0.0001])
            J = np.array([[-np.sin(phi.get()),-a*np.cos(phi.get())],
                          [np.cos(phi.get()),-a*np.sin(phi.get())]])

            # Ley de control
            qpRef = np.linalg.inv(J)@(K@he)
            uRef.set(round(qpRef[0][0],3))
            wRef.set(round(qpRef[1][0],3))
        
            #Calculo wL y  wR
            L = 0.311 # 
            R = 0.034 # 
            wRattr = (2.0*uRef.get() + L*wRef.get()) / (2.0 * R)
            wLattr = (2.0*uRef.get() - L*wRef.get()) / (2.0 * R)
            print ("Velocidad angular llanta derecha (ATRACTIVA): ", wRattr)
            print ("Velocidad angular llanta izquierda (ATRACTIVA): ", wLattr)

            # Calculo de velocidad REPULSIVA
            #Distance and angle between the obstacle and the robot
            do = np.sqrt((hxr - hxo)**2 + (hyr - hyo)**2)
            tho = np.arctan2((hyr - hyo) , (hxr - hxo))
            #print ("Error en x viendo solo el robot: ", hxe)
            #print ("Error en y viendo solo el robot: ", hye)
            #print("")
            #Radious of the circle
            if (do <= s):
                xrep = 1/do * np.cos (tho)
            else:
                xrep = 0 
            if (do <= s):
                yrep = 1/do * np.sin(tho)
            else:
                yrep = 0

            # Repulsiva
            Krep = np.diag([4.0,4.0])

            herep = np.array([[xrep], [yrep]])

            # Ley de control repulsiva
            qpRefRep = np.linalg.inv(J)@(Krep@herep)
            uRefRep.set(round(qpRefRep[0][0],3))
            wRefRep.set(round(qpRefRep[1][0],3))

            wRrep = (2.0*uRefRep.get() + L*wRefRep.get()) / (2.0 * R)
            wLrep = (2.0*uRefRep.get() - L*wRefRep.get()) / (2.0 * R)
            print ("Velocidad angular llanta derecha (REPULSIVA): ", wRrep)
            print ("Velocidad angular llanta izquierda (REPULSIVA): ", wLrep)

            wLfinal = wLattr + wLrep
            wRfinal = wRattr + wRrep
            print ("Velocidad angular izquierda final: ", wLfinal)
            print ("Velocidad angular derecha final: ", wRfinal)

            pub_left.publish(wLfinal)
            pub_right.publish(wRfinal)

              
        else:
            pub_left.publish(0)
            pub_right.publish(0)

       
        img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = Image.fromarray(img)
        #img.thumbnail((1000, 1000))
        tkimage = ImageTk.PhotoImage(img)
        label1.configure(image = tkimage)
        label1.image = tkimage
  
    else:
        onClosing()

    root.after(100,callback)

pub_left = rospy.Publisher('target_left', Float64, queue_size=10)
pub_right = rospy.Publisher('target_right', Float64, queue_size=10)
pub_x_target = rospy.Publisher('x_target', Float64, queue_size=10)
pub_y_target = rospy.Publisher('y_target', Float64, queue_size=10)
pub_x_real = rospy.Publisher('x_real', Float64, queue_size=10)
pub_y_real = rospy.Publisher('y_real', Float64, queue_size=10)


rospy.Subscriber("th_odom_rad", Float64, theta_callback)
rospy.Subscriber("code", String, qr_callback)
rospy.init_node('ros_control_algorithm', anonymous=True)

cap = cv2.VideoCapture(0)

if cap.isOpened():
    print("Cámara inicializada")
else:
    sys.exit("Cámara desconectada")
          
########## Arduino ########

####port = 'COM10'
####arduino = serialArduino(port,baud=9600,sizeData=7)
####arduino.readSerialStart()

######## HMI ########

root = Tk()
root.protocol("WM_DELETE_WINDOW",onClosing)
root.title("Seguidor de codigos QR")


uRef = DoubleVar(root,0)
wRef = DoubleVar(root,0)
uRefRep = DoubleVar(root,0)
wRefRep = DoubleVar(root,0)
phi = DoubleVar(root,0)

label1 = Label(root)
label1.grid(row=0,padx=20,pady=20)


btnVar = StringVar(root,'Pause')
btn = Checkbutton(root, text=btnVar.get(), width=12, variable=btnVar,
                  offvalue='Pause', onvalue='Start', indicator=False,
                  command=toggle)

btn.grid(row=1, padx = 20, pady = 20)


root.after(100,callback)
root.mainloop()
