#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64

from tkinter import *
from PIL import Image, ImageTk
import numpy as np
from pyzbar import pyzbar

import cv2
import sys

      

angular_right = 0
angular_left = 0
theta_odom = 0


def theta_callback(data):
    global theta_odom
    theta_odom = data.data
    print ("Estoy leyendo en theta: ", theta_odom)

def toggle():
    btn.config(text=btnVar.get())


def onClosing():
     #arduino.sendData([0,0])
     #arduino.close()
     cap.release()
     print("Cámara desconectada")
     root.destroy()
     

def qrDetection(frame):

    cx = 0
    cy = 0
    cxd = 0
    cyd = 0
    
    isRobot = False
    isObject = False

    barcodes = pyzbar.decode(frame)
    
    for barcode in barcodes:
        (x, y, w, h) = barcode.rect
        barcodeData = barcode.data.decode("utf-8")

        if(barcodeData=="Robot4"):
            isRobot = True
            cx = x + (w)//2
            cy = y + (h)//2
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 2)

        if(barcodeData=="Objeto"):
            isObject = True
            cxd = x + (w)//2
            cyd = y + (h)//2
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    return frame,cxd,cyd,cx,cy,isRobot,isObject


def callback():
     
     ret, frame = cap.read()

     if ret:
          

          frame,cxd,cyd,cx,cy,isRobot,isObject = qrDetection(frame)

          cv2.circle(frame,(cxd,cyd), 5,(0,255,0),-1)
          cv2.circle(frame,(cx,cy), 5,(0,0,255),-1)
        
          
          

          if(isRobot and isObject):

              ####################### Conversion coordenadas ##################
              hxd = cxd - frame.shape[1]/2
              hyd = frame.shape[0]/2 - cyd

              hx = cx - frame.shape[1]/2
              hy = frame.shape[0]/2 - cy

              ###################### Algoritmo de control ###################
              a = 0.04085
              phi.set(theta_odom)

              # Errores

              hxe = hxd - hx
              hye = hyd - hy
              pub_y_target.publish(hyd)
              pub_x_target.publish(hxd)
              pub_x_real.publish(hx)
              pub_y_real.publish(hy)
              
              he = np.array([[hxe],[hye]])

              K = np.diag([0.0005,0.0005])

              J = np.array([[-np.sin(phi.get()),-a*np.cos(phi.get())],
                            [np.cos(phi.get()),-a*np.sin(phi.get())]])

              # Ley de control

              qpRef = np.linalg.inv(J)@(K@he)

              uRef.set(round(qpRef[0][0],3))
              wRef.set(round(qpRef[1][0],3))
              
          else:
              uRef.set(0)
              wRef.set(0)

          if btnVar.get() == 'Start':
              #Calculo wL y  wR
              L = 0.311 # Cambiar
              R = 0.034 # Cambiar
              wR = (2.0*uRef.get() + L*wRef.get()) / (2.0 * R)
              wL = (2.0*uRef.get() - L*wRef.get()) / (2.0 * R)

              pub_left.publish(wL)
              print ("wL target: ", wL)
              pub_right.publish(wR)
              print ("wR target: ", wR)



              #####arduino.sendData([uRef.get(),wRef.get()])
              pass
              
          else:
              
              #######arduino.sendData([0,0])
              pass
         
          img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
          img = Image.fromarray(img)
          #img.thumbnail((400,400))
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
