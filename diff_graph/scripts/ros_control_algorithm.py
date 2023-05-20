#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64
import math
import time
from roboclaw import Roboclaw

target_left = 0
target_right = 0

def vel_left_callback(data):
    global target_left
    target_left = data.data

def vel_right_callback(data):
    global target_right
    target_right = data.data 

def main_roboclaw():
    #roboclaw settings
    roboclaw = Roboclaw("/dev/ttyACM0", 38400)
    roboclaw.Open()

    elapsed_time = 0
    last_time = 0
    sample_time = 0.1 #in seconds

    #pid variables
    Kp_izq = 10.0
    Ki_izq = 50.0
    Kd_izq = 0.01

    cv1_izq = 0
    error1_izq = 0
    error2_izq = 0

    Kp_der = 10.0
    Ki_der = 50.0
    Kd_der = 0.01

    cv1_der = 0
    error1_der = 0
    error2_der = 0
    
    #parametros del bot
    largo = 0.311
    radio = 0.034

    #parametros iniciales
    theta = 0.0
    wL = 0.0
    wR = 0.0

    pub_rad = rospy.Publisher('th_odom_rad', Float64, queue_size=10)
    pub_right = rospy.Publisher('real_right', Float64, queue_size=10)
    pub_left = rospy.Publisher('real_left', Float64, queue_size=10)
    rospy.Subscriber("target_right", Float64, vel_right_callback)
    rospy.Subscriber("target_left", Float64, vel_left_callback)
    rospy.init_node('main_roboclaw', anonymous=True)

    rate = rospy.Rate(1/sample_time) # 10hz
    roboclaw.ResetEncoders(0x81)
    last_time = time.time()

    while not rospy.is_shutdown():
        
        nR = roboclaw.ReadEncM1(0x81)
        nL = roboclaw.ReadEncM2(0x81)
        #print(nR[1])       
        
        elapsed_time = time.time() - last_time
        #print (elapsed_time)
        
        # Rueda derecha
        wR = nR[1]*math.pi*2/1425.1/elapsed_time
        rpmR = wR * 30.0 / math.pi
        print("Right Real Velocity: ", wR)
        print("Right Target Velocity: ", target_right)
        # print("RPM en llanta derecha: ", rpmR)

        # Rueda izquierda
        wL = nL[1]*math.pi*2/1425.1/elapsed_time
        rpmL = wL * 30.0 / math.pi
        print("Left Real Velocity: ", wL)
        print("Left Target Velocity: ", target_left)
        # print("RPM en llanta izquierda: ", rpmR)
        #Calculo errores
        error_izq  = target_left - wL
        error_der  = target_right - wR


        #Calculo PID
        cv_izq = cv1_izq + (Kp_izq + Kd_izq/elapsed_time)*error_izq + (-Kp_izq + Ki_izq*elapsed_time - 2*Kd_izq/elapsed_time) * error1_izq + (Kd_izq/elapsed_time)*error2_izq
        cv1_izq = cv_izq
        error2_izq = error1_izq
        error1_izq = error_izq

        cv_der = cv1_der + (Kp_der + Kd_der/elapsed_time)*error_der + (-Kp_der + Ki_der*elapsed_time - 2*Kd_der/elapsed_time) * error1_der + (Kd_der/elapsed_time)*error2_der
        cv1_der = cv_der
        error2_der = error1_der
        error1_der = error_der

        #Saturacion
        if (cv_izq > 50.0):
            cv_izq = 50.0

        if (cv_izq < -50):
            cv_izq = -50.0

        if (cv_izq < 10 and cv_izq > 0):
            cv_izq = 10

        if (cv_izq >= 0):
            roboclaw.ForwardM2(0x81, int(cv_izq))
        else:
            roboclaw.BackwardM2(0x81, abs(int(cv_izq)))

        #Saturacion
        if (cv_der > 50.0):
            cv_der = 50.0

        if (cv_der < -50):
            cv_der = -50.0

        if (cv_der < 10 and cv_der > 0):
            cv_der = 10

        if (cv_der >= 0):
            roboclaw.ForwardM1(0x81, int(cv_der))
        else:
            roboclaw.BackwardM1(0x81, abs(int(cv_der)))

        #Print de theta
        theta = theta + (((wR - wL) / largo) * radio) * elapsed_time
        theta_deg = theta*180.0/math.pi
        print ("Orientacion en rads: ", theta)
        print ("Orientacion en degs: ", theta_deg)
        pub_rad.publish(theta)
        pub_right.publish(wR)
        pub_left.publish(wL)


        last_time = time.time()
        roboclaw.ResetEncoders(0x81)
        rate.sleep()

if __name__ == '__main__':
    try:
        main_roboclaw()
    except rospy.ROSInterruptException:
        pass