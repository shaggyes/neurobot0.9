#!/usr/bin/env python
#import sys
#import time
import rospy
#import datetime
import serial
from os import popen
from math import sin, cos, pi
from subprocess import call
from std_msgs.msg import Int32MultiArray  # meuserements
from std_msgs.msg import String  # error msg
from std_msgs.msg import Float32MultiArray  # params
from sensor_msgs.msg import BatteryState  # battery
from nav_msgs.msg import Odometry  # odometry
from geometry_msgs.msg import Twist  # cmd_vel
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3  # odom calc

port = serial.Serial("/dev/ttyAMA0", baudrate=57600, timeout=.20,
                     write_timeout=0.5)

Timing = 10
dt = float(1)/Timing
maxIntegral = 1.0 # = limits
minPWM = 0.28  # min voltage on motors required
PIDpreverrM1 = 0  # for pid
PIDintegralM1 = 0
PIDpreverrM2 = 0
PIDintegralM2 = 0
recv_params = [0, 0, 0.12, 0.268, 0.0003359, 0.5, 1, 0.65, 2.0, 0.1, 1, 2]
processed_speed = [.0, .0]
PoseXYth = [.0, .0, 0]


def callback_params(data):
    global recv_params
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    recv_params = data.data


def callback_twist(data):
    global processed_speed
    processed_speed = [data.linear.x, data.angular.z]


def filter(sensor, prev, filtConst):
    return ((sensor-prev)*filtConst + prev)


def position(dt, vx, vy, vth, x, y, th):
    delta_x = (vx * cos(th) - vy * sin(th)) * dt
    delta_y = (vx * sin(th) + vy * cos(th)) * dt
    delta_th = vth * dt
    x += delta_x
    y += delta_y
    th += delta_th
    # since all odometry is 6DOF we'll need a quaternion created from yaw
    #odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)
    odom_quat = [0, 0, sin(th/2), cos(th/2)]
    return Pose(Point(x, y, 0.), Quaternion(*odom_quat))


def PID(error, integral, previous_error, limits, Kp, Ki, Kd):
    global dt, maxIntegral
    integral = integral + error * dt * Ki
    if (integral > maxIntegral): # or limits
        integral = maxIntegral
    elif (integral < -maxIntegral):
        integral = -maxIntegral
    derivative = (error - previous_error) / dt
    output = float(Kp * error + integral + Kd * derivative) #*limits / 100
    previous_error = error
    if (output > limits):
        output = limits
    elif (output < - limits):
        output = -limits
    return output, previous_error, integral # error here new


def SpeedSender(speed, angle, realSpeed, realAngle):
    global recv_params  # drive, PorV, MaxSpeed, WheelBase, PIDs
    global PIDpreverrM1, PIDintegralM1, PIDpreverrM2, PIDintegralM2, minPWM
    l = ((2*speed) - (angle * recv_params[3]))/2  #command speed
    r = ((2*speed) + (angle * recv_params[3]))/2

    if (recv_params[6] == 1):  # Adaptive
        lout = 0,2425*(15.86**l)
        rout = 0,2425*(15.86**r)
        ##lout = 16.627*l*l*l−12.665*l*l+4.076*l+0.012
        ##rout = 16.627*r*r*r−12.665*r*r+4.076*r+0.012

    elif (recv_params[6] == 2):  # PID

        if (l > recv_params[5]):
	    l = recv_params[5]
        elif (l < -recv_params[5]):
	    l = -recv_params[5]
        if (r > recv_params[5]):
	    r = recv_params[5]
        elif (r < -recv_params[5]):
	    r = -recv_params[5]

        leftS = ((2*realSpeed) - (realAngle * recv_params[3]))/2
        rightS = ((2*realSpeed) + (realAngle * recv_params[3]))/2

        lout, PIDpreverrM1,PIDintegralM1 = PID(l-leftS, PIDintegralM1, PIDpreverrM1, 1.0,
                            recv_params[7], recv_params[8], recv_params[9])
        rout, PIDpreverrM2,PIDintegralM2 = PID(r-rightS, PIDintegralM2, PIDpreverrM2, 1.0,
                            recv_params[7], recv_params[8], recv_params[9])

    elif (recv_params[6] == 3):  # adaptive PID 20%

        if (l > recv_params[5]):
	    l = recv_params[5]
        elif (l < -recv_params[5]):
	    l = -recv_params[5]
        if (r > recv_params[5]):
	    r = recv_params[5]
        elif (r < -recv_params[5]):
	    r = -recv_params[5]

        leftS = ((2*realSpeed) - (realAngle * recv_params[3]))/2
        rightS = ((2*realSpeed) + (realAngle * recv_params[3]))/2

        lout, PIDpreverrM1,PIDintegralM1 = PID(l-leftS, PIDintegralM1, PIDpreverrM1, 1.0,
                            recv_params[7], recv_params[8], recv_params[9])
        rout, PIDpreverrM2,PIDintegralM2 = PID(r-rightS, PIDintegralM2, PIDpreverrM2, 1.0,
                            recv_params[7], recv_params[8], recv_params[9])
        lout = lout*0.2 + 0,194*(15.86**l)
        rout = rout*0.2 + 0,194*(15.86**r)

    else:
        lout = l
        rout = r

    if (lout > 1.0):
        lout = 1.0
    elif (lout < -1.0):
        lout = -1.0
    if (rout > 1.0):
        rout = 1.0
    elif (rout < -1.0):
        rout = -1.0
    if (lout < -minPWM):
        L = 60 + int(lout*51)  # magic number!! importante
    elif (lout > minPWM):
        L = int(lout*51)
    else:
        L = 0
    if (rout < -minPWM):
        R = 60 + int(rout*51)
    elif (lout > minPWM):
        R = int(rout*51)
    else:
        R = 0
    print("\r\033[FRight: %d , Left: %d  " %(R, L))
    return str(chr(125+int(recv_params[10]))+chr(L)+chr(R))


def mainfunc():
    global port, processed_speed, Timing, dt, recv_params, PoseXYth
    print('step1')
  
    rospy.init_node('nbcore', anonymous=True)
    rate = rospy.Rate(Timing)  # 10hz
    
    pubM = rospy.Publisher('neurobotmeasure', Int32MultiArray, queue_size=1)
    pubB = rospy.Publisher('neurobotbattery', BatteryState, queue_size=1)
    pubO = rospy.Publisher('odom', Odometry, queue_size=1)
    # pubError = rospy.Publisher('neuroboterror', String, queue_size=1)
    CoreMeasure = Int32MultiArray()
    CoreMeasure.data = []
    CoreBat = BatteryState()
    CoreOdom = Odometry()
    CoreOdom.header.frame_id = "odom"
    CoreOdom.child_frame_id = "base_link"
    
    # Error = String()
    temporarystr = port.read(5)
    prevVoltage = int((ord(temporarystr[1])*0.0316+7.244)*1000)
    batteryS = int(prevVoltage//3200)
    if batteryS < 1:
        batteryS = 1

    speedRightPrev = 0
    speedRight = 0
    speedLeftPrev = 0
    speedLeft = 0
    speedBase = 0
    angleSpeed = 0
    print('step2')


    rospy.Subscriber('neurobotparams', Float32MultiArray,
                     callback_params)
    rospy.Subscriber('cmd_vel', Twist, callback_twist)  # to inwaiting


    while not rospy.is_shutdown():
        # filter here
        port.write(SpeedSender(processed_speed[0], processed_speed[1],
                               speedBase, angleSpeed))

        if port.inWaiting:

            serialstr = []
            serialstr = port.read(5)

            voltage = int((ord(serialstr[1]) * 0.0316 + 7.244) * 1000)  # in mV
            prevVoltage = int(filter(voltage, prevVoltage, 0.1))
            current = int((-0.0004*(ord(serialstr[2])**2) +
                           (0.0697*ord(serialstr[2]))-0.5384) * 1000)  # in mA
            VperS = float(prevVoltage)/(1000*batteryS)
            bat0l0 = int(-(188.32*(VperS**3))+(2141.2*(VperS**2)) - (7964.4*VperS)+9732.7)
            #print(bat0l0, VperS)
            tempStr = popen('vcgencmd measure_temp').readline()
            tempStr = tempStr.replace("temp=", "")
            tempStr = tempStr.replace(".", "")
            tempStr = tempStr.replace("'C\n", "")
            tempPi = int(tempStr)/10

            enc1 = ord(serialstr[4])
            enc2 = ord(serialstr[3])
            if ((((speedRightPrev < 20) or (speedRightPrev > 245)) or
                 (speedRight < 0)) and (enc1 > 100)):
                speedRight = -(256-enc1)
            else:
                speedRight = enc1
            speedRightPrev = enc1
            if ((((speedLeftPrev < 20) or (speedLeftPrev > 245)) or
                 (speedLeft < 0)) and (enc2 > 100)):
                speedLeft = -(256-enc2)
            else:
                speedLeft = enc2
            speedLeftPrev = enc2
            velocityLeft = speedLeft*recv_params[4]*Timing
            velocityRight = speedRight*recv_params[4]*Timing
            speedBase = (velocityLeft + velocityRight)/2
            angleSpeed = (velocityRight - velocityLeft)/recv_params[3]

            CoreMeasure.data = [ord(serialstr[0]), 0, prevVoltage,
                                bat0l0, current, tempPi, speedLeft, speedRight]
# [ But1, But2, Voltage, Charge%, Current, Temp, EncoderTicks1, Encoder2 ]
            CoreBat.voltage = voltage/1000
            CoreBat.current = current/1000
            CoreBat.percentage = float(bat0l0)/100
            #rospy.loginfo(CoreMeasure)
            pubM.publish(CoreMeasure)
            pubB.publish(CoreBat)

        CoreOdom.header.stamp = rospy.Time.now()
        CoreOdom.pose.pose = position(dt, speedBase, 0, angleSpeed,
                                      PoseXYth[0], PoseXYth[1], PoseXYth[2])
        CoreOdom.twist.twist = Twist(Vector3(speedBase, 0, 0),
                                     Vector3(0, 0, angleSpeed))
        pubO.publish(CoreOdom)
        #Error.data = "none"
        #pubError.publish(Error)

        rate.sleep()


if __name__ == '__main__':
    try:
        mainfunc()
    except rospy.ROSInterruptException:
        pass

