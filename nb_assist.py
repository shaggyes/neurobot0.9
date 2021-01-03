#!/usr/bin/env python
#import time
#import datetime
#import os
import rospy
#from sys import stdout
#from math import sin, cos, pi
#from subprocess import call
from std_msgs.msg import Float32MultiArray  # params
from std_msgs.msg import String  # error msg
from nav_msgs.msg import Odometry  # odometry
from geometry_msgs.msg import TwistStamped, Twist  # speed
from sensor_msgs.msg import LaserScan  # Ultrasonic Scan 6x
Timing = 10
dt = float(1)/Timing

halftimespeed = 0  # speed filter
koefBreak = 2.5  # koef of breaking system to AOA
margin = 0.05  # in m for AOA
pult_speed_cmd = [.0, .0]
odom_speed = [.0, .0]
recv_params = [0, 0, 0.12, 0.268, 0.00406, 0.5, 1, 0.1, 0.1, 0.1, 1, 2]
# AOA, PWR, wheelDia, wheelBase, EncToSpeed, MaxSpeed, PorV, PID.P, PID.I, PID.D, FullDrive, MaxLagInS
rangesScan = [.0, 0, 0, 0, 0, 0]
delayPult = 0


def callback_params(data):
    global recv_params
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    recv_params = data.data
def callback_odom(data):
    global odom_speed
    odom_speed = [data.twist.twist.linear.x, data.twist.twist.angular.z]
def callback_twist(data):
    global pult_speed_cmd, delayPult
    delayPult = data.header.stamp
    pult_speed_cmd = [data.twist.linear.x, data.twist.angular.z]
def callback_ranges(data):
    global rangesScan
    rangesScan = data.ranges


# low pass filter
def filter(sensor, prev, filtConst):
    return ((sensor-prev)*filtConst + prev)


def AOA(speedReal, cmdSpeed, cmdAngle, dist_top, dist_bottom):
    global halftimespeed, koefBreak, margin, dt
    #halftimespeed = filter(speedy, halftimespeed, 0.5)

    if (abs(speedReal) > abs(cmdSpeed)):
        speedy = speedReal
    else:
        speedy = cmdSpeed
        
    if (speedy > 0):
        if (speedy*dt) > (dist_top-margin)/koefBreak:
            cmdSpeed = 0
    elif (speedy < 0):
        if (-speedy*dt) > (dist_bottom-margin)/koefBreak:
            cmdSpeed = 0
    
    return [cmdSpeed, cmdAngle]


def mainfunc():
    global pult_speed_cmd, Timing, dt, recv_params, odom_speed,\
           rangesScan, delayPult
    print('step1')
    #limit = Timing * 60 * 3
    counter = 0
    rospy.init_node('nbcoreassist', anonymous=True)
    rate = rospy.Rate(Timing)  # 10hz

    pubC = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    #pubError = rospy.Publisher('neuroboterror', String, queue_size=1)
    PultSpeed = Twist()
    sonars = [0, 0, 0, 0, 0, 0]
    delayPult = rospy.Time.now()
    print('step1.5')

    rospy.Subscriber('neurobotparams', Float32MultiArray,
                     callback_params)
    rospy.Subscriber('neurobot_vel', TwistStamped, callback_twist)
    rospy.Subscriber('odom', Odometry, callback_odom)
    rospy.Subscriber('neurobotsonarscan', LaserScan, callback_ranges)
        

    while not rospy.is_shutdown():


        #sonars[2] = filter(rangesScan[2], sonars[2], 0.5)
        #sonars[5] = filter(rangesScan[5], sonars[5], 0.5)
        sonars[2] = rangesScan[2]
        sonars[5] = rangesScan[5]
        delay = (rospy.Time.now() - delayPult).to_sec()
        if (delay > recv_params[11]):
            print("\r\033[F\r\033[F BIG lag %.2f s!!!!!!!!" % delay)
            pult_speed_cmd[0] = 0
            pult_speed_cmd[1] = 0
        else:
            print("\r\033[F\r\033[F working with %.2fs lag" % delay)
        print("empty space")
        if (recv_params[6] == 0): # if pid is off
            pult_speed_cmd[0] = pult_speed_cmd[0]*recv_params[5]
        if (recv_params[0] == 1):  # AOA on
            pult_speed_cmd = AOA(odom_speed[0], pult_speed_cmd[0],
                                 pult_speed_cmd[1], sonars[2], sonars[5])
            #print("AOAon  spd %.2f" % pult_speed_cmd[0])
        #else:
            #print("AOAoff spd %.2f" % pult_speed_cmd[0])
        PultSpeed.linear.x = pult_speed_cmd[0]
        PultSpeed.angular.z = pult_speed_cmd[1]
        pubC.publish(PultSpeed)

        if (recv_params[1] == 1):  # shutdown now
            print("shutdown now")
            call("sudo shutdown now", shell=True)
        elif (recv_params[1] == 2): # roscore kill
            print("kill roscore")
            call("sudo killall roscore", shell=True)
        rate.sleep()


if __name__ == '__main__':
    try:
        mainfunc()
    except rospy.ROSInterruptException:
        pass

