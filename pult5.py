#!/usr/bin/env python
import pygame
import socket
import datetime
import sys
import rospy
import tf
import math
from math import sin, cos, pi, acos, asin
from std_msgs.msg import Int32MultiArray    # meuserements
from std_msgs.msg import Float32MultiArray  # params
from std_msgs.msg import String    # error msg
from nav_msgs.msg import Odometry   # odometry
from geometry_msgs.msg import TwistStamped  # cmd_vel
from geometry_msgs.msg import TwistStamped  # cmd_vel
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import LaserScan   # Ultrasonic Scan 6x
from sensor_msgs.msg import BatteryState    # battery
from sensor_msgs.msg import Imu  # Gyro and acel
from sensor_msgs.msg import MagneticField  # Magnetic field
pygame.init()
# preprocedure
# pygame settings
pygame.display.set_caption('Keyboard for robot pobot')
size = [640, 480]
screen = pygame.display.set_mode(size)
clock = pygame.time.Clock()
# by default the key repeat is disabled call set_repeat() to enable it
pygame.key.set_repeat(50, 50)
robot_skin = pygame.image.load('catkin_ws/src/neurobot5/src' +
                               '/robotskin.png').convert()
robot_skin.set_colorkey((255, 255, 255))
robot_skin = pygame.transform.scale(robot_skin,
                                    (robot_skin.get_width()//2,
                                     robot_skin.get_height()//2))
# graphics
# colors
white = (255, 255, 255)
red = (255, 0, 0)
green = (0, 255, 0)
blue = (10, 10, 250)
violet = (200, 10, 160)
grey = (160, 160, 160)
black = (5, 0, 20)
lightred = (255, 200, 200)
lightgreen = (200, 255, 200)
darkred = (155, 0, 0)
orangedark = (200, 150, 0)
darkgreen = (0, 155, 0)
darkblue = (0, 0, 155)
bluishred = (235, 100, 30)
contrastgreen = (20, 175, 20)
popblue = (55, 0, 200)
bluishblue = (30, 100, 230)
# rendered text and pic
myfont = pygame.font.SysFont("monospace", 15)
GOfont = pygame.font.SysFont("monospace", 90)
GOlabel = GOfont.render(str("GAME OVER"), 1, black)
text_filtered = myfont.render("Filtered", 1, violet)
text = ["key LEFT, RIGHT, UP, DOWN - control movement of robot",
        "L_Shift - decrease speed limit", "R_Shift increase speed limit",
        "TAB - increase speed limit to 100% - 'turbo mode'",
        "F - turn on/off low-pass filter for robots params",
        "O - turn on/off odometry in meters", "P - PID turn on/off",
        "I - AOA turn on/off", "SPACE - Full/middle drive",
        "G - show compass and IMU data",
        "Backspace - exit of this application",
        "Delete - exit of this application",
        "ESC - Raspberry Pi shutdown signal", " ", "\v version 0.5",
        " ", " ", "  > Press any button to close legend <"]
legendary = []
for line in text:
	legendary.append(myfont.render(line, True, white))
warningtext = myfont.render("WARNING! BATT disCHG", 1, red)
textAWD = myfont.render("AWD", 2, black)
text2WD = myfont.render("2-WheelD", 2, black)
textAOA = myfont.render("AOA is on", 2, red)
textPressL = myfont.render("Press 'L' for legend", 2, grey)

# params defined
params_common = [0, 0, 0.12, 0.268, 0.0003359, 0.5, 1, 0.65, 2.2, 0.1, 1, 5]
# AOA, PWR, wheelDia, wheelBase, EncToSpeed, MaxSpeed, PorV, PID.P, PID.I,
# PID.D, FullDrive, MaxLagInS

# var list
trigger_animation = True
fps = 10    # FPS!!!!
timePi = 0
timeComp = 0
M1 = 0      # speed for left board
M2 = 0
k = 0.5     # from max speed
prevvoltage = 0     # filter vars
batteryS = 2
prevcurr = 0
odomSpeed = 0
odomAngle = 0
odomDistance = 0
odomX = 0
odomY = 0
odomTheta = 0
deltatoPi = 0
deltafromPi = 0
btn = False     # btns on board
btn2 = False
odometerBtn = False     # btns on comps keyboard
btnFilterOn = False
legend = False
PIDbtn = 1 # 0 - pwm, 1 - adaptive, 2 - pid, 3 - adaptive pid
OnOff = 0
AOAbtn = 0
FullDrivebtn = 1
ShowImubtn = False
filtrSCANdata = [0, 0, 0, 0, 0, 0]  # ranges filtering
unfiltrSCANdata = filtrSCANdata
AnimationCounter = 0    # temporary
dt = float(1)/fps   # integral part
# callback vars
rangesScan = [0.99, 0, 0, 0, 0, 0]
baromData = [-99.9, -999]
measuresCore = [0, 0, 8000, 50, 100, 19, 0, 0]
imuA = Vector3(.0, .0, .0)
imuG = Vector3(0, .0, .0)
compassAngle = 30
# [ But1, But2, Voltage, Charge%, Current, Temp, EncoderTicks1, Encoder2 ]


# legend of keyboard
def legend_show():
    screen.fill(black)
    for line in range(len(legendary)):
        screen.blit(legendary[line], (100, 100+line*15))


# low pass filter common
def filter(sensor, prev, filtConst):
    return ((sensor-prev)*filtConst + prev)


# callback functions
def callback_ranges(data):
    global rangesScan
    rangesScan = data.ranges


def callback_imu(data):
    global imuA, imuG
    imuG = data.angular_velocity
    imuA = data.linear_acceleration


def callback_compass(data):
    global compassAngle
    compassAngle = (math.atan2(data.magnetic_field.x, data.magnetic_field.y)*180/math.pi)
    if (data.magnetic_field.x > -3) and (data.magnetic_field.x < 3):
        compassAngle = 0 - int(data.magnetic_field.x)
    # print(compassAngle)


def callback_measures(data):
    global measuresCore
    measuresCore = data.data


def callback_barometer(data):
    global baromData
    baromData = data.data


# odomList = [0, 0, 0, 0, 0] #speedx, anglespeed, posX, posY, w
def callback_odometry(data):
    global odomX, odomY, odomTheta, odomSpeed, odomAngle, timePi
    timePi = data.header.stamp
    odomSpeed = data.twist.twist.linear.x
    odomAngle = data.twist.twist.angular.z
    odomX = data.pose.pose.position.x
    odomY = data.pose.pose.position.y
    odomTheta = acos(data.pose.pose.orientation.w)*2


# function that changes vars by butns are pressed
def keyboardCheck():
    global k, M1, M2, btnFilterOn, odometerBtn, legend, PIDbtn, \
           OnOff, AOAbtn, FullDrivebtn, AnimationCounter, ShowImubtn
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                M1 = k
                M2 = -k
            elif event.key == pygame.K_RIGHT:
                M1 = -k
                M2 = k
            if event.key == pygame.K_UP:
                M1 = k
                M2 = k
            elif event.key == pygame.K_DOWN:
                M1 = -k
                M2 = -k
            if pygame.key.get_mods() & pygame.KMOD_LSHIFT:
                k = k - 0.05
            elif event.key == pygame.K_TAB:
                k = 1
            elif pygame.key.get_mods() & pygame.KMOD_RSHIFT:
                k = k + 0.05
            if event.key == pygame.K_ESCAPE:
                OnOff = 1
            elif event.key == pygame.K_DELETE:
                OnOff = 2
            if event.key == pygame.K_f:
                if (btnFilterOn is False):
                    btnFilterOn = True
                else:
                    btnFilterOn = False
            if event.key == pygame.K_o:
                if (odometerBtn is False):
                    odometerBtn = True
                else:
                    odometerBtn = False
            if event.key == pygame.K_p: # drive type change
                if (PIDbtn == 0):
                    PIDbtn = 1
                else:
                    PIDbtn = 0
            elif event.key == pygame.K_2:
                PIDbtn = 2
            elif event.key == pygame.K_3:
                PIDbtn = 3
            if event.key == pygame.K_i:
                if (AOAbtn == 0):
                    AOAbtn = 1
                else:
                    AOAbtn = 0
            if event.key == pygame.K_g:
                if (ShowImubtn is False):
                    ShowImubtn = True
                else:
                    ShowImubtn = False
            if event.key == pygame.K_SPACE:
                if (FullDrivebtn == 0):
                    FullDrivebtn = 1
                else:
                    FullDrivebtn = 0
            if (event.key == pygame.K_l):
                legend = True
            else:
                legend = False
            if event.key == pygame.K_BACKSPACE:
                pygame.quit()
                sys.exit()
            AnimationCounter += 1
        else:
            M1 = 0
            M2 = 0
            AnimationCounter = 0
        if k > 1:
            k = 1
        elif k < 0.1:
            k = 0.1


# convert diff drive to steering ackerman
def representSpeed(Motor1, Motor2, BaseWidth, MaxSpeed=params_common[5]): 
    global PIDbtn
    X = (Motor1 + Motor2)/2
    W = (Motor1 - Motor2)/BaseWidth
    if (PIDbtn != 0):
        X = X*MaxSpeed
    return X, W


# func shows media
def blitting():
    cA=compassAngle*pi/180
    # buttons f and space
    if (btnFilterOn is True):
        screen.blit(text_filtered, (440, 410))
    if (FullDrivebtn == 0):
        screen.blit(text2WD, (540, 410))
    else:
        screen.blit(textAWD, (590, 410))
    if (AOAbtn == 1):
        screen.blit(textAOA, (15, 15))
    # obstacles visual:
    pygame.draw.rect(screen, (filtrSCANdata[1], filtrSCANdata[1], 255),
                     ((60, 255-filtrSCANdata[1]), (120, 10)), 0)
    pygame.draw.rect(screen, (filtrSCANdata[2], filtrSCANdata[2], 255),
                     ((260, 255-filtrSCANdata[2]), (120, 10)), 0)
    pygame.draw.rect(screen, (filtrSCANdata[3], filtrSCANdata[3], 255),
                     ((460, 255-filtrSCANdata[3]), (120, 10)), 0)
    # obstacles text:
    label2 = myfont.render(str(filtrSCANdata[1]), 1, darkred)
    label3 = myfont.render(str(filtrSCANdata[2]), 1, darkred)
    label4 = myfont.render(str(filtrSCANdata[3]), 1, darkred)
    label1 = myfont.render(str(filtrSCANdata[0]), 1, popblue)
    label6 = myfont.render(str(filtrSCANdata[5]), 1, popblue)
    label5 = myfont.render(str(filtrSCANdata[4]), 1, popblue)
    screen.blit(label2, (115, 100))
    screen.blit(label3, (320, 100))
    screen.blit(label4, (540, 100))
    screen.blit(label1, (115, 440))
    screen.blit(label6, (310, 450))
    screen.blit(label5, (540, 440))
    # params:
    text_volt = myfont.render(str(float('{:.2f}'.format(prevvoltage/1000))) +
                              "V", 1, (155, 200, 0))
    text_curr = myfont.render(str(float('{:.2f}'.format(prevcurr/1000))) + "A",
                              1, (15, 200, 150))
    text_temp = myfont.render("t:" + str('{:.2f}'.format(baromData[0]))+"c", 1, bluishred)
    text_timeComp = myfont.render("ping:"+str(deltafromPi)+"ms", 1, bluishblue)
    text_tempPi = myfont.render("tPi:" + str(float(measuresCore[5]))+"c", 1, violet)
    text_press = myfont.render("p:" + str('{:.2f}'.format(baromData[1]))+"hPa", 1, darkred)
    if (measuresCore[3] > 10):
        text_bat0l0 = myfont.render("BATT." + str(batteryS) + "sLiPo " +
                                    str(measuresCore[3])+"%", 1, orangedark)
        screen.blit(text_bat0l0, (50, 410))
    else:
        screen.blit(warningtext, (50, 410))
    screen.blit(text_volt, (50, 320))
    screen.blit(text_curr, (140, 320))
    screen.blit(text_tempPi, (50, 345))
    screen.blit(text_timeComp, (50, 370))
    screen.blit(text_temp, (140, 345))
    screen.blit(text_press, (50, 390))
    # odometry:
    if (PIDbtn == 0):
        text_limitSpeed = myfont.render("Speed " + str(k*100)+"% PWM",
                                        1, contrastgreen)
    else:
        text_limitSpeed = myfont.render("Speed " + str(k*(params_common[5])) +
                                        "m/s", 1, contrastgreen)
    screen.blit(text_limitSpeed, (440, 300))
    text_speeed = myfont.render(str('{:.2f}'.format(odomSpeed))+" m/s",
                                1, contrastgreen)
    text_speeedw = myfont.render(str('{:.1f}'.format(odomAngle))+" rad/s",
                                 1, contrastgreen)
    screen.blit(text_speeed, (440, 330))
    screen.blit(text_speeedw, (440, 350))
    if (odometerBtn is True):
        screen.blit(myfont.render("Distance: " +
                                  str('{:.2f}'.format(odomDistance)) +
                                  " m", 1, darkred), (440, 370))
        screen.blit(myfont.render("X:" + str('{:.1f}'.format(odomX)) +
                                  " | Y:" + str('{:.1f}'.format(odomY)),
                                  1, orangedark), (440, 390))
    # draw a robots background
    if (M1 < 0):
        pygame.draw.rect(screen, lightred, ((320, 290), (60, 135)), 0)
    elif (M1 > 0):
        pygame.draw.rect(screen, lightgreen, ((320, 290), (60, 135)), 0)
    if (M2 < 0):
        pygame.draw.rect(screen, lightred, ((260, 290), (60, 135)), 0)
    elif (M2 > 0):
        pygame.draw.rect(screen, lightgreen, ((260, 290), (60, 135)), 0)
    # draw a robot rotation
    if (ShowImubtn is False):
        if (trigger_animation is True):
            if (M1 == 0 and M2 == 0):
                screen.blit(robot_skin, (275, 310))
            else:
                rotSkinRobot = pygame.transform.rotate(robot_skin,
                                                       AnimationCounter*(M1-M2))
                screen.blit(rotSkinRobot, (320-(rotSkinRobot.get_width()//2),
                                           360-(rotSkinRobot.get_width()//2) -
                                           ((M1+M2)/2*AnimationCounter) % 10))
        else:
            rotSkinRobot = pygame.transform.rotate(robot_skin, odomTheta)
            screen.blit(rotSkinRobot,
                        (320 - (rotSkinRobot.get_width()//2) -
                         (sin(odomTheta)*odomSpeed*AnimationCounter % 10)),
                        (360 - (rotSkinRobot.get_width()//2) -
                         (cos(odomTheta)*odomSpeed*AnimationCounter % 10)))
    else:
        text_Ax = myfont.render(str("X  "+'{:.2f}'.format(imuA.x)), 1, darkred)
        text_Ay = myfont.render(str("Y  "+'{:.2f}'.format(imuA.y)), 1, darkgreen)
        text_Az = myfont.render(str("z  "+'{:.2f}'.format(imuA.z)), 1, darkblue)
        text_Gx = myfont.render(str('{:.2f}'.format(imuG.x)), 1, darkred)
        text_Gy = myfont.render(str('{:.2f}'.format(imuG.y)), 1, darkgreen)
        text_Gz = myfont.render(str('{:.2f}'.format(imuG.z)), 1, darkblue)
        text_labels = myfont.render(str("Gyro  __  Acel"), 1, grey)
        pygame.draw.line(screen, blue, [322, 320], [322-15*sin(cA), 320-15*cos(cA)], 4)
        pygame.draw.line(screen, red, [322, 320], [322+15*sin(cA), 320+15*cos(cA)], 4)
        pygame.draw.circle(screen, grey, (322, 320), 25, 3)
        screen.blit(text_Ax, (320, 360))
        screen.blit(text_Ay, (320, 375))
        screen.blit(text_Az, (320, 390))
        screen.blit(text_Gx, (265, 360))
        screen.blit(text_Gy, (265, 375))
        screen.blit(text_Gz, (265, 390))
        screen.blit(text_labels, (260, 340))
    # game over
    if (btn is True):
        screen.blit(GOlabel, (50, 180))
    if (legend is True):
        legend_show()
    screen.blit(textPressL, (450, 2))

# func for evaluations
def calculatingParams():
    global prevcurr, prevvoltage, odometerBtn, odomDistance, fps, rangesScan,\
           filtrSCANdata, timeComp, deltafromPi # , deltatoPi
    #print(rangesScan)
    # odometry
    if (odometerBtn is False):
        odomDistance = 0
    else:
        odomDistance += abs(odomSpeed/fps)
    # low pass filter
    if (btnFilterOn is False):
        filtrSCANdata[0] = int(rangesScan[0]*100)
        filtrSCANdata[5] = int(rangesScan[5]*100)
        filtrSCANdata[4] = int(rangesScan[4]*100)
        filtrSCANdata[1] = int(filter(rangesScan[1]*100, filtrSCANdata[1], 0.5))
        filtrSCANdata[2] = int(filter(rangesScan[2]*100, filtrSCANdata[2], 0.5))
        filtrSCANdata[3] = int(filter(rangesScan[3]*100, filtrSCANdata[3], 0.5))
        prevcurr = filter(measuresCore[4], prevcurr, 0.1)
        prevvoltage = filter(measuresCore[2], prevvoltage, 0.5)
    else:
        filtrSCANdata[1] = int(filter(rangesScan[1]*100, filtrSCANdata[1], 0.2))
        filtrSCANdata[2] = int(filter(rangesScan[2]*100, filtrSCANdata[2], 0.2))
        filtrSCANdata[3] = int(filter(rangesScan[3]*100, filtrSCANdata[3], 0.2))
        filtrSCANdata[0] = int(filter(rangesScan[0]*100, filtrSCANdata[0], 0.2))
        filtrSCANdata[5] = int(filter(rangesScan[5]*100, filtrSCANdata[5], 0.2))
        filtrSCANdata[4] = int(filter(rangesScan[4]*100, filtrSCANdata[4], 0.2))
        prevcurr = filter(measuresCore[4], prevcurr, 0.05)
        prevvoltage = filter(measuresCore[2], prevvoltage, 0.1)
    # delay from pc&pi
    timeComp = rospy.Time.now()
    deltafromPi = ((timeComp - timePi).to_nsec())//1000000  # in ms
    #deltatoPi = 50 - deltafromPi
    #compassAngle = (math.atan2(magnRaw[0],magnRaw[1])*180/math.pi)
    # measuresCore[last] # corecontrol.timenow - twiststamped


# ############################################## main part here #####
def mainfunc():
    global params_common, timeComp, M1, M2, AOAbtn, OnOff, PIDbtn,\
           FullDrivebtn, timePi
    print('step0')
    rospy.init_node('pult5', anonymous=True)
    timePi = rospy.Time.now()
    #pubC = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=1)
    pubC = rospy.Publisher('neurobot_vel', TwistStamped, queue_size=1)
    pubP = rospy.Publisher('neurobotparams', Float32MultiArray, queue_size=1)
    rate = rospy.Rate(fps)  # 20hz
    #PultSpeed = Twist()
    PultSpeed = TwistStamped()
    PultParams = Float32MultiArray()

    rospy.Subscriber('neurobotmeasure', Int32MultiArray, callback_measures)
    rospy.Subscriber('neurobotsonarscan', LaserScan, callback_ranges)
    rospy.Subscriber('neurobotimu', Imu, callback_imu)
    rospy.Subscriber('neurobotcompas', MagneticField, callback_compass)
    rospy.Subscriber('neurobotbarometer', Float32MultiArray, callback_barometer)
    rospy.Subscriber('odom', Odometry, callback_odometry)

    while not rospy.is_shutdown():
        # batteryS = int(prevvoltage//3200)

        keyboardCheck()
        calculatingParams()
        screen.fill(white)
        blitting()
        pygame.display.update()
        params_common[0] = AOAbtn
        params_common[1] = OnOff
        params_common[6] = PIDbtn
        params_common[10] = FullDrivebtn
        PultParams.data = params_common
        pubP.publish(PultParams)
        PultSpeed.twist.linear.x, PultSpeed.twist.angular.z = \
            representSpeed(M1, M2, params_common[3])
        PultSpeed.header.stamp = timeComp
        pubC.publish(PultSpeed)
        rate.sleep()
        #print(rospy.Time.now())

if __name__ == '__main__':
    try:
        mainfunc()
    except rospy.ROSInterruptException:
        pass

