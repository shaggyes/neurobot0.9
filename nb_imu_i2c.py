#!/usr/bin/python

'''
  Here is some sample data from the MPU-9255 IMU.
  {'pressureValid': False, 
    'accelValid': True, 
    'temperature': 0.0, 
    'pressure': 0.0, 
    'fusionQPoseValid': True, 
    'timestamp': 1471314050076560L, 
    'compassValid': True, 
    'compass': (26.2247257232666, 36.678741455078125, -17.60536003112793), 
    'accel': (0.0107421875, 1.013427734375, -0.03369140625), 
    'humidity': 0.0, 'gyroValid': True, 
    'gyro': (0.00553120207041502, -0.0031295656226575375, 0.0031766612082719803), 
    'temperatureValid': False, 
    'humidityValid': False, 
    'fusionQPose': (0.6710100769996643, 0.6879799365997314, -0.20192061364650726, -0.18883132934570312), 
    'fusionPoseValid': True, 
    'fusionPose': (1.5989785194396973, -0.011157416738569736, -0.5601144433021545)}
'''

import sys, getopt
sys.path.append('.')
import RTIMU
import os.path
import math
import rospy
from std_msgs.msg import String  # error msg
from sensor_msgs.msg import Imu  # Gyro and acel
from sensor_msgs.msg import MagneticField  # Magnetic field
from geometry_msgs.msg import Vector3  # for acel & gyro
from smbus import SMBus
addri2cimu = 0x68
bus = SMBus(1)

SETTINGS_FILE = "RTIMULib.ini"  
   
s = RTIMU.Settings(SETTINGS_FILE)
imu = RTIMU.RTIMU(s)  
   
if (not imu.IMUInit()):  
  sys.exit(1)  
   
imu.setSlerpPower(0.02)  
imu.setGyroEnable(True)  
imu.setAccelEnable(True)  
imu.setCompassEnable(True)  


def filter(sensor, prev, filtConst):
    return ((sensor-prev)*filtConst + (1-filtConst)*prev)


def mainfunc():

    print('step1')
    data = dict()
    if imu.IMURead():  
        data = imu.getIMUData()
    rospy.init_node('nbcoreimu', anonymous=True)
    pubI = rospy.Publisher('neurobotimu', Imu, queue_size=1)
    pubM = rospy.Publisher('neurobotcompas', MagneticField, queue_size=1)
    # pubError = rospy.Publisher('neuroboterror', String, queue_size=1)
    rate = rospy.Rate(100)  # 4 - 10hz

    CoreM = MagneticField()
    CoreI = Imu()
    # Error = String()
    print('step2')
    gyro = [0, 0, 0]
    acel = [0, 0, 0]
    while not rospy.is_shutdown():
        gyro = [0, 0, 0]
        acel = [0, 0, 0]
        if imu.IMURead():  
            data = imu.getIMUData()  
            gyro = data['gyro']
            acel = data['accel']
        magnetic = data['compass']
        angle = (math.atan2(magnetic[0],magnetic[1])*180/math.pi)
        #print(angle)
        #print(gyro)
        #print(accel)
	CoreI.orientation.w = angle
        CoreI.angular_velocity = Vector3(gyro[0], gyro[1], gyro[2])
        CoreI.linear_acceleration = Vector3(acel[0], acel[1], acel[2])
        CoreM.magnetic_field = Vector3(magnetic[0], magnetic[1], magnetic[2])
        pubM.publish(CoreM)
        pubI.publish(CoreI)
        #Error.data = "none"
        #pubError.publish(Error)
        rate.sleep()


if __name__ == '__main__':
    try:
        mainfunc()
    except rospy.ROSInterruptException:
        pass

