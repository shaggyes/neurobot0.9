#!/usr/bin/env python
import sys
import time
import rospy
from std_msgs.msg import Float32MultiArray  # some meuserements?
from std_msgs.msg import String  # error msg
from sensor_msgs.msg import LaserScan  # Ultrasonic Scan 6x
from ctypes import c_short
from ctypes import c_byte
from ctypes import c_ubyte
from smbus import SMBus
addri2cSonars = 0x8
addri2cbarometer = 0x77
bus = SMBus(1)

#  bme280 not bmp180
def filter(sensor, prev, filtConst):
    return ((sensor-prev)*filtConst + (1-filtConst)*prev)

def getShort(data, index):
    # return two bytes from data as a signed 16-bit value
    return c_short((data[index+1] << 8) + data[index]).value

def getUShort(data, index):
    # return two bytes from data as an unsigned 16-bit value
    return (data[index+1] << 8) + data[index]

def getChar(data,index):
    # return one byte from data as a signed char
    result = data[index]
    if result > 127:
        result -= 256
    return result

def getUChar(data,index):
    # return one byte from data as an unsigned char
    result =  data[index] & 0xFF
    return result

def readBME280ID(addr=addri2cbarometer):
    # Chip ID Register Address
    REG_ID     = 0xD0
    (chip_id, chip_version) = bus.read_i2c_block_data(addr, REG_ID, 2)
    return (chip_id, chip_version)

def readBME280All(addr=addri2cbarometer):
    # Register Addresses
    REG_DATA = 0xF7
    REG_CONTROL = 0xF4
    REG_CONFIG  = 0xF5

    REG_CONTROL_HUM = 0xF2
    REG_HUM_MSB = 0xFD
    REG_HUM_LSB = 0xFE

    # Oversample setting - page 27
    OVERSAMPLE_TEMP = 2
    OVERSAMPLE_PRES = 2
    MODE = 1

    # Oversample setting for humidity register - page 26
    OVERSAMPLE_HUM = 2
    bus.write_byte_data(addr, REG_CONTROL_HUM, OVERSAMPLE_HUM)

    control = OVERSAMPLE_TEMP<<5 | OVERSAMPLE_PRES<<2 | MODE
    bus.write_byte_data(addr, REG_CONTROL, control)

    # Read blocks of calibration data from EEPROM
    # See Page 22 data sheet
    cal1 = bus.read_i2c_block_data(addr, 0x88, 24)
    cal2 = bus.read_i2c_block_data(addr, 0xA1, 1)
    cal3 = bus.read_i2c_block_data(addr, 0xE1, 7)

    # Convert byte data to word values
    dig_T1 = getUShort(cal1, 0)
    dig_T2 = getShort(cal1, 2)
    dig_T3 = getShort(cal1, 4)

    dig_P1 = getUShort(cal1, 6)
    dig_P2 = getShort(cal1, 8)
    dig_P3 = getShort(cal1, 10)
    dig_P4 = getShort(cal1, 12)
    dig_P5 = getShort(cal1, 14)
    dig_P6 = getShort(cal1, 16)
    dig_P7 = getShort(cal1, 18)
    dig_P8 = getShort(cal1, 20)
    dig_P9 = getShort(cal1, 22)

    dig_H1 = getUChar(cal2, 0)
    dig_H2 = getShort(cal3, 0)
    dig_H3 = getUChar(cal3, 2)

    dig_H4 = getChar(cal3, 3)
    dig_H4 = (dig_H4 << 24) >> 20
    dig_H4 = dig_H4 | (getChar(cal3, 4) & 0x0F)

    dig_H5 = getChar(cal3, 5)
    dig_H5 = (dig_H5 << 24) >> 20
    dig_H5 = dig_H5 | (getUChar(cal3, 4) >> 4 & 0x0F)

    dig_H6 = getChar(cal3, 6)

    # Wait in ms (Datasheet Appendix B: Measurement time and current calculation)
    wait_time = 1.25 + (2.3 * OVERSAMPLE_TEMP) + ((2.3 * OVERSAMPLE_PRES) + 0.575) + ((2.3 * OVERSAMPLE_HUM)+0.575)
    time.sleep(wait_time/1000)  # Wait the required time  

    # Read temperature/pressure/humidity
    data = bus.read_i2c_block_data(addr, REG_DATA, 8)
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4)
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4)
    hum_raw = (data[6] << 8) | data[7]

    #Refine temperature
    var1 = ((((temp_raw>>3)-(dig_T1<<1)))*(dig_T2)) >> 11
    var2 = (((((temp_raw>>4) - (dig_T1)) * ((temp_raw>>4) - (dig_T1))) >> 12) * (dig_T3)) >> 14
    t_fine = var1+var2
    temperature = float(((t_fine * 5) + 128) >> 8);

    # Refine pressure and adjust for temperature
    var1 = t_fine / 2.0 - 64000.0
    var2 = var1 * var1 * dig_P6 / 32768.0
    var2 = var2 + var1 * dig_P5 * 2.0
    var2 = var2 / 4.0 + dig_P4 * 65536.0
    var1 = (dig_P3 * var1 * var1 / 524288.0 + dig_P2 * var1) / 524288.0
    var1 = (1.0 + var1 / 32768.0) * dig_P1
    if var1 == 0:
        pressure=0
    else:
        pressure = 1048576.0 - pres_raw
        pressure = ((pressure - var2 / 4096.0) * 6250.0) / var1
        var1 = dig_P9 * pressure * pressure / 2147483648.0
        var2 = pressure * dig_P8 / 32768.0
        pressure = pressure + (var1 + var2 + dig_P7) / 16.0

    return temperature/100.0, pressure/100.0


def mainfunc():
    global bus
    print('step1')

    rospy.init_node('nbcoredata', anonymous=True)
    pubB = rospy.Publisher('neurobotbarometer', Float32MultiArray, queue_size=1)
    pubUS = rospy.Publisher('neurobotsonarscan', LaserScan, queue_size=1)
    # pubError = rospy.Publisher('neuroboterror', String, queue_size=1)
    rate = rospy.Rate(4)  # 4 - 10hz

    CoreBar = Float32MultiArray()
    CoreScan = LaserScan()
    CoreScan.range_min = 0.02
    CoreScan.range_max = 2.56
    CoreScan.scan_time = 0.3
    CoreScan.angle_increment = 1.04719
    CoreScan.angle_min = -2.09439
    CoreScan.angle_max = 3.1415
    # Error = String()
    #a = bus.read_byte(addri2cSonars)
    #a = bus.read_byte(addri2cSonars)
    #a = bus.read_byte(addri2cSonars)
    #a = bus.read_byte(addri2cSonars)
    #a = bus.read_byte(addri2cSonars)
    print('step2')

    while not rospy.is_shutdown():
        temperature, pressure = readBME280All()
        i2cstr = []
        for i in range(7):
            i2cstr.append(bus.read_byte(addri2cSonars))  # six values in cm
	#print(i2cstr)
        CoreScan.ranges = [float(i2cstr[0])/100, float(i2cstr[1])/100,
                           float(i2cstr[4])/100, float(i2cstr[5])/100,
                           float(i2cstr[3])/100, float(i2cstr[2])/100]

        #rospy.loginfo(CoreScan)
        CoreBar.data = [temperature, pressure]
        pubB.publish(CoreBar)
        pubUS.publish(CoreScan)
        #Error.data = "none"
        #pubError.publish(Error)
        rate.sleep()


if __name__ == '__main__':
    try:
        mainfunc()
    except rospy.ROSInterruptException:
        pass

