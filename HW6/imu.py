# Tony Florida
# 2014-10-28
# JHU RTSW HW 6 - IMU

from Adafruit_I2C import Adafruit_I2C
from time import sleep
import math

# Filter enum
ROLL = 0
PITCH = 1
YAW = 2

# Filter index
index = 0

# Filter array len
FIlTER_SIZE = 5

# Filters
roll_filter=[0,0,0,0,0]
pitch_filter=[0,0,0,0,0]
yaw_filter=[0,0,0,0,0]

def median_filter( filt,  val ):
        if filt == ROLL:
                roll_filter[index]=val
                sorted(roll_filter)
                return roll_filter[2]
        elif filt == PITCH:
                pitch_filter[index]=val
                sorted(pitch_filter)
                return pitch_filter[2]
        elif filt == YAW:
                yaw_filter[index]=val
                sorted(yaw_filter)
                return yaw_filter[2]
        else:
                print "Filter error"
                return 0;

# initialize i2c connection to MPU6050
# i2c address is 0x68
i2c = Adafruit_I2C(0x68)

# wake up the device (out of sleep mode)
# bit 6 on register 0x6B set to 0
i2c.write8(0x6B, 0)

print "Roll   Pitch  Yaw"

while True:
        #Accel
        #read i2c accelerations in m/s^2
        ax = (i2c.readS8(0x3b)*256+i2c.readU8(0x3c))/16384.0
        ay = (i2c.readS8(0x3d)*256+i2c.readU8(0x3e))/16384.0
        az = (i2c.readS8(0x3f)*256+i2c.readU8(0x40))/16384.0
        #print "{:+03.3f}".format(ax),
        #print "{:+03.3f}".format(ay),
        #print "{:+03.3f}".format(az)

        #Gyro
        #read i2c gyroscope in degree/s
        gx = (i2c.readS8(0x43)*256+i2c.readU8(0x44))/131.0
        gy = (i2c.readS8(0x45)*256+i2c.readU8(0x46))/131.0
        gz = (i2c.readS8(0x47)*256+i2c.readU8(0x48))/131.0
        #print "{:+03.3f}".format(gx),
        #print "{:+03.3f}".format(gy),
        #print "{:+03.3f}".format(gz)

        #Calculate Roll, Pitch, Yaw as degrees
        # ** is exponent in Python
        #To get Radians, remove the... * 180) / math.pi
        pitch = (math.atan(ax / math.sqrt(ay ** 2 + az ** 2)) * 180) / math.pi
        roll = (math.atan(ay / math.sqrt(az ** 2 + az ** 2)) * 180) / math.pi
        yaw = gz

        #Median filter
        roll = median_filter(ROLL, roll)
        pitch = median_filter(PITCH, pitch)
        yaw = median_filter(YAW, yaw)

        print "{:+03.3f}".format(roll),
        print "{:+03.3f}".format(pitch),
        print "{:+03.3f}".format(yaw)

        index = index + 1
        if index >= FIlTER_SIZE:
                index = 0

        sleep(0.01)	
