import utime
import json
import network
from umqtt.simple import MQTTClient
import machine
import time
import math
from machine import Pin,I2C

i2c = I2C(scl = Pin(5),sda = Pin(4),freq = 500000)
addr = i2c.scan()[0]

#Setting thresholdS

THRESHOLD = 1
N_LARGEST_VALUES = 10

#---------Declaring the registers to read data from or write data to.-----------

CTRL_REG1 = 0x20 #Control the sampling rate and take the board off low power mode.
LIS3DH_REG_CTRL4 = 0x23
STATUS_REG = 0x27 #Displays the status of the boards operation
OUT_X_L = 0x28 #LsB of X-Axis
OUT_X_H = 0x29 #MsB of X-Axis
OUT_Y_L = 0x2A #LsB of Y-Axis
OUT_Y_H = 0x2B #MsB of Y-Axis
OUT_Z_L = 0x2C #LsB of Z-Axis
OUT_Z_H = 0x2D #MsB of Z-Axis


#Take the board off low power mode.
i2c.writeto_mem(addr,CTRL_REG1,bytearray([71])) #71 means 1 Hz Sampling rate.

#See if the board has been taken off low power mode (change later)
#print(i2c.readfrom_mem(addr,CTRL_REG1,1)) #Uncomment line for CTRL_REG1 value

#The Most Significant Hex indicates if Data has been overwritten
#Lease Significant Hex indicates if new Z,Y,X data is available (in that order)

#set the range
set_range = 0
set_range1 = set_range * 16
i2c.writeto_mem(addr,LIS3DH_REG_CTRL4,bytearray([set_range1]))

#----------Functions to perform conversion/standardisation----------------------

def raw_acceleration(n_h,n_l): #This function takes in input two decimal numbers
    return n_l + n_h * 256    #and shifts the bits to make out_dir_h the MsB.

def twos_comp(val, bits):
    #compute the 2's compliment of int value val
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val

#-------------Set the range and standardisation values for m/s^2----------------

#get the range
get_range = (ord(i2c.readfrom_mem(addr,LIS3DH_REG_CTRL4,1)) / 16)
#range_acc = ord(i2c.readfrom_mem(addr,LIS3DH_REG_CTRL4,1))

divider = 1
if (get_range == 3): divider = 1365
if (get_range == 2): divider = 4096
if (get_range == 1): divider = 8190
if (get_range == 0): divider = 16380

#-------------------Collecting Raw Data from the sensor-------------------------
#ensure that FIFO Bypass mode is enabled

def normalise(l, h, divider):
    
    acc_l_ord = ord(x_acc_l)
    acc_h_ord = ord(x_acc_h)

    acceleration = raw_acceleration(acc_h_ord, acc_l_ord)
    acceleration_signed = twos_comp(acceleration, 16)

    normalised_acc  = acceleration_signed / divider    

    return normalised_acc

while(switch is on):
    
    firstXvalue = normalise(i2c.readfrom_mem(addr,OUT_X_L,1), i2c.readfrom_mem(addr,OUT_X_H,1), divider)
    firstYvalue = normalise(i2c.readfrom_mem(addr,OUT_Y_L,1), i2c.readfrom_mem(addr,OUT_Y_H,1), divider)
    firstZvalue = normalise(i2c.readfrom_mem(addr,OUT_Z_L,1), i2c.readfrom_mem(addr,OUT_Z_H,1), divider)

    secondXvalue = normalise(i2c.readfrom_mem(addr,OUT_X_L,1), i2c.readfrom_mem(addr,OUT_X_H,1), divider)
    secondYvalue = normalise(i2c.readfrom_mem(addr,OUT_Y_L,1), i2c.readfrom_mem(addr,OUT_Y_H,1), divider)
    secondZvalue = normalise(i2c.readfrom_mem(addr,OUT_Z_L,1), i2c.readfrom_mem(addr,OUT_Z_H,1), divider)

    diffX = abs(secondXvalue - firstXvalue)
    diffY = abs(secondYvalue - firstYvalue)
    diffZ = abs(secondZvalue - firstZvalue)

    countX = 0
    countY = 0
    countZ = 0
    nLargestXValues = [0]*N_LARGEST_VALUES
    nLargestYValues = [0]*N_LARGEST_VALUES
    nLargestZValues = [0]*N_LARGEST_VALUES

    if(diffX > THRESHOLD):
        if(countX > N_LARGEST_VALUES - 1):
            if(diffX > nLargestXValues[0]):
                nLargestXValues[0] = diffX
                nLargestXValues.sort()
        else:
            nLargestXValues.append(diffX)
            nLargestXValues.sort()
            countX += 1
    
    if(diffY > THRESHOLD):
        if(countY > N_LARGEST_VALUES - 1):
            if(diffY > nLargestYValues[0]):
                nLargestYValues[0] = diffY
                nLargestYValues.sort()
        else:
            nLargestYValues.append(diffY)
            nLargestYValues.sort()
            countY += 1

    if(diffZ > THRESHOLD):
        if(countZ > N_LARGEST_VALUES - 1):
            if(diffZ > nLargestZValues[0]):
                nLargestZValues[0] = diffZ
                nLargestZValues.sort()
        else:
            nLargestZValues.append(diffZ)
            nLargestZValues.sort()
            countZ += 1         

#when switch is turned off, put these nLargestX,Y,ZValues data structures onto the broker

#------------Network Management and JSON display formatting---------------------
#-----------------Convert to appropriate display format.------------------------
x_acceleration = [str(normalised_x_acc),"m/s^2"]
x = ' '.join(x_acceleration)
y_acceleration = [str(normalised_y_acc),"m/s^2"]
y = ' '.join(y_acceleration)
z_acceleration = [str(normalised_z_acc),"m/s^2"]
z = ' '.join(z_acceleration)

#-------------------Networking the data-----------------------------------------

#Set up the LED for control
led = machine.Pin(16,machine.Pin.OUT)
led.low()


sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)

sta_if.connect('EEERover','exhibition')

#Until it connects
while(not sta_if.isconnected()):
    pass

#if a connection was established
if(sta_if.isconnected()):
    led.high()

time.sleep(0.5)
led.low()

#Connecting to the MQTT Broker
class get_current_time():
    def __init__(self):
        self.time = 0

    def update_time(self,msg):
        self.time = msg
    def printcurrenttime(self):
        print(self.time)

def sub_cb(topic,msg):
    stott_time = str(msg,'utf-8')
    print((topic,msg))
    currenttime.update_time(msg)



client = MQTTClient('unnamed1','192.168.0.10')
client.connect()
currenttime = get_current_time()
client.set_callback(sub_cb)
client.subscribe(b'esys/time')
client.wait_msg()
time.sleep(1)


#---------------The dictionary for the acceleration values.---------------------
acc_data = {
    'x acceleration' : x,
    'y acceleration' : y,
    'z acceleration' : z,
    'time' : currenttime.time
}

payload = json.dumps(acc_data)

client.publish('/unnamed1/test',bytes(payload, 'utf-8'))

#create a file

#f = open('data.txt', 'w')
#f.write('some data')
#f.close()

g = open('data.txt')
print(g.read())
g.close()

#topic for time, esys\time
