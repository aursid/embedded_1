import json
import network
from umqtt.simple import MQTTClient
import machine
import time
from machine import Pin,I2C,RTC

THRESHOLD = 0.05
N_LARGEST_VALUES = 3

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

#This class gets the current value of the time to display in RFC format to use
#in the rest of the program.
class get_current_time():
    def __init__(self):
        self.time = 0

    def update_time(self,msg):
        self.time = msg
    def printcurrenttime(self):
        print(self.time)

def sub_cb(topic,msg):
    stott_time = str(msg,'utf-8')
    currenttime.update_time(stott_time)

def display_date_time(year,month,day,weekday,hour,minute,second):
    date_str = ("%d:%d:%d %d %d/%d/%d" %(hour,minute,second,weekday,day,month,year))
    return date_str

def display_date_time_tuple(timetuple):

    date_string = ("%d:%d:%d %d/%d/%d" %(timetuple[4],timetuple[5],timetuple[6],timetuple[2],timetuple[1],timetuple[0]))
    return date_string

def minimum1(nLargestValues):
    minValueIndex = 0
    minValue = nLargestValues[0]
    for i in range (0, len(nLargestValues)):
        if(nLargestValues[i] < minValue):
            minValue = nLargestValues[i]
            minValueIndex = i
    return minValueIndex

#----------Functions to perform conversion/standardisation----------------------

def raw_acceleration(n_h,n_l): #This function takes in input two decimal numbers
    return n_l + n_h * 256    #and shifts the bits to make out_dir_h the MsB.

def twos_comp(val, bits):
    #compute the 2's compliment of int value val
    if (val & (1 << (bits - 1))) != 0: # if sign bit is set e.g., 8bit: 128-255
        val = val - (1 << bits)        # compute negative value
    return val

def normalise(l, h, divider):

    acc_l_ord = ord(l)
    acc_h_ord = ord(h)

    acceleration = raw_acceleration(acc_h_ord, acc_l_ord)
    acceleration_signed = twos_comp(acceleration, 16)

    normalised_acc  = acceleration_signed / divider

    return normalised_acc

#Setting up the LED for control
led = machine.Pin(16,machine.Pin.OUT)
led.low()

#--------------------------Networking the data----------------------------------

#Connect to the EEERover
sta_if = network.WLAN(network.STA_IF)
sta_if.active(True)

sta_if.connect('EEERover','exhibition')

#Until it connects
while(not sta_if.isconnected()):
    pass

time.sleep(0.5)

#-----------Connecting to the MQTT Broker for getting the time -----------------

client = MQTTClient('unnamed1','192.168.0.10')
client.connect()
currenttime = get_current_time()
client.set_callback(sub_cb)
client.subscribe(b'esys/time')
client.wait_msg()
time.sleep(1)

#To extract the data from esys/time to covert to integer format
time_python = []
time_python = json.loads(currenttime.time) #time as a python dict
time_right_now = time_python["date"] #Gets the RFC string of the time at that instant

if(len(time_python) > 0):
    led.high()

#Parse the time to place in the appropriate tuple for Machine.RTC
year = int(time_right_now[0:4])
month = int(time_right_now[5:7])
day = int(time_right_now[8:10])
hour = int(time_right_now[11:13])
minute = int(time_right_now[14:16])
second = int(time_right_now[17:19])
weekday = 4

rtc = machine.RTC()

rtc.datetime((year,month,day,weekday,hour,minute,second,0))
print(rtc.datetime())

sta_if.active(False)


#Find out which sensor is connected to the NodeMCU
i2c = I2C(scl = Pin(5),sda = Pin(4),freq = 500000)
addr_list = i2c.scan() #for switch control.


#START SENSING WHEN THERE IS AN ADDRESS
while(len(addr_list) == 0):
    addr_list = i2c.scan()
    if(len(addr_list) > 0):
       break
    time.sleep(1)

#Will assign an addr if a sensor is connected.

addr = addr_list[0]

#Take the board off power down mode and decide sampling rate
one_hz_sampling = 23
ten_hz_sampling = 39
twentyfive_hz_sampling = 55
fifty_hz_sampling = 71
hundred_hz_sampling = 87

i2c.writeto_mem(addr,CTRL_REG1,bytearray([fifty_hz_sampling])) #71 means 50 Hz Sampling rate.

#See if the board has been taken off low power mode (change later)
#print(i2c.readfrom_mem(addr,CTRL_REG1,1)) #Uncomment line for CTRL_REG1 value

#The Most Significant Hex indicates if Data has been overwritten
#Lease Significant Hex indicates if new Z,Y,X data is available (in that order)

#set the range
set_range = 0
set_range1 = set_range * 16
i2c.writeto_mem(addr,LIS3DH_REG_CTRL4,bytearray([set_range1]))



#-------------Set the range and standardisation values for g--------------------

#get the range
get_range = (ord(i2c.readfrom_mem(addr,LIS3DH_REG_CTRL4,1)) / 16)

divider = 1
if (get_range == 3): divider = 1365
if (get_range == 2): divider = 4096
if (get_range == 1): divider = 8190
if (get_range == 0): divider = 16380

#-------------------Collecting Raw Data from the sensor-------------------------
countX = 0
countY = 0
countZ = 0
nLargestXValues = []
nLargestYValues = []
nLargestZValues = []
nLargestXValues_time = []
nLargestYValues_time = []
nLargestZValues_time = []

while(len(addr_list) > 0):

    firstXvalue = normalise(i2c.readfrom_mem(addr,OUT_X_L,1), i2c.readfrom_mem(addr,OUT_X_H,1), divider)
    firstYvalue = normalise(i2c.readfrom_mem(addr,OUT_Y_L,1), i2c.readfrom_mem(addr,OUT_Y_H,1), divider)
    firstZvalue = normalise(i2c.readfrom_mem(addr,OUT_Z_L,1), i2c.readfrom_mem(addr,OUT_Z_H,1), divider)

    secondXvalue = normalise(i2c.readfrom_mem(addr,OUT_X_L,1), i2c.readfrom_mem(addr,OUT_X_H,1), divider)
    secondYvalue = normalise(i2c.readfrom_mem(addr,OUT_Y_L,1), i2c.readfrom_mem(addr,OUT_Y_H,1), divider)
    secondZvalue = normalise(i2c.readfrom_mem(addr,OUT_Z_L,1), i2c.readfrom_mem(addr,OUT_Z_H,1), divider)

    diffX = abs(secondXvalue - firstXvalue)
    diffY = abs(secondYvalue - firstYvalue)
    diffZ = abs(secondZvalue - firstZvalue)

    if(diffX > THRESHOLD):
        if(countX > N_LARGEST_VALUES - 1):
            indexX = minimum1(nLargestXValues)
            if(diffX > nLargestXValues[indexX]):
                nLargestXValues[indexX] = diffX
                nLargestXValues_time[indexX] = rtc.datetime()
        else:
            nLargestXValues.append(diffX)
            nLargestXValues_time.append(rtc.datetime())
            countX += 1

    if(diffY > THRESHOLD):
        if(countY > N_LARGEST_VALUES - 1):
            indexY = minimum1(nLargestYValues)
            if(diffY > nLargestYValues[indexY]):
                nLargestYValues[indexY] = diffY
                nLargestYValues_time[indexY] = rtc.datetime()
        else:
            nLargestYValues.append(diffY)
            nLargestYValues_time.append(rtc.datetime())
            countY += 1

    if(diffZ > THRESHOLD):
        if(countZ > N_LARGEST_VALUES - 1):
            indexZ = minimum1(nLargestZValues)
            if(diffZ > nLargestZValues[indexZ]):
                nLargestZValues[indexZ] = diffZ
                nLargestZValues_time[indexZ] = rtc.datetime()
        else:
            nLargestZValues.append(diffZ)
            nLargestZValues_time.append(rtc.datetime())
            countZ += 1

    #print('in while loop')
    #if(len(addr_list) == 0):
    #   break
    time.sleep(0.5)
    addr_list = i2c.scan()

#when switch is turned off, put these nLargestX,Y,ZValues data structures onto the broker
print(nLargestXValues,nLargestYValues,nLargestZValues)
print(nLargestXValues_time,nLargestYValues_time,nLargestZValues_time)

#display_date_time_tuple(rtc.datetime())


#------------Network Management and JSON display formatting---------------------
#-----------------Convert to appropriate display format.------------------------
#puts g in front of the number and puts them in a string like form.
x = 'g '.join(map(str,nLargestXValues))
y = 'g '.join(map(str,nLargestYValues))
z = 'g '.join(map(str,nLargestZValues))

x_acc = "Largest 3 x acceleration values: " + x
y_acc = "Largest 3 y acceleration values: " + y
z_acc = "Largest 3 z acceleration values: " + z
acc_data = [x_acc,y_acc,z_acc]

a = []
for k in range(0, len(nLargestXValues_time)):
    print(display_date_time_tuple(nLargestXValues_time[k]))

#--------------------Connect to the EEERover again------------------------------
sta_if.active(True)

sta_if.connect('EEERover','exhibition')
while(not sta_if.isconnected()):
    pass
time.sleep(0.5)

client = MQTTClient('unnamed1','192.168.0.10')
client.connect()
payload = json.dumps(acc_data)

#Send acceleration and time data to the broker

client.publish('/unnamed1/test',bytes(payload, 'utf-8'))
