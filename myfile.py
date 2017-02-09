from machine import Pin, I2C
import utime

i2c = I2C(scl = Pin(5),sda = Pin(4),freq = 500000)
addr = i2c.scan()[0]

#Declaring the registers to read data from or write data to.

CTRL_REG1 = 0x20 #Control the sampling rate and take the board off low power mode.
STATUS_REG = 0x27
OUT_X_L = 0x28 #LsB of X-Axis
OUT_X_H = 0x29 #MsB of X-Axis
OUT_Y_L = 0x2A #LsB of Y-Axis
OUT_Y_H = 0x2B #MsB of Y-Axis
OUT_Z_L = 0x2C #LsB of Z-Axis
OUT_Z_H = 0x2D #MsB of Z-Axis

LIS3DH_REG_CTRL4 = 0x23

#range
RANGE_16G = 48
RANGE_8G = 32
RANGE_4G = 16
RANGE_2G = 0

READING_SIZE_IN_BITS = 16

#Take the board off low power mode.
i2c.writeto_mem(addr,CTRL_REG1,bytearray([71]))

#See if the board has been taken off low power mode (change later)
print(i2c.readfrom_mem(addr,CTRL_REG1,1))

#The Most Significant Hex indicates if Data has been overwritten
#Lease Significant Hex indicates if new Z,Y,X data is available (in that order)
print("Status Register is:" ,i2c.readfrom_mem(addr,STATUS_REG,1))

#set the range
set_range = RANGE_16G
print("set_range is: ", set_range)
i2c.writeto_mem(addr,LIS3DH_REG_CTRL4,bytearray([set_range]))

#Functions to perform conversion/standardisation

def raw_acceleration(n_h,n_l): #This function takes in input two decimal numbers
    return n_l + n_h * 256

def twos_complement(value):
    if (value & (1 << (READING_SIZE_IN_BITS - 1))) != 0: #if MSB is 1
        value = value - (1 << READING_SIZE_IN_BITS)      #compute negative
    return value

#get the range
get_range = (ord(i2c.readfrom_mem(addr,LIS3DH_REG_CTRL4,1)) / 16)
#range_acc = ord(i2c.readfrom_mem(addr,LIS3DH_REG_CTRL4,1))
print("range is = ", get_range)

divider = 1
if (get_range == 3): divider = 1365
if (get_range == 2): divider = 4096
if (get_range == 1): divider = 8190
if (get_range == 0): divider = 16380

#RAW DATA
# X acceleration
x_acc_l = i2c.readfrom_mem(addr,OUT_X_L,1)
x_acc_h = i2c.readfrom_mem(addr,OUT_X_H,1)

x_acc_l_ord = ord(x_acc_l)
x_acc_h_ord = ord(x_acc_h)
#x_acc_h_ord = 128

x_acceleration = raw_acceleration(x_acc_h_ord,x_acc_l_ord)
x_acceleration_signed = twos_complement(x_acceleration)

normalised_x_acc  = x_acceleration_signed / divider

# Y acceleration
y_acc_l = i2c.readfrom_mem(addr,OUT_Y_L,1)
y_acc_h = i2c.readfrom_mem(addr,OUT_Y_H,1)

y_acc_l_ord = ord(y_acc_l)
y_acc_h_ord = ord(y_acc_h)
#y_acc_h_ord = 128

y_acceleration = raw_acceleration(y_acc_h_ord,y_acc_l_ord)
y_acceleration_signed = twos_complement(y_acceleration)

normalised_y_acc  = y_acceleration_signed / divider

# Z acceleration
z_acc_l = i2c.readfrom_mem(addr,OUT_Z_L,1)
z_acc_h = i2c.readfrom_mem(addr,OUT_Z_H,1)

z_acc_l_ord = ord(z_acc_l)
z_acc_h_ord = ord(z_acc_h)
#z_acc_h_ord = 128

z_acceleration = raw_acceleration(z_acc_h_ord,z_acc_l_ord)
z_acceleration_signed = twos_complement(z_acceleration)

normalised_z_acc  = z_acceleration_signed / divider

#Converting the raw data

#CTRL REG 1 ENABLED, Now 1Hz sampling.
print("X = ",x_acc_h,x_acc_l)
print("X = ",x_acc_h_ord,x_acc_l_ord)
print("x acceleration = ",x_acceleration)
print("x acceleration_signed = ", x_acceleration_signed)
print("normalised_x_acc = ", normalised_x_acc)

print("Y = ",y_acc_h,y_acc_l)
print("Y = ",y_acc_h_ord,y_acc_l_ord)
print("y acceleration = ",y_acceleration)
print("y acceleration_signed = ", y_acceleration_signed)
print("normalised_y_acc = ", normalised_y_acc)

print("Z = ",z_acc_h,z_acc_l)
print("Z = ",z_acc_h_ord,z_acc_l_ord)
print("z acceleration = ",z_acceleration)
print("z acceleration_signed = ", z_acceleration_signed)
print("normalised_z_acc = ", normalised_z_acc)

#x_acc = x_acc_h
#print(x_acc)
#x_acc.append(x_acc_l[0])
