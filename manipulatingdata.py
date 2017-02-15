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

def minimum1(nLargestValues):
    minValueIndex = 0
    minValue = nLargestValues[0]
    for i in range (0, len(nLargestValues)):
        if(nLargestValues[i] < minValue):
            minValue = nLargestValues[i]
            minValueIndex = i
    return minValueIndex    
