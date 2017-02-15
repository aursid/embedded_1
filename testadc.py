import machine
import time

adc = machine.ADC(0)
print(adc.read()*(3.3/1023))
