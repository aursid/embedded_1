  uint8_t deviceid = readRegister8(LIS3DH_REG_WHOAMI);
	  if (deviceid != 0x33)
	  {
	    /* No LIS3DH detected ... return false */
	    //Serial.println(deviceid, HEX);
	    return false;
	  }

// enable all axes, normal mode
	  writeRegister8(LIS3DH_REG_CTRL1, 0x07);
	  // 400Hz rate
	  setDataRate(LIS3DH_DATARATE_400_HZ);
	
	  // High res & BDU enabled
	  writeRegister8(LIS3DH_REG_CTRL4, 0x88);
	
	  // DRDY on INT1
	  writeRegister8(LIS3DH_REG_CTRL3, 0x10);
	
	  // Turn on orientation config
	  //writeRegister8(LIS3DH_REG_PL_CFG, 0x40);
	
	  // enable adcs
	  writeRegister8(LIS3DH_REG_TEMPCFG, 0x80);

void Adafruit_LIS3DH::read(void) {
  // read x y z at once

  if (_cs == -1) {
    // i2c
    Begin a transmission to the I2C slave device with the given address.
    Subsequently, queue bytes for transmission with the write() function and transmit them by calling endTransmission().

    The I2C embedded inside the LIS3DH behaves like a slave device and the following
    protocol must be adhered to. After the start condition (ST) a slave address is sent, once a
    slave acknowledge (SAK) has been returned, an 8-bit sub-address (SUB) is transmitted: the
    7 LSb represent the actual register address while the MSB enables address auto increment. 
    
    Wire.beginTransmission(_i2caddr);
    Wire.write(LIS3DH_REG_OUT_X_L | 0x80); // 0x80 for autoincrement
    Wire.endTransmission();

    Used by the master to request bytes from a slave device. The bytes may then be retrieved with the available() and read() functions.
    Wire.requestFrom(address, quantity)
    address: the 7-bit address of the device to request bytes from
    quantity: the number of bytes to request
    
    Wire.requestFrom(_i2caddr, 6);

    read() reads a byte that was transmitted from a slave device to a master after a call to requestFrom() or was transmitted from a master to a slave.

    a |= b; means the same thing as a = a | b;

    so what's happening is that first LIS3DH_REG_OUT_X_L is being read. then LIS3DH_REG_OUT_X_H is being read
    as a 16 bit uint. LIS3DH_REG_OUT_X_H is then being shifted 8 bits left (as it's the most significant byte).
    finally, LIS3DH_REG_OUT_X_L and LIS3DH_REG_OUT_X_H are being are ORRed to get the final value.
                                                                            
    (<< comes before |= in the c++ operator precedence) 
    (casting comes before << in the c++ operator precedence)
    (the device has 8 bit registers)                                                         

  
    x = Wire.read();
    x |= ((uint16_t)Wire.read()) << 8; means x = x | Wire.read()
    y = Wire.read();
    y |= ((uint16_t)Wire.read()) << 8;
    z = Wire.read();
    z |= ((uint16_t)Wire.read()) << 8;
  } 

  uint8_t range = getRange();
  uint16_t divider = 1;
  if (range == LIS3DH_RANGE_16_G) divider = 1365; // different sensitivity at 16g
  if (range == LIS3DH_RANGE_8_G) divider = 4096;
  if (range == LIS3DH_RANGE_4_G) divider = 8190;
  if (range == LIS3DH_RANGE_2_G) divider = 16380;

  x_g = (float)x / divider;
  y_g = (float)y / divider;
  z_g = (float)z / divider;

}

void Adafruit_LIS3DH::setRange(lis3dh_range_t range)
{
  uint8_t r = readRegister8(LIS3DH_REG_CTRL4);
  r &= ~(0x30);
  r |= range << 4;
  writeRegister8(LIS3DH_REG_CTRL4, r);
}

lis3dh_range_t Adafruit_LIS3DH::getRange(void)
{
  /* Read the data format register to preserve bits */
  return (lis3dh_range_t)((readRegister8(LIS3DH_REG_CTRL4) >> 4) & 0x03);
}

void Adafruit_LIS3DH::setDataRate(lis3dh_dataRate_t dataRate)
{
  uint8_t ctl1 = readRegister8(LIS3DH_REG_CTRL1);
  ctl1 &= ~(0xF0); // mask off bits
  ctl1 |= (dataRate << 4);
  writeRegister8(LIS3DH_REG_CTRL1, ctl1);
}

lis3dh_dataRate_t Adafruit_LIS3DH::getDataRate(void)
{
  return (lis3dh_dataRate_t)((readRegister8(LIS3DH_REG_CTRL1) >> 4)& 0x0F);
}

uint8_t Adafruit_LIS3DH::readRegister8(uint8_t reg) {
  uint8_t value;

  if (_cs == -1) {
    Wire.beginTransmission(_i2caddr);
    Wire.write((uint8_t)reg);
    Wire.endTransmission();

    Wire.requestFrom(_i2caddr, 1);
    value = Wire.read();
  }  
