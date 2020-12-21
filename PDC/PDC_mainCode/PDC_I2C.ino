/* read from a device on I2C, where the arguments are: the I2C address of the device */
void readI2C(uint8_t deviceAddress, uint8_t deviceRegister, uint8_t numBytes, uint8_t *result) {

  /* set the register pointer on the device to the specified location */
  Wire.beginTransmission(deviceAddress);
  Wire.write(deviceRegister);
  Wire.endTransmission();

  /* enable I2C communications by requesting data from our device of length numBytes */
  Wire.requestFrom(deviceAddress, numBytes);

  /* read a byte from the address for the specified number of bytes */
  for (uint8_t i = 0; i < numBytes; i++) {
    result[i] = Wire.read();
  }

  /* the input argument 'result' is an array of size numBytes, it is modified, instead of returned, since this is not possible */
}
