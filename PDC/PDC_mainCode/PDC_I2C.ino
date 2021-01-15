// TODO can we monitor ACK to register a successful communication?
// TODO still need to better understand arduino I2C - not something i've encountered before!
/**
 * @brief  Read a register over I2C
 * @param  the I2C address of the device that we want to read
 * @param  the address of the register that we want to read on the selected device
 * @param  the number of bytes that we want to read in this transaction
 * @param  the value that is returned by the device
 */
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
