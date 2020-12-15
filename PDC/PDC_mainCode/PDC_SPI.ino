/* read a value from a register of a device on SPI. as arguments, pass the device select pin, the address of the register, and the number of bytes that this
   register contains. it will return the value that is stored in the register that we are reading */
unsigned int readSPI(int deviceSelect, byte registerSelect, int numBytes) {

  /* variable for our register value return */
  unsigned int result = 0;

  /* the 'read' or 'write' bit is in different positions for different devices but then the register address occupies the rest of the field,
       so we can shift the register up into place when the R/W occupies LSB
       read = 1, write = 0 */
  if (deviceSelect == IMU_SS) {
    /* r/w in LSB spot */
    registerSelect = (registerSelect << 1) | 1;
  }
  else if (deviceSelect == altimeter_SS) {
    /* r/w in MSB spot */
    registerSelect = registerSelect | (1 << 8);
  }
  else {
    Serial.println("ERROR: device does not exist on SPI");
  }

  /* begin a transaction over SPI using our params. this command also stops interrupts from preventing SPI comms */
  SPI.beginTransaction(SPIParams);

  /* to communicate with the device, we take its slave select pin on the PDC low */
  digitalWrite(deviceSelect, LOW);

  /* if we want to read a particular address, we must send the address of the register to the device */
  SPI.transfer(registerSelect);

  /* now if we send nothing, we are listening for the result - the device will send the value in the register we requested for the first byte, just read the value into 'result' */
  result = SPI.transfer(0x00);
  /* decrement the number of bytes that we have left to read */
  numBytes--;

  while (numBytes != 0) {
    /* if we have more than one byte to read, shift the result so far up by a byte, and fill the empty space with our new byte */
    result = (result << 8) | SPI.transfer(0x00);
    /* decrement the number of bytes until we get to zero, when this while() will exit */
    numBytes--;
  }

  /* stop communications with device by setting the corresponding slave select on the PDC to high */
  digitalWrite(deviceSelect, HIGH);

  /* we're done now! restart interrupt mechanisms */
  SPI.endTransaction();

  /* send our address value back to the caller */
  return (result);
}
