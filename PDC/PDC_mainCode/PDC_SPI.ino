/* read a value from a register of a device on SPI. as arguments, pass the device select pin, the address of the register, and the number of bytes that this
   register contains. it will return the value that is stored in the register that we are reading */
// TODO: lookup 'shiftout()' - seen it mentioned as alternative(?) to SPI.Transfer?
// TODO: revisit the result for multiple bytes
unsigned int readSPI(int deviceSelect, byte registerSelect, int numBytes) {

  /* variable for our register value return */
  unsigned int result = 0;
  unsigned int counter = numBytes;

  /* the r/w bit is first to transfer so we can shift it up to the top of the register
       read = 1, write = 0 */
  registerSelect = registerSelect | (1 << 7);
  
  /* begin a transaction over SPI using our params. this command also stops interrupts from preventing SPI comms */
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));

  /* to communicate with the device, we take its slave select pin on the PDC low */
  digitalWrite(deviceSelect, LOW);

  /* if we want to read from a particular register, we must send the address of the register to the device */
  SPI.transfer(registerSelect);

  /* now if we send nothing, we are listening for the result - the device will send the value in the register we requested for the first byte, just read the value into 'result' */
  result = SPI.transfer(0x00);

  Serial.print("Register: ");
  Serial.print(registerSelect, BIN);
  Serial.print(" Value: ");
  Serial.println(result, BIN);
  /* decrement the number of bytes that we have left to read */
  counter--;

  while (counter != 0) {
    /* if we have more than one byte to read, shift the next result up to fill the MSB byte, and hold the result so far at the bottom
       this is because the registers are usually read in sequence of LSB -> MSB */
    result = (SPI.transfer(0x00) << 8 * (numBytes - counter)) | result;
    /* decrement the number of bytes until we get to zero, when this while() will exit */
    counter--;
  }

  /* stop communications with device by setting the corresponding slave select on the PDC to high */
  digitalWrite(deviceSelect, HIGH);

  /* we're done now! restart interrupt mechanisms */
  SPI.endTransaction();

  /* send our address value back to the caller */
  return (result);
}

/* write some data to a register of a device on the SPI bus */
void writeSPI(int deviceSelect, byte registerSelect, int data) {

  /* the r/w bit is first to transfer so we can shift it up to the top of the register
       read = 1, write = 0 */
  registerSelect = registerSelect | (1 << 7);

  /* begin a transaction over SPI using our params. this command also stops interrupts from preventing SPI comms */
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));

  /* to communicate with the device, we take its slave select pin on the PDC low */
  digitalWrite(deviceSelect, LOW);

  /* if we want to write to a particular register, we must send the address of the register to the device */
  SPI.transfer(registerSelect);

  /* the device now knows which device we want to write to, so lets send our data */
  // TODO: is it possible or required to send more than one byte? if so, add support
  SPI.transfer(data);

  /* stop communications with device by setting the corresponding slave select on the PDC to high */
  digitalWrite(deviceSelect, HIGH);

  /* we're done now! restart interrupt mechanisms */
  SPI.endTransaction();
}
