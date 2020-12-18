/*
   the three parameters for SPI.beginTransaction() are: 1. clock input frequency, 2. MSB/LSB first, and 3. SPI mode
      for more information, see: https://www.arduino.cc/en/reference/SPI
   1. altimeter & gyro/accel have max clock input freq. of 10MHz, micro-sd has 25MHz
      to avoid reconfigs, we'll stick at 10MHz for now - see if this is fast enough for SD
   2. all devices are MSB first
   3. all devices are compatible with mode 00 (clock idle low, output: falling edge, capture: rising edge);
*/

/* read a value from a register of a device on SPI. as arguments, pass the device select pin, the address of the register, and the number of bytes that this
   register contains. it will return the value that is stored in the register that we are reading. return is 32 bits so can read 4 bytes total */
// TODO: lookup 'shiftout()' - seen it mentioned as alternative(?) to SPI.Transfer?
// TODO: revisit the result for multiple bytes - maybe accept an argument for MSB or LSB first?
uint32_t readSPI(uint8_t deviceSelect, uint8_t registerSelect, uint8_t numBytes) {

  /* variable for our register value return */
  uint32_t result = 0;
  uint8_t counter = numBytes;

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
void writeSPI(uint8_t deviceSelect, uint8_t registerSelect, uint8_t data) {

  /* the r/w bit is first to transfer so we can shift it up to the top of the register
       read = 1, write = 0 */
  registerSelect = registerSelect | (0 << 7);

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
