/*
 *  the three parameters for SPI.beginTransaction() are: 1. clock input frequency, 2. MSB/LSB first, and 3. SPI mode
 *  for more information, see: https://www.arduino.cc/en/reference/SPI
 *  1. altimeter & gyro/accel have max clock input freq. of 10MHz, micro-sd has 25MHz
 *    to avoid reconfigs, we'll stick at 10MHz for now - see if this is fast enough for SD
 *  2. all devices are MSB first
 *  3. all devices are compatible with mode 00 (clock idle low, output: falling edge, capture: rising edge);
 */
 
const uint32_t CLOCK_RATE = 10000000; /* all devices on the bus are happy with 10MHz clock */

/**
 * @brief  Read register from device over SPI
 * @param  the pin on the PDC that connects to the device slave select pin
 * @param  the address of the register to read
 * @param  the number of bytes that we want to read in this transaction
 * @param  pointer to an array that we store the result in
 */
void readSPI(uint8_t deviceSelect, uint8_t registerSelect, uint8_t numBytes, uint8_t *result) {
  // TODO: lookup 'shiftout()' - seen it mentioned as alternative(?) to SPI.Transfer?

  /* ---------- SETUP ---------- */
  /* format address r/w bit (MSB). read = 1, write = 0 */
  registerSelect = registerSelect | (1 << 7); 

  /* ---------- BEGIN TRANSACTION ---------- */
  /* begin a transaction over SPI using our params. this command also stops interrupts from preventing SPI comms */
  SPI.beginTransaction(SPISettings(CLOCK_RATE, MSBFIRST, SPI_MODE0));
  /* to communicate with the device, we take its slave select pin on the PDC low */
  digitalWrite(deviceSelect, LOW);                                    

  /* ---------- READ FROM REGISTER(S) ---------- */
  SPI.transfer(registerSelect);     /* send the register address (with read bit) so device knows what to send us next */
  for (uint8_t i = 0; i < numBytes; i++) {
    result[i] = SPI.transfer(0x00); /* send nothing. this is us 'listening' to the bus for the values that we have requested */
    /* if we have requested multiple bytes, the device will auto-increment the address and send values from the next register in sequence next time we 'listen' */
  }

  /* ---------- END TRANSACTION ---------- */
  /* stop communications with device by setting the corresponding slave select on the PDC to high */
  digitalWrite(deviceSelect, HIGH); 
  /* we're done now! restart interrupt mechanisms */
  SPI.endTransaction(); 

  /* *result is then modified, and caller can access it for the data */
}

/**
 * @brief  Write data to a register on device over SPI
 * @param  the pin on the PDC that connects to the device slave select pin
 * @param  the address of the register to write to
 * @param  the data that we want to write to the address
 */
void writeSPI(uint8_t deviceSelect, uint8_t registerSelect, uint8_t data) {

  /* ---------- SETUP ---------- */
  /* format address r/w bit (MSB). read = 1, write = 0 */
  registerSelect = registerSelect | (0 << 7);

  /* ---------- BEGIN TRANSACTION ---------- */
  /* begin a transaction over SPI using our params. this command also stops interrupts from preventing SPI comms */
  SPI.beginTransaction(SPISettings(CLOCK_RATE, MSBFIRST, SPI_MODE0));
  /* to communicate with the device, we take its slave select pin on the PDC low */
  digitalWrite(deviceSelect, LOW);

  /* ---------- WRITE TO REGISTER ---------- */
  // TODO: is it possible or required to send more than one byte? if so, add support
  SPI.transfer(registerSelect); /* send the register address (with write bit) so device knows where we want to put our data */
  SPI.transfer(data);           /* send the data that we want to write to the selected register */    

  /* ---------- END TRANSACTION ---------- */
  /* stop communications with device by setting the corresponding slave select on the PDC to high */
  digitalWrite(deviceSelect, HIGH);
  /* we're done now! restart interrupt mechanisms */
  SPI.endTransaction();
}
