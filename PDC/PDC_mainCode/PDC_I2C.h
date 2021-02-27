// TODO: setup some functions for comms with OBC
  // current plan is for PDC to be slave, OBC to be master. OBC requests data over I2C, PDC registers this as an
  // interrupt (?) event and goes into a 'write I2C data' subroutine

void readI2C(uint8_t deviceAddress, uint8_t deviceRegister, uint8_t numBytes, uint8_t *result);
