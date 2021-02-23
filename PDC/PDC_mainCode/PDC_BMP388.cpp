// Methods TODO:
// read values
// measure offset
// a 'read all' method?
// compensate for supersonic 

/* for example usage, see PDC_BMP388.h */

#include "PDC_BMP388.h"  /* include the definition of the class */

/*********************************************************
   @brief  Read component ID
   @retval 1 in case of success, 0 otherwise
 *********************************************************/
bool PDC_BMP388::isAlive() {
  bool isAlive = 0;       /* signify result - 1 is success */
  uint8_t CHIP_ID[1];     /* internal variable to hold the output */

  readSPI(slaveSelect, CHIP_ID_REG, 1, CHIP_ID);  /* read the 'CHIP_ID' register on the altimeter */

  /* check that it's what we expect */
  if (CHIP_ID[0] == CHIP_ID_VAL) {
    isAlive = 1;  /* if it is, set our success code to true */
  }
  
  return (isAlive);
}

/*********************************************************
   @brief  Restart altimeter
 *********************************************************/
void PDC_BMP388::restart() {
  uint8_t CMD_address = 0x7E;       /* address of the CMD register on the device which allows us to soft reset the device */
  uint8_t PWR_CTRL_address = 0x1B;  /* address of the PWR_CTRL register on the device which allows us to enable measurement and enter 'normal mode' */
  uint8_t dataToWrite = 0xB6;       /* value to soft reset the device */

  writeSPI(slaveSelect, CMD_address, dataToWrite);      /* write the command to the device */
  delay(1000);  /* wait for it to properly reset */

  dataToWrite = 0b00110011;         /* set relevant bits to enter 'normal' mode and to enable the pressure and temperature measurement */
  
  writeSPI(slaveSelect, PWR_CTRL_address, dataToWrite); /* write the command to the device */
  delay(1000);  /* more waiting to give it time to sort itself out */
}

/*********************************************************
   @brief  Initialise the component
   @param  code for output update frequency
 *********************************************************/
void PDC_BMP388::init(uint8_t frequency) {
  uint8_t data = 0;
  
  data |= frequency;  /* set bits [4:0] to configure output frequency */

  /* set the internally stored output frequency */
  switch (frequency) {
    case (0):  outputFrequency = 200;   break;
    case (1):  outputFrequency = 100;   break;
    case (2):  outputFrequency = 50;    break;
    case (3):  outputFrequency = 25;    break;
    case (4):  outputFrequency = 12.5;  break;
    case (5):  outputFrequency = 6.25;  break;
    case (6):  outputFrequency = 3.1;   break;
    case (7):  outputFrequency = 0.78;  break;
    case (8):  outputFrequency = 0.39;  break;
    case (9):  outputFrequency = 0.2;   break;
    case (10): outputFrequency = 0.1;   break;
    case (11): outputFrequency = 0.05;  break;
    case (12): outputFrequency = 0.02;  break;
    case (13): outputFrequency = 0.01;  break;
    default:  outputFrequency = 0;      break;
  }

  writeSPI(slaveSelect, ODR_REG, data);  /* write the data to the ODR register */
}

/*********************************************************
   @brief  Read a value from a specified register
   @param  the address of the LSB data register
   @retval the value 
 *********************************************************/
float PDC_BMP388::readValue(uint8_t LSB_address) {
  //uint8_t rawValue[2];        /* we will read two bytes from the device into here */
  //int16_t rawValueConcat = 0; /* we will concatenate the two bytes into a single value here */
  float measuredValue = 0;    /* and we will convert the concatenated value into a 'measured' value here */


  //readSPI(slaveSelect, LSB_address, 2, rawValue); /* read two bytes from the device.
                                                      //since CTRLC_3 'IF_INC' bit is enabled, the address will
                                                      //auto-increment and read the LSB then MSB registers so we
                                                      //have the two bytes we need!*/

  //rawValueConcat = (rawValue[1] << 8) | rawValue[0];  /* concatenate the two bytes into a single val by shifting the MSB up by one byte */

  //measuredValue = (float(rawValueConcat) / 1000) * resolution;  /* use the sensor resolution to convert raw value into an actual measurement */

  return (measuredValue);
}

void PDC_BMP388::readPressure(){
  
}
