// Methods TODO:
// read values
// measure offset
// measure noise (for kalman)
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
   @brief  Internally take note of important registers
   @param  the address of the first data register
   @param  the address of the ODR register
   @param  the address of the OSR register
 *********************************************************/
void PDC_BMP388::addressSet(uint8_t data_0_add, uint8_t ODR_add, uint8_t OSR_add) {
  pressureAddress_0 = data_0_add;         /* set pressure address 0 attribute as specified */
  temperatureAddress_0 = data_0_add + 3;  /* the temperature address 0 is then past the three consecutive pressure addresses */
  ODR_address = ODR_add;                  /* and then the address to configure the ODR */
  OSR_address = OSR_add;                  /* and then the address to configure the OSR */
}

/*********************************************************
   @brief  Initialise the component
   @param  code for output update frequency
 *********************************************************/
void PDC_BMP388::init(uint8_t frequency, uint8_t pressResolution, uint8_t tempResolution) {
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

  data = 0;
  data = (tempResolution << 3) | pressResolution; /* set bits [5:3] for temperature and [2:0] for pressure */
  
  /* set the internally stored pressure oversampling */
  switch (pressResolution) {
    case (0):  pressureOversampling = 1;  break;
    case (1):  pressureOversampling = 2;  break;
    case (2):  pressureOversampling = 4;  break;
    case (3):  pressureOversampling = 8;  break;
    case (4):  pressureOversampling = 16; break;
    case (5):  pressureOversampling = 32; break;
    default:   pressureOversampling = 0;  break;
  }

  /* set the internally stored temperature oversampling */
  switch (tempResolution) {
    case (0):  temperatureOversampling = 1;  break;
    case (1):  temperatureOversampling = 2;  break;
    case (2):  temperatureOversampling = 4;  break;
    case (3):  temperatureOversampling = 8;  break;
    case (4):  temperatureOversampling = 16; break;
    case (5):  temperatureOversampling = 32; break;
    default:   temperatureOversampling = 0;  break;
  }
  
  writeSPI(slaveSelect, OSR_REG, data);  /* write the data to the OSR register */
}

/*********************************************************
   @brief  Read a value from a specified register
   @param  the address of the first of three data registers
   @retval the raw measured value
 *********************************************************/
int32_t PDC_BMP388::readValue(uint8_t data_address0) {
  /* the altimeter has three consecutive data registers for both the temp and pressure.
      the first address is used to varying degrees based on the chosen resolution (oversampling) */
      
  uint8_t rawValue[3];        /* we will read three bytes from the device into here */
  int32_t rawValueConcat = 0; /* we will concatenate the three bytes into a single value here. actually 24 bit but no such type! */

  readSPI(slaveSelect, data_address0, 3, rawValue); /* read three bytes from the device.
                                                        for the BMP388, the address will
                                                        auto-increment and read the consecutive registers so we
                                                        have the three bytes we need! */

  rawValueConcat = (rawValue[2] << 16) | (rawValue[1] << 8) | rawValue[0];  /* concatenate the three bytes into a single val by shifting the array values up */

  return (rawValueConcat);
}

void PDC_BMP388::readPress(){
  int32_t rawPressure = 0;

  rawPressure = readValue(pressureAddress_0);
  // TODO: convert to 'real' pressure
}

void PDC_BMP388::readTemp(){
  int32_t rawTemperature = 0;

  rawTemperature = readValue(temperatureAddress_0);
  // TODO: convert to 'real' temperature
}
