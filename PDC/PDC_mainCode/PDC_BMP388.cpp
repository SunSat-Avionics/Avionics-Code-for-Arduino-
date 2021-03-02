// Methods TODO:
// consider (research) IIR filter
// measure altitude noise (for kalman)
// compensate for supersonic 
// change from separate press/temp and burst through all data registers instead 
// instead of sea level comparison, get a pressure right at the start and compare??
// verify that temperature/pressure/altitude is correct

/* for example usage, see PDC_BMP388.h */

#include "PDC_BMP388.h"  /* include the definition of the class */

/*********************************************************
   @brief  Read component ID
   @retval 1 in case of success, 0 otherwise
 *********************************************************/
bool PDC_BMP388::isAlive() {
  bool isAlive = 0;       /* signify result - 1 is success */
  uint8_t CHIP_ID[1];     /* internal variable to hold the output */

  /* read the 'CHIP_ID' register on the altimeter, accounting for the fact the BMP388 returns a dummy byte before useful data */
  readSPIwithDummy(slaveSelect, CHIP_ID_REG, 1, CHIP_ID);  
  
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
  uint8_t dataToWrite = 0xB6; /* command to soft reset the device */

  writeSPI(slaveSelect, CMD_REG, dataToWrite);      /* write the command to the CMD register */
  delay(1000);  /* wait for it to properly reset */

  dataToWrite = 0b00110011;   /* set relevant bits to enter 'normal' mode and to enable the pressure and temperature measurement */
  
  writeSPI(slaveSelect, PWR_CTRL_REG, dataToWrite); /* write the command to the PWR_CTRL register */
  delay(1000);  /* more waiting to give it time to sort itself out */
}

/*********************************************************
   @brief  Internally take note of important registers
   @param  the address of the first data register
 *********************************************************/
void PDC_BMP388::addressSet(uint8_t data_0_add) {
  pressureAddress_0 = data_0_add;         /* set pressure address 0 attribute as specified */
  temperatureAddress_0 = data_0_add + 3;  /* the temperature address 0 is then past the three consecutive pressure addresses */
}

/*********************************************************
   @brief  Initialise the component
   @param  32 bits describing the ODR and OSR configs
            (aliases in PDC_BMP388.h)
           format: [0:7]   temperature oversampling (resolution)
                   [8:15]  pressure oversampling (resolution)
                   [16:23] sensor output update frequency
                   [24:32] UNUSED
 *********************************************************/
void PDC_BMP388::init(uint32_t configurationSettings) {
  uint8_t dataToWrite = 0;  /* temporary variable for the data to write to the registers */

  uint8_t frequency = (configurationSettings >> 16) & 255; /* shift the frequency byte down into 0:7 and mask out any additional info */
  dataToWrite |= frequency;  /* set bits [4:0] to configure output frequency as per datasheet */

  /* set the internally stored output frequency in case we need to check it later */
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
  
  writeSPI(slaveSelect, ODR_REG, dataToWrite);  /* write the frequency configuration to the ODR register */

  dataToWrite = 0; /* reset ready for new data */

  /* mask the input to get sensor resolutions */
  uint8_t pressResolution = (configurationSettings >> 8) & 255; /* shift the pressure resolution byte down to 0:7 and mask out any additional info */
  uint8_t tempResolution = configurationSettings & 255;         /* and mask out any remaining info on the configuration to get the temperature resolution */
  dataToWrite = (tempResolution << 3) | pressResolution;        /* set bits [5:3] for temperature and [2:0] for pressure as per datasheet */

  /* set the internally stored pressure oversampling incase we need to check it later */
  switch (pressResolution) {
    case (0):  pressureOversampling = 1;  break;
    case (1):  pressureOversampling = 2;  break;
    case (2):  pressureOversampling = 4;  break;
    case (3):  pressureOversampling = 8;  break;
    case (4):  pressureOversampling = 16; break;
    case (5):  pressureOversampling = 32; break;
    default:   pressureOversampling = 0;  break;
  }

  /* set the internally stored temperature oversampling in case we need to check it later */
  switch (tempResolution) {
    case (0):  temperatureOversampling = 1;  break;
    case (1):  temperatureOversampling = 2;  break;
    case (2):  temperatureOversampling = 4;  break;
    case (3):  temperatureOversampling = 8;  break;
    case (4):  temperatureOversampling = 16; break;
    case (5):  temperatureOversampling = 32; break;
    default:   temperatureOversampling = 0;  break;
  }

  writeSPI(slaveSelect, OSR_REG, dataToWrite);  /* write the resolution data to the OSR register */

  getCompensationParams();  /* get the device specific temperature and pressure compensation parameters and store internally */
}

/*********************************************************
   @brief  get the device specific compensation parameters
 *********************************************************/
void PDC_BMP388::getCompensationParams() {
  /* to convert from raw pressure/temperature measurements to something meaningful, the BMP388
       requires us to compensate for the specific sensor characteristics using internally stored
       (non-volatile) parameters, which we can read as below. these values are stored as different
       data types (unsigned/signed, 8/16bit), and also need some conversion to floating point 
     there are certainly more elegant solutions to reading each of these params from the device and
       writing them to our class, but they typically involve lots of arrays to hold datatypes and 
       conversion factors. this way is likely more memory friendly */
      
  uint8_t rawValue[2];      /* an array for the output bytes (no parameter is longer than two bytes) */  

  /* get the device specific temperature compensation parameters */
  readSPIwithDummy(slaveSelect, NVM_PAR_T1_REG_1, 2, rawValue); /* read the T1 parameter (accounting for dummy return byte) */
  uint16_t PAR_T1 = uint16_t((rawValue[1] << 8) | rawValue[0]); /* concatenate the two read bytes and make sure it is cast into the correct type */
  temperatureCompensationArray[0] = float(PAR_T1) / pow(2, -8); /* apply the floating point conversion as detailed in the datasheet, and store as part of the class attribute */
  rawValue[0] = 0;                                              /* clear the rawValue buffer ready for the next read */
  rawValue[1] = 0;

  readSPIwithDummy(slaveSelect, NVM_PAR_T2_REG_1, 2, rawValue);
  uint16_t PAR_T2 = uint16_t((rawValue[1] << 8) | rawValue[0]);
  temperatureCompensationArray[1] = float(PAR_T2) / pow(2, 30);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPIwithDummy(slaveSelect, NVM_PAR_T3_REG_1, 1, rawValue);
  int8_t PAR_T3 = int8_t(rawValue[0]);
  temperatureCompensationArray[2] = float(PAR_T3) / pow(2, 48);
  rawValue[0] = 0;
  rawValue[1] = 0;


  /* get the device specific pressure compensation parameters */
  readSPIwithDummy(slaveSelect, NVM_PAR_P1_REG_1, 2, rawValue);
  int16_t PAR_P1 = int16_t((rawValue[1] << 8) | rawValue[0]);
  pressureCompensationArray[0] = (float(PAR_P1) - pow(2, 14)) / pow(2, 20);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPIwithDummy(slaveSelect, NVM_PAR_P2_REG_1, 2, rawValue);
  int16_t PAR_P2 = int16_t((rawValue[1] << 8) | rawValue[0]);
  pressureCompensationArray[1] = (float(PAR_P2) - pow(2, 14)) / pow(2, 29);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPIwithDummy(slaveSelect, NVM_PAR_P3_REG_1, 1, rawValue);
  int8_t PAR_P3 = int8_t(rawValue[0]);
  pressureCompensationArray[2] = float(PAR_P3) / pow(2, 32);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPIwithDummy(slaveSelect, NVM_PAR_P4_REG_1, 1, rawValue);
  int8_t PAR_P4 = int8_t(rawValue[0]);
  pressureCompensationArray[3] = float(PAR_P4) / pow(2, 37);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPIwithDummy(slaveSelect, NVM_PAR_P5_REG_1, 2, rawValue);
  uint16_t PAR_P5 = uint16_t((rawValue[1] << 8) | rawValue[0]);
  pressureCompensationArray[4] = float(PAR_P5) / pow(2, -3);
  rawValue[0] = 0;
  rawValue[1] = 0;
  
  readSPIwithDummy(slaveSelect, NVM_PAR_P6_REG_1, 2, rawValue);
  uint16_t PAR_P6 = uint16_t((rawValue[1] << 8) | rawValue[0]);
  pressureCompensationArray[5] = float(PAR_P6) / pow(2, 6);
  rawValue[0] = 0;
  rawValue[1] = 0;
  
  readSPIwithDummy(slaveSelect, NVM_PAR_P7_REG_1, 1, rawValue);
  int8_t PAR_P7 = int8_t(rawValue[0]);
  pressureCompensationArray[6] = float(PAR_P7) / pow(2, 8);
  rawValue[0] = 0;
  rawValue[1] = 0;
  
  readSPIwithDummy(slaveSelect, NVM_PAR_P8_REG_1, 1, rawValue);
  int8_t PAR_P8 = int8_t(rawValue[0]);
  pressureCompensationArray[7] = float(PAR_P8) / pow(2, 15);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPIwithDummy(slaveSelect, NVM_PAR_P9_REG_1, 2, rawValue);
  int16_t PAR_P9 = int16_t((rawValue[1] << 8) | rawValue[0]);
  pressureCompensationArray[8] = float(PAR_P9) / pow(2, 48);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPIwithDummy(slaveSelect, NVM_PAR_P10_REG_1, 1, rawValue);
  int8_t PAR_P10 = int8_t(rawValue[0]);
  pressureCompensationArray[9] = float(PAR_P10) / pow(2, 48);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPIwithDummy(slaveSelect, NVM_PAR_P11_REG_1, 1, rawValue);
  int8_t PAR_P11 = int8_t(rawValue[0]);
  pressureCompensationArray[10] = float(PAR_P11) / pow(2, 65);
  rawValue[0] = 0;
  rawValue[1] = 0;
}

/*********************************************************
   @brief  Read a value from the temp/press data registers
   @param  the address of the first of three data registers
   @retval the raw measured value
 *********************************************************/
uint32_t PDC_BMP388::readValue(uint8_t data_address0) {
  /* the altimeter has three consecutive data registers for each of the temp and pressure.
     the first address is used to varying degrees based on the chosen resolution (oversampling) */
      
  uint8_t rawValue[3];         /* we will read three bytes from the device into here */
  uint32_t rawValueConcat = 0; /* we will concatenate the three bytes into a single value here (actually 24 bit but no such type!) */
  readSPIwithDummy(slaveSelect, data_address0, 3, rawValue);  /* read three bytes from the device. BMP388 auto-increments address on SPI read */
  
  /* concatenate the three bytes into a single val by shifting the array values up 
     need to cast each array element into a 32bit register otherwise overflow occurs on shifting */
  rawValueConcat = (uint32_t(rawValue[2]) << 16) | (uint32_t(rawValue[1]) << 8) | rawValue[0];  
  
  return (rawValueConcat);
}

/*********************************************************
   @brief  read device pressure and compensate w/ params
   @retval the compensated pressure measurement [Pa]
 *********************************************************/
float PDC_BMP388::readPress(){
  uint32_t uncompensatedPressure = 0;   /* the raw pressure data from the device */
  float compensatedPressure = 0;        /* the pressure as compensated for using parameters */
  float compensatedTemperature = 0;     /* the temperature as compensated for using parameters */
  float interim1 = 0;                   /* interim registers to store data */
  float interim2 = 0;
  float interim3 = 0;
  
  compensatedTemperature = readTemp();  /* get the compensated temperature reading */
  
  /* pass the first pressure data address (data_0) to read the 3 consecutive pressure addresses */
  uncompensatedPressure = readValue(pressureAddress_0); 

  /* below compensation calculations are as specified in the datasheet */

  /* PAR_P8 * compTemp^3 + PAR_P7 * compTemp^2 + PAR_P6 * compTemp + PAR_P5 */
  interim1 = pressureCompensationArray[7] * pow(compensatedTemperature, 3) 
             + pressureCompensationArray[6] * pow(compensatedTemperature, 2) 
             + pressureCompensationArray[5] * compensatedTemperature
             + pressureCompensationArray[4];  
  /* uncompPress * (PAR_P4 * compTemp^3 + PAR_P3 * compTemp^2 + PAR_P2 * compTemp + PAR_P1) */
  interim2 = float(uncompensatedPressure) * 
              (pressureCompensationArray[3] * pow(compensatedTemperature, 3) 
             + pressureCompensationArray[2] * pow(compensatedTemperature, 2) 
             + pressureCompensationArray[1] * compensatedTemperature
             + pressureCompensationArray[0]);  
  /* PAR_P11 * uncompPress^3 + (PAR_P9 + PAR_P10 * compTemp) * uncompPress^2 */
  interim3 = pow(float(uncompensatedPressure), 3) * pressureCompensationArray[10]
             + pow(float(uncompensatedPressure), 2) * (pressureCompensationArray[8] + pressureCompensationArray[9] * compensatedTemperature);

  compensatedPressure = interim1 + interim2 + interim3; /* calculate the compensated pressure [Pa] */

  return(compensatedPressure);
}

/*********************************************************
   @brief  read device temperature and compensate w/ params
   @retval the compensated temperature measurement [degC]
 *********************************************************/
float PDC_BMP388::readTemp(){
  uint32_t uncompensatedTemperature = 0;  /* the raw temperature data from the device */
  float compensatedTemperature = 0;       /* the temperature as compensated for using parameters */
  float interim1 = 0;                     /* interim registers to store data */
  float interim2 = 0;
  
  /* pass the first temperature data address (data_0) to read the 3 consecutive temperature addresses */
  uncompensatedTemperature = readValue(temperatureAddress_0); 

  /* below compensation calculations are as per the datasheet */
  
  /* uncomp - PAR_T1 */
  interim1 = float(uncompensatedTemperature) - temperatureCompensationArray[0];
  /* (uncomp - PAR_T1) * PAR_T2 */
  interim2 = interim1 * temperatureCompensationArray[1];
  /* [(uncomp - PAR_T1) * PAR_T2] + (uncomp - PAR_T1)^2 * PAR_T3 */
  compensatedTemperature = interim2 + (interim1 * interim1) * temperatureCompensationArray[2];

  return(compensatedTemperature);
}

/*********************************************************
   @brief  measure the altitude using compensated values
   @retval the absolute altitude [m]
 *********************************************************/
float PDC_BMP388::readAltitude(){
  float atmosphericPressure;  /* measured (compensated) pressure */
  float altitude;             /* calculated altitude based on pressure */

  /* the BMP180 datasheet (https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf) gives the equation for pressure -> altitude */
  
  atmosphericPressure = readPress() / 100.0;  /* read the compensated atmospheric pressure and convert from Pa to hPa */
  altitude = 44330 * (1 - pow(atmosphericPressure/SEA_LEVEL_PRESSURE, 0.190295)); /* use the equation from link above to claculate absolute altitude [m] */

  return(altitude);
}
