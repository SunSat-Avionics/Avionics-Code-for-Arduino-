// Methods TODO:
// read values (e.g. a 'readAltitude' method that auto accounts for mach, temp, etc)
// move compensation calculations into an attribute (i.e. PAR_P1 * PAR_P2 ... or whatever, is a new constant calculated at startup)
// internal SPI setup/enable?
// consider (research) IIR filter
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
   @param  code for the pressure resolution (oversampling)
   @param  code for the temperature resolution (oversampling)
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
   @brief  get the device specific compensation parameters
 *********************************************************/
void PDC_BMP388::getCompensationParams() {
  /* arrays to store the various parameters
  uint8_t NVM_PAR_T1[2];
  uint8_t NVM_PAR_T2[2];
  uint8_t NVM_PAR_T3[1];

  uint8_t NVM_PAR_P1[2];
  uint8_t NVM_PAR_P2[2];
  uint8_t NVM_PAR_P3[1];
  uint8_t NVM_PAR_P4[1];
  uint8_t NVM_PAR_P5[2];
  uint8_t NVM_PAR_P6[2];
  uint8_t NVM_PAR_P7[1];
  uint8_t NVM_PAR_P8[1];
  uint8_t NVM_PAR_P9[2];
  uint8_t NVM_PAR_P10[1];
  uint8_t NVM_PAR_P11[1];
  */

  /*
  uint8_t tempParamLengths[3] = {2, 2, 1};
  uint8_t tempParamSigns[3] = {0, 0, 1}; 
  int8_t tempParamScalingIndex[3] = {-8, 30, 48};
  */


  /*
  uint8_t pressParamLengths[11] = {2, 2, 1, 1, 2, 2, 1, 1, 2, 1, 1};
  uint8_t pressParamSigns[11] = {1, 1, 1, 1, 0, 0, 1, 1, 1, 1, 1};
  int8_t pressParamScalingIndex[11] = {20, 29, 32, 37, -3, 6, 8, 15, 48, 48, 65};
  */

  /* to convert from raw pressure/temperature measurements to something meaningful, the BMP388
       requires us to compensate for the specific sensor characteristics using internally stored
       (non-volatile) parameters, which we can read as below. these values are stored as different
       data types (unsigned/signed, 8/16bit), and also need some conversion to floating point */
  
  /* there are certainly more elegant solutions to reading each of these params from the device and
      writing them to our class, but they typically involve lots of arrays to hold datatypes and 
      conversion factors. this way is likely more memory friendly */
      
  uint8_t rawValue[2];      /* an array for the output bytes (no parameter is longer than two bytes) */  

  // TODO: define aliases for the powers for each of these? or number of bytes for each of these?
  /* get the device specific temperature compensation parameters */
  readSPI(slaveSelect, NVM_PAR_T1_REG_1, 2, rawValue);
  uint16_t PAR_T1 = (rawValue[1] << 8) | rawValue[0];
  temperatureCompensationArray[0] = float(PAR_T1) / pow(2, -8);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPI(slaveSelect, NVM_PAR_T2_REG_1, 2, rawValue);
  uint16_t PAR_T2 = (rawValue[1] << 8) | rawValue[0];
  temperatureCompensationArray[1] = float(PAR_T2) / pow(2, 30);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPI(slaveSelect, NVM_PAR_T3_REG_1, 1, rawValue);
  int8_t PAR_T3 = rawValue[0];
  temperatureCompensationArray[2] = float(PAR_T3) / pow(2, 48);
  rawValue[0] = 0;
  rawValue[1] = 0;


  /* get the device specific pressure compensation parameters */
  readSPI(slaveSelect, NVM_PAR_P1_REG_1, 2, rawValue);
  int16_t PAR_P1 = (rawValue[1] << 8) | rawValue[0];
  pressureCompensationArray[0] = (float(PAR_P1) - pow(2, 14)) / pow(2, 20);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPI(slaveSelect, NVM_PAR_P2_REG_1, 2, rawValue);
  int16_t PAR_P2 = (rawValue[1] << 8) | rawValue[0];
  pressureCompensationArray[1] = (float(PAR_P2) - pow(2, 14)) / pow(2, 29);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPI(slaveSelect, NVM_PAR_P3_REG_1, 1, rawValue);
  int8_t PAR_P3 = rawValue[0];
  pressureCompensationArray[2] = float(PAR_P3) / pow(2, 32);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPI(slaveSelect, NVM_PAR_P4_REG_1, 1, rawValue);
  int8_t PAR_P4 = rawValue[0];
  pressureCompensationArray[3] = float(PAR_P4) / pow(2, 37);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPI(slaveSelect, NVM_PAR_P5_REG_1, 2, rawValue);
  uint16_t PAR_P5 = (rawValue[1] << 8) | rawValue[0];
  pressureCompensationArray[4] = float(PAR_P5) / pow(2, -3);
  rawValue[0] = 0;
  rawValue[1] = 0;
  
  readSPI(slaveSelect, NVM_PAR_P6_REG_1, 2, rawValue);
  uint16_t PAR_P6 = (rawValue[1] << 8) | rawValue[0];
  pressureCompensationArray[5] = float(PAR_P6) / pow(2, 6);
  rawValue[0] = 0;
  rawValue[1] = 0;
  
  readSPI(slaveSelect, NVM_PAR_P7_REG_1, 1, rawValue);
  int8_t PAR_P7 = rawValue[0];
  pressureCompensationArray[6] = float(PAR_P7) / pow(2, 8);
  rawValue[0] = 0;
  rawValue[1] = 0;
  
  readSPI(slaveSelect, NVM_PAR_P8_REG_1, 1, rawValue);
  int8_t PAR_P8 = rawValue[0];
  pressureCompensationArray[7] = float(PAR_P8) / pow(2, 15);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPI(slaveSelect, NVM_PAR_P9_REG_1, 2, rawValue);
  int16_t PAR_P9 = (rawValue[1] << 8) | rawValue[0];
  pressureCompensationArray[8] = float(PAR_P9) / pow(2, 48);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPI(slaveSelect, NVM_PAR_P10_REG_1, 1, rawValue);
  int8_t PAR_P10 = rawValue[0];
  pressureCompensationArray[9] = float(PAR_P10) / pow(2, 48);
  rawValue[0] = 0;
  rawValue[1] = 0;

  readSPI(slaveSelect, NVM_PAR_P11_REG_1, 1, rawValue);
  int8_t PAR_P11 = rawValue[0];
  pressureCompensationArray[10] = float(PAR_P11) / pow(2, 65);
  rawValue[0] = 0;
  rawValue[1] = 0;
  
  /*
  for(i=0; i<sizeof(tempParamLengths); i++){
    bytesToRead = tempParamLengths[i];   get length in bytes of this parameter 
    readSPI(slaveSelect, addressToRead, numBytesToRead, rawValue);   read the parameter over SPI 

     declare a parameter variable with the appropriate signing 
    if(tempParamSigns[i] == 0){
        uint16_t parameter;
    } else if(tempParamSigns[i] == 1){
        int16_t parameter;
    }

    parameter = (rawValue[1] << 8) | rawValue[0]; /* concatenate the read value into the parameter variable 

    

     address = ;
     temperatureCompensationArray[i] = ;

    rawValue[0] = 0;
    rawValue[1] = 0;
  }
  */
}

/*********************************************************
   @brief  Read a value from a specified register
   @param  the address of the first of three data registers
   @retval the raw measured value
 *********************************************************/
uint32_t PDC_BMP388::readValue(uint8_t data_address0) {
  /* the altimeter has three consecutive data registers for both the temp and pressure.
      the first address is used to varying degrees based on the chosen resolution (oversampling) */
  uint8_t rawValue[3];        /* we will read three bytes from the device into here */
  int32_t rawValueConcat = 0; /* we will concatenate the three bytes into a single value here. actually 24 bit but no such type! */
  readSPI(slaveSelect, data_address0, 3, rawValue); /* read three bytes from the device. BMP388 auto-increments address on SPI read */
  rawValueConcat = (rawValue[2] << 16) | (rawValue[1] << 8) | rawValue[0];  /* concatenate the three bytes into a single val by shifting the array values up */
  return (rawValueConcat);
}

/*********************************************************
   @brief  read device pressure and compensate w/ params
   @retval the compensated pressure measurement
 *********************************************************/
float PDC_BMP388::readPress(){
  uint32_t uncompensatedPressure = 0;   /* the raw pressure data from the device */
  float compensatedPressure = 0;        /* the pressure as compensated for with parameters */
  float compensatedTemperature = 0;     /* the temperature as compensated for with parameters */
  float interim1 = 0;                   /* interim registers to store data */
  float interim2 = 0;
  float interim3 = 0;
  
  compensatedTemperature = readTemp();  /* get the compensated temperature reading */

  /* pass the first pressure data address (data_0) to read the 3 consecutive pressure addresses */
  uncompensatedPressure = readValue(pressureAddress_0); 

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

  compensatedPressure = interim1 + interim2 + interim3;

  return(compensatedPressure);
}

/*********************************************************
   @brief  read device temperature and compensate w/ params
   @retval the compensated temperature measurement
 *********************************************************/
float PDC_BMP388::readTemp(){
  uint32_t uncompensatedTemperature = 0;  /* the raw temperature data from the device */
  float compensatedTemperature = 0;       /* the temperature as compensated for with parameters */
  float interim1 = 0;                     /* interim registers to store data */
  float interim2 = 0;
  
  /* pass the first temperature data address (data_0) to read the 3 consecutive temperature addresses */
  uncompensatedTemperature = readValue(temperatureAddress_0); 

  /* uncomp - PAR_T1 */
  interim1 = float(uncompensatedTemperature) - temperatureCompensationArray[0];
  /* (uncomp - PAR_T1) * PAR_T2 */
  interim2 = interim1 * temperatureCompensationArray[1];
  /* [(uncomp - PAR_T1) * PAR_T2] + (uncomp - PAR_T1)^2 * PAR_T3 */
  compensatedTemperature = interim2 + (interim1 * interim1) * temperatureCompensationArray[2];

  return(compensatedTemperature);
}
