// Methods TODO:
// read values (gyro xyz, accel xy, temp?, general reg?)
// measure offset
// class within a class? so can have IMU.gyro.x() for example

/* for example usage, see PDC_LSM6DSO32.h */

#include "PDC_LSM6DSO32.h"  /* include the definition of the class */

/*********************************************************
 * @brief  Read component ID
 * @retval 1 in case of success, 0 otherwise
 *********************************************************/
bool PDC_LSM6DSO32::isAlive() {
  bool isAlive = 0;                     /* signify result - 1 is success */
  uint8_t WHO_AM_I[1];                  /* internal variable to hold the output */
  uint8_t WHO_AM_I_expect = 0b01101100; /* the output that we expect */
  uint8_t WHO_AM_I_address = 0x0f;      /* the address of the WHO_AM_I register */
  
  readSPI(slaveSelect, WHO_AM_I_address, 1, WHO_AM_I);  /* read the 'WHO_AM_I' register on the IMU */

  /* check that it's what we expect */
  if (WHO_AM_I[0] == WHO_AM_I_expect) {
    isAlive = 1;  /* if it is, set our success code to true */
  }
  return (isAlive);
}

/*********************************************************
 * @brief  Restart IMU
 *********************************************************/
void PDC_LSM6DSO32::restart(){
  uint8_t CTRL3_C_address = 0x12;     /* address of the CTRL3_C register on the device which allows us to reboot memory */
  uint8_t dataToWrite = 1 || 1 << 7;  /* set bits 7 and 0 to high which reboots memory and resets software */
  
  writeSPI(slaveSelect, CTRL3_C_address, dataToWrite); /* write the command to the device */
  delay(2000); /* wait for it to properly start up again */
}

/*********************************************************
 * @brief  Internally take note of important registers
 * @param  the address of the x-axis LSB data register
 * @param  the address of the control register
 *********************************************************/
void IMUChild::addressSet(uint8_t x_add, uint8_t CTRL_add) {
  x_address = x_add;        /* set x LSB address attribute as specified */  
  y_address = x_add + 2;    /* y LSB address is then two along */
  z_address = x_add + 4;    /* z LSB address is another two along */
  CTRL_address = CTRL_add;  /* and then the address to configure this child */
}

/*********************************************************
 * @brief  Initialise the component
 * @param  code for output update frequency
 *          0.  off
 *          1.  12.5Hz
 *          2.  26Hz
 *          3.  52Hz
 *          4.  104Hz
 *          5.  208Hz
 *          6.  416Hz
 *          7.  833Hz
 *          8.  1660Hz
 *          9.  3330Hz
 *          10. 6660Hz
 * @param  code for output measurement range
 *          0. 4g  / 250dps
 *          1. --  / 125dps
 *          2. 32g / 500dps
 *          3. --  / --
 *          4. 8g  / 1000dps
 *          5. --  / --
 *          6. 16g / 2000dps
 * @retval the measured X axis value in g [ac] or dps [gy]
 *********************************************************/
void IMUChild::init(uint8_t frequency, uint8_t range) {
  uint8_t data = 0;                            
  
  data |= range << 1;     /* set bits [3:1] to configure range. note accel only uses [3:2] so have padded with bit 1 to make equivalent */
  data |= frequency << 4; /* set bits [7:4] to configure output frequency */

  /* set the internally stored output frequency */
  switch(frequency){
    case(0):  outputFrequency = 0;    break;
    case(1):  outputFrequency = 12.5; break;
    case(2):  outputFrequency = 26;   break;
    case(3):  outputFrequency = 52;   break;
    case(4):  outputFrequency = 104;  break;
    case(5):  outputFrequency = 208;  break;   
    case(6):  outputFrequency = 416;  break;
    case(7):  outputFrequency = 833;  break;  
    case(8):  outputFrequency = 1660; break;
    case(9):  outputFrequency = 3330; break;
    case(10): outputFrequency = 6660; break;
    default:  outputFrequency = 0;    break;
  }

  /* set the internally stored measurement range (if condition checks if accelerometer or gyroscope) */
  if(CTRL_address == 0x10){
    switch(range){
      case(0): measurementRange = 4;  break;
      case(2): measurementRange = 32; break;
      case(4): measurementRange = 8;  break;
      case(6): measurementRange = 16; break;
      default: measurementRange = 0;  break;
    } 
  } else if(CTRL_address == 0x11){
    switch(range){
      case(0):  measurementRange = 250;   break;
      case(1):  measurementRange = 125;   break;
      case(2):  measurementRange = 500;   break;
      case(4):  measurementRange = 1000;  break;
      case(6):  measurementRange = 2000;  break;
      default:  measurementRange = 0;     break;
    }
  }

  writeSPI(slaveSelect, CTRL_address, data);  /* write the data to the control register */

  resolution = (measurementRange * 2.0 * 1000.0) / 65536.0; /* calculate the device resolution per bit (milli-g or milli-dps) */
}

/*********************************************************
 * @brief  Read a value from a specified register
 * @param  the address of the LSB data register
 * @retval the value in g [ac] or dps [gy]
 *********************************************************/
float IMUChild::readValue(uint8_t LSB_address){
  uint8_t rawValue[2];        /* we will read two bytes from the device into here */    
  int16_t rawValueConcat = 0; /* we will concatenate the two bytes into a single value here */
  float measuredValue = 0;    /* and we will convert the concatenated value into a 'measured' value here */
  
  
  readSPI(slaveSelect, LSB_address, 2, rawValue); /* read two bytes from the device. 
                                                   *  since CTRLC_3 'IF_INC' bit is enabled, the address will
                                                   *  auto-increment and read the LSB then MSB registers so we
                                                   *  have the two bytes we need!
                                                   */
  
  rawValueConcat = (rawValue[1] << 8) | rawValue[0];  /* concatenate the two bytes into a single val by shifting the MSB up by one byte */

  measuredValue = (float(rawValueConcat) / 1000) * resolution;  /* use the sensor resolution to convert raw value into an actual measurement */

  return (measuredValue);
}

/*********************************************************
 * @brief  Read data from the X axis
 * @retval the measured X axis value in g [ac] or dps [gy]
 *********************************************************/
float IMUChild::readX(){
  return(readValue(x_address));
}

/*********************************************************
 * @brief  Read data from the Y axis
 * @retval the measured Y axis value in g [ac] or dps [gy]
 *********************************************************/
float IMUChild::readY(){
  return(readValue(y_address));
}

/*********************************************************
 * @brief  Read data from the Y axis
 * @retval the measured Y axis value in g [ac] or dps [gy]
 *********************************************************/
float IMUChild::readZ(){
  return(readValue(z_address));
}

/*********************************************************
 * @brief  measure standard deviation of noise in an axis
 * @retval the measurement noise standard devation
 *********************************************************/
float IMUChild::measureNoiseZ() {
  uint8_t numReadings = 50; /* how many readings to calculate standard deviation over */
  /* stat variables */
  float stdDev = 0;
  float mean = 0;
  float sum = 0;
  float prev_mean = 0;

  float accZ = 0; /* the acceleration in the z-direction in m/s^2 */

  float threshold = 0.3;  /* reject rubbish values that exceed a threshold of reasonable expectation */

  /* for the specified number of readings, measure the acceleration */
  for (uint8_t i = 1; i < numReadings; i++) {
    // TODO: consider replacing with a non-blocking function?
    delay(100); /* force rate of measurements to allow for proper processing */

    accZ = readZ() * GRAVITY_MAGNITUDE; /* get z-axis acceleration */
    
    if (abs(GRAVITY_MAGNITUDE - accZ) > threshold) {
      i -= 1; /* for an erroneous reading, we should take the reading again to avoid skew */
    }
    else {
      /* Welford's algorithm for calculating standard deviation in real time
       *  allows us to sidestep a large array of floats which would very quickly eat up memory & limit the samples we can test! 
       */
      mean = mean + (accZ - mean) / i;
      sum = sum + (accZ - mean) * (accZ - prev_mean);
      prev_mean = mean;
    }
  }

  // TODO: determine if we should be dividing by n or by n-1
  stdDev = pow(sum / float(numReadings), 0.5);

  // TODO: consider putting a cap on stdDev incase of disturbance during setup
  // TODO: worth considering replacement or supplementation with a lookup table - if we want to change mode when switching to attitude determination, we can't measure the noise
  //       or maybe we should go between the measurement modes on the ground and measure stddev in each of them??

  return (stdDev);
}
