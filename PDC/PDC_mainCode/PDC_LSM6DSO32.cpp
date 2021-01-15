// Methods TODO:
// read values (gyro xyz, accel xy, temp?, general reg?)
// measure offset

#include "PDC_LSM6DSO32.h"  /* include the definition of the class */

/**
 * @brief  Read component ID
 * @retval 1 in case of success, an error code otherwise
 */
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

/**
 * @brief  Restart IMU
 */
void PDC_LSM6DSO32::restart(){
  uint8_t CTRL3_C_address = 0x12;     /* address of the CTRL3_C register on the device which allows us to reboot memory */
  uint8_t dataToWrite = 1 || 1 << 7;  /* set bits 7 and 0 to high which reboots memory and resets software */
  
  writeSPI(slaveSelect, CTRL3_C_address, dataToWrite); /* write the command to the device */
  delay(1000); /* wait for it to properly start up again */
}

/**
 * @brief  Configure accelerometer measurement parameters
 * @param  output frequency update rate in Hz (0, 12.5, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660)
 * @param  full measurement range of output in +/- g (4, 8, 16, 32)
 * @retval 0 in case of success, an error code otherwise
 */
bool PDC_LSM6DSO32::setupAccel(float outputFrequency, uint8_t range) {
  uint8_t data = 0;                 /* data to write to the IMU */
  uint8_t CTRL1_XL_address = 0x10;  /* the address of the CTRL1_XL register which controls these parameters */
  bool errFlag = 0;                 /* flag for incase inputs are invalid */
  uint16_t outputFrequencySwitch = outputFrequency; /* switch case for frequency config - helps protect against float values */

  /* range configuration
   *  bits[2:3] in the CTRL1_XL register are the ones with our measurement range config and so we shift values from 0-3 into this spot 
   *  unit is 'g'
   */
  switch (range) {
    case 4:
      data |= (0 << 2);
      break;
    case 8:
      data |= (2 << 2);
      break;
    case 16:
      data |= (3 << 2);
      break;
    case 32:
      data |= (1 << 2);
      break;
    default:
      errFlag = 1;  /* notify of invalid input */
      break;
  }
  
  /* switch case wont let us use 12.5 as a switch, so round it to 12 for use here */
  if (outputFrequency == 12.5) {
    outputFrequencySwitch = 12;
  }

  /* output frequency configuration
   *  bits[4:7] in the CTRL1_XL register are for ODR update frequency config and so we shift values into this spot
   *  unit is 'Hz'
   */
  switch (outputFrequencySwitch) {
    case 0:
      data |= (0 << 4);
      break;
    case 12:
      data |= (1 << 4);
      break;
    case 26:
      data |= (2 << 4);
      break;
    case 52:
      data |= (3 << 4);
      break;
    case 104:
      data |= (4 << 4);
      break;
    case 208:
      data |= (5 << 4);
      break;
    case 416:
      data |= (6 << 4);
      break;
    case 833:
      data |= (7 << 4);
      break;
    case 1660:
      data |= (8 << 4);
      break;
    case 3330:
      data |= (9 << 4);
      break;
    case 6660:
      data |= (10 << 4);
      break;
    default:
      errFlag = 1;  /* notify of invalid input */
      break;
  }

  /* if valid values, write the configuration to the IMU and set corresponding class attributes */
  if (!errFlag) {
    writeSPI(slaveSelect, CTRL1_XL_address, data);
    accelOutputFrequency = outputFrequency;
    accelMeasurementRange = range;
    /* calculate accelerometer resolution
     *  the resolution of the accelerometer is its range (e.g. +/4g) divided by the number of combinations available
     *  the accelerometer output is 16bits, so 2^16 is our combinations.
     *  units are milli-g per bit, so times 1000
     */
    accelResolution = (accelMeasurementRange * 2.0 * 1000.0) / 65536.0;
  }
  
  return (errFlag);
}

/**
 * @brief  Configure gyroscope measurement parameters
 * @param  output frequency update rate in Hz (0, 12.5, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660)
 * @param  full measurement range of output in +/- dps (125, 250, 500, 1000, 2000)
 * @retval 0 in case of success, an error code otherwise
 */
bool PDC_LSM6DSO32::setupGyro(float outputFrequency, uint16_t range) {
  uint8_t data = 0;                 /* data to write to the IMU */
  uint8_t CTRL2_G_address = 0x11;   /* the address of the CTRL2_G register which controls these parameters */
  bool errFlag = 0;                 /* flag for incase inputs are invalid */
  uint16_t outputFrequencySwitch = outputFrequency; /* switch case for frequency config - helps protect against float values */

  /* range configuration
   *  bits[1:3] in the CTRL2_G register are the ones with our measurement range config and so we shift values into this spot 
   *  bit 1 is enabled for 125dps, disabled for anything else
   *  bits 2:3 are set between 0 - 3 for the rest
   *  001 (1) - 125
   *  000 (0) - 250
   *  010 (2) - 500
   *  100 (4) - 1000
   *  110 (6) - 2000
   *  unit is 'dps'
   */
  switch (range) {
    case 125:
      data |= (1 << 2);
      break;
    case 250:
      data |= (0 << 2);
      break;
    case 500:
      data |= (2 << 2);
      break;
    case 1000:
      data |= (4 << 2);
      break;
    case 2000:
      data |= (6 << 2);
      break;
    default:
      errFlag = 1;  /* notify of invalid input */
      break;
  }
  
  
  /* switch case wont let us use 12.5 as a switch, so round it to 12 for use here */
  if (outputFrequency == 12.5) {
    outputFrequencySwitch = 12;
  }
  
  /* output frequency configuration
   *  bits[4:7] in the CTRL2_G register are for ODR update frequency config and so we shift values into this spot
   *  unit is 'Hz'
   */
  switch (outputFrequencySwitch) {
    case 0:
      data |= (0 << 4);
      break;
    case 12:
      data |= (1 << 4);
      break;
    case 26:
      data |= (2 << 4);
      break;
    case 52:
      data |= (3 << 4);
      break;
    case 104:
      data |= (4 << 4);
      break;
    case 208:
      data |= (5 << 4);
      break;
    case 416:
      data |= (6 << 4);
      break;
    case 833:
      data |= (7 << 4);
      break;
    case 1660:
      data |= (8 << 4);
      break;
    case 3330:
      data |= (9 << 4);
      break;
    case 6660:
      data |= (10 << 4);
      break;
    default:
      errFlag = 1;  /* notify of invalid input */
      break;
  }

  /* if valid values, write the configuration to the IMU and set corresponding class attributes */
  if (!errFlag) {
    writeSPI(slaveSelect, CTRL2_G_address, data);
    gyroOutputFrequency = outputFrequency;
    gyroMeasurementRange = range;
    /* calculate gyroscope resolution
     *  the resolution of the gyroscope is its range (e.g. +/-125dps) divided by the number of combinations available
     *  the gyroscope output is 16bits, so 2^16 is our combinations.
     *  units are milli-dps per bit, so times 1000
     */
    gyroResolution = (gyroMeasurementRange * 2.0 * 1000.0) / 65536.0;
  }
  
  return (errFlag);
}

/**
 * @brief  Read z-axis acceleration
 * @retval measured acceleration in m/s^2
 */
float PDC_LSM6DSO32::readAccelZ() {
  uint8_t rawAccelZ[2];             /* for the 2 components of the acceleration measurement */
  int16_t rawAccelConcat = 0;       /* to concatenate the 2 components into */
  float accelerationZ = 0;          /* to hold the actual acceleration in m/s^2 */
  uint8_t OUT_Z_LA_address = 0x2C;  /* address of the OUT_Z_LA register which has the LSB component of acceleration */

  /* read z-axis acceleration.
   *  note in CTRL3_C, there is a default enabled bit which auto-increments the register address when reading multiple bytes so we dont need to read the H and L
   *  registers separately, as long as we tell readSPI() that we expect 2 bytes in the last argument 
   */
  readSPI(slaveSelect, OUT_Z_LA_address, 2, rawAccelZ);
  
  /* we have rawAccelZ array with the LSB (0x2C) and MSB (0x2D) components, so concat these into a single value */
  rawAccelConcat = (rawAccelZ[1] << 8) | rawAccelZ[0];
  
  /* convert our output into an actual acceleration value in m/s^2
   *  the raw value is somewhere in our measurement range, so multiply by resolution to get back to absolute value, then multiply by g to get m/s^2 
   */
  accelerationZ = (float(rawAccelConcat) / 1000) * accelResolution * GRAVITY_MAGNITUDE;
  
  return (accelerationZ);
}

/**
 * @brief  Measure the noise in the accelerometer z-axis
 * @retval standard deviation of the noise in m/s^2
 */
float PDC_LSM6DSO32::measureAccelNoiseZ() {
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

    accZ = readAccelZ(); /* get z-axis acceleration */

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

/**
 * @brief  Read the angular rate in one axis from the gyroscope
 * @param  a character that specifies the axis we want to read the rate in
 * @retval pitch angle rate in 
 */
float PDC_LSM6DSO32::readGyro(uint8_t axis){
  uint8_t L_G_address;
  /* set the L_G (the LSB gyro) register address based on which axis is requested */
  if(axis == 'X' || axis == 'x'){
    L_G_address = 0x22;  /* x-axis (pitch) */
  }
  else if(axis == 'Y' || axis == 'y'){
    L_G_address = 0x24;  /* y-axis (roll) */
  }
  else if(axis == 'Z' || axis == 'z'){
    L_G_address = 0x26;  /* z-axis (yaw) */
  }

  uint8_t rawGyro[2];         /* for the 2 components of the angular rate measurement */
  int16_t rawGyroConcat = 0;  /* to concatenate the 2 components into */
  float gyroRate = 0;         /* to hold the actual angular rate in */

  /* read angular rate
   *  note in CTRL3_C, there is a default enabled bit which auto-increments the register address when reading multiple bytes so we dont need to read the H and L
   *  registers separately, as long as we tell readSPI() that we expect 2 bytes in the last argument 
   */
  readSPI(slaveSelect, L_G_address, 2, rawGyro);
  
  /* we have rawGyro array with the LSB and MSB components, so concat these into a single value */
  rawGyroConcat = (rawGyro[1] << 8) | rawGyro[0];
  
  /* convert our output into an actual rotation rate value in dps
   *  the raw value is somewhere in our measurement range, so multiply by resolution to get back to absolute value
   */
  gyroRate = (float(rawGyroConcat) / 1000) * gyroResolution;
  
  return (gyroRate);
}
