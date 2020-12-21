// Methods TODO:
// read values (gyro xyz, accel xy, temp?, general reg?)
// measure offset

#include "PDC_LSM6DSO32.h"

/* read the 'WHO_AM_I' register to check we have a valid connection to the IMU. returns true if successful */
bool PDC_LSM6DSO32::isAlive() {
  /* have a code to signify result - true is success */
  bool successCode = 0;
  /* 'array' to hold the output */
  uint8_t WHO_AM_I[1];
  /* the output that we expect */
  uint8_t WHO_AM_I_expect = 0b01101100;
  /* the address of the WHO_AM_I register */
  uint8_t regAddress = 0x0f;

  /* read the 'WHO_AM_I' register on the IMU */
  readSPI(slaveSelect, regAddress, 1, WHO_AM_I);

  /* check that it's what we expect */
  if (WHO_AM_I[0] == WHO_AM_I_expect) {
    /* if it is, set our success code to true */
    successCode = 1;
  }
  return (successCode);
}

/* read z-axis of the accelerometer and return the acceleration in m/s2 */
float PDC_LSM6DSO32::readAccelerationZ() {
  /* array for the 2 components of the acceleration measurement */
  uint8_t rawAccelZ[2];
  /* variable to concatenate the 2 components into */
  int16_t rawAccelConcat = 0;
  /* variable to hold the actual acceleration in m/s^2 */
  float accelerationZ = 0;
  /* address of the register which has the first component of acceleration */
  uint8_t regAddress = 0x2C;

  /* the resolution of the accelerometer is its range (e.g. +/4g) divided by the number of combinations available
      the accelerometer output is 16bits, so 2^16 is our combinations.
      units are milli-g per bit, so times 1000*/
  float accelResolution = (accelerometerMeasurementRange * 2.0 * 1000.0) / 65536.0;

  /* read z-axis acceleration.
      note in CTRL3_C, there is a default enabled bit which auto-increments the register address when reading multiple bytes so we dont need to read the H and L
      registers separately, as long as we tell readSPI() that we expect 2 bytes in the last argument */
  readSPI(slaveSelect, regAddress, 2, rawAccelZ);
  
  /* we have rawAccelZ array with the LSB (0x2C) and MSB (0x2D) components, so concat these into a single value */
  rawAccelConcat = (rawAccelZ[1] << 8) | rawAccelZ[0];
  
  /* convert our output into an actual acceleration value in m/s^2
      the raw value is somewhere in our measurement range, so multiply by resolution to get back to absolute value, then multiply by g to get m/s^2 */
  accelerationZ = (float(rawAccelConcat) / 1000) * accelResolution * GRAVITY_MAGNITUDE;
  return (accelerationZ);
}

/* configure the update frequency of the ODR and the measurement range of the accelerometer. returns true if it succeeded */
bool PDC_LSM6DSO32::setupAccelerometer(float outputFrequency, uint8_t range) {
  /* data to write to the IMU */
  uint8_t data = 0;
  /* the address of the CTRL1_XL register which controls these two parameters */
  uint8_t regAddress = 0x10;
  /* flag for incase inputs are invalid */
  bool errFlag = 0;

  /* bits[2:3] in the CTRL1_XL register are the ones with our measurement range config
      and so we shift values from 0-3 into this spot depending on the specified range */
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
      Serial.println("!accelRange");
      errFlag = 1;
      break;
  }

  uint16_t outputFrequencySwitch = outputFrequency;
  /* switch case wont let us use 12.5 as a switch, so round it to 12 for use here */
  if (outputFrequency == 12.5) {
    outputFrequencySwitch = 12;
  }

  /* bits[4:7] in the CTRL1_XL register are for ODR update frequency config
      and so we shift values into this spot depending on the specified frequency */
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
      Serial.println("!accelFrequency");
      errFlag = 1;
      break;
  }

  /* write our data package to the accelerometer which now contains information about measurement range and update freq
      (only write if we got valid inputs) */
  if (!errFlag) {
    writeSPI(slaveSelect, regAddress, data);
    /* now set the corrsponding attributes so we don't have to read from device if we need this value again */
    accelerometerOutputFrequency = outputFrequency;
    accelerometerMeasurementRange = range;
  }
  return (errFlag);
}

/* WHEN AT REST measure the standard deviation of the noise in the z-axis */
float PDC_LSM6DSO32::measureAccelerometerNoiseZ() {
  /* how many readings to calculate standard deviation over */
  uint8_t numReadings = 50;
  /* stat variables */
  float stdDev = 0;
  float mean = 0;
  float sum = 0;
  float prev_mean = 0;
  float accZ = 0;

  /* sometime the accelerometer will spit out useless values that are quite far from the expected value. use this threshold (larger than expected variation)
     to kick them out of the calculations */
  float threshold = 0.3;

  /* for the specified number of readings, measure the acceleration */
  for (uint8_t i = 1; i < numReadings; i++) {
    /* force rate of measurements to allow for proper processing */
    // TODO: consider replacing with a non-blocking function?
    delay(100);

    /* get the acceleration in the z-direction */
    accZ = readAccelerationZ();

    if (abs(GRAVITY_MAGNITUDE - accZ) > threshold) {
      /* for an erroneous reading, we should take the reading again to avoid skew */
      i -= 1;
    }
    else {
      /* Welford's algorithm for calculating standard deviation in real time. allows us to sidestep a large array of floats
          which would very quickly eat up memory & limit the samples we can test! */
      mean = mean + (accZ - mean) / i;
      sum = sum + (accZ - mean) * (accZ - prev_mean);
      prev_mean = mean;
    }
  }

  // TODO: determine if we should be dividing by n or by n-1
  stdDev = pow(sum / float(numReadings), 0.5);

  // TODO: consider putting a cap on stdDev incase of disturbance during setup
  // TODO: worth considering replacement or supplementation with a lookup table - if we want to change mode when switching to attitude determination, we can't measure the noise

  /* return the standard deviation of the noise */
  return (stdDev);
}
