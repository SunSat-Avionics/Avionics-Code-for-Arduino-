// Methods TODO:
// read values (gyro xyz, accel xy, temp?, general reg?)
// write values (measurement range)
// measure noise (loop for fixed time and calculate RMS noise)
// measure offset

#include "PDC_LSM6DSO32.h"
#include "PDC_SPI.h"
#include <stdio.h>

bool PDC_LSM6DSO32::isAlive() {
  /* have a code to signify result - true is success */
  bool successCode = 0;
  uint8_t WHO_AM_I = 0;
  uint8_t WHO_AM_I_expect = 0b01101100;
  uint8_t regAddress = 0x0f;
  
  /* read the 'WHO_AM_I' register on the IMU (at 0x0f) */
  WHO_AM_I = readSPI(slaveSelect, regAddress, 1);

  /* check that it's what we expect */
  if (WHO_AM_I == WHO_AM_I_expect) {
    /* if it is, set our success code to true */
    successCode = 1;
  }

  return (successCode);
}

/* read the z-axis acceleration and convert to an acceleration in m/s2 */
float PDC_LSM6DSO32::readAccelerationZ() {
  uint16_t rawAccelZ;
  float accelerationZ = 0;
  uint8_t regAddress = 0x2C;
  
  /* the resolution of the accelerometer is its range (e.g. +/4g) divided by the number of combinations available
      the accelerometer output is 16bits, so 2^16 is our combinations.
      units are milli-g per bit, so times 1000*/
  float accelResolution = (accelerometerMeasurementRange * 2.0 * 1000.0) / 65536.0;
  
  /* read z-axis acceleration.
      note in CTRL3_C, there is a default enabled bit which auto-increments the register address when reading multiple bytes so we dont need to read the H and L
      registers separately, as long as we tell readSPI() that we expect 2 bytes in the last argument */
  rawAccelZ = readSPI(slaveSelect, regAddress, 2);

  /* convert our output into an actual acceleration value in ms/2
      the raw value is somewhere in our measurement range, so multiply by resolution to get back to absolute value, then multiply by g to get m/s^2 */
  accelerationZ = (float(rawAccelZ) / 1000) * accelResolution * 9.80665;
  Serial.print("Acceleration: ");
  Serial.println(accelerationZ, 10);
  return (accelerationZ);
}

bool PDC_LSM6DSO32::setupAccelerometer(float outputFrequency, uint8_t range) {
  uint8_t data = 0;
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
      (only write if we got valid inputs)*/
  if (!errFlag) {
    writeSPI(slaveSelect, 0x10, data);
    /* now set the corrsponding attributes so we don't have to read from device if we need this value again */
    accelerometerOutputFrequency = outputFrequency;
    accelerometerMeasurementRange = range;
  }

  return (errFlag);
}

float PDC_LSM6DSO32::measureAccelerometerNoiseZ() {
  uint8_t numReadings = 10; /* how many readings to calculate standard deviation over (25 seems to be max!) */
  float accelerationZ[numReadings];
  float avg = 0;
  float stdDev = 0;
    
  /* for the specified number of readings, measure the acceleration */
  for (uint8_t i = 0; i < numReadings; i++) {
    /* get the acceleration in the z-direction */
    accelerationZ[i] = readAccelerationZ();
    /* cumulative sum */
    avg += accelerationZ[i];
    // TODO: implement some sort of timing between readings (e.g. twice per second? faster? wait for new data?)
  }

  /* average of all measurements */
  avg /= numReadings;

  /* calculate the standard deviation of all the readings */
  for(uint8_t i = 0; i < numReadings; i++){
    accelerationZ[i] = pow((accelerationZ[i] - avg), 2);
    stdDev += accelerationZ[i];
  }
  stdDev /= numReadings;
  stdDev = pow(stdDev, 0.5);

  /* return the standard deviation of the noise */
  return (stdDev);
}
