// Methods TODO:
// read temp?
// measure offset
// a 'read all' method? what if we want to read x, y, z all at once?
// add an auto-set for the std dev measurement

/* for example usage, see PDC_LSM6DSO32.h */

#include "PDC_LSM6DSO32.h"  /* include the definition of the class */

/*********************************************************
   @brief  Read component ID
   @retval 1 in case of success, 0 otherwise
 *********************************************************/
bool PDC_LSM6DSO32::isAlive() {
  bool isAlive = 0;     /* signify result. 1 is success */
  uint8_t WHO_AM_I[1];  /* internal variable to hold the output */

  readSPI(slaveSelect, WHO_AM_I_REG, 1, WHO_AM_I);  /* read the 'WHO_AM_I' register on the IMU */

  /* check that it's what we expect */
  if (WHO_AM_I[0] == WHO_AM_I_VAL) {
    isAlive = 1;  /* if it is, set our success code to true */
  }
  
  return (isAlive);
}

/*********************************************************
   @brief  Restart IMU
 *********************************************************/
void PDC_LSM6DSO32::restart() {
  uint8_t dataToWrite = 1 || 1 << 7;  /* set bits 7 and 0 to high which reboots memory and resets software */

  writeSPI(slaveSelect, CTRL3_C_REG, dataToWrite);  /* write the command to the CTRL3_C register */
  delay(2000); /* wait for it to properly start up again */
}

/*********************************************************
   @brief  Self-test the accelerometer and gyroscope
   @retval 0 if success, 1 otherwise
 *********************************************************/
 // TODO: read the initial config (rng, frq) and reset to these after self testing
uint8_t PDC_LSM6DSO32::selfTest() {
  /* acc and gyr can self test. electrostatic force applied to the elements to artifically displace & register a reading
      we can then measure the value when the self test is enabled vs disabled & compare the outputs
      datasheet specifies a range of outputs to expect in self test, so we verify this
  */

  float selfTestOn[3];    /* store values when self test is on */
  float selfTestOff[3];   /* store values when self test is off */
  float difference = 0.0; /* calculate the on/off differences */

  uint8_t j;          /* loop index counter */
  uint16_t dly = 500; /* delay (ms) to allow for self-test to switch on/off */
  uint8_t flag = 0;   /* flag to return. 0 if successful */

  /* ---------- EXPECTED RANGE DEFINITIONS ---------- */
  float accMin = 50.0 / 1000.0;   /* the datasheet-specified minimum self-test change (converted to g) */
  float accMax = 1700.0 / 1000.0; /* the datasheet-specified maximum self-test change (converted to g) */
  float gyrMin = 150.0;           /* the datasheet-specified minimum self-test change (at 2000dps range) */
  float gyrMax = 700.0;           /* the datasheet-specified maximum self-test change (at 2000dps range) */

  uint8_t selfTestAccel = 1;      /* data to write to CTRL5_C to turn accelerometer self test on */
  uint8_t selfTestGyro = 1 << 2;  /* data to write to CTRL5_C to turn gyroscope self test on */
  accel.init(ACC_ODR_3330, ACC_RNG_4);    /* set accel measurement range to 4g to match datasheet test conditions */
  gyro.init(GYR_ODR_3330, GYR_RNG_2000);  /* set gyro measurement range to 2000dps to match datasheet test conditions */

  /* ---------- ACCELEROMETER SELF TEST ---------- */
  /* read all 3 accelerometer axes with self-test off */
  selfTestOff[0] = accel.readX();
  selfTestOff[1] = accel.readY();
  selfTestOff[2] = accel.readZ();

  /* turn on accelerometer self test and wait */
  writeSPI(slaveSelect, CTRL5_C_REG, selfTestAccel);
  delay(dly);

  /* read all 3 accelerometer axes with self-test on */
  selfTestOn[0] = accel.readX();
  selfTestOn[1] = accel.readY();
  selfTestOn[2] = accel.readZ();

  /* turn off accelerometer self test and wait */
  writeSPI(slaveSelect, CTRL5_C_REG, 0);
  delay(dly);

  /* calculate the difference for each axis and check that it is within the expected range specified on the datasheet */
  for (j = 0; j < 3; j++) {
    difference = selfTestOn[j] - selfTestOff[j];
    if ((difference < accMin) || (difference > accMax)) {
      flag = 1;
    }
  }

  difference = 0; /* reset */

  /* ---------- GYROSCOPE SELF TEST ---------- */
  /* read all 3 gyroscope axes with self-test off */
  selfTestOff[0] = gyro.readX();
  selfTestOff[1] = gyro.readY();
  selfTestOff[2] = gyro.readZ();

  /* turn on gyroscope self test and wait */
  writeSPI(slaveSelect, CTRL5_C_REG, selfTestGyro);
  delay(dly);

  /* read all 3 gyroscope axes with self-test on */
  selfTestOn[0] = gyro.readX();
  selfTestOn[1] = gyro.readY();
  selfTestOn[2] = gyro.readZ();

  /* turn off gyroscope self test and wait */
  writeSPI(slaveSelect, CTRL5_C_REG, 0);
  delay(dly);

  /* calculate the difference for each axis and check that it is within the expected range specified on the datasheet */
  for (j = 0; j < 3; j++) {
    difference = selfTestOn[j] - selfTestOff[j];
    if ((difference < gyrMin) || (difference > gyrMax)) {
      flag = 1;
    }
  }

  return (flag);
}

/*****************************************************************************
   @brief  set the child slave select to be the defined IMU slave select pin
 *****************************************************************************/
uint8_t IMUChild::slaveSelect = IMU_SS;

/*********************************************************
   @brief  Internally take note of important registers
   @param  the address of the x-axis LSB data register
   @param  the address of the control register
 *********************************************************/
void IMUChild::addressSet(uint8_t x_add, uint8_t CTRL_add) {
  x_address = x_add;        /* set x LSB address attribute as specified */
  y_address = x_add + 2;    /* y LSB address is then two along */
  z_address = x_add + 4;    /* z LSB address is another two along */
  CTRL_address = CTRL_add;  /* and then the address to configure this child */

  /* work out if we've just created an accelerometer or a gyroscope child */
  if (CTRL_address == ACC_CTRL_REG) {
    devType = 0;
  }
  else if (CTRL_address == GYR_CTRL_REG) {
    devType = 1;
  }
}

/*********************************************************
   @brief  Initialise the component
   @param  code for output update frequency
   @param  code for measurement range
 *********************************************************/
void IMUChild::init(uint8_t frequency, uint8_t range) {
  uint8_t dataToWrite = 0;

  dataToWrite |= range << 1;     /* set bits [3:1] to configure range. note accel config actually only uses [3:2] so have padded with bit 1 to make equivalent between devices */
  dataToWrite |= frequency << 4; /* set bits [7:4] to configure output frequency */

  /* set the internally stored output frequency */
  switch (frequency) {
    case (0):  outputFrequency = 0;    break;
    case (1):  outputFrequency = 12.5; break;
    case (2):  outputFrequency = 26;   break;
    case (3):  outputFrequency = 52;   break;
    case (4):  outputFrequency = 104;  break;
    case (5):  outputFrequency = 208;  break;
    case (6):  outputFrequency = 416;  break;
    case (7):  outputFrequency = 833;  break;
    case (8):  outputFrequency = 1660; break;
    case (9):  outputFrequency = 3330; break;
    case (10): outputFrequency = 6660; break;
    default:  outputFrequency = 0;    break;
  }

  /* set the internally stored measurement range (if condition checks if accelerometer or gyroscope) */
  if (devType == 0) {
    switch (range) {
      case (0): measurementRange = 4;  break;
      case (2): measurementRange = 32; break;
      case (4): measurementRange = 8;  break;
      case (6): measurementRange = 16; break;
      default: measurementRange = 0;  break;
    }
  } else if (devType == 1) {
    switch (range) {
      case (0):  measurementRange = 250;   break;
      case (1):  measurementRange = 125;   break;
      case (2):  measurementRange = 500;   break;
      case (4):  measurementRange = 1000;  break;
      case (6):  measurementRange = 2000;  break;
      default:  measurementRange = 0;     break;
    }
  }

  writeSPI(slaveSelect, CTRL_address, dataToWrite);  /* write the data to the control register */

  resolution = (measurementRange * 2.0 * 1000.0) / 65536.0; /* calculate the device resolution per bit (milli-g or milli-dps) */
}

/*********************************************************
   @brief  Read a value from a specified register
   @param  the address of the LSB data register
   @retval the value in g [ac] or dps [gy]
 *********************************************************/
float IMUChild::readValue(uint8_t LSB_address) {
  uint8_t rawValue[2];        /* we will read two bytes from the device into here */
  int16_t rawValueConcat = 0; /* we will concatenate the two bytes into a single value here */
  float measuredValue = 0;    /* and we will convert the concatenated value into a 'measured' value here */

  readSPI(slaveSelect, LSB_address, 2, rawValue); /* read two bytes from the device.
                                                      since CTRLC_3 'IF_INC' bit is enabled, the address will
                                                      auto-increment and read the LSB then MSB registers so we
                                                      have the two bytes we need! */

  rawValueConcat = (rawValue[1] << 8) | rawValue[0];  /* concatenate the two bytes into a single val by shifting the MSB up by one byte */

  measuredValue = (float(rawValueConcat) / 1000) * resolution;  /* use the sensor resolution to convert raw value into an actual measurement */

  return (measuredValue);
}

/*********************************************************
   @brief  Read data from the X axis
   @retval the measured X axis value in g [ac] or dps [gy]
 *********************************************************/
float IMUChild::readX() {
  float xValue = readValue(x_address);

  logFileLine.accelerometerX = xValue;
  
  return (xValue);
}

/*********************************************************
   @brief  Read data from the Y axis
   @retval the measured Y axis value in g [ac] or dps [gy]
 *********************************************************/
float IMUChild::readY() {
  float yValue = readValue(y_address);

  logFileLine.accelerometerY = yValue;
  
  return (yValue);
}

/*********************************************************
   @brief  Read data from the Y axis
   @retval the measured Y axis value in g [ac] or dps [gy]
 *********************************************************/
float IMUChild::readZ() {
  float zValue = readValue(z_address);

  logFileLine.accelerometerZ = zValue;
  
  return (zValue);
}

/*********************************************************
   @brief  measure standard deviation of noise in an axis
   @retval the measurement noise standard devation in
           g [ac] or dps [gy]
 *********************************************************/
float IMUChild::measureNoiseZ() {
  uint8_t numReadings = 50; /* how many readings to calculate standard deviation over */
  /* stat variables */
  float stdDev = 0;
  float mean = 0;
  float sum = 0;
  float prev_mean = 0;

  float accZ = 0; /* the acceleration in the z-direction in g */

  float threshold = 0.3/GRAVITY_MAGNITUDE;  /* reject rubbish values that exceed a threshold of reasonable expectation */

  uint32_t startTime = millis();
  
  /* for the specified number of readings, measure the acceleration */
  for (uint8_t i = 1; i < numReadings; i++) {
    // TODO: consider replacing with a non-blocking function?
    delay(100); /* force rate of measurements to allow for proper processing */

    accZ = readZ(); /* get z-axis acceleration */

    /* |1 g - measured g| should be approx zero */
    if (abs(1 - accZ) > threshold) {
      i -= 1; /* for an erroneous reading, we should take the reading again to avoid skew */
    }
    else {
      /* Welford's algorithm for calculating standard deviation in real time
          allows us to sidestep a large array of floats which would very quickly eat up memory & limit the samples we can test!
      */
      mean = mean + (accZ - mean) / i;
      sum = sum + (accZ - mean) * (accZ - prev_mean);
      prev_mean = mean;
    }

    /* timeout control */
    if((millis() - startTime) > 10000){
      break; 
    }
  }

  // TODO: determine if we should be dividing by n or by n-1
  stdDev = pow(sum / float(numReadings), 0.5);

  // TODO: consider putting a cap on stdDev incase of disturbance during setup
  // TODO: maybe we should go between the measurement modes on the ground and measure stddev in each of them and store results internally??

  return (stdDev);
}
