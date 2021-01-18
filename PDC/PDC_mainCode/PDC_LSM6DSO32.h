/*******************************************************************
   In this file we define some class structures for the LSM6DSO32
    IMU.
   The class represents the interface and communications with the
    device. For example, the attributes are the pins on the PDC that
    the device is connected to, and the methods are commands for the
    PDC that trigger read/write events.
   One reason for using a class structure for the component is that
    in the future, more than one IMU may be used for redundancy, and
    this will allow us to simply create new class objects for each
    IMU, each with identical methods and easily assignable attributes
   A library for this component already exists via adafruit
    (see https://github.com/adafruit/Adafruit_LSM6DS), but we
    have built our own from scratch for a number of reasons
    - flexibility (if we want to add/remove functionality, it is
      easy to do so)
    - readability (adafruit lib uses lots of abstraction and
      requires much deeper understanding of all of this)
    - efficiency (this library should be much more lightweight
      than the adafruit one)
   Wherever possible, this file & the corresponding .cpp file
    are designed to account for the fact that in future iterations,
    the IMU used may change. hopefully if this is the case, all
    that should be required is the redefining of some high level
    parameters (e.g. register addresses). may not be entirely
    possible as some devices may separate paramters that are here
    contained in one register but we'll see how it goes...
 ************************** Example usage **************************

   --- (GLOBALLY) CREATE A NEW INSTANCE OF LSM6DSO32 ---
   const uint8_t IMU_SS = 5;
   PDC_LSM6DSO32 IMU(IMU_SS);

   --- CHECK IF IMU IS RESPONSIVE (REQUIRES SPI TO BE SET UP) ---
   if (!IMU.isAlive()) {
     // error!
   }

   --- RESTART IMU ---
   IMU.restart();

   --- CONFIGURE ACCELEROMETER TO UPDATE AT 3330Hz AND MEASURE ACROSS +/-32g ---
   IMU.accel.init(9, 2);

   --- READ Z AXIS ACCELERATION ---
   float accelZ = IMU.accel.readZ();

   --- READ Y AXIS ANGULAR RATE ---
   float rateY = IMU.gyro.readY();

 *******************************************************************/

//TODO maybe add a method which measures *all* values in one go for some reason

/* for detailed function information, see PDC_LSM6DSO32.h */

#include <Arduino.h>  /* bring some arduino syntax into the cpp files */
#include "PDC_SPI.h"  /* grab our SPI functions */
#include <stdio.h>    /* std stuff for cpp */
#include "headers.h"

const float GRAVITY_MAGNITUDE = 9.80665;    /* set the magnitude of the gravity vector */

/* DEVICE REGISTER ADDRESSES */
const uint8_t ACCX_L_DATA_REG = 0x28; /* the register address of the accelerometer LSB X-axis data register */
const uint8_t GYRX_L_DATA_REG = 0x22; /* the register address of the gyroscope LSB X-axis data register */
const uint8_t ACC_CTRL_REG = 0x10;    /* the register address of the accelerometer control register */
const uint8_t GYR_CTRL_REG = 0x11;    /* the register address of the gyroscope control register */
const uint8_t WHO_AM_I_REG = 0x0f;    /* the register address of the 'WHO_AM_I' (identification) register */

const uint8_t WHO_AM_I_VAL = 0b01101100;  /* the (fixed) value stored in the 'WHO_AM_I' register */

/************************************************************************
                   IMU CONFIG VALUES - WRITE TO CTRL_REG
    --------------------------------------------------------------------
    PARAM 1 (OUTPUT UPDATE FREQUENCY)  |   PARAM 2 (MEASUREMENT RANGE)
    0.  off                            |   0. 4g  / 250dps
    1.  12.5Hz                         |   1. --  / 125dps
    2.  26Hz                           |   2. 32g / 500dps
    3.  52Hz                           |   3. --  / --
    4.  104Hz                          |   4. 8g  / 1000dps
    5.  208Hz                          |   5. --  / --
    6.  416Hz                          |   6. 16g / 2000dps
    7.  833Hz                          |
    8.  1660Hz                         |
    9.  3330Hz                         |
    10. 6660Hz                         |
 ************************************************************************/
/* ACCELEROMETER OUTPUT DATA REGISTER UPDATE FREQUENCY (Hz) */
const uint8_t ACC_ODR_0    = 0;
const uint8_t ACC_ODR_12   = 1;
const uint8_t ACC_ODR_26   = 2;
const uint8_t ACC_ODR_52   = 3;
const uint8_t ACC_ODR_104  = 4;
const uint8_t ACC_ODR_208  = 5;
const uint8_t ACC_ODR_416  = 6;
const uint8_t ACC_ODR_833  = 7;
const uint8_t ACC_ODR_1660 = 8;
const uint8_t ACC_ODR_3330 = 9;
const uint8_t ACC_ODR_6660 = 10;

/* GYROSCOPE OUTPUT DATA REGISTER UPDATE FREQUENCY (Hz) */
const uint8_t GYR_ODR_0    = 0;
const uint8_t GYR_ODR_12   = 1;
const uint8_t GYR_ODR_26   = 2;
const uint8_t GYR_ODR_52   = 3;
const uint8_t GYR_ODR_104  = 4;
const uint8_t GYR_ODR_208  = 5;
const uint8_t GYR_ODR_416  = 6;
const uint8_t GYR_ODR_833  = 7;
const uint8_t GYR_ODR_1660 = 8;
const uint8_t GYR_ODR_3330 = 9;
const uint8_t GYR_ODR_6660 = 10;

/* ACCELEROMETER FULL SCALE MEASUREMENT RANGE (g) */
const uint8_t ACC_RNG_4  = 0;
const uint8_t ACC_RNG_8  = 4;
const uint8_t ACC_RNG_16 = 6;
const uint8_t ACC_RNG_32 = 2;

/* GYROSCOPE FULL SCALE MEASUREMENT RANGE (dps) */
const uint8_t GYR_RNG_125  = 1;
const uint8_t GYR_RNG_250  = 0;
const uint8_t GYR_RNG_500  = 2;
const uint8_t GYR_RNG_1000 = 4;
const uint8_t GYR_RNG_2000 = 6;

/**************************************************************************
    a child of the LSM6DSO32 IMU
      this class can be instantiated for an accelerometer or a gyroscope
      the class object then self contains useful bits of info like data
        address registers, configuration parameters, etc.
      includes methods to initialise, read data, measure noise
 **************************************************************************/
class IMUChild {
  private:
    float readValue(uint8_t LSB_address); /* a private method to read a value at the provided address */

    /* ---------- ATTRIBUTES ---------- */
    uint8_t devType;            /* the type of device 0 = accel, 1 = gyro */
    float outputFrequency;      /* the rate at which the device output should refresh (Hz [ac/gy]) */
    uint16_t measurementRange;  /* the full scale of measurements (+/- g [ac]; +/- dps [gy]) */
    float resolution;           /* the resolution of the measurement (milli-g per bit [ac]; milli-dps per bit [gy]) */


    uint8_t x_address;          /* the address of the LSB data register in the x-axis */
    uint8_t y_address;          /* the address of the LSB data register in the y-axis */
    uint8_t z_address;          /* the address of the LSB data register in the z-axis */
    uint8_t CTRL_address;       /* the address of the control register (for output frequency / measurement range) */

    static uint8_t slaveSelect; /* the pin on the PDC that connects to the IMU CS pin (static as is same for all children) */

  public:
    /* ---------- INITIALISER ---------- */
    IMUChild(void): 
      devType(0),
      outputFrequency(0),
      measurementRange(0),
      resolution(0),
      x_address(0),
      y_address(0),
      z_address(0),
      CTRL_address(0)
    {};
    
    /* ---------- METHODS ---------- */
    void addressSet(uint8_t x_add, uint8_t CTRL_add); /* remember the device data and control registers */
    void init(uint8_t f, uint8_t r);  /* configure the device over SPI - set the measurement range & output frequency */
    float readX();                    /* read data in the X axis */
    float readY();                    /* read data in the Y axis */
    float readZ();                    /* read data in the Z axis */
    float measureNoiseZ();            /* measure sensor noise */
};

/**************************************************************************
    a class for the LSM6DSO32 IMU
      this class contains an accelerometer and gyroscope 'sub' class of
        type 'IMUChild' defined above.
      this parent class contains some methods for more general functions
        like verifying SPI connection, restarting, etc.
 **************************************************************************/
class PDC_LSM6DSO32 {
  private:
    /* ---------- ATTRIBUTES ---------- */
    uint8_t slaveSelect;  /* the pin on the PDC that the IMU CS pin connects to. is set on contruction */

  public:
    IMUChild accel; /* an accelerometer child */
    IMUChild gyro;  /* a gyroscope child */

    /* ---------- CONSTRUCTOR ---------- */
    PDC_LSM6DSO32(uint8_t CS) {
      slaveSelect = CS; /* set slaveSelect to the specified SS pin */
      accel.addressSet(ACCX_L_DATA_REG, ACC_CTRL_REG);  /* tell the accelerometer where to find its addresses */
      gyro.addressSet(GYRX_L_DATA_REG, GYR_CTRL_REG);   /* teel the gyroscope where to find its addresses */
    };

    /* ---------- METHODS --------- */
    bool isAlive(); /* check if connected and responsive */
    void restart(); /* restart the device */
    uint8_t selfTest();
};
