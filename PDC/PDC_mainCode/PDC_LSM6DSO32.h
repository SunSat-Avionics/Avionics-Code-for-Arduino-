/*******************************************************************
 * In this file we define some class structures for the LSM6DSO32
 *  IMU. 
 * A library for this component already exists via adafruit
 *  (see https://github.com/adafruit/Adafruit_LSM6DS), but we 
 *  have built our own from scratch for a number of reasons
 *  - flexibility (if we want to add/remove functionality, it is 
 *    easy to do so)
 *  - readability (adafruit lib uses lots of abstraction and
 *    requires much deeper understanding of all of this)
 *  - efficiency (this library should be much more lightweight
 *    than the adafruit one)
 * 
 ************************** Example usage **************************
 * 
 * --- CREATE A NEW INSTANCE OF LSM6DSO32 ---
 * const int IMU_SS = 5;
 * PDC_LSM6DSO32 IMU(IMU_SS);
 * 
 * --- CHECK IF IMU IS RESPONSIVE (REQUIRES SPI TO BE SET UP) ---
 * if (!IMU.isAlive()) {
 *   // error!
 * }
 * 
 * --- RESTART IMU ---
 * IMU.restart();
 * 
 * --- CONFIGURE ACCELEROMETER TO UPDATE AT 3330Hz AND MEASURE ACROSS +/-32g ---
 * IMU.accel.init(9, 2);
 * 
 * --- READ Z AXIS ACCELERATION ---
 * float accelZ = IMU.accel.readZ();
 * 
 * --- READ Y AXIS ANGULAR RATE ---
 * float rateY = IMU.gyro.readY();
 * 
 *******************************************************************/

//TODO maybe add a method which measures *all* values in one go for some reason

#include <Arduino.h>  /* bring some arduino syntax into the cpp files */
#include "PDC_SPI.h"  /* grab our SPI functions */
#include <stdio.h>    /* std stuff for cpp */

const float GRAVITY_MAGNITUDE = 9.80665; /* set the magnitude of the gravity vector */

/**************************************************************************
 *  a child of the LSM6DSO32 IMU
 *    this class can be instantiated for an accelerometer or a gyroscope
 *    the class object then self contains useful bits of info like data
 *      address registers, configuration parameters, etc.
 *    includes methods to initialise, read data, measure noise
 **************************************************************************/
class IMUChild{
  private:
    float readValue(uint8_t LSB_address); /* a private method to read a value at the provided address */
    
    /* ---------- ATTRIBUTES ---------- */
    float outputFrequency;      /* the rate at which the device output should refresh (Hz [ac/gy]) */
    uint16_t measurementRange;  /* the full scale of measurements (+/- g [ac]; +/- dps [gy]) */
    float resolution;           /* the resolution of the measurement (milli-g per bit [ac]; milli-dps per bit [gy]) */
    
    uint8_t x_address;          /* the address of the LSB data register in the x-axis */
    uint8_t y_address;          /* the address of the LSB data register in the y-axis */
    uint8_t z_address;          /* the address of the LSB data register in the z-axis */
    uint8_t CTRL_address;       /* the address of the control register (for output frequency / measurement range) */

    uint8_t slaveSelect;        /* the pin on the PDC that connects to the IMU CS pin */

  public:
    /* ---------- CONSTRUCTOR ---------- */
    void addressSet(uint8_t x_add, uint8_t y_add, uint8_t z_add, uint8_t CTRL_add, uint8_t CS) {
      /* internally remember the locations of useful addresses */
      x_address = x_add;  
      y_address = y_add;
      z_address = z_add;
      CTRL_address = CTRL_add;
      
      /* internally remember the slave select pin so we can modify it's state */
      slaveSelect = CS;
    };

    /* ---------- METHODS ---------- */
    void init(uint8_t f, uint8_t r);  /* configure the device over SPI - set the measurement range & output frequency */
    float readX();                    /* read data in the X axis */
    float readY();                    /* read data in the Y axis */
    float readZ();                    /* read data in the Z axis */
    float measureNoiseZ();            /* measure sensor noise */
};

/**************************************************************************
 *  a class for the LSM6DSO32 IMU
 *    this class contains an accelerometer and gyroscope 'sub' class of
 *      type 'IMUChild' defined above. 
 *    this parent class contains some methods for more general functions
 *      like verifying SPI connection, restarting, etc.
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

      /* properly attribute register addresses to the children */
      accel.addressSet(0x28, 0x2A, 0x2C, 0x10, CS); 
      gyro.addressSet(0x22, 0x24, 0x26, 0x11, CS); 
    };
    
    /* ---------- METHODS --------- */
    bool isAlive(); /* check if connected and responsive */
    void restart(); /* restart the device */
};
