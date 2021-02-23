/*******************************************************************
   In this file we define some class structures for the BMP388
    altimeter.
   The class represents the interface and communications with the
    device. For example, the attributes are the pins on the PDC that
    the device is connected to, and the methods are commands for the
    PDC that trigger read/write events.
   One reason for using a class structure for the component is that
    in the future, more than one altimeter may be used for redundancy,
    and this will allow us to simply create new class objects for each
    one, each with identical methods and easily assignable attributes
   A library for this component already exists via adafruit
    (see https://github.com/adafruit/Adafruit_BMP3XX), but we
    have built our own from scratch for a number of reasons
    - flexibility (if we want to add/remove functionality, it is
      easy to do so)
    - readability (adafruit lib uses lots of abstraction and
      requires much deeper understanding of all of this)
    - efficiency (this library should be much more lightweight
      than the adafruit one)
   Wherever possible, this file & the corresponding .cpp file
    are designed to account for the fact that in future iterations,
    the altimeter used may change. hopefully if this is the case, all
    that should be required is the redefining of some high level
    parameters (e.g. register addresses). may not be entirely
    possible as some devices may separate paramters that are here
    contained in one register but we'll see how it goes...
 ************************** Example usage **************************

   --- (GLOBALLY) CREATE A NEW INSTANCE OF BMP388 ---
   const uint8_t altimeter_SS = 4;
   PDC_BMP388 altimeter(altimeter_SS);

   --- CHECK IF ALTIMETER IS RESPONSIVE (REQUIRES SPI TO BE SET UP) ---
   if (!altimeter.isAlive()) {
     // error!
   }

   --- RESTART ALTIMETER ---
   altimeter.restart();

   --- CONFIGURE ALTIMETER TO UPDATE AT 200Hz ---
   altimeter.init(ALT_ODR_200);
    // note that this .h file includes aliases for each possible update rate
    
   --- READ Z AXIS ACCELERATION ---
   float accelZ = IMU.accel.readZ();

   --- READ Y AXIS ANGULAR RATE ---
   float rateY = IMU.gyro.readY();

 *******************************************************************/

//TODO maybe add a method which measures *all* values in one go for some reason

/* for detailed function information, see PDC_BMP388.cpp */

#include <Arduino.h>  /* bring some arduino syntax into the cpp files */
#include "PDC_SPI.h"  /* grab our SPI functions */
#include <stdio.h>    /* std stuff for cpp */
#include "headers.h"

/* DEVICE REGISTER ADDRESSES */
//const uint8_t ACCX_L_DATA_REG = 0x28; /* the register address of the accelerometer LSB X-axis data register */
//const uint8_t GYRX_L_DATA_REG = 0x22; /* the register address of the gyroscope LSB X-axis data register */
const uint8_t ODR_REG = 0x1D;       /* the register address of the 'ODR' (output data rates) register */
const uint8_t CHIP_ID_REG = 0x00;   /* the register address of the 'CHIP_ID' (identification) register */

const uint8_t CHIP_ID_VAL = 0X50;   /* the (fixed) value stored in the 'CHIP_ID' register */

/************************************************************************
                ALTIMETER CONFIG VALUES - WRITE TO ODR
    --------------------------------------------------------------------
    PARAM 1 (OUTPUT UPDATE FREQUENCY)
    0.  200Hz
    1.  100Hz
    2.  50Hz
    3.  25Hz
    4.  12.5Hz
    5.  6.25Hz
    6.  3.1Hz
    7.  0.78Hz
    8.  0.39Hz
    9.  0.2Hz
    10. 0.1Hz
    11. 0.05Hz
    12. 0.02Hz
    13. 0.01Hz
 ************************************************************************/
/* ACCELEROMETER OUTPUT DATA REGISTER UPDATE FREQUENCY (Hz) */
const uint8_t ALT_ODR_200  = 0;
const uint8_t ALT_ODR_100  = 1;
const uint8_t ALT_ODR_50   = 2;
const uint8_t ALT_ODR_25   = 3;
const uint8_t ALT_ODR_12p5 = 4;
const uint8_t ALT_ODR_6p25 = 5;
const uint8_t ALT_ODR_3p1  = 6;
const uint8_t ALT_ODR_0p78 = 7;
const uint8_t ALT_ODR_0p39 = 8;
const uint8_t ALT_ODR_0p2  = 9;
const uint8_t ALT_ODR_0p1  = 10;
const uint8_t ALT_ODR_0p05 = 11;
const uint8_t ALT_ODR_0P02 = 12;
const uint8_t ALT_ODR_0p01 = 13;

/**************************************************************************
    a class for the BMP388 altimeter
 **************************************************************************/
class PDC_BMP388 {
  private:
    float readValue(uint8_t LSB_address); /* a private method to read a value at the provided address */
    
    /* ---------- ATTRIBUTES ---------- */
    uint8_t slaveSelect;  /* the pin on the PDC that the altimeter CS pin connects to. is set on contruction */

    float outputFrequency;      /* the rate at which the device output should refresh (Hz) */

    uint8_t x_address;          /* the address of the LSB data register in the x-axis */
    uint8_t y_address;          /* the address of the LSB data register in the y-axis */
    uint8_t z_address;          /* the address of the LSB data register in the z-axis */
    uint8_t ODR_address;        /* the address of the ODR address (for output frequency) */

  public:
    /* ---------- CONSTRUCTOR ---------- */
    PDC_BMP388(uint8_t CS) {
      slaveSelect = CS; /* set slaveSelect to the specified SS pin */
      //accel.addressSet(ACCX_L_DATA_REG, ACC_CTRL_REG);  /* tell the accelerometer where to find its addresses */
      //gyro.addressSet(GYRX_L_DATA_REG, GYR_CTRL_REG);   /* teel the gyroscope where to find its addresses */
      outputFrequency = 0;
    };

    /* ---------- METHODS --------- */
    bool isAlive(); /* check if connected and responsive */
    void restart(); /* soft reset the device and enable temp/press measurement */
    
    //void addressSet(uint8_t x_add, uint8_t CTRL_add); /* remember the device data and control registers */
    void init(uint8_t f); /* configure the device over SPI - set the output frequency */
    void readPressure();
};
