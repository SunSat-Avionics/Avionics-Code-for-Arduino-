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

   --- CONFIGURE ALTIMETER TO UPDATE AT AND SET MEASUREMENT RESOLUTIONS ---
   altimeter.init(ALT_ODR_200, ALT_OSR_PRESS_HIGH, ALT_OSR_TEMP_ULTRALOW);
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

const float SEA_LEVEL_PRESSURE = 1013.25; /* the pressure at sea level in hPa, for calculations of altitude */

/* DEVICE REGISTER ADDRESSES */
const uint8_t CHIP_ID_REG = 0x00; /* the register address of the 'CHIP_ID' (identification) register */
const uint8_t CHIP_ID_VAL = 0X50; /* the (fixed) value stored in the 'CHIP_ID' register */

const uint8_t DATA_0_REG = 0x04;  /* the register address of the first data register. (pressure is at DATA_0,1,2 and temperature is at DATA_3,4,5) */

const uint8_t OSR_REG = 0x1C;     /* the register address of the 'OSR' (oversampling settings) register */
const uint8_t ODR_REG = 0x1D;     /* the register address of the 'ODR' (output data rates) register */

/* non-volatile memory (NVM) device specific pressure and temperature compensation parameter register addresses */
const uint8_t NVM_PAR_T1_REG_1 = 0x31;
const uint8_t NVM_PAR_T1_REG_2 = 0x32;
const uint8_t NVM_PAR_T2_REG_1 = 0x33;
const uint8_t NVM_PAR_T2_REG_2 = 0x34;
const uint8_t NVM_PAR_T3_REG_1 = 0x35;

const uint8_t NVM_PAR_P1_REG_1  = 0x36;
const uint8_t NVM_PAR_P1_REG_2  = 0x37;
const uint8_t NVM_PAR_P2_REG_1  = 0x38;
const uint8_t NVM_PAR_P2_REG_2  = 0x39;
const uint8_t NVM_PAR_P3_REG_1  = 0x3A;
const uint8_t NVM_PAR_P4_REG_1  = 0x3B;
const uint8_t NVM_PAR_P5_REG_1  = 0x3C;
const uint8_t NVM_PAR_P5_REG_2  = 0x3D;
const uint8_t NVM_PAR_P6_REG_1  = 0x3E;
const uint8_t NVM_PAR_P6_REG_2  = 0x3F;
const uint8_t NVM_PAR_P7_REG_1  = 0x40;
const uint8_t NVM_PAR_P8_REG_1  = 0x41;
const uint8_t NVM_PAR_P9_REG_1  = 0x42;
const uint8_t NVM_PAR_P9_REG_2  = 0x43;
const uint8_t NVM_PAR_P10_REG_1 = 0x44;
const uint8_t NVM_PAR_P11_REG_1 = 0x45;

/************************************************************************************************************
                                          ALTIMETER CONFIG VALUES
  (aliases for each value are defined in PDC_BMP.h)
  ---------------------------------------------------------------------------------------------------------
  PARAM 1 (OUTPUT UPDATE FREQUENCY)  | PARAM 2 (PRESSURE RESOLUTION)     | PARAM 3 (TEMPERATURE RESOLUTION)
  0.  200Hz                          | 0.  Ultra Low Power (2.64Pa)      | 0.  Ultra Low Power (0.005C)
  1.  100Hz                          | 1.  Low Power (1.32Pa)            | 1.  Low Power (0.0025C)
  2.  50Hz                           | 2.  Standard Resolution (0.66Pa)  | 2.  Standard Resolution (0.0012C)
  3.  25Hz                           | 3.  High Resolution (0.33Pa)      | 3.  High Resolution (0.0006C)
  4.  12.5Hz                         | 4.  Ultra High Resoluton (0.17Pa) | 4.  Ultra High Resolution (0.0003C)
  5.  6.25Hz                         | 5.  Highest Resolution (0.0085Pa) | 5.  Highest Resolution (0.00015C)
  6.  3.1Hz                          |                                   |
  7.  0.78Hz                         |                                   |
  8.  0.39Hz                         |                                   |
  9.  0.2Hz                          |                                   |
  10. 0.1Hz                          |                                   |
  11. 0.05Hz                         |                                   |
  12. 0.02Hz                         |                                   |
  13. 0.01Hz                         |                                   |
 ************************************************************************************************************/
/* ALTIMETER OUTPUT DATA REGISTER UPDATE FREQUENCY (Hz) */
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

/* ALTIMETER PRESSURE RESOLUTION */
const uint8_t ALT_OSR_PRESS_ULTRALOW  = 0;
const uint8_t ALT_OSR_PRESS_LOW       = 1;
const uint8_t ALT_OSR_PRESS_STANDARD  = 2;
const uint8_t ALT_OSR_PRESS_HIGH      = 3;
const uint8_t ALT_OSR_PRESS_ULTRAHIGH = 4; 
const uint8_t ALT_OSR_PRESS_HIGHEST   = 5;

/* ALTIMETER TEMPERATURE RESOLUTION */
const uint8_t ALT_OSR_TEMP_ULTRALOW  = 0;
const uint8_t ALT_OSR_TEMP_LOW       = 1;
const uint8_t ALT_OSR_TEMP_STANDARD  = 2;
const uint8_t ALT_OSR_TEMP_HIGH      = 3;
const uint8_t ALT_OSR_TEMP_ULTRAHIGH = 4; 
const uint8_t ALT_OSR_TEMP_HIGHEST   = 5;

/**************************************************************************
    a class for the BMP388 altimeter
 **************************************************************************/
class PDC_BMP388 {
  private:
    uint32_t readValue(uint8_t data_address0);  /* a private method to read a value at the provided address */
    void getCompensationParams();               /* get the pressure and temperature compensation parameters */
    void addressSet(uint8_t data_0_add, uint8_t ODR_add, uint8_t OSR_add); /* remember the device data and control registers */
    
    /* ---------- ATTRIBUTES ---------- */
    uint8_t slaveSelect;  /* the pin on the PDC that the altimeter CS pin connects to. is set on contruction */
    
    float outputFrequency;            /* the rate at which the device output should refresh (Hz) */
    uint8_t pressureOversampling;     /* the oversampling of the pressure measurement */
    uint8_t temperatureOversampling;  /* the oversampling of the temperature measurement */
    uint8_t pressureAddress_0;        /* the address of the first pressure data register */
    uint8_t temperatureAddress_0;     /* the address of the first temperature data register */
    uint8_t ODR_address;              /* the address of the ODR register (for output frequency) */
    uint8_t OSR_address;              /* the address of the OSR register (for oversampling config) */

    float temperatureCompensationArray[3];  /* array for the device specific temperature compensation parameters */
    float pressureCompensationArray[11];    /* array for the device specific pressure compensation parameters */

  public:
    /* ---------- CONSTRUCTOR ---------- */
    PDC_BMP388(uint8_t CS) {
      slaveSelect = CS;                         /* set slaveSelect to the specified SS pin */
      addressSet(DATA_0_REG, ODR_REG, OSR_REG); /* tell the altimeter where to find its registers */
      outputFrequency = 0;                      /* initialise the class output frequency attribute as 0 */
      pressureOversampling = 0;
      temperatureOversampling = 0;
    };

    /* ---------- METHODS --------- */
    bool isAlive();               /* check if connected and responsive */
    void restart();               /* soft reset the device and enable temp/press measurement */
    void init(uint8_t f, uint8_t r_p, uint8_t r_t); /* configure the device over SPI - set the output frequency and resolution */
    float readPress();            /* read the raw pressure measurement and convert to 'actual' value [degC] */
    float readTemp();             /* read the raw temperature measurement and convert to 'actual' value [Pa] */
    float readAltitude();         /* use the compensated pressure to calculate absolute altitude [m] */
};
