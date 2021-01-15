#include <Arduino.h>  /* bring some arduino syntax into the cpp files */
#include "PDC_SPI.h"  /* grab our SPI functions */
#include <stdio.h>    /* std stuff for cpp */

const float GRAVITY_MAGNITUDE = 9.80665; /* set the magnitude of the gravity vector */

class IMUPart{
  private:
    float outputFrequency;     
    uint16_t measurementRange;
    float resolution;
    
    uint8_t x_address;
    uint8_t y_address;
    uint8_t z_address;

    uint8_t slaveSelect;

  public:
    void addressSet(uint8_t x_add, uint8_t y_add, uint8_t z_add, uint8_t CS) {
      x_address = x_add;
      y_address = y_add;
      z_address = z_add;
      slaveSelect = CS;
    };
    bool init(float outputFrequency, uint16_t range);
    float readX();
    float readY();
    float readZ();
    float readValue(uint8_t LSB_address, uint8_t slaveSelect);
};

/* LSM6DSO32 CLASS
 *  we define an LSM6DSO32 class to keep everything packed away neatly. 
 *  it allows us to keep hold of things that we set e.g. measurement range, so that we don't have to read them from the device directly
 *  also gives us fine control over functionality
 *  adafruit have a library for this component already but this is more readable, flexible, and lightweight
 */
class PDC_LSM6DSO32 {
  private:
    /* ---------- ATTRIBUTES ---------- */
    uint8_t slaveSelect;            /* the pin on the PDC that the IMU CS pin connects to. is set on contruction */
    float accelOutputFrequency;     /* the rate at which the accelerometer ODR updates */
    float gyroOutputFrequency;      /* the rate at which the gyroscope ODR updates */
    uint8_t accelMeasurementRange;  /* the accelerometer measurement range in g (can be +/-4, 8, 16, 32) */
    uint16_t gyroMeasurementRange;  /* the gyroscope measurement range in dps (can be +/-125, 250, 500, 1000, 2000) */
    float accelResolution;          /* the resolution of the accelerometer in milli-g per bit */
    float gyroResolution;           /* the resolution of the gyroscope in milli-dps per bit */
    
  public:
    IMUPart accel;
    IMUPart gyro;
    
    /* ---------- CONSTRUCTOR ---------- */
    PDC_LSM6DSO32(uint8_t CS) {
      slaveSelect = CS; /* set slaveSelect to the specified SS pin */
      accel.addressSet(0x28, 0x2A, 0x2C, CS);
      gyro.addressSet(0x22, 0x24, 0x26, CS);
    };
    
    /* ---------- METHODS --------- */
    bool isAlive();      /* check if connected and responsive */
    void restart();      /* restart the device */
    bool setupAccel(float outputFrequency, uint8_t range);  /* set the accelerometer output update frequency (Hz) + measurement range (+/- range g) */
    bool setupGyro(float outputFrequency, uint16_t range);   /* set the gyroscope output update frequency (Hz) + measurement range (+/- range dps) */
    float readAccelZ();  /* read the acceleration in z-direction (m/s2) */
    float measureAccelNoiseZ(); /* measure the RMS noise in the z-direction of the accelerometer for a given number of readings */
    float readGyro(uint8_t axis);   
};


/* Example usage

  const int IMU_SS = 5;

  PDC_LSM6DSO32 IMU(IMU_SS); where the argument is the CS pin
  if(IMU.isAlive()){
	if flag is 1, we're all good
  }
  IMU.setAccelerometerMeasurementRange(4); (+/- in g)
  IMU.measureAccelerometerNoiseZ(); measure the standard deviation (m/s^2) of the noise in the z-direction
  accZ = IMU.readAccelerationZ(); get the acceleration in m/s2 in the z-direction

*/
