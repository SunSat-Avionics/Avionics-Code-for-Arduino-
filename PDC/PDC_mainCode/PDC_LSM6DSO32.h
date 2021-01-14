#include <Arduino.h>  /* bring some arduino syntax into the cpp files */
#include "PDC_SPI.h"  /* grab our SPI functions */
#include <stdio.h>    /* std stuff for cpp */

const float GRAVITY_MAGNITUDE = 9.80665; /* set the magnitude of the gravity vector */

/* LSM6DSO32 CLASS
 *  we define an LSM6DSO32 class to keep everything packed away neatly. 
 *  it allows us to keep hold of things that we set e.g. measurement range, so that we don't have to read them from the device directly
 *  also gives us fine control over functionality
 *  adafruit have a library for this component already but this is more readable, flexible, and lightweight
 */
class PDC_LSM6DSO32 {
  private:
    /* ---------- ATTRIBUTES ---------- */
    uint8_t slaveSelect;                    /* the pin on the PDC that the IMU CS pin connects to. is set on contruction */
    float accelerometerOutputFrequency;     /* the rate at which the accelerometer ODR updates */
    uint8_t accelerometerMeasurementRange;  /* the accelerometer measurement range in g (can be +/-4, 8, 16, 32) */
    float accelResolution;                  /* the resolution of the accelerometer in milli-g per bit */

  public:
    /* ---------- CONSTRUCTOR ---------- */
    PDC_LSM6DSO32(uint8_t CS) {
      slaveSelect = CS; /* set slaveSelect to the specified SS pin */
    };

    /* ---------- METHODS --------- */
    bool isAlive();             /* check if connected and responsive */
    void restart();             /* restart the device */
    bool setupAccelerometer(float outputFrequency, uint8_t range);  /* set the measurement range (+/- range g) */
    float readAccelerationZ();  /* read the acceleration in z-direction (m/s2) */
    float measureAccelerometerNoiseZ(); /* measure the RMS noise in the z-direction of the accelerometer for a given number of readings */
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
