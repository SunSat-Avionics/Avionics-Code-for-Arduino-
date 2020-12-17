#include <Arduino.h>

/* we define an LSM6DSO32 class to keep everything packed away neatly. it allows us to keep hold of things that we set
    e.g. measurement range, so that we don't have to read them from the device directly, plus it gives some neat
    '.readAcceleration()' (or similar) functionality, which is nice and readable */
class PDC_LSM6DSO32 {
  private:
    /* the pin on the PDC that the IMU CS pin connects to. is set on contruction */
    uint8_t slaveSelect;
    /* the rate at which the accelerometer ODR updates */
    float accelerometerOutputFrequency;
    /* the accelerometer measurement range in g (can be +/-4, 8, 16, 32) */
    uint8_t accelerometerMeasurementRange;

  public:
    /* constructor - set slaveSelect to the specified SS pin */
    PDC_LSM6DSO32(uint8_t CS) {
      slaveSelect = CS;
    };
    /* check if connected and responsive */
    bool isAlive();
    /* read the acceleration in z-direction (m/s2) */
    float readAccelerationZ();
    /* set the measurement range (+/- range g) */
    bool setupAccelerometer(float outputFrequency, uint8_t range);
    /* measure the RMS noise in the z-direction of the accelerometer for a given number of readings */
    float measureAccelerometerNoiseZ(uint8_t numReadings);
};

/* Example usage

  const int IMU_SS = 5;

  PDC_LSM6DSO32 IMU(IMU_SS); where the argument is the CS pin
  if(IMU.isAlive()){
	if flag is 1, we're all good
  }
  IMU.setAccelerometerMeasurementRange(4); (+/- in g)
  IMU.measureAccelerometerNoiseZ(10); measure the noise in the z direction for 10 samples
  accZ = IMU.readAccelerationZ(); get the acceleration in m/s2 in the z-direction

*/
