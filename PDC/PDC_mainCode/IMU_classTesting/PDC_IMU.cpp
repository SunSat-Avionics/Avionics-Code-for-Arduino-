// TODO: might be worth creating an IMU class with methods to read the different registers, and so we can keep track of configurations (e.g. measurement range)
// Methods:
  // read values (gyro xyz, accel xyz, temp?, general reg e.g. whoami?
  // write values (measurement range)
  // measure noise (loop for fixed time and calculate RMS noise)
  // measure offset

#include "PDC_SPI.h"

PDC_IMU::PDC_IMU(const int CS){
	self.slaveSelect = CS;
}

bool PDC_IMU::isAlive(){
	// TODO: implement the "whoami" checker
	// return 0 if success, 1 otherwise
}

/* read the z-axis acceleration and convert to an acceleration in m/s2 */
float PDC_IMU::readAccelerationZ(){
  /* the resolution of the accelerometer is its range (e.g. +/4g) divided by the number of combinations available
     the accelerometer output is 16bits, so 2^16 is our combinations. 
      units are milli-g per bit, so times 1000*/
  float accelResolution = (self.measurementRange*2 / 65536) * 1000;

  /* for the output from the accelerometer */
  int rawAccelZ;
  /* for the actual acceleration in g */
  float acceleration = 0;

  /* read z-axis acceleration.
       note in CTRL3_C, there is a default enabled bit which auto-increments the register address when reading multiple bytes so we dont need to read the H and L
       registers separately ! */
  rawAccelZ = readSPI(IMU_SS, 0x2C, 2);

  /* convert our output into an actual acceleration value in ms/2 */
  acceleration = (rawAccelZ * GRAVITY_MAGNITUDE * accelResolution) / 1000;

  return (acceleration);
}

void PDC_IMU::setMeasurementRange(int range){
	self.measurementRange = range;
	// TODO: actually write to controller (requires an SPI write function!)
}

void PDC_IMU::measureAccelerometerNoiseZ(int numReadings){
	for(int i = 0; i < 5; i++){
		// TODO: read z axis noise
		readAccelerationZ;
	}
}