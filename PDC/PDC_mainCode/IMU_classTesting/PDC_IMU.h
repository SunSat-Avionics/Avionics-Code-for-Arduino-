float readAccelerationZ();

class PDC_IMU{
  public:
	const int slaveSelect;
    int measurementRange;
	
	/* constructor */
    PDC_IMU(const int CS){
		slaveSelect = CS;
	}; 
	/* check if connected and responsive */
	bool isAlive();
	/* read the acceleration in z-direction (m/s2) */
    float readAccelerationZ();
	/* set the measurement range (+/- range g) */
	void setMeasurementRange(int range);
	/* measure the RMS noise in the z-direction of the accelerometer for a given number of readings */
	float measureAccelerometerNoiseZ(int numReadings);
}

/* Example usage 

PDC_IMU IMU(10); where the argument is the CS pin
flag = IMU.isAlive(); if flag is 0, we're all good
IMU.setMeasurementRange(4); (+/- in g)
IMU.measureAccelerometerNoiseZ(10); measure the noise in the z direction for 10 samples


*/
