float readAccelerationZ();

class PDC_IMU{
  public:
	const int slaveSelect;
    int measurementRange;
	
    PDC_IMU();
	bool isAlive();
    float readAccelerationZ();
	void setMeasurementRange(int range);
	float measureAccelerometerNoiseZ(int numReadings);
}

/* Example usage 

PDC_IMU IMU(10); where the argument is the CS pin
flag = IMU.isAlive(); if flag is 0, we're all good
IMU.setMeasurementRange(4); (+/- in g)
IMU.measureAccelerometerNoiseZ(10); measure the noise in the z direction for 10 samples


*/
