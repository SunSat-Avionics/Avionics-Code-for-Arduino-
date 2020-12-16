float readAccelerationZ();

class PDC_IMU{
	private:
		const int slaveSelect;
		int measurementRange;
	
	public:
		/* constructor */
		void PDC_IMU(const int CS){
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

const int IMU_SS = 5;

PDC_IMU IMU(IMU_SS); where the argument is the CS pin
if(IMU.isAlive()){
	if flag is 1, we're all good
}
IMU.setMeasurementRange(4); (+/- in g)
measurementRange = IMU.getMeasurementRange(); get the range of measurements
IMU.measureAccelerometerNoiseZ(10); measure the noise in the z direction for 10 samples
accZ = IMU.readAccelerationZ(); get the acceleration in m/s2 in the z-direction

*/
