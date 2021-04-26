#ifndef _LOGFILE /* include guard */
#define _LOGFILE

/* a structure containing the latest measurements to be written to the SD card or main OBC */
struct PDC_logFileFields{
  uint8_t logTime;    // TODO: ensure data types are correct (logTime definitely isnt!)
  uint8_t flightPhase;
  float accelerometerX;
  float accelerometerY;
  float accelerometerZ;
  float gyroscopeX;
  float gyroscopeY;
  float gyroscopeZ;
  float altimeterTemperature;
  float altimeterPressure;
  float altimeterAltitude;
  float light1;
  float light2;
  float light3;
  float light4;
  float estimateAccelerationZ;
  float estimateVelocityZ;
  float estimatePositionZ;
  uint8_t note;
};

extern struct PDC_logFileFields logFileLine;

#endif
