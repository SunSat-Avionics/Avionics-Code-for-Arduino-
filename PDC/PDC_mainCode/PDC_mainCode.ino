/* SUNSAT Parachute Deployment and Attitude Determination Software

   Created 22 Nov 20
   Contributors: Rory Haggart, Waleed Hamad

   Description
   -----------
   For the Sheffield University Nova SATellite (SUNSAT) platform, this code is to be onboard the 'parachute deployment computer' (PDC).
   The PDC is an Arduino Nano serving the dual purpose of parachute deployment activities (e.g. apogee detection), and attitude determination.
*/

/* the accel/gyro, barometer & micro-SD unit are on SPI, so include library for SPI commands (https://www.arduino.cc/en/reference/SPI) */
#include <SPI.h>
/* then included our own SPI functions */
#include "PDC_SPI.h"
/* we want the SD card library too (https://www.arduino.cc/en/reference/SD) */
#include <SD.h>
/* include the functions for the kalman filter */
#include "PDC_kalman.h"
/* for Matrix operations  
   if this line throws an error, you probably don't have the Matrix Library locally. 
   see: https://github.com/TheForeignMan/ArduinoMatrixLibrary */
#include <MatrixLibrary.h>

/* ---------- SPI CONFIG ---------- */
/*
   create an SPISettngs object to define the characteristics of the bus
   the three parameters are: 1. clock input frequency, 2. MSB/LSB first, and 3. SPI mode
      for more information, see: https://www.arduino.cc/en/reference/SPI
   1. altimeter & gyro/accel have max clock input freq. of 10MHz, micro-sd has 25MHz
      to avoid reconfigs, we'll stick at 10MHz for now - see if this is fast enough for SD
   2. all devices are MSB first
   3. all devices are compatible with mode 00 (clock idle low, output: falling edge, capture: rising edge);
*/

/* this is then our object with settings for our transactions */
SPISettings SPIParams(10000000, MSBFIRST, SPI_MODE0);

/* the arduino nano has an 'SS' pin (10) which helps us choose if we want to be master or slave. pin 10 as output = PDC as master */
const int PDC_SS = 10;
/* define the DIGIN pins on the PDC that are connected to the 'slave select' (SS) pin of each device */
const int altimeter_SS = 4;
const int IMU_SS = 5;
const int microSD_SS = 6;

/* ---------- I2C CONFIG ---------- */
// TODO

/* ---------- KALMAN FILTER CONFIG ---------- */
/* define number of states and measurements as this allows for more dynamic matrix sizing 
   NOTE: if changing states and measurements, make sure to change any relevant matrices in setup() */
const int numStates = 3;
const int numMeasurements = 2;
/* state transition matrix which maps previous state to current state.
   leave this as an empty variable for now as it's value changes per timestep */
Matrix F_matrix(numStates, numStates);
/* measurement matrix which maps the measurements to the state variables */
Matrix H_matrix(numMeasurements, numStates);
/* kalman gain matrix. dimensional analysis of the update equation gives us a 3x2 matrix so can declare here */
Matrix K_matrix(numStates, numMeasurements);
/* measurement noise covariance matrix */
Matrix R_matrix(numStates, numStates);
/* process noise covariance matrix */
Matrix Q_matrix(numStates, numStates);
/* error covariance matrix */
Matrix P_matrix(numStates, numStates);

/* various device configurations to setup communications and verify that things are working and ready to go */
void setup() {
  /* to store the return value of the accelerometer 'WHO_AM_I' identification register */
  unsigned int IMU_WHO_AM_I;
  /* to store the return value of the altimeter 'CHIP_ID' identification register */
  unsigned int altimeter_CHIP_ID;
  /* new instance of the 'File' class (part of the SD library) that we will use to control the .csv file on the microSD card */
  File dataLogFile;

  /* ---------- Serial Setup ---------- */
  /* open serial comms at 9600 baud to allow us to monitor the process
       serial may become irrelevant - once the code is on the PDC we might not be connecting via serial
       but it's useful for ground testing */
  Serial.begin(9600);
  while (!Serial) {
    ; /* wait for port to connect */
  }

  /* ---------- SPI Setup ---------- */
  /* we want to be the master of this bus! so set the 'SS' pin on the PDC as an output */
  pinMode(PDC_SS, OUTPUT);

  /* set each slave select pin as an output.
       initialise each pin to be high (i.e. communication disabled)
       to communicate with a specific device, take its SS pin low with digitalWrite(device_SS, LOW); */
  pinMode(altimeter_SS, OUTPUT);
  digitalWrite(altimeter_SS, HIGH);
  pinMode(IMU_SS, OUTPUT);
  digitalWrite(IMU_SS, HIGH);
  pinMode(microSD_SS, OUTPUT);
  digitalWrite(microSD_SS, HIGH);

  /* initialise all lines and CPU to use SPI */
  SPI.begin();

  /* ---------- I2C Setup ---------- */
  // TODO

  /* ---------- SPI Verification ---------- */

  /* communicate with altimeter: read the 'CHIP_ID' register. expect 0x50 */
  altimeter_CHIP_ID = readSPI(altimeter_SS, 0x00, 1);
  /* we have read the 'CHIP_ID' register and now should check that the value read is as we expect */
  // TODO: when a little more developed, this should be replaced with something more meaningful! e.g. comms to ground to indicate 'yes we're good on the altimeter'
  if (altimeter_CHIP_ID == 0x50) {
    Serial.println("Altimeter successfully connected!");
  }
  else {
    Serial.println("Altimeter could not be reached!");
  }

  /* communicate with IMU: read the 'WHO_AM_I' register. expect 0110110 */
  IMU_WHO_AM_I = readSPI(IMU_SS, 0x0f, 1);
  /* we have read the 'WHO_AM_I' register and now should check that the value read is as we expect */
  // TODO: when a little more developed, this should be replaced with something more meaningful!
  if (IMU_WHO_AM_I == 0110110) {
    Serial.println("IMU successfully connected!");
  }
  else {
    Serial.println("IMU could not be reached!");
  }

  /* attempt to init micro SD card */
  // TODO: if we have a shield with 'CD' (chip detect) pin, make use of this to check pin is in place.
  if (SD.begin(microSD_SS)) {
    Serial.println("micro-SD card initialised");
  }
  else {
    Serial.println("micro-SD initialisation failed!");
  }

  /* attempt to open a .csv file which we want to log data to */
  // TODO: once RTC is up & running, name the file with timestamp as per ISO 8601 format (kind of..)(yyyy-mm-ddThh-mm-ss.csv)
  dataLogFile = SD.open("temp.csv", FILE_WRITE);

  /* if the file can't open, the return is false */
  if (!dataLogFile) {
    Serial.println("Data log file could not be opened!");
  }

  /* communicate with micro SD - write the csv headers to our file
       should define (either here or elsewhere) the units of each of these headers... perhaps in readme */
  // TODO: populate this more fully - what are the raw measurements from BMP, GYRO, light sensors, etc?
  dataLogFile.print("Date, \
                     Time, \
                     acc_x, \
                     acc_y, \
                     acc_z, \
                     Note");


  /* ---------- ---------- */
  // light sensor pin configuration (digital output to SI pin, analogue input(s) from AO pins, clock signal to CLK pins)

  // setup interrupt pin? TBD - can we simply configure one of the GPIO to go high and connect this to OBC interrupt, and then execute
  // the interrupt routine on OBC? or will we communicate with main OBC via I2C?

  // take measurements in the ground state (e.g. temp and pressure). write them to SD with a note of 'ground conditions' or similar.
  // also worth storing them in variables to use to calculate local mach etc.

  // indicate that setup is complete - write to SD 'setup complete' and maybe talk to main OBC to tell ground that we're ready to go

  /* ---------- KALMAN SETUP ---------- */
  /* set measurment matrix non-zero values */
  H_matrix.SetValueAt(0, 0, 1);
  H_matrix.SetValueAt(2, 2, 1);
  /* setup kalman filter for apogee detection (function in kalmanFilter.ino) */
  initKalman();
}

void loop() {
  // parachute deployment tasks
  // light sensor check (poll the sensor every x seconds to check ambient light levels. If new value much greater than old on all 4 sensors,
  // register apogee)

  // altimeter check (check stream of readings and determine when the pressure begins to increase again). potentially feed this into the
  // Kalman Filter to mitigate noise and back up the accel readings)

  // accelerometer check (update Kalman Filter of integral of acceleration to detect when the velocity is zero)

  // write data to SD card (line format with timestamped measurements)


  // attitude determination tasks
  // quaternion conversion from gyro Euler output then update kalman filter with gyro readings
  // try coarse sun sensing to determine relative pose of sun
}