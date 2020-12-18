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
/* the RTC is on I2C, so include the library for I2C commands (https://www.arduino.cc/en/reference/wire) */
#include <Wire.h>
/* include the functions for the kalman filter */
#include "PDC_kalman.h"
/* include our IMU class */
#include "PDC_LSM6DSO32.h"
/* include our micro-SD class */
#include "PDC_254.h"
/* for Matrix operations
   if this line throws an error, you probably don't have the Matrix Library locally.
   see: https://github.com/tomstewart89/BasicLinearAlgebra or search 'basic linear algebra' in the IDE library manager */
#include <BasicLinearAlgebra.h>
using namespace BLA;

// TODO consider the below library for faster read/write/mode if timings become an issue. requires us to know the pin at compile time, so won't help in the SPI
// might help in I2C where RTC is the only (currently) device  https://github.com/NicksonYap/digitalWriteFast 


/* ---------- SPI CONFIG ---------- */
/* the arduino nano has an 'SS' pin (10) which helps us choose if we want to be master or slave. pin 10 as output = PDC as master */
const uint8_t PDC_SS = 10;
/* define the DIG pins on the PDC that are connected to the 'slave select' (SS) pin of each device */
const uint8_t altimeter_SS = 4;
const uint8_t IMU_SS = 5;
const uint8_t microSD_SS = 6;
/* the microSD card module has a chip detect pin which shorts to ground if the card isn't inserted */
const uint8_t microSD_CD = 7;

/* create an LSM6DSO32 object for our IMU. class defines are in 'PDC_LSM6DSO32.h' and 'PDC_LSM6DSO32.cpp' */
PDC_LSM6DSO32 IMU(IMU_SS);
/* create a 254 breakout class for the microSD card module. class defines are in 'PDC_254.h' and 'PDC_254.cpp' */
PDC_254 microSD(microSD_SS, microSD_CD);

/* ---------- I2C CONFIG ---------- */
/* the real-time clock (RTC) module is connected via I2C. The nano's data line for I2C (SDA) is at pin 23 */
const uint8_t RTC = 23;

/* ---------- KALMAN FILTER CONFIG ---------- */
/* define number of states and measurements as this allows for more controlled matrix sizing
   NOTE: if changing states and measurements, make sure to change any relevant matrices in setup() or initKalman() */
const uint8_t numStates = 3;
const uint8_t numMeasurements = 2;
/* state transition matrix which maps previous state to current state.
   leave this as an empty variable for now as it's value changes per timestep */
Matrix<numStates, numStates> F_matrix;
/* measurement matrix which maps the measurements to the state variables */
Matrix<numMeasurements, numStates> H_matrix;
/* kalman gain matrix. dimensional analysis of the update equation gives us a 3x2 matrix so can declare here */
Matrix<numStates, numMeasurements> K_matrix;
/* error covariance matrix */
Matrix<numStates, numStates> P_matrix;
/* Q and R matrices are only needed in setup, so they aren't needed globally */

/* -------------------- SETUP -------------------- */
/* value to be used whenever we want to detect some error */
bool errCode = 0;
/* if we do encounter an error, set the flag to true so we can warn that setup failed */
bool errFlag = 0;

void setup() {
  /* to store the return value of the altimeter 'CHIP_ID' identification register */
  uint8_t altimeter_CHIP_ID;
  /* new instance of the 'File' class (part of the SD library) that we will use to control the .csv file on the microSD card */
  File dataLogFile;

  /* ---------- Serial Setup ---------- */
  /* open serial comms at 9600 baud to allow us to monitor the process
       serial may become irrelevant - once the code is on the PDC we might not be connecting via serial
       but it's useful for ground testing.
     NOTE: keep serial comms sparse and short! they take up significant memory if too verbose or unique!! */
  Serial.begin(9600);
  while (!Serial) {
    ; /* wait for port to connect */
  }
  Serial.println("\n-------\n SETUP\n-------\n");
  
  /* ---------- SPI Setup ---------- */
  /* we want to be the master of this bus! so set the 'SS' pin on the PDC as an output */
  pinMode(PDC_SS, OUTPUT);
  digitalWrite(PDC_SS, HIGH);

  /* set each slave select pin as an output.
       initialise each pin to be high (i.e. communication disabled)
       to communicate with a specific device, take its SS pin low with digitalWrite(device_SS, LOW);  */
  pinMode(altimeter_SS, OUTPUT);
  digitalWrite(altimeter_SS, HIGH);
  pinMode(IMU_SS, OUTPUT);
  digitalWrite(IMU_SS, HIGH);
  pinMode(microSD_SS, OUTPUT);
  digitalWrite(microSD_SS, HIGH);
  /* set the card detect pin to be an input that we can measure to check for a card */
  pinMode(microSD_CD, INPUT);

  /* initialise all lines and CPU to use SPI */
  SPI.begin();

  /* ---------- I2C Setup ---------- */
  // TODO

  /* ---------- SPI Verification ---------- */
  Serial.println("Altimeter?");
  /* communicate with altimeter: read the 'CHIP_ID' register. expect 0x50 */
  altimeter_CHIP_ID = readSPI(altimeter_SS, 0x00, 1);
  // TODO: maybe it'd be more consistent to create a BMP388 object like we have for the LSM6DSO32
  /* we have read the 'CHIP_ID' register and now should check that the value read is as we expect */
  if (altimeter_CHIP_ID == 0x50) {
    Serial.println("  :)");
  }
  else {
    Serial.println("  :(");
    errFlag = 1;
  }

  Serial.println("IMU?");
  /* our IMU class has an 'isAlive()' method, which reads the 'WHO_AM_I' register to check our connection. returns true if connected & working! */
  if (IMU.isAlive()) {
    Serial.println("  :)");
  }
  else {
    Serial.println("  :(");
    errFlag = 1;
  }

  Serial.println("micro-SD?");
  /* attempt to init micro SD card */
  // TODO: if we have a shield with 'CD' (chip detect) pin, make use of this to check pin is in place.
  if (microSD.isAlive()) {
    Serial.println("  :)");
  }
  else {
    Serial.println("  :(");
    errFlag = 1;
  }

  Serial.println("log file?");
  /* attempt to open a .csv file which we want to log data to */
  // TODO: once RTC is up & running, name the file with timestamp as per ISO 8601 format (kind of..)(yyyy-mm-ddThh-mm-ss.csv)
  dataLogFile = SD.open("temp.csv", FILE_WRITE);

  /* if the file can't open, the return is false */
  if (!dataLogFile) {
    Serial.println("  :(");
    errFlag = 1;
  }
  else {
    Serial.println("  :)");
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

  /* ---------- SENSOR SETUP ---------- */
  /* accelerometer setup: 1. output update freq in Hz   (0, 12.5, 26, 52, 104, 208, 416, 833, 1660, 3330, 6660)
                          2. measurement range in +/- g (4, 8, 16, 32)
  */
  /* set the accelerometer update rate high enough to allow us to capture lots of data, and 32g mode as launch will be quite tough.*/
  errCode = IMU.setupAccelerometer(3330, 32);
  if (errCode != 0) {
    /* the function returns TRUE if the setup succeeded
        don't need to print anything to serial for this error as it's taken care of in the class */
    errFlag = 1;
    errCode = 0;
  }

  // TODO: IMU self test

  /* ---------- KALMAN FILTER SETUP ---------- */
  /* fill all matrices with 0 to start */
  F_matrix.Fill(0.0);
  H_matrix.Fill(0.0);
  K_matrix.Fill(0.0);
  P_matrix.Fill(0.0);
  /* setup kalman filter for apogee detection (function in kalmanFilter.ino) */
  initKalman();

  /* ---------- SETUP COMPLETE ---------- */
  if (errFlag) {
    Serial.println("\n----------\n SETUP :(\n----------");
  }
  else {
    Serial.println("\n----------\n SETUP :)\n----------");
  }
}

/* -------------------- LOOP -------------------- */

void loop() {
  // filler code to keep us entertained during testing
  float accelZ = IMU.readAccelerationZ();
  Serial.print("Acceleration: ");
  Serial.println(accelZ, 5);
  
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
