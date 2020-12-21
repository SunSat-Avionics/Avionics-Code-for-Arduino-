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
/* then include our own I2C functions */
#include "PDC_I2C.h"
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
/* to communicate with an I2C device, we have to know the device address, and for the RTC it is 0x50 */
const uint8_t RTCaddress = 0x50;

/* ---------- KALMAN FILTER CONFIG ---------- */
/* time step between Kalman iterations in seconds */
// TODO: refine this based on tests. how long does measurement take? calculation time?
const float kalmanTime = 0.5;
/* define number of states and measurements as this allows for more controlled matrix sizing
   NOTE: if changing states and measurements, make sure to change any relevant matrices in setup() or initKalman() */
const uint8_t numStates = 3;
const uint8_t numMeasurements = 2;
/* state transition matrix which maps previous state to current state */
Matrix<numStates, numStates> F_matrix = {1.0,                   0.0, 0.0,
                                         kalmanTime,            1.0, 0.0,
                                         0.5*pow(kalmanTime,2), 0.0, 1.0};
/* measurement matrix which maps the measurements to the state variables */
Matrix<numMeasurements, numStates> H_matrix;
/* kalman gain matrix. dimensional analysis of the update equation gives us a 3x2 matrix so can declare here */
Matrix<numStates, numMeasurements> K_matrix;
/* P, Q and R matrices are only needed in setup, so they aren't needed globally */

/* matrix that contains the current system state */
Matrix<numStates, 1> stateMatrix;
/* matrix that contains the previous system state */
Matrix<numStates, 1> previousStateMatrix;
/* matrix that contains the prediction of the next system state */
Matrix<numStates, 1> predictedStateMatrix;
/* matrix that contains the most recent measurements */
Matrix<numMeasurements, 1> measurementMatrix;

// TODO: define our kalman frequency (e.g. const int kalmanFrequency = 10; (Hz)) and use this to define the F matrix
  // the frequency will probably involve some experimenting to work out how high we can reasonably go based on calc times. maybe worth
  // then using this to set update frequency of sensors by rounding up (e.g. if kalman update freq is 10Hz, set accelerometer
  // to 12.5Hz in setup)

/* -------------------- ERRORS -------------------- */
/* value to be used whenever we want to detect some error */
bool errCode = 0;
/* individual bit sets for an error at different components. will allow us to identify specific errors in an efficient way */
const uint8_t altErr = (1 << 0);
const uint8_t imuErr = (1 << 1);
const uint8_t msdErr = (1 << 2);
const uint8_t logErr = (1 << 3);
const uint8_t rtcErr = (1 << 4);

/* -------------------- SETUP -------------------- */
void setup() {
  /* to store the return value of the altimeter 'CHIP_ID' identification register */
  uint8_t altimeter_CHIP_ID[1];

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
  /* initialise CPU to use I2C */
  Wire.begin();

  /* ---------- SPI Verification ---------- */
  /* communicate with altimeter: read the 'CHIP_ID' register. expect 0x50 */
  readSPI(altimeter_SS, 0x00, 1, altimeter_CHIP_ID);
  // TODO: maybe it'd be more consistent to create a BMP388 object like we have for the LSM6DSO32
  /* we have read the 'CHIP_ID' register and now should check that the value read is as we expect */
  if (altimeter_CHIP_ID[0] != 0x50) {
    errCode |= altErr;
  }

  /* our IMU class has an 'isAlive()' method, which reads the 'WHO_AM_I' register to check our connection. returns true if connected & working! */
  if (!IMU.isAlive()) {
    errCode |= imuErr;
  }

  /* attempt to init micro SD card */
  // TODO: if we have a shield with 'CD' (chip detect) pin, make use of this to check pin is in place.
  if (!microSD.isAlive()) {
    errCode |= msdErr;
  }

  /* attempt to open a .csv file which we want to log data to
     returns 0 if successful */
  if (microSD.openFile() != 0) {
    errCode |= logErr;
  }

  // TODO write a note to the microSD to signify SD begin - maybe need a .writeNote() method which blanks everything but date, time and note
  // TODO 
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
    /* the function returns TRUE if the setup succeeded. error if values are invalid */
    errCode |= imuErr;
  }

  // TODO: IMU self test

  /* ---------- KALMAN FILTER SETUP ---------- */
  /* fill all matrices with 0 to start */
  H_matrix.Fill(0.0);
  K_matrix.Fill(0.0);
  stateMatrix.Fill(0.0);
  previousStateMatrix.Fill(0.0);
  predictedStateMatrix.Fill(0.0);
  measurementMatrix.Fill(0.0);
  
  /* setup kalman filter for apogee detection (function in kalmanFilter.ino) */
  initKalman();

  /* ---------- SETUP COMPLETE ---------- */
  if (errCode) {
    Serial.println("\n----------\n");
    Serial.print("Error Code: ");
    Serial.println(errCode, BIN);
    Serial.println("\n----------");
  }
  else {
    Serial.println("\n----------\n SETUP :)\n----------");
  }
  // TODO write a note to the microSD to signify end of setup - maybe need a .writeNote() method which blanks everything but date, time and note
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

  
  /* use the underlying dynamical model to predict the current state of the system */
  kalmanPredict();
  /* update the prediction by taking measurements */
  kalmanUpdate();

  // TODO: implement timing to fit with gain calculation. e.g. while(current time step - previous timestep < kalman time), do nothing (or something...))
  // TODO: write state vector to file


  // write any other data (e.g. raw readings, notes) to SD card (line format with timestamped measurements)


  // attitude determination tasks
  // quaternion conversion from gyro Euler output then update kalman filter with gyro readings
  // try coarse sun sensing to determine relative pose of sun
}
