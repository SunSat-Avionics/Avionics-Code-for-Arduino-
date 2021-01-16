/***********************************************************************************************************************************************
 * SUNSAT Parachute Deployment and Attitude Determination Software
 *
 * Created 22 Nov 20
 * Contributors: Rory Haggart, Waleed Hamad
 *
 * Description
 * -----------
 * For the Sheffield University Nova SATellite (SUNSAT) platform, this code is to be onboard the 'parachute deployment computer' (PDC).
 * The PDC is an Arduino Nano serving the dual purpose of parachute deployment activities (e.g. apogee detection), and attitude determination.
 ***********************************************************************************************************************************************/

#include <SPI.h>                /* the IMU, barometer & micro-SD unit are on SPI. include library for SPI commands (https://www.arduino.cc/en/reference/SPI) */
#include "PDC_SPI.h"            /* then included our own SPI functions */
#include <Wire.h>               /* the RTC is on I2C, so include the library for I2C commands (https://www.arduino.cc/en/reference/wire) */
#include "PDC_I2C.h"            /* then include our own I2C functions */
#include "PDC_kalman.h"         /* include the functions for the kalman filter */
#include "PDC_LSM6DSO32.h"      /* include our IMU class */
#include "PDC_254.h"            /* include our micro-SD class */
#include <BasicLinearAlgebra.h> /* for Matrix operations
                                 *   if this line throws an error, you probably don't have the Matrix Library locally.
                                 *   see: https://github.com/tomstewart89/BasicLinearAlgebra or search 'basic linear algebra' in the IDE library manager 
                                 */
using namespace BLA;            /* use the basic linear algebra namespace */

// TODO consider the below library for faster read/write/mode if timings become an issue. requires us to know the pin at compile time, so won't help in the SPI
// might help in I2C where RTC is the only (currently) device  https://github.com/NicksonYap/digitalWriteFast

/* ---------- SPI CONFIG ---------- */
const uint8_t PDC_SS = 10;      /* the arduino nano has an 'SS' pin (10) which helps us choose if we want to be master or slave. pin 10 as output = PDC as master */
const uint8_t altimeter_SS = 4; /* the DIG pin connected to the altimer SS pin */
const uint8_t IMU_SS = 5;       /* the DIG pin connected to the IMU SS pin */
const uint8_t microSD_SS = 6;   /* the DIG pin connected to the micro-SD SS pin */

PDC_LSM6DSO32 IMU(IMU_SS);                /* create an LSM6DSO32 object for our IMU. class defines are in 'PDC_LSM6DSO32.h' and 'PDC_LSM6DSO32.cpp' */
uint8_t IMUChild::slaveSelect = IMU_SS;   /* ensure that our IMU children have access to the SS pin */

const uint8_t microSD_CD = 7;             /* the microSD card module has a chip detect pin which shorts to ground if the card isn't inserted */
PDC_254 microSD(microSD_SS, microSD_CD);  /* create a 254 breakout class for the microSD card module. class defines are in 'PDC_254.h' and 'PDC_254.cpp' */

/* ---------- I2C CONFIG ---------- */
const uint8_t RTC = 23;           /* the real-time clock (RTC) module is connected via I2C. the nano's data line for I2C (SDA) is at pin 23 */
const uint8_t RTCaddress = 0x50;  /* to communicate with an I2C device, we have to know the device address, and for the RTC it is 0x50 */

/* ---------- KALMAN FILTER CONFIG ---------- */
/* 
 *  we use a Kalman filter to estimate the point of apogee in flight
 *  the implementation is explained and described in 'kalmanFilter.pdf' in the github repo
 *  the state matrix is as below and we are measuring acceleration with the IMU, and displacement
 *   with the altimeter. Looking to get a good velocity estimate so point of zero crossing is apogee.
 *  | acceleration (z) |
 *  | velocity (z)     |
 *  | displacement (z) |
 */
 
// TODO: can some of these variables become a bit less global?
// TODO: refine kalmanTime based on tests. how long does measurement take? calculation time?
// TODO: then *consider* using this to set update frequency of sensors by rounding up (e.g. if kalman update freq is 10Hz, set accelerometer to 12.5Hz in setup)
const float kalmanTime = 0.5;       /* time step (s) between Kalman iterations */
const uint8_t numStates = 3;        /* number of states that we're interested in */
const uint8_t numMeasurements = 2;  /* number of measurements that we're taking */

/* state transition matrix which maps previous state to current state  */
Matrix<numStates, numStates> F_matrix = {1.0,                      0.0, 0.0,
                                         kalmanTime,               1.0, 0.0,
                                         0.5 * pow(kalmanTime, 2), 0.0, 1.0}; 

Matrix<numMeasurements, numStates> H_matrix;  /* measurement matrix which maps the measurements to the state variables */
Matrix<numStates, numMeasurements> K_matrix;  /* kalman gain matrix which weights our measurements against the underlying model */

/* P, Q and R matrices are only needed in setup, so they aren't needed globally */

Matrix<numStates, 1> stateMatrix;             /* matrix that contains the current system state */
Matrix<numStates, 1> previousStateMatrix;     /* matrix that contains the previous system state */
Matrix<numStates, 1> predictedStateMatrix;    /* matrix that contains the prediction of the next system state */
Matrix<numMeasurements, 1> measurementMatrix; /* matrix that contains the most recent measurements */

/* -------------------- ERRORS -------------------- */
uint8_t errCode = 0;  /* value to be used whenever we want to detect some error */

/* individual bit sets for an error at different components. will allow us to identify specific errors in an efficient way */
const uint8_t altErr = (1 << 0);  /* if the altimeter encounters an issue, set bit 0 */
const uint8_t imuErr = (1 << 1);  /* if the IMU encounters an issue, set bit 1 */
const uint8_t msdErr = (1 << 2);  /* if the micro-SD encounters an issue, set bit 2 */
const uint8_t logErr = (1 << 3);  /* if the log file encounters an issue, set bit 3 */
const uint8_t rtcErr = (1 << 4);  /* if the real-time clock encounters an issue, set bit 4 */
//const uint8_t TODO = (1 << 5);
//const uint8_t TODO = (1 << 6);
//const uint8_t TODO = (1 << 7);

bool apogee = 0;  /* flag that we can set when we think apogee has been reached */

/* -------------------- SETUP -------------------- */
void setup() {
  uint8_t altimeter_CHIP_ID[1]; /* for the return value of the altimeter 'CHIP_ID' identification register */
  bool errFlag = 0;             /* to flag errors when they're encountered */
  
  /* ---------- Serial Setup ---------- */
  /* open serial comms at 9600 baud to allow us to monitor the process
   *   serial will become irrelevant - once the code is on the PDC we shouldn't need it but it's useful for ground testing.
   * NOTE: keep serial comms sparse and short! they take up significant memory if too verbose or unique!! */
  Serial.begin(9600);
  while(!Serial){/* wait for port to connect */};
  Serial.println("\n-----\nSETUP\n-----\n");  /* inform start of setup */

  /* ---------- SPI Setup ---------- */
  pinMode(PDC_SS, OUTPUT);          /* we want to be the master of this bus! so set the 'SS' pin on the PDC as an output */
  digitalWrite(PDC_SS, HIGH);       /* now take it high */
  pinMode(altimeter_SS, OUTPUT);    /* set altimeter SS pin as output & disable communication by taking it high */
  digitalWrite(altimeter_SS, HIGH);
  pinMode(IMU_SS, OUTPUT);          /* set IMU SS pin as output & disable communication by taking it high */
  digitalWrite(IMU_SS, HIGH);
  pinMode(microSD_SS, OUTPUT);      /* set micro-SD SS pin as output & disable communication by taking it high */
  digitalWrite(microSD_SS, HIGH);

  SPI.begin();  /* initialise all lines and CPU to use SPI */

  /* ---------- PERIPHERAL SETUP ---------- */
  IMU.accel.addressSet(0x28, 0x10); /* tell the accelerometer where to find the start of it's data registers, and where to find it's control register */
  IMU.gyro.addressSet(0x22, 0x11);  /* tell the gyroscope where to find the start of it's data registers, and where to find it's control register */

  pinMode(microSD_CD, INPUT);       /* set the card detect pin to be an input that we can measure to check for a card */

  // TODO: light sensor pin configuration (digital output to SI pin, analogue input(s) from AO pins, clock signal to CLK pins)
  
  /* ---------- I2C Setup ---------- */
  Wire.begin(); /* initialise CPU to use I2C */

  /* ---------- SPI Verification ---------- */
  readSPI(altimeter_SS, 0x00, 1, altimeter_CHIP_ID);  /* communicate with altimeter: read the 'CHIP_ID' register. expect 0x50 */
  // TODO: maybe it'd be more consistent to create a BMP388 object like we have for the LSM6DSO32
  /* we have read the 'CHIP_ID' register and now should check that the value read is as we expect */
  if (altimeter_CHIP_ID[0] != 0x50) {
    errCode |= altErr;  /* if not alive, flag the altimeter error bit in our code */
  }

  /* our LSM6DSO32 class has an 'isAlive()' method, which reads the 'WHO_AM_I' register to check our connection. returns true if connected & working! */
  if (!IMU.isAlive()) {
    errCode |= imuErr;  /* if not alive, flag the IMU error bit in our code */
  }

  /* check the card detect pin for a card, and try to initialise it */
  if (!microSD.isAlive()) {
    errCode |= msdErr;  /* if not alive, flag the micro-SD error bit in our code */
  }

  /* attempt to open a .csv file which we want to log data to
     returns 0 if successful */
  if (microSD.openFile() != 0) {
    errCode |= logErr;  /* if there was some problem creating the file, flag the log file error bit in our code */
  }

  // TODO write a note to the microSD to signify SD begin - maybe need a .writeNote() method which blanks everything but date, time and note
  
  // setup interrupt pin? TBD - can we simply configure one of the GPIO to go high and connect this to main OBC interrupt, and then execute
  // the interrupt routine on OBC? or will we communicate with main OBC via I2C?

  /* ---------- PERIPHERAL CONFIGURATION ---------- */
  IMU.restart();        /* reboot & clear the IMU, giving it a bit of time to start back up */
  // TODO reboot other peripherals

  /************************************************************************
   *                        IMU CONFIG VALUES                             *
   *  --------------------------------------------------------------------*
   *  PARAM 1 (OUTPUT UPDATE FREQUENCY)  |   PARAM 2 (MEASUREMENT RANGE)  *
   *  0.  off                            |   0. 4g  / 250dps              *
   *  1.  12.5Hz                         |   1. --  / 125dps              *
   *  2.  26Hz                           |   2. 32g / 500dps              *
   *  3.  52Hz                           |   3. --  / --                  *
   *  4.  104Hz                          |   4. 8g  / 1000dps             *
   *  5.  208Hz                          |   5. --  / --                  *
   *  6.  416Hz                          |   6. 16g / 2000dps             *
   *  7.  833Hz                          |                                *
   *  8.  1660Hz                         |                                *
   *  9.  3330Hz                         |                                *
   *  10. 6660Hz                         |                                *
   ************************************************************************/
  // TODO: work out a sensible dps range
  IMU.accel.init(9, 2); /*  set the accelerometer output update frequency and measurement range */
  IMU.gyro.init(9, 0);  /*  set the gyroscope output update frequency and measurement range */
  
  // TODO: IMU self test

  /* ---------- KALMAN FILTER SETUP ---------- */
  /* fill all matrices with 0 to start */
  H_matrix.Fill(0.0);
  K_matrix.Fill(0.0);
  stateMatrix.Fill(0.0);
  previousStateMatrix.Fill(0.0);
  predictedStateMatrix.Fill(0.0);
  measurementMatrix.Fill(0.0);
  
  initKalman(); /* setup kalman filter for apogee detection (see kalmanFilter.ino) */
  
  /* ---------- SETUP COMPLETE ---------- */
  if (errCode != 0) {
    Serial.print("\n-\n");
    Serial.print(" Err: ");
    Serial.print(errCode, BIN);
    Serial.println(" \n-");
  }
  else {
    Serial.println("\n-\nSETUP :)\n-");
  }

  delay(2000);

  // TODO: take measurements in the ground state (e.g. temp and pressure). write them to SD with a note of 'ground conditions' or similar.
  // also worth storing them in variables to use to calculate local mach etc.

  // TODO write a note to the microSD to signify end of setup - maybe need a .writeNote() method which blanks everything but date, time and note
}

/* -------------------- LOOP -------------------- */
void loop() {
  // filler code to keep us entertained during testing
  float gyroY = IMU.gyro.readY();
  Serial.print("Y: ");
  Serial.println(gyroY, 5);

  // parachute deployment tasks
  // light sensor check (poll the sensor every x seconds to check ambient light levels. If new value much greater than old on all 4 sensors,
  // register apogee)

  /* tasks to perform before we have detected apogee */
  if(!apogee){
    // while(current time step - previous timestep < kalman time), do nothing (or something like check light sensor))
    // could also do lots of prediction steps before one update step?
    /* use the underlying dynamical model to predict the current state of the system */
    kalmanPredict();
    /* update the prediction by taking measurements */
    kalmanUpdate();
    // write measurements and states to micro-SD

    /* get the velocity from our state vector */
    float velocity = stateMatrix(1, 0);
    /* once the velocity is negative, we have crossed the point of zero-velocity in the z-direction and so apogee is reached  */
    // TODO maybe change the 0 to some threshold so that we start taking more frequent data below 10m/s for example
    if(velocity < 0){
      /* flag that apogee has been detected, and allow us to move on */
      apogee = 1;
      // write a note with data to inform apogee
    }
  }
 
  // TODO: write state vector to file


  // write any other data (e.g. raw readings, notes) to SD card (line format with timestamped measurements)


  // attitude determination tasks
  // quaternion conversion from gyro Euler output
  // kalman filter likely wont work for attitude det after all - too non-linear. some other filtering necessary
  // try coarse sun sensing to determine relative pose of sun
}
