/***********************************************************************************************************************************************
   SUNSAT Parachute Deployment and Attitude Determination Software

   Created 22 Nov 20
   Contributors: Rory Haggart

   Description
   -----------
   For the Sheffield University Nova SATellite (SUNSAT) platform, this code is to be onboard the 'parachute deployment computer' (PDC).
   The PDC is an Arduino Nano serving the dual purpose of parachute deployment activities (e.g. apogee detection), and attitude determination.
 ***********************************************************************************************************************************************/

#include "headers.h"            /* contains a few specific parameter and function definitions */
#include <SPI.h>                /* the IMU, barometer & micro-SD unit are on SPI. include library for SPI commands (https://www.arduino.cc/en/reference/SPI) */
#include "PDC_SPI.h"            /* then included our own SPI functions */
#include <Wire.h>               /* the RTC is on I2C, so include the library for I2C commands (https://www.arduino.cc/en/reference/wire) */
#include "PDC_I2C.h"            /* then include our own I2C functions */
#include "PDC_kalman.h"         /* include the functions for the kalman filter */
#include "PDC_LSM6DSO32.h"      /* include our IMU class */
#include "PDC_BMP388.h"         /* include our altimeter class */
#include "PDC_254.h"            /* include our micro-SD class */
#include "PDC_logFile.h"        /* include our log file storage struct */
#include <BasicLinearAlgebra.h> /* for Matrix operations
                                     if this line throws an error, you probably don't have the Matrix Library locally.
                                     see: https://github.com/tomstewart89/BasicLinearAlgebra or search 'basic linear algebra' in the IDE library manager 
*/
using namespace BLA;            /* use the basic linear algebra namespace */

/* ---------- GENERAL PARAMETERS ---------- */
const uint8_t LAUNCH_SITE_ALTITUDE = 142; /* [m] how high above sea level is the launch site? used for measuring noise in altimeter */
const uint8_t ACC_LIFTOFF_THRESHOLD = 5;  /* [g] the threshold value that tells us we have liftoff. this triggers the move from 'wait' mode to 'flight' mode */

// TODO: refine kalmanTime based on tests. how long does measurement take? calculation time?
// TODO: then *consider* using this to set update frequency of sensors by rounding up (e.g. if kalman update freq is 10Hz, set accelerometer to 12.5Hz in setup)
const float kalmanTime = 0.5;             /* time step (s) between Kalman iterations */

/* ---------- SPI CONFIG ---------- */
const uint8_t PDC_SS = 10;      /* the arduino nano has an 'SS' pin (10) which helps us choose if we want to be master or slave. pin 10 as output = PDC as master */
const uint8_t altimeter_SS = 4; /* the DIG pin connected to the altimer SS pin */
const uint8_t IMU_SS = 5;       /* the DIG pin connected to the IMU SS pin */
const uint8_t microSD_SS = 6;   /* the DIG pin connected to the micro-SD SS pin */

PDC_LSM6DSO32 IMU(IMU_SS);                /* create an LSM6DSO32 object for our IMU. class defines are in 'PDC_LSM6DSO32.h' and 'PDC_LSM6DSO32.cpp' */
PDC_BMP388 altimeter(altimeter_SS);       /* create a BMP388 object for our altimeter. class defines are in 'PDC_BMP388.h' and 'PDC_BMP388.cpp' */

const uint8_t microSD_CD = 7;             /* the microSD card module has a chip detect pin which shorts to ground if the card isn't inserted */
PDC_254 microSD(microSD_SS, microSD_CD);  /* create a 254 breakout class for the microSD card module. class defines are in 'PDC_254.h' & 'PDC_254.cpp' */

/* ---------- I2C CONFIG ---------- */
const uint8_t RTC = 23;           /* the real-time clock (RTC) module is connected via I2C. the nano's data line for I2C (SDA) is at pin 23 */
const uint8_t RTCaddress = 0x50;  /* to communicate with an I2C device, we have to know the device address, and for the RTC it is 0x50 */
// TODO: config for comms with main OBC

/* ---------- LIGHT SENSOR CONFIG ---------- */
// TODO: populate with light sensor stuff

/* ---------- LOG FILE CONFIG ---------- */    
PDC_logFileFields logFileLine = {}; /* create a new instance of the storage for our log file fields and initialise all fields to 0 */

/* ---------- KALMAN FILTER CONFIG ---------- */
/*
    we use a Kalman filter to estimate the point of apogee in flight
    the implementation is explained and described in 'kalmanFilter.pdf' in the github repo
    the state matrix is as below and we are measuring acceleration with the IMU, and displacement
     with the altimeter. Looking to get a good velocity estimate so point of zero crossing is apogee.
    | acceleration (z) |
    | velocity (z)     |
    | displacement (z) |
*/

const uint8_t numStates = 3;        /* number of states that we're interested in */
const uint8_t numMeasurements = 2;  /* number of measurements that we're taking */

/* P, Q and R matrices are only needed in setup, so they aren't needed globally */
Matrix<numStates, 1> stateMatrix;             /* matrix that contains the current system state */
Matrix<numStates, 1> previousStateMatrix;     /* matrix that contains the previous system state */
Matrix<numStates, 1> predictedStateMatrix;    /* matrix that contains the prediction of the next system state */
Matrix<numMeasurements, 1> measurementMatrix; /* matrix that contains the most recent measurements */

/* -------------------- ERRORS -------------------- */
uint8_t errCode = 0;  /* to store component errors in setup */

/* individual bit sets for an error at different components. will allow us to identify specific errors in an efficient way */
// TODO: probably turn this into a 16 bit so we have better understanding of what actually caused the issue (isAlive, selfTest, etc)
// TODO: once we better understand PDC <-> main OBC comms, work out if we can send this code to main OBC then route it to ground station for info
// TODO: maybe reserve a couple of bits to identify this message as the 'PDC startup' so ground station knows how to interpret it? or get OBC to
  // append some identifying header tag
const uint8_t altErr = (1 << 0);  /* altimeter issue, set bit 0 */
const uint8_t imuErr = (1 << 1);  /* IMU encounters issue, set bit 1 */
const uint8_t msdErr = (1 << 2);  /* micro-SD issue, set bit 2 */
const uint8_t logErr = (1 << 3);  /* log file issue, set bit 3 */
const uint8_t rtcErr = (1 << 4);  /* real-time clock issue, set bit 4 */
const uint8_t obcErr = (1 << 5);  /* main obc issue, set bit 5 */
const uint8_t alsErr = (1 << 6);  /* analog light sensor issue, set bit 6 */
//const uint8_t TODO = (1 << 7);

/* ---------- SUBROUTINE CONTROL ---------- */
const uint8_t WAIT_FOR_LAUNCH = 0;  /* on the pad, waiting for launch */
const uint8_t LAUNCH = 1;           /* in the ascent phase, waiting for apogee */
const uint8_t APOGEE = 2;           /* apogee reached, waiting for deployment */
const uint8_t DESCENT = 3;          /* deployed and descending */
const uint8_t LANDING = 4;          /* back on Earth */

uint8_t subRoutine = WAIT_FOR_LAUNCH; /* initialise our current phase as waiting for launch */

/* -------------------- SETUP -------------------- */
void setup() {
  bool errFlag = 0;             /* to flag errors when they're encountered */

  /* ------------- Serial Setup --------------
     this will become irrelevant. once code
     is on the PDC we shouldn't need it but
     it's useful for ground testing.
     NOTE: keep serial comms sparse and short!
     they take up significant memory if too
     verbose or unique!!
    ------------------------------------------*/
  Serial.begin(9600);                 /* open serial comms at 9600 baud to allow us to monitor the process */
  while (!Serial) {};                 /* wait for port to connect */
  Serial.println("\n-\nSETUP\n-\n");  /* inform start of setup */

  /* ---------- SPI Setup ---------- */
  pinMode(PDC_SS, OUTPUT);          /* we want to be the master of this bus! so set the 'SS' pin on the PDC as a HIGH output (https://www.arduino.cc/en/reference/SPI) */
  digitalWrite(PDC_SS, HIGH);
  pinMode(altimeter_SS, OUTPUT);    /* set altimeter SS pin as output & disable communication by taking it high */
  digitalWrite(altimeter_SS, HIGH);
  pinMode(IMU_SS, OUTPUT);          /* set IMU SS pin as output & disable communication by taking it high */
  digitalWrite(IMU_SS, HIGH);
  pinMode(microSD_SS, OUTPUT);      /* set micro-SD SS pin as output & disable communication by taking it high */
  digitalWrite(microSD_SS, HIGH);

  SPI.begin();  /* initialise all lines and CPU to use SPI */

  /* ---------- Peripheral Setup ---------- */
  pinMode(microSD_CD, INPUT);       /* set the card detect pin to be an input that we can measure to check for a card */
  // TODO: configure light sensor pins

  /* ---------- I2C Setup ---------- */
  Wire.begin(); /* initialise CPU to use I2C */
  // TODO: setup RTC and OBC on I2C

  /* ---------- SPI Verification ---------- */
  /* our LSM6DSO32 class has an 'isAlive()' method, which reads the 'WHO_AM_I' register to check our connection. returns true if connected & working! */
  if (!IMU.isAlive()) {
    errCode |= imuErr;  /* if not alive, flag the IMU error bit in our code */
  }
  
  /* our BMP388 class has an 'isAlive()' method, which reads the 'CHIP_ID' register to check our connection. returns true if connected & working! */
  if (!altimeter.isAlive()) {
    errCode |= altErr;
  }

  /* our 254 class has an 'isAlive()' method, which checks the card detect pin for a card, and tries to initialise it */
  if (!microSD.isAlive()) {
    errCode |= msdErr;
  }

  // TODO write a note (status code) to the microSD to signify SD begin - maybe need a .writeNote() method which blanks everything but time and note

  // TODO light sensor checks: do they agree, is it dark?

  /* ---------- PERIPHERAL CONFIGURATION ---------- */
  IMU.restart();        /* reboot & clear the IMU, giving it a bit of time to start back up */
  altimeter.restart();  /* soft reset the altimeter and enable the pressure and temperature measurements */
  
  // TODO: can we reboot microsd? 

  // TODO: decide if self test is actually sensible... what if vehicle isn't perfectly still?
  errFlag = IMU.selfTest(); /* run self-test routine on IMU to check accel & gyro are working */
  if (errFlag) {
    errCode |= imuErr;  /* if self test failed, IMU error */
    errFlag = 0;        /* clear error flag */
  }
  
  // TODO: some sort of altimeter testing - if we know where we're launching we can estimate expected pressure (or we measure at alt=0 and work from there)

  // TODO: pass the datetime string into this function to name the file in a useful way
  /* attempt to open a .csv file which we want to log data to */
  if (microSD.openFile() != 0) {
    errCode |= logErr;  /* if there was some problem creating the file, flag the log file error bit in our code */
  }
  
  /**********************************************************************
                            IMU CONFIG VALUES
      (aliases for each value are defined in PDC_LSM6DSO32.h)
      ------------------------------------------------------------------
      PARAM 1 (OUTPUT UPDATE FREQUENCY) |   PARAM 2 (MEASUREMENT RANGE)
      +  off                            |   + 4g  / 250dps
      +  12.5Hz                         |   + --  / 125dps
      +  26Hz                           |   + 32g / 500dps
      +  52Hz                           |   + --  / --
      +  104Hz                          |   + 8g  / 1000dps
      +  208Hz                          |   + --  / --
      +  416Hz                          |   + 16g / 2000dps
      +  833Hz                          |
      +  1660Hz                         |
      +  3330Hz                         |
      +  6660Hz                         |
   **********************************************************************/
  // TODO: work out a sensible gyro range for this operation
  IMU.accel.init(ACC_ODR_3330, ACC_RNG_32); /* set the accelerometer output update frequency and measurement range */
  IMU.gyro.init(GYR_ODR_3330, GYR_RNG_250); /* set the gyroscope output update frequency and measurement range */

  /************************************************************************************************************
                                            ALTIMETER CONFIG VALUES
      (aliases for each value are defined in PDC_BMP.h)
      ---------------------------------------------------------------------------------------------------------
      + MODE 1: Low Power (pressure resolution = 1.32Pa, temperature resolution = 0.005C, update frequency = 100Hz)
      + MODE 2: TODO
      + MODE 3: TODO
      + MODE 4: TODO
      + MODE 5: Ultra High Resolution (pressure resolution = 0.17Pa, temperature resolution = 0.0025C, update frequency = 25Hz)
   ************************************************************************************************************/  
  altimeter.init(ALT_MEASUREMENT_MODE_5); /* set the altimeter output data rate and resolutions */
  
  /* ---------- KALMAN FILTER SETUP ---------- */
  initKalman(); /* setup kalman filter for apogee detection (see PDC_kalman.ino) */

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
  // TODO: define a global 'measurement array' that allows us to plug various components into it at every measurement (e.g. when 'resdAltitude' is called, the temp & pressure
    // functions can access the array and auto plug these values into it
  // TODO: write a note to the microSD to signify end of setup - maybe need a .writeNote() method which blanks everything but date, time and note
}

/* -------------------- LOOP -------------------- */
void loop() {
  /* store the most recently collected packet of data in it's own instance */
  PDC_logFileFields previousLogFileLine = logFileLine;
  /* clear the active log file line ready for the next set of measurements */
  logFileLine = {}; 
  
  // TODO: timing and OBC comms
  /* PDCLogRate = 100
   * OBCWriteRate = 10
   * prevTime = getPrevTime()
   * 
   * while(1){
   *  currentTime = getCurrentTime()
   *  if (currentTime - prevTime > 1/PDCLogRate){
   *    break into logging
   *  }
   *  
   *  if (currenTime - prevTime > 1/OBCWriteRate){
   *    write latest data packet to OBC
   *  }
   *  
   * }
   */
  
  // TODO: maybe disable interrupts (i2c requests) until the bottom of this loop so that we can collect all data at this timestep
    // before servicing the I2C request

  // TODO: get current time and store in SD card structure
  //logFileLine.logTime = (TODO);
  logFileLine.flightPhase = subRoutine;  /* store the current phase of flight so we can see how accurately the transition points are determined */

  /* switch case to check the current phase of the flight and execute appropriate subroutine */
  switch(subRoutine){
    case WAIT_FOR_LAUNCH:
      waitForLaunch();
  
      // filler code to keep us entertained during testing
      float alt = altimeter.readAltitude();
      Serial.print("alt: ");
      Serial.println(alt, 5);
      break;

    case LAUNCH:
      launch();
      break;
    
    case APOGEE:
      apogee();
      break;

    case DESCENT:
      descent();
      break;

    case LANDING:
      landing();
      break;
  }

  // TODO: log file write
}

// TODO: if we can detect a component failure in flight, write a note to SD card
// this and other things should probably be prompts for error/status code development as strings are expensive

void waitForLaunch(){
  /* stay in wait mode until we exceed a pre-defined upwards acceleration */
  if(IMU.accel.readZ() > ACC_LIFTOFF_THRESHOLD){
    subRoutine = LAUNCH;
    // TODO: fill with some 'wait' routine like measuring conditions for e.g.
  }
}

void launch(){
  // TODO: what if a sensor fails? need to add a fallback mode where kalman stops and we just LPF (or similar) instead
  // TODO: is z acceleration sufficient? what if some pitch angle means the x/y axes are accelerating upward?
    
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
  if (velocity < 0) {
    subRoutine = APOGEE;
  }
}

void apogee(){
  // see if we can detect parachute deploy? 
  // continue checking light sensor incase launch phase predicted apogee too early
    // parachute deployment tasks
  // parachute detection? e.g. estimating speed & checking it's below a certain value? looking for an upward acceleration after apogee?
  // light sensor check (poll the sensor every x seconds to check ambient light levels. If new value much greater than old on all 4 sensors,
  // register apogee)
}

void descent(){
  // attitude determination tasks in a second subroutine
  // quaternion conversion from gyro Euler output
  // consider distance of gyro from CoM and if this needs compensating for
  // kalman filter likely wont work for attitude det after all - too non-linear. some other filtering necessary
  // is it possible to use kalman filter again for altitude and speed estimation during descent? potentially much less linear process on descent
}

void landing(){
  // final logs, control LEDs if necessary??
}
