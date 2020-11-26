/* SUNSAT Parachute Deployment and Attitude Determination Software
  
   Created 22 Nov 20
   Contributors: Rory Haggart
   
   Description
   -----------
   For the Sheffield University Nova SATellite (SUNSAT) platform, this code is to be onboard the 'parachute deployment computer' (PDC).
   The PDC is an Arduino Nano serving the dual purpose of parachute deployment activities (e.g. apogee detection), and attitude determination.
*/


void setup() {
  // SPI setup (accel/gryo, altimeter, SD card)
  
  // I2C setup (RTC unit)
  
  // light sensor pin configuration (digital output to SI pin, analogue input(s) from AO pins, clock signal to CLK pins) 

  // setup interrupt pin? TBD - can we simply configure one of the GPIO to go high and connect this to OBC interrupt, and then execute
    // the interrupt routine on OBC?
  
  // initialise Kalman Filter (e.g. velocity = 0, altitude = 0, or whatever else)
  
  // confirm setup has succeeded? e.g. ask SPI for accel values and verify zero (check 'who am i' reg or similar)
    // then alert main OBC that PDC is setup and ready to go
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
