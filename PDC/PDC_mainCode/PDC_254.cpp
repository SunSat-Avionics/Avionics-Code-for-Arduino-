#include "PDC_254.h"  /* grab the class definition */

/**********************************************
   @brief  Check if connection to device is ok
   @retval 1 in case of success, 0 otherwise
 **********************************************/
bool PDC_254::isAlive() {
  bool isAlive = 1; /* aliveness indicator */

  /* check that a card is inserted. if so, try to initialise the card */
  if (!cardInserted()) {
    isAlive = 0;  /* flag error if card not inserted */
  }
  else {
    if (!SD.begin(slaveSelect)) {
      isAlive = 0;  /* flag error if card can't be initialised */
    }
  }

  return (isAlive);
}

/**********************************************
   @brief  Detect that a card is inserted
   @retval 1 in case of success, 0 otherwise
 **********************************************/
bool PDC_254::cardInserted() {
  /* card detect shorts to ground when no card is inserted */
  if (cardDetect == LOW) {
    return (0);
  }
  else {
    return (1);
  }
}

/**********************************************
   @brief  Open a file to write the data to
   @retval 0 in case of success, 1 otherwise
 **********************************************/
bool PDC_254::openFile() {
  /* new instance of the 'File' class (part of the SD library) that we will use to control the .csv file on the microSD card */
  // TODO: once RTC is up & running, name the file with timestamp as per ISO 8601 format (kind of..)(yyyy-mm-ddThh-mm-ss.csv)
  dataLogFile = SD.open("temp.csv", FILE_WRITE);

  /* if the file fails to open, return an error */
  if (!dataLogFile) {
    return (1);
  }
  else {
    /* get the time since we started */
    // TODO print a new line at currentTime - timeSinceStartup with a note of 'program start' or similar
    uint32_t timeSinceStartup = millis();
    return (0);
  }
}

/*****************************************************
   @brief  Write data to the file on the microSD card
   @retval 0 in case of success, 1 otherwise
 *****************************************************/
/* data format:
    Time, phase of flight, acc_x (measured), acc_y (measured), acc_z (measured), temp, pressure, altitude (altimeter), light sensor 1, 2, 3, 4, acc_z (estimate), vel_z (estimate), altitude (estimate), Note"
*/
// TODO: populate this more fully - what are the raw measurements from BMP, GYRO, light sensors, etc? Make a .txt with headers and units
// TODO: name value pairs? leaves us better equipped for changing headers and incomplete data
bool PDC_254::writeData(char *data) {
  bool err = 0;

  // date = ...
  // time = ...
  // for all data
  // writeSPI(date + ",");
  // ...

  // if data doesn't fully populate all columns, fill with err or 0
  // note: it's *probably* a better idea to do individual writes via SPI in a for loop (e.g. for each entry, write SPI)
  // instead of trying to pass all entries into write SPI instead - would require some weird trickery of the input to the
  // function, and might end up wiriting to consecutive registers? needs more research

  if (cardInserted()) {
    dataLogFile.print(data);
  }
  else {
    err = 1;
  }
  return (err);
}
