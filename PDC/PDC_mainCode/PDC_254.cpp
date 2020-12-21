#include "PDC_254.h"

bool PDC_254::isAlive() {
  /* have a code to signify result - true is success */
  bool success = 1;

  /* first, check that a card is inserted. if so, try to initialise the card */
  if (!cardInserted()) {
    /* flag error if card not inserted, isAlive will return false */
    success = 0;
  }
  else {
    /* initialise card if inserted */
    if (!SD.begin(slaveSelect)) {
      /* flag error if card can't be initialised, isAlive will return false */
      success = 0;
    }
  }

  return (success);
}

/* detect if a card is inserted. will be useful for writing data and flagging errors (e.g. if !microSD.cardInserted then error;) */
bool PDC_254::cardInserted() {
  if (cardDetect == LOW) {
    return (0);
  }
  else {
    return (1);
  }
}

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

/* write some data to the microSD card */
/* data format:
    Date, Time, acc_x, acc_y, acc_z, Note" */
// TODO: populate this more fully - what are the raw measurements from BMP, GYRO, light sensors, etc? Make a .txt with headers and units
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
