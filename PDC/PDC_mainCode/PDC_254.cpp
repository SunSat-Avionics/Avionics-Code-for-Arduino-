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
    /* communicate with micro SD - write the csv headers to our file
       should define (either here or elsewhere) the units of each of these headers... perhaps in readme */
    // TODO: populate this more fully - what are the raw measurements from BMP, GYRO, light sensors, etc?
    char headerData[] = "Date, Time, acc_x, acc_y, acc_z, Note";
    writeData(headerData);
    
    /* get the time since we started */
    // TODO print a new line at currentTime - timeSinceStartup with a note of 'program start' or similar
    uint32_t timeSinceStartup = millis();
    return (0);
  }
}

/* write some data to the microSD card */
bool PDC_254::writeData(char *data) {
  bool err = 0;

  if (cardInserted()) {
    dataLogFile.print(data);
  }
  else {
    err = 1;
  }
  return (err);
}
