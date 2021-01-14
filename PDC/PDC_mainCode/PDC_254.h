#include <Arduino.h>  /* for some arduino syntax in these cpp files */
#include "PDC_SPI.h"  /* get the SPI functions we've defined */
#include <stdio.h>    /* std stuff for printing */
#include <SD.h>       /* we want the SD card library too (https://www.arduino.cc/en/reference/SD) */

/* 254 CLASS
 *  we define a 254 class to keep everything packed away neatly. 
 *  it allows us to keep hold of things that we need in attributes
 *  also gives us fine control over functionality
 *  adafruit have a library for this component already but this is more readable, flexible, and lightweight
 */
class PDC_254 {
  private:
    /* ---------- ATTRIBUTES ---------- */
    uint8_t slaveSelect;  /* the pin on the PDC that the 254 CS pin connects to. is set on contruction */
    uint8_t cardDetect;   /* the pin on the PDC that the 254 CD pin connects to. shorts to GND when card not inserted */
    File dataLogFile;     /* the log file that we will store data on */

  public:
    /* ---------- CONSTRUCTOR ---------- */
    PDC_254(uint8_t CS, uint8_t CD) {
      slaveSelect = CS;
      cardDetect = CD;
    };

    /* ---------- METHODS ---------- */
    bool isAlive();             /* check if connected and responsive */
    bool cardInserted();        /* check if card is inserted */
    bool writeData(char *data); /* write some data to the microSD card. returns 0 if successful */
    bool openFile();            /* open a new file to log data to */
};

/* Example usage

*/
