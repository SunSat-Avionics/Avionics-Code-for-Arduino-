/*******************************************************************
   In this file we define some class structures for the 254
    micro-SD breakout board
   A library for this component already exists via adafruit
    (see https://github.com/adafruit/MicroSD-breakout-board), but
    we have built our own from scratch for a number of reasons
    - flexibility (if we want to add/remove functionality, it is
      easy to do so)
    - readability (adafruit lib uses lots of abstraction and
      requires much deeper understanding of all of this)
    - efficiency (this library should be much more lightweight
      than the adafruit one)

 ************************** Example usage **************************

   --- TODO ---

 *******************************************************************/

#include <Arduino.h>      /* for some arduino syntax in these cpp files */
#include "PDC_SPI.h"      /* get the SPI functions we've defined */
#include <stdio.h>        /* std stuff for printing */
#include <SD.h>           /* we want the SD card library too (https://www.arduino.cc/en/reference/SD) */
#include "PDC_logFile.h"  /* include our log file line structure so we can access the global one defined in main */

/**************************************************************************************************************
    254 MICRO-SD BREAKOUT CLASS
      we define a 254 class to keep everything packed away neatly.
      it allows us to keep hold of things that we need in attributes
      also gives us fine control over functionality
 **************************************************************************************************************/
class PDC_254 {
  private:
    /* ---------- ATTRIBUTES ---------- */
    uint8_t slaveSelect;  /* the pin on the PDC that the 254 CS pin connects to. is set on contruction */
    uint8_t cardDetect;   /* the pin on the PDC that the 254 CD pin connects to. shorts to GND when card not inserted */
    String logFileName;   /* the name of the log file that we want to write our data to */
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
