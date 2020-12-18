#include <Arduino.h>
#include "PDC_SPI.h"
#include <stdio.h>
/* we want the SD card library too (https://www.arduino.cc/en/reference/SD) */
#include <SD.h>

/* we define a class for the 254 micro-SD breakout board */
class PDC_254 {
  private:
    /* the pin on the PDC that the 254 CS pin connects to. is set on contruction */
    uint8_t slaveSelect;
    /* the pin on the PDC that the 254 CD pin connects to. shorts to GND when card not inserted */
    uint8_t cardDetect;
    /* the log file that we will store data on */
    File dataLogFile;

  public:
    /* constructor - set slaveSelect to the specified SS pin */
    PDC_254(uint8_t CS, uint8_t CD) {
      slaveSelect = CS;
      cardDetect = CD;
    };
    /* check if connected and responsive */
    bool isAlive();
    /* check if card is inserted */
    bool cardInserted();
    /* write some data to the microSD card. returns 0 if successful */
    bool writeData(char *data);
    /* open a new file to log data to */
    bool openFile();
};

/* Example usage

*/
