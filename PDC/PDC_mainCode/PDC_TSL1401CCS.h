/*******************************************************************
   In this file we define some class structures for the TSL1401CCS
    linear photodiode arrays.
   The class represents the interface and communications with the
    device. For example, the attributes are the pins on the PDC that
    the device is connected to, and the methods are commands for the
    PDC that trigger read/write events.
   Due to the nature of their connection to the PDC, it makes sense
    to wrap all four sensors into a single class, since they are
    cascaded and between them, they only require a single input,
    output, and clock signal.
 ************************** Example usage **************************

  //TODO

 *******************************************************************/

#include "headers.h"  /* for processor functions - particularly for starting the clock signal on OC1A pin */
#include <Arduino.h>  /* bring some arduino syntax into the cpp files */
#include <stdio.h>    /* std stuff for cpp */

/* ---------- OC1A CLOCK SIGNAL FREQUENCY ALIASES ---------- */
/* TSL1401CCS linear photodiode array accepts clock between 5kHz and 8MHz, so facilitate this range */
const uint32_t OC1A_5KHZ   = 5000;
const uint32_t OC1A_10KHZ  = 10000;
const uint32_t OC1A_50KHZ  = 50000;
const uint32_t OC1A_100KHZ = 100000;
const uint32_t OC1A_250KHZ = 250000;
const uint32_t OC1A_500KHZ = 500000;
const uint32_t OC1A_1MHZ   = 1000000;
const uint32_t OC1A_2MHZ   = 2000000;
const uint32_t OC1A_4MHZ   = 4000000;
const uint32_t OC1A_8MHZ   = 8000000;

/* note the range of frequencies to check entries are valid */
const uint32_t TSL1401CCS_CLK_MIN = OC1A_5KHZ;
const uint32_t TSL1401CCS_CLK_MAX = OC1A_8MHZ;

/**************************************************************************
    a class for the TSL1401CCS linear photodiode array group
      this class contains methods and attributes of all four LPAs in one
      structure to make controllability much easier
 **************************************************************************/
class PDC_TSL1401CCS_GROUP {
  private:
    /* ---------- ATTRIBUTES ---------- */
    uint8_t analogOut;        /* the pin on the PDC connected to the analog out of the LPA group (multi-die continuous scan connection) */
    uint8_t serialIn;         /* the pin on the PDC connected to the serial input to the LPA group */
    uint8_t clockPin;         /* the pin on the PDC that generates the clock for the LPA group (should be the OC1A pin on PDC) */
    
    uint32_t clockFrequency;  /* the frequency that the clock signal for the LPA group is running at */

  public:
    /* ---------- CONSTRUCTOR ---------- */
    PDC_TSL1401CCS_GROUP(uint8_t SI, uint8_t CLK, uint8_t AO){
      serialIn = SI;  /* note the pins that are in control of this device */
      clockPin = CLK;
      analogOut = AO;
    };

    /* ---------- METHODS ---------- */
    uint8_t startClockOC1A(uint32_t clockFrq);  /* generate a clock signal on the PDC OC1A pin */
};
