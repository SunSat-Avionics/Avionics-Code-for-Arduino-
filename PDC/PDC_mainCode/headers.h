#ifndef _HEADER /* include guard */
#define _HEADER

#include <Arduino.h>  /* bring some arduino syntax into the cpp files */
#include <stdio.h> 

/* ---------- HARDWARE PIN VARIABLE DECLARATIONS ---------- */
extern const uint8_t PDC_SS;        /* the SS pin on the arduino PDC (=10 for nano) */
extern const uint8_t altimeter_SS;  /* the arduino PDC pin connected to the altimiter slave select pin */
extern const uint8_t IMU_SS;        /* the arduino PDC pin connected to the IMU slave select pin */
extern const uint8_t microSD_SS;    /* the arduino PDC pin connected to the micro-SD module slave select pin */

extern const uint8_t microSD_CD;    /* the arduino PDC pin connected to the micro-SD module card-detect pin */

extern const uint8_t LPA_SI;  /* the serial input pin that is used to trigger a new output from the LPAs */
extern const uint8_t LPA_AO;  /* the analog output pin that the LPAs will send their values to */
extern const uint8_t LPA_CLK; /* the pin that will provide clock signal to the LPAs. SHOULD BE KEPT AS PIN 9 ON NANO */
const uint8_t OC1A_PIN = 9;   /* the ATMega OC1A pin for nano is pin 9. this pin can be used to generate a clock signal up to 8MHz */

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

/* ---------- FUNCTION DECLARATIONS ---------- */
void setClockOC1A(uint32_t clkFrq);

#endif
