#ifndef _HEADER /* include guard */
#define _HEADER

#include <Arduino.h>  /* bring some arduino syntax into the cpp files */
#include <stdio.h> 

/* ---------- GENERAL PARAMETERS ---------- */
extern const uint8_t LAUNCH_SITE_ALTITUDE;  /* [m] how high above sea level is the launch site? used for measuring noise in altimeter */
extern const uint8_t ACC_LIFTOFF_THRESHOLD; /* [m/s^2] the threshold value that tells us we have liftoff. this triggers the move from 'wait' mode to 'flight' mode */
extern const float kalmanTime;              /* time step (s) between Kalman iterations */

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

/* ---------- FUNCTION DECLARATIONS ---------- */
void setClockOC1A(uint32_t clkFrq); /* a function that accesses the ATmega registers and sets the OC1A pin to provide a clock signal */
void detectClockEdge(uint8_t clockSignal, uint8_t edgeType);

#endif
