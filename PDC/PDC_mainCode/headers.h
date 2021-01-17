#ifndef HEADER  /* include guard */
#define HEADER

/* ---------- HARDWARE PIN VARIABLE DECLARATIONS ---------- */
extern const uint8_t PDC_SS;        /* the SS pin on the arduino PDC (=10 for nano) */
extern const uint8_t altimeter_SS;  /* the arduino PDC pin connected to the altimiter slave select pin */
extern const uint8_t IMU_SS;        /* the arduino PDC pin connected to the IMU slave select pin */
extern const uint8_t microSD_SS;    /* the arduino PDC pin connected to the micro-SD module slave select pin */

extern const uint8_t microSD_CD;    /* the arduino PDC pin connected to the micro-SD module card-detect pin */

/* ----------  ---------- */


#endif
