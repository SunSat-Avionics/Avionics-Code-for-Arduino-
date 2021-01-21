// TODO 'startLPAClock' method or similar, calling header function. can store clock frequency as attribute for taking measurements
// 'takeMeasurement' method to set the SI high for half a clock cycle

#include "PDC_TSL1401CCS.h"

/***************************************************************************
   @brief  Start the OC1A clock signal on the PDC
   @retval 0 in case of success, 1 otherwise
 ***************************************************************************/
uint8_t PDC_TSL1401CCS_GROUP::startClockOC1A(uint32_t clockFreq){
  /* if the class object has been told that its clock signal pin is anything other than the PDC OC1A pin, error */
  if(clockPin != OC1A_PIN){
    return(1);
  }

  /* if the requested clock frequency is out of the bounds of the TSL1401CCS (5Khz - 8MHz), error */
  if((clockFreq < TSL1401CCS_CLK_MIN) | (clockFreq > TSL1401CCS_CLK_MAX)){
    return(1);
  }
  
  setClockOC1A(clockFreq);    /* set up a clock signal on OC1A pin (pin 9 on nano) at selected frequency, used to control the LPAs */
  clockFrequency = clockFreq; /* internally remember what frequency we have started the clock at */

  return(0);
}

/***************************************************************************
   @brief  Read a new value from the group of LPAs
 ***************************************************************************/
void PDC_TSL1401CCS_GROUP::readValues(){
  uint8_t pixel;  /* loop index variable */

  //TODO: how to store the values?
  
  float delayTime = 1000.0 * (1.0 / (2.0 * clockFrequency));  /* approximate half cycle delay in ms */
  
  const uint8_t RISING_EDGE = 1;  /* alias for clock signal rising edge option */
  const uint8_t FALLING_EDGE = 0; /* alias for clock signal falling edge option */
  
  detectClockEdge(clockPin, RISING_EDGE);   /* wait for a rising edge on the clock pin */
  digitalWrite(serialIn, HIGH);             /* set the LPA SI high to trigger a new output read */
  detectClockEdge(clockPin, FALLING_EDGE);  /* wait for a falling edge on the clock pin */
  digitalWrite(serialIn, LOW);              /* set the LPA SI low before the next rising edge on the clock */

  //TODO: how to do this for each of the four LPAs on continuous scan multi-die?
  for(pixel = 0; pixel < 128; pixel++){
    detectClockEdge(clockPin, RISING_EDGE); /* LPA sets each pixel value on rising edges after SI goes low again */
    delay(delayTime);                       /* delay for approximately half a clock cycle before reading value to allow value to settle */
    //TODO: READ PIXEL                      /* read the value of this pixel */
  }

  //TODO: how return?
}
