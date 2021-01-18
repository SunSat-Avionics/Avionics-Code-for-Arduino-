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
