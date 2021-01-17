#include "headers.h"

/*********************************************************
   @brief  Setup counter 1 to provide a clock on pin 9
   @param  the desired frequency in Hz
 *********************************************************/
void setCLK(uint32_t clkFrq){
  pinMode(OC1A_PIN, OUTPUT); /* clock generation on OC1A - arduino clock is 16 MHz */
  
  TCCR1A = 0; /* reset the Timer/Counter register 1 A */
  TCCR1B = 0; /* reset the Timer/Counter register 1 B */
  
  TCCR1A |= (1 << 6); /* toggle OC1A (output compare for timer 1) on compare match event */
  
  /* set waveform generation mode to CTC (Clear Counter on Match). WGM13:10 should be 0100 */
  TCCR1A &= !((1 << 1) | (1 << 0)); /* set WGM11:10 to 00 */
  TCCR1B &= !(1 << 4);              /* set WGM13 to 0 */
  TCCR1B |= (1 << 3);               /* set WGM12 to 1 */

  /* f_OC1A = f_clk / (2 * N * (1 + OCR1A)) so the OC1A frequency at 16Mhz depends on N (prescaler factor) and factor OCR1A */
   
  TCCR1B |= 1;  /* set the clock source as system clock over 1 (prescaler) so f_clk = 16Mhz */
  
  /* set output compare register A to 'x'. if OC1A toggles every x clock cycles, the f_OC1A is (16Mhz / 2*x) with N = 1 */
  OCR1A = uint16_t((16000000 / (2 * clkFrq)) - 1);  /* rearrange to work out what we need for the provided target frequency & make sure is integer */
}
