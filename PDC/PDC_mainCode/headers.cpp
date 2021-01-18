#include "headers.h"

/*********************************************************
   @brief  Setup counter 1 to provide a clock on OC1A pin
   @param  the desired frequency in Hz
 *********************************************************/
void setClockOC1A(uint32_t clkFrq) {
  /*
     this function uses the ATMega counter1 to generate a clock signal on OC1A.
     the way that this works, is that we have an 'output compare' register OCR1A
        which holds some pre-defined value. we configure the counter registers TCCR1x
        so that the counter compares itself against the value in OCR1A, and when it
        detects a match, the counter resets and the OC1A pin toggles from low-high or
        from high-low.
     depending on the value in OCR1A, we will then have a square wave signal that is
        at some fraction of the internal system clock (16Mhz).
  */

  pinMode(OC1A_PIN, OUTPUT); /* clock generation will be on OC1A, so set as an output */

  TCCR1A = 0; /* reset the Timer/Counter register 1 A */
  TCCR1B = 0; /* reset the Timer/Counter register 1 B */

  TCCR1A |= (1 << 6); /* setting bit 6 means that we toggle the pin state on output compare
                         this means that on the first output compare, we go low-high, and
                         on the second output compare, we go high-low, etc etc */

  /*
     set waveform generation mode to CTC (Clear Counter on Match).
     every time we have an output compare match, we reset the counter to 0, and let it count
        back up to the predefined value.
     in CTC mode, the maximum value is the value in OCR1A, and so every time the counter hits
        this value, the counter will clear and pin OC1A will toggle
     to enable this mode, the waveform generator bits WGM13:10 should be 0100 as per datasheet
  */
  TCCR1B &= !(1 << 4);              /* set WGM13 to 0 */
  TCCR1B |= (1 << 3);               /* set WGM12 to 1 */
  /* WGM11:10 are already zero as we reset TCCR1A */

  /*
     f_OC1A = f_clk / (2 * N * (1 + OCR1A))
     so the OC1A frequency with processor speed of 16Mhz depends on N (prescaler factor) and factor OCR1A
  */

  TCCR1B |= 1;  /* set the clock source as system clock over 1 (no prescaler) so f_clk = 16Mhz; N = 1 */

  /* set our output compare register ORC1A to define our maximum counter value, and thereby set the frequency on pin OC1A */
  OCR1A = uint16_t((16000000 / (2 * clkFrq)) - 1);  /* rearrange to work out what we need for the provided target frequency & make sure is integer */
}
