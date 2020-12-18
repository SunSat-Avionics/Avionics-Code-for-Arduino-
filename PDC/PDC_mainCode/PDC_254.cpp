#include "PDC_254.h"

bool PDC_254::isAlive() {
  /* have a code to signify result - true is success */
  bool successCode = 1;

  /* first, check that a card is inserted. if so, try to initialise the card */
  if(cardDetect == LOW){
    /* flag error if card not inserted, isAlive will return false */
    successCode = 0; 
  }
  else{
    if(!SD.begin(slaveSelect)){
      /* flag error if card can't be initialised, isAlive will return false */
      successCode = 0;      
    }
  }

  return (successCode);
}

void PDC_254::writeData(){
  ;
}
