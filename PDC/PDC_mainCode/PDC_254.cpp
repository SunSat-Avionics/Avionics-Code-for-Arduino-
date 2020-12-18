#include "PDC_254.h"

bool PDC_254::isAlive() {
  /* have a code to signify result - true is success */
  bool successCode = 1;

  /* first, check that a card is inserted. if so, try to initialise the card */
  if(!cardInserted()){
    /* flag error if card not inserted, isAlive will return false */
    successCode = 0; 
  }
  else{
    /* initialise card if inserted */
    if(!SD.begin(slaveSelect)){
      /* flag error if card can't be initialised, isAlive will return false */
      successCode = 0;      
    }
  }

  return (successCode);
}

/* detect if a card is inserted. will be useful for writing data and flagging errors (e.g. if !microSD.cardInserted then error;) */
bool PDC_254::cardInserted(){
  if(cardDetect == LOW){
    return(0);
  }
  else {
    return(1);
  }
  
}

/* write some data to the microSD card */
void PDC_254::writeData(){
  if(cardInserted()){
    // write data
  }
  else{
    // ERR
  }
}
