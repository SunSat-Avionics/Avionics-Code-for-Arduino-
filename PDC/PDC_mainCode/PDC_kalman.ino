/* setup the kalman filter matrices and gain. this works on steady-state assumption so gain isn't calculated at every time step */
void initKalman(int* H)
{ 
  // define relevant matrices (F, H, etc.)

  // either measure noise and create R matrix from this, or ask sensors which mode they are in and use
    // an enum to get the noise stats as per datasheet

  // initialise kalman gain using iterative method
}

/* this function is used to filter the sensor data during ascent to help us predict apogee */
// TODO: rory make a document to summarise why we need this and what it is doing
void parachuteKalmanFilter()
{
}
