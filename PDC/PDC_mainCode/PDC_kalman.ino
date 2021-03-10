Matrix<numStates, numStates> F_matrix = {1.0,                      0.0, 0.0,
                                         kalmanTime,               1.0, 0.0,
                                         0.5 * pow(kalmanTime, 2), 0.0, 1.0
                                        };    /* state transition matrix which maps previous state to current state  */
Matrix<numMeasurements, numStates> H_matrix;  /* measurement matrix which maps the measurements to the state variables */
Matrix<numStates, numMeasurements> K_matrix;  /* kalman gain matrix which weights our measurements against the underlying model */

/**************************************************************************
   @brief  Initialise the kalman filter based on steady-state assumption
 **************************************************************************/
void initKalman() {
  /* ---------- Define Matrices ---------- */
  /* fill all matrices with 0 to start */
  stateMatrix.Fill(0.0);
  previousStateMatrix.Fill(0.0);
  predictedStateMatrix.Fill(0.0);
  measurementMatrix.Fill(0.0);

  H_matrix.Fill(0.0); /* fill with zeroes */
  K_matrix.Fill(0.0); /* fill with zeroes */
  H_matrix(0, 0) = 1.0; /* 0,0 in measurement says that we have an accelerometer measurement */
  H_matrix(1, 2) = 1.0; /* 1,2 in measurement says that we have a displacement measurement */

  /* create an identity matrix the size of the number of states */
  Matrix<numStates, numStates> stateIdentity;
  stateIdentity.Fill(0.0);
  for (int i = 0; i < numStates; i++) {
    stateIdentity(i, i) = 1.0;
  }

  /* and another that is size of the number of measurements */
  Matrix<numMeasurements, numMeasurements> measurementIdentity;
  measurementIdentity.Fill(0.0);
  for (uint8_t i = 0; i < numMeasurements; i++) {
    measurementIdentity(i, i) = 1.0;
  }

  /* the P,Q,R matrices are needed for setup, but not for operation, so are defined locally */

  Matrix<numStates, numStates> P_matrix = stateIdentity;                    /* error covariance matrix (initial guess = identity) */
  Matrix<numStates, numStates> Q_matrix = stateIdentity;                    /* process noise covariance matrix */
  Matrix<numMeasurements, numMeasurements> R_matrix = measurementIdentity;  /* measurement noise covariance matrix */

  // TODO: either measure noise and create R matrix from this, or ask sensors which mode they are in and use
  // an enum to get the noise stats as per datasheet. current values are temporary!
  // e.g. for loop, read accelerometer_z over SPI for 10 seconds, we know it should be zero, so the output is just noise
  // then use this to calculate variance.
  // this is useful for altimeter too, as we know altitude is fixed and can find variance of altitude as a single number
  // rather than taking it separately for pressure and temperature!
  // the looping would be slower but would save memory that would be used for storing an enum which we'd probably only use once
  // and the loop would still allow us to change R based on the setups of the sensors

  R_matrix(0, 0) = pow(IMU.accel.measureNoiseZ(), 2); /* measure accelerometer noise standard deviation in z-axis, square for variance */
  Serial.print("varA: ");
  Serial.println(R_matrix(0, 0), 8);

  //TODO repeat for the altimeter

  /* ---------- initialise Kalman Gain matrix, K ---------- */
  Matrix<numMeasurements, numMeasurements> sum_HPHT_R;  /* declare matrix for an interim storage */

  /* iterative calculations for K and P */
  // TODO: determine how many iterations needed for convergence
  for (uint8_t i = 0; i < 5; i++) {
    /* ---------- K = PH^T[HPH^T + R]^-1 ---------- */
    sum_HPHT_R = (H_matrix * P_matrix * ~H_matrix) + R_matrix;  /* store the term to be inverted */
    K_matrix = (P_matrix * ~H_matrix) * sum_HPHT_R.Inverse();   /* multiply P*H^T by (H*P*H^T + R)^1 and store in K */

    /* ---------- P = (I - KH)P ---------- */
    P_matrix = (stateIdentity - (K_matrix * H_matrix)) * P_matrix;

    /* ---------- P = FPF^T + Q ---------- */
    P_matrix = (F_matrix * P_matrix * ~F_matrix) + Q_matrix;

    //Serial.println("  Matrices: ");
    //Serial << " P: " << P_matrix << '\n';
    //Serial << " K: " << K_matrix << '\n';
    //Serial << " R: " << R_matrix << '\n';
    //Serial << " Q: " << Q_matrix << '\n';
  }

  // TODO: manual calculation of K and P for arbitrary setup to verify the above has worked
}

/**************************************************************************
   @brief  Kalman predict the current state of the system
 **************************************************************************/
void kalmanPredict() {
  predictedStateMatrix = F_matrix * previousStateMatrix;  /* x_k+1 = F*x_k */
}

/**************************************************************************
   @brief  Kalman update the current state of the system
 **************************************************************************/
void kalmanUpdate() {
  /* take measurements of the Z acceleration (IMU) and the altitude (altimeter) and modify the measurement matrix */
  measurementMatrix(0,0) = IMU.accel.readZ();
  measurementMatrix(1,0) = altimeter.readAltitude();

  /* x_k = x_k-1 + K*[z_k - H*x_k-1] */
  stateMatrix = predictedStateMatrix + K_matrix * (measurementMatrix - H_matrix * predictedStateMatrix);
  previousStateMatrix = stateMatrix;
}
