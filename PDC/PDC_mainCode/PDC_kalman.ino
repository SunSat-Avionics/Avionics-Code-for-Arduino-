/**
 * @brief  Initialise the kalman filter based on steady-state assumption
 */
void initKalman() {

  /* ---------- Define Matrices ---------- */
  /* set measurment matrix to map measurements to states */
  H_matrix(0, 0) = 1.0;
  H_matrix(1, 2) = 1.0;

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
  /* error covariance matrix */
  Matrix<numStates, numStates> P_matrix;
  /* process noise covariance matrix */
  Matrix<numStates, numStates> Q_matrix = stateIdentity;
  /* measurement noise covariance matrix */
  Matrix<numMeasurements, numMeasurements> R_matrix = measurementIdentity;

  /* make a transpose of the measurement and transition matrices for calculations */
  Matrix<numStates, numMeasurements> H_matrixTranspose = ~H_matrix;
  Matrix<numStates, numStates> F_matrixTranspose = ~F_matrix;

  // TODO: either measure noise and create R matrix from this, or ask sensors which mode they are in and use
  // an enum to get the noise stats as per datasheet. current values are temporary!
  // e.g. for loop, read accelerometer_z over SPI for 10 seconds, we know it should be zero, so the output is just noise
  // then use this to calculate variance.
  // this is useful for altimeter too, as we know altitude is fixed and can find variance of altitude as a single number
  // rather than taking it separately for pressure and temperature!
  // the looping would be slower but would save memory that would be used for storing an enum which we'd probably only use once
  // and the loop would still allow us to change R based on the setups of the sensors

  /* measure the standard deviation of the accelerometer noise, then square it to get variance for R */
  R_matrix(0, 0) = pow(IMU.measureAccelerometerNoiseZ(), 2);
  Serial.print("Variance_a: ");
  Serial.println(R_matrix(0, 0), 8);

  //TODO repeat for the altimeter
  
  /* ---------- initialise Kalman Gain matrix, K ---------- */
  /* declare matrix for an interim storage */
  Matrix<numMeasurements, numMeasurements> sum_HPHT_R;

  /* initial guess for P */
  P_matrix = stateIdentity;

  /* iterative calculations for K and P */
  // TODO: determine how many iterations needed for convergence
  for (uint8_t i = 0; i < 5; i++) {
    /* ---------- K = PH^T[HPH^T + R]^-1 ---------- */
    /* store the term to be inverted */
    sum_HPHT_R = (H_matrix * P_matrix * H_matrixTranspose) + R_matrix;

    /* multiply P*H^T by (H*P*H^T + R)^1 and store in K */
    K_matrix = (P_matrix * H_matrixTranspose) * sum_HPHT_R.Inverse();

    /* ---------- P = (I - KH)P ---------- */
    /* multiply I-KH by P and store in P */
    P_matrix = (stateIdentity - (K_matrix * H_matrix)) * P_matrix;

    /* ---------- P = FPF^T + Q ---------- */
    /* add Q to F*P*F^T and store in P */
    P_matrix = (F_matrix * P_matrix * F_matrixTranspose) + Q_matrix;

    Serial.println("  Matrices: ");
    Serial << " P: " << P_matrix << '\n';
    Serial << " K: " << K_matrix << '\n';
    Serial << " R: " << R_matrix << '\n';
    Serial << " Q: " << Q_matrix << '\n';
  }

  // TODO: manual calculation of K and P for arbitrary setup to verify the above has worked
}

/**
 * @brief  Kalman predict the current state of the system
 */
void kalmanPredict() {
  /* x_k+1 = F*x_k */
  predictedStateMatrix = F_matrix * previousStateMatrix;
}

/**
 * @brief  Kalman update the current state of the system
 */
void kalmanUpdate() {
  float accelerationZ = IMU.readAccelerationZ();
  // TODO: take altitude measurement from altimeter
  // TODO: create measurement vector from the readings
  
  /* x_k = x_k-1 + K*[z_k - H*x_k-1] */
  stateMatrix = predictedStateMatrix + K_matrix * (measurementMatrix - H_matrix * predictedStateMatrix);
  previousStateMatrix = stateMatrix;
}
