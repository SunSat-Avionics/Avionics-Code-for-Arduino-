/* setup the kalman filter matrices and gain. this works on steady-state assumption so gain isn't calculated at every time step */
// TODO: verify that these calcs are actually working, and verify that the matrices carry between functions since they were delcared globally
void initKalman() {

  Serial.println("Kalman init");

  /* ---------- Define Matrices ---------- */
  /* set measurment matrix to map measurements to states */
  H_matrix(0, 0) = 1.0;
  H_matrix(1, 2) = 1.0;

  /* create an identity matrix the size of the number of states */
  Matrix<numStates, numStates> stateIdentity;
  stateIdentity.Fill(0.0);
  for (int i = 0; i < numStates; i++) {
    /* define identity matrix */
    stateIdentity(i, i) = 1.0;
  }

  /* and another that is size of the number of measurements */
  Matrix<numMeasurements, numMeasurements> measurementIdentity;
  measurementIdentity.Fill(0.0);
  for (uint8_t i = 0; i < numMeasurements; i++) {
    /* define identity matrix */
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

  /* ---------- initialise Kalman Gain matrix, K ---------- */
  /* declare matrices for interim storage */
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

  Serial.println("  :)");
  
  // TODO: manual calculation of K and P for arbitrary setup to verify the above has worked
}

/* use the previous states and the underlying model to predict the current state of the system */
Matrix kalmanPredict(){
  /* x_k+1 = F*x_k */
  predictedStateMatrix = F_matrix * previousStateMatrix;
}

/* use the sensor data to update (refine) the state predictions */
// TODO: should we force trigger the prediction at regular intervals for consistency in the F matrix? i.e. the F matrix (and therefore the gain) depends on the 
  // change in time, so might make sense to only take SPI readings at forced intervals, rather than just taking readings and measuring the time since the last reading
void kalmanUpdate(){
  // TODO: take measurements
  
  /* x_k = x_k-1 + K*[z_k - H*x_k-1] */
  stateMatrix = predictedStateMatrix + K_matrix(measurementMatrix - H*predictedStateMatrix);
}
