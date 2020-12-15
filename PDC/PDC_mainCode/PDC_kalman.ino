/* setup the kalman filter matrices and gain. this works on steady-state assumption so gain isn't calculated at every time step */
// TODO: verify that these calcs are actually working, and verify that the matrices carry between functions since they were delcared globally
void initKalman() {

  Serial.println("Starting Kalman Filter Initialisation");
  
  /* ---------- Define Matrices ---------- */
  /* set measurment matrix to map measurements to states */
  H_matrix(0, 0) = 1.0;
  H_matrix(1, 2) = 1.0;

  /* make a transpose of the measurement and transition matrices for calculations */
  Matrix<numStates, numMeasurements> H_matrixTranspose = ~H_matrix;
  Matrix<numStates, numStates> F_matrixTranspose = ~F_matrix;

  // either measure noise and create R matrix from this, or ask sensors which mode they are in and use
  // an enum to get the noise stats as per datasheet

  /* ---------- initialise Kalman Gain matrix, K ---------- */
  // TODO: determine how many iterations needed for convergence
  for (int i = 0; i < 5; i++) {
    /* Matrix.Multiply function is part of the matrix math library and requires the matrices and their sizes */

    /* ---------- K = PH^T[HPH^T + R]^-1 ---------- */
    /* multiply P with H^T */
    Matrix<numStates, numMeasurements> product_PHT = P_matrix * H_matrixTranspose;

    /* multiply H with PH^T */
    Matrix<numMeasurements, numMeasurements> product_HPHT = H_matrix * product_PHT;

    /* add R to H*P*H^T */
    Matrix<numMeasurements, numMeasurements> sum_HPHT_R = R_matrix + product_HPHT;

    /* invert (H*P*H^T + R) */
    Matrix<numMeasurements, numMeasurements> inverse_sum_HPHT_R = sum_HPHT_R.Inverse();

    /* multiply P*H^T by (H*P*H^T + R)^1 and store in K */
    K_matrix = product_PHT * inverse_sum_HPHT_R;

    // TODO: manual calculation of K for arbitrary setup to verify the above has worked

    /* ---------- P = (I - KH)P ---------- */
    /* multiply K by P */
    Matrix<numStates, numStates> product_KH = K_matrix * H_matrix;

    /* subtract K*H from I */
    Matrix<numStates, numStates> identity;
    for (int i = 0; i < numStates; i++) {
      identity(i, i) = 1.0;
    }

    Matrix<numStates, numStates> sub_KH_I = identity - product_KH;

    /* multiply I-KH by P and store in P */
    P_matrix = sub_KH_I * P_matrix;

    /* ---------- P = FPF^T + Q ---------- */
    /* multiply P by F^T */
    Matrix<numStates, numStates> product_PFT = P_matrix * F_matrixTranspose;

    /* multiply F by P*F^T */
    Matrix<numStates, numStates> product_FPFT = F_matrix * product_PFT;

    /* add Q to F*P*F^T and store in P */
    P_matrix = product_FPFT + Q_matrix;
  }
  
  Serial.println("Kalman Setup Complete!");
  Serial.println("Matrices: ");
  Serial << "P: " << P_matrix << '\n';
  Serial << "K: " << K_matrix << '\n';
  Serial << "R: " << R_matrix << '\n';
  Serial << "Q: " << Q_matrix << '\n';
}

/* this function is used to filter the sensor data during ascent to help us predict apogee */
// TODO: rory make a document to summarise why we need this and what it is doing
// TODO: split into 'predict' and 'update'? and should we force trigger the prediction at regular intervals for consistency in the F matrix (and therefore the gain)?
void parachuteKalmanFilter()
{
}
