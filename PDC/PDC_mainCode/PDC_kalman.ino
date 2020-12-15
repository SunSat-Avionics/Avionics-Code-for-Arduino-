/* setup the kalman filter matrices and gain. this works on steady-state assumption so gain isn't calculated at every time step */
// TODO: verify that these calcs are actually working, and verify that the matrices carry between functions since they were delcared globally
void initKalman()
{
  Serial.println("H:");
  H_matrix.PrintMatrix();
  
  /* make a transpose of the measurement and transition matrices for calculations*/
  Matrix H_matrixTranspose = H_matrix.Transpose();
  Matrix F_matrixTranspose = F_matrix.Transpose();

  Serial.println("H_transpose:");
  H_matrixTranspose.PrintMatrix();

  // either measure noise and create R matrix from this, or ask sensors which mode they are in and use
  // an enum to get the noise stats as per datasheet
  
  /* initialise Kalman Gain matrix, K */
  // TODO: determine how many iterations needed for convergence
  for (int i = 0; i < 5; i++) {
    /* Matrix.Multiply function is part of the matrix math library and requires the matrices and their sizes */
    
    /* ---------- K = PH^T[HPH^T + R]^-1 ---------- */         
    /* multiply P with H^T */
    Matrix product_PHT = P_matrix.Math(Matrix::MULTIPLY, &H_matrixTranspose);
                    
    /* multiply H with PH^T */
    Matrix product_HPHT = H_matrix.Math(Matrix::MULTIPLY, &product_PHT);

    /* add R to H*P*H^T */
    Matrix sum_HPHT_R = R_matrix.Math(Matrix::ADD, &product_HPHT);
       
    /* invert (H*P*H^T + R) */
    Matrix inverse_sum_HPHT_R = sum_HPHT_R.Inverse();
                    
    /* multiply P*H^T by (H*P*H^T + R)^1 and store in K */
    K_matrix = product_PHT.Math(Matrix::MULTIPLY, &inverse_sum_HPHT_R);

    // TODO: manual calculation of K for arbitrary setup to verify the above has worked

    /* ---------- P = (I - KP)P ---------- */
    /* multiply K by P */
    Matrix product_KP = K_matrix.Math(Matrix::MULTIPLY, &P_matrix);

    /* subtract K*P from I */
    Matrix eye(product_KP.Rows(), true);
    Matrix sub_KP_I = eye.Math(Matrix::SUBTRACT, &product_KP);

    /* multiply I-KP by P and store in P */
    P_matrix = sub_KP_I.Math(Matrix::MULTIPLY, &P_matrix);
      
    /* ---------- P = FPF^T + Q ---------- */
    // TODO
    /* multiply P by F^T */
    Matrix product_PFT = P_matrix.Math(Matrix::MULTIPLY, &F_matrixTranspose);

    /* multiply F by P*F^T */
    Matrix product_FPFT = F_matrix.Math(Matrix::MULTIPLY, &product_PFT);

    /* add Q to F*P*F^T and store in P */
    P_matrix = product_FPFT.Math(Matrix::ADD, &Q_matrix);    
  }

  
}

/* this function is used to filter the sensor data during ascent to help us predict apogee */
// TODO: rory make a document to summarise why we need this and what it is doing
void parachuteKalmanFilter()
{
}
