/* setup the kalman filter matrices and gain. this works on steady-state assumption so gain isn't calculated at every time step */
// TODO: add a separate function for the rows/cols fetching? remove matrixmath altogether?
void initKalman()
{
  /* make a transpose of the measurement matrix for calculations*/
  Matrix H_matrixTranspose = H_matrix.Transpose();

  /* find size of P for matrix ops later */
  /*int rowsP = sizeof(P) / sizeof(P[0]);
  int colsP = sizeof(P[0]) / sizeof(P[0][0]);*/

  // either measure noise and create R matrix from this, or ask sensors which mode they are in and use
  // an enum to get the noise stats as per datasheet
  
  /* initialise Kalman Gain matrix, K */
  for (int i = 0; i < 5; i++) {
    /* Matrix.Multiply function is part of the matrix math library and requires the matrices and their sizes */
    
    /* ---------- K = PH^T[HPH^T + R]^-1 ---------- */
    /* store the product of P*H^T */
    /* float product_PHT[rowsP][colsHT]; */
    /* multiply P with H^T */
    /*Matrix.Multiply((mtx_type *)P,
                    (mtx_type *)transposeH,
                      rowsP, colsP, colsHT,
                    (mtx_type *)product_PHT);*/
                    
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
    // TODO
    /* multiply K by P */
    Matrix product_KP = K_matrix.Math(Matrix::MULTIPLY, &P_matrix);

    /* subtract K*P from I */
    Matrix eye(product_KP.Rows(), true);
    Matrix sub_KP_I = eye.Math(Matrix::SUBTRACT, &product_KP);

    /* multiply I-KP by P and store in P */
    P_matrix = sub_KP_I.Math(Matrix::MULTIPLY, &P_matrix);
      
    /* ---------- P = FPF^T + Q ---------- */
    // TODO
    
  }

  
}

/* this function is used to filter the sensor data during ascent to help us predict apogee */
// TODO: rory make a document to summarise why we need this and what it is doing
void parachuteKalmanFilter()
{
}
