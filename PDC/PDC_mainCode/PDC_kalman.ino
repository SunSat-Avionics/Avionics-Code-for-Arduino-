/* setup the kalman filter matrices and gain. this works on steady-state assumption so gain isn't calculated at every time step */
// TODO: add a separate function for the rows/cols fetching? remove matrixmath altogether?
void initKalman()
{
  /* dyanmically find the size of H, just incase it ends up changing... 
     a 2D array is an 'array of arrays', so the number of rows is just the total size divided by the first array
     then the number of cols is the size of one of the arrays, divided by the size of just one element in that array */
  int rowsH = sizeof(H) / sizeof(H[0]);
  int colsH = sizeof(H[0]) / sizeof(H[0][0]);

  /* then the size of the transpose is opposite */
  int rowsHT = colsH;
  int colsHT = rowsH;

  /* make a transpose of the measurement matrix */
  float transposeH[rowsHT][colsHT];

  /* find size of P for matrix ops later */
  int rowsP = sizeof(P) / sizeof(P[0]);
  int colsP = sizeof(P[0]) / sizeof(P[0][0]);

  /* transpose H */
  for (int i = 0; i < rowsHT; i++)
  {
    for (int j = 0; j < colsHT; j++)
    {
      transposeH[i][j] = H[j][i];
    }
  }

  // either measure noise and create R matrix from this, or ask sensors which mode they are in and use
  // an enum to get the noise stats as per datasheet
  
  /* initialise Kalman Gain matrix, K */
  for (int i = 0; i < 5; i++) {
    /* Matrix.Multiply function is part of the matrix math library and requires the matrices and their sizes */
    
    /* ---------- K = PH^T[HPH^T + R]^-1 ---------- */
    /* store the product of P*H^T */
    float temp1[rowsP][colsHT];
    /* multiply P with H^T */
    Matrix.Multiply((mtx_type *)P, (mtx_type *)transposeH, rowsP, colsP, colsHT, (mtx_type *)temp1);
    /* store the product of H with temp1 (PH^T) */
    float temp2[rowsH][colsHT];
    /* multiply H with PH^T */
    Matrix.Multiply((mtx_type *)H, (mtx_type *)temp1, rowsH, colsH, colsHT, (mtx_type *)temp2);
    /* store the sum of temp2 (HPH^T) with R */
    float temp3[rowsH][colsHT];
    /* add R to HPH^T */
    Matrix.Add((mtx_type *)temp2, (mtx_type *)R, rowsH, colsHT, (mtx_type *)temp3);
    /* invert (HPH^T + R) */
    Matrix.Invert((mtx_type *)temp3, rowsH);
    /* multiply PH^T by (HPT^T + R)^1 and store in K */
    Matrix.Multiply((mtx_type *)temp1, (mtx_type *)temp3, rowsP, colsHT, colsHT, (mtx_type *)K);

    // TODO: manual calculation of K for arbitrary setup to verify the above has worked

    /* ---------- P = (I - KP)P ---------- */
    // TODO 
    /* ---------- P = FPF^T + Q ---------- */
    // TODO
    
  }

  
}

/* this function is used to filter the sensor data during ascent to help us predict apogee */
// TODO: rory make a document to summarise why we need this and what it is doing
void parachuteKalmanFilter()
{
}
