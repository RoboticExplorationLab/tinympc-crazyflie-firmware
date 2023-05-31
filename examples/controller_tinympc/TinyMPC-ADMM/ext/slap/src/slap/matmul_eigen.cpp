//
// Created by Sam Schoedel on 5/2/23.
// Copyright (c) 2022 Robotic Exploration Lab. All rights reserved.
//

#include <Eigen/Dense>

using Eigen::Dynamic;
using Eigen::Map;

#ifdef __cplusplus
extern "C" {
#endif

#include "matmul.h"
#include "tri.h"
#include "copy_matrix.h"

enum slap_ErrorCode slap_MatMulAdd(
    Matrix C, Matrix A, Matrix B, sfloat alpha,
    sfloat beta) {

  int n = slap_NumRows(A);
  int m = slap_NumCols(A);
  int p = slap_NumCols(B);

  Map<Eigen::Matrix<sfloat, Dynamic, Dynamic>>(C.data, m, p) = 
      beta * Map<Eigen::Matrix<sfloat, Dynamic, Dynamic>>(C.data, m, p)
      + alpha * Map<Eigen::Matrix<sfloat, Dynamic, Dynamic>>(A.data, m, n) 
      * Map<Eigen::Matrix<sfloat, Dynamic, Dynamic>>(B.data, n, p);

  return SLAP_NO_ERROR;
}

#ifdef __cplusplus
} // extern "C"
#endif