#include "test_utils.h"

// This will not work with sub-matrices
sfloat SumOfSquaredError(const sfloat* x, const sfloat* y, const int len) {
  sfloat err = 0;
  for (int i = 0; i < len; ++i) {
    sfloat diff = x[i] - y[i];
    err += diff * diff;
  }
  return sqrt(err);
}

sfloat SumOfSquaredErrorMatrices(const sfloat* x, Matrix* Y, const int num) {
  sfloat err = 0;
  int k = 0;
  for (int i = 0; i < num; ++i) {
    for (int j = 0; j < Y[i].cols*Y[i].rows; ++j) {
      sfloat diff = x[k++] - Y[i].data[j];
      err += diff * diff;
    }
  }
  return sqrt(err);
}