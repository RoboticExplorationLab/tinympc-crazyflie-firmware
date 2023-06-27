#include "bicycle_3d.h"

//========================================
// Codes generated from julia/bicycle_tvlqr
// Discrete dynamics of bicycle model with predefined model params
//========================================
void tiny_Bicycle3dNonlinearDynamics_Raw(sfloat* xn, const sfloat* x,
                                         const sfloat* u) {
  xn[0] =
      0.16666666666666666 * (0.4 * cos(0.05 * tan(u[1]) * u[0] + x[2]) * u[0] +
                             0.1 * cos(0.1 * tan(u[1]) * u[0] + x[2]) * u[0] +
                             0.1 * cos(x[2]) * u[0]) +
      x[0];
  xn[1] =
      0.16666666666666666 * (0.4 * sin(0.05 * tan(u[1]) * u[0] + x[2]) * u[0] +
                             0.1 * sin(0.1 * tan(u[1]) * u[0] + x[2]) * u[0] +
                             0.1 * sin(x[2]) * u[0]) +
      x[1];
  xn[2] = 0.1 * tan(u[1]) * u[0] + x[2];
}

void tiny_Bicycle3dNonlinearDynamics(Matrix* xn, const Matrix x,
                                     const Matrix u) {
  tiny_Bicycle3dNonlinearDynamics_Raw(xn->data, x.data, u.data);
}

//========================================
// Codes generated from julia/bicycle_tvlqr
// Jacobians of discrete dynamics of bicycle model with predefined model params
//========================================
void tiny_Bicycle3dGetJacobianA_Raw(sfloat* A, const sfloat* x,
                                    const sfloat* u) {
  A[0] = 1;
  A[1] = 0;
  A[2] = 0;
  A[3] = 0;
  A[4] = 1;
  A[5] = 0;
  A[6] =
      0.16666666666666666 * (-0.4 * sin(0.05 * tan(u[1]) * u[0] + x[2]) * u[0] +
                             -0.1 * sin(0.1 * tan(u[1]) * u[0] + x[2]) * u[0] +
                             -0.1 * sin(x[2]) * u[0]);
  A[7] =
      0.16666666666666666 * (0.4 * cos(0.05 * tan(u[1]) * u[0] + x[2]) * u[0] +
                             0.1 * cos(0.1 * tan(u[1]) * u[0] + x[2]) * u[0] +
                             0.1 * cos(x[2]) * u[0]);
  A[8] = 1;
}

void tiny_Bicycle3dGetJacobianB_Raw(sfloat* B, const sfloat* x,
                                    const sfloat* u) {
  B[0] = 0.16666666666666666 *
         (0.4 * cos(0.05 * tan(u[1]) * u[0] + x[2]) +
          0.1 * cos(0.1 * tan(u[1]) * u[0] + x[2]) + 0.1 * cos(x[2]) +
          -0.01 * sin(0.1 * tan(u[1]) * u[0] + x[2]) * tan(u[1]) * u[0] +
          -0.02 * sin(0.05 * tan(u[1]) * u[0] + x[2]) * tan(u[1]) * u[0]);
  B[1] = 0.16666666666666666 *
         (0.4 * sin(0.05 * tan(u[1]) * u[0] + x[2]) +
          0.1 * sin(0.1 * tan(u[1]) * u[0] + x[2]) + 0.1 * sin(x[2]) +
          0.02 * cos(0.05 * tan(u[1]) * u[0] + x[2]) * tan(u[1]) * u[0] +
          0.01 * cos(0.1 * tan(u[1]) * u[0] + x[2]) * tan(u[1]) * u[0]);
  B[2] = 0.1 * tan(u[1]);
  B[3] = 0.16666666666666666 * (-0.01 * (1 + pow(tan(u[1]), 2)) * pow(u[0], 2) *
                                    sin(0.1 * tan(u[1]) * u[0] + x[2]) +
                                -0.02 * (1 + pow(tan(u[1]), 2)) * pow(u[0], 2) *
                                    sin(0.05 * tan(u[1]) * u[0] + x[2]));
  B[4] = 0.16666666666666666 * (0.01 * (1 + pow(tan(u[1]), 2)) * pow(u[0], 2) *
                                    cos(0.1 * tan(u[1]) * u[0] + x[2]) +
                                0.02 * (1 + pow(tan(u[1]), 2)) * pow(u[0], 2) *
                                    cos(0.05 * tan(u[1]) * u[0] + x[2]));
  B[5] = 0.1 * (1 + pow(tan(u[1]), 2)) * u[0];
}

void tiny_Bicycle3dGetJacobians(Matrix* A, Matrix* B, const Matrix x,
                                const Matrix u) {
  tiny_Bicycle3dGetJacobianA_Raw(A->data, x.data, u.data);
  tiny_Bicycle3dGetJacobianB_Raw(B->data, x.data, u.data);
}