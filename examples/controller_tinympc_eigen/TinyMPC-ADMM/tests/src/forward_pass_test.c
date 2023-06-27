#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "data/forward_pass_data.h"
#include "simpletest.h"
#include "slap/slap.h"
#include "test_utils.h"
#include "tinympc/lqr.h"
#include "tinympc/auxil.h"

#define NSTATES 4
#define NINPUTS 2
#define NHORIZON 3

sfloat A_data[NSTATES * NSTATES] = {1,   0, 0, 0, 0, 1,   0, 0,
                                    0.1, 0, 1, 0, 0, 0.1, 0, 1};
sfloat B_data[NSTATES * NINPUTS] = {0.005, 0, 0.1, 0, 0, 0.005, 0, 0.1};
sfloat f_data[NSTATES] = {0, 0, 0, 0};
// sfloat x0_data[NSTATES] = {5,7,2,-1.4};
sfloat Q_data[NSTATES*NSTATES] = {0};
sfloat R_data[NINPUTS*NINPUTS] = {0};
sfloat q_data[NSTATES*(NHORIZON-1)] = {0};
sfloat r_data[NINPUTS*(NHORIZON-1)] = {0};

sfloat Xref_data[NSTATES] = {0};
sfloat Uref_data[NINPUTS] = {0};

  Matrix A;
  Matrix B;
  Matrix f;
  Matrix X[NHORIZON];
  Matrix Xsln[NHORIZON];
  Matrix U[NHORIZON - 1];
  Matrix Kinf;
  Matrix d[NHORIZON - 1];
  Matrix Xref[NHORIZON];
  Matrix Uref[NHORIZON-1];
  Matrix q[NHORIZON-1];
  Matrix r[NHORIZON-1];

void ForwardPassTest() {
  const sfloat tol = 1e-6;

  tiny_Model model;
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, 0.1);
  tiny_AdmmSettings stgs;
  tiny_InitSettings(&stgs);  //if switch on/off during run, initialize all
  tiny_AdmmData data;
  tiny_AdmmInfo info;
  tiny_AdmmSolution soln;
  tiny_AdmmWorkspace work;
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  
  sfloat temp_data[work.data_size];
  T_INIT_ZEROS(temp_data);

  tiny_InitWorkspaceTempData(&work, 0, 0, 0, 0, temp_data);
  tiny_InitModelFromArray(&model, &A, &B, &f, A_data, B_data, f_data);

  tiny_InitSolnTrajFromArray(&work, X, U, x_data, u_data);
  tiny_InitSolnGainsFromArray(&work, d, 0, d_data, 0, K_data, 0);

  sfloat* xsol_ptr = xsol_data;
  for (int i = 0; i < NHORIZON; ++i) {
    Xsln[i] = slap_MatrixFromArray(NSTATES, 1, xsol_ptr);
    xsol_ptr += NSTATES;
  }

  data.x0 = X[0];  // check if possible
  data.Xref = Xref;
  data.Uref = Uref;

  data.Q = slap_MatrixFromArray(NSTATES, NSTATES, Q_data);
  slap_SetIdentity(data.Q, 1);
  data.R = slap_MatrixFromArray(NINPUTS, NINPUTS, R_data);
  slap_SetIdentity(data.R, 1);
  data.q[0] = slap_MatrixFromArray(NSTATES, 1, q_data);
  data.q[1] = slap_MatrixFromArray(NSTATES, 1, &q_data[NSTATES]);
  data.r[0] = slap_MatrixFromArray(NINPUTS, 1, r_data);
  data.r[1] = slap_MatrixFromArray(NINPUTS, 1, &r_data[NINPUTS]);
  Xref[0] = slap_MatrixFromArray(NSTATES, 1, Xref_data);
  Xref[1] = slap_MatrixFromArray(NSTATES, 1, Xref_data);
  Xref[2] = slap_MatrixFromArray(NSTATES, 1, Xref_data);
  Uref[0] = slap_MatrixFromArray(NINPUTS, 1, Uref_data);
  Uref[1] = slap_MatrixFromArray(NINPUTS, 1, Uref_data);
  tiny_UpdateLinearCost(&work);

  uptr = u_data;
  xsol_ptr = xsol_data;
  xptr = x_data;
  Kptr = K_data;
  dptr = d_data;
  for (int i = 0; i < NHORIZON; ++i) {
    if (i < NHORIZON - 1) {
      TEST(U[i].rows == NINPUTS);
      TEST(U[i].cols == 1);
      TEST(SumOfSquaredError(U[i].data, uptr, NINPUTS) < tol);
      uptr += NINPUTS;
      TEST(soln.K[i].rows == NINPUTS);
      TEST(soln.K[i].cols == NSTATES);
      TEST(SumOfSquaredError(soln.K[i].data, Kptr, NINPUTS * NSTATES) < tol);
      Kptr += NINPUTS * NSTATES;
      TEST(soln.d[i].rows == NINPUTS);
      TEST(soln.d[i].cols == 1);
      TEST(SumOfSquaredError(soln.d[i].data, dptr, NINPUTS) < tol);
      dptr += NINPUTS;
    }
    TEST(X[i].rows == NSTATES);
    TEST(X[i].cols == 1);
    TEST(SumOfSquaredError(X[i].data, xptr, NSTATES) < tol);
    xptr += NSTATES;
    TEST(Xsln[i].rows == NSTATES);
    TEST(Xsln[i].cols == 1);
    TEST(SumOfSquaredError(Xsln[i].data, xsol_ptr, NSTATES) < tol);
    xsol_ptr += NSTATES;
  }

  // Include discrete dynamics test
  tiny_ForwardPass(&work);
  for (int i = 0; i < NHORIZON; ++i) {
    TEST(SumOfSquaredError(soln.X[i].data, Xsln[i].data, NSTATES) < tol);
  }
  //FIXME: cost is not exact!!
  // printf("%f\n", info.obj_val);
  // PrintMatrix(soln.U[0]);
  // PrintMatrix(soln.U[1]);
  // PrintMatrix(soln.X[0]);PrintMatrix(soln.X[1]);PrintMatrix(soln.X[2]);
}

int main() {
  printf("=== Forward Pass Test ===\n");
  ForwardPassTest();
  PrintTestResult();
  return TestResult();
}
