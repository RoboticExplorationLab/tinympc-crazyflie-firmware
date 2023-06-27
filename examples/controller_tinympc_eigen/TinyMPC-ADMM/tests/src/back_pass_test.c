#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "data/back_pass_data.h"
#include "simpletest.h"
#include "slap/slap.h"
#include "test_utils.h"
#include "tinympc/lqr.h"
#include "tinympc/auxil.h"

#define NSTATES 4
#define NINPUTS 2
#define NHORIZON 3
// U, X, Psln
void BackPassTest() {
  sfloat A_data[NSTATES * NSTATES] = {1,   0, 0, 0, 0, 1,   0, 0,
                                      0.1, 0, 1, 0, 0, 0.1, 0, 1};
  sfloat B_data[NSTATES * NINPUTS] = {0.005, 0, 0.1, 0, 0, 0.005, 0, 0.1};
  sfloat f_data[NSTATES] = {0};
  sfloat x0_data[NSTATES] = {5, 7, 2, -1.4};
  sfloat Xref_data[NSTATES * NHORIZON] = {0};
  sfloat Uref_data[NINPUTS * (NHORIZON - 1)] = {0};
  // sfloat X_data[NSTATES*NHORIZON] = {0};
  // sfloat U_data[NINPUTS*(NHORIZON-1)] = {0};
  sfloat K_data[NINPUTS * NSTATES * (NHORIZON - 1)] = {0};
  sfloat d_data[NINPUTS * (NHORIZON - 1)] = {0};
  sfloat P_data[NSTATES * NSTATES * (NHORIZON)] = {0};
  sfloat p_data[NSTATES * NHORIZON] = {0};
  sfloat Q_data[NSTATES * NSTATES] = {0};
  sfloat R_data[NINPUTS * NINPUTS] = {0};
  sfloat Qf_data[NSTATES * NSTATES] = {0};
  sfloat q_data[NSTATES*(NHORIZON-1)] = {0};
  sfloat r_data[NINPUTS*(NHORIZON-1)] = {0};
  sfloat qf_data[NSTATES] = {0};

  // const sfloat tol = 1e-6;

  tiny_Model model;
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 1, 0.1);
  tiny_AdmmSettings stgs;
  tiny_InitSettings(&stgs);  //if switch on/off during run, initialize all
  tiny_AdmmData data;
  tiny_AdmmInfo info;
  tiny_AdmmSolution soln;
  tiny_AdmmWorkspace work;
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  
  sfloat temp_data[work.data_size];
  T_INIT_ZEROS(temp_data);

  tiny_InitWorkspaceTempData(&work, temp_data);

  Matrix A;
  Matrix B;
  Matrix f;
  tiny_InitModelFromArray(&model, &A, &B, &f, A_data, B_data, f_data);

  Matrix Xref[NHORIZON];
  Matrix Uref[NHORIZON - 1];
  Matrix K[NHORIZON - 1];
  Matrix d[NHORIZON - 1];
  Matrix P[NHORIZON];
  Matrix p[NHORIZON];
  Matrix q[NHORIZON-1];
  Matrix r[NHORIZON-1];

  sfloat* Xref_ptr = Xref_data;
  sfloat* Uref_ptr = Uref_data;
  sfloat* Kptr = K_data;
  sfloat* dptr = d_data;
  sfloat* Pptr = P_data;
  sfloat* pptr = p_data;

  for (int i = 0; i < NHORIZON; ++i) {
    if (i < NHORIZON - 1) {
      Uref[i] = slap_MatrixFromArray(NINPUTS, 1, Uref_ptr);
      Uref_ptr += NINPUTS;
      K[i] = slap_MatrixFromArray(NINPUTS, NSTATES, Kptr);
      Kptr += NINPUTS * NSTATES;
      d[i] = slap_MatrixFromArray(NINPUTS, 1, dptr);
      dptr += NINPUTS;
    }
    Xref[i] = slap_MatrixFromArray(NSTATES, 1, Xref_ptr);
    Xref_ptr += NSTATES;
    P[i] = slap_MatrixFromArray(NSTATES, NSTATES, Pptr);
    Pptr += NSTATES * NSTATES;
    p[i] = slap_MatrixFromArray(NSTATES, 1, pptr);
    pptr += NSTATES;
  }

  data.Q = slap_MatrixFromArray(NSTATES, NSTATES, Q_data);
  slap_SetIdentity(data.Q, 1e-1);
  data.R = slap_MatrixFromArray(NINPUTS, NINPUTS, R_data);
  slap_SetIdentity(data.R, 1e-1);
  data.Qf = slap_MatrixFromArray(NSTATES, NSTATES, Qf_data);
  slap_SetIdentity(data.Qf, 100 * 1e-1);
  data.Xref = Xref;
  data.Uref = Uref;
  data.x0 = slap_MatrixFromArray(NSTATES, 1, x0_data);
  data.q = q;
  data.r = r;
  data.q[0] = slap_MatrixFromArray(NSTATES, 1, q_data);
  data.q[1] = slap_MatrixFromArray(NSTATES, 1, &q_data[NSTATES]);
  data.r[0] = slap_MatrixFromArray(NINPUTS, 1, r_data);
  data.r[1] = slap_MatrixFromArray(NINPUTS, 1, &r_data[NINPUTS]);
  data.qf = slap_MatrixFromArray(NSTATES, 1, qf_data);   
  soln.K = K;
  soln.d = d;
  soln.P = P;
  soln.p = p; 
  if (0) {
    printf("\nProblem Info: \n");
    PrintMatrix(work.data->model->A[0]);
    PrintMatrix(work.data->model->B[0]);
    PrintMatrix(work.data->model->f[0]);
    PrintMatrix(work.data->Q);
    PrintMatrix(work.data->R);
    PrintMatrix(work.data->Qf);
  }
  tiny_UpdateLinearCost(&work);
  tiny_BackwardPass(&work);
  if (1) {
    for (int k = 0; k < NHORIZON - 1; ++k) {
      PrintMatrix(work.soln->d[k]);
    }
  }
  // TEST(SumOfSquaredError(d_data, dsln_data, (NHORIZON - 1) * NINPUTS) < tol);
}

int main() {
  printf("=== Backward Pass Test ===\n");
  BackPassTest();
  PrintTestResult();
  return TestResult();
}
