// Test TVLQR
// Scenerio: Drive bicycle to track references.

#include "bicycle_5d.h"
#include "data/lqr_ltv_data.h"
#include "simpletest.h"
#include "test_utils.h"
#include "tinympc/model.h"
#include "tinympc/lqr.h"
#include "tinympc/auxil.h"

#define H 0.1
#define NSTATES 5
#define NINPUTS 2
#define NHORIZON 51
// NO GRADIENT VANISHING/EXPLOSION WHEN NHORIZON = 91

sfloat x0_data[NSTATES] = {1, -1, 0, 0, 0};
sfloat xg_data[NSTATES] = {0};
sfloat ug_data[NINPUTS] = {0};
sfloat Q_data[NSTATES * NSTATES] = {0};
sfloat R_data[NINPUTS * NINPUTS] = {0};
sfloat Qf_data[NSTATES * NSTATES] = {0};
sfloat q_data[NSTATES*(NHORIZON-1)] = {0};
sfloat r_data[NINPUTS*(NHORIZON-1)] = {0};
sfloat qf_data[NSTATES] = {0};

Matrix X[NHORIZON];
Matrix U[NHORIZON - 1];
Matrix Xref[NHORIZON];
Matrix Uref[NHORIZON - 1];
Matrix Xnom[NHORIZON];
Matrix Unom[NHORIZON - 1];
Matrix K[NHORIZON - 1];
Matrix d[NHORIZON - 1];
Matrix P[NHORIZON];
Matrix p[NHORIZON];
Matrix A[NHORIZON - 1];
Matrix B[NHORIZON - 1];
Matrix f[NHORIZON - 1];
Matrix q[NHORIZON-1];
Matrix r[NHORIZON-1];

void DeltaLqrLtvTest() {
  sfloat X_data[NSTATES * NHORIZON] = {0};
  sfloat U_data[NINPUTS * (NHORIZON - 1)] = {0};
  sfloat K_data[NINPUTS * NSTATES * (NHORIZON - 1)] = {0};
  sfloat d_data[NINPUTS * (NHORIZON - 1)] = {0};
  sfloat P_data[NSTATES * NSTATES * (NHORIZON)] = {0};
  sfloat p_data[NSTATES * NHORIZON] = {0};
  sfloat A_data[NSTATES * NSTATES * (NHORIZON - 1)] = {0};
  sfloat B_data[NSTATES * NINPUTS * (NHORIZON - 1)] = {0};
  sfloat* Xptr = X_data;
  sfloat* Xref_ptr = Xref_data;
  sfloat* Uptr = U_data;
  sfloat* Uref_ptr = Uref_data;
  sfloat* Kptr = K_data;
  sfloat* dptr = d_data;
  sfloat* Pptr = P_data;
  sfloat* pptr = p_data;
  sfloat* Aptr = A_data;
  sfloat* Bptr = B_data;
  sfloat* qptr = q_data;
  sfloat* rptr = r_data;

  tiny_Model model;
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 1, 0, 0.1);  // feasible reference then
  // affine = 0;
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

  tiny_InitModelFromArray(&model, A, B, TINY_NULL, A_data, B_data, TINY_NULL);

  for (int i = 0; i < NHORIZON; ++i) {
    if (i < NHORIZON - 1) {
      A[i] = slap_MatrixFromArray(NSTATES, NSTATES, Aptr);
      Aptr += NSTATES * NSTATES;
      B[i] = slap_MatrixFromArray(NSTATES, NINPUTS, Bptr);
      Bptr += NSTATES * NINPUTS;
      U[i] = slap_MatrixFromArray(NINPUTS, 1, Uptr);
      // slap_SetConst(U[i], 0.01);
      Uptr += NINPUTS;
      Unom[i] = slap_MatrixFromArray(NINPUTS, 1, Uref_ptr);
      Uref_ptr += NINPUTS;
      Uref[i] = slap_MatrixFromArray(NINPUTS, 1, ug_data);
      K[i] = slap_MatrixFromArray(NINPUTS, NSTATES, Kptr);
      Kptr += NINPUTS * NSTATES;
      d[i] = slap_MatrixFromArray(NINPUTS, 1, dptr);
      dptr += NINPUTS;
      q[i] = slap_MatrixFromArray(NSTATES, 1, qptr);
      qptr += NSTATES;
      r[i] = slap_MatrixFromArray(NINPUTS, 1, rptr);
      rptr += NINPUTS;  
    }
    X[i] = slap_MatrixFromArray(NSTATES, 1, Xptr);
    Xptr += NSTATES;
    Xnom[i] = slap_MatrixFromArray(NSTATES, 1, Xref_ptr);
    Xref_ptr += NSTATES;
    Xref[i] = slap_MatrixFromArray(NSTATES, 1, xg_data);
    P[i] = slap_MatrixFromArray(NSTATES, NSTATES, Pptr);
    Pptr += NSTATES * NSTATES;
    p[i] = slap_MatrixFromArray(NSTATES, 1, pptr);
    pptr += NSTATES;
  }

  model.get_jacobians = tiny_Bicycle5dGetJacobians;  // from Bicycle
  soln.X = X;
  soln.U = U;
  soln.K = K;
  soln.d = d;
  soln.P = P;
  soln.p = p;
  soln.U = U;
  soln.X = X;
  data.x0 = slap_MatrixFromArray(NSTATES, 1, x0_data);  // check if possible  
  data.Xref = Xref;
  data.Uref = Uref;
  data.q = q;
  data.r = r;
  data.qf = slap_MatrixFromArray(NSTATES, 1, qf_data);  

  data.Q = slap_MatrixFromArray(NSTATES, NSTATES, Q_data);
  slap_SetIdentity(data.Q, 10e-1);
  data.R = slap_MatrixFromArray(NINPUTS, NINPUTS, R_data);
  slap_SetIdentity(data.R, 1e-1);
  data.Qf = slap_MatrixFromArray(NSTATES, NSTATES, Qf_data);
  slap_SetIdentity(data.Qf, 10e-1);

  // DELTA FORMULATION: Xref in objective and Xnom linearization, state is tracking
  // error and will go to zero.
  // Compute and store A, B offline
  tiny_UpdateModelJacAbout(&work, Xnom, Unom);

  tiny_UpdateLinearCost(&work);

  if (0) {
    printf("\nProblem Info: \n");
    PrintMatrix(work.data->model->A[0]);
    PrintMatrix(work.data->model->B[0]);
    // PrintMatrix(work.data->model->f[0]);
    PrintMatrix(work.data->Q);
    PrintMatrix(work.data->R);
    PrintMatrix(work.data->Qf);
    PrintMatrixT(work.data->x0);
    PrintMatrixT(work.data->Xref[NHORIZON-5]);
    PrintMatrixT(work.data->Uref[NHORIZON-5]);
    PrintMatrixT(work.data->q[NHORIZON-5]);
    PrintMatrixT(work.data->r[NHORIZON-5]);
  }

  tiny_SolveLqr(&work);

  if (0) {
    for (int k = 0; k < NHORIZON - 1; ++k) {
      printf("\n=>k = %d\n", k);
      printf("ex = %.4f\n", slap_NormTwo(X[k]));
      // PrintMatrix(p[k]);
      // PrintMatrixT(Xref[k]);
      // PrintMatrixT(U[k]);
      // PrintMatrixT(X[k]);
    }
    PrintMatrixT(X[NHORIZON - 1]);
  }

  for (int k = NHORIZON - 5; k < NHORIZON; ++k) {
    TEST(SumOfSquaredError(X[k].data, Xref[k].data, NSTATES) < 0.5);
  }
}

void AbsLqrLtvTest() {
  // Allocate all neccesarry memory
  sfloat X_data[NSTATES * NHORIZON] = {0};
  sfloat U_data[NINPUTS * (NHORIZON - 1)] = {0};
  sfloat K_data[NINPUTS * NSTATES * (NHORIZON - 1)] = {0};
  sfloat d_data[NINPUTS * (NHORIZON - 1)] = {0};
  sfloat P_data[NSTATES * NSTATES * (NHORIZON)] = {0};
  sfloat p_data[NSTATES * NHORIZON] = {0};
  sfloat A_data[NSTATES * NSTATES * (NHORIZON - 1)] = {0};
  sfloat B_data[NSTATES * NINPUTS * (NHORIZON - 1)] = {0};
  sfloat f_data[NSTATES * (NHORIZON - 1)] = {0};
  sfloat* Xptr = X_data;
  sfloat* Xref_ptr = Xref_data;
  sfloat* Uptr = U_data;
  sfloat* Uref_ptr = Uref_data;
  sfloat* Kptr = K_data;
  sfloat* dptr = d_data;
  sfloat* Pptr = P_data;
  sfloat* pptr = p_data;
  sfloat* Aptr = A_data;
  sfloat* Bptr = B_data;
  sfloat* fptr = f_data;
  sfloat* qptr = q_data;
  sfloat* rptr = r_data;

  for (int i = 0; i < NHORIZON; ++i) {
    if (i < NHORIZON - 1) {
      A[i] = slap_MatrixFromArray(NSTATES, NSTATES, Aptr);
      Aptr += NSTATES * NSTATES;
      B[i] = slap_MatrixFromArray(NSTATES, NINPUTS, Bptr);
      Bptr += NSTATES * NINPUTS;
      f[i] = slap_MatrixFromArray(NSTATES, 1, fptr);
      fptr += NSTATES;
      U[i] = slap_MatrixFromArray(NINPUTS, 1, Uptr);
      // slap_SetConst(U[i], 0.01);
      Uptr += NINPUTS;
      Uref[i] = slap_MatrixFromArray(NINPUTS, 1, Uref_ptr);
      Uref_ptr += NINPUTS;
      K[i] = slap_MatrixFromArray(NINPUTS, NSTATES, Kptr);
      Kptr += NINPUTS * NSTATES;
      d[i] = slap_MatrixFromArray(NINPUTS, 1, dptr);
      dptr += NINPUTS;
      q[i] = slap_MatrixFromArray(NSTATES, 1, qptr);
      qptr += NSTATES;
      r[i] = slap_MatrixFromArray(NINPUTS, 1, rptr);
      rptr += NINPUTS;        
    }
    X[i] = slap_MatrixFromArray(NSTATES, 1, Xptr);
    Xptr += NSTATES;
    Xref[i] = slap_MatrixFromArray(NSTATES, 1, Xref_ptr);
    Xref_ptr += NSTATES;
    P[i] = slap_MatrixFromArray(NSTATES, NSTATES, Pptr);
    Pptr += NSTATES * NSTATES;
    p[i] = slap_MatrixFromArray(NSTATES, 1, pptr);
    pptr += NSTATES;
  }

  // Create model and settings first due to essential problem setup
  tiny_Model model;
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 1, 1, 0.1);
  tiny_AdmmSettings stgs;
  tiny_InitSettings(&stgs);  //if switch on/off during run, initialize all
  tiny_SetUnconstrained(&stgs);

  // Create workspace
  tiny_AdmmData data;
  tiny_AdmmInfo info;
  tiny_AdmmSolution soln;
  tiny_AdmmWorkspace work;
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);

  sfloat temp_data[work.data_size];
  T_INIT_ZEROS(temp_data);
  tiny_InitWorkspaceTempData(&work, temp_data);

  // Now can fill in all the remaining struct
  tiny_InitModelFromArray(&model, A, B, f, A_data, B_data, f_data);
  model.get_jacobians = tiny_Bicycle5dGetJacobians;  // from Bicycle
  model.get_nonl_model = tiny_Bicycle5dNonlinearDynamics;

  tiny_InitSolutionFromMatrix(&work, X, U, P, p, K, d, TINY_NULL, TINY_NULL, TINY_NULL_MAT);
  data.x0 = slap_MatrixFromArray(NSTATES, 1, x0_data);  // check if possible  
  data.Xref = Xref;
  data.Uref = Uref;
  data.q = q;
  data.r = r;
  data.qf = slap_MatrixFromArray(NSTATES, 1, qf_data);  

  data.Q = slap_MatrixFromArray(NSTATES, NSTATES, Q_data);
  slap_SetIdentity(data.Q, 10e-1);
  data.R = slap_MatrixFromArray(NINPUTS, NINPUTS, R_data);
  slap_SetIdentity(data.R, 1e-1);
  data.Qf = slap_MatrixFromArray(NSTATES, NSTATES, Qf_data);
  slap_SetIdentity(data.Qf, 10e-1);

  // Absolute formulation
  // Compute and store A, B offline
  tiny_UpdateModelJac(&work);

  tiny_UpdateLinearCost(&work);

  if (0) {
    printf("\nProblem Info: \n");
    PrintMatrix(work.data->model->A[10]);
    PrintMatrix(work.data->model->B[10]);
    PrintMatrix(work.data->model->f[10]);
    PrintMatrix(work.data->Q);
    PrintMatrix(work.data->R);
    PrintMatrix(work.data->Qf);
    PrintMatrixT(work.data->x0);
    PrintMatrixT(work.data->Xref[NHORIZON-5]);
    PrintMatrixT(work.data->Uref[NHORIZON-5]);
    PrintMatrixT(work.data->q[NHORIZON-5]);
    PrintMatrixT(work.data->r[NHORIZON-5]);
  }

  tiny_SolveLqr(&work);

  if (0) {
    for (int k = 0; k < NHORIZON - 1; ++k) {
      printf("\n=>k = %d\n", k);
      printf("ex = %.4f\n", slap_NormedDifference(Xref[k], X[k]));
      // PrintMatrix(p[k]);
      // PrintMatrixT(Xref[k]);
      // PrintMatrixT(U[k]);
      // PrintMatrixT(X[k]);
    }
    PrintMatrixT(X[NHORIZON - 1]);
  }

  for (int k = NHORIZON - 5; k < NHORIZON; ++k) {
    TEST(SumOfSquaredError(X[k].data, Xref[k].data, NSTATES) < 0.5);
  }
}

int main() {
  printf("=== LQR LTV Tracking Test ===\n");
  DeltaLqrLtvTest();
  AbsLqrLtvTest();
  PrintTestResult();
  return TestResult();
}
