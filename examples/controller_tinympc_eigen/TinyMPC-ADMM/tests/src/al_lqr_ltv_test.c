// Test AL-TVLQR
// Scenerio: Drive bicycle to track references with constraints.

// === BETTER TURN OFF GOAL_CONSTRAINT IN PROJECT CMAKELISTS.TXT TO PASS ===
// IF BOX CONSTRAINTS OFF, CAN HANDLE GOAL CONSTRAINT
// IF BOX CONSTRAINTS ON, UNLIKELY TO HANDLE GOAL CONSTRAINT
// NO GRADIENT VANISHING/EXPLOSION WHEN NHORIZON = 65 (MORE MAY FAIL)
// GREATER NHORIZON, GREATER ITERATION, GREATER CHANCE OF EXPLOSION
// TODO: Let user choose constraints, compile options with #IFDEF

#include "bicycle_5d.h"
#include "data/lqr_ltv_data.h"
#include "simpletest.h"
#include "slap/slap.h"
#include "test_utils.h"
#include "tinympc/al_lqr.h"
#include "tinympc/auxil.h"

#define H 0.1
#define NSTATES 5
#define NINPUTS 2
#define NHORIZON 41

void AbsLqrLtvTest() {
  sfloat x0_data[NSTATES] = {1, -1, 0, 0, 0};
  // sfloat xg_data[NSTATES] = {0};
  // sfloat ug_data[NINPUTS] = {0};
  sfloat Q_data[NSTATES * NSTATES] = {0};
  sfloat R_data[NINPUTS * NINPUTS] = {0};
  sfloat Qf_data[NSTATES * NSTATES] = {0};
  sfloat X_data[NSTATES * NHORIZON] = {0};
  sfloat U_data[NINPUTS * (NHORIZON - 1)] = {0};
  sfloat K_data[NINPUTS * NSTATES * (NHORIZON - 1)] = {0};
  sfloat d_data[NINPUTS * (NHORIZON - 1)] = {0};
  sfloat P_data[NSTATES * NSTATES * (NHORIZON)] = {0};
  sfloat p_data[NSTATES * NHORIZON] = {0};
  sfloat A_data[NSTATES * NSTATES * (NHORIZON - 1)] = {0};
  sfloat B_data[NSTATES * NINPUTS * (NHORIZON - 1)] = {0};
  sfloat f_data[NSTATES * (NHORIZON - 1)] = {0};
  sfloat input_dual_data[2 * NINPUTS * (NHORIZON - 1)] = {0};
  sfloat state_dual_data[2 * NSTATES * (NHORIZON)] = {0};
  sfloat goal_dual_data[NSTATES] = {0};
  sfloat q_data[NSTATES*(NHORIZON-1)] = {0};
  sfloat r_data[NINPUTS*(NHORIZON-1)] = {0};
  sfloat qf_data[NSTATES] = {0};  
  // sfloat umin_data[NINPUTS] = {-5, -2};
  // sfloat umax_data[NINPUTS] = {5, 2};
  // sfloat xmin_data[NSTATES] = {-100, -100, -100, -100, -100};
  // sfloat xmax_data[NSTATES] = {100, 100, 100, 100, 100};

  // Put constraints on u, x4, x5
  sfloat Acstr_input_data[2 * NINPUTS * NINPUTS] = {0};
  sfloat Acstr_state_data[2 * NSTATES * NSTATES] = {0};
  // [u_max, -u_min]
  sfloat bcstr_input_data[2 * NINPUTS] = {2.0, 0.9, 2.0, 0.9};
  // [x_max, -x_min]
  sfloat bcstr_state_data[2 * NSTATES] = {100, 100, 100, 4.0, 0.55,
                                          100, 100, 100, 4.0, 0.55};
                                          
  Matrix X[NHORIZON];
  Matrix U[NHORIZON - 1];
  Matrix Xref[NHORIZON];
  Matrix Uref[NHORIZON - 1];
  Matrix K[NHORIZON - 1];
  Matrix d[NHORIZON - 1];
  Matrix P[NHORIZON];
  Matrix p[NHORIZON];
  Matrix A[NHORIZON - 1];
  Matrix B[NHORIZON - 1];
  Matrix f[NHORIZON - 1];
  Matrix YU[NHORIZON - 1];
  Matrix YX[NHORIZON];
  Matrix q[NHORIZON-1];
  Matrix r[NHORIZON-1];

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
  sfloat* udual_ptr = input_dual_data;
  sfloat* xdual_ptr = state_dual_data;
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
      YU[i] = slap_MatrixFromArray(2 * NINPUTS, 1, udual_ptr);
      udual_ptr += 2 * NINPUTS;
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
    YX[i] = slap_MatrixFromArray(2 * NSTATES, 1, xdual_ptr);
    xdual_ptr += 2 * NSTATES;
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

  tiny_InitSolutionFromMatrix(&work, X, U, P, p, K, d, YX, YU, TINY_NULL_MAT);
  soln.YG = slap_MatrixFromArray(NSTATES, 1, goal_dual_data);
  data.x0 = slap_MatrixFromArray(NSTATES, 1, x0_data);  // check if possible  
  data.Xref = Xref;
  data.Uref = Uref;
  data.q = q;
  data.r = r;
  data.qf = slap_MatrixFromArray(NSTATES, 1, qf_data);  

  data.Q = slap_MatrixFromArray(NSTATES, NSTATES, Q_data);
  slap_SetIdentity(data.Q, 1e-1);
  data.R = slap_MatrixFromArray(NINPUTS, NINPUTS, R_data);
  slap_SetIdentity(data.R, 1e-1);
  data.Qf = slap_MatrixFromArray(NSTATES, NSTATES, Qf_data);
  slap_SetIdentity(data.Qf, 10e-1);

  //TODO: API set bound
  data.Acx = slap_MatrixFromArray(2 * NSTATES, NSTATES, Acstr_state_data);
  Matrix upper_half = slap_CreateSubMatrix(data.Acx, 0, 0, NSTATES, NSTATES);
  Matrix lower_half = slap_CreateSubMatrix(data.Acx, NSTATES, 0, NSTATES, NSTATES);
  slap_SetIdentity(upper_half, 1);
  slap_SetIdentity(lower_half, -1);
  data.Acu = slap_MatrixFromArray(2 * NINPUTS, NINPUTS, Acstr_input_data);
  upper_half = slap_CreateSubMatrix(data.Acu, 0, 0, NINPUTS, NINPUTS);
  lower_half = slap_CreateSubMatrix(data.Acu, NINPUTS, 0, NINPUTS, NINPUTS);
  slap_SetIdentity(upper_half, 1);
  slap_SetIdentity(lower_half, -1);
  data.bcx = slap_MatrixFromArray(2 * NSTATES, 1, bcstr_state_data);
  data.bcu = slap_MatrixFromArray(2 * NINPUTS, 1, bcstr_input_data);
  
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

  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 1;
  stgs.max_iter_riccati = 1;
  stgs.max_iter = 6;
  stgs.verbose = 0;
  stgs.reg_min = 1e-6;

  tiny_SolveAlLqr(&work);

  if (0) {
    for (int k = 0; k < NHORIZON - 1; ++k) {
      printf("\n=>k = %d\n", k);
      printf("ex = %.4f\n", slap_NormedDifference(Xref[k], X[k]));
      // PrintMatrix(p[k]);
      // PrintMatrixT(Xref[k]);
      PrintMatrixT(U[k]);
      // PrintMatrixT(X[k]);
    }
    PrintMatrixT(X[NHORIZON - 1]);
  }

  // ========== Test ==========
  for (int k = 0; k < NHORIZON - 1; ++k) {
    // PrintMatrix(U[k]);
    for (int i = 0; i < NSTATES; ++i) {
      TEST(X[k].data[i] < bcstr_state_data[i] + stgs.tol_abs_dual);
      TEST(X[k].data[i] > -bcstr_state_data[i] - stgs.tol_abs_dual);
    }
    for (int i = 0; i < NINPUTS; ++i) {
      TEST(U[k].data[i] > -bcstr_input_data[i] - stgs.tol_abs_dual);
      TEST(U[k].data[i] < bcstr_input_data[i] + stgs.tol_abs_dual);
    }
  }
  for (int k = NHORIZON - 5; k < NHORIZON; ++k) {
    TEST(SumOfSquaredError(X[k].data, Xref[k].data, NSTATES) < 0.1);
  }
  // --------------------------
}

int main() {
  printf("=== AL LQR LTV Test ===\n");
  AbsLqrLtvTest();
  PrintTestResult();
  return TestResult();
}
