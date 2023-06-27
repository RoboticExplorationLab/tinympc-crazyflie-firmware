// MPC
// Scenerio: Drive bicycle to track references with constraints.
// Check trajopt at test/al_lqr_test/al_lqr_ltv_test.c

// === BETTER TURN OFF GOAL_CONSTRAINT IN PROJECT CMAKELISTS.TXT TO PASS ===
// IF BOX CONSTRAINTS OFF, CAN HANDLE GOAL CONSTRAINT
// IF BOX CONSTRAINTS ON, UNLIKELY TO HANDLE GOAL CONSTRAINT
// DON"T WORRY ABOUT GRADIENT VANISHING/EXPLOSION SINCE SMALL MPC HORIZON
// GREATER NHORIZON, GREATER ITERATION, GREATER CHANCE OF EXPLOSION
// TODO: Let user choose constraints, compile options with #IFDEF

#include <stdlib.h>

#include "bicycle_5d.h"
#include "data/lqr_ltv_data.h"
#include "simpletest.h"
#include "slap/slap.h"
#include "tinympc/tinympc.h"

#define H 0.1        // dt
#define NSTATES 5    // no. of states
#define NINPUTS 2    // no. of controls
#define NHORIZON 10  // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define NSIM 101     // simulation steps (fixed with reference data)

int main() {
  // ===== Created data =====
  sfloat x0_data[NSTATES] = {1, -1, 0, 0, 0};  // initial state
  // sfloat xg_data[NSTATES] = {0};  // goal state if needed
  // sfloat ug_data[NINPUTS] = {0};   // goal input if needed
  sfloat Xhrz_data[NSTATES * NHORIZON] = {0};  // save X for one horizon
  sfloat X_data[NSTATES * NSIM] = {0};         // save X for the whole run
  sfloat Uhrz_data[NINPUTS * (NHORIZON - 1)] = {0};
  sfloat K_data[NINPUTS * NSTATES * (NHORIZON - 1)] = {0};  // feedback gain
  sfloat d_data[NINPUTS * (NHORIZON - 1)] = {0};            // feedforward gain
  sfloat P_data[NSTATES * NSTATES * (NHORIZON)] = {0};      // cost-to-go func
  sfloat p_data[NSTATES * NHORIZON] = {0};                  // cost-to-go func
  sfloat A_data[NSTATES * NSTATES * (NHORIZON - 1)] = {0};  // A in model
  sfloat B_data[NSTATES * NINPUTS * (NHORIZON - 1)] = {0};  // B in model
  sfloat f_data[NSTATES * (NHORIZON - 1)] = {0};            // f in model
  sfloat YU_data[2 * NINPUTS * (NHORIZON - 1)] = {0};  // dual vars
  sfloat YX_data[2 * NSTATES * (NHORIZON)] = {0};      // dual vars
  sfloat Q_data[NSTATES * NSTATES] = {0};   // Q matrix in obj
  sfloat R_data[NINPUTS * NINPUTS] = {0};   // R matrix in obj
  sfloat Qf_data[NSTATES * NSTATES] = {0};  // Qf matrix in obj
  sfloat q_data[NSTATES*(NHORIZON-1)] = {0};
  sfloat r_data[NINPUTS*(NHORIZON-1)] = {0};
  sfloat qf_data[NSTATES] = {0};  

  // Put constraints on u, x4, x5
  sfloat Acu_data[2 * NINPUTS * NINPUTS] = {0};  // A1*u <= b1
  sfloat Acx_data[2 * NSTATES * NSTATES] = {0};  // A2*x <= b2
  // [u_max, -u_min]
  sfloat bcu_data[2 * NINPUTS] = {2.0, 0.9, 2.0, 0.9};
  // [x_max, -x_min]
  sfloat bcx_data[2 * NSTATES] = {100, 100, 100, 4.2, 0.8,
                                  100, 100, 100, 4.2, 0.8};

  // ===== Created matrices =====
  Matrix X[NSIM];
  Matrix Xref[NSIM];
  Matrix Uref[NSIM - 1];
  Matrix Xhrz[NHORIZON];
  Matrix Uhrz[NHORIZON - 1];
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

  for (int i = 0; i < NSIM; ++i) {
    if (i < NSIM - 1) {
      Uref[i] = slap_MatrixFromArray(NINPUTS, 1, &Uref_data[i * NINPUTS]);
    }
    X[i] = slap_MatrixFromArray(NSTATES, 1, &X_data[i * NSTATES]);
    Xref[i] = slap_MatrixFromArray(NSTATES, 1, &Xref_data[i * NSTATES]);
    // PrintMatrix(Xref[i]);
  }

  // Create model and settings first due to essential problem setup
  tiny_Model model;
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 1, 1, 0.1);
  tiny_AdmmSettings stgs;
  tiny_InitSettings(&stgs);  //if switch on/off during run, initialize all

  // Create workspace
  tiny_AdmmData data;
  tiny_AdmmInfo info;
  tiny_AdmmSolution soln;
  tiny_AdmmWorkspace work;
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);

  // Now can fill in all the remaining struct
  sfloat temp_data[work.data_size];
  T_INIT_ZEROS(temp_data);
  tiny_InitWorkspaceTempData(&work, temp_data);

  tiny_InitModelFromArray(&model, A, B, f, A_data, B_data, f_data);
  model.get_jacobians = tiny_Bicycle5dGetJacobians;  // from Bicycle
  model.get_nonl_model = tiny_Bicycle5dNonlinearDynamics;

  tiny_InitSolnTrajFromArray(&work, Xhrz, Uhrz, Xhrz_data, Uhrz_data);
  tiny_InitSolnDualsFromArray(&work, YX, YU, YX_data, YU_data, TINY_NULL);
  tiny_InitSolnGainsFromArray(&work, K, d, P, p, K_data, d_data, P_data, p_data);

  data.x0 = slap_MatrixFromArray(NSTATES, 1, x0_data);  
  data.Xref = Xref;
  data.Uref = Uref;
  tiny_InitDataQuadCostFromArray(&work, Q_data, R_data, Qf_data);
  slap_SetIdentity(data.Q, 10e-1);  
  slap_SetIdentity(data.R, 1e-1);  
  slap_SetIdentity(data.Qf, 10e-1);
  tiny_InitDataLinearCostFromArray(&work, q, r, q_data, r_data, qf_data);

  tiny_SetInputBound(&work, Acu_data, bcu_data);
  tiny_SetStateBound(&work, Acx_data, bcx_data);

  // Compute and store A, B offline
  tiny_UpdateModelJac(&work);

  tiny_UpdateLinearCost(&work);

  tiny_WarmStartInput(&work, Uref_data);

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
  stgs.max_iter_riccati = 2;
  stgs.max_iter = 6;
  stgs.verbose = 0;
  stgs.reg_min = 1e-6;

  // ===== Absolute formulation =====
  // Warm-starting since horizon data is reused
  // At each time step (stop earlier as horizon exceeds the end)
  srand(1);  // random seed
  slap_Copy(X[0], work.data->x0);  
  for (int k = 0; k < NSIM - NHORIZON - 1; ++k) {
    printf("\n=> k = %d\n", k);
    printf("ex[%d] = %.4f\n", k, slap_NormedDifference(X[k], Xref[k]));

    // === 1. Setup and solve MPC ===
    X[k].data[0] += X[k].data[0] * T_NOISE(1);  // noise 1% of current X
    X[k].data[1] += X[k].data[1] * T_NOISE(1);
    X[k].data[2] += X[k].data[2] * T_NOISE(1);
    X[k].data[3] += X[k].data[3] * T_NOISE(1);
    X[k].data[4] += X[k].data[4] * T_NOISE(1);
    slap_Copy(work.data->x0, X[k]);  // update current measurement

    // Update reference
    data.Xref = &Xref[k];
    data.Uref = &Uref[k];
    tiny_UpdateLinearCost(&work);

    // Update A, B within horizon (as we have Jacobians function)
    tiny_UpdateModelJac(&work);

    // Solve optimization problem using Augmented Lagrangian TVLQR
    tiny_SolveAlLqr(&work);
    // PrintSolveInfo(&work);
    if(work.info->status_val != TINY_SOLVED) {
      printf("!!! NOT SOLVED !!!\n");
      return 0;
    }
   // Test control constraints here (since we didn't save U)
    for (int i = 0; i < NINPUTS; ++i) {
      TEST(Uhrz[0].data[i] < bcu_data[i] + stgs.tol_abs_dual);
      TEST(Uhrz[0].data[i] > -bcu_data[i] - stgs.tol_abs_dual);
    }

    // === 2. Simulate dynamics using the first control solution ===
    tiny_Bicycle5dNonlinearDynamics(&X[k + 1], X[k], Uhrz[0]);
  }

  // ========== Test ==========
  // Test state constraints
  for (int k = 0; k < NSIM - NHORIZON - 1; ++k) {
    for (int i = 0; i < NSTATES; ++i) {
      TEST(X[k].data[i] < bcx_data[i] + stgs.tol_abs_dual);
      TEST(X[k].data[i] > -bcx_data[i] - stgs.tol_abs_dual);
    }
  }
  // Test tracking performance
  for (int k = NSIM - NHORIZON - 5; k < NSIM - NHORIZON; ++k) {
    TEST(slap_NormedDifference(X[k], Xref[k]) < 0.5);
  }
  // --------------------------

  PrintTestResult();
  return TestResult();
}
