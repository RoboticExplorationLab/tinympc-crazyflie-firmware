// Test tracking LQR
// Scenerio: Drive sfloat integrator to track reference.

#include "data/lqr_lti_track_data.h"
#include "simpletest.h"
#include "slap/slap.h"
#include "test_utils.h"
#include "tinympc/lqr.h"
#include "tinympc/auxil.h"

#define NSTATES 4
#define NINPUTS 2
#define NHORIZON 51
// U, X, Psln
void LqrLtiTest() {
  sfloat A_data[NSTATES * NSTATES] = {1,   0, 0, 0, 0, 1,   0, 0,
                                      0.1, 0, 1, 0, 0, 0.1, 0, 1};
  sfloat B_data[NSTATES * NINPUTS] = {0.005, 0, 0.1, 0, 0, 0.005, 0, 0.1};
  sfloat f_data[NSTATES] = {0};
  sfloat x0_data[NSTATES] = {4, 7, 2, -1.0};
  sfloat X_data[NSTATES * NHORIZON] = {0};
  sfloat U_data[NINPUTS * (NHORIZON - 1)] = {0};
  sfloat Kinf_data[NINPUTS*NSTATES] = {
    2.483925f,0.000000f,
    0.000000f,2.483925f,
    3.337324f,0.000000f,
    0.000000f,3.337324f,
  };sfloat Pinf_data[NSTATES*NSTATES] = {
    134.356886f,0.000000f,33.541020f,0.000000f,
    0.000000f,134.356886f,0.000000f,33.541020f,
    33.541020f,0.000000f,48.387619f,0.000000f,
    0.000000f,33.541020f,0.000000f,48.387619f,
  };
  sfloat Quu_inv_data[NINPUTS*NINPUTS] = {
    0.616988f,0.000000f,
    0.000000f,0.616988f,
  };
  sfloat AmBKt_data[NSTATES*NSTATES] = {
    0.987580f,0.000000f,0.083313f,0.000000f,
    0.000000f,0.987580f,0.000000f,0.083313f,
    -0.248393f,0.000000f,0.666268f,0.000000f,
    0.000000f,-0.248393f,0.000000f,0.666268f,
  };
  sfloat coeff_d2p_data[NSTATES*NINPUTS] = {
    -0.000000f,0.000000f,-0.000000f,0.000000f,
    0.000000f,-0.000000f,0.000000f,-0.000000f,
  };
  sfloat d_data[NINPUTS * (NHORIZON - 1)] = {0};
  sfloat p_data[NSTATES * NHORIZON] = {0};
  sfloat Q_data[NSTATES * NSTATES] = {0};
  sfloat R_data[NINPUTS * NINPUTS] = {0};
  sfloat q_data[NSTATES*(NHORIZON-1)] = {0};
  sfloat r_data[NINPUTS*(NHORIZON-1)] = {0};
  sfloat r_tilde_data[NINPUTS*(NHORIZON-1)] = {0};

  Matrix A;
  Matrix B;
  Matrix f;
  Matrix X[NHORIZON];
  Matrix U[NHORIZON - 1];
  Matrix Xref[NHORIZON];
  Matrix Uref[NHORIZON - 1];
  Matrix d[NHORIZON - 1];
  Matrix p[NHORIZON];
  Matrix q[NHORIZON-1];
  Matrix r[NHORIZON-1];
  Matrix r_tilde[NHORIZON-1];

  tiny_Model model;
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, 0.1);
  // tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 1, 0.1);
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
  tiny_InitPrimalCache(&work, Quu_inv_data, AmBKt_data, coeff_d2p_data);
  
  tiny_InitModelFromArray(&model, &A, &B, &f, A_data, B_data, f_data);
  tiny_InitSolnTrajFromArray(&work, X, U, X_data, U_data);
  tiny_InitSolnGainsFromArray(&work, d, p, d_data, p_data, Kinf_data, Pinf_data);

  tiny_SetInitialState(&work, x0_data);  
  tiny_SetReference(&work, Xref, Uref, Xref_data, Uref_data);

  tiny_InitDataQuadCostFromArray(&work, Q_data, R_data);
  slap_SetIdentity(data.Q, 10);  
  slap_SetIdentity(data.R, 0.1);
  slap_AddIdentity(data.R, work.rho); // \tilde{R}
  tiny_InitDataLinearCostFromArray(&work, q, r, r_tilde, q_data, r_data, r_tilde_data);

  tiny_SetUnconstrained(&stgs);
  tiny_UpdateLinearCost(&work);

  if (0) {
    printf("\nProblem Info: \n");
    PrintMatrix(work.data->model->A[0]);
    PrintMatrix(work.data->model->B[0]);
    // PrintMatrix(work.data->model->f[0]);
    PrintMatrix(work.data->Q);
    PrintMatrix(work.data->R);
    PrintMatrixT(work.data->x0);
    PrintMatrixT(work.data->Xref[NHORIZON-1]);
    PrintMatrixT(work.data->Uref[NHORIZON-1]);
    PrintMatrixT(work.data->q[NHORIZON-5]);
    PrintMatrixT(work.data->r[NHORIZON-5]);
    PrintMatrixT(work.soln->Kinf);
  }
 
  tiny_SolveLqr(&work);

  if (0) {
    for (int k = 0; k < NHORIZON-1; ++k) {
      printf("\n=>k = %d\n", k);
      printf("  %f\n", slap_NormedDifference(Xref[k], X[k]));
      // PrintMatrix(p[k]);
      // PrintMatrixT(Xref[k]);
      // PrintMatrixT(U[k]);
      // PrintMatrixT(X[k]);
    }
    // PrintMatrixT(X[NHORIZON - 1]);
  }
  TEST(SumOfSquaredError(X[NHORIZON - 1].data, Xref[NHORIZON - 1].data, NSTATES) < 1e-1);
}

int main() {
  printf("=== LQR LTI Tracking Test ===\n");
  LqrLtiTest();
  PrintTestResult();
  return TestResult();
}