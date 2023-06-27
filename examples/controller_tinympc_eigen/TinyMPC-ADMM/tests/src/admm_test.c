#include "simpletest.h"
#include "slap/slap.h"
#include "test_utils.h"
#include "tinympc/admm.h"
#include "tinympc/auxil.h"
#include "tinympc/constraint_linear.h"

#define NSTATES 4
#define NINPUTS 2
#define NHORIZON 3

sfloat A_data[NSTATES * NSTATES] = {1,   0, 0, 0, 0, 1,   0, 0,
                                    0.1, 0, 1, 0, 0, 0.1, 0, 1};
sfloat B_data[NSTATES * NINPUTS] = {0.005, 0, 0.1, 0, 0, 0.005, 0, 0.1};
sfloat f_data[NSTATES] = {0};
// sfloat x0_data[NSTATES] = {5, 7, 2, -1.4};
sfloat x0_data[NSTATES] = {1, 0, 0, 1.0};  
// sfloat xg_data[NSTATES] = {2, 5, 0, 0};
sfloat xg_data[NSTATES] = {0};
sfloat ug_data[NINPUTS] = {0};
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

sfloat umin_data[NINPUTS] = {-1, -1};
sfloat umax_data[NINPUTS] = {1, 1};
sfloat xmin_data[NSTATES] = {-2, -2, -2, -2};
sfloat xmax_data[NSTATES] = {6, 8, 3, 2};
// Put constraints on u, x
sfloat Acu_data[NINPUTS * NINPUTS] = {0};  
sfloat Acx_data[NSTATES * NSTATES] = {0};  
sfloat YU_data[NINPUTS * (NHORIZON - 1)] = {0};
sfloat YX_data[NSTATES * (NHORIZON)] = {0};
sfloat YG_data[NSTATES] = {0};

Matrix X[NHORIZON];
Matrix U[NHORIZON - 1];
Matrix Xref[NHORIZON];
Matrix Uref[NHORIZON - 1];
Matrix K[NHORIZON - 1];
Matrix d[NHORIZON - 1];
Matrix P[NHORIZON];
Matrix p[NHORIZON];
Matrix YU[NHORIZON - 1];
Matrix YX[NHORIZON];
Matrix ZU[NHORIZON - 1];
Matrix ZU_new[NHORIZON - 1];
Matrix q[NHORIZON-1];
Matrix r[NHORIZON-1];
Matrix r_tilde[NHORIZON-1];
Matrix A;
Matrix B;
Matrix f;

void CheckTerminationTest() {
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

  tiny_InitWorkspaceTempData(&work, ZU, ZU_new, 0, 0, temp_data);
  tiny_InitPrimalCache(&work, Quu_inv_data, AmBKt_data, coeff_d2p_data);
  
  tiny_InitModelFromArray(&model, &A, &B, &f, A_data, B_data, f_data);
  tiny_InitSolnTrajFromArray(&work, X, U, X_data, U_data);
  tiny_InitSolnGainsFromArray(&work, d, p, d_data, p_data, Kinf_data, Pinf_data);
  tiny_InitSolnDualsFromArray(&work, 0, YU, 0, YU_data, 0);

  tiny_SetInitialState(&work, x0_data);  
  tiny_SetGoalReference(&work, Xref, Uref, xg_data, ug_data);

  tiny_InitDataQuadCostFromArray(&work, Q_data, R_data);
  slap_SetIdentity(data.Q, 10);  
  slap_SetIdentity(data.R, 0.1);
  slap_AddIdentity(data.R, work.rho); // \tilde{R}
  tiny_InitDataLinearCostFromArray(&work, q, r, r_tilde, q_data, r_data, r_tilde_data);

  tiny_SetInputBound(&work, Acu_data, umin_data, umax_data);

  if (0) {
    printf("\nProblem Info: \n");
    PrintMatrix(work.data->model->A[0]);
    PrintMatrix(work.data->model->B[0]);
    // PrintMatrix(work.data->model->f[0]);
    PrintMatrix(work.data->Q);
    PrintMatrix(work.data->R);
    PrintMatrixT(work.data->x0);
    PrintMatrixT(work.data->Xref[NHORIZON-5]);
    PrintMatrixT(work.data->Uref[NHORIZON-5]);
    PrintMatrixT(work.data->q[NHORIZON-5]);
    PrintMatrixT(work.data->r[NHORIZON-5]);
    PrintMatrixT(work.soln->Kinf);
  }

  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 0;
  stgs.max_iter = 1000;
  stgs.verbose = 1;
  stgs.check_termination = 10;
  stgs.rho_init = 1e0;
  stgs.tol_abs_dual = 100;
  stgs.tol_abs_prim = 100;

  soln.U[0].data = (sfloat[]){1, 2};
  soln.YU[0].data = (sfloat[]){-0.5, 0.5};
  work.ZU[0].data = (sfloat[]){0, 1};
  work.ZU_new[0].data = (sfloat[]){0, 2};
  soln.U[1].data = (sfloat[]){0, 3};
  soln.YU[1].data = (sfloat[]){-5, -3.5};
  work.ZU[1].data = (sfloat[]){20, 1};
  work.ZU_new[1].data = (sfloat[]){10, 9};

  ComputePrimalResidual(&work);
  TESTAPPROX(info.pri_res, 10, 1e-6);

  ComputeDualResidual(&work);
  TESTAPPROX(info.dua_res, 10, 1e-6);

  CheckTermination(&work); 
  TEST(work.info->status_val == TINY_SOLVED);

  UpdateSlackDual(&work);
  // PrintMatrix(work.ZU_new[0]);
  // PrintMatrix(work.ZU_new[1]);
  // PrintMatrix(work.soln->YU[0]);
  // PrintMatrix(work.soln->YU[1]);
}

int main() {
  printf("=== ADMM Functions Test ===\n");
  CheckTerminationTest();
  PrintTestResult();
  return TestResult();
}