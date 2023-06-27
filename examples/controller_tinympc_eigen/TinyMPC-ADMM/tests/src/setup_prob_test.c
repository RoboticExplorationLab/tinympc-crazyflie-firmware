#include "simpletest.h"
#include "slap/slap.h"
#include "test_utils.h"
#include "tinympc/auxil.h"
#include "tinympc/model.h"

#define NSTATES 2
#define NINPUTS 1
#define NHORIZON 3

sfloat dt = 0.1;
sfloat A_data[NSTATES * NSTATES] = {1, 0, 1, 1};  // NOLINT
sfloat B_data[NSTATES * NINPUTS] = {1, 2};        // NOLINT
sfloat f_data[NSTATES] = {4, 5};                  // NOLINT
sfloat x0_data[NSTATES] = {0.1, 0.2};
sfloat xg_data[NSTATES] = {0};
sfloat Xref_data[NSTATES * NHORIZON] = {0.2, 1.1, 2.5, 3.7, 2.1, 4.5};
sfloat Uref_data[NINPUTS * (NHORIZON - 1)] = {1, 2};
sfloat Kinf_data[NINPUTS * NSTATES] = {0};
sfloat d_data[NINPUTS * (NHORIZON - 1)] = {0};
sfloat Pinf_data[NSTATES * NSTATES] = {0};
sfloat p_data[NSTATES * NHORIZON] = {0};
sfloat Q_data[NSTATES * NSTATES] = {1, 0, 0, 1};  // NOLINT
sfloat R_data[NINPUTS * NINPUTS] = {1};           // NOLINT
sfloat q_data[NSTATES] = {0.1, 0.2};              // NOLINT
sfloat r_data[NINPUTS] = {-0.6};                  // NOLINT
sfloat Acu_data[NINPUTS * NINPUTS] = {0};  // A1*u <= b1
sfloat Acx_data[NSTATES * NSTATES] = {0};  // A2*x <= b2
sfloat ucu_data[NINPUTS] = {1.1};
sfloat lcu_data[NINPUTS] = {-1.1};
sfloat ucx_data[NSTATES] = {1.6, 1.7};
sfloat lcx_data[NSTATES] = {-1.6, -1.7};
sfloat YU_data[NINPUTS * (NHORIZON - 1)] = {1, 2};
sfloat YX_data[NSTATES * (NHORIZON)] = {1, 2, 3, 4, 5, 6};
sfloat YG_data[NSTATES] = {1, 2};

sfloat Quu_inv_data[NINPUTS * NINPUTS] = {10.1};
sfloat AmBKt_data[NSTATES * NSTATES] = {1.2, 2.3, 21.1, 12.3};
sfloat coeff_d2p_data[NSTATES * NINPUTS] = {1.2, 2.3};

sfloat reg = 1e-6;
sfloat reg_min = 1;
sfloat reg_max = 100;
sfloat rho_max = 1e5;
sfloat rho_mul = 1;
int max_outer_iters = 100;
int max_search_iters = 10;

void SettingsTest() {
  tiny_AdmmSettings settings;
  tiny_InitSettings(&settings);

  TEST(settings.reg_max == (sfloat)REG_MAX);
  TEST(settings.max_iter == MAX_ITER);

  settings.reg_max = (sfloat)10.5;
  TEST(settings.reg_max == (sfloat)10.5);
}

void SolutionTest() {
  // int n = NSTATES;
  // int m = NINPUTS;
  // int N = NHORIZON;
  tiny_Model model;
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, dt);
  tiny_AdmmSettings stgs;
  tiny_InitSettings(&stgs);  //if switch on/off during run, initialize all
  tiny_AdmmData data;
  tiny_AdmmInfo info;
  tiny_AdmmSolution soln;
  tiny_AdmmWorkspace work;
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  // int true_size = m*m + m + n*n + n*m + 2*m*(N - 1);
  // TEST(work.data_size == true_size);
}

void DataTest() {
  // int n = NSTATES;
  // int m = NINPUTS;
  // int N = NHORIZON;
  tiny_Model model;
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, dt);
  tiny_AdmmSettings stgs;
  tiny_InitSettings(&stgs);  //if switch on/off during run, initialize all
  tiny_AdmmData data;
  tiny_AdmmInfo info;
  tiny_AdmmSolution soln;
  tiny_AdmmWorkspace work;
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  // int true_size = n + n*n*2 + m*m + (N-1)*(n + m) + m + N*n + (N-1)*m;
  // true_size += 2*m*m + 2*m + 2*n*n + 2*n;
  // TEST(work.data->data_size == true_size);
}

void WorkspaceTest() {
  int n = NSTATES;
  int m = NINPUTS;
  // int N = NHORIZON;
  tiny_Model model;
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, dt);
  tiny_AdmmSettings stgs;
  tiny_InitSettings(&stgs);  //if switch on/off during run, initialize all
  tiny_AdmmData data;
  tiny_AdmmInfo info;
  tiny_AdmmSolution soln;
  tiny_AdmmWorkspace work;
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);

  TEST(work.first_run == 1);

  int true_size = m + 2*m*(NHORIZON - 1);
  TEST(work.data_size == true_size);
  
  sfloat temp_data[work.data_size];
  T_INIT_ZEROS(temp_data);
  temp_data[work.data_size-1] = 1.0;
  Matrix ZU[NHORIZON-1];
  Matrix ZU_new[NHORIZON-1];

  tiny_InitWorkspaceTempData(&work, ZU, ZU_new, TINY_NULL, TINY_NULL, temp_data);

  TEST((work.ZU[0].rows == m) && work.ZU[1].cols == 1);
  TEST((work.Qu.rows == m) && work.Qu.cols == 1);
  TEST(work.ZX == TINY_NULL);
  TESTAPPROX(work.ZU_new[NHORIZON-2].data[m-1], 1.0, 1e-6);
  TESTAPPROX(work.ZU[NHORIZON-2].data[m-1], 0.0, 1e-6);

  tiny_InitPrimalCache(&work, Quu_inv_data, AmBKt_data, coeff_d2p_data);
  TEST((work.Quu_inv.rows == m) && (work.Quu_inv.cols == m));
  TEST((work.coeff_d2p.rows == n) && (work.coeff_d2p.cols == m));
  TESTAPPROX(SumOfSquaredError(work.Quu_inv.data, Quu_inv_data, m*m), 0, 1e-6);
}

int main() {
  printf("=== Workspace Test ===\n");
  SettingsTest();
  SolutionTest();
  DataTest();
  WorkspaceTest();
  PrintTestResult();
  return TestResult();
}
