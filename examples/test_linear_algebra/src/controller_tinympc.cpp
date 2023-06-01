/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2019 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * controller_tinympc.c - App layer application of TinyMPC.
 */

// 50HZ

// #include <Eigen.h>

#ifdef __cplusplus
extern "C" {
#endif

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "controller.h"
#include "physicalConstants.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "slap/slap.h"
#include "tinympc/tinympc.h"
#include "cf_math.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "CONTROLLER_TINYMPC"
#include "debug.h"

#define nop()  __asm__("nop")
int _sbrk() { return -1; }
int _close() { return -1; }
int _read() { return -1; }
int _fstat() { return -1; }
int _isatty() { return -1; }
int _lseek() { return -1; }

int _write(int file, char* ptr, int len)
{
   nop();
}

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));

    // Remove the DEBUG_PRINT.
    // DEBUG_PRINT("Hello World!\n");
  }
}

// Macro variables
#define DT 0.02f       // dt
#define NSTATES 12    // no. of states (error state)
#define NINPUTS 4     // no. of controls
#define NHORIZON 3   // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define NSIM NHORIZON      // length of reference trajectory
#define MPC_RATE RATE_50_HZ  // control frequency

// #include "params_50hz_agg.h"
#include "params_50hz.h"

/* Allocate global variables for MPC */

// Precompute data offline
static float A_data[NSTATES*NSTATES] = {
  1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,-0.000841f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,-0.168171f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000841f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.168171f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.010000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.010000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.010000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,-0.000001f,0.000000f,0.005000f,0.000000f,0.000000f,0.000000f,-0.000420f,0.000000f,1.000000f,0.000000f,0.000000f,
  0.000001f,0.000000f,0.000000f,0.000000f,0.005000f,0.000000f,0.000420f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.005000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,
};

static float B_data[NSTATES*NINPUTS] = {
  -0.000001f,0.000001f,0.000210f,-0.006884f,-0.006918f,0.000494f,-0.000388f,0.000386f,0.042043f,-2.753546f,-2.767070f,0.197477f,
  0.000001f,0.000001f,0.000210f,-0.007581f,0.007607f,-0.000181f,0.000426f,0.000425f,0.042043f,-3.032340f,3.042784f,-0.072236f,
  0.000001f,-0.000001f,0.000210f,0.006892f,0.006939f,-0.000696f,0.000389f,-0.000386f,0.042043f,2.756626f,2.775695f,-0.278438f,
  -0.000001f,-0.000001f,0.000210f,0.007573f,-0.007629f,0.000383f,-0.000428f,-0.000425f,0.042043f,3.029260f,-3.051409f,0.153197f,
};


static float At_data[NSTATES*NSTATES] = {
  1.000000f,0.000000f,0.000000f,0.000000f,0.000841f,0.000000f,0.010000f,0.000000f,0.000000f,0.000000f,0.000001f,0.000000f,
  0.000000f,1.000000f,0.000000f,-0.000841f,0.000000f,0.000000f,0.000000f,0.010000f,0.000000f,-0.000001f,0.000000f,0.000000f,
  0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.010000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.005000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.005000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.005000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.168171f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000420f,0.000000f,
  0.000000f,0.000000f,0.000000f,-0.168171f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,-0.000420f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,
};

static float Bt_data[NSTATES*NINPUTS] = {
  -0.000001f,0.000001f,0.000001f,-0.000001f,
  0.000001f,0.000001f,-0.000001f,-0.000001f,
  0.000210f,0.000210f,0.000210f,0.000210f,
  -0.006884f,-0.007581f,0.006892f,0.007573f,
  -0.006918f,0.007607f,0.006939f,-0.007629f,
  0.000494f,-0.000181f,-0.000696f,0.000383f,
  -0.000388f,0.000426f,0.000389f,-0.000428f,
  0.000386f,0.000425f,-0.000386f,-0.000425f,
  0.042043f,0.042043f,0.042043f,0.042043f,
  -2.753546f,-3.032340f,2.756626f,3.029260f,
  -2.767070f,3.042784f,2.775695f,-3.051409f,
  0.197477f,-0.072236f,-0.278438f,0.153197f,
};

static float C_data[NSTATES*NINPUTS] = {0.0f};

static float f_data[NSTATES] = {0};

// Create data array, all zero initialization
static float x0_data[NSTATES] = {0.0f};       // initial state
static float xg_data[NSTATES] = {0.0f};       // goal state (if not tracking)
static float ug_data[NINPUTS] = {0.0f};       // goal input 
static float X_data[NSTATES * NHORIZON] = {0.0f};        // X in MPC solve
static float U_data[NINPUTS * (NHORIZON - 1)] = {0.0f};  // U in MPC solve
static float d_data[NINPUTS * (NHORIZON - 1)] = {0.0f};
static float p_data[NSTATES * NHORIZON] = {0.0f};
// static float Q_data[NSTATES * NSTATES] = {0.0f};
// static float R_data[NINPUTS * NINPUTS] = {0.0f};
static float q_data[NSTATES*(NHORIZON-1)] = {0.0f};
static float r_data[NINPUTS*(NHORIZON-1)] = {0.0f};
static float r_tilde_data[NINPUTS*(NHORIZON-1)] = {0.0f};
static float Acu_data[NINPUTS * NINPUTS] = {0.0f};  
static float YU_data[NINPUTS * (NHORIZON - 1)] = {0.0f};
static float umin_data[NINPUTS] = {0.0f};
static float umax_data[NINPUTS] = {0.0f};
static float temp_data[NINPUTS + 2*NINPUTS*(NHORIZON - 1)] = {0.0f};

// Created matrices
static Matrix Xref[NSIM];
static Matrix Uref[NSIM - 1];
static Matrix X[NHORIZON];
static Matrix U[NHORIZON - 1];
static Matrix d[NHORIZON - 1];
static Matrix p[NHORIZON];
static Matrix YU[NHORIZON - 1];
static Matrix ZU[NHORIZON - 1];
static Matrix ZU_new[NHORIZON - 1];
static Matrix q[NHORIZON-1];
static Matrix r[NHORIZON-1];
static Matrix r_tilde[NHORIZON-1];
static Matrix A;
static Matrix B;
static Matrix C;
static Matrix f;

arm_matrix_instance_f32 A_arm;
arm_matrix_instance_f32 B_arm;
arm_matrix_instance_f32 C_arm;

// Helper variables
static bool isInit = false;  // fix for tracking problem
uint32_t time1 = 0;
uint32_t time2 = 0;
float res = 0;
uint64_t startTimestamp = 0;

void controllerOutOfTreeInit(void) {
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {

  /* Controller rate */
  if (!RATE_DO_EXECUTE(MPC_RATE, tick)) {
    return;
  }

  A_arm = {NSTATES, NSTATES, At_data};
  B_arm = {NSTATES, NINPUTS, Bt_data};
  C_arm = {NSTATES, NINPUTS, C_data};
  A = slap_MatrixFromArray(NSTATES, NSTATES, A_data);
  B = slap_MatrixFromArray(NSTATES, NINPUTS, B_data);
  C = slap_MatrixFromArray(NSTATES, NINPUTS, C_data);

  // Get current time
  startTimestamp = usecTimestamp();
  for (int i = 0; i < 50; ++i) {
    // mat_mult(&A_arm, &B_arm, &C_arm);
    // arm_mat_add_f32(&B_arm, &B_arm, &C_arm);
    arm_copy_f32(B.data, C.data, B.cols * B.rows);
  }
  time1 = usecTimestamp() - startTimestamp;
  float res1 = slap_NormTwo(C);

  startTimestamp = usecTimestamp();
  for (int i = 0; i < 50; ++i) {
    // slap_MatMulAB(C, A, B);
    // MatAdd(C, B, B, 1.0);
    MatCpy(C, B);
  }  
  time2 = usecTimestamp() - startTimestamp;
  float res2 = slap_NormTwo(C);

  res = res1 - res2;

  // DEBUG_PRINT("U[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(U[0].data[0]), (double)(U[0].data[1]), (double)(U[0].data[2]), (double)(U[0].data[3]));
  // DEBUG_PRINT("ZU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(ZU[0].data[0]), (double)(ZU[0].data[1]), (double)(ZU[0].data[2]), (double)(ZU[0].data[3]));
  // DEBUG_PRINT("YU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(YU[0].data[0]), (double)(YU[0].data[1]), (double)(YU[0].data[2]), (double)(YU[0].data[3]));
  // DEBUG_PRINT("info.pri_res: %f\n", (double)(info.pri_res));
  // DEBUG_PRINT("info.dua_res: %f\n", (double)(info.dua_res));
  // result =  info.status_val * info.iter;
  // DEBUG_PRINT("%d %d %d \n", info.status_val, info.iter, mpcTime);
  // DEBUG_PRINT("[%.2f, %.2f, %.2f]\n", (double)(x0_data[0]), (double)(x0_data[1]), (double)(x0_data[2]));

  control->normalizedForces[0] = 0.0f;
  control->normalizedForces[1] = 0.0f;
  control->normalizedForces[2] = 0.0f;
  control->normalizedForces[3] = 0.0f;

  control->controlMode = controlModePWM;
}

/**
 * Tunning variables for the full state quaternion LQR controller
 */
// PARAM_GROUP_START(ctrlMPC)
// /**
//  * @brief K gain
//  */
// PARAM_ADD(PARAM_FLOAT, u_hover, &u_hover)

// PARAM_GROUP_STOP(ctrlMPC)

/**
 * Logging variables for the command and reference signals for the
 * MPC controller
 */

LOG_GROUP_START(ctrlMPC)
LOG_ADD(LOG_FLOAT, res, &res)
LOG_ADD(LOG_UINT32, time1, &time1)
LOG_ADD(LOG_UINT32, time2, &time2)
LOG_GROUP_STOP(ctrlMPC)

#ifdef __cplusplus
}
#endif
