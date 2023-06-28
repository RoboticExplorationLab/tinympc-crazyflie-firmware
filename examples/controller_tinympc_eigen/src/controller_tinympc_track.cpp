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

/** 
 * Single lap
 */

#include "Eigen.h"

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

#include "tinympc/tinympc.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TINYMPC-E"
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
#define DT 0.002f       // dt
// #define NSTATES 12    // no. of states (error state)
// #define NINPUTS 4     // no. of controls
#define NHORIZON 7   // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define MPC_RATE RATE_500_HZ  // control frequency

using namespace Eigen;

// #include "traj_fig8_12.h"
// #include "traj_circle_500hz.h"
#include "traj_perching.h"

// Precomputed data and cache
static MatrixNf A;
static MatrixNMf B;
static MatrixMNf Kinf;
static MatrixNf Pinf;
static MatrixMf Quu_inv;
static MatrixNf AmBKt;
static MatrixNMf coeff_d2p;
static MatrixNf Q;
static MatrixMf R;

/* Allocate global variables for MPC */

static VectorNf Xhrz[NHORIZON];
static VectorMf Uhrz[NHORIZON-1]; 
static VectorMf d[NHORIZON-1];
static VectorNf p[NHORIZON];
static VectorMf YU[NHORIZON];

static VectorNf q[NHORIZON-1];
static VectorMf r[NHORIZON-1];
static VectorMf r_tilde[NHORIZON-1];

static VectorNf Xref[NHORIZON];
static VectorMf Uref[NHORIZON-1];

static MatrixMf Acu;
static VectorMf ucu;
static VectorMf lcu;

static VectorMf Qu;
static VectorMf ZU[NHORIZON-1]; 
static VectorMf ZU_new[NHORIZON-1];

static VectorNf x0;
static VectorNf xg;
static VectorMf ug;

// Create TinyMPC struct
static tiny_Model model;
static tiny_AdmmSettings stgs;
static tiny_AdmmData data;
static tiny_AdmmInfo info;
static tiny_AdmmSolution soln;
static tiny_AdmmWorkspace work;


// Helper variables
static uint64_t startTimestamp;
static bool isInit = false;  // fix for tracking problem
static uint32_t mpcTime = 0;
static float u_hover = 0.67f;
static int8_t result = 0;
static uint32_t step = 0;
static bool en_traj = false;
static uint32_t traj_length = T_ARRAY_SIZE(X_ref_data) / NSTATES;
static int8_t user_traj_iter = 1;  // number of times to execute full trajectory
static int8_t traj_hold = 1;  // hold current trajectory for this no of steps
static int8_t traj_iter = 0;
static uint32_t traj_idx = 0;

static struct vec desired_rpy;
static struct quat attitude;
static struct vec phi;

void controllerOutOfTreeInit(void) {
  // if (isInit) {
  //   return;
  // }
  
  /* Start MPC initialization*/

  // Precompute/Cache
A << 
1.000000f,0.000000f,0.000000f,0.000000f,0.000039f,0.000000f,0.002000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
0.000000f,1.000000f,0.000000f,-0.000039f,0.000000f,0.000000f,0.000000f,0.002000f,0.000000f,-0.000000f,0.000000f,0.000000f,
0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.002000f,0.000000f,0.000000f,0.000000f,
0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.001000f,0.000000f,0.000000f,
0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.001000f,0.000000f,
0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.001000f,
0.000000f,0.000000f,0.000000f,0.000000f,0.039240f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000020f,0.000000f,
0.000000f,0.000000f,0.000000f,-0.039240f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,-0.000020f,0.000000f,0.000000f,
0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,
0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,
0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,
0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f;

B << 
-0.000000f,0.000000f,0.000000f,-0.000000f,
0.000000f,0.000000f,-0.000000f,-0.000000f,
0.000008f,0.000008f,0.000008f,0.000008f,
-0.000275f,-0.000303f,0.000276f,0.000303f,
-0.000277f,0.000304f,0.000278f,-0.000305f,
0.000020f,-0.000007f,-0.000028f,0.000015f,
-0.000004f,0.000004f,0.000004f,-0.000004f,
0.000004f,0.000004f,-0.000004f,-0.000004f,
0.008409f,0.008409f,0.008409f,0.008409f,
-0.550709f,-0.606468f,0.551325f,0.605852f,
-0.553414f,0.608557f,0.555139f,-0.610282f,
0.039495f,-0.014447f,-0.055688f,0.030639f;

Kinf << 
-0.114173f,0.112349f,0.292487f,-0.292346f,-0.300866f,-0.164833f,-0.069354f,0.067790f,0.090898f,-0.040118f,-0.043559f,-0.084483f,
0.116585f,0.109939f,0.292487f,-0.278803f,0.305372f,0.270863f,0.070579f,0.065426f,0.090898f,-0.034821f,0.043842f,0.104540f,
0.102398f,-0.107718f,0.292487f,0.274828f,0.256506f,-0.534809f,0.060555f,-0.064321f,0.090898f,0.034615f,0.029718f,-0.154034f,
-0.104809f,-0.114570f,0.292487f,0.296320f,-0.261012f,0.428780f,-0.061779f,-0.068895f,0.090898f,0.040324f,-0.030001f,0.133978f;

Pinf << 
19308.976658f,-20.305121f,-0.000000f,60.467745f,8175.928853f,1204.780291f,3510.718747f,-13.231492f,-0.000000f,10.040773f,96.567593f,297.369827f,
-20.305121f,19288.758671f,0.000000f,-8107.921040f,-60.479559f,-470.823843f,-13.232994f,3496.569098f,0.000000f,-82.720650f,-10.042505f,-116.971720f,
-0.000000f,0.000000f,73880.135065f,-0.000000f,-0.000000f,-0.000000f,-0.000000f,0.000000f,13778.508684f,-0.000000f,-0.000000f,-0.000000f,
60.467745f,-8107.921040f,-0.000000f,19205.400520f,189.099709f,1641.387679f,40.517000f,-4642.589214f,-0.000000f,215.170373f,34.875313f,438.592382f,
8175.928853f,-60.479559f,-0.000000f,189.099709f,19443.206828f,4180.168772f,4691.278591f,-40.520232f,-0.000000f,34.877251f,268.371587f,1110.384284f,
1204.780291f,-470.823843f,-0.000000f,1641.387679f,4180.168772f,209670.869527f,857.948497f,-336.247065f,-0.000000f,381.585632f,963.847217f,17245.522153f,
3510.718747f,-13.232994f,-0.000000f,40.517000f,4691.278591f,857.948497f,1535.168450f,-8.760189f,-0.000000f,7.143114f,60.365712f,221.004206f,
-13.231492f,3496.569098f,0.000000f,-4642.589214f,-40.520232f,-336.247065f,-8.760189f,1525.133339f,0.000000f,-49.891610f,-7.143356f,-87.154869f,
-0.000000f,0.000000f,13778.508684f,-0.000000f,-0.000000f,-0.000000f,-0.000000f,0.000000f,4277.001380f,-0.000000f,-0.000000f,-0.000000f,
10.040773f,-82.720650f,-0.000000f,215.170373f,34.877251f,381.585632f,7.143114f,-49.891610f,-0.000000f,31.060718f,9.515787f,136.215730f,
96.567593f,-10.042505f,-0.000000f,34.875313f,268.371587f,963.847217f,60.365712f,-7.143356f,-0.000000f,9.515787f,48.145869f,342.538094f,
297.369827f,-116.971720f,-0.000000f,438.592382f,1110.384284f,17245.522153f,221.004206f,-87.154869f,-0.000000f,136.215730f,342.538094f,5879.042227f;

Quu_inv << 
0.002428f,0.000000f,0.000103f,-0.000001f,
0.000000f,0.002419f,-0.000000f,0.000111f,
0.000103f,-0.000000f,0.002424f,0.000003f,
-0.000001f,0.000111f,0.000003f,0.002418f;

AmBKt << 
1.000000f,-0.000000f,-0.000000f,0.000007f,-0.000127f,0.000008f,-0.000002f,-0.000000f,-0.000000f,0.014873f,-0.254942f,0.015107f,
-0.000000f,1.000000f,0.000000f,0.000129f,-0.000007f,-0.000003f,-0.000000f,-0.000002f,0.000000f,0.257347f,-0.014850f,-0.005337f,
0.000000f,-0.000000f,0.999990f,-0.000000f,0.000000f,-0.000000f,0.000000f,0.000000f,-0.009838f,-0.000000f,0.000000f,-0.000000f,
0.000000f,-0.000039f,-0.000000f,0.999669f,0.000018f,0.000007f,0.000000f,-0.039236f,-0.000000f,-0.661128f,0.036150f,0.013744f,
0.000039f,-0.000000f,-0.000000f,0.000018f,0.999673f,0.000019f,0.039236f,-0.000000f,-0.000000f,0.036224f,-0.654027f,0.038576f,
0.000000f,-0.000000f,-0.000000f,0.000054f,0.000151f,0.999984f,0.000002f,-0.000001f,-0.000000f,0.108571f,0.302513f,-0.032496f,
0.002000f,-0.000000f,-0.000000f,0.000004f,-0.000076f,0.000005f,0.999999f,-0.000000f,-0.000000f,0.008653f,-0.152652f,0.009024f,
-0.000000f,0.002000f,0.000000f,0.000077f,-0.000004f,-0.000002f,-0.000000f,0.999999f,0.000000f,0.154213f,-0.008638f,-0.003203f,
0.000000f,-0.000000f,0.001997f,-0.000000f,0.000000f,-0.000000f,0.000000f,-0.000000f,0.996943f,-0.000000f,0.000000f,-0.000000f,
0.000000f,-0.000000f,0.000000f,0.000957f,0.000002f,0.000001f,0.000000f,-0.000019f,0.000000f,0.913274f,0.004382f,0.001774f,
0.000000f,-0.000000f,-0.000000f,0.000002f,0.000957f,0.000002f,0.000019f,-0.000000f,-0.000000f,0.004392f,0.914406f,0.004928f,
0.000000f,-0.000000f,-0.000000f,0.000010f,0.000028f,0.000996f,0.000000f,-0.000000f,-0.000000f,0.020627f,0.056902f,0.992164f;

coeff_d2p << 
0.543173f,-0.546427f,-0.468820f,0.472074f,
-0.523220f,-0.495995f,0.493582f,0.525632f,
-0.883010f,-0.883010f,-0.883010f,-0.883010f,
1.564914f,1.432870f,-1.434894f,-1.562889f,
1.657138f,-1.657799f,-1.330935f,1.331596f,
1.352614f,-1.210724f,0.836832f,-0.978722f,
0.303134f,-0.302924f,-0.249389f,0.249178f,
-0.287599f,-0.265460f,0.266150f,0.286909f,
-0.387645f,-0.387645f,-0.387645f,-0.387645f,
0.027209f,0.007258f,-0.012189f,-0.022278f,
0.041960f,-0.037070f,-0.004588f,-0.000302f,
0.397700f,-0.335583f,0.176752f,-0.238870f;

Q << 
100.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
0.000000f,100.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
0.000000f,0.000000f,400.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1111.111111f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,
0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,2.040816f,0.000000f,0.000000f,
0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,2.040816f,0.000000f,
0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,25.000000f;

R << 
144.000000f,0.000000f,0.000000f,0.000000f,
0.000000f,144.000000f,0.000000f,0.000000f,
0.000000f,0.000000f,144.000000f,0.000000f,
0.000000f,0.000000f,0.000000f,144.000000f;

  // End of Precompute/Cache

  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT, &A, &B, 0);
  tiny_InitSettings(&stgs);
  stgs.rho_init = 250.0;  // Important (select offline, associated with precomp.)
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  
  // Fill in the remaining struct 
  tiny_InitWorkspaceTemp(&work, &Qu, ZU, ZU_new, 0, 0);
  tiny_InitPrimalCache(&work, &Quu_inv, &AmBKt, &coeff_d2p);
  tiny_InitSolution(&work, Xhrz, Uhrz, 0, YU, 0, &Kinf, d, &Pinf, p);

  tiny_SetInitialState(&work, &x0);  
  // tiny_SetGoalState(&work, Xref, &xg);
  tiny_SetStateReference(&work, Xref);
  tiny_SetInputReference(&work, Uref);
  // tiny_SetGoalInput(&work, Uref, &ug);

  /* Set up LQR cost */
  tiny_InitDataCost(&work, &Q, q, &R, r, r_tilde);
  // R = R + stgs.rho_init * MatrixMf::Identity();
  // /* Set up constraints */
  ucu.fill(1 - u_hover);
  lcu.fill(-u_hover);
  tiny_SetInputBound(&work, &Acu, &lcu, &ucu);

  tiny_UpdateLinearCost(&work);

  /* Solver settings */
  // Solver settings 
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 0;
  stgs.max_iter = 8;           // limit this if needed
  stgs.verbose = 0;
  stgs.check_termination = 2;
  stgs.tol_abs_dual = 5e-2;
  stgs.tol_abs_prim = 5e-2;

  /* End of MPC initialization */  
  en_traj = true;
  step = 0;  
  traj_iter = 0;
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
  // Get current time
  startTimestamp = usecTimestamp();

  // Update reference: from stored trajectory or commander
  if (en_traj) {
    if (step % traj_hold == 0) {
      traj_idx = (int)(step / traj_hold);
      for (int i = 0; i < NHORIZON; ++i) {
        for (int j = 0; j < NSTATES; ++j) {
          Xref[i](j) = X_ref_data[(traj_idx + i)*NSTATES + j];
        }
        if (i < NHORIZON - 1) {
          for (int j = 0; j < NINPUTS; ++j) {
            Uref[i](j) = U_ref_data[(traj_idx + i)*NINPUTS + j];
          }          
        }
      }
    }
  }
  else {
    xg(0)  = setpoint->position.x;
    xg(1)  = setpoint->position.y;
    xg(2)  = setpoint->position.z;
    xg(6)  = setpoint->velocity.x;
    xg(7)  = setpoint->velocity.y;
    xg(8)  = setpoint->velocity.z;
    xg(9)  = radians(setpoint->attitudeRate.roll);
    xg(10) = radians(setpoint->attitudeRate.pitch);
    xg(11) = radians(setpoint->attitudeRate.yaw);
    desired_rpy = mkvec(radians(setpoint->attitude.roll), 
                        radians(setpoint->attitude.pitch), 
                        radians(setpoint->attitude.yaw));
    attitude = rpy2quat(desired_rpy);
    phi = quat2rp(qnormalize(attitude));  
    xg(3) = phi.x;
    xg(4) = phi.y;
    xg(5) = phi.z;
    tiny_SetGoalState(&work, Xref, &xg);
    tiny_SetGoalInput(&work, Uref, &ug);
    // xg(1) = 1.0;
    // xg(2) = 2.0;
  }
  // DEBUG_PRINT("z_ref = %.2f\n", (double)(Xref[0](2)));

  /* Get current state (initial state for MPC) */
  // delta_x = x - x_bar; x_bar = 0
  // Positon error, [m]
  x0(0) = state->position.x;
  x0(1) = state->position.y;
  x0(2) = state->position.z;
  // Body velocity error, [m/s]                          
  x0(6) = state->velocity.x;
  x0(7) = state->velocity.y;
  x0(8) = state->velocity.z;
  // Angular rate error, [rad/s]
  x0(9)  = radians(sensors->gyro.x);   
  x0(10) = radians(sensors->gyro.y);
  x0(11) = radians(sensors->gyro.z);
  attitude = mkquat(
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
    state->attitudeQuaternion.w);  // current attitude
  phi = quat2rp(qnormalize(attitude));  // quaternion to Rodriquez parameters  
  // Attitude error
  x0(3) = phi.x;
  x0(4) = phi.y;
  x0(5) = phi.z;

  /* MPC solve */
  
  // Warm-start by previous solution  // TODO: should I warm-start U with previous ZU
  // tiny_ShiftFill(U, T_ARRAY_SIZE(U));

  // Solve optimization problem using ADMM
  tiny_UpdateLinearCost(&work);
  tiny_SolveAdmm(&work);

  // // JUST LQR
  // Uhrz[0] = -(Kinf) * (x0 - xg);

  mpcTime = usecTimestamp() - startTimestamp;
 
  // DEBUG_PRINT("Uhrz[0] = [%.2f, %.2f]\n", (double)(Uhrz[0](0)), (double)(Uhrz[0](1)));
  // DEBUG_PRINT("ZU[0] = [%.2f, %.2f]\n", (double)(ZU_new[0](0)), (double)(ZU_new[0](1)));
  // DEBUG_PRINT("YU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(YU[0].data[0]), (double)(YU[0].data[1]), (double)(YU[0].data[2]), (double)(YU[0].data[3]));
  // DEBUG_PRINT("info.pri_res: %f\n", (double)(info.pri_res));
  // DEBUG_PRINT("info.dua_res: %f\n", (double)(info.dua_res));
  // result =  info.status_val * info.iter;
  DEBUG_PRINT("%d %d %d \n", info.status_val, info.iter, mpcTime);
  // DEBUG_PRINT("%.2f, %.2f, %.2f, %.2f \n", (double)(Xref[0](5)), (double)(Uhrz[0](2)), (double)(Uhrz[0](3)), (double)(ZU_new[0](0)));
  
  /* Output control */
  if (setpoint->mode.z == modeDisable) {
    control->normalizedForces[0] = 0.0f;
    control->normalizedForces[1] = 0.0f;
    control->normalizedForces[2] = 0.0f;
    control->normalizedForces[3] = 0.0f;
  } else {
    control->normalizedForces[0] = ZU_new[0](0) + u_hover;  // PWM 0..1
    control->normalizedForces[1] = ZU_new[0](1) + u_hover;
    control->normalizedForces[2] = ZU_new[0](2) + u_hover;
    control->normalizedForces[3] = ZU_new[0](3) + u_hover;
  } 
  // DEBUG_PRINT("pwm = [%.2f, %.2f]\n", (double)(control->normalizedForces[0]), (double)(control->normalizedForces[1]));

  // control->normalizedForces[0] = 0.0f;
  // control->normalizedForces[1] = 0.0f;
  // control->normalizedForces[2] = 0.0f;
  // control->normalizedForces[3] = 0.0f;

  control->controlMode = controlModePWM;
  
  //// stop trajectory executation
  if (en_traj) {
    if (traj_iter >= user_traj_iter) en_traj = false;

    if (traj_idx >= traj_length - 1 - NHORIZON + 1) { 
      // complete one trajectory, do it again
      step = 0; 
      traj_iter += 1;
    } 
    else step += 1;
  }
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

// LOG_ADD(LOG_FLOAT, x, &(x0(0)))
// LOG_ADD(LOG_FLOAT, y, &(x0(1)))
// LOG_ADD(LOG_FLOAT, z, &(x0(2)))

// LOG_ADD(LOG_FLOAT, roll,  &(x0(3)))
// LOG_ADD(LOG_FLOAT, pitch, &(x0(4)))
// LOG_ADD(LOG_FLOAT, yaw,   &(x0(5)))

// LOG_ADD(LOG_FLOAT, vx, &(x0(6)))
// LOG_ADD(LOG_FLOAT, vy, &(x0(7)))
// LOG_ADD(LOG_FLOAT, vz, &(x0(8)))

// LOG_ADD(LOG_FLOAT, wroll,  &(x0(9)))
// LOG_ADD(LOG_FLOAT, wpitch, &(x0(10)))
// LOG_ADD(LOG_FLOAT, wyaw,   &(x0(11)))

// LOG_ADD(LOG_INT8, result, &result)
// LOG_ADD(LOG_UINT32, mpcTime, &mpcTime)

// LOG_ADD(LOG_FLOAT, u0, &(Uhrz[0](0)))
// LOG_ADD(LOG_FLOAT, u1, &(Uhrz[0](1)))
// LOG_ADD(LOG_FLOAT, u2, &(Uhrz[0](2)))
// LOG_ADD(LOG_FLOAT, u3, &(Uhrz[0](3)))

// LOG_ADD(LOG_FLOAT, yu0, &(YU[0](0)))
// LOG_ADD(LOG_FLOAT, yu1, &(YU[0](1)))
// LOG_ADD(LOG_FLOAT, yu2, &(YU[0](2)))
// LOG_ADD(LOG_FLOAT, yu3, &(YU[0](3)))

LOG_GROUP_STOP(ctrlMPC)

#ifdef __cplusplus
}
#endif
