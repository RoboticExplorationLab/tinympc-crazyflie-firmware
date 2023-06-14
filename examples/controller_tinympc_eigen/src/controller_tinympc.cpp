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

// 100HZ

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
#define DT 0.01f       // dt
// #define NSTATES 12    // no. of states (error state)
// #define NINPUTS 4     // no. of controls
#define NHORIZON 3   // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define NSIM NHORIZON      // length of reference trajectory
#define MPC_RATE RATE_100_HZ  // control frequency

using namespace Eigen;

// #include "params_100hz.h"

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

static VectorNf Xref[NSIM];
static VectorMf Uref[NSIM-1];

static MatrixMf Acu;
static VectorMf ucu;
static VectorMf lcu;

static VectorMf Qu;
static VectorMf ZU[NHORIZON-1]; 
static VectorMf ZU_new[NHORIZON-1];

static VectorNf x0;
static VectorNf xg = (VectorNf() << 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
static VectorMf ug = (VectorMf() << 0, 0, 0, 0).finished();

static VectorNf X[NSIM];

// Create TinyMPC struct
static tiny_Model model;
static tiny_AdmmSettings stgs;
static tiny_AdmmData data;
static tiny_AdmmInfo info;
static tiny_AdmmSolution soln;
static tiny_AdmmWorkspace work;

// Helper variables
static bool isInit = false;  // fix for tracking problem
static uint64_t startTimestamp;
static uint32_t mpcTime = 0;
static float u_hover = 0.6f;

static float setpoint_z = 0.1f;
static float setpoint_x = 0.0f;
static int z_sign = 1;
static int8_t result = 0;

static struct vec desired_rpy;
static struct quat attitude;
static struct vec phi;

void controllerOutOfTreeInit(void) {
  // if (isInit) {
  //   return;
  // }
  
  /* Start MPC initialization*/
  A << 
  1.000000f,0.000000f,0.000000f,0.000000f,0.000981f,0.000000f,0.010000f,0.000000f,0.000000f,0.000000f,0.000002f,0.000000f,
  0.000000f,1.000000f,0.000000f,-0.000981f,0.000000f,0.000000f,0.000000f,0.010000f,0.000000f,-0.000002f,0.000000f,0.000000f,
  0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.010000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.005000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.005000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.005000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.196200f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000491f,0.000000f,
  0.000000f,0.000000f,0.000000f,-0.196200f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,-0.000491f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f;

  B << 
  -0.000001f,0.000001f,0.000001f,-0.000001f,
  0.000001f,0.000001f,-0.000001f,-0.000001f,
  0.000210f,0.000210f,0.000210f,0.000210f,
  -0.006884f,-0.007581f,0.006892f,0.007573f,
  -0.006918f,0.007607f,0.006939f,-0.007629f,
  0.000494f,-0.000181f,-0.000696f,0.000383f,
  -0.000452f,0.000497f,0.000454f,-0.000499f,
  0.000450f,0.000496f,-0.000451f,-0.000495f,
  0.042043f,0.042043f,0.042043f,0.042043f,
  -2.753546f,-3.032340f,2.756626f,3.029260f,
  -2.767070f,3.042784f,2.775695f,-3.051409f,
  0.197477f,-0.072236f,-0.278438f,0.153197f;

  Kinf << 
  -0.312260f,0.304883f,0.428867f,-0.855749f,-0.931259f,-0.821987f,-0.180993f,0.173195f,0.241446f,-0.058366f,-0.070072f,-0.307631f,
  0.304775f,0.283439f,0.428867f,-0.722432f,0.901351f,0.840037f,0.175893f,0.156169f,0.241446f,-0.040561f,0.067789f,0.311377f,
  0.278452f,-0.291426f,0.428867f,0.753441f,0.675057f,-0.883715f,0.151735f,-0.161529f,0.241446f,0.042902f,0.031386f,-0.320428f,
  -0.270967f,-0.296895f,0.428867f,0.824740f,-0.645149f,0.865664f,-0.146635f,-0.167835f,0.241446f,0.056025f,-0.029104f,0.316681f;

  Pinf << 
  5621.717598f,-16.999819f,0.000000f,44.642509f,3293.481980f,238.946972f,1352.895591f,-9.736375f,0.000000f,2.280192f,24.531210f,56.725447f,
  -16.999819f,5598.859190f,-0.000000f,-3232.957759f,-44.763451f,-95.176553f,-9.747921f,1339.573926f,0.000000f,-21.672346f,-2.290968f,-22.603260f,
  0.000000f,-0.000000f,5623.057189f,0.000000f,-0.000000f,-0.000000f,0.000000f,-0.000000f,1353.708470f,-0.000000f,-0.000000f,-0.000000f,
  44.642509f,-3232.957759f,-0.000000f,8044.073369f,170.914880f,606.771943f,29.442129f,-1751.840465f,-0.000000f,59.494802f,13.068636f,168.651920f,
  3293.481980f,-44.763451f,-0.000000f,170.914880f,8312.966125f,1519.441300f,1794.885702f,-29.479318f,-0.000000f,13.058986f,81.515019f,422.170302f,
  238.946972f,-95.176553f,-0.000000f,606.771943f,1519.441300f,29392.836577f,194.635185f,-77.634157f,-0.000000f,68.858907f,172.288849f,2831.879754f,
  1352.895591f,-9.747921f,0.000000f,29.442129f,1794.885702f,194.635185f,589.167197f,-5.894599f,0.000000f,1.720063f,14.691560f,48.626542f,
  -9.736375f,1339.573926f,0.000000f,-1751.840465f,-29.479318f,-77.634157f,-5.894599f,580.781214f,0.000000f,-12.180653f,-1.724297f,-19.403118f,
  0.000000f,-0.000000f,1353.708470f,0.000000f,-0.000000f,-0.000000f,0.000000f,-0.000000f,757.339155f,-0.000000f,-0.000000f,-0.000000f,
  2.280192f,-21.672346f,-0.000000f,59.494802f,13.058986f,68.858907f,1.720063f,-12.180653f,-0.000000f,5.035126f,1.733579f,25.292495f,
  24.531210f,-2.290968f,-0.000000f,13.068636f,81.515019f,172.288849f,14.691560f,-1.724297f,-0.000000f,1.733579f,8.325373f,63.260972f,
  56.725447f,-22.603260f,-0.000000f,168.651920f,422.170302f,2831.879754f,48.626542f,-19.403118f,-0.000000f,25.292495f,63.260972f,1048.140886f;

  Quu_inv << 
  0.005445f,0.000027f,0.001908f,0.000003f,
  0.000027f,0.005348f,0.000007f,0.002001f,
  0.001908f,0.000007f,0.005428f,0.000039f,
  0.000003f,0.002001f,0.000039f,0.005340f;

  AmBKt << 
  0.999999f,-0.000000f,0.000000f,0.000294f,-0.008478f,0.000507f,-0.000554f,-0.000019f,0.000000f,0.117602f,-3.391143f,0.202723f,
  -0.000000f,0.999999f,0.000000f,0.008504f,-0.000290f,-0.000188f,-0.000019f,-0.000556f,0.000000f,3.401720f,-0.115848f,-0.075393f,
  -0.000000f,0.000000f,0.999639f,0.000000f,0.000000f,-0.000000f,0.000000f,0.000000f,-0.072123f,0.000000f,0.000000f,-0.000000f,
  0.000000f,-0.000977f,-0.000000f,0.977194f,0.000639f,0.000501f,0.000042f,-0.194709f,-0.000000f,-9.122312f,0.255584f,0.200244f,
  0.000977f,-0.000000f,0.000000f,0.000656f,0.977095f,0.001340f,0.194702f,-0.000043f,0.000000f,0.262382f,-9.161845f,0.535809f,
  0.000000f,-0.000000f,-0.000000f,0.000244f,0.000660f,0.999611f,0.000043f,-0.000016f,-0.000000f,0.097650f,0.263871f,-0.155672f,
  0.009999f,-0.000000f,0.000000f,0.000152f,-0.004762f,0.000283f,0.999689f,-0.000010f,0.000000f,0.060915f,-1.904636f,0.113160f,
  -0.000000f,0.009999f,-0.000000f,0.004760f,-0.000149f,-0.000105f,-0.000010f,0.999689f,-0.000000f,1.904151f,-0.059726f,-0.042185f,
  0.000000f,0.000000f,0.009797f,0.000000f,0.000000f,-0.000000f,0.000000f,0.000000f,0.959396f,-0.000000f,0.000000f,-0.000000f,
  0.000000f,-0.000001f,0.000000f,0.003571f,0.000034f,0.000030f,0.000002f,-0.000397f,0.000000f,0.428313f,0.013789f,0.011959f,
  0.000001f,-0.000000f,-0.000000f,0.000036f,0.003560f,0.000080f,0.000396f,-0.000002f,-0.000000f,0.014255f,0.423915f,0.031932f,
  0.000000f,-0.000000f,-0.000000f,0.000053f,0.000143f,0.004864f,0.000009f,-0.000003f,-0.000000f,0.021116f,0.057044f,0.945509f;

  coeff_d2p << 
  0.001658f,-0.001578f,-0.001371f,0.001291f,
  -0.001457f,-0.001255f,0.001342f,0.001369f,
  -0.005144f,-0.005144f,-0.005144f,-0.005144f,
  0.004056f,0.003558f,-0.003805f,-0.003809f,
  0.004565f,-0.004337f,-0.003934f,0.003707f,
  0.006094f,-0.006809f,0.008656f,-0.007940f,
  0.001188f,-0.001123f,-0.001051f,0.000987f,
  -0.001055f,-0.000931f,0.001001f,0.000985f,
  -0.002049f,-0.002049f,-0.002049f,-0.002049f,
  0.000056f,-0.000005f,0.000006f,-0.000056f,
  0.000100f,-0.000100f,0.000055f,-0.000054f,
  0.001152f,-0.001220f,0.001394f,-0.001326f;

  Q << 
  100.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,100.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,100.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,400.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,2.040816f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,2.040816f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,25.000000f;

  R << 
  100.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,100.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,100.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,100.000000f;

  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT, &A, &B, 0);
  tiny_InitSettings(&stgs);
  stgs.rho_init = 30.0;  // Important (select offline, associated with precomp.)
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  
  // Fill in the remaining struct 
  tiny_InitWorkspaceTemp(&work, &Qu, ZU, ZU_new, 0, 0);
  tiny_InitPrimalCache(&work, &Quu_inv, &AmBKt, &coeff_d2p);
  tiny_InitSolution(&work, Xhrz, Uhrz, 0, YU, 0, &Kinf, d, &Pinf, p);

  tiny_SetInitialState(&work, &x0);  
  tiny_SetGoalState(&work, Xref, &xg);
  tiny_SetGoalInput(&work, Uref, &ug);

  /* Set up LQR cost */
  tiny_InitDataCost(&work, &Q, q, &R, r, r_tilde);

  // /* Set up constraints */
  tiny_SetInputBound(&work, &Acu, &lcu, &ucu);
  ucu.fill(0.5);
  lcu.fill(-0.5);
  tiny_UpdateLinearCost(&work);

  /* Solver settings */
  stgs.max_iter = 2;           // limit this if needed
  stgs.verbose = 0;
  stgs.check_termination = 1;
  stgs.tol_abs_dual = 5e-2;
  stgs.tol_abs_prim = 5e-2;

  setpoint_z = 0.1f;
  setpoint_x = 0.0f;
  z_sign = 1;

  // isInit = true;
  /* End of MPC initialization */  
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

  // Rule to take-off and land gradually
  // if (RATE_DO_EXECUTE(10, tick)) {    
  //   setpoint_z += z_sign * 0.1f;
  //   if (setpoint_z > 1.0f) z_sign = -1;
  //   if (z_sign == -1 && setpoint_z < 0.2f) setpoint_z = 0.2f;
  //   setpoint_x += 1.0f;
  //   if (setpoint_x > 2.0f) setpoint_x = 2.0f;
  // }

  /* Get goal state (reference) */
  // xg[0]  = setpoint_x; 
  // xg[2]  = setpoint_z; 
  xg(0)  = setpoint->position.x;
  xg(1)  = setpoint->position.y;
  xg(2)  = setpoint->position.z;
  xg(6)  = setpoint->velocity.x;
  xg(7)  = setpoint->velocity.y;
  xg(8)  = setpoint->velocity.z;
  xg(9)  = setpoint->attitudeRate.roll;
  xg(10) = setpoint->attitudeRate.pitch;
  xg(11) = setpoint->attitudeRate.yaw;
  desired_rpy = mkvec(radians(setpoint->attitude.roll), 
                                 radians(setpoint->attitude.pitch), 
                                 radians(setpoint->attitude.yaw));
  attitude = rpy2quat(desired_rpy);
  phi = quat2rp(qnormalize(attitude));  
  xg(3) = phi.x;
  xg(4) = phi.y;
  xg(5) = phi.z;
  
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
  // tiny_SolveAdmm(&work);
  Uhrz[0] = -(Kinf) * (x0 - xg);
  mpcTime = usecTimestamp() - startTimestamp;

  DEBUG_PRINT("Uhrz[0] = [%.2f, %.2f]\n", (double)(Uhrz[0](0)), (double)(Uhrz[0](1)));
  DEBUG_PRINT("ZU[0] = [%.2f, %.2f]\n", (double)(ZU_new[0](0)), (double)(ZU_new[0](1)));
  // DEBUG_PRINT("YU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(YU[0].data[0]), (double)(YU[0].data[1]), (double)(YU[0].data[2]), (double)(YU[0].data[3]));
  // DEBUG_PRINT("info.pri_res: %f\n", (double)(info.pri_res));
  // DEBUG_PRINT("info.dua_res: %f\n", (double)(info.dua_res));
  // result =  info.status_val * info.iter;
  // DEBUG_PRINT("%d %d %d \n", info.status_val, info.iter, mpcTime);
  // DEBUG_PRINT("[%.2f, %.2f, %.2f]\n", (double)(Kinf(0,0)), (double)(A(0,0)), (double)(x0(2)));

  /* Output control */
  if (setpoint->mode.z == modeDisable) {
    control->normalizedForces[0] = 0.0f;
    control->normalizedForces[1] = 0.0f;
    control->normalizedForces[2] = 0.0f;
    control->normalizedForces[3] = 0.0f;
  } else {
    control->normalizedForces[0] = Uhrz[0](0) + u_hover;  // PWM 0..1
    control->normalizedForces[1] = Uhrz[0](1) + u_hover;
    control->normalizedForces[2] = Uhrz[0](2) + u_hover;
    control->normalizedForces[3] = Uhrz[0](3) + u_hover;
  } 

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

LOG_ADD(LOG_FLOAT, x, &x0(0))
LOG_ADD(LOG_FLOAT, y, &x0(1))
LOG_ADD(LOG_FLOAT, z, &x0(2))

LOG_ADD(LOG_FLOAT, roll,  &x0(3))
LOG_ADD(LOG_FLOAT, pitch, &x0(4))
LOG_ADD(LOG_FLOAT, yaw,   &x0(5))

LOG_ADD(LOG_FLOAT, vx, &x0(6))
LOG_ADD(LOG_FLOAT, vy, &x0(7))
LOG_ADD(LOG_FLOAT, vz, &x0(8))

LOG_ADD(LOG_FLOAT, wroll,  &x0(9))
LOG_ADD(LOG_FLOAT, wpitch, &x0(10))
LOG_ADD(LOG_FLOAT, wyaw,   &x0(11))

LOG_ADD(LOG_INT8, result, &result)
LOG_ADD(LOG_UINT32, mpcTime, &mpcTime)

LOG_ADD(LOG_FLOAT, u0, &(Uhrz[0](0)))
LOG_ADD(LOG_FLOAT, u1, &(Uhrz[0].data()[0]))
LOG_ADD(LOG_FLOAT, u2, &(Uhrz[0](2)))
LOG_ADD(LOG_FLOAT, u3, &(Uhrz[0](3)))

LOG_ADD(LOG_FLOAT, yu0, &(YU[0](0)))
LOG_ADD(LOG_FLOAT, yu1, &(YU[0](1)))
LOG_ADD(LOG_FLOAT, yu2, &(YU[0](2)))
LOG_ADD(LOG_FLOAT, yu3, &(YU[0](3)))

LOG_GROUP_STOP(ctrlMPC)

#ifdef __cplusplus
}
#endif
