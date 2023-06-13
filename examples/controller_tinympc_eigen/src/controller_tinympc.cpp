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
#define DT 0.002f       // dt
// #define NSTATES 12    // no. of states (error state)
// #define NINPUTS 4     // no. of controls
#define NHORIZON 3   // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define MPC_RATE RATE_250_HZ  // control frequency

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

static VectorNf Xref[NHORIZON];
static VectorMf Uref[NHORIZON-1];

static MatrixMf Acu;
static VectorMf ucu;
static VectorMf lcu;

static VectorMf Qu;
static VectorMf ZU[NHORIZON-1]; 
static VectorMf ZU_new[NHORIZON-1];

static VectorNf x0;
static VectorNf xg = (VectorNf() << 0, 0, 0.5, 0, 0, 0, 0, 0, 0, 0, 0, 0).finished();
static VectorMf ug = (VectorMf() << 0, 0, 0, 0).finished();

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
static float u_hover = 0.67f;

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

  // Precompute/Cache
  A << 
  1.000000f,0.000000f,0.000000f,0.000000f,0.000157f,-0.000000f,0.004000f,0.000000f,0.000000f,0.000000f,0.000000f,-0.000000f,
  0.000000f,1.000000f,0.000000f,-0.000157f,0.000000f,-0.000000f,0.000000f,0.004000f,0.000000f,-0.000000f,0.000000f,-0.000000f,
  0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.004000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.002000f,-0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,-0.000000f,1.000000f,-0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.002000f,-0.000000f,
  0.000000f,0.000000f,0.000000f,-0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,-0.000000f,0.000000f,0.002000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.078480f,-0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000078f,-0.000000f,
  0.000000f,0.000000f,0.000000f,-0.078480f,0.000000f,-0.000000f,0.000000f,1.000000f,0.000000f,-0.000078f,0.000000f,-0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,-0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,-0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,-0.000000f,0.000000f,1.000000f;

  B << 
  -0.000000f,0.000000f,0.000000f,-0.000000f,
  0.000000f,0.000000f,-0.000000f,-0.000000f,
  0.000034f,0.000034f,0.000034f,0.000034f,
  -0.001101f,-0.001213f,0.001103f,0.001212f,
  -0.001107f,0.001217f,0.001110f,-0.001221f,
  0.000079f,-0.000029f,-0.000111f,0.000061f,
  -0.000029f,0.000032f,0.000029f,-0.000032f,
  0.000029f,0.000032f,-0.000029f,-0.000032f,
  0.016817f,0.016817f,0.016817f,0.016817f,
  -1.101418f,-1.212936f,1.102651f,1.211704f,
  -1.106828f,1.217114f,1.110278f,-1.220564f,
  0.078991f,-0.028895f,-0.111375f,0.061279f;

  Kinf << 
  -0.219651f,0.215086f,0.214454f,-0.621733f,-0.673423f,-0.712498f,-0.126429f,0.121002f,0.126804f,-0.046103f,-0.055218f,-0.234805f,
  0.219136f,0.206687f,0.214454f,-0.543327f,0.661569f,0.729705f,0.125272f,0.112190f,0.126804f,-0.033172f,0.053660f,0.231969f,
  0.200889f,-0.207553f,0.214454f,0.556106f,0.509516f,-0.771945f,0.107802f,-0.113537f,0.126804f,0.034784f,0.026928f,-0.224824f,
  -0.200374f,-0.214220f,0.214454f,0.608954f,-0.497662f,0.754737f,-0.106646f,-0.119655f,0.126804f,0.044490f,-0.025370f,0.227660f;

  Pinf << 
  13695.647678f,-45.504380f,-0.000000f,142.291407f,8185.845738f,1478.968921f,3236.669483f,-26.781119f,-0.000000f,9.468293f,93.978690f,239.245176f,
  -45.504380f,13648.195042f,0.000000f,-8016.818097f,-142.943568f,-589.785445f,-26.832673f,3206.924076f,0.000000f,-82.252377f,-9.564855f,-96.068401f,
  -0.000000f,-0.000000f,15819.771958f,-0.000000f,-0.000000f,-0.000000f,-0.000000f,0.000000f,5055.767687f,-0.000000f,-0.000000f,-0.000000f,
  142.291407f,-8016.818097f,-0.000000f,20203.734151f,622.398986f,3575.640208f,96.867922f,-4255.728553f,-0.000000f,238.969101f,62.173621f,816.904211f,
  8185.845738f,-142.943568f,-0.000000f,622.398986f,21137.164711f,8952.778900f,4385.796807f,-97.088586f,-0.000000f,62.018696f,344.583199f,2039.917409f,
  1478.968921f,-589.785445f,-0.000000f,3575.640208f,8952.778900f,158827.873909f,1190.704277f,-475.247428f,-0.000000f,447.411966f,1119.431498f,18353.272774f,
  3236.669483f,-26.832673f,-0.000000f,96.867922f,4385.796807f,1190.704277f,1370.876664f,-16.758323f,-0.000000f,7.895120f,58.197874f,233.837783f,
  -26.781119f,3206.924076f,0.000000f,-4255.728553f,-97.088586f,-475.247428f,-16.758323f,1350.466302f,0.000000f,-46.312079f,-7.940061f,-93.735950f,
  -0.000000f,-0.000000f,5055.767687f,-0.000000f,-0.000000f,-0.000000f,-0.000000f,0.000000f,2984.931622f,-0.000000f,-0.000000f,-0.000000f,
  9.468293f,-82.252377f,-0.000000f,238.969101f,62.018696f,447.411966f,7.895120f,-46.312079f,-0.000000f,19.242630f,9.537939f,139.810771f,
  93.978690f,-9.564855f,-0.000000f,62.173621f,344.583199f,1119.431498f,58.197874f,-7.940061f,-0.000000f,9.537939f,37.466484f,349.405577f,
  239.245176f,-96.068401f,-0.000000f,816.904211f,2039.917409f,18353.272774f,233.837783f,-93.735950f,-0.000000f,139.810771f,349.405577f,5677.686139f;

  Quu_inv << 
  0.002299f,0.000006f,0.000209f,0.000002f,
  0.000006f,0.002285f,0.000002f,0.000223f,
  0.000209f,0.000002f,0.002299f,0.000006f,
  0.000002f,0.000223f,0.000006f,0.002285f;

  AmBKt << 
  1.000000f,-0.000000f,-0.000000f,0.000045f,-0.000977f,0.000058f,-0.000026f,-0.000001f,-0.000000f,0.045153f,-0.977441f,0.058335f,
  -0.000000f,1.000000f,0.000000f,0.000976f,-0.000045f,-0.000021f,-0.000001f,-0.000026f,0.000000f,0.976027f,-0.044527f,-0.021007f,
  0.000000f,-0.000000f,0.999971f,-0.000000f,0.000000f,-0.000000f,0.000000f,-0.000000f,-0.014426f,0.000000f,0.000000f,-0.000000f,
  0.000000f,-0.000157f,-0.000000f,0.997305f,0.000099f,0.000058f,0.000003f,-0.078410f,-0.000000f,-2.694873f,0.098973f,0.058032f,
  0.000157f,-0.000000f,-0.000000f,0.000102f,0.997276f,0.000160f,0.078409f,-0.000003f,-0.000000f,0.101922f,-2.723700f,0.159553f,
  0.000000f,-0.000000f,-0.000000f,0.000037f,0.000102f,0.999945f,0.000003f,-0.000001f,-0.000000f,0.036994f,0.101531f,-0.054859f,
  0.004000f,-0.000000f,-0.000000f,0.000023f,-0.000542f,0.000032f,0.999986f,-0.000001f,-0.000000f,0.023052f,-0.542264f,0.032148f,
  -0.000000f,0.004000f,0.000000f,0.000540f,-0.000023f,-0.000012f,-0.000001f,0.999986f,0.000000f,0.539531f,-0.022607f,-0.011629f,
  0.000000f,-0.000000f,0.003983f,-0.000000f,0.000000f,-0.000000f,0.000000f,-0.000000f,0.991470f,-0.000000f,0.000000f,-0.000000f,
  0.000000f,-0.000000f,0.000000f,0.001817f,0.000005f,0.000004f,0.000000f,-0.000074f,-0.000000f,0.816722f,0.005029f,0.003831f,
  0.000000f,-0.000000f,-0.000000f,0.000005f,0.001813f,0.000010f,0.000074f,-0.000000f,-0.000000f,0.005317f,0.812708f,0.010466f,
  -0.000000f,0.000000f,-0.000000f,-0.000005f,-0.000015f,0.001986f,-0.000000f,0.000000f,-0.000000f,-0.005210f,-0.014731f,0.986260f;

  coeff_d2p << 
  0.173971f,-0.173787f,-0.236941f,0.236757f,
  -0.192299f,-0.216926f,0.217494f,0.191731f,
  -0.331731f,-0.331731f,-0.331731f,-0.331731f,
  0.374062f,0.329949f,-0.345903f,-0.358109f,
  0.406969f,-0.391493f,-0.336669f,0.321193f,
  0.852457f,-0.768968f,0.563511f,-0.647000f,
  0.089226f,-0.088619f,-0.117937f,0.117331f,
  -0.097128f,-0.107824f,0.108610f,0.096342f,
  -0.318491f,-0.318491f,-0.318491f,-0.318491f,
  0.009587f,-0.002185f,0.001712f,-0.009114f,
  0.018174f,-0.017662f,0.010051f,-0.010563f,
  0.232927f,-0.230490f,0.226110f,-0.228547f;

  Q << 
  100.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,100.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,100.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
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

  // setpoint_z = 0.1f;
  // setpoint_x = 0.0f;
  // z_sign = 1;

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
  xg[0]  = setpoint->position.x;
  xg[1]  = setpoint->position.y;
  xg[2]  = setpoint->position.z;
  xg[6]  = setpoint->velocity.x;
  xg[7]  = setpoint->velocity.y;
  xg[8]  = setpoint->velocity.z;
  xg[9]  = radians(setpoint->attitudeRate.roll);
  xg[10] = radians(setpoint->attitudeRate.pitch);
  xg[11] = radians(setpoint->attitudeRate.yaw);
  struct vec desired_rpy = mkvec(radians(setpoint->attitude.roll), 
                                 radians(setpoint->attitude.pitch), 
                                 radians(setpoint->attitude.yaw));
  struct quat attitude = rpy2quat(desired_rpy);
  struct vec phi = quat2rp(qnormalize(attitude));  
  xg[3] = phi.x;
  xg[4] = phi.y;
  xg[5] = phi.z;
  
  /* Get current state (initial state for MPC) */
  // delta_x = x - x_bar; x_bar = 0
  // Positon error, [m]
  x0[0] = state->position.x;
  x0[1] = state->position.y;
  x0[2] = state->position.z;
  // Body velocity error, [m/s]                          
  x0[6] = state->velocity.x;
  x0[7] = state->velocity.y;
  x0[8] = state->velocity.z;
  // Angular rate error, [rad/s]
  x0[9]  = radians(sensors->gyro.x);   
  x0[10] = radians(sensors->gyro.y);
  x0[11] = radians(sensors->gyro.z);
  attitude = mkquat(
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
    state->attitudeQuaternion.w);  // current attitude
  phi = quat2rp(qnormalize(attitude));  // quaternion to Rodriquez parameters  
  // Attitude error
  x0[3] = phi.x;
  x0[4] = phi.y;
  x0[5] = phi.z;

  /* MPC solve */
  
  // Warm-start by previous solution  // TODO: should I warm-start U with previous ZU
  // tiny_ShiftFill(U, T_ARRAY_SIZE(U));

  // Solve optimization problem using ADMM
  tiny_UpdateLinearCost(&work);
  tiny_SolveAdmm(&work);
  // DEBUG_PRINT("[%.2f, %.2f, %.2f]\n", (double)(Kinf(0,0)), (double)(xg(1)), (double)(xg(2)));
  // Uhrz[0] = -(Kinf) * (x0 - xg);
  mpcTime = usecTimestamp() - startTimestamp;

  // DEBUG_PRINT("Uhrz[0] = [%.2f, %.2f]\n", (double)(Uhrz[0](0)), (double)(Uhrz[0](1)));
  // DEBUG_PRINT("ZU[0] = [%.2f, %.2f]\n", (double)(ZU_new[0](0)), (double)(ZU_new[0](1)));
  // DEBUG_PRINT("YU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(YU[0].data[0]), (double)(YU[0].data[1]), (double)(YU[0].data[2]), (double)(YU[0].data[3]));
  // DEBUG_PRINT("info.pri_res: %f\n", (double)(info.pri_res));
  // DEBUG_PRINT("info.dua_res: %f\n", (double)(info.dua_res));
  // result =  info.status_val * info.iter;
  // DEBUG_PRINT("%d %d %d \n", info.status_val, info.iter, mpcTime);
  
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
  // DEBUG_PRINT("pwm = [%.2f, %.2f]\n", (double)(control->normalizedForces[0]), (double)(control->normalizedForces[2]));

  // control->normalizedForces[0] = 0.0f;
  // control->normalizedForces[1] = 0.0f;
  // control->normalizedForces[2] = 0.0f;
  // control->normalizedForces[3] = 0.0f;

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

// LOG_ADD(LOG_FLOAT, yu0, &(YU[0](0)))
// LOG_ADD(LOG_FLOAT, yu1, &(YU[0](1)))
// LOG_ADD(LOG_FLOAT, yu2, &(YU[0](2)))
// LOG_ADD(LOG_FLOAT, yu3, &(YU[0](3)))

LOG_GROUP_STOP(ctrlMPC)

#ifdef __cplusplus
}
#endif
