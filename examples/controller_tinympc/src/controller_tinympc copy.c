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

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "CONTROLLER_TINYMPC"
#include "debug.h"

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
static sfloat A_data[NSTATES*NSTATES] = {
  1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,-0.004234f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,-0.392400f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.004234f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.392400f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.020000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.020000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.020000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,-0.000014f,0.000000f,0.010000f,0.000000f,0.000000f,0.000000f,-0.001807f,0.000000f,1.000000f,0.000000f,0.000000f,
  0.000014f,0.000000f,0.000000f,0.000000f,0.010000f,0.000000f,0.001807f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.010000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,
};

static sfloat B_data[NSTATES*NINPUTS] = {
  -0.000019f,0.000019f,0.000962f,-0.027012f,-0.027145f,0.001937f,-0.002989f,0.002974f,0.096236f,-5.402457f,-5.428992f,0.387450f,
  0.000021f,0.000021f,0.000962f,-0.029747f,0.029850f,-0.000709f,0.003287f,0.003275f,0.096236f,-5.949452f,5.969943f,-0.141728f,
  0.000019f,-0.000019f,0.000962f,0.027043f,0.027230f,-0.002731f,0.002998f,-0.002978f,0.096236f,5.408501f,5.445914f,-0.546295f,
  -0.000021f,-0.000021f,0.000962f,0.029717f,-0.029934f,0.001503f,-0.003296f,-0.003272f,0.096236f,5.943408f,-5.986864f,0.300572f,
};

static sfloat f_data[NSTATES] = {0};

// Create data array, all zero initialization
static sfloat x0_data[NSTATES] = {0.0f};       // initial state
static sfloat xg_data[NSTATES] = {0.0f};       // goal state (if not tracking)
static sfloat ug_data[NINPUTS] = {0.0f};       // goal input 
static sfloat X_data[NSTATES * NHORIZON] = {0.0f};        // X in MPC solve
static sfloat U_data[NINPUTS * (NHORIZON - 1)] = {0.0f};  // U in MPC solve
static sfloat d_data[NINPUTS * (NHORIZON - 1)] = {0.0f};
static sfloat p_data[NSTATES * NHORIZON] = {0.0f};
// static sfloat Q_data[NSTATES * NSTATES] = {0.0f};
// static sfloat R_data[NINPUTS * NINPUTS] = {0.0f};
static sfloat q_data[NSTATES*(NHORIZON-1)] = {0.0f};
static sfloat r_data[NINPUTS*(NHORIZON-1)] = {0.0f};
static sfloat r_tilde_data[NINPUTS*(NHORIZON-1)] = {0.0f};
static sfloat Acu_data[NINPUTS * NINPUTS] = {0.0f};  
static sfloat YU_data[NINPUTS * (NHORIZON - 1)] = {0.0f};
static sfloat umin_data[NINPUTS] = {0.0f};
static sfloat umax_data[NINPUTS] = {0.0f};
static sfloat temp_data[NINPUTS + 2*NINPUTS*(NHORIZON - 1)] = {0.0f};

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
static Matrix f;

// Create TinyMPC struct
tiny_Model model;
tiny_AdmmSettings stgs;
tiny_AdmmData data;
tiny_AdmmInfo info;
tiny_AdmmSolution soln;
tiny_AdmmWorkspace work;

// Helper variables
static bool isInit = false;  // fix for tracking problem
#define U_HOVER (36.0f / 60.0f);  // pwm, = weight/max thrust 
float setpoint_z = 0.1f;
float setpoint_x = 0.0f;
int z_sign = 1;
int8_t result = 0;
void controllerOutOfTreeInit(void) {
  // if (isInit) {
  //   return;
  // }

  /* Start MPC initialization*/
  
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT);
  tiny_InitSettings(&stgs);

  stgs.rho_init = 50.0f;  // IMPORTANT (select offline, associated with precomp.)

  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  
  // Fill in the remaining struct 
  tiny_InitWorkspaceTempData(&work, ZU, ZU_new, 0, 0, temp_data);
  tiny_InitPrimalCache(&work, Quu_inv_data, AmBKt_data, coeff_d2p_data);

  tiny_InitModelFromArray(&model, &A, &B, &f, A_data, B_data, f_data);
  tiny_InitSolnTrajFromArray(&work, X, U, X_data, U_data);
  tiny_InitSolnGainsFromArray(&work, d, p, d_data, p_data, Kinf_data, Pinf_data);
  tiny_InitSolnDualsFromArray(&work, 0, YU, 0, YU_data, 0);

  tiny_SetInitialState(&work, x0_data);  
  tiny_SetGoalReference(&work, Xref, Uref, xg_data, ug_data);

  // Set up LQR cost 
  tiny_InitDataQuadCostFromArray(&work, Q_data, R_data);
  // sfloat Qdiag[NSTATES] = {10, 10, 10, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  // slap_SetDiagonal(data.Q, Qdiag, NSTATES);
  // slap_SetIdentity(data.R, 1);
  slap_AddIdentity(data.R, work.rho); // \tilde{R}
  tiny_InitDataLinearCostFromArray(&work, q, r, r_tilde, q_data, r_data, r_tilde_data);

  // Set up constraints 
  tiny_SetInputBound(&work, Acu_data, umin_data, umax_data);
  slap_SetConst(data.ucu, 0.5);   // UPPER CONTROL BOUND 
  slap_SetConst(data.lcu, -0.5);  // LOWER CONTROL BOUND 

  // Initialize linear cost (for tracking)
  tiny_UpdateLinearCost(&work);

  // Solver settings 
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 0;
  stgs.max_iter = 5;           // limit this if needed
  stgs.verbose = 0;
  stgs.check_termination = 1;
  stgs.tol_abs_dual = 10e-2;
  stgs.tol_abs_prim = 10e-2;

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

  // Rule to take-off and land gradually
  // if (RATE_DO_EXECUTE(10, tick)) {    
  //   setpoint_z += z_sign * 0.1f;
  //   if (setpoint_z > 1.0f) z_sign = -1;
  //   if (z_sign == -1 && setpoint_z < 0.2f) setpoint_z = 0.2f;
  //   setpoint_x += 1.0f;
  //   if (setpoint_x > 2.0f) setpoint_x = 2.0f;
  // }

  /* Get goal state (reference) */
  // xg_data[0]  = setpoint_x; 
  // xg_data[2]  = setpoint_z; 
  xg_data[0]  = setpoint->position.x;
  xg_data[1]  = setpoint->position.y;
  xg_data[2]  = setpoint->position.z;
  xg_data[6]  = setpoint->velocity.x;
  xg_data[7]  = setpoint->velocity.y;
  xg_data[8]  = setpoint->velocity.z;
  xg_data[9]  = setpoint->attitudeRate.roll;
  xg_data[10] = setpoint->attitudeRate.pitch;
  xg_data[11] = setpoint->attitudeRate.yaw;
  struct vec desired_rpy = mkvec(radians(setpoint->attitude.roll), 
                                 radians(setpoint->attitude.pitch), 
                                 radians(setpoint->attitude.yaw));
  struct quat attitude = rpy2quat(desired_rpy);
  struct vec phi = quat2rp(qnormalize(attitude));  
  xg_data[3] = phi.x;
  xg_data[4] = phi.y;
  xg_data[5] = phi.z;

  // Get current time
  // uint64_t startTimestamp = usecTimestamp();
  
  /* Get current state (initial state for MPC) */
  // delta_x = x - x_bar; x_bar = 0
  // Positon error, [m]
  x0_data[0] = state->position.x;
  x0_data[1] = state->position.y;
  x0_data[2] = state->position.z;
  // Body velocity error, [m/s]                          
  x0_data[6] = state->velocity.x;
  x0_data[7] = state->velocity.y;
  x0_data[8] = state->velocity.z;
  // Angular rate error, [rad/s]
  x0_data[9]  = radians(sensors->gyro.x);   
  x0_data[10] = radians(sensors->gyro.y);
  x0_data[11] = radians(sensors->gyro.z);
  attitude = mkquat(
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
    state->attitudeQuaternion.w);  // current attitude
  phi = quat2rp(qnormalize(attitude));  // quaternion to Rodriquez parameters  
  // Attitude error
  x0_data[3] = phi.x;
  x0_data[4] = phi.y;
  x0_data[5] = phi.z;

  /* MPC solve */
  
  // Warm-start by previous solution  // TODO: should I warm-start U with previous ZU
  // tiny_ShiftFill(U, T_ARRAY_SIZE(U));

  // Solve optimization problem using ADMM
  tiny_UpdateLinearCost(&work);
  tiny_SolveAdmm(&work);
  // MatMulAdd(U[0], soln.Kinf, data.x0, -1, 0);
  // uint32_t mpcTime = usecTimestamp() - startTimestamp;

  // DEBUG_PRINT("U[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(U[0].data[0]), (double)(U[0].data[1]), (double)(U[0].data[2]), (double)(U[0].data[3]));
  // DEBUG_PRINT("ZU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(ZU[0].data[0]), (double)(ZU[0].data[1]), (double)(ZU[0].data[2]), (double)(ZU[0].data[3]));
  // DEBUG_PRINT("YU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(YU[0].data[0]), (double)(YU[0].data[1]), (double)(YU[0].data[2]), (double)(YU[0].data[3]));
  // DEBUG_PRINT("info.pri_res: %f\n", (double)(info.pri_res));
  // DEBUG_PRINT("info.dua_res: %f\n", (double)(info.dua_res));
  result =  info.status_val * info.iter;
  // DEBUG_PRINT("%d %d %d \n", info.status_val, info.iter, mpcTime);
  // DEBUG_PRINT("[%.2f, %.2f, %.2f]\n", (double)(x0_data[0]), (double)(x0_data[1]), (double)(x0_data[2]));

  /* Output control */
  if (setpoint->mode.z == modeDisable) {
    control->normalizedForces[0] = 0.0f;
    control->normalizedForces[1] = 0.0f;
    control->normalizedForces[2] = 0.0f;
    control->normalizedForces[3] = 0.0f;
  } else {
    control->normalizedForces[0] = U[0].data[0] + U_HOVER;  // PWM 0..1
    control->normalizedForces[1] = U[0].data[1] + U_HOVER;
    control->normalizedForces[2] = U[0].data[2] + U_HOVER;
    control->normalizedForces[3] = U[0].data[3] + U_HOVER;
  } 

  // control->normalizedForces[0] = 0.0f;
  // control->normalizedForces[1] = 0.0f;
  // control->normalizedForces[2] = 0.0f;
  // control->normalizedForces[3] = 0.0f;

  control->controlMode = controlModePWM;
}

/**
 * Logging variables for the command and reference signals for the
 * MPC controller
 */

LOG_GROUP_START(ctrlMPC)

LOG_ADD(LOG_FLOAT, x, &x0_data[0])
LOG_ADD(LOG_FLOAT, y, &x0_data[1])
LOG_ADD(LOG_FLOAT, z, &x0_data[2])

LOG_ADD(LOG_FLOAT, roll,  &x0_data[3])
LOG_ADD(LOG_FLOAT, pitch, &x0_data[4])
LOG_ADD(LOG_FLOAT, yaw,   &x0_data[5])

LOG_ADD(LOG_FLOAT, vx, &x0_data[6])
LOG_ADD(LOG_FLOAT, vy, &x0_data[7])
LOG_ADD(LOG_FLOAT, vz, &x0_data[8])

LOG_ADD(LOG_FLOAT, wroll,  &x0_data[9])
LOG_ADD(LOG_FLOAT, wpitch, &x0_data[10])
LOG_ADD(LOG_FLOAT, wyaw,   &x0_data[11])

LOG_ADD(LOG_INT8, result, &result)

LOG_ADD(LOG_FLOAT, u0, &(U_data[0]))
LOG_ADD(LOG_FLOAT, u1, &(U_data[1]))
LOG_ADD(LOG_FLOAT, u2, &(U_data[2]))
LOG_ADD(LOG_FLOAT, u3, &(U_data[3]))

LOG_ADD(LOG_FLOAT, yu0, &(YU_data[0]))
LOG_ADD(LOG_FLOAT, yu1, &(YU_data[1]))
LOG_ADD(LOG_FLOAT, yu2, &(YU_data[2]))
LOG_ADD(LOG_FLOAT, yu3, &(YU_data[3]))

LOG_GROUP_STOP(ctrlMPC)