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
 * Test waypoint storage
 */

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
#define DT 0.002f       // dt
#define NSTATES 12    // no. of states (error state)
#define NINPUTS 4     // no. of controls
#define NHORIZON 5   // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define MPC_RATE RATE_500_HZ  // control frequency

#include "params_500hz.h"
#include "traj_swerve.h"
// #include "traj_fig8.h"

/* Allocate global variables for MPC */
static sfloat f_data[NSTATES] = {0};

// Create data array, all zero initialization
static sfloat x0_data[NSTATES] = {0.0f};       // initial state
static sfloat xg_data[NSTATES] = {0.0f};       // goal state (if not tracking)
static sfloat ug_data[NINPUTS] = {0.0f};       // goal input 
static sfloat Xref_data[NSTATES * NHORIZON] = {0};
static sfloat X_data[NSTATES * NHORIZON] = {0.0f};        // X in MPC solve
static sfloat U_data[NINPUTS * (NHORIZON - 1)] = {0.0f};  // U in MPC solve
static sfloat d_data[NINPUTS * (NHORIZON - 1)] = {0.0f};
static sfloat p_data[NSTATES * NHORIZON] = {0.0f};
static sfloat q_data[NSTATES*(NHORIZON-1)] = {0.0f};
static sfloat r_data[NINPUTS*(NHORIZON-1)] = {0.0f};
static sfloat r_tilde_data[NINPUTS*(NHORIZON-1)] = {0.0f};
static sfloat Acu_data[NINPUTS * NINPUTS] = {0.0f};  
static sfloat YU_data[NINPUTS * (NHORIZON - 1)] = {0.0f};
static sfloat umin_data[NINPUTS] = {0.0f};
static sfloat umax_data[NINPUTS] = {0.0f};
static sfloat temp_data[NINPUTS + 2*NINPUTS*(NHORIZON - 1)] = {0.0f};

// Created matrices
static Matrix Xref[NHORIZON];
static Matrix Uref[NHORIZON - 1];
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
static tiny_Model model;
static tiny_AdmmSettings stgs;
static tiny_AdmmData data;
static tiny_AdmmInfo info;
static tiny_AdmmSolution soln;
static tiny_AdmmWorkspace work;

// Helper variables
static bool isInit = false;  // fix for tracking problem
static uint32_t mpcTime = 0;
static float u_hover = 0.67f;
static int8_t result = 0;
static uint32_t step = 0;
static bool en_traj = false;
static uint32_t traj_length = T_ARRAY_SIZE(X_ref_data) / 12;
static int8_t user_traj_iter = 1;  // number of times to execute full trajectory
static int8_t traj_hold = 1;  // hold current trajectory for this no of steps
static int8_t traj_iter = 0;
static uint32_t traj_idx = 0;

void controllerOutOfTreeInit(void) {
  /* Start MPC initialization*/
  
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT);
  tiny_InitSettings(&stgs);

  stgs.rho_init = 250.0f;  // IMPORTANT (select offline, associated with precomp.)

  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  
  // Fill in the remaining struct 
  tiny_InitWorkspaceTempData(&work, ZU, ZU_new, 0, 0, temp_data);
  tiny_InitPrimalCache(&work, Quu_inv_data, AmBKt_data, coeff_d2p_data);

  tiny_InitModelFromArray(&model, &A, &B, &f, A_data, B_data, f_data);
  tiny_InitSolnTrajFromArray(&work, X, U, X_data, U_data);
  tiny_InitSolnGainsFromArray(&work, d, p, d_data, p_data, Kinf_data, Pinf_data);
  tiny_InitSolnDualsFromArray(&work, 0, YU, 0, YU_data, 0);

  tiny_SetInitialState(&work, x0_data);  
  // tiny_SetGoalReference(&work, Xref, Uref, xg_data, ug_data);

  data.Xref = Xref;
  data.Uref = Uref;
  for (int i = 0; i < NHORIZON; ++i) {
    if (i < NHORIZON - 1) {
      Uref[i] = slap_MatrixFromArray(NINPUTS, 1, ug_data);
    }
    Xref[i] = slap_MatrixFromArray(NSTATES, 1, &X_ref_data[i * NSTATES]);
  }

  // Set up LQR cost 
  tiny_InitDataQuadCostFromArray(&work, Q_data, R_data);
  slap_AddIdentity(data.R, work.rho); // \tilde{R}
  tiny_InitDataLinearCostFromArray(&work, q, r, r_tilde, q_data, r_data, r_tilde_data);

  // Set up constraints 
  tiny_SetInputBound(&work, Acu_data, umin_data, umax_data);
  slap_SetConst(data.ucu, (1 - u_hover));   // UPPER CONTROL BOUND 
  slap_SetConst(data.lcu, (-u_hover));  // LOWER CONTROL BOUND 

  // Initialize linear cost (for tracking)
  tiny_UpdateLinearCost(&work);

  // Solver settings 
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 0;
  stgs.max_iter = 6;           // limit this if needed
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
  uint64_t startTimestamp = usecTimestamp();

  // Update reference: 3 positions, k counts each MPC step
  if (en_traj) {
    if (step % traj_hold == 0) {
      traj_idx = (int)(step / traj_hold);
      for (int i = 0; i < NHORIZON; ++i) {
        (Xref[i]).data = &(X_ref_data[traj_idx * NSTATES]); 
      }
    }
  }
  
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
  struct quat attitude = mkquat(
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
    state->attitudeQuaternion.w);  // current attitude
  struct vec phi = quat2rp(qnormalize(attitude));  // quaternion to Rodriquez parameters  
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

  // // JUST LQR
  // MatAdd(data.x0, data.x0, Xref[0], -1);
  // MatMulAdd(U[0], soln.Kinf, data.x0, -1, 0);

  mpcTime = usecTimestamp() - startTimestamp;

  // DEBUG_PRINT("U[0] = [%.2f, %.2f]\n", (double)(U[0].data[0]), (double)(U[0].data[1]));

  // DEBUG_PRINT("U[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(U[0].data[0]), (double)(U[0].data[1]), (double)(U[0].data[2]), (double)(U[0].data[3]));
  // DEBUG_PRINT("ZU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(ZU[0].data[0]), (double)(ZU[0].data[1]), (double)(ZU[0].data[2]), (double)(ZU[0].data[3]));
  // DEBUG_PRINT("YU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(YU[0].data[0]), (double)(YU[0].data[1]), (double)(YU[0].data[2]), (double)(YU[0].data[3]));
  // DEBUG_PRINT("info.pri_res: %f\n", (double)(info.pri_res));
  // DEBUG_PRINT("info.dua_res: %f\n", (double)(info.dua_res));
  // result =  info.status_val * info.iter;
  DEBUG_PRINT("%d %d %d %d \n", step, info.status_val, info.iter, mpcTime);
  // DEBUG_PRINT("%d\n", mpcTime);
  // DEBUG_PRINT("[%.2f, %.2f, %.2f]\n", (double)(x0_data[0]), (double)(x0_data[1]), (double)(x0_data[2]));

  /* Output control */
  if (setpoint->mode.z == modeDisable) {
    control->normalizedForces[0] = 0.0f;
    control->normalizedForces[1] = 0.0f;
    control->normalizedForces[2] = 0.0f;
    control->normalizedForces[3] = 0.0f;
  } else {
    control->normalizedForces[0] = U[0].data[0] + u_hover;  // PWM 0..1
    control->normalizedForces[1] = U[0].data[1] + u_hover;
    control->normalizedForces[2] = U[0].data[2] + u_hover;
    control->normalizedForces[3] = U[0].data[3] + u_hover;
  } 
  // DEBUG_PRINT("pwm = [%.2f, %.2f]\n", (double)(control->normalizedForces[0]), (double)(control->normalizedForces[2]));
  // control->normalizedForces[0] = 0.0f;
  // control->normalizedForces[1] = 0.0f;
  // control->normalizedForces[2] = 0.0f;
  // control->normalizedForces[3] = 0.0f;

  control->controlMode = controlModePWM;
  
  // stop trajectory executation
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
PARAM_GROUP_START(ctrlMPC)

PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, uHover, &u_hover)
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, trajLength, &traj_length)
PARAM_ADD_CORE(PARAM_INT8 | PARAM_PERSISTENT, trajHold, &traj_hold)
PARAM_ADD_CORE(PARAM_INT8 | PARAM_PERSISTENT, trajIter, &user_traj_iter)
PARAM_ADD_CORE(PARAM_INT8 | PARAM_PERSISTENT, stgs_cstr_inputs, &(stgs.en_cstr_inputs))
PARAM_ADD_CORE(PARAM_INT8 | PARAM_PERSISTENT, stgs_max_iter, &(stgs.max_iter))

PARAM_GROUP_STOP(ctrlMPC)

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
LOG_ADD(LOG_UINT32, mpcTime, &mpcTime)

LOG_ADD(LOG_FLOAT, u0, &(U_data[0]))
LOG_ADD(LOG_FLOAT, u1, &(U_data[1]))
LOG_ADD(LOG_FLOAT, u2, &(U_data[2]))
LOG_ADD(LOG_FLOAT, u3, &(U_data[3]))

LOG_ADD(LOG_FLOAT, yu0, &(YU_data[0]))
LOG_ADD(LOG_FLOAT, yu1, &(YU_data[1]))
LOG_ADD(LOG_FLOAT, yu2, &(YU_data[2]))
LOG_ADD(LOG_FLOAT, yu3, &(YU_data[3]))

LOG_GROUP_STOP(ctrlMPC)

// #ifdef __cplusplus
// }
// #endif
