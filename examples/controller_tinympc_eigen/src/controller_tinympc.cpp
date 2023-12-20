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
using namespace Eigen;

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
#include "eventtrigger.h"

#include "cpp_compat.h"   // needed to compile Cpp to C

#include "tinympc/tinympc.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TINYMPC-E"
#include "debug.h"

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

// Macro variables, model dimensions in tinympc/types.h
#define DT 0.002f       // dt
#define NHORIZON 15   // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define MPC_RATE 500  // control frequency

/* Include trajectory to track */
#include "traj_fig8_12.h"
// #include "traj_circle_500hz.h"
// #include "traj_perching.h"

// Precomputed data and cache, in params_*.h
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

static float Xhrz_log[NHORIZON*12];
static float Uhrz_log[(NHORIZON-1)*4];
static uint32_t iter_log;
static float pri_resid_log;
static float dual_resid_log;
static float ref_x;
static float ref_y;
static float ref_z;
// static float Xhrz_log_x_0;
// static float Xhrz_log_x_1;
// static float Xhrz_log_x_2;
// static float Xhrz_log_x_3;
// static float Xhrz_log_x_4;
// static float Xhrz_log_x_5;
// static float Xhrz_log_x_6;
// static float Xhrz_log_x_7;
// static float Xhrz_log_x_8;
// static float Xhrz_log_x_9;
// static float Xhrz_log_x_10;
// static float Xhrz_log_x_11;
// static float Xhrz_log_x_12;
// static float Xhrz_log_x_13;
// static float Xhrz_log_x_14;

// static float Xhrz_log_y_0;
// static float Xhrz_log_y_1;
// static float Xhrz_log_y_2;
// static float Xhrz_log_y_3;
// static float Xhrz_log_y_4;
// static float Xhrz_log_y_5;
// static float Xhrz_log_y_6;
// static float Xhrz_log_y_7;
// static float Xhrz_log_y_8;
// static float Xhrz_log_y_9;
// static float Xhrz_log_y_10;
// static float Xhrz_log_y_11;
// static float Xhrz_log_y_12;
// static float Xhrz_log_y_13;
// static float Xhrz_log_y_14;

// static float Xhrz_log_z_0;
// static float Xhrz_log_z_1;
// static float Xhrz_log_z_2;
// static float Xhrz_log_z_3;
// static float Xhrz_log_z_4;
// static float Xhrz_log_z_5;
// static float Xhrz_log_z_6;
// static float Xhrz_log_z_7;
// static float Xhrz_log_z_8;
// static float Xhrz_log_z_9;
// static float Xhrz_log_z_10;
// static float Xhrz_log_z_11;
// static float Xhrz_log_z_12;
// static float Xhrz_log_z_13;
// static float Xhrz_log_z_14;


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
// static float u_hover[4] = {0.7479f, 0.6682f, 0.78f, 0.67f};  // cf1
// static float u_hover[4] = {0.5074f, 0.5188f, 0.555f, 0.4914f};  // cf3
static float u_hover[4] = {0.6641f, 0.6246f, 0.7216f, 0.5756f};  // cf1
// static float u_hover[4] = {0.7f, 0.7f, 0.7f, 0.7f};  // cf2 not correct
static int8_t result = 0;
static uint32_t step = 0;
static bool en_traj = false;
static uint32_t traj_length = T_ARRAY_SIZE(X_ref_data);
static int8_t user_traj_iter = 1;  // number of times to execute full trajectory
static int8_t traj_hold = 3;       // hold current trajectory for this no of steps
static int8_t traj_iter = 0;
static uint32_t traj_idx = 0;

static struct vec desired_rpy;
static struct quat attitude;
static struct vec phi;

// declares eventTrigger_[name] and eventTrigger_[name]_payload
// EVENTTRIGGER(traj_ref, int32, traj_en, int32, traj_step, float, x, float, y, float, z);
// EVENTTRIGGER(traj_pos, float, x, float, y, float, z);
// EVENTTRIGGER(traj_vel, float, x, float, y, float, z);
// EVENTTRIGGER(traj_att, float, x, float, y, float, z);
// EVENTTRIGGER(traj_rate, float, x, float, y, float, z);
// EVENTTRIGGER(control, float, u1, float, u2, float, u3, float, u4);
// EVENTTRIGGER(solver_stats, int32, solvetime_us, int32, iters);
  
void updateInitialState(const sensorData_t *sensors, const state_t *state) {
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
}

void updateHorizonReference(const setpoint_t *setpoint) {
  // Update reference: from stored trajectory or commander
  if (en_traj) {
    if (step % traj_hold == 0) {
      traj_idx = (int)(step / traj_hold);
      for (int i = 0; i < NHORIZON; ++i) {
        for (int j = 0; j < 3; ++j) {
          Xref[i](j) = X_ref_data[traj_idx][j];
        }
        for (int j = 6; j < 9; ++j) {
          Xref[i](j) = X_ref_data[traj_idx][j];
        }
        ref_x = X_ref_data[traj_idx][0];
        ref_y = X_ref_data[traj_idx][1];
        ref_z = X_ref_data[traj_idx][2];
        // if (i < NHORIZON - 1) {
        //   for (int j = 0; j < NINPUTS; ++j) {
        //     Uref[i](j) = U_ref_data[traj_idx][j];
        //   }          
        // }
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
    // tiny_SetGoalInput(&work, Uref, &ug);
    // // xg(1) = 1.0;
    // // xg(2) = 2.0;
  }
  // DEBUG_PRINT("z_ref = %.2f\n", (double)(Xref[0](2)));

  // stop trajectory executation
  if (en_traj) {
    if (traj_iter >= user_traj_iter) en_traj = false;

    if (traj_idx >= traj_length - 1 - NHORIZON + 1) { 
      // complete one trajectory
      step = 0; 
      traj_iter += 1;
    } 
    else step += 1;
  }
}

void controllerOutOfTreeInit(void) { 
  /* Start MPC initialization*/

  // Precompute/Cache
  #include "params_500hz.h"

  // End of Precompute/Cache
  traj_length = traj_length * traj_hold;
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT, &A, &B, 0);
  tiny_InitSettings(&stgs);
  stgs.rho_init = 250.0;  // Important (select offline, associated with precomp.)
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  
  // Fill in the remaining struct 
  tiny_InitWorkspaceTemp(&work, &Qu, ZU, ZU_new, 0, 0);
  tiny_InitPrimalCache(&work, &Quu_inv, &AmBKt, &coeff_d2p);
  tiny_InitSolution(&work, Xhrz, Uhrz, 0, YU, 0, &Kinf, d, &Pinf, p);

  tiny_SetInitialState(&work, &x0);  
  tiny_SetStateReference(&work, Xref);
  tiny_SetInputReference(&work, Uref);
  // tiny_SetGoalState(&work, Xref, &xg);
  // tiny_SetGoalInput(&work, Uref, &ug);

  /* Set up LQR cost */
  tiny_InitDataCost(&work, &Q, q, &R, r, r_tilde);
  // R = R + stgs.rho_init * MatrixMf::Identity();
  // /* Set up constraints */
  ucu << 1 - u_hover[0], 1 - u_hover[1], 1 - u_hover[2], 1 - u_hover[3];
  lcu << -u_hover[0], -u_hover[1], -u_hover[2], -u_hover[3];
  tiny_SetInputBound(&work, &Acu, &lcu, &ucu);

  tiny_UpdateLinearCost(&work);

  /* Solver settings */
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 0;
  stgs.max_iter = 6;           // limit this if needed
  stgs.verbose = 0;
  stgs.check_termination = 2;
  stgs.tol_abs_dual = 1e-2;
  stgs.tol_abs_prim = 1e-2;

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
  updateHorizonReference(setpoint);
  /* Get current state (initial state for MPC) */
  // delta_x = x - x_bar; x_bar = 0
  // Positon error, [m]
  updateInitialState(sensors, state);

  /* MPC solve */
  
  // Warm-start by previous solution  // TODO: should I warm-start U with previous ZU
  // tiny_ShiftFill(U, T_ARRAY_SIZE(U));

  // Solve optimization problem using ADMM
  tiny_UpdateLinearCost(&work);
  startTimestamp = usecTimestamp();
  tiny_SolveAdmm(&work);
  // vTaskDelay(M2T(1));
  // tiny_SolveAdmm(&work);

  // // JUST LQR
  // Uhrz[0] = -(Kinf) * (x0 - xg);

  mpcTime = usecTimestamp() - startTimestamp;
 
  // DEBUG_PRINT("Uhrz[0] = [%.2f, %.2f]\n", (double)(Uhrz[0](0)), (double)(Uhrz[0](1)));
  // DEBUG_PRINT("ZU[0] = [%.2f, %.2f]\n", (double)(ZU_new[0](0)), (double)(ZU_new[0](1)));
  // DEBUG_PRINT("YU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(YU[0].data[0]), (double)(YU[0].data[1]), (double)(YU[0].data[2]), (double)(YU[0].data[3]));
  // DEBUG_PRINT("info.pri_res: %f\n", (double)(info.pri_res));
  // DEBUG_PRINT("info.dua_res: %f\n", (double)(info.dua_res));
  result =  info.status_val * info.iter;
  // DEBUG_PRINT("%d %d %d \n", info.status_val, info.iter, mpcTime);
  // DEBUG_PRINT("%.2f, %.2f, %.2f, %.2f \n", (double)(Xref[0](5)), (double)(Uhrz[0](2)), (double)(Uhrz[0](3)), (double)(ZU_new[0](0)));
  
  /* Output control */
  if (setpoint->mode.z == modeDisable) {
    control->normalizedForces[0] = 0.0f;
    control->normalizedForces[1] = 0.0f;
    control->normalizedForces[2] = 0.0f;
    control->normalizedForces[3] = 0.0f;
  } else {
    control->normalizedForces[0] = ZU_new[0](0) + u_hover[0];  // PWM 0..1
    control->normalizedForces[1] = ZU_new[0](1) + u_hover[1];
    control->normalizedForces[2] = ZU_new[0](2) + u_hover[2];
    control->normalizedForces[3] = ZU_new[0](3) + u_hover[3];
  } 
  // DEBUG_PRINT("pwm = [%.2f, %.2f]\n", (double)(control->normalizedForces[0]), (double)(control->normalizedForces[1]));

  // control->normalizedForces[0] = 0.0f;
  // control->normalizedForces[1] = 0.0f;
  // control->normalizedForces[2] = 0.0f;
  // control->normalizedForces[3] = 0.0f;

  control->controlMode = controlModePWM;


  //Save results into c array
  iter_log = info.iter;

  pri_resid_log = info.pri_res;
  dual_resid_log = info.dua_res;


  // Xhrz_log_x_0 = Xhrz[0](0);
  // Xhrz_log_x_1 = Xhrz[1](0);
  // Xhrz_log_x_2 = Xhrz[2](0);
  // Xhrz_log_x_3 = Xhrz[3](0);
  // Xhrz_log_x_4 = Xhrz[4](0);
  // Xhrz_log_x_5 = Xhrz[5](0);
  // Xhrz_log_x_6 = Xhrz[6](0);
  // Xhrz_log_x_7 = Xhrz[7](0);
  // Xhrz_log_x_8 = Xhrz[8](0);
  // Xhrz_log_x_9 = Xhrz[9](0);
  // Xhrz_log_x_10 = Xhrz[10](0);
  // Xhrz_log_x_11 = Xhrz[11](0);
  // Xhrz_log_x_12 = Xhrz[12](0);
  // Xhrz_log_x_13 = Xhrz[13](0);
  // Xhrz_log_x_14 = Xhrz[14](0);

  // Xhrz_log_y_0 = Xhrz[0](1);
  // Xhrz_log_y_1 = Xhrz[1](1);
  // Xhrz_log_y_2 = Xhrz[2](1);
  // Xhrz_log_y_3 = Xhrz[3](1);
  // Xhrz_log_y_4 = Xhrz[4](1);
  // Xhrz_log_y_5 = Xhrz[5](1);
  // Xhrz_log_y_6 = Xhrz[6](1);
  // Xhrz_log_y_7 = Xhrz[7](1);
  // Xhrz_log_y_8 = Xhrz[8](1);
  // Xhrz_log_y_9 = Xhrz[9](1);
  // Xhrz_log_y_10 = Xhrz[10](1);
  // Xhrz_log_y_11 = Xhrz[11](1);
  // Xhrz_log_y_12 = Xhrz[12](1);
  // Xhrz_log_y_13 = Xhrz[13](1);
  // Xhrz_log_y_14 = Xhrz[14](1);

  // Xhrz_log_z_0 = Xhrz[0](2);
  // Xhrz_log_z_1 = Xhrz[1](2);
  // Xhrz_log_z_2 = Xhrz[2](2);
  // Xhrz_log_z_3 = Xhrz[3](2);
  // Xhrz_log_z_4 = Xhrz[4](2);
  // Xhrz_log_z_5 = Xhrz[5](2);
  // Xhrz_log_z_6 = Xhrz[6](2);
  // Xhrz_log_z_7 = Xhrz[7](2);
  // Xhrz_log_z_8 = Xhrz[8](2);
  // Xhrz_log_z_9 = Xhrz[9](2);
  // Xhrz_log_z_10 = Xhrz[10](2);
  // Xhrz_log_z_11 = Xhrz[11](2);
  // Xhrz_log_z_12 = Xhrz[12](2);
  // Xhrz_log_z_13 = Xhrz[13](2);
  // Xhrz_log_z_14 = Xhrz[14](2);


  // for (int i=0; i<NHORIZON; i++) {
  //   for (int j=0; j<12; j++) {
  //     Xhrz_log[i*12 + j] = Xhrz[i](j);
  //   }
  // }
  // for (int i=0; i<NHORIZON-1; i++) {
  //   for (int j=0; j<4; j++) {
  //     Uhrz_log[i*4 + j] = Uhrz[i](j);
  //   }
  // }

  // eventTrigger_traj_ref_payload.traj_en = en_traj;
  // eventTrigger_traj_ref_payload.traj_step = step;
  // eventTrigger_traj_ref_payload.x = Xref[0](0);
  // eventTrigger_traj_ref_payload.y = Xref[0](1);
  // eventTrigger_traj_ref_payload.z = Xref[0](2);
  // eventTrigger_traj_pos_payload.x = x0(0);
  // eventTrigger_traj_pos_payload.y = x0(1);
  // eventTrigger_traj_pos_payload.z = x0(2);
  // eventTrigger_traj_att_payload.x = x0(3);
  // eventTrigger_traj_att_payload.y = x0(4);
  // eventTrigger_traj_att_payload.z = x0(5);
  // eventTrigger_traj_vel_payload.x = x0(6);
  // eventTrigger_traj_vel_payload.y = x0(7);
  // eventTrigger_traj_vel_payload.z = x0(8);
  // eventTrigger_traj_rate_payload.x = x0(9);
  // eventTrigger_traj_rate_payload.y = x0(10);
  // eventTrigger_traj_rate_payload.z = x0(11);
  // eventTrigger_control_payload.u1 = control->normalizedForces[0];
  // eventTrigger_control_payload.u2 = control->normalizedForces[1];
  // eventTrigger_control_payload.u3 = control->normalizedForces[2];
  // eventTrigger_control_payload.u4 = control->normalizedForces[3];
  // eventTrigger_solver_stats_payload.solvetime_us = mpcTime;
  // eventTrigger_solver_stats_payload.iters = info.iter;

  // eventTrigger(&eventTrigger_traj_ref);
  // eventTrigger(&eventTrigger_traj_pos);
  // eventTrigger(&eventTrigger_traj_att);
  // eventTrigger(&eventTrigger_traj_vel);
  // eventTrigger(&eventTrigger_traj_rate);
  // eventTrigger(&eventTrigger_control);
  // eventTrigger(&eventTrigger_solver_stats);
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

LOG_ADD(LOG_INT32, iters, &iter_log)
LOG_ADD(LOG_UINT32, mpcTime, &mpcTime)


LOG_ADD(LOG_FLOAT, primal_residual, &pri_resid_log)
LOG_ADD(LOG_FLOAT, dual_residual, &dual_resid_log)
LOG_ADD(LOG_FLOAT, ref_x, &ref_x)
LOG_ADD(LOG_FLOAT, ref_y, &ref_y)
LOG_ADD(LOG_FLOAT, ref_z, &ref_z)

// LOG_ADD(LOG_FLOAT, u0, &(Uhrz[0](0)))
// LOG_ADD(LOG_FLOAT, u1, &(Uhrz[0](1)))
// LOG_ADD(LOG_FLOAT, u2, &(Uhrz[0](2)))
// LOG_ADD(LOG_FLOAT, u3, &(Uhrz[0](3)))

// LOG_ADD(LOG_FLOAT, zu0, &(ZU_new[0](0)))
// LOG_ADD(LOG_FLOAT, zu1, &(ZU_new[0](1)))
// LOG_ADD(LOG_FLOAT, zu2, &(ZU_new[0](2)))
// LOG_ADD(LOG_FLOAT, zu3, &(ZU_new[0](3)))

// LOG_ADD(LOG_FLOAT, h_x_0, &Xhrz_log_x_0)
// LOG_ADD(LOG_FLOAT, h_x_1, &Xhrz_log_x_1)
// LOG_ADD(LOG_FLOAT, h_x_2, &Xhrz_log_x_2)
// LOG_ADD(LOG_FLOAT, h_x_3, &Xhrz_log_x_3)
// LOG_ADD(LOG_FLOAT, h_x_4, &Xhrz_log_x_4)
// LOG_ADD(LOG_FLOAT, h_x_5, &Xhrz_log_x_5)
// LOG_ADD(LOG_FLOAT, h_x_6, &Xhrz_log_x_6)
// LOG_ADD(LOG_FLOAT, h_x_7, &Xhrz_log_x_7)
// LOG_ADD(LOG_FLOAT, h_x_8, &Xhrz_log_x_8)
// LOG_ADD(LOG_FLOAT, h_x_9, &Xhrz_log_x_9)
// LOG_ADD(LOG_FLOAT, h_x_10, &Xhrz_log_x_10)
// LOG_ADD(LOG_FLOAT, h_x_11, &Xhrz_log_x_11)
// LOG_ADD(LOG_FLOAT, h_x_12, &Xhrz_log_x_12)
// LOG_ADD(LOG_FLOAT, h_x_13, &Xhrz_log_x_13)
// LOG_ADD(LOG_FLOAT, h_x_14, &Xhrz_log_x_14)

// LOG_ADD(LOG_FLOAT, h_y_0, &Xhrz_log_y_0)
// LOG_ADD(LOG_FLOAT, h_y_1, &Xhrz_log_y_1)
// LOG_ADD(LOG_FLOAT, h_y_2, &Xhrz_log_y_2)
// LOG_ADD(LOG_FLOAT, h_y_3, &Xhrz_log_y_3)
// LOG_ADD(LOG_FLOAT, h_y_4, &Xhrz_log_y_4)
// LOG_ADD(LOG_FLOAT, h_y_5, &Xhrz_log_y_5)
// LOG_ADD(LOG_FLOAT, h_y_6, &Xhrz_log_y_6)
// LOG_ADD(LOG_FLOAT, h_y_7, &Xhrz_log_y_7)
// LOG_ADD(LOG_FLOAT, h_y_8, &Xhrz_log_y_8)
// LOG_ADD(LOG_FLOAT, h_y_9, &Xhrz_log_y_9)
// LOG_ADD(LOG_FLOAT, h_y_10, &Xhrz_log_y_10)
// LOG_ADD(LOG_FLOAT, h_y_11, &Xhrz_log_y_11)
// LOG_ADD(LOG_FLOAT, h_y_12, &Xhrz_log_y_12)
// LOG_ADD(LOG_FLOAT, h_y_13, &Xhrz_log_y_13)
// LOG_ADD(LOG_FLOAT, h_y_14, &Xhrz_log_y_14)

// LOG_ADD(LOG_FLOAT, h_z_0, &Xhrz_log_z_0)
// LOG_ADD(LOG_FLOAT, h_z_1, &Xhrz_log_z_1)
// LOG_ADD(LOG_FLOAT, h_z_2, &Xhrz_log_z_2)
// LOG_ADD(LOG_FLOAT, h_z_3, &Xhrz_log_z_3)
// LOG_ADD(LOG_FLOAT, h_z_4, &Xhrz_log_z_4)
// LOG_ADD(LOG_FLOAT, h_z_5, &Xhrz_log_z_5)
// LOG_ADD(LOG_FLOAT, h_z_6, &Xhrz_log_z_6)
// LOG_ADD(LOG_FLOAT, h_z_7, &Xhrz_log_z_7)
// LOG_ADD(LOG_FLOAT, h_z_8, &Xhrz_log_z_8)
// LOG_ADD(LOG_FLOAT, h_z_9, &Xhrz_log_z_9)
// LOG_ADD(LOG_FLOAT, h_z_10, &Xhrz_log_z_10)
// LOG_ADD(LOG_FLOAT, h_z_11, &Xhrz_log_z_11)
// LOG_ADD(LOG_FLOAT, h_z_12, &Xhrz_log_z_12)
// LOG_ADD(LOG_FLOAT, h_z_13, &Xhrz_log_z_13)
// LOG_ADD(LOG_FLOAT, h_z_14, &Xhrz_log_z_14)

LOG_GROUP_STOP(ctrlMPC)

#ifdef __cplusplus
}
#endif
