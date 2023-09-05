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

#include "app.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "sensors.h"
#include "static_mem.h"
#include "string.h"
#include "stdint.h"
#include "stdbool.h"
#include "system.h"

#include "controller.h"
#include "physicalConstants.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"

#include "cpp_compat.h"   // needed to compile Cpp to C

#include "tinympc/tinympc.h"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TINYMPC"
#include "debug.h"

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

static SemaphoreHandle_t runTaskSemaphore;
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

static void tinympcControllerTask(void* parameters);

STATIC_MEM_TASK_ALLOC(tinympcControllerTask, TINYMPC_TASK_STACKSIZE);

// Macro variables, model dimensions in tinympc/types.h
#define NHORIZON 7   // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define MPC_RATE RATE_100_HZ  // control frequency
#define LQR_RATE RATE_500_HZ  // control frequency

/* Include trajectory to track */
// #include "traj_fig8_12.h"
#include "traj_circle_500hz.h"
// #include "traj_perching.h"

// Precomputed data and cache, in params_*.h
static MatrixNf A;
static MatrixNMf B;
static MatrixMNf Kinf;
static MatrixMNf Klqr;
static MatrixNf Pinf;
static MatrixMf Quu_inv;
static MatrixNf AmBKt;
static MatrixNMf coeff_d2p;
static MatrixNf Q;
static MatrixMf R;

/* Allocate global variables for MPC */

static VectorNf Xhrz[NHORIZON];
static VectorMf Uhrz[NHORIZON-1]; 
static VectorMf Ulqr;
static VectorMf d[NHORIZON-1];
static VectorNf p[NHORIZON];
static VectorMf YU[NHORIZON-1];
static VectorNf YX[NHORIZON];

static VectorNf q[NHORIZON-1];
static VectorMf r[NHORIZON-1];
static VectorNf q_tilde[NHORIZON];
static VectorMf r_tilde[NHORIZON-1];

static VectorNf Xref[NHORIZON];
static VectorMf Uref[NHORIZON-1];

static MatrixMf Acu;
static VectorMf ucu;
static VectorMf lcu;
static VectorNf Acx;
static VectorNf ucx;
static VectorNf lcx;

static VectorMf Qu;
static VectorMf ZU[NHORIZON-1]; 
static VectorMf ZU_new[NHORIZON-1];
static VectorNf ZX[NHORIZON]; 
static VectorNf ZX_new[NHORIZON];

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
static float u_hover[4] = {0.7711f, 0.7647f, 0.7854f, 0.8229f};  // cf1
// static float u_hover[4] = {0.7467, 0.667f, 0.78, 0.7f};  // cf2 not correct
static int8_t result = 0;
static uint32_t step = 0;
static bool en_traj = false;
static uint32_t traj_length = T_ARRAY_SIZE(X_ref_data);
static int8_t user_traj_iter = 1;  // number of times to execute full trajectory
static int8_t traj_hold = 1;       // hold current trajectory for this no of steps
static int8_t traj_iter = 0;
static uint32_t traj_idx = 0;

static struct vec desired_rpy;
static struct quat attitude;
static struct vec phi;

// Structs to keep track of data sent to and received by stabilizer loop

// Updates at 1khz
control_t control_data;
setpoint_t setpoint_data;
sensorData_t sensors_data;
state_t state_data;

// Updates at update_rate
setpoint_t setpoint_task;
sensorData_t sensors_task;
state_t state_task;
control_t control_task;

uint32_t prevLqrMs;

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
  // This will carry out the included trajectory first
  // Better idea is to set `en_traj` from crazyswarm
  // If just want to follow cmd, replace en_traj with `0`
  if (en_traj) {
    if (step % traj_hold == 0) {
      traj_idx = (int)(step / traj_hold);
      for (int i = 0; i < NHORIZON; ++i) {
        for (int j = 0; j < NSTATES; ++j) {
          Xref[i](j) = X_ref_data[traj_idx][j];
        }
        if (i < NHORIZON - 1) {
          for (int j = 0; j < NINPUTS; ++j) {
            Uref[i](j) = U_ref_data[traj_idx][j];
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
  
  en_traj = false;
  step = 0;  
  traj_iter = 0;

  if (isInit) {
    return;
  }

  // Precompute/Cache
  #include "params_500hz.h"
  // #include "params_100hz_old.h"

  // End of Precompute/Cache

  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, 0.002, &A, &B, 0);
  tiny_InitSettings(&stgs);
  stgs.rho_init = 500.0;  // Important (select offline, associated with precomp.)
  tiny_InitWorkspace(&work, &info, &model, &data, &soln, &stgs);
  
  // Fill in the remaining struct 
  tiny_InitWorkspaceTemp(&work, &Qu, ZU, ZU_new, ZX, ZX_new);
  tiny_InitPrimalCache(&work, &Quu_inv, &AmBKt, &coeff_d2p);
  tiny_InitSolution(&work, Xhrz, Uhrz, YX, YU, 0, &Kinf, d, &Pinf, p);

  tiny_SetInitialState(&work, &x0);  
  tiny_SetStateReference(&work, Xref);
  tiny_SetInputReference(&work, Uref);
  // tiny_SetGoalState(&work, Xref, &xg);
  // tiny_SetGoalInput(&work, Uref, &ug);

  /* Set up LQR cost */
  tiny_InitDataCost(&work, &Q, q, q_tilde, &R, r, r_tilde);
  // R = R + stgs.rho_init * MatrixMf::Identity();
  // /* Set up constraints */
  ucu << 1 - u_hover[0], 1 - u_hover[1], 1 - u_hover[2], 1 - u_hover[3];
  lcu << -u_hover[0], -u_hover[1], -u_hover[2], -u_hover[3];
  tiny_SetInputBound(&work, &Acu, &lcu, &ucu);

  tiny_SetStateConstraint(&work, &Acx, &lcx, &ucx);
  Acx << 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  ucx << 0.9f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;  
  // make it more aggressive

  tiny_UpdateLinearCost(&work);

  /* Solver settings */
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 0;
  stgs.max_iter = 10;           // limit this if needed
  stgs.verbose = 0;
  stgs.check_termination = 0;
  stgs.tol_abs_dual = 1e-2;
  stgs.tol_abs_prim = 1e-2;

  Klqr << 
  -0.123589f,0.123635f,0.285625f,-0.394876f,-0.419547f,-0.474536f,-0.073759f,0.072612f,0.186504f,-0.031569f,-0.038547f,-0.187738f,
  0.120236f,0.119379f,0.285625f,-0.346222f,0.403763f,0.475821f,0.071330f,0.068348f,0.186504f,-0.020972f,0.037152f,0.187009f,
  0.121600f,-0.122839f,0.285625f,0.362241f,0.337953f,-0.478858f,0.069310f,-0.070833f,0.186504f,0.022379f,0.015573f,-0.185212f,
  -0.118248f,-0.120176f,0.285625f,0.378857f,-0.322169f,0.477573f,-0.066881f,-0.070128f,0.186504f,0.030162f,-0.014177f,0.185941f;

  /* End of MPC initialization */  

  /* Start task initialization */

  runTaskSemaphore = xSemaphoreCreateBinary();
  ASSERT(runTaskSemaphore);

  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

  STATIC_MEM_TASK_CREATE(tinympcControllerTask, tinympcControllerTask, TINYMPC_TASK_NAME, NULL, TINYMPC_TASK_PRI);

  isInit = true;

  /* End of task initialization */
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

static void tinympcControllerTask(void* parameters) {
  systemWaitStart();

  uint32_t nowMs = T2M(xTaskGetTickCount());
  uint32_t nextMpcMs = nowMs;
  uint32_t nextLqrMs = nowMs;
  // uint32_t prevMpcMs = nowMs;

  startTimestamp = usecTimestamp();

  while (true) {
    // Update task data with most recent stabilizer loop data
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(&setpoint_task, &setpoint_data, sizeof(setpoint_t));
    memcpy(&sensors_task, &sensors_data, sizeof(sensorData_t));
    memcpy(&state_task, &state_data, sizeof(state_t));
    memcpy(&control_task, &control_data, sizeof(control_t));
    xSemaphoreGive(dataMutex);

    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
    nowMs = T2M(xTaskGetTickCount()); // would be nice if this had a precision higher than 1ms...

    /* Controller rate */
    if (nowMs >= nextMpcMs) {
      // startTimestamp = usecTimestamp();
      // DEBUG_PRINT("M: %d\n", startTimestamp - prevMpcMs);
      // prevMpcMs = startTimestamp;
      
      nextMpcMs = nowMs + (1000.0f / MPC_RATE);
      updateHorizonReference(&setpoint_task);
      updateInitialState(&sensors_task, &state_task);

      tiny_UpdateLinearCost(&work);
      tiny_SolveAdmm(&work);
      DEBUG_PRINT("U = %.1f, %.1f", ZU_new[0](0), ZU_new[0](1));
      // DEBUG_PRINT("X = %.2f, %.2f", ZX_new[0](2), ZX_new[1](2));

      mpcTime = usecTimestamp() - startTimestamp;

      // DEBUG_PRINT("H: %d\n", mpcTime);
    }

    // Copy the controls calculated by the task loop to the global control_data
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(&control_data, &control_task, sizeof(control_t));
    xSemaphoreGive(dataMutex);
  } 
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // This function is called from the stabilizer loop. It is important that this call returns
  // as quickly as possible. The dataMutex must only be locked short periods by the task.
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  memcpy(&setpoint_data, setpoint, sizeof(setpoint_t));
  memcpy(&sensors_data, sensors, sizeof(sensorData_t));
  memcpy(&state_data, state, sizeof(state_t));

  if (!RATE_DO_EXECUTE(LQR_RATE, tick)) {
    return;
  }

  // startTimestamp = usecTimestamp();
  // DEBUG_PRINT("L: %d\n", startTimestamp - prevLqrMs);
  // prevLqrMs = startTimestamp;

  updateInitialState(sensors, state);

  //// LQR tracks the future knotpoint from MPC
  Ulqr = -Klqr * (x0 - Xhrz[3]) * 1.5 + ZU_new[2];    // can tune this!
  
  /* Output control */
  if (setpoint->mode.z == modeDisable) {
    control->normalizedForces[0] = 0.0f;
    control->normalizedForces[1] = 0.0f;
    control->normalizedForces[2] = 0.0f;
    control->normalizedForces[3] = 0.0f;
  } else {
    control->normalizedForces[0] = Ulqr(0) + u_hover[0];  // PWM 0..1
    control->normalizedForces[1] = Ulqr(1) + u_hover[1];
    control->normalizedForces[2] = Ulqr(2) + u_hover[2];
    control->normalizedForces[3] = Ulqr(3) + u_hover[3];
  } 
  control->controlMode = controlModePWM;
  // DEBUG_PRINT("L: %d\n", usecTimestamp() - startTimestamp);
  // control->normalizedForces[0] = 0.0f;
  // control->normalizedForces[1] = 0.0f;
  // control->normalizedForces[2] = 0.0f;
  // control->normalizedForces[3] = 0.0f;
  xSemaphoreGive(dataMutex);
  xSemaphoreGive(runTaskSemaphore);
}

/**
 * Tunning variables for the full state quaternion LQR controller
 */
// PARAM_GROUP_START(ctrlMPC)
// /**
//  * @brief K gain
//  */
// PARAM_ADD_CORE(PARAM_INT8 | PARAM_PERSISTENT, stgs_cstr_inputs, &(stgs.en_cstr_inputs))
// PARAM_ADD_CORE(PARAM_INT8 | PARAM_PERSISTENT, stgs_max_iter, &(stgs.max_iter))

// PARAM_GROUP_STOP(ctrlMPC)

/**
 * Logging variables for the command and reference signals for the
 * MPC controller
 */

LOG_GROUP_START(ctrlMPC)

LOG_ADD(LOG_INT8, result, &result)
LOG_ADD(LOG_UINT32, mpcTime, &mpcTime)

LOG_ADD(LOG_FLOAT, u0, &(Uhrz[0](0)))
LOG_ADD(LOG_FLOAT, u1, &(Uhrz[0](1)))
LOG_ADD(LOG_FLOAT, u2, &(Uhrz[0](2)))
LOG_ADD(LOG_FLOAT, u3, &(Uhrz[0](3)))

LOG_ADD(LOG_FLOAT, zu0, &(ZU_new[0](0)))
LOG_ADD(LOG_FLOAT, zu1, &(ZU_new[0](1)))
LOG_ADD(LOG_FLOAT, zu2, &(ZU_new[0](2)))
LOG_ADD(LOG_FLOAT, zu3, &(ZU_new[0](3)))

LOG_GROUP_STOP(ctrlMPC)

#ifdef __cplusplus
}
#endif
