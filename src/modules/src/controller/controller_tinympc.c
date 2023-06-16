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

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "sensors.h"
#include "static_mem.h"
#include "string.h"
#include "stdint.h"
#include "stdbool.h"

#include "controller_tinympc.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "physicalConstants.h"

#include "slap/slap.h"
#include "tinympc/tinympc.h"

#define DEBUG_MODULE "TINYMPC"
#include "debug.h"
#include "cfassert.h"


static SemaphoreHandle_t runTaskSemaphore;
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;


// Macro variables
#define DT 0.02f       // dt
#define NSTATES 12    // no. of states (error state)
#define NINPUTS 4     // no. of controls
#define NHORIZON 3   // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define NSIM NHORIZON      // length of reference trajectory
#define MPC_RATE RATE_250_HZ  // control frequency

#if MPC_RATE == RATE_50_HZ
  #include "tinympc/data/params_50hz.h"
#elif MPC_RATE == RATE_250_HZ
  #include "tinympc/data/params_250hz.h"
#endif

/* Allocate global variables for MPC */

// Precompute data offline
static sfloat A_data[NSTATES*NSTATES] = {
  1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,-0.000157f,0.000000f,1.000000f,-0.000000f,-0.000000f,0.000000f,-0.078480f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000157f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.078480f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  -0.000000f,-0.000000f,0.000000f,0.000000f,-0.000000f,1.000000f,-0.000000f,-0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.004000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.004000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.004000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,-0.000000f,0.000000f,0.002000f,0.000000f,-0.000000f,0.000000f,-0.000078f,0.000000f,1.000000f,0.000000f,-0.000000f,
  0.000000f,0.000000f,0.000000f,-0.000000f,0.002000f,0.000000f,0.000078f,0.000000f,0.000000f,-0.000000f,1.000000f,0.000000f,
  -0.000000f,-0.000000f,0.000000f,0.000000f,-0.000000f,0.002000f,-0.000000f,-0.000000f,0.000000f,0.000000f,-0.000000f,1.000000f,
};

static sfloat B_data[NSTATES*NINPUTS] = {
  -0.000000f,0.000000f,0.000034f,-0.001101f,-0.001107f,0.000079f,-0.000029f,0.000029f,0.016817f,-1.101418f,-1.106828f,0.078991f,
  0.000000f,0.000000f,0.000034f,-0.001213f,0.001217f,-0.000029f,0.000032f,0.000032f,0.016817f,-1.212936f,1.217114f,-0.028895f,
  0.000000f,-0.000000f,0.000034f,0.001103f,0.001110f,-0.000111f,0.000029f,-0.000029f,0.016817f,1.102651f,1.110278f,-0.111375f,
  -0.000000f,-0.000000f,0.000034f,0.001212f,-0.001221f,0.000061f,-0.000032f,-0.000032f,0.016817f,1.211704f,-1.220564f,0.061279f,
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


// Misc variables
static uint32_t mpcTime = 0;
static float u_hover = 0.66f;  // ~ mass/max_thrust/4
static bool isInit = false;


static void tinympcControllerTask(void* parameters);

STATIC_MEM_TASK_ALLOC(tinympcTask, TINYMPC_TASK_STACKSIZE);

void controllerTinyMPCInit(void)
{
  if (isInit) {
    return;
  }

  /* Initialize MPC */
  
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT);
  tiny_InitSettings(&stgs);

  stgs.rho_init = 100.0f;  // IMPORTANT (select offline, associated with precomp.)

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
  stgs.max_iter = 6;           // limit this if needed
  stgs.verbose = 0;
  stgs.check_termination = 1;
  stgs.tol_abs_dual = 5e-2;
  stgs.tol_abs_prim = 5e-2;


  /* Initialize task */

  runTaskSemaphore = xSemaphoreCreateBinary();
  ASSERT(runTaskSemaphore);

  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

  STATIC_MEM_TASK_CREATE(tinympcTask, tinympcControllerTask, TINYMPC_TASK_NAME, NULL, TINYMPC_TASK_PRI);

  isInit = true;
}


void controllerTinyMPC(control_t *control,
                                 const setpoint_t *setpoint,
                                 const sensorData_t *sensors,
                                 const state_t *state,
                                 const uint32_t tick) {
  
  // This function is called from the stabilizer loop. It is important that this call returns
  // as quickly as possible. The dataMutex must only be locked short periods by the task.
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  memcpy(&setpoint_data, setpoint, sizeof(setpoint_t));
  memcpy(&sensors_data, sensors, sizeof(sensorData_t));
  memcpy(&state_data, state, sizeof(state_t));

  // Copy the latest controls, calculated by the TinyMPC task
  memcpy(control, &control_data, sizeof(control_t));
  xSemaphoreGive(dataMutex);

  xSemaphoreGive(runTaskSemaphore);
}

static void tinympcControllerTask(void* parameters) {
  systemWaitStart();

  uint32_t nowMs = T2M(xTaskGetTickCount());
  uint32_t nextPredictionMs = nowMs;

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
    
    // // Get current time
    // uint64_t startTimestamp = usecTimestamp();

    /* Controller rate */
    if (nowMs >= nextPredictionMs) {
      nextPredictionMs = nowMs + (1000.0f / MPC_RATE);

      // /* Get goal state_task (reference) */
      xg_data[0]  = setpoint_task.position.x;
      xg_data[1]  = setpoint_task.position.y;
      xg_data[2]  = setpoint_task.position.z;
      xg_data[6]  = setpoint_task.velocity.x;
      xg_data[7]  = setpoint_task.velocity.y;
      xg_data[8]  = setpoint_task.velocity.z;
      xg_data[9]  = radians(setpoint_task.attitudeRate.roll);
      xg_data[10] = radians(setpoint_task.attitudeRate.pitch);
      xg_data[11] = radians(setpoint_task.attitudeRate.yaw);
      struct vec desired_rpy = mkvec(radians(setpoint_task.attitude.roll), 
                                    radians(setpoint_task.attitude.pitch), 
                                    radians(setpoint_task.attitude.yaw));
      struct quat attitude = rpy2quat(desired_rpy);
      struct vec phi = quat2rp(qnormalize(attitude));  
      xg_data[3] = phi.x;
      xg_data[4] = phi.y;
      xg_data[5] = phi.z;
      
      /* Get current state_task (initial state_task for MPC) */
      // delta_x = x - x_bar; x_bar = 0
      // Positon error, [m]
      x0_data[0] = state_task.position.x;
      x0_data[1] = state_task.position.y;
      x0_data[2] = state_task.position.z;
      // Body velocity error, [m/s]                          
      x0_data[6] = state_task.velocity.x;
      x0_data[7] = state_task.velocity.y;
      x0_data[8] = state_task.velocity.z;
      // Angular rate error, [rad/s]
      x0_data[9]  = radians(sensors_task.gyro.x);   
      x0_data[10] = radians(sensors_task.gyro.y);
      x0_data[11] = radians(sensors_task.gyro.z);
      attitude = mkquat(
        state_task.attitudeQuaternion.x,
        state_task.attitudeQuaternion.y,
        state_task.attitudeQuaternion.z,
        state_task.attitudeQuaternion.w);  // current attitude
      phi = quat2rp(qnormalize(attitude));  // quaternion to Rodriquez parameters  
      // Attitude error
      x0_data[3] = phi.x;
      x0_data[4] = phi.y;
      x0_data[5] = phi.z;

      /* MPC solve */
      // Warm-start by previous solution  // TODO: should I warm-start U with previous ZU
      // tiny_ShiftFill(U, T_ARRAY_SIZE(U));

      //// Solve optimization problem using ADMM
      // tiny_UpdateLinearCost(&work);
      // tiny_SolveAdmm(&work);
      /* MPC solve end */


      /* LQR solve */
      MatAdd(data.x0, data.x0, Xref[0], -1);
      MatMulAdd(U[0], soln.Kinf, data.x0, -1, 0);
      /* LQR solve end */

      // mpcTime = usecTimestamp() - startTimestamp;

      // DEBUG_PRINT("U[0] = [%.2f, %.2f]\n", (double)(U[0].data[0]), (double)(U[0].data[1]));

      // DEBUG_PRINT("U[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(U[0].data[0]), (double)(U[0].data[1]), (double)(U[0].data[2]), (double)(U[0].data[3]));
      // DEBUG_PRINT("ZU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(ZU[0].data[0]), (double)(ZU[0].data[1]), (double)(ZU[0].data[2]), (double)(ZU[0].data[3]));
      // DEBUG_PRINT("YU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(YU[0].data[0]), (double)(YU[0].data[1]), (double)(YU[0].data[2]), (double)(YU[0].data[3]));
      // DEBUG_PRINT("info.pri_res: %f\n", (double)(info.pri_res));
      // DEBUG_PRINT("info.dua_res: %f\n", (double)(info.dua_res));
      // result =  info.status_val * info.iter;
      // DEBUG_PRINT("%d %d %d \n", info.status_val, info.iter, mpcTime);
      // DEBUG_PRINT("%d\n", mpcTime);
      // DEBUG_PRINT("[%.2f, %.2f, %.2f]\n", (double)(x0_data[0]), (double)(x0_data[1]), (double)(x0_data[2]));

      /* Output control_task */
      if (setpoint_task.mode.z == modeDisable) {
        control_task.normalizedForces[0] = 0.0f;
        control_task.normalizedForces[1] = 0.0f;
        control_task.normalizedForces[2] = 0.0f;
        control_task.normalizedForces[3] = 0.0f;
      } else {
        control_task.normalizedForces[0] = U[0].data[0] + u_hover;  // PWM 0..1
        control_task.normalizedForces[1] = U[0].data[1] + u_hover;
        control_task.normalizedForces[2] = U[0].data[2] + u_hover;
        control_task.normalizedForces[3] = U[0].data[3] + u_hover;
        // control_task.normalizedForces[0] = U[0].data[0];  // PWM 0..1
        // control_task.normalizedForces[1] = U[0].data[1];
        // control_task.normalizedForces[2] = U[0].data[2];
        // control_task.normalizedForces[3] = U[0].data[3];
      } 
      // DEBUG_PRINT("pwm = [%.2f, %.2f]\n", (double)(control_task.normalizedForces[0]), (double)(control_task.normalizedForces[2]));
      // control_task.normalizedForces[0] = 0.0f;
      // control_task.normalizedForces[1] = 0.0f;
      // control_task.normalizedForces[2] = 0.0f;
      // control_task.normalizedForces[3] = 0.0f;

      control_task.controlMode = controlModePWM;
      
      // Copy the controls calculated by the task loop to the global control_data
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      memcpy(&control_data, &control_task, sizeof(control_t));
      xSemaphoreGive(dataMutex);
    }
  }
}

bool controllerTinyMPCTest() {
  return isInit;
}

/**
 * Tuning variables for the full state quaternion LQR controller
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

// LOG_GROUP_START(ctrlTinyMPC)


// // LOG_ADD(LOG_FLOAT, x, &x0_data[0])
// // LOG_ADD(LOG_FLOAT, y, &x0_data[1])
// // LOG_ADD(LOG_FLOAT, z, &x0_data[2])

// // LOG_ADD(LOG_FLOAT, roll,  &x0_data[3])
// // LOG_ADD(LOG_FLOAT, pitch, &x0_data[4])
// // LOG_ADD(LOG_FLOAT, yaw,   &x0_data[5])

// // LOG_ADD(LOG_FLOAT, vx, &x0_data[6])
// // LOG_ADD(LOG_FLOAT, vy, &x0_data[7])
// // LOG_ADD(LOG_FLOAT, vz, &x0_data[8])

// // LOG_ADD(LOG_FLOAT, wroll,  &x0_data[9])
// // LOG_ADD(LOG_FLOAT, wpitch, &x0_data[10])
// // LOG_ADD(LOG_FLOAT, wyaw,   &x0_data[11])

// // LOG_ADD(LOG_INT8, result, &result)
// LOG_ADD(LOG_UINT32, mpcTime, &mpcTime)

// // LOG_ADD(LOG_FLOAT, u0, &(control_data.normalizedForces[0]))
// // LOG_ADD(LOG_FLOAT, u1, &(control_data.normalizedForces[1]))
// // LOG_ADD(LOG_FLOAT, u2, &(control_data.normalizedForces[2]))
// // LOG_ADD(LOG_FLOAT, u3, &(control_data.normalizedForces[3]))

// // LOG_ADD(LOG_FLOAT, yu0, &(YU_data[0]))
// // LOG_ADD(LOG_FLOAT, yu1, &(YU_data[1]))
// // LOG_ADD(LOG_FLOAT, yu2, &(YU_data[2]))
// // LOG_ADD(LOG_FLOAT, yu3, &(YU_data[3]))

// LOG_GROUP_STOP(ctrlTinyMPC)

#ifdef __cplusplus
}
#endif
