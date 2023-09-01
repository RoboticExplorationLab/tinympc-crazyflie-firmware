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
#include "queue.h"
#include "semphr.h"
#include "sensors.h"
#include "static_mem.h"
#include "system.h"

#include "controller.h"
#include "physicalConstants.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"

#include "cpp_compat.h"   // needed to compile Cpp to C

// Include tinympc and PID controllers, params, and trajectory to follow
#include "tinympc/admm.hpp"
#include "controller_pid.h"

#include "quadrotor_250hz_params.hpp"
// #include "quadrotor_100hz_ref_hover.hpp"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "MPCTASK"
#include "debug.h"

#define MPC_RATE RATE_250_HZ  // control frequency
#define LQR_RATE RATE_250_HZ  // control frequency

// Semaphore to signal that we got data from the stabilizer loop to process
static SemaphoreHandle_t runTaskSemaphore;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

static void tinympcControllerTask(void* parameters);

STATIC_MEM_TASK_ALLOC(tinympcControllerTask, TINYMPC_TASK_STACKSIZE);

// Structs to keep track of data sent to and received by stabilizer loop
// Stabilizer loop updates/uses these
control_t control_data;
setpoint_t setpoint_data;
sensorData_t sensors_data;
state_t state_data;
tiny_VectorNx mpc_setpoint;
setpoint_t mpc_setpoint_pid;
// Copies that stay constant for duration of MPC loop
setpoint_t setpoint_task;
sensorData_t sensors_task;
state_t state_task;
control_t control_task;
tiny_VectorNx mpc_setpoint_task;


/* Allocate global variables for MPC */
static tinytype u_hover[4] = {.65, .65, .65, .65};
static struct tiny_cache cache;
static struct tiny_params params;
static struct tiny_problem problem;
// static Eigen::Matrix<tinytype, NSTATES, NTOTAL, Eigen::ColMajor> Xref_total;
static Eigen::Matrix<tinytype, NSTATES, 1, Eigen::ColMajor> Xref_origin;
static tiny_VectorNu u_lqr;
static tiny_VectorNx current_state;

// Helper variables
static bool enable_traj = false;
static uint64_t startTimestamp;
static uint64_t timeTaken;
static struct vec phi; // For converting from the current state estimate's quaternion to Rodrigues parameters
static bool isInit = false;


static inline float quat_dot(quaternion_t a, quaternion_t b) {
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

static inline quaternion_t make_quat(float x, float y, float z, float w) {
	quaternion_t q;
	q.x = x; q.y = y; q.z = z; q.w = w;
	return q;
}

static inline quaternion_t normalize_quat(quaternion_t q) {
	float s = 1.0f / sqrtf(quat_dot(q, q));
	return make_quat(s*q.x, s*q.y, s*q.z, s*q.w);
}

static inline struct vec quat_2_rp(quaternion_t q) {
	struct vec v;
	v.x = q.x/q.w;
	v.y = q.y/q.w;
	v.z = q.z/q.w;
	return v;
}


void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));
  }
}

void UpdateHorizonReference(const setpoint_t *setpoint) {
  if (enable_traj) {
    // params.Xref = Xref_total.block<NSTATES, NHORIZON>(0, 0);
    params.Xref = Xref_origin.replicate<1,NHORIZON>();
  }
  else {
    phi = quat_2_rp(normalize_quat(setpoint->attitudeQuaternion));  // quaternion to Rodrigues parameters
    tiny_VectorNx xg = {setpoint->position.x, setpoint->position.y, setpoint->position.z, 
                        phi.x, phi.y, phi.z, 
                        setpoint->velocity.x, setpoint->velocity.y, setpoint->velocity.z,
                        radians(setpoint->attitudeRate.roll), radians(setpoint->attitudeRate.pitch), radians(setpoint->attitudeRate.yaw)};
    params.Xref = xg.replicate<1,NHORIZON>();
  }
}

void controllerOutOfTreeInit(void) {

  controllerPidInit();

  // Copy cache data from problem_data/quadrotor*.hpp
  cache.Adyn = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Adyn_data);
  cache.Bdyn = Eigen::Map<Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(Bdyn_data);
  cache.rho = rho_value;
  cache.Kinf = Eigen::Map<Matrix<tinytype, NINPUTS, NSTATES, Eigen::RowMajor>>(Kinf_data);
  // cache.Pinf = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Pinf_data);
  cache.Quu_inv = Eigen::Map<Matrix<tinytype, NINPUTS, NINPUTS, Eigen::RowMajor>>(Quu_inv_data);
  cache.AmBKt = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(AmBKt_data);
  cache.coeff_d2p = Eigen::Map<Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(coeff_d2p_data);

  // Copy parameter data
  params.Q = Eigen::Map<tiny_VectorNx>(Q_data);
  params.Qf = Eigen::Map<tiny_VectorNx>(Qf_data);
  params.R = Eigen::Map<tiny_VectorNu>(R_data);
  params.u_min = tiny_VectorNu(-u_hover[0], -u_hover[1], -u_hover[2], -u_hover[3]).replicate<1, NHORIZON-1>();
  params.u_max = tiny_VectorNu(1 - u_hover[0], 1 - u_hover[1], 1 - u_hover[2], 1 - u_hover[3]).replicate<1, NHORIZON-1>();
  // params.u_min = tiny_MatrixNuNhm1::Constant(-0.5);
  // params.u_max = tiny_MatrixNuNhm1::Constant(0.5);
  for (int i=0; i<NHORIZON; i++) {
      params.x_min[i] = tiny_VectorNc::Constant(-1000); // Currently unused
      params.x_max[i] = tiny_VectorNc::Constant(1000);
      params.A_constraints[i] = tiny_MatrixNcNx::Zero();
  }
  // params.x_min = Matrix<tinytype, NHORIZON, NSTATE_CONSTRAINTS>::Constant(-99999);
  // params.x_max = Matrix<tinytype, NHORIZON, NSTATE_CONSTRAINTS>::Constant(99999);
  // params.A_constraints = Matrix<tinytype, NHORIZON, NSTATES>::Zero();
  params.Xref = tiny_MatrixNxNh::Zero();
  params.Uref = tiny_MatrixNuNhm1::Zero();
  params.cache = cache;

  // Copy problem data
  problem.x = tiny_MatrixNxNh::Zero();
  problem.q = tiny_MatrixNxNh::Zero();
  problem.p = tiny_MatrixNxNh::Zero();
  problem.v = tiny_MatrixNxNh::Zero();
  problem.vnew = tiny_MatrixNxNh::Zero();
  problem.g = tiny_MatrixNxNh::Zero();

  problem.u = tiny_MatrixNuNhm1::Zero();
  problem.r = tiny_MatrixNuNhm1::Zero();
  problem.d = tiny_MatrixNuNhm1::Zero();
  problem.z = tiny_MatrixNuNhm1::Zero();
  problem.znew = tiny_MatrixNuNhm1::Zero();
  problem.y = tiny_MatrixNuNhm1::Zero();

  problem.primal_residual_state = 0;
  problem.primal_residual_input = 0;
  problem.dual_residual_state = 0;
  problem.dual_residual_input = 0;
  problem.abs_tol = 0.001;
  problem.status = 0;
  problem.iter = 0;
  problem.max_iter = 5;
  problem.iters_check_rho_update = 10;

  // // Copy reference trajectory into Eigen matrix
  // Xref_total = Eigen::Map<Matrix<tinytype, NTOTAL, NSTATES, Eigen::RowMajor>>(Xref_data).transpose();
  Xref_origin << 0.0000000,	0.0000000,	1.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000,	0.0000000;

  enable_traj = true;


  /* Begin task initialization */

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
  // systemWaitStart();

  uint32_t nowMs = T2M(xTaskGetTickCount());
  uint32_t nextMpcMs = nowMs;
  
  // while (true) {
  //   xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
  // }

  while (true) {
    // if (runTaskSemaphore != NULL) {
    //   if (xSemaphoreTake(runTaskSemaphore, portMAX_DELAY) == pdTRUE ) {

        // Update task data with most recent stabilizer loop data
        xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
        
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        memcpy(&setpoint_task, &setpoint_data, sizeof(setpoint_t));
        memcpy(&sensors_task, &sensors_data, sizeof(sensorData_t));
        memcpy(&state_task, &state_data, sizeof(state_t));
        memcpy(&control_task, &control_data, sizeof(control_t));
        xSemaphoreGive(dataMutex);

        nowMs = T2M(xTaskGetTickCount());
        if (nowMs >= nextMpcMs) {
          nextMpcMs = nowMs + (1000.0f / MPC_RATE);

          // DEBUG_PRINT("us: %d\n", usecTimestamp()- startTimestamp);

          // TODO: predict into the future and set initial x to wherever we think we'll be
          //    by the time we're done computing the input for that state. If we just set
          //    initial x to current state then by the time we compute the optimal input for
          //    that state we'll already be at the next state and there will be a mismatch
          //    in the input we're using for our current state.
          // Set initial x to current state
          phi = quat_2_rp(normalize_quat(state_task.attitudeQuaternion));  // quaternion to Rodrigues parameters
          problem.x.col(0) << state_task.position.x, state_task.position.y, state_task.position.z, 
                              phi.x, phi.y, phi.z, 
                              state_task.velocity.x, state_task.velocity.y, state_task.velocity.z,
                              radians(sensors_task.gyro.x), radians(sensors_task.gyro.y), radians(sensors_task.gyro.z);

          // Get command reference
          UpdateHorizonReference(&setpoint_task);

          /* MPC solve */
          // solve_admm(&problem, &params);
          solve_lqr(&problem, &params);
          
          // /* Output control */
          if (setpoint_task.mode.z == modeDisable) {
            control_task.normalizedForces[0] = 0.0f;
            control_task.normalizedForces[1] = 0.0f;
            control_task.normalizedForces[2] = 0.0f;
            control_task.normalizedForces[3] = 0.0f;
          } else {
            control_task.normalizedForces[0] = problem.u.col(0)(0) + u_hover[0];  // PWM 0..1
            control_task.normalizedForces[1] = problem.u.col(0)(1) + u_hover[1];
            control_task.normalizedForces[2] = problem.u.col(0)(2) + u_hover[2];
            control_task.normalizedForces[3] = problem.u.col(0)(3) + u_hover[3];
          }
          control_task.controlMode = controlModePWM;
          // DEBUG_PRINT("ctrl[0] = %.4f\n", control_task.normalizedForces[0]);
          // control_task.normalizedForces[0] = 0.0f;
          // control_task.normalizedForces[1] = 0.0f;
          // control_task.normalizedForces[2] = 0.0f;
          // control_task.normalizedForces[3] = 0.0f;

          // Copy the controls calculated by the task loop to the global control_data
          mpc_setpoint_task = problem.x.col(3);
          xSemaphoreTake(dataMutex, portMAX_DELAY);
          memcpy(&mpc_setpoint, &mpc_setpoint_task, sizeof(tiny_VectorNx));
          // memcpy(&control_data, &control_task, sizeof(control_t));
          xSemaphoreGive(dataMutex);
          // DEBUG_PRINT("ctdata: %.4f\n", control_data.normalizedForces[0]);
          // startTimestamp = usecTimestamp();
        }
    //   }
    // }
    // else {
    //   DEBUG_PRINT("runTaskSemaphore == NULL\n");
    // }
  }
}

/**
 * This function is called from the stabilizer loop. It is important that this call returns
 * as quickly as possible. The dataMutex must only be locked short periods by the task.
*/
void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  xSemaphoreTake(dataMutex, portMAX_DELAY);
  memcpy(&setpoint_data, setpoint, sizeof(setpoint_t));
  memcpy(&sensors_data, sensors, sizeof(sensorData_t));
  memcpy(&state_data, state, sizeof(state_t));
  // memcpy(control, &control_data, sizeof(state_t));


  controllerPid(control, setpoint, sensors, state, tick);

  // if (RATE_DO_EXECUTE(LQR_RATE, tick)) {

  //   phi = quat_2_rp(normalize_quat(state->attitudeQuaternion));  // quaternion to Rodrigues parameters
  //   current_state << state->position.x, state->position.y, state->position.z, 
  //                     phi.x, phi.y, phi.z, 
  //                     state->velocity.x, state->velocity.y, state->velocity.z,
  //                     radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z);

  //   // u_lqr = -params.cache.Kinf * (current_state - mpc_setpoint);
  //   u_lqr = -params.cache.Kinf * (current_state - Xref_origin);
  //   // u_lqr = -params.cache.Kinf * (current_state - params.Xref.col(0));

  //   if (setpoint->mode.z == modeDisable) {
  //     control->normalizedForces[0] = 0.0f;
  //     control->normalizedForces[1] = 0.0f;
  //     control->normalizedForces[2] = 0.0f;
  //     control->normalizedForces[3] = 0.0f;
  //   } else {
  //     control->normalizedForces[0] = u_lqr(0) + u_hover[0];  // PWM 0..1
  //     control->normalizedForces[1] = u_lqr(1) + u_hover[1];
  //     control->normalizedForces[2] = u_lqr(2) + u_hover[2];
  //     control->normalizedForces[3] = u_lqr(3) + u_hover[3];
  //   }
  //   control->controlMode = controlModePWM;
  // }

  // // if (setpoint->mode.z == modeDisable) {
  // //   control->normalizedForces[0] = 0.0f;
  // //   control->normalizedForces[1] = 0.0f;
  // //   control->normalizedForces[2] = 0.0f;
  // //   control->normalizedForces[3] = 0.0f;
  // // } else {
  // //   control->normalizedForces[0] = control_data.normalizedForces[0];
  // //   control->normalizedForces[1] = control_data.normalizedForces[1];
  // //   control->normalizedForces[2] = control_data.normalizedForces[2];
  // //   control->normalizedForces[3] = control_data.normalizedForces[3];
  // // }
  // // control->controlMode = controlModePWM;


  xSemaphoreGive(dataMutex);

  // Allows mpc task to run again
  xSemaphoreGive(runTaskSemaphore);
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

// LOG_ADD(LOG_INT8, result, &result)
// LOG_ADD(LOG_UINT32, mpcTime, &mpcTime)

// LOG_ADD(LOG_FLOAT, u0, &(Uhrz[0](0)))
// LOG_ADD(LOG_FLOAT, u1, &(Uhrz[0](1)))
// LOG_ADD(LOG_FLOAT, u2, &(Uhrz[0](2)))
// LOG_ADD(LOG_FLOAT, u3, &(Uhrz[0](3)))

// LOG_ADD(LOG_FLOAT, zu0, &(ZU_new[0](0)))
// LOG_ADD(LOG_FLOAT, zu1, &(ZU_new[0](1)))
// LOG_ADD(LOG_FLOAT, zu2, &(ZU_new[0](2)))
// LOG_ADD(LOG_FLOAT, zu3, &(ZU_new[0](3)))

LOG_GROUP_STOP(ctrlMPC)

#ifdef __cplusplus
} /* extern "C" */
#endif