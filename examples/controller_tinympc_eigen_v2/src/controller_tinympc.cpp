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

#include "cpp_compat.h"   // needed to compile Cpp to C

// Include tinympc, params, and trajectory to follow
#include "tinympc/admm.hpp"
#include "quadrotor_20hz_simple.hpp"
#include "quadrotor_20hz_figure_eight.hpp"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "TINYMPC-E"
#include "debug.h"


#define MPC_RATE RATE_100_HZ  // control frequency
#define LQR_RATE RATE_500_HZ  // control frequency

// Precomputed data and cache, in params_*.h


/* Allocate global variables for MPC */
static struct tiny_cache cache;
static struct tiny_params params;
static struct tiny_problem problem;
static Eigen::Matrix<tinytype, NSTATES, NTOTAL, Eigen::ColMajor> Xref_total;

// Helper variables
static uint64_t startTimestamp;
static bool enable_traj = false;
static struct vec phi; // For converting from the current state estimate's quaternion to Rodrigues parameters

// static bool isInit = false;  // fix for tracking problem
// static uint32_t mpcTime = 0;
// static float u_hover[4] = {0.7f, 0.663f, 0.7373f, 0.633f};  // cf1
// // static float u_hover[4] = {0.7467, 0.667f, 0.78, 0.7f};  // cf2 not correct
// static int8_t result = 0;
// static uint32_t step = 0;
// static uint32_t traj_length = T_ARRAY_SIZE(X_ref_data);
// static int8_t user_traj_iter = 1;  // number of times to execute full trajectory
// static int8_t traj_hold = 1;       // hold current trajectory for this no of steps
// static int8_t traj_iter = 0;
// static uint32_t traj_idx = 0;

// static struct vec desired_rpy;
// static struct quat attitude;

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
    params.Xref = Xref_total.block<NSTATES, NHORIZON>(0, 0);
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

  // Copy cache data from problem_data/quadrotor*.hpp
  cache.Adyn = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Adyn_data);
  cache.Bdyn = Eigen::Map<Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(Bdyn_data);
  cache.rho = rho_value;
  cache.Kinf = Eigen::Map<Matrix<tinytype, NINPUTS, NSTATES, Eigen::RowMajor>>(Kinf_data);
  cache.Pinf = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Kinf_data);
  cache.Quu_inv = Eigen::Map<Matrix<tinytype, NINPUTS, NINPUTS, Eigen::RowMajor>>(Quu_inv_data);
  cache.AmBKt = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(AmBKt_data);
  cache.coeff_d2p = Eigen::Map<Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(coeff_d2p_data);

  // Copy parameter data
  params.Q = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Q_data);
  params.Qf = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Qf_data);
  params.R = Eigen::Map<Matrix<tinytype, NINPUTS, NINPUTS, Eigen::RowMajor>>(R_data);
  params.u_min = tiny_MatrixNuNhm1::Constant(-0.5);
  params.u_max = tiny_MatrixNuNhm1::Constant(0.5);
  for (int i=0; i<NHORIZON; i++) {
      params.x_min[i] = tiny_VectorNc::Constant(-99999); // Currently unused
      params.x_max[i] = tiny_VectorNc::Zero();
      params.A_constraints[i] = tiny_MatrixNcNx::Zero();
  }
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
  problem.max_iter = 100;
  problem.iters_check_rho_update = 10;

  // Copy reference trajectory into Eigen matrix
  Xref_total = Eigen::Map<Matrix<tinytype, NTOTAL, NSTATES, Eigen::RowMajor>>(Xref_data).transpose();
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}




void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // Get current time
  startTimestamp = usecTimestamp();

  /* Controller rate */
  if (RATE_DO_EXECUTE(MPC_RATE, tick)) { 
    // TODO: predict into the future and set initial x to wherever we think we'll be
    //    by the time we're done computing the input for that state. If we just set
    //    initial x to current state then by the time we compute the optimal input for
    //    that state we'll already be at the next state and there will be a mismatch
    //    in the input we're using for our current state.
    // Set initial x to current state
    phi = quat_2_rp(normalize_quat(state->attitudeQuaternion));  // quaternion to Rodrigues parameters
    problem.x.col(0) << state->position.x, state->position.y, state->position.z, 
                        phi.x, phi.y, phi.z, 
                        state->velocity.x, state->velocity.y, state->velocity.z,
                        radians(sensors->gyro.x), radians(sensors->gyro.y), radians(sensors->gyro.z);

    // Get command reference
    UpdateHorizonReference(setpoint);

    /* MPC solve */
    // Solve optimization problem using ADMM
    solve_admm(&problem, &params);
 
    // // DEBUG_PRINT("Uhrz[0] = [%.2f, %.2f]\n", (double)(Uhrz[0](0)), (double)(Uhrz[0](1)));
    // // DEBUG_PRINT("ZU[0] = [%.2f, %.2f]\n", (double)(ZU_new[0](0)), (double)(ZU_new[0](1)));
    // // DEBUG_PRINT("YU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(YU[0].data[0]), (double)(YU[0].data[1]), (double)(YU[0].data[2]), (double)(YU[0].data[3]));
    // // DEBUG_PRINT("info.pri_res: %f\n", (double)(info.pri_res));
    // // DEBUG_PRINT("info.dua_res: %f\n", (double)(info.dua_res));
    // result =  info.status_val * info.iter;
    // // DEBUG_PRINT("%d %d %d \n", info.status_val, info.iter, mpcTime);
    // // DEBUG_PRINT("%.2f, %.2f, %.2f, %.2f \n", (double)(Xref[0](5)), (double)(Uhrz[0](2)), (double)(Uhrz[0](3)), (double)(ZU_new[0](0)));
  }

  // if (RATE_DO_EXECUTE(LQR_RATE, tick)) {
  //   // Reference from MPC
  //   Ulqr = -(Kinf) * (x0 - Xhrz[1]) + ZU_new[0];
    
  //   /* Output control */
  //   if (setpoint->mode.z == modeDisable) {
  //     control->normalizedForces[0] = 0.0f;
  //     control->normalizedForces[1] = 0.0f;
  //     control->normalizedForces[2] = 0.0f;
  //     control->normalizedForces[3] = 0.0f;
  //   } else {
  //     control->normalizedForces[0] = Ulqr(0) + u_hover[0];  // PWM 0..1
  //     control->normalizedForces[1] = Ulqr(1) + u_hover[1];
  //     control->normalizedForces[2] = Ulqr(2) + u_hover[2];
  //     control->normalizedForces[3] = Ulqr(3) + u_hover[3];
  //   } 
  //   control->controlMode = controlModePWM;
  // }
  // // DEBUG_PRINT("pwm = [%.2f, %.2f]\n", (double)(control->normalizedForces[0]), (double)(control->normalizedForces[1]));

  // // control->normalizedForces[0] = 0.0f;
  // // control->normalizedForces[1] = 0.0f;
  // // control->normalizedForces[2] = 0.0f;
  // // control->normalizedForces[3] = 0.0f;
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