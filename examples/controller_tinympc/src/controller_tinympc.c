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
#define NHORIZON 5    // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define NRUN 100      // length of reference trajectory
#define MPC_RATE RATE_50_HZ  // control frequency

/* Allocate global variables for MPC */

// Precompute data offline
sfloat A_data[NSTATES*NSTATES]= {
  1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,-0.003849f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,-0.392400f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.003849f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.392400f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.020000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.020000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.020000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,-0.000013f,0.000000f,0.010000f,0.000000f,0.000000f,0.000000f,-0.001999f,0.000000f,1.000000f,0.000000f,0.000000f,
  0.000013f,0.000000f,0.000000f,0.000000f,0.010000f,0.000000f,0.001999f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.010000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,
};
sfloat B_data[NSTATES*NINPUTS]= {
  0.000019f,0.000019f,0.000962f,-0.029620f,0.030207f,-0.004066f,0.004101f,0.004021f,0.096236f,-5.923901f,6.041486f,-0.813298f,
  -0.000018f,0.000017f,0.000962f,-0.027140f,-0.027503f,0.005295f,-0.003734f,0.003685f,0.096236f,-5.428009f,-5.500535f,1.059020f,
  -0.000019f,-0.000019f,0.000962f,0.029845f,-0.029577f,-0.001855f,-0.004016f,-0.004052f,0.096236f,5.968959f,-5.915321f,-0.370998f,
  0.000017f,-0.000017f,0.000962f,0.026915f,0.026872f,0.000626f,0.003648f,-0.003654f,0.096236f,5.382950f,5.374370f,0.125275f,
};
sfloat f_data[NSTATES] = {0.0f};
sfloat Kinf_data[NINPUTS*NSTATES] = {
  0.074926f,-0.087307f,-0.170980f,0.183361f,
  0.142503f,0.116560f,-0.104167f,-0.154896f,
  1.397135f,1.397135f,1.397135f,1.397135f,
  -0.653577f,-0.529393f,0.472648f,0.710322f,
  0.334850f,-0.391543f,-0.788093f,0.844787f,
  -0.272398f,0.270565f,-0.275297f,0.277130f,
  0.055876f,-0.065199f,-0.129091f,0.138415f,
  0.107376f,0.087484f,-0.078152f,-0.116708f,
  0.583418f,0.583418f,0.583418f,0.583418f,
  -0.051908f,-0.041623f,0.037125f,0.056407f,
  0.025881f,-0.030376f,-0.062909f,0.067404f,
  -0.287018f,0.285076f,-0.290085f,0.292027f,
};
sfloat Pinf_data[NSTATES*NSTATES] = {
  376.096473f,-0.013921f,0.000000f,0.061321f,338.072148f,0.081573f,112.687691f,-0.010361f,0.000000f,0.004544f,1.647141f,0.084648f,
  -0.013921f,376.082931f,-0.000000f,-338.012160f,-0.061321f,-0.032658f,-0.010361f,112.677580f,0.000000f,-1.642698f,-0.004544f,-0.033889f,
  -0.000000f,-0.000000f,208.790813f,0.000000f,-0.000000f,-0.000000f,-0.000000f,-0.000000f,16.505696f,0.000000f,-0.000000f,-0.000000f,
  0.061321f,-338.012160f,0.000000f,1446.515966f,0.277569f,0.153801f,0.046110f,-247.529765f,0.000000f,7.178445f,0.021058f,0.159773f,
  338.072148f,-0.061321f,-0.000000f,0.277569f,1446.790778f,0.384124f,247.575126f,-0.046110f,-0.000000f,0.021058f,7.199401f,0.399039f,
  0.081573f,-0.032658f,-0.000000f,0.153801f,0.384124f,97.779835f,0.062145f,-0.024881f,-0.000000f,0.012386f,0.030932f,0.949901f,
  112.687691f,-0.010361f,-0.000000f,0.046110f,247.575126f,0.062145f,66.874239f,-0.007743f,-0.000000f,0.003439f,1.217014f,0.064496f,
  -0.010361f,112.677580f,-0.000000f,-247.529765f,-0.046110f,-0.024881f,-0.007743f,66.866651f,0.000000f,-1.213627f,-0.003439f,-0.025822f,
  0.000000f,-0.000000f,16.505696f,0.000000f,0.000000f,-0.000000f,0.000000f,-0.000000f,7.227418f,0.000000f,-0.000000f,-0.000000f,
  0.004544f,-1.642698f,0.000000f,7.178445f,0.021058f,0.012386f,0.003439f,-1.213627f,0.000000f,1.043715f,0.001687f,0.013041f,
  1.647141f,-0.004544f,-0.000000f,0.021058f,7.199401f,0.030932f,1.217014f,-0.003439f,-0.000000f,0.001687f,1.045421f,0.032567f,
  0.084648f,-0.033889f,-0.000000f,0.159773f,0.399039f,0.949901f,0.064496f,-0.025822f,-0.000000f,0.013041f,0.032567f,1.495774f,
};
sfloat Quu_inv_data[NINPUTS*NINPUTS] = {
  0.172741f,0.050332f,0.167886f,0.047495f,
  0.050332f,0.171730f,0.049139f,0.167253f,
  0.167886f,0.049139f,0.175090f,0.046339f,
  0.047495f,0.167253f,0.046339f,0.177367f,
};
sfloat AmBKt_data[NSTATES*NSTATES] = {
  0.999991f,-0.000000f,-0.000000f,0.000000f,0.003807f,0.000000f,0.019993f,-0.000000f,-0.000000f,0.000000f,0.000009f,0.000000f,
  -0.000000f,0.999991f,-0.000000f,-0.003806f,-0.000000f,-0.000000f,-0.000000f,0.019993f,-0.000000f,-0.000009f,-0.000000f,-0.000000f,
  -0.000000f,-0.000000f,0.994622f,0.000000f,-0.000000f,-0.000000f,-0.000000f,-0.000000f,0.017754f,0.000000f,-0.000000f,-0.000000f,
  0.000018f,0.014662f,0.000000f,0.933049f,0.000075f,0.000032f,0.000013f,0.011028f,0.000000f,0.004707f,0.000006f,0.000033f,
  -0.014649f,-0.000018f,-0.000000f,0.000075f,0.933106f,0.000080f,-0.011019f,-0.000013f,0.000000f,0.000006f,0.004711f,0.000083f,
  0.000335f,-0.000134f,-0.000000f,0.000577f,0.001444f,0.996775f,0.000246f,-0.000098f,-0.000000f,0.000043f,0.000107f,0.006602f,
  -0.001989f,-0.000002f,-0.000000f,0.000010f,0.383318f,0.000011f,0.998504f,-0.000002f,-0.000000f,0.000001f,0.001281f,0.000011f,
  -0.000002f,-0.001991f,-0.000000f,-0.383310f,-0.000010f,-0.000004f,-0.000002f,0.998503f,-0.000000f,-0.001281f,-0.000001f,-0.000005f,
  -0.000000f,-0.000000f,-0.537819f,0.000000f,-0.000000f,-0.000000f,-0.000000f,-0.000000f,0.775417f,0.000000f,-0.000000f,-0.000000f,
  0.003504f,2.932430f,0.000000f,-13.390115f,0.014964f,0.006424f,0.002565f,2.205670f,0.000000f,-0.058661f,0.001107f,0.006666f,
  -2.929753f,-0.003504f,-0.000000f,0.014963f,-13.378707f,0.016071f,-2.203712f,-0.002565f,-0.000000f,0.001107f,-0.057819f,0.016676f,
  0.066993f,-0.026783f,-0.000000f,0.115450f,0.288773f,-0.644926f,0.049258f,-0.019693f,-0.000000f,0.008569f,0.021434f,0.320463f,
};
sfloat coeff_d2p_data[NSTATES*NINPUTS] = {
  0.074927f,0.142503f,1.397135f,-0.653575f,0.334852f,-0.271611f,0.055876f,0.107376f,0.583418f,-0.051908f,0.025881f,-0.287010f,
  -0.087308f,0.116561f,1.397135f,-0.529393f,-0.391547f,0.269611f,-0.065200f,0.087485f,0.583418f,-0.041623f,-0.030376f,0.285066f,
  -0.170980f,-0.104168f,1.397135f,0.472648f,-0.788090f,-0.274808f,-0.129091f,-0.078153f,0.583418f,0.037125f,-0.062909f,-0.290080f,
  0.183361f,-0.154896f,1.397135f,0.710321f,0.844785f,0.276807f,0.138414f,-0.116708f,0.583418f,0.056407f,0.067403f,0.292024f,
};

  
// Create data array, all zero initialization
static sfloat x0_data[NSTATES] = {0.0f};       // initial state
static sfloat xg_data[NSTATES] = {0.0f};       // goal state (if not tracking)
static sfloat ug_data[NINPUTS] = {0.0f};       // goal input 
static sfloat X_data[NSTATES * NHORIZON] = {0.0f};        // X in MPC solve
static sfloat U_data[NINPUTS * (NHORIZON - 1)] = {0.0f};  // U in MPC solve
static sfloat d_data[NINPUTS * (NHORIZON - 1)] = {0.0f};
static sfloat p_data[NSTATES * NHORIZON] = {0.0f};
static sfloat Q_data[NSTATES * NSTATES] = {0.0f};
static sfloat R_data[NINPUTS * NINPUTS] = {0.0f};
static sfloat q_data[NSTATES*(NHORIZON-1)] = {0.0f};
static sfloat r_data[NINPUTS*(NHORIZON-1)] = {0.0f};
static sfloat r_tilde_data[NINPUTS*(NHORIZON-1)] = {0.0f};
static sfloat Acu_data[NINPUTS * NINPUTS] = {0.0f};  
static sfloat YU_data[NINPUTS * (NHORIZON - 1)] = {0.0f};
static sfloat umin_data[NINPUTS] = {0.0f};
static sfloat umax_data[NINPUTS] = {0.0f};
static sfloat temp_data[NINPUTS + 2*NINPUTS*(NHORIZON - 1)] = {0.0f};

// Created matrices
static Matrix Xref[NRUN];
static Matrix Uref[NRUN - 1];
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
#define U_HOVER (30.0f / 60.0f);  // pwm, = weight/max thrust 

void controllerOutOfTreeInit(void) {
  if (isInit) {
    return;
  }
  
  /* Start MPC initialization*/
  
  tiny_InitModel(&model, NSTATES, NINPUTS, NHORIZON, 0, 0, DT);
  tiny_InitSettings(&stgs);

  stgs.rho_init = 1e0;  // IMPORTANT (select offline, associated with precomp.)

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
  sfloat Qdiag[NSTATES] = {10, 10, 10, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  slap_SetDiagonal(data.Q, Qdiag, NSTATES);
  slap_SetIdentity(data.R, 1);
  slap_AddIdentity(data.R, work.rho); // \tilde{R}
  tiny_InitDataLinearCostFromArray(&work, q, r, r_tilde, q_data, r_data, r_tilde_data);

  // Set up constraints 
  tiny_SetInputBound(&work, Acu_data, umin_data, umax_data);
  slap_SetConst(data.ucu, 0.5);   // UPPER CONTROL BOUND 
  slap_SetConst(data.lcu, -0.5);  // LOWER CONTROL BOUND 

  // Initialize linear cost (for tracking)
  // tiny_UpdateLinearCost(&work);

  // Solver settings 
  stgs.en_cstr_goal = 0;
  stgs.en_cstr_inputs = 1;
  stgs.en_cstr_states = 0;
  stgs.max_iter = 10;           // limit this if needed
  stgs.verbose = 0;
  stgs.check_termination = 2;
  stgs.tol_abs_dual = 5e-2;
  stgs.tol_abs_prim = 5e-2;

  isInit = true;
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
  uint64_t startTimestamp = usecTimestamp();

  /* Get current tracking errors (initial state for MPC) */
  // Positon error, [m]
  x0_data[0] = state->position.x - 0*setpoint->position.x;
  x0_data[1] = state->position.y - 0*setpoint->position.y;
  x0_data[2] = state->position.z - 0.5f - 0*setpoint->position.z;

  // Body velocity error, [m/s]                          
  x0_data[6] = state->velocity.x - 0*setpoint->velocity.x;
  x0_data[7] = state->velocity.y - 0*setpoint->velocity.y;
  x0_data[8] = state->velocity.z - 0*setpoint->velocity.z;

  // Angular rate error, [rad/s]
  x0_data[9]  = radians(sensors->gyro.x - 0*setpoint->attitudeRate.roll);   
  x0_data[10] = radians(sensors->gyro.y - 0*setpoint->attitudeRate.pitch);
  x0_data[11] = radians(sensors->gyro.z - 0*setpoint->attitudeRate.yaw);

  struct vec desired_rpy = mkvec(radians(setpoint->attitude.roll), 
                                 radians(setpoint->attitude.pitch), 
                                 radians(setpoint->attitude.yaw));
  struct quat attitude_g = rpy2quat(desired_rpy);

  struct quat attitude = mkquat(
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
    state->attitudeQuaternion.w);  // current attitude

  struct quat attitude_gI = qinv(attitude_g);  
  struct quat q_error = qnormalize(qqmul(attitude_gI, attitude));
  struct vec phi = quat2rp(q_error);  // quaternion to Rodriquez parameters
  
  // Attitude error
  x0_data[3] = phi.x;
  x0_data[4] = phi.y;
  x0_data[5] = phi.z;

  /* MPC solve */
  
  // Warm-start by previous solution  // TODO: should I warm-start U with previous ZU
  // tiny_ShiftFill(U, T_ARRAY_SIZE(U));

  // Solve optimization problem using ADMM
  tiny_SolveAdmm(&work);
  uint32_t mpcTime = usecTimestamp() - startTimestamp;

  // DEBUG_PRINT("U[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(U[0].data[0]), (double)(U[0].data[1]), (double)(U[0].data[2]), (double)(U[0].data[3]));
  // DEBUG_PRINT("ZU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(ZU[0].data[0]), (double)(ZU[0].data[1]), (double)(ZU[0].data[2]), (double)(ZU[0].data[3]));
  // DEBUG_PRINT("YU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(YU[0].data[0]), (double)(YU[0].data[1]), (double)(YU[0].data[2]), (double)(YU[0].data[3]));
  // DEBUG_PRINT("info.pri_res: %f\n", (double)(info.pri_res));
  // DEBUG_PRINT("info.dua_res: %f\n", (double)(info.dua_res));
  // DEBUG_PRINT("%d %d %d \n", info.status_val, info.iter, mpcTime);
  // DEBUG_PRINT("[%.2f, %.2f, %.2f]\n", (double)(x0_data[0]), (double)(x0_data[1]), (double)(x0_data[2]));

  /* Output control */
  // if (setpoint->mode.z == modeDisable) {
  //   control->normalizedForces[0] = 0.0f;
  //   control->normalizedForces[1] = 0.0f;
  //   control->normalizedForces[2] = 0.0f;
  //   control->normalizedForces[3] = 0.0f;
  // } else {
    control->normalizedForces[0] = U[0].data[0] + U_HOVER;  // PWM 0..1
    control->normalizedForces[1] = U[0].data[1] + U_HOVER;
    control->normalizedForces[2] = U[0].data[2] + U_HOVER;
    control->normalizedForces[3] = U[0].data[3] + U_HOVER;
  // } 

  // control->normalizedForces[0] = 0.0f;
  // control->normalizedForces[1] = 0.0f;
  // control->normalizedForces[2] = 0.0f;
  // control->normalizedForces[3] = 0.0f;

  control->controlMode = controlModePWM;
}
