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
#define DT 0.01f       // dt
#define NSTATES 12    // no. of states (error state)
#define NINPUTS 4     // no. of controls
#define NHORIZON 3   // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define NSIM NHORIZON      // length of reference trajectory
#define MPC_RATE RATE_100_HZ  // control frequency

/* Allocate global variables for MPC */

// Precompute data offline
sfloat A_data[NSTATES*NSTATES] = {
  1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,-0.001059f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,-0.196200f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.001059f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.196200f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.010000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.010000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.010000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,-0.000002f,0.000000f,0.005000f,0.000000f,0.000000f,0.000000f,-0.000452f,0.000000f,1.000000f,0.000000f,0.000000f,
  0.000002f,0.000000f,0.000000f,0.000000f,0.005000f,0.000000f,0.000452f,0.000000f,0.000000f,0.000000f,1.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.005000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,1.000000f,
};

sfloat B_data[NSTATES*NINPUTS] = {
  -0.000001f,0.000001f,0.000241f,-0.006753f,-0.006786f,0.000484f,-0.000374f,0.000372f,0.048118f,-2.701229f,-2.714496f,0.193725f,
  0.000001f,0.000001f,0.000241f,-0.007437f,0.007462f,-0.000177f,0.000411f,0.000409f,0.048118f,-2.974726f,2.984971f,-0.070864f,
  0.000001f,-0.000001f,0.000241f,0.006761f,0.006807f,-0.000683f,0.000375f,-0.000372f,0.048118f,2.704250f,2.722957f,-0.273147f,
  -0.000001f,-0.000001f,0.000241f,0.007429f,-0.007484f,0.000376f,-0.000412f,-0.000409f,0.048118f,2.971704f,-2.993432f,0.150286f,
};
sfloat Kinf_data[NINPUTS*NSTATES] = {
  -0.185864f,0.183541f,0.159443f,-0.157121f,
  0.180278f,0.166895f,-0.169782f,-0.177391f,
  0.461060f,0.461060f,0.461060f,0.461060f,
  -0.636400f,-0.528924f,0.549587f,0.615736f,
  -0.701818f,0.682786f,0.484211f,-0.465178f,
  -0.407242f,0.422112f,-0.458550f,0.443680f,
  -0.119654f,0.117279f,0.095884f,-0.093509f,
  0.113294f,0.101126f,-0.103829f,-0.110590f,
  0.223558f,0.223558f,0.223558f,0.223558f,
  -0.049461f,-0.033619f,0.035573f,0.047506f,
  -0.060174f,0.058320f,0.025417f,-0.023563f,
  -0.269965f,0.277318f,-0.295330f,0.287977f,
};
sfloat Pinf_data[NSTATES*NSTATES] = {
  6219.799667f,-30.104257f,0.000000f,97.812931f,4777.810197f,304.576237f,1704.896166f,-19.014239f,-0.000000f,5.496384f,48.204434f,154.553087f,
  -30.104257f,6174.062753f,0.000000f,-4628.993867f,-98.177455f,-121.073984f,-19.045550f,1675.676695f,0.000000f,-40.379578f,-5.533559f,-61.443748f,
  0.000000f,0.000000f,19397.558818f,-0.000000f,0.000000f,-0.000000f,0.000000f,0.000000f,4406.817356f,-0.000000f,-0.000000f,-0.000000f,
  97.812931f,-4628.993867f,-0.000000f,14313.359935f,457.323356f,935.892232f,71.075734f,-2756.783970f,-0.000000f,140.246207f,37.854603f,502.119349f,
  4777.810197f,-98.177455f,-0.000000f,457.323356f,15079.580516f,2345.606472f,2870.517759f,-71.197479f,0.000000f,37.812207f,206.405154f,1258.233095f,
  304.576237f,-121.073984f,-0.000000f,935.892232f,2345.606472f,47551.723834f,269.033632f,-107.158588f,-0.000000f,122.446387f,306.492251f,5059.987462f,
  1704.896166f,-19.045550f,-0.000000f,71.075734f,2870.517759f,269.033632f,813.952681f,-12.716063f,-0.000000f,4.546653f,32.336039f,138.213198f,
  -19.014239f,1675.676695f,-0.000000f,-2756.783970f,-71.197479f,-107.158588f,-12.716063f,793.903971f,-0.000000f,-25.147917f,-4.562708f,-55.058131f,
  0.000000f,-0.000000f,4406.817356f,-0.000000f,0.000000f,-0.000000f,0.000000f,0.000000f,2116.748908f,-0.000000f,-0.000000f,-0.000000f,
  5.496384f,-40.379578f,-0.000000f,140.246207f,37.812207f,122.446387f,4.546653f,-25.147917f,-0.000000f,12.910743f,5.450012f,79.888673f,
  48.204434f,-5.533559f,-0.000000f,37.854603f,206.405154f,306.492251f,32.336039f,-4.562708f,-0.000000f,5.450012f,23.367829f,199.908198f,
  154.553087f,-61.443748f,-0.000000f,502.119349f,1258.233095f,5059.987462f,138.213198f,-55.058131f,-0.000000f,79.888673f,199.908198f,3339.522640f,
};
sfloat Quu_inv_data[NINPUTS*NINPUTS] = {
  0.001674f,0.000005f,0.000454f,-0.000006f,
  0.000005f,0.001653f,-0.000004f,0.000474f,
  0.000454f,-0.000004f,0.001668f,0.000010f,
  -0.000006f,0.000474f,0.000010f,0.001650f,
};
sfloat AmBKt_data[NSTATES*NSTATES] = {
  0.999999f,-0.000000f,0.000000f,0.000000f,0.001056f,0.000000f,0.009999f,-0.000000f,-0.000000f,0.000000f,0.000002f,0.000000f,
  -0.000000f,0.999999f,0.000000f,-0.001056f,-0.000000f,-0.000000f,-0.000000f,0.009999f,-0.000000f,-0.000002f,-0.000000f,-0.000000f,
  -0.000000f,0.000000f,0.999556f,-0.000000f,-0.000000f,0.000000f,0.000000f,-0.000000f,0.009785f,0.000000f,-0.000000f,0.000000f,
  0.000199f,0.004924f,-0.000000f,0.983479f,0.000521f,0.000193f,0.000111f,0.003041f,0.000000f,0.003823f,0.000031f,0.000096f,
  -0.004892f,-0.000194f,0.000000f,0.000495f,0.983365f,0.000528f,-0.003040f,-0.000107f,-0.000000f,0.000029f,0.003807f,0.000264f,
  0.000290f,-0.000107f,0.000000f,0.000358f,0.000966f,0.999792f,0.000179f,-0.000066f,0.000000f,0.000024f,0.000066f,0.004870f,
  -0.000269f,-0.000011f,-0.000000f,0.000027f,0.195284f,0.000029f,0.999833f,-0.000006f,-0.000000f,0.000002f,0.000386f,0.000015f,
  -0.000011f,-0.000271f,0.000000f,-0.195290f,-0.000029f,-0.000011f,-0.000006f,0.999833f,-0.000000f,-0.000387f,-0.000002f,-0.000005f,
  -0.000000f,0.000000f,-0.088741f,-0.000000f,-0.000000f,0.000000f,0.000000f,0.000000f,0.956971f,0.000000f,-0.000000f,0.000000f,
  0.079666f,1.969722f,-0.000000f,-6.608473f,0.208275f,0.077162f,0.044246f,1.216274f,0.000000f,0.529017f,0.012233f,0.038572f,
  -1.956879f,-0.077511f,-0.000000f,0.197979f,-6.654142f,0.211288f,-1.215878f,-0.042643f,-0.000000f,0.011433f,0.522833f,0.105609f,
  0.116177f,-0.042814f,0.000000f,0.143387f,0.386515f,-0.083125f,0.071735f,-0.026522f,0.000000f,0.009777f,0.026274f,0.948003f,
};
sfloat coeff_d2p_data[NSTATES*NINPUTS] = {
  0.015998f,-0.013987f,-0.008620f,0.027394f,0.035929f,0.224699f,0.009414f,-0.007982f,-0.005206f,0.000968f,0.002132f,0.031661f,
  -0.015385f,-0.011861f,-0.008620f,0.016714f,-0.034963f,-0.239872f,-0.009008f,-0.006393f,-0.005206f,-0.000637f,-0.002223f,-0.033554f,
  -0.012372f,0.012549f,-0.008620f,-0.017799f,-0.011856f,0.279120f,-0.006544f,0.006841f,-0.005206f,0.000731f,0.002120f,0.038446f,
  0.011759f,0.013300f,-0.008620f,-0.026309f,0.010889f,-0.263946f,0.006138f,0.007534f,-0.005206f,-0.001062f,-0.002029f,-0.036553f,
};
sfloat Q_data[NSTATES*NSTATES] = {
  100.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,100.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,400.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,400.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,4.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,100.000000f,
};
sfloat R_data[NINPUTS*NINPUTS] = {
  400.000000f,0.000000f,0.000000f,0.000000f,
  0.000000f,400.000000f,0.000000f,0.000000f,
  0.000000f,0.000000f,400.000000f,0.000000f,
  0.000000f,0.000000f,0.000000f,400.000000f,
};

sfloat f_data[NSTATES] = {0};
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
#define U_HOVER (37.0f / 60.0f);  // pwm, = weight/max thrust 
float setpoint_z = 0.0f;
float setpoint_x = 0.0f;
int z_sign = 1;

void controllerOutOfTreeInit(void) {
  if (isInit) {
    return;
  }

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
  stgs.check_termination = 2;
  stgs.tol_abs_dual = 10e-2;
  stgs.tol_abs_prim = 10e-2;

  setpoint_z = 0.0f;
  setpoint_x = 0.0f;
  z_sign = 1;

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

  // Rule to take-off and land gradually
  // setpoint_z += z_sign * 0.002f;
  // if (setpoint_z > 0.7f) z_sign = -1;
  // if (z_sign == -1 && setpoint_z < 0.2f) setpoint_z = 0.2f;
  // setpoint_x += 0.002f;
  // if (setpoint_x > 1.0f) setpoint_x = 1.0f;

  /* Get goal state (reference) */
  // xg_data[0]  = setpoint_x; 
  xg_data[0]  = setpoint->position.x;
  xg_data[1]  = setpoint->position.y;
  // xg_data[2]  = setpoint_z; 
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
  DEBUG_PRINT("%d %d\n", info.status_val, info.iter);
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