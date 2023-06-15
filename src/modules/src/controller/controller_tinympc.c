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

// #include <Eigen.h>

#ifdef __cplusplus
extern "C" {
#endif

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

#define DEBUG_MODULE "CONTROLLER_TINYMPC"
#include "debug.h"
#include "cfassert.h"


static SemaphoreHandle_t runTaskSemaphore;
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

#define ONE_SECOND 1000
#define UPDATE_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz


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


// Macro variables
#define DT 0.02f       // dt
#define NSTATES 12    // no. of states (error state)
#define NINPUTS 4     // no. of controls
#define NHORIZON 3   // horizon steps (NHORIZON states and NHORIZON-1 controls)
#define NSIM NHORIZON      // length of reference trajectory
#define MPC_RATE RATE_50_HZ  // control frequency

#define ONE_SECOND 1000

// #include "params_50hz_agg.h"
#include "tinympc/data/params_50hz.h"


/////// BRESCIANINI VARS TO DELETE LATER


static struct mat33 CRAZYFLIE_INERTIA =
    {{{16.6e-6f, 0.83e-6f, 0.72e-6f},
      {0.83e-6f, 16.6e-6f, 1.8e-6f},
      {0.72e-6f, 1.8e-6f, 29.3e-6f}}};


// tau is a time constant, lower -> more aggressive control (weight on position error)
// zeta is a damping factor, higher -> more damping (weight on velocity error)

static float tau_xy = 0.3;
static float zeta_xy = 0.85; // this gives good performance down to 0.4, the lower the more aggressive (less damping)

static float tau_z = 0.3;
static float zeta_z = 0.85;

// time constant of body angle (thrust direction) control
static float tau_rp = 0.25;
// what percentage is yaw control speed in terms of roll/pitch control speed \in [0, 1], 0 means yaw not controlled
static float mixing_factor = 1.0;

// time constant of rotational rate control
static float tau_rp_rate = 0.015;
static float tau_yaw_rate = 0.0075;

// minimum and maximum thrusts
static float coll_min = 1;  // in Gs?
static float coll_max = 18;
// if too much thrust is commanded, which axis is reduced to meet maximum thrust?
// 1 -> even reduction across x, y, z
// 0 -> z gets what it wants (eg. maintain height at all costs)
static float thrust_reduction_fairness = 0.25;

// minimum and maximum body rates
static float omega_rp_max = 30;  // in rad/s
static float omega_yaw_max = 10;
static float heuristic_rp = 12;
static float heuristic_yaw = 5;


///////

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
uint32_t mpcTime = 0;
float u_hover = 0.6f;

float setpoint_z = 0.1f;
float setpoint_x = 0.0f;
int z_sign = 1;
int8_t result = 0;

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

static void tinympcControllerTask(void* parameters);

STATIC_MEM_TASK_ALLOC(tinympcTask, TINYMPC_TASK_STACKSIZE);

void controllerTinyMPCInit(void)
{
  DEBUG_PRINT("init tinympc controller\n");

  if (isInit) {
    return;
  }

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
  DEBUG_PRINT("copying tinympc data to stab\n");
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

  // rateSupervisorInit(&rateSupervisorContext, nowMs, ONE_SECOND, MPC_RATE - 1, MPC_RATE + 1, 1);

  while (true) {
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
    nowMs = T2M(xTaskGetTickCount()); // would be nice if this had a precision higher than 1ms...

    // Update task data with most recent stabilizer loop data
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(&setpoint_task, &setpoint_data, sizeof(setpoint_t));
    memcpy(&sensors_task, &sensors_data, sizeof(sensorData_t));
    memcpy(&state_task, &state_data, sizeof(state_t));
    memcpy(&control_task, &control_data, sizeof(control_t));
    xSemaphoreGive(dataMutex);

    
    /* Controller rate */
    if (nowMs >= nextPredictionMs) {
      nextPredictionMs = nowMs + (1000.0f / MPC_RATE);

      DEBUG_PRINT("in tinympc controller task loop\n");

      // control_task.normalizedForces[0] = 1;
      // control_task.normalizedForces[1] = 1;
      // control_task.normalizedForces[2] = 1;
      // control_task.normalizedForces[3] = 1;
      // control_task.normalizedForces[0] = u_hover;
      // control_task.normalizedForces[1] = u_hover;
      // control_task.normalizedForces[2] = u_hover;
      // control_task.normalizedForces[3] = u_hover;

    // DEBUG_PRINT("control: %f %f %f %f\n", (double)control.normalizedForces[0], (double)control.normalizedForces[1], (double)control.normalizedForces[2], (double)control.normalizedForces[3]);
      
      // Get current time
      uint64_t startTimestamp = usecTimestamp();

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
      xg_data[0]  = setpoint_data.position.x;
      xg_data[1]  = setpoint_data.position.y;
      xg_data[2]  = setpoint_data.position.z;
      xg_data[6]  = setpoint_data.velocity.x;
      xg_data[7]  = setpoint_data.velocity.y;
      xg_data[8]  = setpoint_data.velocity.z;
      xg_data[9]  = setpoint_data.attitudeRate.roll;
      xg_data[10] = setpoint_data.attitudeRate.pitch;
      xg_data[11] = setpoint_data.attitudeRate.yaw;
      struct vec desired_rpy = mkvec(radians(setpoint_data.attitude.roll), 
                                    radians(setpoint_data.attitude.pitch), 
                                    radians(setpoint_data.attitude.yaw));
      struct quat attitude = rpy2quat(desired_rpy);
      struct vec phi = quat2rp(qnormalize(attitude));  
      xg_data[3] = phi.x;
      xg_data[4] = phi.y;
      xg_data[5] = phi.z;
      
      /* Get current state (initial state for MPC) */
      // delta_x = x - x_bar; x_bar = 0
      // Positon error, [m]
      x0_data[0] = state_data.position.x;
      x0_data[1] = state_data.position.y;
      x0_data[2] = state_data.position.z;
      // Body velocity error, [m/s]                          
      x0_data[6] = state_data.velocity.x;
      x0_data[7] = state_data.velocity.y;
      x0_data[8] = state_data.velocity.z;
      // Angular rate error, [rad/s]
      x0_data[9]  = radians(sensors_data.gyro.x);   
      x0_data[10] = radians(sensors_data.gyro.y);
      x0_data[11] = radians(sensors_data.gyro.z);
      attitude = mkquat(
        state_data.attitudeQuaternion.x,
        state_data.attitudeQuaternion.y,
        state_data.attitudeQuaternion.z,
        state_data.attitudeQuaternion.w);  // current attitude
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
      mpcTime = usecTimestamp() - startTimestamp;

      // DEBUG_PRINT("U[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(U[0].data[0]), (double)(U[0].data[1]), (double)(U[0].data[2]), (double)(U[0].data[3]));
      // DEBUG_PRINT("ZU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(ZU[0].data[0]), (double)(ZU[0].data[1]), (double)(ZU[0].data[2]), (double)(ZU[0].data[3]));
      // DEBUG_PRINT("YU[0] = [%.2f, %.2f, %.2f, %.2f]\n", (double)(YU[0].data[0]), (double)(YU[0].data[1]), (double)(YU[0].data[2]), (double)(YU[0].data[3]));
      // DEBUG_PRINT("info.pri_res: %f\n", (double)(info.pri_res));
      // DEBUG_PRINT("info.dua_res: %f\n", (double)(info.dua_res));
      // result =  info.status_val * info.iter;
      // DEBUG_PRINT("%d %d %d \n", info.status_val, info.iter, mpcTime);
      // DEBUG_PRINT("[%.2f, %.2f, %.2f]\n", (double)(x0_data[0]), (double)(x0_data[1]), (double)(x0_data[2]));

      /* Output control */
      if (setpoint_data.mode.z == modeDisable) {
        control_task.normalizedForces[0] = 0.0f;
        control_task.normalizedForces[1] = 0.0f;
        control_task.normalizedForces[2] = 0.0f;
        control_task.normalizedForces[3] = 0.0f;
      } else {
        control_task.normalizedForces[0] = U[0].data[0] + u_hover;  // PWM 0..1
        control_task.normalizedForces[1] = U[0].data[1] + u_hover;
        control_task.normalizedForces[2] = U[0].data[2] + u_hover;
        control_task.normalizedForces[3] = U[0].data[3] + u_hover;
      } 

      DEBUG_PRINT("control: %f %f %f %f\n", (double)control_task.normalizedForces[0], (double)control_task.normalizedForces[1], (double)control_task.normalizedForces[2], (double)control_task.normalizedForces[3]);
    }
    
    control_task.controlMode = controlModePWM;

    // Copy the controls calculated by the task loop to the global control_data
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(&control_data, &control_task, sizeof(control_t));
    xSemaphoreGive(dataMutex);

    // //////////////////////////////////////////////////////////////////////
    // /////// BRESCIANINI CONTROLLER CORE TO DELETE LATER
    // //////////////////////////////////////////////////////////////////////

    // static float control_omega[3];
    // static struct vec control_torque;
    // static float control_thrust;

    // // define this here, since we do body-rate control_task at 1000Hz below the following if statement
    // float omega[3] = {0};
    // omega[0] = radians(sensors_task.gyro.x);
    // omega[1] = radians(sensors_task.gyro.y);
    // omega[2] = radians(sensors_task.gyro.z);

    // // if (RATE_DO_EXECUTE(UPDATE_RATE, tick)) {
    // if (nowMs >= nextPredictionMs) {
    //   nextPredictionMs = nowMs + (1000.0f / UPDATE_RATE);

    //   // desired accelerations
    //   struct vec accDes = vzero();
    //   // desired thrust
    //   float collCmd = 0;

    //   // attitude error as computed by the reduced attitude controller
    //   struct quat attErrorReduced = qeye();

    //   // attitude error as computed by the full attitude controller
    //   struct quat attErrorFull = qeye();

    //   // desired attitude as computed by the full attitude controller
    //   struct quat attDesiredFull = qeye();

    //   // current attitude
    //   struct quat attitude = mkquat(
    //     state_task.attitudeQuaternion.x,
    //     state_task.attitudeQuaternion.y,
    //     state_task.attitudeQuaternion.z,
    //     state_task.attitudeQuaternion.w);

    //   // inverse of current attitude
    //   struct quat attitudeI = qinv(attitude);

    //   // body frame -> inertial frame :  vI = R * vB
    //   // float R[3][3] = {0};
    //   // struct quat q = attitude;
    //   // R[0][0] = q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z;
    //   // R[0][1] = 2 * q.x * q.y - 2 * q.w * q.z;
    //   // R[0][2] = 2 * q.x * q.z + 2 * q.w * q.y;

    //   // R[1][0] = 2 * q.x * q.y + 2 * q.w * q.z;
    //   // R[1][1] = q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z;
    //   // R[1][2] = 2 * q.y * q.z - 2 * q.w * q.x;

    //   // R[2][0] = 2 * q.x * q.z - 2 * q.w * q.y;
    //   // R[2][1] = 2 * q.y * q.z + 2 * q.w * q.x;
    //   // R[2][2] = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;

    //   // We don't need all terms of R, only compute the necessary parts
    //   // projections of e3_B to Ixyz

    //   float R02 = 2 * attitude.x * attitude.z + 2 * attitude.w * attitude.y;
    //   float R12 = 2 * attitude.y * attitude.z - 2 * attitude.w * attitude.x;
    //   float R22 = attitude.w * attitude.w - attitude.x * attitude.x - attitude.y * attitude.y + attitude.z * attitude.z;

    //   // a few temporary quaternions
    //   struct quat temp1 = qeye();
    //   struct quat temp2 = qeye();

    //   // compute the position and (body) velocity errors
    //   struct vec pError = mkvec(setpoint_task.position.x - state_task.position.x,
    //                             setpoint_task.position.y - state_task.position.y,
    //                             setpoint_task.position.z - state_task.position.z);

    //   struct vec vError = mkvec(setpoint_task.velocity.x - state_task.velocity.x,
    //                             setpoint_task.velocity.y - state_task.velocity.y,
    //                             setpoint_task.velocity.z - state_task.velocity.z);


    //   // ====== LINEAR CONTROL ======

    //   // compute desired accelerations in X, Y and Z
    //   accDes.x = 0;
    //   accDes.x += 1.0f / tau_xy / tau_xy * pError.x;
    //   accDes.x += 2.0f * zeta_xy / tau_xy * vError.x;
    //   accDes.x += setpoint_task.acceleration.x;  // m/s^2
    //   accDes.x = constrain(accDes.x, -coll_max, coll_max);

    //   accDes.y = 0;
    //   accDes.y += 1.0f / tau_xy / tau_xy * pError.y;
    //   accDes.y += 2.0f * zeta_xy / tau_xy * vError.y;
    //   accDes.y += setpoint_task.acceleration.y;
    //   accDes.y = constrain(accDes.y, -coll_max, coll_max);

    //   accDes.z = GRAVITY_MAGNITUDE;  // compensate gravity
    //   accDes.z += 1.0f / tau_z / tau_z * pError.z;
    //   accDes.z += 2.0f * zeta_z / tau_z * vError.z;
    //   accDes.z += setpoint_task.acceleration.z;
    //   accDes.z = constrain(accDes.z, -coll_max, coll_max);


    //   // ====== THRUST CONTROL ======

    //   // compute commanded thrust required to achieve the z acceleration
    //   collCmd = accDes.z / R22;  // eq.(44)

    //   if (fabsf(collCmd) > coll_max) {
    //     // exceeding the thrust threshold
    //     // we compute a reduction factor r based on fairness f \in [0,1] such that:
    //     // collMax^2 = (r*x)^2 + (r*y)^2 + (r*f*z + (1-f)z + g)^2
    //     float x = accDes.x;
    //     float y = accDes.y;
    //     float z = accDes.z - GRAVITY_MAGNITUDE;
    //     float g = GRAVITY_MAGNITUDE;
    //     float f = constrain(thrust_reduction_fairness, 0, 1);

    //     float r = 0;

    //     // solve as a quadratic
    //     float a = powf(x, 2) + powf(y, 2) + powf(z*f, 2);
    //     if (a<0) { a = 0; }

    //     float b = 2 * z*f*((1-f)*z + g);
    //     float c = powf(coll_max, 2) - powf((1-f)*z + g, 2);
    //     if (c<0) { c = 0; }

    //     if (fabsf(a)<1e-6f) {
    //       r = 0;
    //     } else {
    //       float sqrtterm = powf(b, 2) + 4.0f*a*c;
    //       r = (-b + sqrtf(sqrtterm))/(2.0f*a);
    //       r = constrain(r,0,1);
    //     }
    //     accDes.x = r*x;
    //     accDes.y = r*y;
    //     accDes.z = (r*f+(1-f))*z + g;
    //   }
    //   collCmd = constrain(accDes.z / R22, coll_min, coll_max);

    //   // FYI: this thrust will result in the accelerations
    //   // xdd = R02*coll
    //   // ydd = R12*coll

    //   // a unit vector pointing in the direction of the desired thrust (ie. the direction of body's z axis in the inertial frame)
    //   struct vec zI_des = vnormalize(accDes);

    //   // a unit vector pointing in the direction of the current thrust (in I frame)
    //   struct vec zI_cur = vnormalize(mkvec(R02, R12, R22));

    //   // a unit vector pointing in the direction of the inertial frame z-axis
    //   struct vec zI = mkvec(0, 0, 1);



    //   // ====== REDUCED ATTITUDE CONTROL ======

    //   // compute the error angle between the current and the desired thrust directions
    //   float dotProd = vdot(zI_cur, zI_des);
    //   dotProd = constrain(dotProd, -1, 1);
    //   float alpha = acosf(dotProd);

    //   // the axis around which this rotation needs to occur in the inertial frame (ie. an axis orthogonal to the two)
    //   struct vec rotAxisI = vzero();
    //   if (fabsf(alpha) > 1 * ARCMINUTE) {
    //     rotAxisI = vnormalize(vcross(zI_cur, zI_des));
    //   } else {
    //     rotAxisI = mkvec(1, 1, 0);
    //   }

    //   // the attitude error quaternion
    //   attErrorReduced.w = cosf(alpha / 2.0f);
    //   attErrorReduced.x = sinf(alpha / 2.0f) * rotAxisI.x;
    //   attErrorReduced.y = sinf(alpha / 2.0f) * rotAxisI.y;
    //   attErrorReduced.z = sinf(alpha / 2.0f) * rotAxisI.z;

    //   // choose the shorter rotation
    //   if (sinf(alpha / 2.0f) < 0) {
    //     rotAxisI = vneg(rotAxisI);
    //   }
    //   if (cosf(alpha / 2.0f) < 0) {
    //     rotAxisI = vneg(rotAxisI);
    //     attErrorReduced = qneg(attErrorReduced);
    //   }

    //   attErrorReduced = qnormalize(attErrorReduced);


    //   // ====== FULL ATTITUDE CONTROL ======

    //   // compute the error angle between the inertial and the desired thrust directions 
    //   dotProd = vdot(zI, zI_des);
    //   dotProd = constrain(dotProd, -1, 1);
    //   alpha = acosf(dotProd);

    //   // the axis around which this rotation needs to occur in the inertial frame (ie. an axis orthogonal to the two)
    //   if (fabsf(alpha) > 1 * ARCMINUTE) {
    //     rotAxisI = vnormalize(vcross(zI, zI_des));
    //   } else {
    //     rotAxisI = mkvec(1, 1, 0);
    //   }

    //   // the quaternion corresponding to a roll and pitch around this axis
    //   struct quat attFullReqPitchRoll = mkquat(sinf(alpha / 2.0f) * rotAxisI.x,
    //                                           sinf(alpha / 2.0f) * rotAxisI.y,
    //                                           sinf(alpha / 2.0f) * rotAxisI.z,
    //                                           cosf(alpha / 2.0f));

    //   // the quaternion corresponding to a rotation to the desired yaw
    //   struct quat attFullReqYaw = mkquat(0, 0, sinf(radians(setpoint_task.attitude.yaw) / 2.0f), cosf(radians(setpoint_task.attitude.yaw) / 2.0f));

    //   // the full rotation (roll & pitch, then yaw)
    //   attDesiredFull = qqmul(attFullReqPitchRoll, attFullReqYaw);

    //   // back transform from the current attitude to get the error between rotations
    //   attErrorFull = qqmul(attitudeI, attDesiredFull);

    //   // correct rotation
    //   if (attErrorFull.w < 0) {
    //     attErrorFull = qneg(attErrorFull);
    //     attDesiredFull = qqmul(attitude, attErrorFull);
    //   }

    //   attErrorFull = qnormalize(attErrorFull);
    //   attDesiredFull = qnormalize(attDesiredFull);


    //   // ====== MIXING FULL & REDUCED CONTROL ======

    //   struct quat attError = qeye();

    //   if (mixing_factor <= 0) {
    //     // 100% reduced control_task (no yaw control_task)
    //     attError = attErrorReduced;
    //   } else if (mixing_factor >= 1) {
    //     // 100% full control_task (yaw controlled with same time constant as roll & pitch)
    //     attError = attErrorFull;
    //   } else {
    //     // mixture of reduced and full control_task

    //     // calculate rotation between the two errors
    //     temp1 = qinv(attErrorReduced);
    //     temp2 = qnormalize(qqmul(temp1, attErrorFull));

    //     // by defintion this rotation has the form [cos(alpha/2), 0, 0, sin(alpha/2)]
    //     // where the first element gives the rotation angle, and the last the direction
    //     alpha = 2.0f * acosf(constrain(temp2.w, -1, 1));

    //     // bisect the rotation from reduced to full control_task
    //     temp1 = mkquat(0,
    //                     0,
    //                     sinf(alpha * mixing_factor / 2.0f) * (temp2.z < 0 ? -1 : 1), // rotate in the correct direction
    //                     cosf(alpha * mixing_factor / 2.0f));

    //     attError = qnormalize(qqmul(attErrorReduced, temp1));
    //   }

    //   // ====== COMPUTE CONTROL SIGNALS ======

    //   // compute the commanded body rates
    //   control_omega[0] = 2.0f / tau_rp * attError.x;
    //   control_omega[1] = 2.0f / tau_rp * attError.y;
    //   control_omega[2] = 2.0f / tau_rp * attError.z + radians(setpoint_task.attitudeRate.yaw); // due to the mixing, this will behave with time constant tau_yaw

    //   // apply the rotation heuristic
    //   if (control_omega[0] * omega[0] < 0 && fabsf(omega[0]) > heuristic_rp) { // desired rotational rate in direction opposite to current rotational rate
    //     control_omega[0] = omega_rp_max * (omega[0] < 0 ? -1 : 1); // maximum rotational rate in direction of current rotation
    //   }

    //   if (control_omega[1] * omega[1] < 0 && fabsf(omega[1]) > heuristic_rp) { // desired rotational rate in direction opposite to current rotational rate
    //     control_omega[1] = omega_rp_max * (omega[1] < 0 ? -1 : 1); // maximum rotational rate in direction of current rotation
    //   }

    //   if (control_omega[2] * omega[2] < 0 && fabsf(omega[2]) > heuristic_yaw) { // desired rotational rate in direction opposite to current rotational rate
    //     control_omega[2] = omega_yaw_max * (omega[2] < 0 ? -1 : 1);
    //     // control_omega[2] = omega_rp_max * (omega[2] < 0 ? -1 : 1); // maximum rotational rate in direction of current rotation
    //   }

    //   // scale the commands to satisfy rate constraints
    //   float scaling = 1;
    //   scaling = fmax(scaling, fabsf(control_omega[0]) / omega_rp_max);
    //   scaling = fmax(scaling, fabsf(control_omega[1]) / omega_rp_max);
    //   scaling = fmax(scaling, fabsf(control_omega[2]) / omega_yaw_max);

    //   control_omega[0] /= scaling;
    //   control_omega[1] /= scaling;
    //   control_omega[2] /= scaling;  // rad/s
    //   control_thrust = collCmd;  // m/s^2
    // }

    // if (setpoint_task.mode.z == modeDisable) {
    //   // DEBUG_PRINT("setpoint_task.mode.z == modeDisable\n");
    //   control_task.thrustSi = 0.0f;
    //   control_task.torque[0] =  0.0f;
    //   control_task.torque[1] =  0.0f;
    //   control_task.torque[2] =  0.0f;
    // } else {
    //   // DEBUG_PRINT("setpoint_task.mode.z != modeDisable\n");
    //   // control_task the body torques
    //   struct vec omegaErr = mkvec((control_omega[0] - omega[0])/tau_rp_rate,
    //                       (control_omega[1] - omega[1])/tau_rp_rate,
    //                       (control_omega[2] - omega[2])/tau_yaw_rate);

    //   // update the commanded body torques based on the current error in body rates
    //   control_torque = mvmul(CRAZYFLIE_INERTIA, omegaErr);

    //   // control_task.thrustSi = control_thrust * 0.033; // force to provide control_thrust
    //   control_task.thrustSi = control_thrust * CF_MASS; // force to provide control_thrust
    //   control_task.torqueX = control_torque.x;
    //   control_task.torqueY = control_torque.y;
    //   control_task.torqueZ = control_torque.z;

    // }

    // control_task.controlMode = controlModeForceTorque;


    // //////////////////////////////////////////////////////////////////////
    // /////// END OF BRESCIANINI CONTROLLER CORE ///////////////////////////
    // //////////////////////////////////////////////////////////////////////

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

LOG_GROUP_START(ctrlTinyMPC)

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

LOG_ADD(LOG_FLOAT, u0, &(control_data.normalizedForces[0]))
LOG_ADD(LOG_FLOAT, u1, &(control_data.normalizedForces[1]))
LOG_ADD(LOG_FLOAT, u2, &(control_data.normalizedForces[2]))
LOG_ADD(LOG_FLOAT, u3, &(control_data.normalizedForces[3]))

LOG_ADD(LOG_FLOAT, yu0, &(YU_data[0]))
LOG_ADD(LOG_FLOAT, yu1, &(YU_data[1]))
LOG_ADD(LOG_FLOAT, yu2, &(YU_data[2]))
LOG_ADD(LOG_FLOAT, yu3, &(YU_data[3]))

LOG_GROUP_STOP(ctrlTinyMPC)

#ifdef __cplusplus
}
#endif
