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

#define DEBUG_MODULE "CONTROLLER_TINYMPC"
#include "debug.h"
#include "cfassert.h"


static SemaphoreHandle_t runTaskSemaphore;
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

#define NX  13  // no. state variable s       [position (3), attitude (4), body velocity (3), angular rate (3)]
#define NXt 12  // no. state error variables  [position (3), attitude (3), body velocity (3), angular rate (3)]
#define NU  4   // no. control input          [pwm1, pwm2, pwm3, pwm4] from [0..1]
#define LQR_RATE RATE_500_HZ  // control frequency

static float K[NU][NXt] = {
  {-0.209415f,0.190469f,0.471501f,-0.643639f,-0.716745f,-0.335579f,-0.130498f,0.118040f,0.224146f,-0.046397f,-0.052328f,-0.182753f},
  {0.207305f,0.160584f,0.471501f,-0.527361f,0.705213f,0.368519f,0.128825f,0.098303f,0.224146f,-0.036927f,0.051228f,0.199755f},
  {0.138321f,-0.163029f,0.471501f,0.539924f,0.449777f,-0.451224f,0.084350f,-0.100176f,0.224146f,0.038094f,0.031069f,-0.242445f},
  {-0.136211f,-0.188024f,0.471501f,0.631076f,-0.438245f,0.418285f,-0.082677f,-0.116168f,0.224146f,0.045230f,-0.029969f,0.225443f},
};

static float x_error[NXt];  
static float control_input[NU];
static float u_hover = 0.66f;  // ~ mass/max_thrust

// Struct for logging position information
static bool isInit = false;

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
  // DEBUG_PRINT("init tinympc controller\n");

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
      nextPredictionMs = nowMs + (1000.0f / LQR_RATE);

      // DEBUG_PRINT("in tinympc controller task loop\n");
      
      // Get current time
      uint64_t startTimestamp = usecTimestamp();

      //////// LQR

      // Positon error, [m]
      x_error[0] = state_task.position.x - 1*setpoint_task.position.x;
      x_error[1] = state_task.position.y - 1*setpoint_task.position.y;
      x_error[2] = state_task.position.z - 1*setpoint_task.position.z;

      // Frame velocity error, [m/s]                          
      x_error[6] = state_task.velocity.x - 1*setpoint_task.velocity.x;
      x_error[7] = state_task.velocity.y - 1*setpoint_task.velocity.y;
      x_error[8] = state_task.velocity.z - 1*setpoint_task.velocity.z;

      // Angular rate error, [rad/s]
      x_error[9]  = radians(sensors_task.gyro.x - 1*setpoint_task.attitudeRate.roll);   
      x_error[10] = radians(sensors_task.gyro.y - 1*setpoint_task.attitudeRate.pitch);
      x_error[11] = radians(sensors_task.gyro.z - 1*setpoint_task.attitudeRate.yaw);

      // struct quat attitude_g = qeye();  // goal attitude
      // struct quat attitude_g = mkquat(setpoint_task.attitudeQuaternion.x, 
      //                                 setpoint_task.attitudeQuaternion.y, 
      //                                 setpoint_task.attitudeQuaternion.z, 
      //                                 setpoint_task.attitudeQuaternion.w);  // do not have quat cmd

      struct vec desired_rpy = mkvec(radians(setpoint_task.attitude.roll), 
                                    radians(setpoint_task.attitude.pitch), 
                                    radians(setpoint_task.attitude.yaw));
      struct quat attitude_g = rpy2quat(desired_rpy);

      struct quat attitude = mkquat(
        state_task.attitudeQuaternion.x,
        state_task.attitudeQuaternion.y,
        state_task.attitudeQuaternion.z,
        state_task.attitudeQuaternion.w);  // current attitude (right pitch)

      struct quat attitude_gI = qinv(attitude_g);
      struct quat q_error = qnormalize(qqmul(attitude_gI, attitude));
      struct vec phi = quat2rp(q_error);  // quaternion to Rodriquez parameters
      
      // Attitude error
      x_error[3] = phi.x;
      x_error[4] = phi.y;
      x_error[5] = phi.z;

      // Matrix multiplication, compute control_task input
      for (int i = 0; i < STABILIZER_NR_OF_MOTORS; i++){
        control_input[i] = 0;
        for (int j = 0; j < NXt; j++) {
          control_input[i] += -K[i][j] * x_error[j] * 1.0f;
        }
      }

      // Debug printing
      // DEBUG_PRINT("U = [%.2f, %.2f, %.2f, %.2f]\n", (double)(control_input[0]), (double)(control_input[1]), (double)(control_input[2]), (double)(control_input[3]));

      /* Output control_task */
      if (setpoint_task.mode.z == modeDisable) {
        control_task.normalizedForces[0] = 0.0f;
        control_task.normalizedForces[1] = 0.0f;
        control_task.normalizedForces[2] = 0.0f;
        control_task.normalizedForces[3] = 0.0f;
      } else {
        control_task.normalizedForces[0] = control_input[0] + u_hover;  // PWM 0..1
        control_task.normalizedForces[1] = control_input[1] + u_hover;
        control_task.normalizedForces[2] = control_input[2] + u_hover;
        control_task.normalizedForces[3] = control_input[3] + u_hover;
      } 
    }

    control_task.controlMode = controlModePWM;
      
      ////////////////////////
      ////// END LQR
      //////////////////////////////

    // Copy the controls calculated by the task loop to the global control_data
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(&control_data, &control_task, sizeof(control_t));
    xSemaphoreGive(dataMutex);
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
