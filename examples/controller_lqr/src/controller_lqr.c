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
 * controller_lqr.c - App layer application of a quaternion-based LQR.
 * This is one of the best existing LQR controllers you can find online
 * for Crazyflie.
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

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "CONTROLLER_LQR"
#include "debug.h"

void appMain() {
  DEBUG_PRINT("Waiting for activation ...\n");

  while(1) {
    vTaskDelay(M2T(2000));

    // Remove the DEBUG_PRINT.
    // DEBUG_PRINT("Hello World!\n");
  }
}

#define NX  13  // no. state variable s       [position (3), attitude (4), body velocity (3), angular rate (3)]
#define NXt 12  // no. state error variables  [position (3), attitude (3), body velocity (3), angular rate (3)]
#define NU  4   // no. control input          [thrust, torque_x, torque_y, torque_z]

static float K[NU][NXt] = {
  {0.000000f,0.000000f,0.689184f,0.000000f,0.000000f,0.000000f,0.000000f,0.000000f,0.715675f,0.000000f,0.000000f,0.000000f},
  {0.000039f,-0.000787f,0.000000f,0.008870f,0.000443f,0.000036f,0.000058f,-0.001153f,0.000000f,0.000874f,0.000044f,0.000036f},
  {0.000787f,-0.000039f,0.000000f,0.000443f,0.008870f,0.000090f,0.001153f,-0.000058f,0.000000f,0.000044f,0.000874f,0.000090f},
  {0.000085f,-0.000034f,0.000000f,0.000385f,0.000962f,0.001458f,0.000125f,-0.000050f,0.000000f,0.000038f,0.000095f,0.001472f},
};

static float x_error[NXt];  
static float control_input[NU];

// Struct for logging position information
static bool isInit = false;

void controllerOutOfTreeInit(void) {
  if (isInit) {
    return;
  }

  isInit = true;
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  // Control frequency
  if (!RATE_DO_EXECUTE(RATE_50_HZ, tick)) {
    return;
  }
  // Positon error, [m]
  x_error[0] = state->position.x - 1*setpoint->position.x;
  x_error[1] = state->position.y - 1*setpoint->position.y;
  x_error[2] = state->position.z - setpoint->position.z;

  // Body velocity error, [m/s]                          
  x_error[6] = state->velocity.x - 0*setpoint->velocity.x;
  x_error[7] = state->velocity.y - 0*setpoint->velocity.y;
  x_error[8] = state->velocity.z - 0*setpoint->velocity.z;

  // Angular rate error, [rad/s]
  x_error[9]  = radians(sensors->gyro.x - 0*setpoint->attitudeRate.roll);   
  x_error[10] = radians(sensors->gyro.y - 0*setpoint->attitudeRate.pitch);
  x_error[11] = radians(sensors->gyro.z - 0*setpoint->attitudeRate.yaw);

  struct quat attitude_g = qeye();  // goal attitude
  struct quat attitude = mkquat(
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
    state->attitudeQuaternion.w);  // current attitude

  struct quat attitude_gI = qinv(attitude_g);  
  struct quat q_error = qnormalize(qqmul(attitude_gI, attitude));
  struct vec phi = quat2rp(q_error);  // quaternion to Rodriquez parameters
  
  // Attitude error
  x_error[3] = phi.x;
  x_error[4] = phi.y;
  x_error[5] = phi.z;

  // Matrix multiplication, compute control input
  for (int i = 0; i < STABILIZER_NR_OF_MOTORS; i++){
    control_input[i] = 0;

    for (int j = 0; j < NXt; j++) {
      control_input[i] += -K[i][j] * x_error[j];
    }
  }

  // Compensate for gravity 
  control_input[0] += setpoint->thrust + CF_MASS * GRAVITY_MAGNITUDE;  

  if (setpoint->mode.z == modeDisable) {
    control->thrustSi = 0.0f;
    control->torque[0] =  0.0f;
    control->torque[1] =  0.0f;
    control->torque[2] =  0.0f;
  } else {
    control->thrustSi = control_input[0];  // [N]
    control->torqueX = control_input[1];   // [N.m]
    control->torqueY = control_input[2];
    control->torqueZ = control_input[3];
  }

  control->controlMode = controlModeForceTorque;
}
