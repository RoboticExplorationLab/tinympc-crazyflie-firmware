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
#define NU  4   // no. control input          [pwm1, pwm2, pwm3, pwm4] from [0..1]
#define LQR_RATE RATE_500_HZ  // control frequency

// 100HZ
// static float K[NU][NXt] = {
//   {-0.185864f,0.180278f,0.461060f,-0.636400f,-0.701818f,-0.707242f,-0.119654f,0.113294f,0.223558f,-0.049461f,-0.060174f,-0.269965f},
//   {0.183541f,0.166895f,0.461060f,-0.528924f,0.682786f,0.722112f,0.117279f,0.101126f,0.223558f,-0.033619f,0.058320f,0.277318f},
//   {0.159443f,-0.169782f,0.461060f,0.549587f,0.484211f,-0.758550f,0.095884f,-0.103829f,0.223558f,0.035573f,0.025417f,-0.295330f},
//   {-0.157121f,-0.177391f,0.461060f,0.615736f,-0.465178f,0.743680f,-0.093509f,-0.110590f,0.223558f,0.047506f,-0.023563f,0.287977f},
// };

// 50HZ
// static float K[NU][NXt] = {
//   {-0.103700f,0.099567f,0.561104f,-0.457236f,-0.518552f,-0.422322f,-0.082672f,0.077223f,0.247898f,-0.037991f,-0.048517f,-0.268282f},
//   {0.102132f,0.088876f,0.561104f,-0.353546f,0.502257f,0.437605f,0.080828f,0.066049f,0.247898f,-0.022275f,0.046847f,0.275589f},
//   {0.081775f,-0.090862f,0.561104f,0.371276f,0.303009f,-0.474485f,0.059959f,-0.068186f,0.247898f,0.024029f,0.013571f,-0.293212f},
//   {-0.080207f,-0.097581f,0.561104f,0.439507f,-0.286714f,0.459201f,-0.058116f,-0.075086f,0.247898f,0.036236f,-0.011901f,0.285906f},
// };

static float K[NU][NXt] = {
  {-0.219651f,0.215086f,0.214454f,-0.621733f,-0.673423f,-0.712498f,-0.126429f,0.121002f,0.126804f,-0.046103f,-0.055218f,-0.234805f},
  {0.219136f,0.206687f,0.214454f,-0.543327f,0.661569f,0.729705f,0.125272f,0.112190f,0.126804f,-0.033172f,0.053660f,0.231969f},
  {0.200889f,-0.207553f,0.214454f,0.556106f,0.509516f,-0.771945f,0.107802f,-0.113537f,0.126804f,0.034784f,0.026928f,-0.224824f},
  {-0.200374f,-0.214220f,0.214454f,0.608954f,-0.497662f,0.754737f,-0.106646f,-0.119655f,0.126804f,0.044490f,-0.025370f,0.227660f},
};

// static float K[NU][NXt] = {
//   {-0.209415f,0.190469f,0.471501f,-0.643639f,-0.716745f,-0.335579f,-0.130498f,0.118040f,0.224146f,-0.046397f,-0.052328f,-0.182753f},
//   {0.207305f,0.160584f,0.471501f,-0.527361f,0.705213f,0.368519f,0.128825f,0.098303f,0.224146f,-0.036927f,0.051228f,0.199755f},
//   {0.138321f,-0.163029f,0.471501f,0.539924f,0.449777f,-0.451224f,0.084350f,-0.100176f,0.224146f,0.038094f,0.031069f,-0.242445f},
//   {-0.136211f,-0.188024f,0.471501f,0.631076f,-0.438245f,0.418285f,-0.082677f,-0.116168f,0.224146f,0.045230f,-0.029969f,0.225443f},
// };

static float x_error[NXt];  
static float control_input[NU];
static float u_hover = 0.67f;  // ~ mass/max_thrust

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
  if (!RATE_DO_EXECUTE(LQR_RATE, tick)) {
    return;
  }
  // Positon error, [m]
  x_error[0] = state->position.x - 1*setpoint->position.x;
  x_error[1] = state->position.y - 1*setpoint->position.y;
  x_error[2] = state->position.z - 1*setpoint->position.z;

  // Frame velocity error, [m/s]                          
  x_error[6] = state->velocity.x - 1*setpoint->velocity.x;
  x_error[7] = state->velocity.y - 1*setpoint->velocity.y;
  x_error[8] = state->velocity.z - 1*setpoint->velocity.z;

  // Angular rate error, [rad/s]
  x_error[9]  = radians(sensors->gyro.x - 1*setpoint->attitudeRate.roll);   
  x_error[10] = radians(sensors->gyro.y - 1*setpoint->attitudeRate.pitch);
  x_error[11] = radians(sensors->gyro.z - 1*setpoint->attitudeRate.yaw);

  // struct quat attitude_g = qeye();  // goal attitude
  // struct quat attitude_g = mkquat(setpoint->attitudeQuaternion.x, 
  //                                 setpoint->attitudeQuaternion.y, 
  //                                 setpoint->attitudeQuaternion.z, 
  //                                 setpoint->attitudeQuaternion.w);  // do not have quat cmd

  struct vec desired_rpy = mkvec(radians(setpoint->attitude.roll), 
                                 radians(setpoint->attitude.pitch), 
                                 radians(setpoint->attitude.yaw));
  struct quat attitude_g = rpy2quat(desired_rpy);

  struct quat attitude = mkquat(
    state->attitudeQuaternion.x,
    state->attitudeQuaternion.y,
    state->attitudeQuaternion.z,
    state->attitudeQuaternion.w);  // current attitude (right pitch)

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
      control_input[i] += -K[i][j] * x_error[j] * 1.0f;
    }
  }

  // Debug printing
  // DEBUG_PRINT("U = [%.2f, %.2f, %.2f, %.2f]\n", (double)(control_input[0]), (double)(control_input[1]), (double)(control_input[2]), (double)(control_input[3]));

  /* Output control */
  if (setpoint->mode.z == modeDisable) {
    control->normalizedForces[0] = 0.0f;
    control->normalizedForces[1] = 0.0f;
    control->normalizedForces[2] = 0.0f;
    control->normalizedForces[3] = 0.0f;
  } else {
    control->normalizedForces[0] = control_input[0] + u_hover;  // PWM 0..1
    control->normalizedForces[1] = control_input[1] + u_hover;
    control->normalizedForces[2] = control_input[2] + u_hover;
    control->normalizedForces[3] = control_input[3] + u_hover;
  } 

  // control->normalizedForces[0] = 0.0f;
  // control->normalizedForces[1] = 0.0f;
  // control->normalizedForces[2] = 0.0f;
  // control->normalizedForces[3] = 0.0f;

  control->controlMode = controlModePWM;
}

/**
 * Tunning variables for the full state quaternion LQR controller
 */
PARAM_GROUP_START(ctrlLQR)
/**
 * @brief K gain
 */
PARAM_ADD(PARAM_FLOAT, u_hover, &u_hover)

PARAM_GROUP_STOP(ctrlLQR)

/**
 * Logging variables for the command and reference signals for the
 * LQR controller
 */

LOG_GROUP_START(ctrlLQR)

LOG_ADD(LOG_FLOAT, e_x, &x_error[0])
LOG_ADD(LOG_FLOAT, e_y, &x_error[1])
LOG_ADD(LOG_FLOAT, e_z, &x_error[2])

LOG_ADD(LOG_FLOAT, e_roll,  &x_error[3])
LOG_ADD(LOG_FLOAT, e_pitch, &x_error[4])
LOG_ADD(LOG_FLOAT, e_yaw,   &x_error[5])

LOG_ADD(LOG_FLOAT, e_vx, &x_error[6])
LOG_ADD(LOG_FLOAT, e_vy, &x_error[7])
LOG_ADD(LOG_FLOAT, e_vz, &x_error[8])

LOG_ADD(LOG_FLOAT, e_vroll,  &x_error[9])
LOG_ADD(LOG_FLOAT, e_vpitch, &x_error[10])
LOG_ADD(LOG_FLOAT, e_vyaw,   &x_error[11])

LOG_GROUP_STOP(ctrlLQR)