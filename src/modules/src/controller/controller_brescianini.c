/**
 * Authored by Michael Hamer (http://www.mikehamer.info), November 2016.
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
 * ============================================================================
 *
 * The controller implemented in this file is based on the paper:
 *
 * "Nonlinear Quadrocopter Attitude Control"
 * http://e-collection.library.ethz.ch/eserv/eth:7387/eth-7387-01.pdf
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @ARTICLE{BrescianiniNonlinearController2013,
               title={Nonlinear quadrocopter attitude control},
               author={Brescianini, Dario and Hehn, Markus and D'Andrea, Raffaello},
               year={2013},
               publisher={ETH Zurich}}
 *
 * ============================================================================
 */


#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "sensors.h"
#include "static_mem.h"
#include "string.h"

#include "controller_brescianini.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "num.h"
#include "math3d.h"
#include "physicalConstants.h"

// #include "rateSupervisor.h"

#define DEBUG_MODULE "BRESCIANINI"
#include "debug.h"
#include "cfassert.h"


///// Copied from estimator_kalman.c

// Semaphore to signal that we got data from the stabilizer loop to process
static SemaphoreHandle_t runTaskSemaphore;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

#define ONE_SECOND 1000
#define UPDATE_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz

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


// Struct for logging position information
static bool isInit = false;


// Structs to keep track of data sent to and received by stabilizer loop

// Updates at 1khz
static control_t control_data;
static setpoint_t setpoint_data;
static sensorData_t sensors_data;
static state_t state_data;

// Updates at update_rate
static setpoint_t setpoint_task;
static sensorData_t sensors_task;
static state_t state_task;
static control_t control_task;


///// Modified from estimator_kalman.c

static void brescianiniControllerTask(void* parameters);
// static void updateQueuedMeasurements(const uint32_t nowMs, const bool quadIsFlying);

// STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(brescianiniTask, KALMAN_TASK_STACKSIZE);
STATIC_MEM_TASK_ALLOC(brescianiniTask, KALMAN_TASK_STACKSIZE);

/////


uint64_t startTimestamp;

void controllerBrescianiniInit(void) {
  // Created in the 'empty' state, meaning the semaphore must first be given, that is it will block in the task
  // until released by the stabilizer loop

  // DEBUG_PRINT("init brescianini\n");

  startTimestamp = usecTimestamp();
  // DEBUG_PRINT("init time: %d\n", startTimestamp);

  if (isInit) {
    return;
  }

  runTaskSemaphore = xSemaphoreCreateBinary();
  // DEBUG_PRINT("createBinary\n");
  ASSERT(runTaskSemaphore);
  // DEBUG_PRINT("runTaskSem\n");

  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);
  // DEBUG_PRINT("createDataMutex\n");

  STATIC_MEM_TASK_CREATE(brescianiniTask, brescianiniControllerTask, "BRESCIANINI_TASK", NULL, KALMAN_TASK_PRI);

  // DEBUG_PRINT("staticMemTaskCreate\n");

  isInit = true;
}


// #define UPDATE_RATE RATE_100_HZ

// This runs at every stabilizer step at 1000 Hz
void controllerBrescianini(control_t *control,
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

  // DEBUG_PRINT("control: %f %f %f %f\n", (double)control->thrustSi, (double)control->torqueX, (double)control->torqueY, (double)control->torqueZ);
  // DEBUG_PRINT("control_data: %f %f %f %f\n\n", (double)control_data.thrustSi, (double)control_data.torqueX, (double)control_data.torqueY, (double)control_data.torqueZ);

  // DEBUG_PRINT("setpoint: %f %f %f\n", (double)setpoint->position.x, (double)setpoint->position.y, (double)setpoint->position.z);
  // DEBUG_PRINT("setpoint_data: %f %f %f\n\n", (double)setpoint_data.position.x, (double)setpoint_data.position.y, (double)setpoint_data.position.z);

  xSemaphoreGive(runTaskSemaphore);
}

void brescianiniControllerTask(void *args) {
  systemWaitStart();

  // DEBUG_PRINT("taskStarted\n");
  // DEBUG_PRINT("task started time: %lld\n", usecTimestamp() - startTimestamp);

  uint32_t nowMs = T2M(xTaskGetTickCount());
  uint32_t nextPredictionMs = nowMs;


  while (1) {
    startTimestamp = usecTimestamp();
    // DEBUG_PRINT("while loop time: %lld\n", usecTimestamp() - startTimestamp);
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);
    nowMs = T2M(xTaskGetTickCount());
    
    // Copy data from global to local scope
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(&setpoint_task, &setpoint_data, sizeof(setpoint_t));
    memcpy(&sensors_task, &sensors_data, sizeof(sensorData_t));
    memcpy(&state_task, &state_data, sizeof(state_t));
    memcpy(&control_task, &control_data, sizeof(control_t));
    xSemaphoreGive(dataMutex);


    static float control_omega[3];
    static struct vec control_torque;
    static float control_thrust;

    // define this here, since we do body-rate control_task at 1000Hz below the following if statement
    float omega[3] = {0};
    omega[0] = radians(sensors_task.gyro.x);
    omega[1] = radians(sensors_task.gyro.y);
    omega[2] = radians(sensors_task.gyro.z);

    // if (RATE_DO_EXECUTE(UPDATE_RATE, tick)) {
    if (nowMs >= nextPredictionMs) {
      nextPredictionMs = nowMs + (1000.0f / UPDATE_RATE);

      // desired accelerations
      struct vec accDes = vzero();
      // desired thrust
      float collCmd = 0;

      // attitude error as computed by the reduced attitude controller
      struct quat attErrorReduced = qeye();

      // attitude error as computed by the full attitude controller
      struct quat attErrorFull = qeye();

      // desired attitude as computed by the full attitude controller
      struct quat attDesiredFull = qeye();

      // current attitude
      struct quat attitude = mkquat(
        state_task.attitudeQuaternion.x,
        state_task.attitudeQuaternion.y,
        state_task.attitudeQuaternion.z,
        state_task.attitudeQuaternion.w);

      // inverse of current attitude
      struct quat attitudeI = qinv(attitude);

      // body frame -> inertial frame :  vI = R * vB
      // float R[3][3] = {0};
      // struct quat q = attitude;
      // R[0][0] = q.w * q.w + q.x * q.x - q.y * q.y - q.z * q.z;
      // R[0][1] = 2 * q.x * q.y - 2 * q.w * q.z;
      // R[0][2] = 2 * q.x * q.z + 2 * q.w * q.y;

      // R[1][0] = 2 * q.x * q.y + 2 * q.w * q.z;
      // R[1][1] = q.w * q.w - q.x * q.x + q.y * q.y - q.z * q.z;
      // R[1][2] = 2 * q.y * q.z - 2 * q.w * q.x;

      // R[2][0] = 2 * q.x * q.z - 2 * q.w * q.y;
      // R[2][1] = 2 * q.y * q.z + 2 * q.w * q.x;
      // R[2][2] = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z;

      // We don't need all terms of R, only compute the necessary parts
      // projections of e3_B to Ixyz

      float R02 = 2 * attitude.x * attitude.z + 2 * attitude.w * attitude.y;
      float R12 = 2 * attitude.y * attitude.z - 2 * attitude.w * attitude.x;
      float R22 = attitude.w * attitude.w - attitude.x * attitude.x - attitude.y * attitude.y + attitude.z * attitude.z;

      // a few temporary quaternions
      struct quat temp1 = qeye();
      struct quat temp2 = qeye();

      // compute the position and (body) velocity errors
      struct vec pError = mkvec(setpoint_task.position.x - state_task.position.x,
                                setpoint_task.position.y - state_task.position.y,
                                setpoint_task.position.z - state_task.position.z);

      struct vec vError = mkvec(setpoint_task.velocity.x - state_task.velocity.x,
                                setpoint_task.velocity.y - state_task.velocity.y,
                                setpoint_task.velocity.z - state_task.velocity.z);


      // ====== LINEAR CONTROL ======

      // compute desired accelerations in X, Y and Z
      accDes.x = 0;
      accDes.x += 1.0f / tau_xy / tau_xy * pError.x;
      accDes.x += 2.0f * zeta_xy / tau_xy * vError.x;
      accDes.x += setpoint_task.acceleration.x;  // m/s^2
      accDes.x = constrain(accDes.x, -coll_max, coll_max);

      accDes.y = 0;
      accDes.y += 1.0f / tau_xy / tau_xy * pError.y;
      accDes.y += 2.0f * zeta_xy / tau_xy * vError.y;
      accDes.y += setpoint_task.acceleration.y;
      accDes.y = constrain(accDes.y, -coll_max, coll_max);

      accDes.z = GRAVITY_MAGNITUDE;  // compensate gravity
      accDes.z += 1.0f / tau_z / tau_z * pError.z;
      accDes.z += 2.0f * zeta_z / tau_z * vError.z;
      accDes.z += setpoint_task.acceleration.z;
      accDes.z = constrain(accDes.z, -coll_max, coll_max);


      // ====== THRUST CONTROL ======

      // compute commanded thrust required to achieve the z acceleration
      collCmd = accDes.z / R22;  // eq.(44)

      if (fabsf(collCmd) > coll_max) {
        // exceeding the thrust threshold
        // we compute a reduction factor r based on fairness f \in [0,1] such that:
        // collMax^2 = (r*x)^2 + (r*y)^2 + (r*f*z + (1-f)z + g)^2
        float x = accDes.x;
        float y = accDes.y;
        float z = accDes.z - GRAVITY_MAGNITUDE;
        float g = GRAVITY_MAGNITUDE;
        float f = constrain(thrust_reduction_fairness, 0, 1);

        float r = 0;

        // solve as a quadratic
        float a = powf(x, 2) + powf(y, 2) + powf(z*f, 2);
        if (a<0) { a = 0; }

        float b = 2 * z*f*((1-f)*z + g);
        float c = powf(coll_max, 2) - powf((1-f)*z + g, 2);
        if (c<0) { c = 0; }

        if (fabsf(a)<1e-6f) {
          r = 0;
        } else {
          float sqrtterm = powf(b, 2) + 4.0f*a*c;
          r = (-b + sqrtf(sqrtterm))/(2.0f*a);
          r = constrain(r,0,1);
        }
        accDes.x = r*x;
        accDes.y = r*y;
        accDes.z = (r*f+(1-f))*z + g;
      }
      collCmd = constrain(accDes.z / R22, coll_min, coll_max);

      // FYI: this thrust will result in the accelerations
      // xdd = R02*coll
      // ydd = R12*coll

      // a unit vector pointing in the direction of the desired thrust (ie. the direction of body's z axis in the inertial frame)
      struct vec zI_des = vnormalize(accDes);

      // a unit vector pointing in the direction of the current thrust (in I frame)
      struct vec zI_cur = vnormalize(mkvec(R02, R12, R22));

      // a unit vector pointing in the direction of the inertial frame z-axis
      struct vec zI = mkvec(0, 0, 1);



      // ====== REDUCED ATTITUDE CONTROL ======

      // compute the error angle between the current and the desired thrust directions
      float dotProd = vdot(zI_cur, zI_des);
      dotProd = constrain(dotProd, -1, 1);
      float alpha = acosf(dotProd);

      // the axis around which this rotation needs to occur in the inertial frame (ie. an axis orthogonal to the two)
      struct vec rotAxisI = vzero();
      if (fabsf(alpha) > 1 * ARCMINUTE) {
        rotAxisI = vnormalize(vcross(zI_cur, zI_des));
      } else {
        rotAxisI = mkvec(1, 1, 0);
      }

      // the attitude error quaternion
      attErrorReduced.w = cosf(alpha / 2.0f);
      attErrorReduced.x = sinf(alpha / 2.0f) * rotAxisI.x;
      attErrorReduced.y = sinf(alpha / 2.0f) * rotAxisI.y;
      attErrorReduced.z = sinf(alpha / 2.0f) * rotAxisI.z;

      // choose the shorter rotation
      if (sinf(alpha / 2.0f) < 0) {
        rotAxisI = vneg(rotAxisI);
      }
      if (cosf(alpha / 2.0f) < 0) {
        rotAxisI = vneg(rotAxisI);
        attErrorReduced = qneg(attErrorReduced);
      }

      attErrorReduced = qnormalize(attErrorReduced);


      // ====== FULL ATTITUDE CONTROL ======

      // compute the error angle between the inertial and the desired thrust directions 
      dotProd = vdot(zI, zI_des);
      dotProd = constrain(dotProd, -1, 1);
      alpha = acosf(dotProd);

      // the axis around which this rotation needs to occur in the inertial frame (ie. an axis orthogonal to the two)
      if (fabsf(alpha) > 1 * ARCMINUTE) {
        rotAxisI = vnormalize(vcross(zI, zI_des));
      } else {
        rotAxisI = mkvec(1, 1, 0);
      }

      // the quaternion corresponding to a roll and pitch around this axis
      struct quat attFullReqPitchRoll = mkquat(sinf(alpha / 2.0f) * rotAxisI.x,
                                              sinf(alpha / 2.0f) * rotAxisI.y,
                                              sinf(alpha / 2.0f) * rotAxisI.z,
                                              cosf(alpha / 2.0f));

      // the quaternion corresponding to a rotation to the desired yaw
      struct quat attFullReqYaw = mkquat(0, 0, sinf(radians(setpoint_task.attitude.yaw) / 2.0f), cosf(radians(setpoint_task.attitude.yaw) / 2.0f));

      // the full rotation (roll & pitch, then yaw)
      attDesiredFull = qqmul(attFullReqPitchRoll, attFullReqYaw);

      // back transform from the current attitude to get the error between rotations
      attErrorFull = qqmul(attitudeI, attDesiredFull);

      // correct rotation
      if (attErrorFull.w < 0) {
        attErrorFull = qneg(attErrorFull);
        attDesiredFull = qqmul(attitude, attErrorFull);
      }

      attErrorFull = qnormalize(attErrorFull);
      attDesiredFull = qnormalize(attDesiredFull);


      // ====== MIXING FULL & REDUCED CONTROL ======

      struct quat attError = qeye();

      if (mixing_factor <= 0) {
        // 100% reduced control_task (no yaw control_task)
        attError = attErrorReduced;
      } else if (mixing_factor >= 1) {
        // 100% full control_task (yaw controlled with same time constant as roll & pitch)
        attError = attErrorFull;
      } else {
        // mixture of reduced and full control_task

        // calculate rotation between the two errors
        temp1 = qinv(attErrorReduced);
        temp2 = qnormalize(qqmul(temp1, attErrorFull));

        // by defintion this rotation has the form [cos(alpha/2), 0, 0, sin(alpha/2)]
        // where the first element gives the rotation angle, and the last the direction
        alpha = 2.0f * acosf(constrain(temp2.w, -1, 1));

        // bisect the rotation from reduced to full control_task
        temp1 = mkquat(0,
                        0,
                        sinf(alpha * mixing_factor / 2.0f) * (temp2.z < 0 ? -1 : 1), // rotate in the correct direction
                        cosf(alpha * mixing_factor / 2.0f));

        attError = qnormalize(qqmul(attErrorReduced, temp1));
      }

      // ====== COMPUTE CONTROL SIGNALS ======

      // compute the commanded body rates
      control_omega[0] = 2.0f / tau_rp * attError.x;
      control_omega[1] = 2.0f / tau_rp * attError.y;
      control_omega[2] = 2.0f / tau_rp * attError.z + radians(setpoint_task.attitudeRate.yaw); // due to the mixing, this will behave with time constant tau_yaw

      // apply the rotation heuristic
      if (control_omega[0] * omega[0] < 0 && fabsf(omega[0]) > heuristic_rp) { // desired rotational rate in direction opposite to current rotational rate
        control_omega[0] = omega_rp_max * (omega[0] < 0 ? -1 : 1); // maximum rotational rate in direction of current rotation
      }

      if (control_omega[1] * omega[1] < 0 && fabsf(omega[1]) > heuristic_rp) { // desired rotational rate in direction opposite to current rotational rate
        control_omega[1] = omega_rp_max * (omega[1] < 0 ? -1 : 1); // maximum rotational rate in direction of current rotation
      }

      if (control_omega[2] * omega[2] < 0 && fabsf(omega[2]) > heuristic_yaw) { // desired rotational rate in direction opposite to current rotational rate
        control_omega[2] = omega_yaw_max * (omega[2] < 0 ? -1 : 1);
        // control_omega[2] = omega_rp_max * (omega[2] < 0 ? -1 : 1); // maximum rotational rate in direction of current rotation
      }

      // scale the commands to satisfy rate constraints
      float scaling = 1;
      scaling = fmax(scaling, fabsf(control_omega[0]) / omega_rp_max);
      scaling = fmax(scaling, fabsf(control_omega[1]) / omega_rp_max);
      scaling = fmax(scaling, fabsf(control_omega[2]) / omega_yaw_max);

      control_omega[0] /= scaling;
      control_omega[1] /= scaling;
      control_omega[2] /= scaling;  // rad/s
      control_thrust = collCmd;  // m/s^2
    }

    //// This is outside of UPDATE_RATE
    if (setpoint_task.mode.z == modeDisable) {
      // DEBUG_PRINT("setpoint_task.mode.z == modeDisable\n");
      control_task.thrustSi = 0.0f;
      control_task.torque[0] =  0.0f;
      control_task.torque[1] =  0.0f;
      control_task.torque[2] =  0.0f;
    } else {
      // DEBUG_PRINT("setpoint_task.mode.z != modeDisable\n");
      // control_task the body torques
      struct vec omegaErr = mkvec((control_omega[0] - omega[0])/tau_rp_rate,
                          (control_omega[1] - omega[1])/tau_rp_rate,
                          (control_omega[2] - omega[2])/tau_yaw_rate);

      // update the commanded body torques based on the current error in body rates
      control_torque = mvmul(CRAZYFLIE_INERTIA, omegaErr);

      // control_task.thrustSi = control_thrust * 0.033; // force to provide control_thrust
      control_task.thrustSi = control_thrust * CF_MASS; // force to provide control_thrust
      control_task.torqueX = control_torque.x;
      control_task.torqueY = control_torque.y;
      control_task.torqueZ = control_torque.z;

    }

    control_task.controlMode = controlModeForceTorque;

    // Copy the controls calculated locally by the task to the global control_data
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    memcpy(&control_data, &control_task, sizeof(control_t));
    xSemaphoreGive(dataMutex);

    // DEBUG_PRINT("setpoint->mode.z: %d\n", setpoint->mode.z);

  // DEBUG_PRINT("control: %f %f %f\n", (double)control->torqueX, (double)control->torqueY, (double)control->torqueZ);
  // DEBUG_PRINT("control_data: %f %f %f\n\n", (double)control_data.torqueX, (double)control_data.torqueY, (double)control_data.torqueZ);

  }
}

bool controllerBrescianiniTest(void) {
  return true;
}


PARAM_GROUP_START(ctrlAtt)
PARAM_ADD(PARAM_FLOAT, tau_xy, &tau_xy)
PARAM_ADD(PARAM_FLOAT, zeta_xy, &zeta_xy)
PARAM_ADD(PARAM_FLOAT, tau_z, &tau_z)
PARAM_ADD(PARAM_FLOAT, zeta_z, &zeta_z)
PARAM_ADD(PARAM_FLOAT, tau_rp, &tau_rp)
PARAM_ADD(PARAM_FLOAT, mixing_factor, &mixing_factor)
PARAM_ADD(PARAM_FLOAT, coll_fairness, &thrust_reduction_fairness)
// PARAM_ADD(PARAM_FLOAT, heuristic_rp, &heuristic_rp)
// PARAM_ADD(PARAM_FLOAT, heuristic_yaw, &heuristic_yaw)
// PARAM_ADD(PARAM_FLOAT, tau_rp_rate, &tau_rp_rate)
// PARAM_ADD(PARAM_FLOAT, tau_yaw_rate, &tau_yaw_rate)
// PARAM_ADD(PARAM_FLOAT, coll_min, &coll_min)
// PARAM_ADD(PARAM_FLOAT, coll_max, &coll_max)
// PARAM_ADD(PARAM_FLOAT, omega_rp_max, &omega_rp_max)
// PARAM_ADD(PARAM_FLOAT, omega_yaw_max, &omega_yaw_max)
PARAM_GROUP_STOP(ctrlAtt)
