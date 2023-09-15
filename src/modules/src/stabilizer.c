/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
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
 */
#define DEBUG_MODULE "STAB"
#define STOCK_TRACK 1

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "pm.h"
#include "platform.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "crtp_localization_service.h"
#include "controller.h"
#include "power_distribution.h"
#include "collision_avoidance.h"
#include "health.h"
#include "supervisor.h"

#include "estimator.h"
#include "usddeck.h"
#include "quatcompress.h"
#include "statsCnt.h"
#include "static_mem.h"
#include "rateSupervisor.h"

// #include "eventtrigger.h"


// EVENTTRIGGER(traj_ref, int8, traj_en, int32, traj_step, float, x, float, y, float, z);
// EVENTTRIGGER(traj_pos, float, x, float, y, float, z);
// EVENTTRIGGER(traj_vel, float, x, float, y, float, z);
// EVENTTRIGGER(traj_att, float, x, float, y, float, z);
// EVENTTRIGGER(traj_rate, float, x, float, y, float, z);
// EVENTTRIGGER(control_event, int32, u1, int32, u2, int32, u3, int32, u4);

static bool isInit;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

static uint32_t inToOutLatency;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;

static motors_thrust_uncapped_t motorThrustUncapped;
static motors_thrust_uncapped_t motorThrustBatCompUncapped;
static motors_thrust_pwm_t motorPwm;

// For scratch storage - never logged or passed to other subsystems.
static setpoint_t tempSetpoint;

static StateEstimatorType estimatorType;
static ControllerType controllerType;

static STATS_CNT_RATE_DEFINE(stabilizerRate, 500);
static rateSupervisor_t rateSupervisorContext;
static bool rateWarningDisplayed = false;

static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
  // compressed quaternion, see quatcompress.h
  int32_t quat;
  // angular velocity - milliradians / sec
  int16_t rateRoll;
  int16_t ratePitch;
  int16_t rateYaw;
} stateCompressed;

static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
} setpointCompressed;

#ifdef STOCK_TRACK
// FIXME 
// START OF MY TRAJECTORY
#include "traj_fig8_12.h"
static uint32_t step = 0;
static int8_t en_traj = 0;
static int8_t en_vel_ref = 0;
static uint32_t traj_length = 1405;
static int8_t user_traj_iter = 1;  // number of times to execute full trajectory
static int8_t traj_hold = 3;       // hold current trajectory for this no of steps
static int8_t traj_iter = 0;
static uint32_t traj_idx = 0;
static setpoint_t setpoint_traj;

void updateHorizonReference(const setpoint_t *setpoint) {
  // Update reference: from stored trajectory or commander
  // DEBUG_PRINT("%d %d\n", en_traj, step);
  if (en_traj == 1) {
    if (step % traj_hold == 0) {
      traj_idx = (int)(step / traj_hold);
      {
        setpoint_traj.position.x = X_ref_data[traj_idx][0];
        setpoint_traj.position.y = X_ref_data[traj_idx][1];
        setpoint_traj.position.z = X_ref_data[traj_idx][2];
        if (en_vel_ref == 1) {
          setpoint_traj.velocity.x = X_ref_data[traj_idx][6];
          setpoint_traj.velocity.y = X_ref_data[traj_idx][7];
          setpoint_traj.velocity.z = X_ref_data[traj_idx][8];
        }
        // setpoint_traj.attitude.roll = degrees(X_ref_data[traj_idx][];)
        setpoint_traj.mode.x = modeAbs;
        setpoint_traj.mode.z == modeAbs;
        setpoint_traj.mode.yaw == modeAbs;

      }
    }
  }
  else {
    setpoint_traj = *setpoint;
  }
  // DEBUG_PRINT("z_ref = %.2f\n", (double)(Xref[0](2)));

  // stop trajectory executation
  if (en_traj == 1) {
    if (traj_iter >= user_traj_iter) {
      en_traj = 0;
      DEBUG_PRINT("stop\n");
    }

    if (traj_idx >= traj_length - 1 - 10 + 1) { 
      // complete one trajectory
      step = 0; 
      traj_iter += 1;
    } 
    else step += 1;
  }
}
// END OF MY TRAJECTORY


// void usdLog() {
//   // eventTrigger_traj_ref_payload.traj_en = en_traj;
//   // eventTrigger_traj_ref_payload.traj_step = step;
//   // eventTrigger_traj_ref_payload.x = setpoint_traj.position.x;
//   // eventTrigger_traj_ref_payload.y = setpoint_traj.position.y;
//   // eventTrigger_traj_ref_payload.z = setpoint_traj.position.z;
//   // eventTrigger_traj_pos_payload.x = state.position.x;
//   // eventTrigger_traj_pos_payload.y = state.position.y;
//   // eventTrigger_traj_pos_payload.z = state.position.z;
//   // eventTrigger_traj_att_payload.x = radians(state.attitude.roll);
//   // eventTrigger_traj_att_payload.y = radians(-state.attitude.pitch);
//   // eventTrigger_traj_att_payload.z = radians(state.attitude.yaw);
//   // eventTrigger_traj_vel_payload.x = state.velocity.x;
//   // eventTrigger_traj_vel_payload.y = state.velocity.y;
//   // eventTrigger_traj_vel_payload.z = state.velocity.z;
//   // eventTrigger_traj_rate_payload.x = radians(sensorData.gyro.x);
//   // eventTrigger_traj_rate_payload.y = radians(sensorData.gyro.x);
//   // eventTrigger_traj_rate_payload.z = radians(sensorData.gyro.z);
//   // eventTrigger_control_event_payload.u1 = motorThrustUncapped.list[0];
//   // eventTrigger_control_event_payload.u2 = motorThrustUncapped.list[1];
//   // eventTrigger_control_event_payload.u3 = motorThrustUncapped.list[2];
//   // eventTrigger_control_event_payload.u4 = motorThrustUncapped.list[3];

//   // eventTrigger(&eventTrigger_traj_ref);
//   // eventTrigger(&eventTrigger_traj_pos);
//   // eventTrigger(&eventTrigger_traj_att);
//   // eventTrigger(&eventTrigger_traj_vel);
//   // eventTrigger(&eventTrigger_traj_rate);
//   // eventTrigger(&eventTrigger_control_event);
// }
#endif

STATIC_MEM_TASK_ALLOC(stabilizerTask, STABILIZER_TASK_STACKSIZE);

static void stabilizerTask(void* param);

static void calcSensorToOutputLatency(const sensorData_t *sensorData)
{
  uint64_t outTimestamp = usecTimestamp();
  inToOutLatency = outTimestamp - sensorData->interruptTimestamp;
}

static void compressState()
{
  stateCompressed.x = state.position.x * 1000.0f;
  stateCompressed.y = state.position.y * 1000.0f;
  stateCompressed.z = state.position.z * 1000.0f;

  stateCompressed.vx = state.velocity.x * 1000.0f;
  stateCompressed.vy = state.velocity.y * 1000.0f;
  stateCompressed.vz = state.velocity.z * 1000.0f;

  stateCompressed.ax = state.acc.x * 9.81f * 1000.0f;
  stateCompressed.ay = state.acc.y * 9.81f * 1000.0f;
  stateCompressed.az = (state.acc.z + 1) * 9.81f * 1000.0f;

  float const q[4] = {
    state.attitudeQuaternion.x,
    state.attitudeQuaternion.y,
    state.attitudeQuaternion.z,
    state.attitudeQuaternion.w};
  stateCompressed.quat = quatcompress(q);

  float const deg2millirad = ((float)M_PI * 1000.0f) / 180.0f;
  stateCompressed.rateRoll = sensorData.gyro.x * deg2millirad;
  stateCompressed.ratePitch = -sensorData.gyro.y * deg2millirad;
  stateCompressed.rateYaw = sensorData.gyro.z * deg2millirad;
}

static void compressSetpoint()
{
  setpointCompressed.x = setpoint.position.x * 1000.0f;
  setpointCompressed.y = setpoint.position.y * 1000.0f;
  setpointCompressed.z = setpoint.position.z * 1000.0f;

  setpointCompressed.vx = setpoint.velocity.x * 1000.0f;
  setpointCompressed.vy = setpoint.velocity.y * 1000.0f;
  setpointCompressed.vz = setpoint.velocity.z * 1000.0f;

  setpointCompressed.ax = setpoint.acceleration.x * 1000.0f;
  setpointCompressed.ay = setpoint.acceleration.y * 1000.0f;
  setpointCompressed.az = setpoint.acceleration.z * 1000.0f;
}

void stabilizerInit(StateEstimatorType estimator)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit(estimator);
  controllerInit(ControllerTypeAutoSelect);
  powerDistributionInit();
  motorsInit(platformConfigGetMotorMapping());
  // collisionAvoidanceInit();
  estimatorType = stateEstimatorGetType();
  controllerType = controllerGetType();

  STATIC_MEM_TASK_CREATE(stabilizerTask, stabilizerTask, STABILIZER_TASK_NAME, NULL, STABILIZER_TASK_PRI);

  isInit = true;

  // FIXME
#ifdef STOCK_TRACK
  traj_length = traj_length * traj_hold;
  en_traj = 0;
  en_vel_ref = 0;
#endif
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= controllerTest();
  pass &= powerDistributionTest();
  pass &= motorsTest();
  // pass &= collisionAvoidanceTest();

  return pass;
}

static void checkEmergencyStopTimeout()
{
  if (emergencyStopTimeout >= 0) {
    emergencyStopTimeout -= 1;

    if (emergencyStopTimeout == 0) {
      emergencyStop = true;
    }
  }
}

static void batteryCompensation(const motors_thrust_uncapped_t* motorThrustUncapped, motors_thrust_uncapped_t* motorThrustBatCompUncapped)
{
  float supplyVoltage = pmGetBatteryVoltage();

  for (int motor = 0; motor < STABILIZER_NR_OF_MOTORS; motor++)
  {
    motorThrustBatCompUncapped->list[motor] = motorsCompensateBatteryVoltage(motor, motorThrustUncapped->list[motor], supplyVoltage);
  }
}

static void setMotorRatios(const motors_thrust_pwm_t* motorPwm)
{
  motorsSetRatio(MOTOR_M1, motorPwm->motors.m1);
  motorsSetRatio(MOTOR_M2, motorPwm->motors.m2);
  motorsSetRatio(MOTOR_M3, motorPwm->motors.m3);
  motorsSetRatio(MOTOR_M4, motorPwm->motors.m4);
}

/* The stabilizer loop runs at 1kHz. It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */
static void stabilizerTask(void* param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  DEBUG_PRINT("Wait for sensor calibration...\n");

  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  // Initialize tick to something else then 0
  tick = 1;

  rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), M2T(1000), 997, 1003, 1);

  DEBUG_PRINT("Ready to fly.\n");

  // uint32_t prevStabilizerMs = usecTimestamp();

  while(1) {
    // The sensor should unlock at 1kHz    
    sensorsWaitDataReady();
    sensorsWaitDataReady();
    tick++;

    // DEBUG_PRINT("M: %d\n", usecTimestamp() - prevStabilizerMs);
    // prevStabilizerMs = usecTimestamp();

    // update sensorData struct (for logging variables)
    sensorsAcquire(&sensorData, tick);

    if (healthShallWeRunTest()) {
      healthRunTests(&sensorData);
    } else {
      // allow to update estimator dynamically
      if (stateEstimatorGetType() != estimatorType) {
        stateEstimatorSwitchTo(estimatorType);
        estimatorType = stateEstimatorGetType();
      }
      // allow to update controller dynamically
      if (controllerGetType() != controllerType) {
        controllerInit(controllerType);
        controllerType = controllerGetType();
      }

      stateEstimator(&state, tick);
      // compressState(); 

      if (crtpCommanderHighLevelGetSetpoint(&tempSetpoint, &state, tick)) {
        commanderSetSetpoint(&tempSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
      }

      commanderGetSetpoint(&setpoint, &state);
      // compressSetpoint();

      // FIXME 
#ifdef STOCK_TRACK 
      updateHorizonReference(&setpoint);  // call every 500hz just like mpc
      controller(&control, &setpoint_traj, &sensorData, &state, tick);
#else
      controller(&control, &setpoint, &sensorData, &state, tick);
#endif

      // collisionAvoidanceUpdateSetpoint(&setpoint, &sensorData, &state, tick);

      checkEmergencyStopTimeout();

      //
      // The supervisor module keeps track of Crazyflie state such as if
      // we are ok to fly, or if the Crazyflie is in flight.
      //
      supervisorUpdate(&sensorData);

      if (emergencyStop || (systemIsArmed() == false)) {
        motorsStop();
      } else {
        powerDistribution(&control, &motorThrustUncapped);
        batteryCompensation(&motorThrustUncapped, &motorThrustBatCompUncapped);
        powerDistributionCap(&motorThrustBatCompUncapped, &motorPwm);
        setMotorRatios(&motorPwm);
      }

#ifdef CONFIG_DECK_USD
      // Log data to uSD card if configured
      if (usddeckLoggingEnabled()
          && usddeckLoggingMode() == usddeckLoggingMode_SynchronousStabilizer
          && RATE_DO_EXECUTE(usddeckFrequency(), tick)) {
        usddeckTriggerLogging();
      }
#endif
      calcSensorToOutputLatency(&sensorData);
      tick++;
      STATS_CNT_RATE_EVENT(&stabilizerRate);

      if (!rateSupervisorValidate(&rateSupervisorContext, xTaskGetTickCount())) {
        if (!rateWarningDisplayed) {
          DEBUG_PRINT("WARNING: stabilizer loop rate is off (%lu)\n", rateSupervisorLatestCount(&rateSupervisorContext));
          rateWarningDisplayed = true;
        }
      }
    }
#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
    motorsBurstDshot();
#endif
  }

}

void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

/**
 * Parameters to set the estimator and controller type
 * for the stabilizer module, or to do an emergency stop
 */
PARAM_GROUP_START(stabilizer)
#ifdef STOCK_TRACK
PARAM_ADD_CORE(PARAM_INT8, en_traj, &en_traj)
PARAM_ADD_CORE(PARAM_INT8, en_vel_ref, &en_vel_ref)
#endif
/**
 * @brief Estimator type Auto select(0), complementary(1), extended kalman(2), **unscented kalman(3)  (Default: 0)
 *
 * ** Experimental, needs to be enabled in kbuild
 */
PARAM_ADD_CORE(PARAM_UINT8, estimator, &estimatorType)
/**
 * @brief Controller type Auto select(0), PID(1), Mellinger(2), INDI(3), Brescianini(4) (Default: 0)
 */
PARAM_ADD_CORE(PARAM_UINT8, controller, &controllerType)
/**
 * @brief If set to nonzero will turn off power
 */
PARAM_ADD_CORE(PARAM_UINT8, stop, &emergencyStop)
PARAM_GROUP_STOP(stabilizer)


/**
 * Log group for the current controller target
 *
 * Note: all members may not be updated depending on how the system is used
 */
LOG_GROUP_START(ctrltarget)

/**
 * @brief Desired position X [m]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &setpoint.position.x)

/**
 * @brief Desired position Y [m]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &setpoint.position.y)

/**
 * @brief Desired position X [m]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &setpoint.position.z)

/**
 * @brief Desired velocity X [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vx, &setpoint.velocity.x)

/**
 * @brief Desired velocity Y [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vy, &setpoint.velocity.y)

/**
 * @brief Desired velocity Z [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vz, &setpoint.velocity.z)

/**
 * @brief Desired acceleration X [m/s^2]
 */
LOG_ADD_CORE(LOG_FLOAT, ax, &setpoint.acceleration.x)

/**
 * @brief Desired acceleration Y [m/s^2]
 */
LOG_ADD_CORE(LOG_FLOAT, ay, &setpoint.acceleration.y)

/**
 * @brief Desired acceleration Z [m/s^2]
 */
LOG_ADD_CORE(LOG_FLOAT, az, &setpoint.acceleration.z)

/**
 * @brief Desired attitude, roll [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, roll, &setpoint.attitude.roll)

/**
 * @brief Desired attitude, pitch [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, pitch, &setpoint.attitude.pitch)

/**
 * @brief Desired attitude rate, yaw rate [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, yaw, &setpoint.attitude.yaw)  // CHANGED from yaw rate

/**
 * @brief Desired attitude quaternion
 */
LOG_ADD_CORE(LOG_FLOAT, qx, &setpoint.attitudeQuaternion.x)
LOG_ADD_CORE(LOG_FLOAT, qy, &setpoint.attitudeQuaternion.y)
LOG_ADD_CORE(LOG_FLOAT, qz, &setpoint.attitudeQuaternion.z)
LOG_ADD_CORE(LOG_FLOAT, qw, &setpoint.attitudeQuaternion.w)
LOG_GROUP_STOP(ctrltarget)

/**
 * Log group for the current controller target, compressed format.
 * This flavour of the controller target logs are defined with types
 * that use less space and makes it possible to add more logs to a
 * log configuration.
 *
 * Note: all members may not be updated depending on how the system is used
 */

LOG_GROUP_START(ctrltargetZ)
/**
 * @brief Desired position X [mm]
 */
LOG_ADD(LOG_INT16, x, &setpointCompressed.x)

/**
 * @brief Desired position Y [mm]
 */
LOG_ADD(LOG_INT16, y, &setpointCompressed.y)

/**
 * @brief Desired position Z [mm]
 */
LOG_ADD(LOG_INT16, z, &setpointCompressed.z)

/**
 * @brief Desired velocity X [mm/s]
 */
LOG_ADD(LOG_INT16, vx, &setpointCompressed.vx)

/**
 * @brief Desired velocity Y [mm/s]
 */
LOG_ADD(LOG_INT16, vy, &setpointCompressed.vy)

/**
 * @brief Desired velocity Z [mm/s]
 */
LOG_ADD(LOG_INT16, vz, &setpointCompressed.vz)

/**
 * @brief Desired acceleration X [mm/s^2]
 */
LOG_ADD(LOG_INT16, ax, &setpointCompressed.ax)

/**
 * @brief Desired acceleration Y [mm/s^2]
 */
LOG_ADD(LOG_INT16, ay, &setpointCompressed.ay)

/**
 * @brief Desired acceleration Z [mm/s^2]
 */
LOG_ADD(LOG_INT16, az, &setpointCompressed.az)
LOG_GROUP_STOP(ctrltargetZ)

/**
 * Logs to set the estimator and controller type
 * for the stabilizer module
 */
LOG_GROUP_START(stabilizer)
/**
 * @brief Estimated roll
 *   Note: Same as stateEstimate.roll
 */
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
/**
 * @brief Estimated pitch
 *   Note: Same as stateEstimate.pitch
 */
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
/**
 * @brief Estimated yaw
 *   Note: same as stateEstimate.yaw
 */
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
/**
 * @brief Current thrust
 */
LOG_ADD(LOG_FLOAT, thrust, &control.thrust)
/**
 * @brief Rate of stabilizer loop
 */
STATS_CNT_RATE_LOG_ADD(rtStab, &stabilizerRate)
/**
 * @brief Latency from sampling of sensor to motor output
 *    Note: Used for debugging but could also be used as a system test
 */ 
LOG_ADD(LOG_UINT32, intToOut, &inToOutLatency)
LOG_GROUP_STOP(stabilizer)

/**
 * Log group for accelerometer sensor measurement, based on body frame.
 * Compensated for a miss-alignment by gravity at startup.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what accelerometer sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
LOG_GROUP_START(acc)

/**
 * @brief Acceleration in X [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.acc.x)

/**
 * @brief Acceleration in Y [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.acc.y)

/**
 * @brief Acceleration in Z [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

/**
 * Log group for the barometer.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what barometer sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
LOG_GROUP_START(baro)

/**
 * @brief Altitude above Sea Level [m]
 */
LOG_ADD_CORE(LOG_FLOAT, asl, &sensorData.baro.asl)

/**
 * @brief Temperature [degrees Celsius]
 */
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)

/**
 * @brief Air preassure [mbar]
 */
LOG_ADD_CORE(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

/**
 * Log group for gyroscopes.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what gyroscope sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
LOG_GROUP_START(gyro)

/**
 * @brief Angular velocity (rotation) around the X-axis, after filtering [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.gyro.x)

/**
 * @brief Angular velocity (rotation) around the Y-axis, after filtering [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.gyro.y)

/**
 * @brief Angular velocity (rotation) around the Z-axis, after filtering [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif

/**
 * Log group for magnetometer.
 *
 * Currently only present on Crazyflie 2.0
 */
LOG_GROUP_START(mag)
/**
 * @brief Magnetometer X axis, after filtering [gauss]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.mag.x)
/**
 * @brief Magnetometer Y axis, after filtering [gauss]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.mag.y)
/**
 * @brief Magnetometer Z axis, after filtering [gauss]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)

/**
 * Log group for the state estimator, the currently estimated state of the platform.
 *
 * Note: all values may not be updated depending on which estimator that is used.
 */
LOG_GROUP_START(stateEstimate)

/**
 * @brief The estimated position of the platform in the global reference frame, X [m]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &state.position.x)

/**
 * @brief The estimated position of the platform in the global reference frame, Y [m]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &state.position.y)

/**
 * @brief The estimated position of the platform in the global reference frame, Z [m]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &state.position.z)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, X [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vx, &state.velocity.x)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, Y [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vy, &state.velocity.y)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, Z [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vz, &state.velocity.z)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, X [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, ax, &state.acc.x)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, Y [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, ay, &state.acc.y)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, without considering gravity, Z [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, az, &state.acc.z)

/**
 * @brief Attitude, roll angle [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, roll, &state.attitude.roll)

/**
 * @brief Attitude, pitch angle (legacy CF2 body coordinate system, where pitch is inverted) [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, pitch, &state.attitude.pitch)

/**
 * @brief Attitude, yaw angle [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, yaw, &state.attitude.yaw)

/**
 * @brief Attitude as a quaternion, x
 */
LOG_ADD_CORE(LOG_FLOAT, qx, &state.attitudeQuaternion.x)

/**
 * @brief Attitude as a quaternion, y
 */
LOG_ADD_CORE(LOG_FLOAT, qy, &state.attitudeQuaternion.y)

/**
 * @brief Attitude as a quaternion, z
 */
LOG_ADD_CORE(LOG_FLOAT, qz, &state.attitudeQuaternion.z)

/**
 * @brief Attitude as a quaternion, w
 */
LOG_ADD_CORE(LOG_FLOAT, qw, &state.attitudeQuaternion.w)
LOG_GROUP_STOP(stateEstimate)

/**
 * Log group for the state estimator, compressed format. This flavour of the
 * estimator logs are defined with types that use less space and makes it possible to
 * add more logs to a log configuration.
 *
 * Note: all values may not be updated depending on which estimator that is used.
 */
LOG_GROUP_START(stateEstimateZ)

/**
 * @brief The position of the Crazyflie in the global reference frame, X [mm]
 */
LOG_ADD(LOG_INT16, x, &stateCompressed.x)

/**
 * @brief The position of the Crazyflie in the global reference frame, Y [mm]
 */
LOG_ADD(LOG_INT16, y, &stateCompressed.y)

/**
 * @brief The position of the Crazyflie in the global reference frame, Z [mm]
 */
LOG_ADD(LOG_INT16, z, &stateCompressed.z)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, X [mm/s]
 */
LOG_ADD(LOG_INT16, vx, &stateCompressed.vx)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, Y [mm/s]
 */
LOG_ADD(LOG_INT16, vy, &stateCompressed.vy)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, Z [mm/s]
 */
LOG_ADD(LOG_INT16, vz, &stateCompressed.vz)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, X [mm/s]
 */
LOG_ADD(LOG_INT16, ax, &stateCompressed.ax)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, Y [mm/s]
 */
LOG_ADD(LOG_INT16, ay, &stateCompressed.ay)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, including gravity, Z [mm/s]
 */
LOG_ADD(LOG_INT16, az, &stateCompressed.az)

/**
 * @brief Attitude as a compressed quaternion, see see quatcompress.h for details
 */
LOG_ADD(LOG_UINT32, quat, &stateCompressed.quat)

/**
 * @brief Roll rate (angular velocity) [milliradians / sec]
 */
LOG_ADD(LOG_INT16, rateRoll, &stateCompressed.rateRoll)

/**
 * @brief Pitch rate (angular velocity) [milliradians / sec]
 */
LOG_ADD(LOG_INT16, ratePitch, &stateCompressed.ratePitch)

/**
 * @brief Yaw rate (angular velocity) [milliradians / sec]
 */
LOG_ADD(LOG_INT16, rateYaw, &stateCompressed.rateYaw)
LOG_GROUP_STOP(stateEstimateZ)


LOG_GROUP_START(motor)

/**
 * @brief Requested motor power for m1, including battery compensation. Same scale as the motor PWM but uncapped
 * and may have values outside the [0 - UINT16_MAX] range.
 */
LOG_ADD(LOG_INT32, m1req, &motorThrustBatCompUncapped.motors.m1)

/**
 * @brief Requested motor power for m1, including battery compensation. Same scale as the motor PWM but uncapped
 * and may have values outside the [0 - UINT16_MAX] range.
 */
LOG_ADD(LOG_INT32, m2req, &motorThrustBatCompUncapped.motors.m2)

/**
 * @brief Requested motor power for m1, including battery compensation. Same scale as the motor PWM but uncapped
 * and may have values outside the [0 - UINT16_MAX] range.
 */
LOG_ADD(LOG_INT32, m3req, &motorThrustBatCompUncapped.motors.m3)

/**
 * @brief Requested motor power for m1, including battery compensation. Same scale as the motor PWM but uncapped
 * and may have values outside the [0 - UINT16_MAX] range.
 */
LOG_ADD(LOG_INT32, m4req, &motorThrustBatCompUncapped.motors.m4)
LOG_GROUP_STOP(motor)

// #ifdef STOCK_TRACK
LOG_GROUP_START(motorUncapped)
LOG_ADD(LOG_INT32, m1, &motorThrustUncapped.motors.m1)
LOG_ADD(LOG_INT32, m2, &motorThrustUncapped.motors.m2)
LOG_ADD(LOG_INT32, m3, &motorThrustUncapped.motors.m3)
LOG_ADD(LOG_INT32, m4, &motorThrustUncapped.motors.m4)
LOG_GROUP_STOP(motor)
// #endif

#ifdef STOCK_TRACK
LOG_GROUP_START(trajRef)
LOG_ADD(LOG_FLOAT, x, &setpoint_traj.position.x)
LOG_ADD(LOG_FLOAT, y, &setpoint_traj.position.y)
LOG_ADD(LOG_FLOAT, z, &setpoint_traj.position.z)
LOG_ADD(LOG_INT8, traj_en, &en_traj)
LOG_ADD(LOG_UINT32, traj_step, &step)
LOG_GROUP_STOP(trajRef)
#endif
