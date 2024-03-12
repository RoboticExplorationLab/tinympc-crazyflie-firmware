#include "Eigen.h"

#ifdef __cplusplus
extern "C"
{
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
#include "eventtrigger.h"
#include "param.h"
#include "num.h"
#include "math3d.h"

#include "cpp_compat.h" // needed to compile Cpp to C

// TinyMPC and low level controllers
#include "tinympc/admm.hpp"
#include "controller_pid.h"
#include "controller_brescianini.h"
#include "controller_mellinger.h"

// Params
// #include "quadrotor_50hz_Q1e1.hpp"
#include "quadrotor_50hz_Q1e2_1e1.hpp"
// #include "quadrotor_20hz.hpp"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "MPC"
#include "debug.h"

// #define MPC_RATE RATE_250_HZ  // control frequency
// #define RATE_25_HZ 25
// #define MPC_RATE      RATE_50_HZ
#define MPC_RATE RATE_100_HZ
#define LOWLEVEL_RATE RATE_500_HZ
#define ALPHA_LPF 1.0 // how much to use current setpoint
#define TRACK_MODE 14 // 0 for position, 1 for velocity, 2 for acceleration, 3 for pos and vel, 4 for vel and acc, 5 for all, 14 for bres

  // Semaphore to signal that we got data from the stabilizer loop to process
  static SemaphoreHandle_t runTaskSemaphore;

  // Mutex to protect data that is shared between the task and
  // functions called by the stabilizer loop
  static SemaphoreHandle_t dataMutex;
  static StaticSemaphore_t dataMutexBuffer;

  static void tinympcControllerTask(void *parameters);

  STATIC_MEM_TASK_ALLOC(tinympcControllerTask, TINYMPC_TASK_STACKSIZE);

  // Structs to keep track of data sent to and received by stabilizer loop
  // Stabilizer loop updates/uses these
  static control_t control_data;
  static setpoint_t setpoint_data;
  static sensorData_t sensors_data;
  static state_t state_data;
  static setpoint_t setpoint_low_level;
  static Matrix<tinytype, NSTATES + NINPUTS, 1> mpc_setpoint;
  // Copies that stay constant for duration of MPC loop
  static setpoint_t setpoint_task;
  static sensorData_t sensors_task;
  static state_t state_task;
  static control_t control_task;
  static Matrix<tinytype, NSTATES + NINPUTS, 1> mpc_setpoint_task;
  static Matrix<tinytype, NSTATES + NINPUTS, 1> mpc_setpoint_prev;

  /* Allocate global variables for MPC */
  static TinyBounds bounds;
  static TinySocs socs;
  static TinyWorkspace work;
  static TinyCache cache;
  static TinySettings settings;
  static TinySolver solver{&settings, &cache, &work};

  static tiny_VectorNx x0;
  static tiny_VectorNx x1;
  static tiny_VectorNx x2;

  // Helper variables
  static int8_t result = 0;
  static uint32_t step = 0; // mpc steps taken
  static bool en_traj = true;
  static int8_t user_traj_iter = 1; // number of times to execute full trajectory
  static int8_t traj_hold = 3;      // hold current trajectory for this no of steps
  static int8_t traj_iter = 0;      // number of times trajectory has been executed
  static uint32_t traj_idx = 0;     // actual traj index in the trajectory
  static uint64_t startTimestamp;
  static uint32_t timestamp;
  static uint32_t mpc_start_timestamp;
  static uint32_t mpc_time_us;
  static struct vec phi; // For converting from the current state estimate's quaternion to Rodrigues parameters
  static bool isInit = false;

  // Global state variable used in the
  // firmware as the only instance and in bindings
  // to hold the default values
  static controllerMellinger_t g_self = {
      .mass = CF_MASS,
      .massThrust = 132000,

      // XY Position PID
      .kp_xy = 0.4,  // P
      .kd_xy = 0.2,  // D
      .ki_xy = 0.05, // I
      .i_range_xy = 2.0,

      // Z Position
      .kp_z = 1.25, // P
      .kd_z = 0.4,  // D
      .ki_z = 0.05, // I
      .i_range_z = 0.4,

      // Attitude
      .kR_xy = 70000, // P
      .kw_xy = 20000, // D
      .ki_m_xy = 0.0, // I
      .i_range_m_xy = 1.0,

      // Yaw
      .kR_z = 60000, // P
      .kw_z = 12000, // D
      .ki_m_z = 500, // I
      .i_range_m_z = 1500,

      // roll and pitch angular velocity
      .kd_omega_rp = 200, // D

      // Helper variables
      .i_error_x = 0,
      .i_error_y = 0,
      .i_error_z = 0,

      .i_error_m_x = 0,
      .i_error_m_y = 0,
      .i_error_m_z = 0,
  };

  void appMain()
  {
    DEBUG_PRINT("Waiting for activation ...\n");
    while (1)
    {
      vTaskDelay(M2T(10000)); // TODO: Can I make this really big?
    }
  }
  bool controllerOutOfTreeTest()
  {
    // Always return true
    return true;
  }

  void updateHorizonReference(const setpoint_t *setpoint)
  {
    traj_idx++;
    if (traj_idx <= NTOTAL)
    {
      work.Xref = x0.replicate(1, NHORIZON);
    }
    else if (traj_idx <= 2*NTOTAL)
    {
      work.Xref = x1.replicate(1, NHORIZON);
    }
    else if (traj_idx <= 3*NTOTAL)
    {
      work.Xref = x2.replicate(1, NHORIZON);
    }
    else 
    {
      traj_idx = 3*NTOTAL+1;
      work.Xref = x0.replicate(1, NHORIZON);
    }
  }

  void controllerOutOfTreeInit(void)
  {
    // controllerPidInit();
    controllerBrescianiniInit();
    // controllerMellingerFirmwareInit();

    work.bounds = &bounds;
    work.socs = &socs;

    /* Map data from problem_data (array in row-major order) */

    //////// Cache
    cache.rho = rho_value;
    cache.Kinf = Eigen::Map<const Matrix<tinytype, NINPUTS, NSTATES, Eigen::RowMajor>>(Kinf_data);
    cache.Pinf = Eigen::Map<const Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Pinf_data);
    cache.Quu_inv = Eigen::Map<const Matrix<tinytype, NINPUTS, NINPUTS, Eigen::RowMajor>>(Quu_inv_data);
    cache.AmBKt = Eigen::Map<const Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(AmBKt_data);
    cache.APf = Eigen::Map<const Matrix<tinytype, NSTATES, 1>>(APf_data);
    cache.BPf = Eigen::Map<const Matrix<tinytype, NINPUTS, 1>>(BPf_data);

    //////// Workspace (dynamics and LQR cost matrices)
    work.Adyn = Eigen::Map<const Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Adyn_data);
    work.Bdyn = Eigen::Map<const Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(Bdyn_data);
    work.fdyn = Eigen::Map<const Matrix<tinytype, NSTATES, 1>>(fdyn_data);
    work.Q = Eigen::Map<const tiny_VectorNx>(Q_data);
    work.R = Eigen::Map<const tiny_VectorNu>(R_data);

    //////// Box constraints
    tiny_VectorNu u_min_one_time_step(-18, -18, 0.0);
    tiny_VectorNu u_max_one_time_step(18, 18, 18.0);
    work.bounds->u_min = u_min_one_time_step.replicate(1, NHORIZON - 1);
    work.bounds->u_max = u_max_one_time_step.replicate(1, NHORIZON - 1);
    // tiny_VectorNx x_min_one_time_step(-5.0, -5.0, -0.5, -10.0, -10.0, -20.0);
    // tiny_VectorNx x_max_one_time_step(5.0, 5.0, 100.0, 10.0, 10.0, 20.0);
    // work.bounds->x_min = x_min_one_time_step.replicate(1, NHORIZON);
    // work.bounds->x_max = x_max_one_time_step.replicate(1, NHORIZON);

    //////// Second order cone constraints
    work.socs->cu[0] = 0.3; // coefficients for input cones (mu)
    // work.socs->cx[0] = 0.6; // coefficients for state cones (mu)
    // work.socs->Acu[0] = 0; // start indices for input cones
    // work.socs->Acx[0] = 0; // start indices for state cones
    work.socs->qcu[0] = 3; // dimensions for input cones
    // work.socs->qcx[0] = 3; // dimensions for state cones

    //////// Settings
    // settings.abs_pri_tol = 0.01;  // no termination
    // settings.abs_dua_tol = 0.01;
    settings.max_iter = 2;
    // settings.check_termination = 0;
    // settings.en_state_bound = 0;
    settings.en_input_bound = 1;
    // settings.en_state_soc = 0;
    settings.en_input_soc = 1;

    //////// Initialize other workspace values automatically
    // reset_problem(&solver);

    // Initial state
    x0 << 0, 0, 0.5, 0, 0, 0;
    x1 << 3, 0, 0.5, 0, 0, 0;
    x2 << 3, 3, 0.5, 0, 0, 0;

    // Uref stays constant, Xref interpolate between start and goal states
    for (int i = 0; i < NHORIZON - 1; i++)
    {
      work.Uref.col(i)(2) = GRAVITY_MAGNITUDE;
    }
    // updateHorizonReference(&setpoint_task);

    /* Begin task initialization */
    runTaskSemaphore = xSemaphoreCreateBinary();
    ASSERT(runTaskSemaphore);

    dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

    STATIC_MEM_TASK_CREATE(tinympcControllerTask, tinympcControllerTask, TINYMPC_TASK_NAME, NULL, TINYMPC_TASK_PRI);

    isInit = true;
    /* End of task initialization */
  }

  static void tinympcControllerTask(void *parameters)
  {
    // systemWaitStart();

    uint32_t nowMs = T2M(xTaskGetTickCount());
    uint32_t nextMpcMs = nowMs;

    startTimestamp = usecTimestamp();

    while (true)
    {
      xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);

      nowMs = T2M(xTaskGetTickCount());
      if (nowMs >= nextMpcMs)
      {
        nextMpcMs = nowMs + (uint32_t)(1000.0 / MPC_RATE);

        // Update task data with most recent stabilizer loop data
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        memcpy(&setpoint_task, &setpoint_data, sizeof(setpoint_t));
        memcpy(&sensors_task, &sensors_data, sizeof(sensorData_t));
        memcpy(&state_task, &state_data, sizeof(state_t));
        memcpy(&control_task, &control_data, sizeof(control_t));
        xSemaphoreGive(dataMutex);

        //// DO MPC HERE
        // 1. Update measurement
        work.x.col(0) << state_task.position.x, state_task.position.y, state_task.position.z, state_task.velocity.x, state_task.velocity.y, state_task.velocity.z;
        // work.x.col(0) = x0;

        // 2. Update reference
        updateHorizonReference(&setpoint_task);

        // 3. Reset dual variables if needed
        reset_dual(&solver);

        // 4. Solve MPC problem
        mpc_start_timestamp = usecTimestamp();
        tiny_solve(&solver);
        vTaskDelay(M2T(1));
        tiny_solve(&solver);
        vTaskDelay(M2T(1));
        tiny_solve(&solver);

        // 5. Extract control from solution (state + input)
        if (TRACK_MODE == 14) // no MPC, direct low-level tracking
        {
          mpc_setpoint_task << solver.work->Xref.col(0)(0), solver.work->Xref.col(0)(1), solver.work->Xref.col(0)(2), solver.work->Xref.col(0)(3), solver.work->Xref.col(0)(4), solver.work->Xref.col(0)(5), 0, 0, 0;
        }
        else
        {
          int cmd_idx = 5;
          mpc_setpoint_task << solver.work->x.col(cmd_idx)(0), solver.work->x.col(cmd_idx)(1), solver.work->x.col(cmd_idx)(2), solver.work->x.col(cmd_idx)(3), solver.work->x.col(cmd_idx)(4), solver.work->x.col(cmd_idx)(5), solver.work->u.col(0)(0), solver.work->u.col(0)(1), solver.work->u.col(0)(2) - GRAVITY_MAGNITUDE;
        }
        // Low-pass filter the setpoint
        // mpc_setpoint_task = ALPHA_LPF * mpc_setpoint_task + (1 - ALPHA_LPF) * mpc_setpoint_prev;
        // mpc_setpoint_prev = mpc_setpoint_task;

        if (1)
        {
          // mpc_time_us = usecTimestamp() - mpc_start_timestamp - 1000;
          // DEBUG_PRINT("t: %lu\n", mpc_time_us);

          // DEBUG_PRINT("xr: %.2f %.2f %.2f %.2f\n", solver.work->Xref.col(1)(0), solver.work->Xref.col(2)(0), solver.work->Xref.col(3)(0), solver.work->Xref.col(4)(0));

          // DEBUG_PRINT("z: %.2f %.2f %.2f %.2f\n", solver.work->x.col(1)(2), solver.work->x.col(2)(2), solver.work->x.col(3)(2), solver.work->x.col(4)(2));

          // DEBUG_PRINT("u: %.2f %.2f %.2f\n", solver.work->u.col(0)(0), solver.work->u.col(0)(1), solver.work->u.col(0)(2));

          // float err = (solver.work->x.col(0) - solver.work->Xref.col(0)).norm();
          // DEBUG_PRINT("e: %.2f\n", err);

          DEBUG_PRINT("sp: %.2f %.2f %.2f\n", mpc_setpoint_task(0), mpc_setpoint_task(1), mpc_setpoint_task(2));
        }

        // Copy the setpoint calculated by the task loop to the global mpc_setpoint
        xSemaphoreTake(dataMutex, portMAX_DELAY);
        memcpy(&mpc_setpoint, &mpc_setpoint_task, sizeof(Matrix<tinytype, NSTATES + NINPUTS, 1>));
        // memcpy(&init_vel_z, &problem.x.col(0)(8), sizeof(float));
        xSemaphoreGive(dataMutex);
      }
    }
  }

  /**
   * This function is called from the stabilizer loop. It is important that this call returns
   * as quickly as possible. The dataMutex must only be locked short periods by the task.
   */
  void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick)
  {
    if (RATE_DO_EXECUTE(LOWLEVEL_RATE, tick))
    {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      memcpy(&setpoint_data, setpoint, sizeof(setpoint_t));
      memcpy(&sensors_data, sensors, sizeof(sensorData_t));
      memcpy(&state_data, state, sizeof(state_t));

      setpoint_low_level.mode.yaw = modeAbs;
      setpoint_low_level.mode.x = modeAbs;
      setpoint_low_level.mode.y = modeAbs;
      setpoint_low_level.mode.z = modeAbs;
      setpoint_low_level.attitudeRate.yaw = 0.0;
      if (TRACK_MODE == 0 || TRACK_MODE == 14)
      {
        setpoint_low_level.position.x = mpc_setpoint(0);
        setpoint_low_level.position.y = mpc_setpoint(1);
        setpoint_low_level.position.z = mpc_setpoint(2);
        // controllerPid(control, &setpoint_low_level, sensors, state, tick);
        controllerBrescianini(control, &setpoint_low_level, sensors, state, tick);
      }
      if (TRACK_MODE == 1)
      {
        setpoint_low_level.attitude.yaw = 12341.0; // custom mode selection
        setpoint_low_level.velocity.x = mpc_setpoint(3);
        setpoint_low_level.velocity.y = mpc_setpoint(4);
        setpoint_low_level.velocity.z = mpc_setpoint(5);
        controllerBrescianini(control, &setpoint_low_level, sensors, state, tick);
      }
      if (TRACK_MODE == 2)
      {
        setpoint_low_level.attitude.yaw = 12342.0;
        setpoint_low_level.acceleration.x = mpc_setpoint(6);
        setpoint_low_level.acceleration.y = mpc_setpoint(7);
        setpoint_low_level.acceleration.z = mpc_setpoint(8);
        controllerBrescianini(control, &setpoint_low_level, sensors, state, tick);
      }
      if (TRACK_MODE == 3)
      {
        setpoint_low_level.position.x = mpc_setpoint(0);
        setpoint_low_level.position.y = mpc_setpoint(1);
        setpoint_low_level.position.z = mpc_setpoint(2);
        setpoint_low_level.velocity.x = mpc_setpoint(3);
        setpoint_low_level.velocity.y = mpc_setpoint(4);
        setpoint_low_level.velocity.z = mpc_setpoint(5);
        controllerBrescianini(control, &setpoint_low_level, sensors, state, tick);
        // controllerMellingerFirmware(control, &setpoint_low_level, sensors, state, tick);
      }
      if (TRACK_MODE == 4)
      {
        setpoint_low_level.attitude.yaw = 12344.0;
        setpoint_low_level.velocity.x = mpc_setpoint(3);
        setpoint_low_level.velocity.y = mpc_setpoint(4);
        setpoint_low_level.velocity.z = mpc_setpoint(5);
        setpoint_low_level.acceleration.x = mpc_setpoint(6);
        setpoint_low_level.acceleration.y = mpc_setpoint(7);
        setpoint_low_level.acceleration.z = mpc_setpoint(8);
        controllerBrescianini(control, &setpoint_low_level, sensors, state, tick);
      }
      if (TRACK_MODE == 5)
      {
        setpoint_low_level.position.x = mpc_setpoint(0);
        setpoint_low_level.position.y = mpc_setpoint(1);
        setpoint_low_level.position.z = mpc_setpoint(2);
        setpoint_low_level.velocity.x = mpc_setpoint(3);
        setpoint_low_level.velocity.y = mpc_setpoint(4);
        setpoint_low_level.velocity.z = mpc_setpoint(5);
        setpoint_low_level.acceleration.x = mpc_setpoint(6);
        setpoint_low_level.acceleration.y = mpc_setpoint(7);
        setpoint_low_level.acceleration.z = mpc_setpoint(8);
        controllerBrescianini(control, &setpoint_low_level, sensors, state, tick);
        // controllerMellingerFirmware(control, &setpoint_low_level, sensors, state, tick);
      }
      if (0 && RATE_DO_EXECUTE(10, tick))
      {
        struct vec pError = mkvec(solver.work->x.col(0)(0) - solver.work->Xref.col(0)(0),
                                  solver.work->x.col(0)(1) - solver.work->Xref.col(0)(1),
                                  solver.work->x.col(0)(2) - solver.work->Xref.col(0)(2));
        DEBUG_PRINT("exz: %.2f %.2f\n", pError.x, pError.z);
        // DEBUG_PRINT("a: %.2f %.2f %.2f\n", mpc_setpoint(0), mpc_setpoint(1), mpc_setpoint(2));
      }
      // Don't fly away
      if (1)
      {
        control->normalizedForces[0] = 0.0f;
        control->normalizedForces[1] = 0.0f;
        control->normalizedForces[2] = 0.0f;
        control->normalizedForces[3] = 0.0f;
      }

      xSemaphoreGive(dataMutex);
    }

    // Allows mpc task to run again
    xSemaphoreGive(runTaskSemaphore);
  }

  /**
   * Logging variables for the command and reference signals for the
   * MPC controller
   */

  LOG_GROUP_START(tinympc)
  LOG_GROUP_STOP(tinympc)

#ifdef __cplusplus
} /* extern "C" */
#endif