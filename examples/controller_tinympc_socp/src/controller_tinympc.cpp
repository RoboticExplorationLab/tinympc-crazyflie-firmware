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

// TinyMPC and PID controllers
#include "tinympc/admm.hpp"
#include "controller_pid.h"
#include "controller_brescianini.h"

// Params
// #include "quadrotor_20hz_params.hpp"
#include "rocket_landing_params_20hz.hpp"

// Trajectory
// #include "quadrotor_100hz_ref_hover.hpp"

// Edit the debug name to get nice debug prints
#define DEBUG_MODULE "MPCTASK"
#include "debug.h"


// #define MPC_RATE RATE_250_HZ  // control frequency
// #define RATE_25_HZ 25
// #define MPC_RATE RATE_50_HZ
#define MPC_RATE RATE_100_HZ
#define LOWLEVEL_RATE RATE_500_HZ
#define USE_PID 1  // 1 for PID (state setpoint), 0 for Brescianini (accel setpoint)


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
static tiny_VectorNx mpc_setpoint;
static setpoint_t mpc_setpoint_pid;
// Copies that stay constant for duration of MPC loop
static setpoint_t setpoint_task;
static sensorData_t sensors_task;
static state_t state_task;
static control_t control_task;
static tiny_VectorNx mpc_setpoint_task;

/* Allocate global variables for MPC */
static TinyBounds bounds;
static TinySocs socs;
static TinyWorkspace work;
static TinyCache cache;
static TinySettings settings;
static TinySolver solver{&settings, &cache, &work};

static tiny_VectorNu u_min_one_time_step(-10.0, -10.0, -10.0);
static tiny_VectorNu u_max_one_time_step(105.0, 105.0, 105.0);
static tiny_VectorNx x_min_one_time_step(-5.0, -5.0, -0.5, -10.0, -10.0, -20.0);
static tiny_VectorNx x_max_one_time_step(5.0, 5.0, 100.0, 10.0, 10.0, 20.0);

static tiny_MatrixNxNh problem_x;
static float horizon_nh_z;
static tiny_VectorNu u_lqr;
static tiny_VectorNx current_state;
static tiny_VectorNx xinit;
static tiny_VectorNx xg;
static tiny_VectorNx x0;

// Helper variables
static int8_t result = 0;
static uint32_t step = 0;         // mpc steps taken
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


void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  while (1)
  {
    vTaskDelay(M2T(2000));  //TODO: Can I make this really big?
  }
}
bool controllerOutOfTreeTest()
{
  // Always return true
  return true;
}


void controllerOutOfTreeInit(void)
{
  if (USE_PID) {
    controllerPidInit();
  }
  else {
    controllerBrescianiniInit();
  }

  work.bounds = &bounds;
  work.socs = &socs;

  /* Map data from problem_data (array in row-major order) */

  //////// Cache
  cache.rho = rho_value;
  cache.Kinf = Eigen::Map<Matrix<tinytype, NINPUTS, NSTATES, Eigen::RowMajor>>(Kinf_data);
  cache.Pinf = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Pinf_data);
  cache.Quu_inv = Eigen::Map<Matrix<tinytype, NINPUTS, NINPUTS, Eigen::RowMajor>>(Quu_inv_data);
  cache.AmBKt = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(AmBKt_data);
  cache.APf = Eigen::Map<Matrix<tinytype, NSTATES, 1>>(APf_data);
  cache.BPf = Eigen::Map<Matrix<tinytype, NINPUTS, 1>>(BPf_data);

  //////// Workspace (dynamics and LQR cost matrices)
  work.Adyn = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Adyn_data);
  work.Bdyn = Eigen::Map<Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(Bdyn_data);
  work.fdyn = Eigen::Map<Matrix<tinytype, NSTATES, 1>>(fdyn_data);
  work.Q = Eigen::Map<tiny_VectorNx>(Q_data);
  work.R = Eigen::Map<tiny_VectorNu>(R_data);

  //////// Box constraints

  work.bounds->u_min = u_min_one_time_step.replicate(1, NHORIZON - 1);
  work.bounds->u_max = u_max_one_time_step.replicate(1, NHORIZON - 1);
  work.bounds->x_min = x_min_one_time_step.replicate(1, NHORIZON);
  work.bounds->x_max = x_max_one_time_step.replicate(1, NHORIZON);

  //////// Second order cone constraints
  work.socs->cu[0] = 0.25; // coefficients for input cones (mu)
  work.socs->cx[0] = 0.6;  // coefficients for state cones (mu)
  work.socs->Acu[0] = 0; // start indices for input cones
  work.socs->Acx[0] = 0; // start indices for state cones
  work.socs->qcu[0] = 3; // dimensions for input cones
  work.socs->qcx[0] = 3; // dimensions for state cones

  //////// Settings
  settings.abs_pri_tol = 0.01;
  settings.abs_dua_tol = 0.01;
  settings.max_iter = 2;
  settings.check_termination = 1;
  settings.en_state_bound = 0;
  settings.en_input_bound = 1;
  settings.en_state_soc = 0;
  settings.en_input_soc = 1;

  //////// Initialize other workspace values automatically
  reset_problem(&solver);

  // Initial state
  // xinit << 4, 2, 20, -3, 2, -4.5;
  xinit << 0, 0, 1, 0, 0, 0.0;
  xg << 0, 1, 1, 0, 0, 0.0;
  x0 = xinit * 1.1;

  // Uref stays constant, Xref interpolate between start and goal states
  for (int i = 0; i < NHORIZON - 1; i++)
  {
    work.Uref.col(i)(2) = 10;
  }
  // for (int i = 0; i < NHORIZON; i++)
  // {
  //   work.Xref.col(i) = xinit + (xg - xinit) * tinytype(i) / (NTOTAL);
  // }
  // tiny_solve(&solver);

  /* Begin task initialization */
  runTaskSemaphore = xSemaphoreCreateBinary();
  ASSERT(runTaskSemaphore);

  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

  STATIC_MEM_TASK_CREATE(tinympcControllerTask, tinympcControllerTask, TINYMPC_TASK_NAME, NULL, TINYMPC_TASK_PRI);

  isInit = true;
  /* End of task initialization */
}


void updateHorizonReference(const setpoint_t *setpoint) {
  traj_idx++;
  for (int i = 0; i < NHORIZON; i++)
  {
    if (traj_idx + i >= NTOTAL)
    {
        work.Xref.col(i) = xg;
    }
    else
    {
        work.Xref.col(i) = xinit + (xg - xinit) * tinytype(i + traj_idx) / (NTOTAL);
    }
  }
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
      work.x.col(0) = x0;

      // 2. Update reference
      updateHorizonReference(&setpoint_task);

      // 3. Reset dual variables if needed

      // 4. Solve MPC problem
      // unsigned long start = micros();
      tiny_solve(&solver);
      // unsigned long end = micros();

      mpc_setpoint_task = solver.work->x.col(1);

      // Copy the setpoint calculated by the task loop to the global mpc_setpoint
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      memcpy(&mpc_setpoint, &mpc_setpoint_task, sizeof(tiny_VectorNx));
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
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  if (RATE_DO_EXECUTE(LOWLEVEL_RATE, tick))
  {
    if (USE_PID) {
    mpc_setpoint_pid.mode.yaw = modeAbs;
    mpc_setpoint_pid.mode.x = modeAbs;
    mpc_setpoint_pid.mode.y = modeAbs;
    mpc_setpoint_pid.mode.z = modeAbs;
    mpc_setpoint_pid.position.x = mpc_setpoint(0);
    mpc_setpoint_pid.position.y = mpc_setpoint(1);
    mpc_setpoint_pid.position.z = mpc_setpoint(2);
    // mpc_setpoint_pid.position.x = 0.0;
    // mpc_setpoint_pid.position.y = 0.0;
    // mpc_setpoint_pid.position.z = 1.0;
    mpc_setpoint_pid.attitude.yaw = 0.0;
    controllerPid(control, &mpc_setpoint_pid, sensors, state, tick);
    }
    else {
    mpc_setpoint_pid.mode.yaw = modeAbs;
    mpc_setpoint_pid.mode.x = modeAbs;
    mpc_setpoint_pid.mode.y = modeAbs;
    mpc_setpoint_pid.mode.z = modeAbs;
    mpc_setpoint_pid.attitude.yaw = 0.0;
    mpc_setpoint_pid.attitudeRate.yaw = 0.0;
    mpc_setpoint_pid.position.x = 1234.0;  // magic number to tell Bres to use MPC
    mpc_setpoint_pid.acceleration.x = mpc_setpoint(0);
    mpc_setpoint_pid.acceleration.y = mpc_setpoint(1);
    mpc_setpoint_pid.acceleration.z = mpc_setpoint(2);
    controllerBrescianini(control, &mpc_setpoint_pid, sensors, state, tick);
    }

    // Don't fly away
    if (1) {
      control->normalizedForces[0] = 0.0f;
      control->normalizedForces[1] = 0.0f;
      control->normalizedForces[2] = 0.0f;
      control->normalizedForces[3] = 0.0f;
    }
  }

  xSemaphoreGive(dataMutex);

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