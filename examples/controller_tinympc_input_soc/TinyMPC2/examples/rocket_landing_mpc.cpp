// Quadrotor tracking example

// This script is just to show how to use the library, the data for this example is not tuned for our Crazyflie demo. Check the firmware code for more details.

// Make sure in glob_opts.hpp:
// - NSTATES = 6, NINPUTS=3
// - NHORIZON = anything you want
// - NTOTAL = 301 if using reference trajectory from trajectory_data/
// - tinytype = float if you want to run on microcontrollers
// States: x (m), y, z, phi, theta, psi, dx, dy, dz, dphi, dtheta, dpsi
// phi, theta, psi are NOT Euler angles, they are Rodiguez parameters
// check this paper for more details: https://ieeexplore.ieee.org/document/9326337
// Inputs: u1, u2, u3, u4 (motor thrust 0-1, order from Crazyflie)

#include <iostream>
#include <fstream> // for writing data to csv file for visualization later

#include <tinympc/admm.hpp>
#include "problem_data/rocket_landing_params_20hz.hpp"
// #include "trajectory_data/rocket_landing_ref_traj_20hz.hpp"

Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
Eigen::IOFormat SaveData(4, 0, ", ", "\n");

#define NRUNS (NTOTAL - NHORIZON - 1)

extern "C"
{

    int main()
    {
        TinyBounds bounds;
        TinySocs socs;
        TinyWorkspace work;
        work.bounds = &bounds;
        work.socs = &socs;

        TinyCache cache;
        TinySettings settings;
        TinySolver solver{&settings, &cache, &work};

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
        tiny_VectorNu u_min_one_time_step(-10.0, -10.0, -10.0);
        tiny_VectorNu u_max_one_time_step(105.0, 105.0, 105.0);
        work.bounds->u_min = u_min_one_time_step.replicate(1, NHORIZON-1);
        work.bounds->u_max = u_max_one_time_step.replicate(1, NHORIZON-1);
        tiny_VectorNx x_min_one_time_step(-5.0, -5.0, -0.5, -10.0, -10.0, -20.0);
        tiny_VectorNx x_max_one_time_step(5.0, 5.0, 100.0, 10.0, 10.0, 20.0);
        work.bounds->x_min = x_min_one_time_step.replicate(1, NHORIZON);
        work.bounds->x_max = x_max_one_time_step.replicate(1, NHORIZON);

        //////// Second order cone constraints
        work.socs->cu[0] = 0.25; // coefficients for input cones (mu)
        work.socs->cx[0] = 0.5; // coefficients for state cones (mu)
        // Number of contiguous input variables to constrain with each cone
        // For example if all inputs are [thrust_x, thrust_y, thrust_z, thrust_2x, thrust_2y, thrust_2z]
        // and we want to put a thrust cone on [thrust_y, thrust_z] we need to set socs->Acu to 1 and socs->qcu to 2
        // which corresponds to a subvector of all input variables starting at index 1 with length 2.
        // Support for arbitrary input constraints will be added in the future.
        work.socs->Acu[0] = 0; // start indices for input cones
        work.socs->Acx[0] = 0; // start indices for state cones
        work.socs->qcu[0] = 3; // dimensions for input cones
        work.socs->qcx[0] = 3; // dimensions for state cones
        
        //////// Settings
        settings.abs_pri_tol = 0.01;
        settings.abs_dua_tol = 0.01;
        settings.max_iter = 100;
        settings.check_termination = 1;
        settings.en_state_bound = 0;
        settings.en_input_bound = 1;
        settings.en_state_soc = 0;
        settings.en_input_soc = 1;

        //////// Initialize other workspace values automatically
        reset_problem(&solver);

        tiny_VectorNx x0, x1; // current and next simulation states
        tiny_VectorNx xinit, xg; // initial and goal states

        // Map data from trajectory_data
        // Matrix<tinytype, NSTATES, NTOTAL> Xref_total = Eigen::Map<Matrix<tinytype, NSTATES, NTOTAL, Eigen::ColMajor>>(Xref_data);
        // Matrix<tinytype, NINPUTS, NTOTAL-1> Uref_total = Eigen::Map<Matrix<tinytype, NINPUTS, NTOTAL-1, Eigen::ColMajor>>(Uref_data);
        // work.Xref = Xref_total.block<NSTATES, NHORIZON>(0, 0);
        // work.Uref = Xref_total.block<NINPUTS, NHORIZON-1>(0, 0);
        
        // Initial state
        xinit << 4, 2, 20, -3, 2, -4.5;
        xg << 0, 0, 0, 0, 0, 0.0;
        x0 = xinit*1.1;
        
        // Uref stays constant
        for (int i=0; i<NHORIZON-1; i++) {
            work.Uref.col(i)(2) = 10;
        }
        for (int i=0; i<NHORIZON; i++) {
            work.Xref.col(i) = xinit + (xg - xinit)*tinytype(i)/(NTOTAL-1);
        }
        work.p.col(NHORIZON-1) = -cache.Pinf*work.Xref.col(NHORIZON-1);

        std::ofstream solver_ref_data_f;
        std::ofstream solver_traj_data_f;
        std::ofstream solver_control_data_f;
        solver_ref_data_f.open("rocket_landing_ref_data.csv");
        solver_traj_data_f.open("rocket_landing_traj_data.csv");
        solver_control_data_f.open("rocket_landing_control_data.csv");

        for (int k = 0; k < NRUNS; k++)
        // for (int k=0; k < 1; k++)
        {
            // std::cout << "tracking error: " << (x0 - work.Xref.col(1)).norm() << std::endl;

            // 1. Update measurement
            work.x.col(0) = x0;

            // 2. Update reference
            for (int i=0; i<NHORIZON; i++) {
                work.Xref.col(i) = xinit + (xg - xinit)*tinytype(i+k)/(NTOTAL-1);
                // work.Uref.col(i)(2) = 10 // uref stays constant
            }
            // work.Xref = Xref_total.block<NSTATES, NHORIZON>(0, k);
            // work.Uref = Uref_total.block<NINPUTS, NHORIZON-1>(0, k);

            // 3. Reset dual variables if needed
            // reset_dual(&solver);

            // 4. Solve MPC problem
            tiny_solve(&solver);

            std::cout << "\n<<<< SOLVE COMPLETE <<<<" << std::endl;
            std::cout << "time step: " << k << std::endl;
            std::cout << "iterations: " << work.iter << std::endl;
            std::cout << "states: " << x0.transpose().format(CleanFmt) << std::endl;
            std::cout << "reference: " << work.Xref.col(0).transpose().format(CleanFmt) << std::endl;
            std::cout << "controls: " << work.u.col(0).transpose().format(CleanFmt) << std::endl;
            std::cout << std::endl;

            // Save data to file
            solver_ref_data_f << work.Xref.col(0).transpose().format(SaveData) << std::endl;
            solver_traj_data_f << x0.transpose().format(SaveData) << std::endl;
            solver_control_data_f << work.u.col(0).transpose().format(SaveData) << std::endl;

            // 5. Simulate forward
            x1 = work.Adyn * x0 + work.Bdyn * work.u.col(0) + work.fdyn;
            x0 = x1;
        }

        solver_ref_data_f.close();
        solver_traj_data_f.close();
        solver_control_data_f.close();

        return 0;
    }

} /* extern "C" */