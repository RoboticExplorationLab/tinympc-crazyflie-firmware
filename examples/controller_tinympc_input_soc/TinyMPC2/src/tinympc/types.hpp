#pragma once

#include <Eigen.h>
#include "glob_opts.hpp"

using Eigen::Matrix;

#ifdef __cplusplus
extern "C"
{
#endif

    typedef Matrix<tinytype, NSTATES, 1> tiny_VectorNx;
    typedef Matrix<tinytype, NINPUTS, 1> tiny_VectorNu;
    typedef Matrix<tinytype, NSTATES, NSTATES> tiny_MatrixNxNx;
    typedef Matrix<tinytype, NSTATES, NINPUTS> tiny_MatrixNxNu;
    typedef Matrix<tinytype, NINPUTS, NSTATES> tiny_MatrixNuNx;
    typedef Matrix<tinytype, NINPUTS, NINPUTS> tiny_MatrixNuNu;

    typedef Matrix<tinytype, NSTATES, NHORIZON> tiny_MatrixNxNh;       // Nu x Nh
    typedef Matrix<tinytype, NINPUTS, NHORIZON - 1> tiny_MatrixNuNhm1; // Nu x Nh-1

    /**
     * Matrices that must be recomputed with changes in time step, rho
     */
    typedef struct
    {
        tinytype rho;
        tiny_MatrixNuNx Kinf;
        tiny_MatrixNxNx Pinf;
        tiny_MatrixNuNu Quu_inv;
        tiny_MatrixNxNx AmBKt;
        tiny_VectorNx APf;
        tiny_VectorNu BPf;
    } TinyCache;

    /**
     * User settings
     */
    typedef struct
    {
        tinytype abs_pri_tol;
        tinytype abs_dua_tol;
        int max_iter;
        int check_termination;
        int en_state_bound;
        int en_input_bound;
        int en_state_soc;
        int en_input_soc;
    } TinySettings;

    /**
     * Constraint bounds and their slack and dual variables
    */
    typedef struct
    {
        tiny_MatrixNuNhm1 u_min;
        tiny_MatrixNuNhm1 u_max;
        tiny_MatrixNxNh x_min;
        tiny_MatrixNxNh x_max;

        // Slack variables
        tiny_MatrixNuNhm1 z; // input slack variables
        tiny_MatrixNuNhm1 znew;
        tiny_MatrixNxNh v; // state slack variables
        tiny_MatrixNxNh vnew;

        // Dual variables
        tiny_MatrixNuNhm1 y; // input dual variables
        tiny_MatrixNxNh g; // state dual variables
    } TinyBounds;

    /**
     * Second order cone variables
    */
    typedef struct
    {
        tinytype cu[NUM_INPUT_CONES]; // coefficients for input cones
        tinytype cx[NUM_STATE_CONES]; // coefficients for state cones
        int Acu[NUM_INPUT_CONES]; // start indices for input cones
        int Acx[NUM_STATE_CONES]; // start indices for state cones
        int qcu[NUM_INPUT_CONES]; // dimensions for input cones
        int qcx[NUM_STATE_CONES]; // dimensions for state cones

        // Slack variables
        tiny_MatrixNuNhm1 zc[NUM_INPUT_CONES]; // input slack variables for cones
        tiny_MatrixNuNhm1 zcnew[NUM_INPUT_CONES];
        tiny_MatrixNxNh vc[NUM_STATE_CONES]; // state slack variables for cones
        tiny_MatrixNxNh vcnew[NUM_STATE_CONES];

        // Dual variables
        tiny_MatrixNuNhm1 yc[NUM_INPUT_CONES]; // input dual variables
        tiny_MatrixNxNh gc[NUM_STATE_CONES]; // state dual variables
    } TinySocs;
    
    /**
     * Problem variables
     */
    typedef struct
    {
        // State and input
        tiny_MatrixNxNh x;
        tiny_MatrixNuNhm1 u;

        // Linear control cost terms
        tiny_MatrixNxNh q;
        tiny_MatrixNuNhm1 r;

        // Linear Riccati backward pass terms
        tiny_MatrixNxNh p;
        tiny_MatrixNuNhm1 d;

        // Termination conditions
        tinytype primal_residual_state;
        tinytype primal_residual_input;
        tinytype dual_residual_state;
        tinytype dual_residual_input;

        int status;
        int iter;

        // LQR variables
        tiny_VectorNx Q;
        tiny_VectorNu R;
        tiny_MatrixNxNx Adyn; // state transition matrix
        tiny_MatrixNxNu Bdyn; // control matrix
        tiny_VectorNx fdyn; // affine vector

        // Reference trajectory for one horizon
        tiny_MatrixNxNh Xref;   // Nx x Nh
        tiny_MatrixNuNhm1 Uref; // Nu x Nh-1

        // Intermediate calculations
        tiny_VectorNu Qu;

        // Constraints
        TinyBounds *bounds;
        TinySocs *socs;
    } TinyWorkspace;

    /**
     * Main TinyMPC solver structure that holds all information
     */
    typedef struct
    {
        TinySettings *settings; // Problem settings
        TinyCache *cache;       // Problem cache
        TinyWorkspace *work;    // Solver workspace
    } TinySolver;

#ifdef __cplusplus
}
#endif