#ifndef TYPES_H
# define TYPES_H

# include <Eigen.h>
#include "constants.h"
#include "errors.h"

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

/**
 * Linear algebra type
 */
namespace Eigen
{ 
    typedef Matrix<float, NSTATES, NSTATES> MatrixNf;
    typedef Matrix<float, NSTATES, NINPUTS> MatrixNMf;
    typedef Matrix<float, NINPUTS, NSTATES> MatrixMNf;
    typedef Matrix<float, NINPUTS, NINPUTS> MatrixMf;
    typedef Vector<float, NSTATES>          VectorNf;  
    typedef Vector<float, NINPUTS>          VectorMf; 
}

// for a horizon of N x(0)->x(N-1), need N-1 matrices
typedef struct {
  int nstates;
  int ninputs;
  int nhorizon;

  int   ltv;            ///< Boolean, true if model is LTV  
  int   affine;         ///< Boolean, true if model is affine
  float dt;          ///< Sample time Ts of the discrete model

  Eigen::MatrixNf*  A;
  Eigen::MatrixNMf* B;
  Eigen::VectorNf*  f;

  void (*get_jacobians)(Eigen::MatrixNf*, Eigen::MatrixNMf*, Eigen::VectorNf*, Eigen::VectorMf*);
  void (*get_nonl_model)(Eigen::VectorNf*, Eigen::VectorNf*, Eigen::VectorMf*);
  int data_size;
} tiny_Model;


/**
 * Solution structure
 */
typedef struct {
  Eigen::VectorNf* X;      ///< State trajectory solution 
  Eigen::VectorMf* U;      ///< Input trajectory solution

  Eigen::MatrixMNf* Kinf;    ///< Feedback gain of IHLQR
  Eigen::VectorMf*  d;      ///< Feedforward gain
  Eigen::MatrixNf*  Pinf;    ///< Terminal cost Hessian of IHLQR
  Eigen::VectorNf*  p;      ///< Terminal cost gradient
  
  Eigen::VectorMf* YU;     ///< Dual variables for input constraints
  Eigen::VectorNf* YX;     ///< Dual variables for state constraints
  Eigen::VectorNf* YG;      ///< Dual variables for goal constraint

  int data_size;
} tiny_AdmmSolution;


/**
 * Solver return information
 */
typedef struct {
  int iter;           ///< Number of AL iterations taken
  int iter_riccati;   ///< Number of Riccati iterations taken
  int status_val;     ///< Integer, status defined in constants.h

  float obj_val;     ///< primal objective
  float pri_res;     ///< norm of primal residual
  float dua_res;     ///< norm of dual residual
} tiny_AdmmInfo;


/**********************************
* Main structures and Data Types *
**********************************/

/**
 * Settings struct
 */
typedef struct {
  float reg_min;             ///< Minimum regularization
  float reg_max;             ///< Maximum regularization
  float reg_mul;             ///< Regularization update multiplier
  int   en_reg_update;       ///< Boolean, enable regularization update (tighter solve)
  
  float rho_init;            ///< Initial rho
  float rho_max;             ///< Maximum rho
  float rho_mul;             ///< Penalty multiplier

  float alpha_mul;           ///< Line-search step multiplier

  int   max_iter;            ///< Maximum number of AL iterations
  int   max_iter_riccati;    ///< Maximum number of Riccati solve iterations
  int   max_iter_ls;         ///< Maximum number of line-search iterations

  float tol_abs_prim;        ///< Riccati solve tolerance
  float tol_abs_dual;        ///< Constraint tolerance

  int   en_cstr_states;      ///< Boolean, enable inequality constraints on states
  int   en_cstr_inputs;      ///< Boolean, enable inequality constraints on inputs
  int   en_cstr_goal;        ///< Boolean, enable equality constraint on goal

  int   verbose;             ///< Integer, level to write out progress
  int   adaptive_horizon;    ///< Integer, after `adaptive_horizon` steps, use the second model with longer interval; if 0, disabled 
  int   check_riccati;       ///< Boolean, if 0, then termination checking is disabled
  int   check_termination;   ///< Integer, check termination interval; if 0, then termination checking is disabled
  int   warm_start;          ///< boolean, enable warm start
  float time_limit;          ///< Time limit of each MPC step; if 0, disabled
} tiny_AdmmSettings;

// void tiny_InitSettings(tiny_AdmmSettings* solver);


/**
 * Data structure
 */
typedef struct {
  tiny_Model* model;    ///< System model
  Eigen::VectorNf* x0;

  Eigen::MatrixNf* Q;
  Eigen::MatrixMf* R;
  Eigen::VectorNf* q;
  Eigen::VectorNf* q_tilde;
  Eigen::VectorMf* r;
  Eigen::VectorMf* r_tilde;
  
  Eigen::VectorNf* Xref;
  Eigen::VectorMf* Uref;

  Eigen::VectorNf* Acx;
  Eigen::VectorNf* ucx;
  Eigen::VectorNf* lcx;
  Eigen::MatrixMf* Acu;
  Eigen::VectorMf* ucu;
  Eigen::VectorMf* lcu;
  
  int data_size;
} tiny_AdmmData;

// void tiny_InitProblemData(tiny_ProblemData* prob);

typedef struct {
  tiny_AdmmData*      data;      ///< problem data
  tiny_AdmmSettings*  stgs;      ///< problem settings
  tiny_AdmmSolution*  soln;      ///< problem solution
  tiny_AdmmInfo*      info;      ///< solver information

  float reg;
  float alpha;
  float rho;

  // Temporary data
  Eigen::VectorMf*  Qu;          ///< temporary 
  Eigen::MatrixMf*  Quu_inv;     ///< mxm cache for (R + B'*Pinf*B)\I 
  Eigen::MatrixNf*  AmBKt;       ///< nxn cache for (A - BKinf)'
  Eigen::MatrixNMf* coeff_d2p;   ///< nxm cache for Kinf'*R - AmBKt*Pinf*B
  
  Eigen::VectorMf* ZU;         ///< Slack variable for input
  Eigen::VectorMf* ZU_new;     ///< Updated slack variable for input
  Eigen::VectorNf* ZX;         ///< Slack variable for input
  Eigen::VectorNf* ZX_new;     ///< Updated slack variable for input

  int data_size;      ///< sum data size of all temporary data //TODO: + model + solution 
  int first_run;      ///< flag indicating whether the solve function has been run before
} tiny_AdmmWorkspace;


# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef TYPES_H
