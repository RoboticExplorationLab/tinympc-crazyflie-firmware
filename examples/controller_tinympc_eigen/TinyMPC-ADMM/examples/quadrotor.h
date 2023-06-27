#ifndef QUADROTOR_H
# define QUADROTOR_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus


#include <math.h>

#include "slap/slap.h"

//========================================
// Full 3D quadrotor model (Crazyflie 2.1)
// X = [x; y; z; qw; qx; qy; qz; vx; vy; vz; omega_x; omega_y; omega_z]
// Velocities in body frame
// U = [pwm1; pwm2; pwm3; pwm4] : pwm of four motors
// pwm in [0..1]
//========================================

//========================================
// Codes generated from julia/quadrotor_quat/quadrotor_gen
// Discrete dynamics of quadrotor
//========================================
void tiny_QuadNonlinearDynamics_Raw(sfloat* xn, const sfloat* x,
                                    const sfloat* u);

void tiny_QuadNonlinearDynamics(Matrix* xn, const Matrix x, const Matrix u);

// //========================================
// // Codes generated from julia/quadrotor_quat/quadrotor_gen
// // Jacobians of discrete dynamics of quadrotor
// //========================================
// void tiny_QuadGetJacobianA_Raw(sfloat* A, const sfloat* x, const sfloat* u);

// void tiny_QuadGetJacobianB_Raw(sfloat* B, const sfloat* x, const sfloat* u);

// void tiny_QuadGetJacobians(Matrix* A, Matrix* B, const Matrix x,
//                             const Matrix u);


# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef QUADROTOR_H