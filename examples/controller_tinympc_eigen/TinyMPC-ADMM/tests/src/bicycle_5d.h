#ifndef BICYCLE_5D_H
# define BICYCLE_5D_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus


#include <math.h>
#include <slap/slap.h>

//========================================
// Bicycle model parameters
// X = [x; y; theta; v; delta] : x, y, yaw, linear vel, steering angle
// U = [a; delta_dot] : linear accel and steering rate
//========================================
// struct tiny_Model_Bicycle {
//   sfloat drive_min[2];
//   sfloat drive_max[2];
//   sfloat u_min[2];
//   sfloat u_max[2];
// } tiny_DefaultModel_Bicycle = {{-2, -0.5}, {2, 0.5}, {-4, -0.7}, {4, 0.7}};

//========================================
// Codes generated from julia/bicycle_tvlqr
// Discrete dynamics of bicycle model with predefined model params
//========================================
void tiny_Bicycle5dNonlinearDynamics_Raw(sfloat* xn, const sfloat* x,
                                         const sfloat* u);

void tiny_Bicycle5dNonlinearDynamics(Matrix* xn, const Matrix x,
                                     const Matrix u);

//========================================
// Codes generated from julia/bicycle_tvlqr
// Jacobians of discrete dynamics of bicycle model with predefined model params
//========================================
void tiny_Bicycle5dGetJacobianA_Raw(sfloat* A, const sfloat* x,
                                    const sfloat* u);

void tiny_Bicycle5dGetJacobianB_Raw(sfloat* B, const sfloat* x,
                                    const sfloat* u);

void tiny_Bicycle5dGetJacobians(Matrix* A, Matrix* B, const Matrix x,
                                const Matrix u);


# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef BICYCLE_5D_H