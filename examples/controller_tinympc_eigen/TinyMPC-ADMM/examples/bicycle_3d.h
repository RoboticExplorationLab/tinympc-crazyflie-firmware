#ifndef BICYCLE_3D_H
# define BICYCLE_3D_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus


#include <math.h>
#include <slap/slap.h>

//========================================
// Bicycle model parameters
// X = [x; y; theta] : x, y, yaw
// U = [v; delta] : velocity and steering angle
//========================================

//========================================
// Codes generated from julia/bicycle_tvlqr
// Discrete dynamics of bicycle model with predefined model params
//========================================
void tiny_Bicycle3dNonlinearDynamics_Raw(sfloat* xn, const sfloat* x,
                                         const sfloat* u);

void tiny_Bicycle3dNonlinearDynamics(Matrix* xn, const Matrix x,
                                     const Matrix u);

//========================================
// Codes generated from julia/bicycle_tvlqr
// Jacobians of discrete dynamics of bicycle model with predefined model params
//========================================
void tiny_Bicycle3dGetJacobianA_Raw(sfloat* A, const sfloat* x,
                                    const sfloat* u);

void tiny_Bicycle3dGetJacobianB_Raw(sfloat* B, const sfloat* x,
                                    const sfloat* u);

void tiny_Bicycle3dGetJacobians(Matrix* A, Matrix* B, const Matrix x,
                                const Matrix u);



# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef BICYCLE_3D_H