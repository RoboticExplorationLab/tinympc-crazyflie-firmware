#ifndef CONSTRAINT_LINEAR_H
# define CONSTRAINT_LINEAR_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus


#include "types.h"
#include "utils.h"

enum tiny_ErrorCode tiny_SetInputBound(tiny_AdmmWorkspace* work, sfloat* Ac_data, sfloat* lc_data, sfloat* uc_data);

enum tiny_ErrorCode tiny_SetStateBound(tiny_AdmmWorkspace* work, sfloat* Ac_data, sfloat* lc_data, sfloat* uc_data);

// enum tiny_ErrorCode tiny_ProjectInput(tiny_AdmmWorkspace* work);

int IsConstrained(tiny_AdmmWorkspace* work);


# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef CONSTRAINT_LINEAR_H