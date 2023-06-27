#ifndef COST_LQR_H
# define COST_LQR_H

#include "types.h"
#include "utils.h"

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus

enum tiny_ErrorCode tiny_AddStageCost(tiny_AdmmWorkspace* work, const int k);

enum tiny_ErrorCode tiny_AddTerminalCost(tiny_AdmmWorkspace* work);

enum tiny_ErrorCode tiny_UpdateLinearCost(tiny_AdmmWorkspace* work);

enum tiny_ErrorCode tiny_UpdateConstrainedLinearCost(tiny_AdmmWorkspace* work);

# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef COST_LQR_H