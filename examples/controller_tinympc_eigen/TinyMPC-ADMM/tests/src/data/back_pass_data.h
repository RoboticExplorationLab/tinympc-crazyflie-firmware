#ifndef BACK_PASS_DATA_H
# define BACK_PASS_DATA_H

# ifdef __cplusplus
extern "C" {
# endif // ifdef __cplusplus


#include "slap/slap.h"
sfloat X_data[12] = {5.0, 7.0,  2.0, -1.4, 5.0, 7.0,
                     2.0, -1.4, 5.0, 7.0,  2.0, -1.4};

sfloat U_data[4] = {-7.522015227259879e-5, -0.0019667347526769585,
                    0.005620890315977286, -0.01944557581703168};

sfloat dsln_data[4] = {10.75088111978507, 1.104847001842924, 11.238761987040757,
                       -5.253156339850558};


# ifdef __cplusplus
}
# endif // ifdef __cplusplus

#endif // ifndef BACK_PASS_DATA_H