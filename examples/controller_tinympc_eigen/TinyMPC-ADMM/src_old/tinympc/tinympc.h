#ifndef TINYMPC_H
# define TINYMPC_H

// NOTE: this is an odd fix to get gcov to run correctly on GitHub Actions:
// https://www.osadl.org/fileadmin/dam/interface/docbook/howtos/coverage.pdf
// void __gcov_flush(void);

#include "constants.h"
#include "types.h"
#include "utils.h"
#include "model.h"
#include "auxil.h"
#include "cost_lqr.h"
#include "lqr.h"
#include "constraint_linear.h"
#include "admm.h"

#endif // ifndef TINYMPC_H