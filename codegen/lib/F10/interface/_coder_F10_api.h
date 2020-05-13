/*
 * File: _coder_F10_api.h
 *
 * MATLAB Coder version            : 4.0
 * C/C++ source code generated on  : 31-May-2018 01:32:48
 */

#ifndef _CODER_F10_API_H
#define _CODER_F10_API_H

/* Include Files */
#include "tmwtypes.h"
#include "mex.h"
#include "emlrt.h"
#include <stddef.h>
#include <stdlib.h>
#include "_coder_F10_api.h"

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

/* Function Declarations */
extern void F10(real_T in1[10], real_T in2[11], real_T F[10]);
extern void F10_api(const mxArray * const prhs[2], int32_T nlhs, const mxArray
                    *plhs[1]);
extern void F10_atexit(void);
extern void F10_initialize(void);
extern void F10_terminate(void);
extern void F10_xil_terminate(void);

#endif

/*
 * File trailer for _coder_F10_api.h
 *
 * [EOF]
 */
