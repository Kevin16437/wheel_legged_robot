/*
 * File: _coder_leg_conv_api.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Oct-2024 09:51:06
 */

#ifndef _CODER_LEG_CONV_API_H
#define _CODER_LEG_CONV_API_H

/* Include Files */
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <string.h>

/* Variable Declarations */
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
void leg_conv(real32_T F, real32_T Tp, real32_T phi1, real32_T phi4,
              real32_T T[2]);

void leg_conv_api(const mxArray *const prhs[4], const mxArray **plhs);

void leg_conv_atexit(void);

void leg_conv_initialize(void);

void leg_conv_terminate(void);

void leg_conv_xil_shutdown(void);

void leg_conv_xil_terminate(void);

void leg_pos(real32_T phi1, real32_T phi4, real32_T pos[2]);

void leg_pos_api(const mxArray *const prhs[2], const mxArray **plhs);

void leg_spd(real32_T dphi1, real32_T dphi4, real32_T phi1, real32_T phi4,
             real32_T spd[2]);

void leg_spd_api(const mxArray *const prhs[4], const mxArray **plhs);

void lqr_k(real32_T L0, real32_T K[12]);

void lqr_k_api(const mxArray *prhs, const mxArray **plhs);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for _coder_leg_conv_api.h
 *
 * [EOF]
 */
