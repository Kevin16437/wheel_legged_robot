/*
 * File: rt_nonfinite.h
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 05-Dec-2024 17:00:59
 */

#ifndef RT_NONFINITE_H
#define RT_NONFINITE_H

/* Include Files */
#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

extern real_T rtInf;
extern real_T rtMinusInf;
extern real_T rtNaN;
extern real32_T rtInfF;
extern real32_T rtMinusInfF;
extern real32_T rtNaNF;

extern boolean_T rtIsInf(real_T value);
extern boolean_T rtIsInfF(real32_T value);
extern boolean_T rtIsNaN(real_T value);
extern boolean_T rtIsNaNF(real32_T value);

#ifdef __cplusplus
}
#endif
#endif
/*
 * File trailer for rt_nonfinite.h
 *
 * [EOF]
 */
