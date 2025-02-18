/*
 * File: lqr_k.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 17-Oct-2024 09:51:06
 */

/* Include Files */
#include "lqr_k.h"
#include "rt_nonfinite.h"
#include "rt_nonfinite.h"
#include <math.h>

/* Function Declarations */
static float rt_powf_snf(float u0, float u1);

/* Function Definitions */
/*
 * Arguments    : float u0
 *                float u1
 * Return Type  : float
 */
static float rt_powf_snf(float u0, float u1)
{
  float y;
  if (rtIsNaNF(u0) || rtIsNaNF(u1)) {
    y = rtNaNF;
  } else {
    float f;
    float f1;
    f = fabsf(u0);
    f1 = fabsf(u1);
    if (rtIsInfF(u1)) {
      if (f == 1.0F) {
        y = 1.0F;
      } else if (f > 1.0F) {
        if (u1 > 0.0F) {
          y = rtInfF;
        } else {
          y = 0.0F;
        }
      } else if (u1 > 0.0F) {
        y = 0.0F;
      } else {
        y = rtInfF;
      }
    } else if (f1 == 0.0F) {
      y = 1.0F;
    } else if (f1 == 1.0F) {
      if (u1 > 0.0F) {
        y = u0;
      } else {
        y = 1.0F / u0;
      }
    } else if (u1 == 2.0F) {
      y = u0 * u0;
    } else if ((u1 == 0.5F) && (u0 >= 0.0F)) {
      y = sqrtf(u0);
    } else if ((u0 < 0.0F) && (u1 > floorf(u1))) {
      y = rtNaNF;
    } else {
      y = powf(u0, u1);
    }
  }
  return y;
}

/*
 * LQR_K
 *     K = LQR_K(L0)
 *
 * Arguments    : float L0
 *                float K[12]
 * Return Type  : void
 */
void lqr_k(float L0, float K[12])
{
  float t2;
  float t3;
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     2024-10-17 09:51:04 */
  t2 = L0 * L0;
  t3 = rt_powf_snf(L0, 3.0F);
  K[0] = ((L0 * -92.5476379F + t2 * 196.543381F) - t3 * 222.798615F) -
         0.200150236F;
  K[1] =
      ((L0 * -29.916954F + t2 * 60.2758331F) - t3 * 42.8188171F) + 7.66083717F;
  K[2] =
      ((L0 * -8.99449F + t2 * 1.98185742F) - t3 * 5.44713068F) + 0.110737577F;
  K[3] = ((L0 * -2.64304662F + t2 * 4.93876696F) - t3 * 3.12577033F) +
         0.872653246F;
  K[4] =
      ((L0 * -34.001133F + t2 * 128.642487F) - t3 * 171.995667F) - 2.72714806F;
  K[5] =
      ((L0 * -85.8332138F + t2 * 270.36319F) - t3 * 316.922943F) + 11.397275F;
  K[6] =
      ((L0 * -25.1489906F + t2 * 82.3510818F) - t3 * 108.060913F) - 1.99973762F;
  K[7] =
      ((L0 * -60.586071F + t2 * 192.564438F) - t3 * 227.765457F) + 8.07050323F;
  K[8] = ((L0 * -222.699265F + t2 * 749.631531F) - t3 * 928.239F) + 27.8462429F;
  K[9] =
      ((L0 * 279.250732F - t2 * 1080.83203F) + t3 * 1458.62708F) + 33.906868F;
  K[10] =
      ((L0 * -8.61835575F + t2 * 25.4567318F) - t3 * 29.007185F) + 1.51188564F;
  K[11] =
      ((L0 * 11.6953573F - t2 * 42.1043816F) + t3 * 54.2976646F) + 0.969880402F;
}

/*
 * File trailer for lqr_k.c
 *
 * [EOF]
 */
