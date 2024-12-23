/*
 * File: lqr_k.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 13-Aug-2024 22:10:22
 */

/* Include Files */
#include "lqr_k.h"
#include "arm_math.h"

/* Function Definitions */
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
   /*     2024-10-11 16:27:42 */
  t2 = L0 * L0;
  t3 = L0 * L0 * L0;
  K[0] =
      ((L0 * -102.172974F + t2 * 205.990891F) - t3 * 178.501068F) + 1.19033861F;
  K[1] = ((L0 * 204.556488F - t2 * 850.466675F) + t3 * 1008.02698F) + 2.43186F;
  K[2] = ((L0 * -17.4115715F + t2 * 28.4797878F) - t3 * 26.4078121F) +
         0.441204906F;
  K[3] =
      ((L0 * 37.475235F - t2 * 158.514618F) + t3 * 188.460175F) + 1.33957219F;
  K[4] = ((L0 * -25.2884159F + t2 * 63.9108772F) - t3 * 59.0160599F) +
         0.0803580955F;
  K[5] =
      ((L0 * 46.3387718F - t2 * 235.932617F) + t3 * 301.879944F) + 3.24664426F;
  K[6] = ((L0 * -24.4128723F + t2 * 60.7964821F) - t3 * 57.2897949F) -
         0.201834753F;
  K[7] = ((L0 * 36.995327F - t2 * 209.714142F) + t3 * 275.99173F) + 4.58759069F;
  K[8] =
      ((L0 * -25.5811195F + t2 * 1.8538928F) + t3 * 35.5645485F) + 10.9986372F;
  K[9] = ((L0 * 151.749771F - t2 * 341.928F) + t3 * 278.822357F) + 5.48028803F;
  K[10] =
      ((L0 * -1.83016443F + t2 * 1.37659299F) + t3 * 0.418746978F) + 0.837129F;
  K[11] =
      ((L0 * 5.72250748F - t2 * 9.21521F) + t3 * 4.36989164F) + 0.372868448F;
}
/*
 * File trailer for lqr_k.c
 *
 * [EOF]
 */
