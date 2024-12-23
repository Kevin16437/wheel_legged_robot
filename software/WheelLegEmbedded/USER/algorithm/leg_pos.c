/*
 * File: leg_pos.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 11-Sep-2024 18:11:05
 */

/* Include Files */
#include "leg_pos.h"
#include "arm_math.h"

/* Function Definitions */
/*
 * LEG_POS
 *     POS = LEG_POS(PHI1,PHI4)
 *
 * Arguments    : float phi1
 *                float phi4
 *                float pos[2]
 * Return Type  : void
 */
void leg_pos(float phi1, float phi4, float pos[2])
{
  float a;
  float b_a;
  float t10;
  float t12;
  float t13;
  float t2;
  float t3;
  float t4;
  float t5;
  float t6;
  /*     This function was generated by the Symbolic Math Toolbox version 23.2.
   */
  /*     2024-09-11 15:53:50 */
  t2 = cosf(phi1);
  t3 = cosf(phi4);
  t4 = sinf(phi1);
  t5 = sinf(phi4);
  t6 = t2 * 0.15F;
  t10 = t4 * 0.15F;
  t12 = t4 * 0.075F;
  t13 = t5 * 0.075F;
  t5 = t10 - t5 * 0.15F;
  a = t12 - t13;
  b_a = (t3 * 0.15F - t6) + 0.125F;
  t4 = (t3 * 0.075F - t2 * 0.075F) + 0.0625F;
  t5 = t5 * t5 + b_a * b_a;
  t4 = atanf(1.0F / (t4 + t5) *
             ((t13 - t12) + sqrtf((a * a + t4 * t4) - t5 * t5))) *
       2.0F;
  t5 = t10 + sinf(t4) / 4.0F;
  t4 = (t6 + cosf(t4) / 4.0F) - 0.0625F;
  pos[0] = sqrtf(t5 * t5 + t4 * t4);
  pos[1] = atan2f(t5, t4);
}

/*
 * File trailer for leg_pos.c
 *
 * [EOF]
 */
