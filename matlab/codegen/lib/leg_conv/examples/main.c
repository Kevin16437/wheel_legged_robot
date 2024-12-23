/*
 * File: main.c
 *
 * MATLAB Coder version            : 23.2
 * C/C++ source code generated on  : 10-Oct-2024 14:36:29
 */

/*************************************************************************/
/* This automatically generated example C main file shows how to call    */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

/* Include Files */
#include "main.h"
#include "leg_conv.h"
#include "leg_conv_terminate.h"
#include "leg_pos.h"
#include "leg_spd.h"
#include "lqr_k.h"
#include "rt_nonfinite.h"

/* Function Declarations */
static float argInit_real32_T(void);

/* Function Definitions */
/*
 * Arguments    : void
 * Return Type  : float
 */
static float argInit_real32_T(void)
{
  return 0.0F;
}

/*
 * Arguments    : int argc
 *                char **argv
 * Return Type  : int
 */
int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  /* The initialize function is being called automatically from your entry-point
   * function. So, a call to initialize is not included here. */
  /* Invoke the entry-point functions.
You can call entry-point functions multiple times. */
  main_leg_conv();
  main_leg_pos();
  main_leg_spd();
  main_lqr_k();
  /* Terminate the application.
You do not need to do this more than one time. */
  leg_conv_terminate();
  return 0;
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_leg_conv(void)
{
  float T[2];
  float F_tmp;
  /* Initialize function 'leg_conv' input arguments. */
  F_tmp = argInit_real32_T();
  /* Call the entry-point 'leg_conv'. */
  leg_conv(F_tmp, F_tmp, F_tmp, F_tmp, T);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_leg_pos(void)
{
  float pos[2];
  float phi1_tmp;
  /* Initialize function 'leg_pos' input arguments. */
  phi1_tmp = argInit_real32_T();
  /* Call the entry-point 'leg_pos'. */
  leg_pos(phi1_tmp, phi1_tmp, pos);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_leg_spd(void)
{
  float spd[2];
  float dphi1_tmp;
  /* Initialize function 'leg_spd' input arguments. */
  dphi1_tmp = argInit_real32_T();
  /* Call the entry-point 'leg_spd'. */
  leg_spd(dphi1_tmp, dphi1_tmp, dphi1_tmp, dphi1_tmp, spd);
}

/*
 * Arguments    : void
 * Return Type  : void
 */
void main_lqr_k(void)
{
  float K[12];
  /* Initialize function 'lqr_k' input arguments. */
  /* Call the entry-point 'lqr_k'. */
  lqr_k(argInit_real32_T(), K);
}

/*
 * File trailer for main.c
 *
 * [EOF]
 */
