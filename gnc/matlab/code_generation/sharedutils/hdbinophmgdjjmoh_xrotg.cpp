//
// File: hdbinophmgdjjmoh_xrotg.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Dec  4 08:33:06 2017
//
#include "rtwtypes.h"
#include <math.h>
#include "hdbinophmgdjjmoh_xrotg.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void hdbinophmgdjjmoh_xrotg(real32_T *a, real32_T *b, real32_T *c, real32_T *s)
{
  real32_T roe;
  real32_T absa;
  real32_T absb;
  real32_T scale;
  real32_T ads;
  real32_T bds;
  roe = *b;
  absa = (real32_T)fabs((real_T)*a);
  absb = (real32_T)fabs((real_T)*b);
  if (absa > absb) {
    roe = *a;
  }

  scale = absa + absb;
  if (scale == 0.0F) {
    *s = 0.0F;
    *c = 1.0F;
    scale = 0.0F;
    absa = 0.0F;
  } else {
    ads = absa / scale;
    bds = absb / scale;
    scale *= (real32_T)sqrt((real_T)(ads * ads + bds * bds));
    if (roe < 0.0F) {
      scale = -scale;
    }

    *c = *a / scale;
    *s = *b / scale;
    if (absa > absb) {
      absa = *s;
    } else if (*c != 0.0F) {
      absa = 1.0F / *c;
    } else {
      absa = 1.0F;
    }
  }

  *a = scale;
  *b = absa;
}

//
// File trailer for generated code.
//
// [EOF]
//
