//
// File: phdjbaaiglngbaaa_xzlarfg.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 11:58:17 2018
//
#include "rtwtypes.h"
#include <math.h>
#include "mglnnophaaimmgln_xnrm2.h"
#include "rt_hypotf_snf.h"
#include "phdjbaaiglngbaaa_xzlarfg.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
real32_T phdjbaaiglngbaaa_xzlarfg(int32_T n, real32_T *alpha1, real32_T x_data[],
  int32_T ix0)
{
  real32_T tau;
  real32_T xnorm;
  int32_T knt;
  int32_T b_k;
  int32_T c_k;
  tau = 0.0F;
  if (!(n <= 0)) {
    xnorm = mglnnophaaimmgln_xnrm2((int32_T)(n - 1), x_data, ix0);
    if (xnorm != 0.0F) {
      xnorm = rt_hypotf_snf(*alpha1, xnorm);
      if (*alpha1 >= 0.0F) {
        xnorm = -xnorm;
      }

      if ((real32_T)fabs((real_T)xnorm) < 9.86076132E-32F) {
        knt = 0;
        do {
          knt++;
          b_k = (int32_T)((int32_T)(ix0 + n) - 2);
          for (c_k = ix0; c_k <= b_k; c_k++) {
            x_data[(int32_T)(c_k - 1)] *= 1.01412048E+31F;
          }

          xnorm *= 1.01412048E+31F;
          *alpha1 *= 1.01412048E+31F;
        } while (!((real32_T)fabs((real_T)xnorm) >= 9.86076132E-32F));

        xnorm = rt_hypotf_snf(*alpha1, mglnnophaaimmgln_xnrm2((int32_T)(n - 1),
          x_data, ix0));
        if (*alpha1 >= 0.0F) {
          xnorm = -xnorm;
        }

        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0F / (*alpha1 - xnorm);
        b_k = (int32_T)((int32_T)(ix0 + n) - 2);
        for (c_k = ix0; c_k <= b_k; c_k++) {
          x_data[(int32_T)(c_k - 1)] *= *alpha1;
        }

        for (b_k = 1; b_k <= knt; b_k++) {
          xnorm *= 9.86076132E-32F;
        }

        *alpha1 = xnorm;
      } else {
        tau = (xnorm - *alpha1) / xnorm;
        *alpha1 = 1.0F / (*alpha1 - xnorm);
        knt = (int32_T)((int32_T)(ix0 + n) - 2);
        for (b_k = ix0; b_k <= knt; b_k++) {
          x_data[(int32_T)(b_k - 1)] *= *alpha1;
        }

        *alpha1 = xnorm;
      }
    }
  }

  return tau;
}

//
// File trailer for generated code.
//
// [EOF]
//
