//
// File: hdbiaaaabiecmgdb_do_vectors.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Dec  4 08:34:45 2017
//
#include "rtwtypes.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include <math.h>
#include "kngdbaaaiecjbiec_skip_to_last_equal_value.h"
#include "mgdjimgdopphcjek_skip_to_last_equal_value.h"
#include "hdbiaaaabiecmgdb_do_vectors.h"

// Function for MATLAB Function: '<S83>/generate_output'
void hdbiaaaabiecmgdb_do_vectors(const real32_T a[50], const real32_T b_data[],
  const int32_T b_sizes, real32_T c_data[], int32_T *c_sizes, int32_T ia_data[],
  int32_T *ia_sizes, int32_T ib_data[], int32_T *ib_sizes)
{
  int32_T nc;
  int32_T iafirst;
  int32_T ialast;
  int32_T ibfirst;
  int32_T iblast;
  int32_T b_ialast;
  real32_T ak;
  int32_T b_iblast;
  real32_T bk;
  boolean_T p;
  real32_T absxk;
  int32_T exponent;
  int32_T ia_data_0[50];
  real32_T c_data_0[50];
  if (50 <= b_sizes) {
    *ia_sizes = 50;
  } else {
    *ia_sizes = b_sizes;
  }

  *c_sizes = (int32_T)(int8_T)*ia_sizes;
  *ib_sizes = *ia_sizes;
  nc = 0;
  iafirst = 0;
  ialast = 1;
  ibfirst = 0;
  iblast = 1;
  while ((ialast <= 50) && (iblast <= b_sizes)) {
    b_ialast = ialast;
    ak = kngdbaaaiecjbiec_skip_to_last_equal_value(&b_ialast, a);
    ialast = b_ialast;
    b_iblast = iblast;
    bk = mgdjimgdopphcjek_skip_to_last_equal_value(&b_iblast, b_data, b_sizes);
    iblast = b_iblast;
    absxk = (real32_T)fabs((real_T)(bk / 2.0F));
    if ((!rtIsInfF(absxk)) && (!rtIsNaNF(absxk))) {
      if (absxk <= 1.17549435E-38F) {
        absxk = 1.4013E-45F;
      } else {
        frexp((real_T)absxk, &exponent);
        absxk = (real32_T)ldexp((real_T)1.0F, (int32_T)(exponent - 24));
      }
    } else {
      absxk = (rtNaNF);
    }

    if (((real32_T)fabs((real_T)(bk - ak)) < absxk) || (rtIsInfF(ak) && rtIsInfF
         (bk) && ((ak > 0.0F) == (bk > 0.0F)))) {
      p = true;
    } else {
      p = false;
    }

    if (p) {
      nc++;
      c_data[(int32_T)(nc - 1)] = ak;
      ia_data[(int32_T)(nc - 1)] = (int32_T)(iafirst + 1);
      ib_data[(int32_T)(nc - 1)] = (int32_T)(ibfirst + 1);
      ialast = (int32_T)(b_ialast + 1);
      iafirst = b_ialast;
      iblast = (int32_T)(b_iblast + 1);
      ibfirst = b_iblast;
    } else {
      if ((ak < bk) || rtIsNaNF(bk)) {
        p = true;
      }

      if (p) {
        ialast = (int32_T)(b_ialast + 1);
        iafirst = b_ialast;
      } else {
        iblast = (int32_T)(b_iblast + 1);
        ibfirst = b_iblast;
      }
    }
  }

  if (*ia_sizes > 0) {
    if (1 > nc) {
      *ia_sizes = 0;
    } else {
      *ia_sizes = nc;
    }

    for (iafirst = 0; iafirst <= (int32_T)(*ia_sizes - 1); iafirst++) {
      ia_data_0[iafirst] = ia_data[iafirst];
    }

    for (iafirst = 0; iafirst <= (int32_T)(*ia_sizes - 1); iafirst++) {
      ia_data[iafirst] = ia_data_0[iafirst];
    }

    if (1 > nc) {
      *ib_sizes = 0;
    } else {
      *ib_sizes = nc;
    }

    for (iafirst = 0; iafirst <= (int32_T)(*ib_sizes - 1); iafirst++) {
      ia_data_0[iafirst] = ib_data[iafirst];
    }

    for (iafirst = 0; iafirst <= (int32_T)(*ib_sizes - 1); iafirst++) {
      ib_data[iafirst] = ia_data_0[iafirst];
    }

    if (1 > nc) {
      nc = 0;
    }

    for (iafirst = 0; iafirst <= (int32_T)(nc - 1); iafirst++) {
      c_data_0[iafirst] = c_data[iafirst];
    }

    *c_sizes = nc;
    for (iafirst = 0; iafirst <= (int32_T)(nc - 1); iafirst++) {
      c_data[iafirst] = c_data_0[iafirst];
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
