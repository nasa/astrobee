//
// File: imohknglphlfpphd_repmat.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed May 17 14:42:31 2017
//
#include "rtwtypes.h"
#include "imohknglphlfpphd_repmat.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void imohknglphlfpphd_repmat(real32_T b[256])
{
  int32_T iacol;
  int32_T ibmat;
  int32_T ibcol;
  int32_T jcol;
  int32_T itilerow;
  static const int8_T c[16] = { 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1 };

  for (jcol = 0; jcol < 4; jcol++) {
    iacol = (int32_T)(jcol << 2);
    ibmat = (int32_T)(jcol << 6);
    for (itilerow = 0; itilerow < 16; itilerow++) {
      ibcol = (int32_T)((int32_T)(itilerow << 2) + ibmat);
      b[ibcol] = (real32_T)c[iacol];
      b[(int32_T)(ibcol + 1)] = (real32_T)c[(int32_T)(iacol + 1)];
      b[(int32_T)(ibcol + 2)] = (real32_T)c[(int32_T)(iacol + 2)];
      b[(int32_T)(ibcol + 3)] = (real32_T)c[(int32_T)(iacol + 3)];
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
