//
// File: aimgnophaiekjecb_eml_rand_mt19937ar.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed May 17 14:43:55 2017
//
#include "rtwtypes.h"
#include "aimgnophaiekjecb_eml_rand_mt19937ar.h"

// Function for MATLAB Function: '<S47>/generate_output'
real_T aimgnophaiekjecb_eml_rand_mt19937ar(uint32_T state[625])
{
  real_T r;
  uint32_T u[2];
  uint32_T y;
  int32_T kk;
  int32_T k;
  boolean_T b_isvalid;
  uint32_T c_r;
  int32_T exitg1;
  boolean_T exitg2;

  // ========================= COPYRIGHT NOTICE ============================
  //  This is a uniform (0,1) pseudorandom number generator based on:
  //
  //  A C-program for MT19937, with initialization improved 2002/1/26.
  //  Coded by Takuji Nishimura and Makoto Matsumoto.
  //
  //  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura,
  //  All rights reserved.
  //
  //  Redistribution and use in source and binary forms, with or without
  //  modification, are permitted provided that the following conditions
  //  are met:
  //
  //    1. Redistributions of source code must retain the above copyright
  //       notice, this list of conditions and the following disclaimer.
  //
  //    2. Redistributions in binary form must reproduce the above copyright
  //       notice, this list of conditions and the following disclaimer
  //       in the documentation and/or other materials provided with the
  //       distribution.
  //
  //    3. The names of its contributors may not be used to endorse or
  //       promote products derived from this software without specific
  //       prior written permission.
  //
  //  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  //  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  //  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  //  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT
  //  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  //  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  //  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  //  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  //  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  //  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  //  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  //
  // =============================   END   =================================
  do {
    exitg1 = 0;
    for (k = 0; k < 2; k++) {
      c_r = (uint32_T)(state[624] + 1U);
      if (c_r >= 625U) {
        for (kk = 0; kk < 227; kk++) {
          y = (uint32_T)((uint32_T)(state[(int32_T)(1 + kk)] & 2147483647U) |
                         (uint32_T)(state[kk] & 2147483648U));
          if ((int32_T)(uint32_T)(y & 1U) == 0) {
            y >>= 1U;
          } else {
            y = (uint32_T)((uint32_T)(y >> 1U) ^ 2567483615U);
          }

          state[kk] = (uint32_T)(state[(int32_T)(397 + kk)] ^ y);
        }

        for (kk = 0; kk < 396; kk++) {
          y = (uint32_T)((uint32_T)(state[(int32_T)(kk + 227)] & 2147483648U) |
                         (uint32_T)(state[(int32_T)(228 + kk)] & 2147483647U));
          if ((int32_T)(uint32_T)(y & 1U) == 0) {
            y >>= 1U;
          } else {
            y = (uint32_T)((uint32_T)(y >> 1U) ^ 2567483615U);
          }

          state[(int32_T)(kk + 227)] = (uint32_T)(state[kk] ^ y);
        }

        y = (uint32_T)((uint32_T)(state[623] & 2147483648U) | (uint32_T)(state[0]
          & 2147483647U));
        if ((int32_T)(uint32_T)(y & 1U) == 0) {
          y >>= 1U;
        } else {
          y = (uint32_T)((uint32_T)(y >> 1U) ^ 2567483615U);
        }

        state[623] = (uint32_T)(state[396] ^ y);
        c_r = 1U;
      }

      y = state[(int32_T)((int32_T)c_r - 1)];
      state[624] = c_r;
      y ^= (uint32_T)(y >> 11U);
      y ^= (uint32_T)((uint32_T)(y << 7U) & 2636928640U);
      y ^= (uint32_T)((uint32_T)(y << 15U) & 4022730752U);
      y ^= (uint32_T)(y >> 18U);
      u[k] = y;
    }

    r = ((real_T)(uint32_T)(u[0] >> 5U) * 6.7108864E+7 + (real_T)(uint32_T)(u[1]
          >> 6U)) * 1.1102230246251565E-16;
    if (r == 0.0) {
      if ((state[624] >= 1U) && (state[624] < 625U)) {
        b_isvalid = true;
      } else {
        b_isvalid = false;
      }

      if (b_isvalid) {
        b_isvalid = false;
        k = 1;
        exitg2 = false;
        while ((!exitg2) && (k < 625)) {
          if (state[(int32_T)(k - 1)] == 0U) {
            k++;
          } else {
            b_isvalid = true;
            exitg2 = true;
          }
        }
      }

      if (!b_isvalid) {
        c_r = 5489U;
        state[0] = 5489U;
        for (k = 0; k < 623; k++) {
          c_r = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(c_r >>
            30U) ^ c_r) * 1812433253U) + (uint32_T)k) + 1U);
          state[(int32_T)(k + 1)] = c_r;
        }

        state[624] = 624U;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  return r;
}

//
// File trailer for generated code.
//
// [EOF]
//
