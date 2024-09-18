/**************************************************************************************************
*                                                                                                 *
* This file is part of HPIPM.                                                                     *
*                                                                                                 *
* HPIPM -- High-Performance Interior Point Method.                                                *
* Copyright (C) 2017-2018 by Gianluca Frison.                                                     *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* This program is free software: you can redistribute it and/or modify                            *
* it under the terms of the GNU General Public License as published by                            *
* the Free Software Foundation, either version 3 of the License, or                               *
* (at your option) any later version                                                              *.
*                                                                                                 *
* This program is distributed in the hope that it will be useful,                                 *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                                  *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                                   *
* GNU General Public License for more details.                                                    *
*                                                                                                 *
* You should have received a copy of the GNU General Public License                               *
* along with this program.  If not, see <https://www.gnu.org/licenses/>.                          *
*                                                                                                 *
* The authors designate this particular file as subject to the "Classpath" exception              *
* as provided by the authors in the LICENSE file that accompained this code.                      *
*                                                                                                 *
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/
#define fmin(x,y) CASADI_PREFIX(fmin)(x,y)
real_t CASADI_PREFIX(fmax)(real_t x, real_t y) { return x>y ? x : y;}
#define fmax(x,y) CASADI_PREFIX(fmax)(x,y)
#endif

#define PRINTF printf
#ifndef CASADI_SYMBOL_EXPORT
#if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
#if defined(STATIC_LINKED)
#define CASADI_SYMBOL_EXPORT
#else /* defined(STATIC_LINKED) */
#define CASADI_SYMBOL_EXPORT __declspec(dllexport)
#endif /* defined(STATIC_LINKED) */
#elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
#define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
#else /* defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__) */
#define CASADI_SYMBOL_EXPORT
#endif /* defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__) */
#endif /* CASADI_SYMBOL_EXPORT */
real_t CASADI_PREFIX(sq)(real_t x) { return x*x;}
#define sq(x) CASADI_PREFIX(sq)(x)

real_t CASADI_PREFIX(sign)(real_t x) { return x<0 ? -1 : x>0 ? 1 : x;}
#define sign(x) CASADI_PREFIX(sign)(x)

static const int CASADI_PREFIX(s0)[8] = {4, 1, 0, 4, 0, 1, 2, 3};
#define s0 CASADI_PREFIX(s0)
static const int CASADI_PREFIX(s1)[5] = {1, 1, 0, 1, 0};
#define s1 CASADI_PREFIX(s1)
/* odeFun */
CASADI_SYMBOL_EXPORT int odeFun(const real_t** arg, real_t** res, int* iw, real_t* w, int mem) {
  real_t a0=arg[0] ? arg[0][2] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[0] ? arg[0][3] : 0;
  if (res[0]!=0) res[0][1]=a0;
  real_t a1=arg[0] ? arg[0][1] : 0;
  real_t a2=sin(a1);
  real_t a3=-8.0000000000000016e-02;
  a2=(a3*a2);
  real_t a4=sq(a0);
  a2=(a2*a4);
  a4=arg[1] ? arg[1][0] : 0;
  a2=(a2+a4);
  real_t a5=cos(a1);
  real_t a6=9.8100000000000009e-01;
  a5=(a6*a5);
  real_t a7=sin(a1);
  a5=(a5*a7);
  a2=(a2+a5);
  a5=cos(a1);
  a5=sq(a5);
  a7=1.0000000000000001e-01;
  a5=(a7*a5);
  real_t a8=1.1000000000000001e+00;
  a5=(a8-a5);
  a2=(a2/a5);
  if (res[0]!=0) res[0][2]=a2;
  a2=cos(a1);
  a3=(a3*a2);
  a2=sin(a1);
  a3=(a3*a2);
  a0=sq(a0);
  a3=(a3*a0);
  a0=cos(a1);
  a4=(a4*a0);
  a3=(a3+a4);
  a4=sin(a1);
  a6=(a6*a4);
  a3=(a3+a6);
  a6=sin(a1);
  a4=9.8100000000000005e+00;
  a4=(a4*a6);
  a3=(a3+a4);
  a1=cos(a1);
  a1=sq(a1);
  a7=(a7*a1);
  a8=(a8-a7);
  a7=8.0000000000000004e-01;
  a7=(a7*a8);
  a3=(a3/a7);
  if (res[0]!=0) res[0][3]=a3;
  return 0;
}

CASADI_SYMBOL_EXPORT void odeFun_incref(void) {
}

CASADI_SYMBOL_EXPORT void odeFun_decref(void) {
}

CASADI_SYMBOL_EXPORT int odeFun_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT int odeFun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT const char* odeFun_name_in(int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* odeFun_name_out(int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const int* odeFun_sparsity_in(int i) {
  switch (i) {
    case 0: return s0;
    case 1: return s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const int* odeFun_sparsity_out(int i) {
  switch (i) {
    case 0: return s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int odeFun_work(int *sz_arg, int* sz_res, int *sz_iw, int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 9;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
