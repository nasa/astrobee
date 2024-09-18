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
static const int CASADI_PREFIX(s2)[23] = {4, 4, 0, 4, 8, 12, 16, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3};
#define s2 CASADI_PREFIX(s2)
/* jacFun */
CASADI_SYMBOL_EXPORT int jacFun(const real_t** arg, real_t** res, int* iw, real_t* w, int mem) {
  real_t a0=arg[0] ? arg[0][2] : 0;
  if (res[0]!=0) res[0][0]=a0;
  a0=arg[0] ? arg[0][3] : 0;
  if (res[0]!=0) res[0][1]=a0;
  real_t a1=arg[0] ? arg[0][1] : 0;
  real_t a2=sin(a1);
  real_t a3=-8.0000000000000016e-02;
  a2=(a3*a2);
  real_t a4=sq(a0);
  real_t a5=(a2*a4);
  real_t a6=arg[1] ? arg[1][0] : 0;
  a5=(a5+a6);
  real_t a7=cos(a1);
  real_t a8=9.8100000000000009e-01;
  a7=(a8*a7);
  real_t a9=sin(a1);
  real_t a10=(a7*a9);
  a5=(a5+a10);
  a10=cos(a1);
  real_t a11=sq(a10);
  real_t a12=1.0000000000000001e-01;
  a11=(a12*a11);
  real_t a13=1.1000000000000001e+00;
  a11=(a13-a11);
  a5=(a5/a11);
  if (res[0]!=0) res[0][2]=a5;
  real_t a14=cos(a1);
  a14=(a3*a14);
  real_t a15=sin(a1);
  real_t a16=(a14*a15);
  real_t a17=sq(a0);
  real_t a18=(a16*a17);
  real_t a19=cos(a1);
  a19=(a6*a19);
  a18=(a18+a19);
  a19=sin(a1);
  a19=(a8*a19);
  a18=(a18+a19);
  a19=sin(a1);
  real_t a20=9.8100000000000005e+00;
  a19=(a20*a19);
  a18=(a18+a19);
  a19=cos(a1);
  real_t a21=sq(a19);
  a21=(a12*a21);
  a13=(a13-a21);
  a21=8.0000000000000004e-01;
  a13=(a21*a13);
  a18=(a18/a13);
  if (res[0]!=0) res[0][3]=a18;
  real_t a22=0.;
  if (res[1]!=0) res[1][0]=a22;
  if (res[1]!=0) res[1][1]=a22;
  if (res[1]!=0) res[1][2]=a22;
  if (res[1]!=0) res[1][3]=a22;
  if (res[1]!=0) res[1][4]=a22;
  if (res[1]!=0) res[1][5]=a22;
  real_t a23=cos(a1);
  a23=(a3*a23);
  a4=(a4*a23);
  a23=cos(a1);
  a7=(a7*a23);
  a23=sin(a1);
  a23=(a8*a23);
  a9=(a9*a23);
  a7=(a7-a9);
  a4=(a4+a7);
  a4=(a4/a11);
  a5=(a5/a11);
  a10=(a10+a10);
  a7=sin(a1);
  a10=(a10*a7);
  a10=(a12*a10);
  a5=(a5*a10);
  a4=(a4-a5);
  if (res[1]!=0) res[1][6]=a4;
  a4=cos(a1);
  a14=(a14*a4);
  a4=sin(a1);
  a3=(a3*a4);
  a15=(a15*a3);
  a14=(a14-a15);
  a17=(a17*a14);
  a14=sin(a1);
  a6=(a6*a14);
  a17=(a17-a6);
  a6=cos(a1);
  a8=(a8*a6);
  a17=(a17+a8);
  a8=cos(a1);
  a20=(a20*a8);
  a17=(a17+a20);
  a17=(a17/a13);
  a18=(a18/a13);
  a19=(a19+a19);
  a1=sin(a1);
  a19=(a19*a1);
  a12=(a12*a19);
  a21=(a21*a12);
  a18=(a18*a21);
  a17=(a17-a18);
  if (res[1]!=0) res[1][7]=a17;
  a17=1.;
  if (res[1]!=0) res[1][8]=a17;
  if (res[1]!=0) res[1][9]=a22;
  if (res[1]!=0) res[1][10]=a22;
  if (res[1]!=0) res[1][11]=a22;
  if (res[1]!=0) res[1][12]=a22;
  if (res[1]!=0) res[1][13]=a17;
  a17=(a0+a0);
  a2=(a2*a17);
  a2=(a2/a11);
  if (res[1]!=0) res[1][14]=a2;
  a0=(a0+a0);
  a16=(a16*a0);
  a16=(a16/a13);
  if (res[1]!=0) res[1][15]=a16;
  return 0;
}

CASADI_SYMBOL_EXPORT void jacFun_incref(void) {
}

CASADI_SYMBOL_EXPORT void jacFun_decref(void) {
}

CASADI_SYMBOL_EXPORT int jacFun_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT int jacFun_n_out(void) { return 2;}

CASADI_SYMBOL_EXPORT const char* jacFun_name_in(int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* jacFun_name_out(int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const int* jacFun_sparsity_in(int i) {
  switch (i) {
    case 0: return s0;
    case 1: return s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const int* jacFun_sparsity_out(int i) {
  switch (i) {
    case 0: return s0;
    case 1: return s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int jacFun_work(int *sz_arg, int* sz_res, int *sz_iw, int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 24;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
