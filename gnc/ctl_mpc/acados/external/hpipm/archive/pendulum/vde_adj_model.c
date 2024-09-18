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
static const int CASADI_PREFIX(s2)[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
#define s2 CASADI_PREFIX(s2)
/* adjFun */
CASADI_SYMBOL_EXPORT int adjFun(const real_t** arg, real_t** res, int* iw, real_t* w, int mem) {
  real_t a0=arg[0] ? arg[0][1] : 0;
  real_t a1=cos(a0);
  real_t a2=sq(a1);
  real_t a3=1.0000000000000001e-01;
  a2=(a3*a2);
  real_t a4=1.1000000000000001e+00;
  a2=(a4-a2);
  real_t a5=8.0000000000000004e-01;
  a2=(a5*a2);
  real_t a6=arg[1] ? arg[1][3] : 0;
  real_t a7=(a6/a2);
  real_t a8=cos(a0);
  real_t a9=(a8*a7);
  real_t a10=cos(a0);
  real_t a11=sq(a10);
  a11=(a3*a11);
  a4=(a4-a11);
  a11=arg[1] ? arg[1][2] : 0;
  real_t a12=(a11/a4);
  a9=(a9+a12);
  if (res[0]!=0) res[0][0]=a9;
  a9=0.;
  if (res[0]!=0) res[0][1]=a9;
  a9=9.8100000000000005e+00;
  real_t a13=(a9*a7);
  real_t a14=cos(a0);
  a14=(a14*a13);
  a1=(a1+a1);
  a13=cos(a0);
  real_t a15=-8.0000000000000016e-02;
  a13=(a15*a13);
  real_t a16=sin(a0);
  real_t a17=(a13*a16);
  real_t a18=arg[0] ? arg[0][3] : 0;
  real_t a19=sq(a18);
  real_t a20=(a17*a19);
  real_t a21=arg[2] ? arg[2][0] : 0;
  a8=(a21*a8);
  a20=(a20+a8);
  a8=sin(a0);
  real_t a22=9.8100000000000009e-01;
  a8=(a22*a8);
  a20=(a20+a8);
  a8=sin(a0);
  a9=(a9*a8);
  a20=(a20+a9);
  a20=(a20/a2);
  a20=(a20/a2);
  a20=(a20*a6);
  a5=(a5*a20);
  a5=(a3*a5);
  a1=(a1*a5);
  a5=sin(a0);
  a5=(a5*a1);
  a14=(a14-a5);
  a5=(a22*a7);
  a1=cos(a0);
  a1=(a1*a5);
  a14=(a14+a1);
  a1=(a21*a7);
  a5=sin(a0);
  a5=(a5*a1);
  a14=(a14-a5);
  a19=(a19*a7);
  a13=(a13*a19);
  a5=cos(a0);
  a5=(a5*a13);
  a14=(a14+a5);
  a16=(a16*a19);
  a16=(a15*a16);
  a19=sin(a0);
  a19=(a19*a16);
  a14=(a14-a19);
  a10=(a10+a10);
  a19=sin(a0);
  a19=(a15*a19);
  a16=sq(a18);
  a5=(a19*a16);
  a5=(a5+a21);
  a21=cos(a0);
  a21=(a22*a21);
  a13=sin(a0);
  a1=(a21*a13);
  a5=(a5+a1);
  a5=(a5/a4);
  a5=(a5/a4);
  a5=(a5*a11);
  a3=(a3*a5);
  a10=(a10*a3);
  a3=sin(a0);
  a3=(a3*a10);
  a14=(a14-a3);
  a21=(a21*a12);
  a3=cos(a0);
  a3=(a3*a21);
  a14=(a14+a3);
  a13=(a13*a12);
  a22=(a22*a13);
  a13=sin(a0);
  a13=(a13*a22);
  a14=(a14-a13);
  a16=(a16*a12);
  a15=(a15*a16);
  a0=cos(a0);
  a0=(a0*a15);
  a14=(a14+a0);
  if (res[0]!=0) res[0][2]=a14;
  a14=arg[1] ? arg[1][0] : 0;
  if (res[0]!=0) res[0][3]=a14;
  a14=(a18+a18);
  a17=(a17*a7);
  a14=(a14*a17);
  a18=(a18+a18);
  a19=(a19*a12);
  a18=(a18*a19);
  a14=(a14+a18);
  a18=arg[1] ? arg[1][1] : 0;
  a14=(a14+a18);
  if (res[0]!=0) res[0][4]=a14;
  return 0;
}

CASADI_SYMBOL_EXPORT void adjFun_incref(void) {
}

CASADI_SYMBOL_EXPORT void adjFun_decref(void) {
}

CASADI_SYMBOL_EXPORT int adjFun_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT int adjFun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT const char* adjFun_name_in(int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* adjFun_name_out(int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const int* adjFun_sparsity_in(int i) {
  switch (i) {
    case 0: return s0;
    case 1: return s0;
    case 2: return s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const int* adjFun_sparsity_out(int i) {
  switch (i) {
    case 0: return s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int adjFun_work(int *sz_arg, int* sz_res, int *sz_iw, int *sz_w) {
  if (sz_arg) *sz_arg = 3;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 23;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
