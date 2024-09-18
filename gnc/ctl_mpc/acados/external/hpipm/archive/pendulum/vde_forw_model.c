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
static const int CASADI_PREFIX(s1)[23] = {4, 4, 0, 4, 8, 12, 16, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3, 0, 1, 2, 3};
#define s1 CASADI_PREFIX(s1)
static const int CASADI_PREFIX(s2)[5] = {1, 1, 0, 1, 0};
#define s2 CASADI_PREFIX(s2)
/* vdeFun */
CASADI_SYMBOL_EXPORT int vdeFun(const real_t** arg, real_t** res, int* iw, real_t* w, int mem) {
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
  real_t a6=arg[3] ? arg[3][0] : 0;
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
  real_t a20=(a6*a19);
  a18=(a18+a20);
  a20=sin(a1);
  a20=(a8*a20);
  a18=(a18+a20);
  a20=sin(a1);
  real_t a21=9.8100000000000005e+00;
  a20=(a21*a20);
  a18=(a18+a20);
  a20=cos(a1);
  real_t a22=sq(a20);
  a22=(a12*a22);
  a13=(a13-a22);
  a22=8.0000000000000004e-01;
  a13=(a22*a13);
  a18=(a18/a13);
  if (res[0]!=0) res[0][3]=a18;
  real_t a23=arg[1] ? arg[1][2] : 0;
  if (res[1]!=0) res[1][0]=a23;
  a23=arg[1] ? arg[1][3] : 0;
  if (res[1]!=0) res[1][1]=a23;
  real_t a24=cos(a1);
  real_t a25=arg[1] ? arg[1][1] : 0;
  real_t a26=(a24*a25);
  a26=(a3*a26);
  a26=(a4*a26);
  real_t a27=(a0+a0);
  real_t a28=(a27*a23);
  a28=(a2*a28);
  a26=(a26+a28);
  a28=cos(a1);
  real_t a29=(a28*a25);
  a29=(a7*a29);
  real_t a30=sin(a1);
  real_t a31=(a30*a25);
  a31=(a8*a31);
  a31=(a9*a31);
  a29=(a29-a31);
  a26=(a26+a29);
  a26=(a26/a11);
  a29=(a5/a11);
  a31=(a10+a10);
  real_t a32=sin(a1);
  real_t a33=(a32*a25);
  a33=(a31*a33);
  a33=(a12*a33);
  a33=(a29*a33);
  a26=(a26-a33);
  if (res[1]!=0) res[1][2]=a26;
  a26=cos(a1);
  a33=(a26*a25);
  a33=(a14*a33);
  real_t a34=sin(a1);
  real_t a35=(a34*a25);
  a35=(a3*a35);
  a35=(a15*a35);
  a33=(a33-a35);
  a33=(a17*a33);
  a35=(a0+a0);
  a23=(a35*a23);
  a23=(a16*a23);
  a33=(a33+a23);
  a23=sin(a1);
  real_t a36=(a23*a25);
  a36=(a6*a36);
  a33=(a33-a36);
  a36=cos(a1);
  real_t a37=(a36*a25);
  a37=(a8*a37);
  a33=(a33+a37);
  a37=cos(a1);
  real_t a38=(a37*a25);
  a38=(a21*a38);
  a33=(a33+a38);
  a33=(a33/a13);
  a38=(a18/a13);
  real_t a39=(a20+a20);
  real_t a40=sin(a1);
  a25=(a40*a25);
  a25=(a39*a25);
  a25=(a12*a25);
  a25=(a22*a25);
  a25=(a38*a25);
  a33=(a33-a25);
  if (res[1]!=0) res[1][3]=a33;
  a33=arg[1] ? arg[1][6] : 0;
  if (res[1]!=0) res[1][4]=a33;
  a33=arg[1] ? arg[1][7] : 0;
  if (res[1]!=0) res[1][5]=a33;
  a25=arg[1] ? arg[1][5] : 0;
  real_t a41=(a24*a25);
  a41=(a3*a41);
  a41=(a4*a41);
  real_t a42=(a27*a33);
  a42=(a2*a42);
  a41=(a41+a42);
  a42=(a28*a25);
  a42=(a7*a42);
  real_t a43=(a30*a25);
  a43=(a8*a43);
  a43=(a9*a43);
  a42=(a42-a43);
  a41=(a41+a42);
  a41=(a41/a11);
  a42=(a32*a25);
  a42=(a31*a42);
  a42=(a12*a42);
  a42=(a29*a42);
  a41=(a41-a42);
  if (res[1]!=0) res[1][6]=a41;
  a41=(a26*a25);
  a41=(a14*a41);
  a42=(a34*a25);
  a42=(a3*a42);
  a42=(a15*a42);
  a41=(a41-a42);
  a41=(a17*a41);
  a33=(a35*a33);
  a33=(a16*a33);
  a41=(a41+a33);
  a33=(a23*a25);
  a33=(a6*a33);
  a41=(a41-a33);
  a33=(a36*a25);
  a33=(a8*a33);
  a41=(a41+a33);
  a33=(a37*a25);
  a33=(a21*a33);
  a41=(a41+a33);
  a41=(a41/a13);
  a25=(a40*a25);
  a25=(a39*a25);
  a25=(a12*a25);
  a25=(a22*a25);
  a25=(a38*a25);
  a41=(a41-a25);
  if (res[1]!=0) res[1][7]=a41;
  a41=arg[1] ? arg[1][10] : 0;
  if (res[1]!=0) res[1][8]=a41;
  a41=arg[1] ? arg[1][11] : 0;
  if (res[1]!=0) res[1][9]=a41;
  a25=arg[1] ? arg[1][9] : 0;
  a33=(a24*a25);
  a33=(a3*a33);
  a33=(a4*a33);
  a42=(a27*a41);
  a42=(a2*a42);
  a33=(a33+a42);
  a42=(a28*a25);
  a42=(a7*a42);
  a43=(a30*a25);
  a43=(a8*a43);
  a43=(a9*a43);
  a42=(a42-a43);
  a33=(a33+a42);
  a33=(a33/a11);
  a42=(a32*a25);
  a42=(a31*a42);
  a42=(a12*a42);
  a42=(a29*a42);
  a33=(a33-a42);
  if (res[1]!=0) res[1][10]=a33;
  a33=(a26*a25);
  a33=(a14*a33);
  a42=(a34*a25);
  a42=(a3*a42);
  a42=(a15*a42);
  a33=(a33-a42);
  a33=(a17*a33);
  a41=(a35*a41);
  a41=(a16*a41);
  a33=(a33+a41);
  a41=(a23*a25);
  a41=(a6*a41);
  a33=(a33-a41);
  a41=(a36*a25);
  a41=(a8*a41);
  a33=(a33+a41);
  a41=(a37*a25);
  a41=(a21*a41);
  a33=(a33+a41);
  a33=(a33/a13);
  a25=(a40*a25);
  a25=(a39*a25);
  a25=(a12*a25);
  a25=(a22*a25);
  a25=(a38*a25);
  a33=(a33-a25);
  if (res[1]!=0) res[1][11]=a33;
  a33=arg[1] ? arg[1][14] : 0;
  if (res[1]!=0) res[1][12]=a33;
  a33=arg[1] ? arg[1][15] : 0;
  if (res[1]!=0) res[1][13]=a33;
  a25=arg[1] ? arg[1][13] : 0;
  a24=(a24*a25);
  a24=(a3*a24);
  a24=(a4*a24);
  a27=(a27*a33);
  a27=(a2*a27);
  a24=(a24+a27);
  a28=(a28*a25);
  a28=(a7*a28);
  a30=(a30*a25);
  a30=(a8*a30);
  a30=(a9*a30);
  a28=(a28-a30);
  a24=(a24+a28);
  a24=(a24/a11);
  a32=(a32*a25);
  a31=(a31*a32);
  a31=(a12*a31);
  a29=(a29*a31);
  a24=(a24-a29);
  if (res[1]!=0) res[1][14]=a24;
  a26=(a26*a25);
  a26=(a14*a26);
  a34=(a34*a25);
  a34=(a3*a34);
  a34=(a15*a34);
  a26=(a26-a34);
  a26=(a17*a26);
  a35=(a35*a33);
  a35=(a16*a35);
  a26=(a26+a35);
  a23=(a23*a25);
  a23=(a6*a23);
  a26=(a26-a23);
  a36=(a36*a25);
  a36=(a8*a36);
  a26=(a26+a36);
  a37=(a37*a25);
  a37=(a21*a37);
  a26=(a26+a37);
  a26=(a26/a13);
  a40=(a40*a25);
  a39=(a39*a40);
  a39=(a12*a39);
  a39=(a22*a39);
  a38=(a38*a39);
  a26=(a26-a38);
  if (res[1]!=0) res[1][15]=a26;
  a26=arg[2] ? arg[2][2] : 0;
  if (res[2]!=0) res[2][0]=a26;
  a26=arg[2] ? arg[2][3] : 0;
  if (res[2]!=0) res[2][1]=a26;
  a38=cos(a1);
  a39=arg[2] ? arg[2][1] : 0;
  a38=(a38*a39);
  a38=(a3*a38);
  a4=(a4*a38);
  a38=(a0+a0);
  a38=(a38*a26);
  a2=(a2*a38);
  a4=(a4+a2);
  a2=cos(a1);
  a2=(a2*a39);
  a7=(a7*a2);
  a2=sin(a1);
  a2=(a2*a39);
  a2=(a8*a2);
  a9=(a9*a2);
  a7=(a7-a9);
  a4=(a4+a7);
  a4=(a4/a11);
  a5=(a5/a11);
  a10=(a10+a10);
  a7=sin(a1);
  a7=(a7*a39);
  a10=(a10*a7);
  a10=(a12*a10);
  a5=(a5*a10);
  a4=(a4-a5);
  a11=(1./a11);
  a11=(a11+a4);
  if (res[2]!=0) res[2][2]=a11;
  a19=(a19/a13);
  a11=cos(a1);
  a11=(a11*a39);
  a14=(a14*a11);
  a11=sin(a1);
  a11=(a11*a39);
  a3=(a3*a11);
  a15=(a15*a3);
  a14=(a14-a15);
  a17=(a17*a14);
  a0=(a0+a0);
  a0=(a0*a26);
  a16=(a16*a0);
  a17=(a17+a16);
  a16=sin(a1);
  a16=(a16*a39);
  a6=(a6*a16);
  a17=(a17-a6);
  a6=cos(a1);
  a6=(a6*a39);
  a8=(a8*a6);
  a17=(a17+a8);
  a8=cos(a1);
  a8=(a8*a39);
  a21=(a21*a8);
  a17=(a17+a21);
  a17=(a17/a13);
  a18=(a18/a13);
  a20=(a20+a20);
  a1=sin(a1);
  a1=(a1*a39);
  a20=(a20*a1);
  a12=(a12*a20);
  a22=(a22*a12);
  a18=(a18*a22);
  a17=(a17-a18);
  a19=(a19+a17);
  if (res[2]!=0) res[2][3]=a19;
  return 0;
}

CASADI_SYMBOL_EXPORT void vdeFun_incref(void) {
}

CASADI_SYMBOL_EXPORT void vdeFun_decref(void) {
}

CASADI_SYMBOL_EXPORT int vdeFun_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT int vdeFun_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT const char* vdeFun_name_in(int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* vdeFun_name_out(int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const int* vdeFun_sparsity_in(int i) {
  switch (i) {
    case 0: return s0;
    case 1: return s1;
    case 2: return s0;
    case 3: return s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const int* vdeFun_sparsity_out(int i) {
  switch (i) {
    case 0: return s0;
    case 1: return s1;
    case 2: return s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int vdeFun_work(int *sz_arg, int* sz_res, int *sz_iw, int *sz_w) {
  if (sz_arg) *sz_arg = 4;
  if (sz_res) *sz_res = 3;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 44;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
