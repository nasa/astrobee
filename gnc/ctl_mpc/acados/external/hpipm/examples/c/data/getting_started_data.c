/**************************************************************************************************
*                                                                                                 *
* This file is part of HPIPM.                                                                     *
*                                                                                                 *
* HPIPM -- High-Performance Interior Point Method.                                                *
* Copyright (C) 2019 by Gianluca Frison.                                                          *
* Developed at IMTEK (University of Freiburg) under the supervision of Moritz Diehl.              *
* All rights reserved.                                                                            *
*                                                                                                 *
* The 2-Clause BSD License                                                                        *
*                                                                                                 *
* Redistribution and use in source and binary forms, with or without                              *
* modification, are permitted provided that the following conditions are met:                     *
*                                                                                                 *
* 1. Redistributions of source code must retain the above copyright notice, this                  *
*    list of conditions and the following disclaimer.                                             *
* 2. Redistributions in binary form must reproduce the above copyright notice,                    *
*    this list of conditions and the following disclaimer in the documentation                    *
*    and/or other materials provided with the distribution.                                       *
*                                                                                                 *
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND                 *
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED                   *
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                          *
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR                 *
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES                  *
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;                    *
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND                     *
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                      *
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS                   *
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                    *
*                                                                                                 *
* Author: Gianluca Frison, gianluca.frison (at) imtek.uni-freiburg.de                             *
*                                                                                                 *
**************************************************************************************************/

#include <stdlib.h>



// QP size

// horizon lenght
int N = 5;
// number of input
static int nnu[6] = {1, 1, 1, 1, 1, 0};
// number of states
static int nnx[6] = {2, 2, 2, 2, 2, 2};
// number of input box constraints
static int nnbu[6] = {0, 0, 0, 0, 0, 0};
// number of states box constraints
static int nnbx[6] = {2, 0, 0, 0, 0, 0};
// number of general constraints
static int nng[6] = {0, 0, 0, 0, 0, 0};
// number of softed constraints on state box constraints
static int nnsbx[6] = {0, 0, 0, 0, 0, 0};
// number of softed constraints on input box constraints
static int nnsbu[6] = {0, 0, 0, 0, 0, 0};
// number of softed constraints on general constraints
static int nnsg[6] = {0, 0, 0, 0, 0, 0};
// number of input box constraints considered as equality
static int nnbue[6] = {0, 0, 0, 0, 0, 0};
// number of states box constraints considered as equality
static int nnbxe[6] = {0, 0, 0, 0, 0, 0};
// number of general constraints considered as equality
static int nnge[6] = {0, 0, 0, 0, 0, 0};


// QP data

//
static double A[] = {1, 0, 1, 1};
//
static double B[] = {0, 1};
//
static double b[] = {0, 0};

//
static double Q[] = {1, 0, 0, 1};
//
static double R[] = {1};
//
static double S[] = {0, 0};
//
static double q[] = {1, 1};
//
static double r[] = {0};

//
static double lbx0[] = {1, 1};
//
static double ubx0[] = {1, 1};
//
static int idxbx0[] = {0, 1};

//
static double u_guess[] = {0};
//
static double x_guess[] = {0, 0};
//
static double sl_guess[] = {};
//
static double su_guess[] = {};

// array of pointers

//
static double *AA[5] = {A, A, A, A, A};
//
static double *BB[5] = {B, B, B, B, B};
//
static double *bb[5] = {b, b, b, b, b};
//
static double *QQ[6] = {Q, Q, Q, Q, Q, Q};
//
static double *RR[6] = {R, R, R, R, R, R};
//
static double *SS[6] = {S, S, S, S, S, S};
//
static double *qq[6] = {q, q, q, q, q, q};
//
static double *rr[6] = {r, r, r, r, r, r};
//
static int *iidxbx[6] = {idxbx0, NULL, NULL, NULL, NULL, NULL};
//
static double *llbx[6] = {lbx0, NULL, NULL, NULL, NULL, NULL};
//
static double *uubx[6] = {ubx0, NULL, NULL, NULL, NULL, NULL};
//
static int *iidxbu[6] = {};
//
static double *llbu[6] = {};
//
static double *uubu[6] = {};
//
static double *CC[6] = {};
//
static double *DD[6] = {};
//
static double *llg[6] = {};
//
static double *uug[6] = {};
//
static double *ZZl[6] = {};
//
static double *ZZu[6] = {};
//
static double *zzl[6] = {};
//
static double *zzu[6] = {};
//
static int *iidxs[6] = {};
//
static double *llls[6] = {};
//
static double *llus[6] = {};
//
static double *iidxe[6] = {};

//
static double *uu_guess[6] = {u_guess, u_guess, u_guess, u_guess, u_guess, u_guess};
//
static double *xx_guess[6] = {x_guess, x_guess, x_guess, x_guess, x_guess, x_guess};
//
static double *ssl_guess[6] = {sl_guess, sl_guess, sl_guess, sl_guess, sl_guess, sl_guess};
//
static double *ssu_guess[6] = {su_guess, su_guess, su_guess, su_guess, su_guess, su_guess};



// export as global data

int *nu = nnu;
int *nx = nnx;
int *nbu = nnbu;
int *nbx = nnbx;
int *ng = nng;
int *nsbx = nnsbx;
int *nsbu = nnsbu;
int *nsg = nnsg;
int *nbue = nnbue;
int *nbxe = nnbxe;
int *nge = nnge;

double **hA = AA;
double **hB = BB;
double **hb = bb;
double **hQ = QQ;
double **hR = RR;
double **hS = SS;
double **hq = qq;
double **hr = rr;
int **hidxbx = iidxbx;
double **hlbx = llbx;
double **hubx = uubx;
int **hidxbu = iidxbu;
double **hlbu = llbu;
double **hubu = uubu;
double **hC = CC;
double **hD = DD;
double **hlg = llg;
double **hug = uug;
double **hZl = ZZl;
double **hZu = ZZu;
double **hzl = zzl;
double **hzu = zzu;
int **hidxs = iidxs;
double **hlls = llls;
double **hlus = llus;
double **hidxe = iidxe;

double **hu_guess = uu_guess;
double **hx_guess = xx_guess;
double **hsl_guess = ssl_guess;
double **hsu_guess = ssu_guess;

// arg
int mode = 1;
int iter_max = 30;
double alpha_min = 1e-8;
double mu0 = 1e4;
double tol_stat = 1e-4;
double tol_eq = 1e-5;
double tol_ineq = 1e-5;
double tol_comp = 1e-5;
double reg_prim = 1e-12;
int warm_start = 0;
int pred_corr = 1;
int ric_alg = 0;
int split_step = 1;

