/**************************************************************************************************
*                                                                                                 *
* This file is part of BLASFEO.                                                                   *
*                                                                                                 *
* BLASFEO -- BLAS For Embedded Optimization.                                                      *
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
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <blasfeo_common.h>



#define STR(x) #x
#define SHOW_DEFINE(x) printf("%-16s= %s\n", #x, STR(x));



#ifndef BLASFEO_LA
	#error BLASFEO_LA undefined
#endif

#ifndef BLASFEO_TARGET
	#error BLASFEO_TARGET undefined
#endif

#ifndef PRECISION
	#error PRECISION undefined
#endif

#ifndef ROUTINE
	#error ROUTINE undefined
#endif

#ifndef CONTINUE_ON_ERROR
	#define CONTINUE_ON_ERROR 0
#endif

#define concatenate(var, post) var ## post
#define string(var) STR(var)

//#define REF(fun) concatenate(fun, _ref)
#define REF(fun) concatenate(ref_, fun)
#define BLASFEO(fun) concatenate(blasfeo_, fun)
#define BLASFEO_BLAS(fun) concatenate(blas_, fun)
#define BLAS(fun) concatenate(fun, _)
#define WORKSIZE(fun) concatenate(fun, _worksize)

#ifndef VERBOSE
#define VERBOSE 0
#endif


#define ANSI_COLOR_RED     "\x1b[31m"
#define ANSI_COLOR_GREEN   "\x1b[32m"
#define ANSI_COLOR_YELLOW  "\x1b[33m"
#define ANSI_COLOR_BLUE    "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN    "\x1b[36m"
#define ANSI_COLOR_RESET   "\x1b[0m"


// Collection of macros  and functions inteded to be used to compute compare and check matrices

#ifdef PRECISION_DOUBLE

#define MATEL_LIBSTR BLASFEO_DMATEL
#define MATEL_REF BLASFEO_DMATEL
#define VECEL_LIBSTR BLASFEO_DVECEL
#define VECEL_REF BLASFEO_DVECEL

#else

#ifdef PRECISION_SINGLE

#define MATEL_LIBSTR BLASFEO_SMATEL
#define MATEL_REF BLASFEO_SMATEL
#define VECEL_LIBSTR BLASFEO_SVECEL
#define VECEL_REF BLASFEO_SVECEL

#else

#error None of double and single precision defined !!!

#endif

#endif

//#if defined(LA_HIGH_PERFORMANCE)
// Panel major element extraction macro
//#define MATEL_LIBSTR(sA,ai,aj) ((sA)->pA[((ai)-((ai)&(PS-1)))*(sA)->cn+(aj)*PS+((ai)&(PS-1))])
//#define MATEL_LIB(sA,ai,aj) ((sA)->pA[(ai)+(aj)*(sA)->m])
//#elif defined(LA_EXTERNAL_BLAS_WRAPPER) | defined(LA_REFERENCE)
//#define MATEL_LIBSTR(sA,ai,aj) ((sA)->pA[(ai)+(aj)*(sA)->m])
//#else
//#error : wrong LA choice
//#endif

// Column major element extraction macro
//
//#define VECEL_LIBSTR(sa,ai) ((sa)->pa[ai])
//#define VECEL_LIB(sa,ai) ((sa)->pa[ai])

struct RoutineArgs
	{
	// coefficients
	REAL alpha;
	REAL beta;

	int err_i;
	int err_j;

	// sizes
	int n;
	int m;
	int k;

	// offset
	int ai;
	int aj;

	int bi;
	int bj;

	int ci;
	int cj;

	int di;
	int dj;

	// indexes arrays
	// struct signature
	int *sipiv;
	// ref signature
	int *ripiv;
	// ref compare signature
	int *cipiv;

	// matrices
	struct STRMAT *sA;
	struct STRMAT *sA_po;
	struct STRMAT *sB;
	struct STRMAT *sC;
	struct STRMAT *sD;

	struct STRMAT_REF *rA;
	struct STRMAT_REF *rA_po;
	struct STRMAT_REF *rB;
	struct STRMAT_REF *rC;
	struct STRMAT_REF *rD;

	struct STRMAT_REF *cA;
	struct STRMAT_REF *cA_po;
	struct STRMAT_REF *cB;
	struct STRMAT_REF *cC;
	struct STRMAT_REF *cD;

//	void * work;
	int info;

	// blas_api
	char ta;
	char tb;
	char uplo;
	};



struct TestArgs
	{

	// sub-mastrix offset, sweep start
	int ai0;
	int bi0;
	int di0;
	int xj0;

	// sub-mastrix offset, sweep lenght
	int ais;
	int bis;
	int dis;
	int xjs;

	// sub-matrix dimensions, sweep start
	int ni0;
	int nj0;
	int nk0;

	// sub-matrix dimensions, sweep lenght
	int nis;
	int njs;
	int nks;

	int alphas;
	int betas;

	REAL alpha_l[6];
	REAL beta_l[6];

	// blas_api parameters
	int tas;
	int tbs;
	int uplos;

	const char* ta_l[2];
	const char* tb_l[2];
	const char* uplos_l[2];

	// statistics
	int total_calls;
	};



void initialize_args(struct RoutineArgs * args);
void set_test_args(struct TestArgs * targs);
int compute_total_calls(struct TestArgs * targs);
void call_routines(struct RoutineArgs *args);
void print_routine(struct RoutineArgs *args);
void print_routine_matrices(struct RoutineArgs *args);

void print_xmat_debug(
	int m, int n, struct STRMAT_REF *sA,
	int ai, int aj, int err_i, int err_j, int ERR);

void blasfeo_print_xmat_debug(int m, int n, struct STRMAT *sA, int ai, int aj, int err_i, int err_j, int ERR, char *label);

int GECMP_LIBSTR(
	int n, int m, int bi, int bj, struct STRMAT *sC,
	struct STRMAT_REF *rC, int* err_i, int* err_j, int debug);

int GECMP_BLASAPI(
	int n, int m, int bi, int bj, struct STRMAT_REF *cC,
	struct STRMAT_REF *rC, int* err_i, int* err_j, int debug);


// template base on routine class
#include string(ROUTINE_CLASS_C)
