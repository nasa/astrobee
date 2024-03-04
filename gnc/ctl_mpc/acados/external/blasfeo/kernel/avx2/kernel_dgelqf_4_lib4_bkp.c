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

#include <math.h>
#include <stdio.h>

#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
#include <smmintrin.h>  // SSE4
#include <immintrin.h>  // AVX

#include "../../include/blasfeo_common.h"
#include "../../include/blasfeo_d_aux.h"
#include "../../include/blasfeo_d_kernel.h"



// assume n>=4
void kernel_dgelqf_dlarft_4_lib4(int n, double *pD, double *dD, double *pT)
	{
	return;
	int ii, jj, ll;
	double alpha, beta, tmp, w0, w1, w2, w3;
	const int ps = 4;
	// zero tau matrix
	for(ii=0; ii<16; ii++)
		pT[ii] = 0.0;
	// first column
	beta = 0.0;
	for(ii=1; ii<n; ii++)
		{
		tmp = pD[0+ps*ii];
		beta += tmp*tmp;
		}
	if(beta==0.0)
		{
		dD[0] = 0.0;
		goto col2;
		}
	alpha = pD[0+ps*0];
	beta += alpha*alpha;
	beta = sqrt(beta);
	if(alpha>0)
		beta = -beta;
	dD[0] = (beta-alpha) / beta;
	pT[0+ps*0] = - dD[0];
	tmp = -1.0 / (beta-alpha);
	//
	pD[0+ps*0] = beta;
	w1 = pD[1+ps*0];
	w2 = pD[2+ps*0];
	w3 = pD[3+ps*0];
	//
	pD[0+ps*1] *= tmp;
	w1 += pD[1+ps*1] * pD[0+ps*1];
	w2 += pD[2+ps*1] * pD[0+ps*1];
	w3 += pD[3+ps*1] * pD[0+ps*1];
	//
	pD[0+ps*2] *= tmp;
	w1 += pD[1+ps*2] * pD[0+ps*2];
	w2 += pD[2+ps*2] * pD[0+ps*2];
	w3 += pD[3+ps*2] * pD[0+ps*2];
	//
	pD[0+ps*3] *= tmp;
	w1 += pD[1+ps*3] * pD[0+ps*3];
	w2 += pD[2+ps*3] * pD[0+ps*3];
	w3 += pD[3+ps*3] * pD[0+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[0+ps*ii] *= tmp;
		w1 += pD[1+ps*ii] * pD[0+ps*ii];
		w2 += pD[2+ps*ii] * pD[0+ps*ii];
		w3 += pD[3+ps*ii] * pD[0+ps*ii];
		}
	//
	w1 = - dD[0] * w1;
	w2 = - dD[0] * w2;
	w3 = - dD[0] * w3;
	//
	pD[1+ps*0] += w1;
	pD[2+ps*0] += w2;
	pD[3+ps*0] += w3;
	//
	pD[1+ps*1] += w1 * pD[0+ps*1];
	pD[2+ps*1] += w2 * pD[0+ps*1];
	pD[3+ps*1] += w3 * pD[0+ps*1];
	//
	pD[1+ps*2] += w1 * pD[0+ps*2];
	pD[2+ps*2] += w2 * pD[0+ps*2];
	pD[3+ps*2] += w3 * pD[0+ps*2];
	beta = pD[1+ps*2] * pD[1+ps*2];
	//
	pD[1+ps*3] += w1 * pD[0+ps*3];
	pD[2+ps*3] += w2 * pD[0+ps*3];
	pD[3+ps*3] += w3 * pD[0+ps*3];
	beta += pD[1+ps*3] * pD[1+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[1+ps*ii] += w1 * pD[0+ps*ii];
		pD[2+ps*ii] += w2 * pD[0+ps*ii];
		pD[3+ps*ii] += w3 * pD[0+ps*ii];
		beta += pD[1+ps*ii] * pD[1+ps*ii];
		}
	// second column
col2:
	if(beta==0.0)
		{
		dD[1] = 0.0;
		tmp = 0.0;
		goto col3;
		}
	alpha = pD[1+ps*1];
	beta += alpha*alpha;
	beta = sqrt(beta);
	if(alpha>0)
		beta = -beta;
	dD[1] = (beta-alpha) / beta;
	pT[1+ps*1] = - dD[1];
	tmp = 1.0 / (alpha-beta);
	//
	pD[1+ps*1] = beta;
	w0 = pD[0+ps*1]; //
	w2 = pD[2+ps*1];
	w3 = pD[3+ps*1];
	//
	pD[1+ps*2] *= tmp;
	w0 += pD[0+ps*2] * pD[1+ps*2]; //
	w2 += pD[2+ps*2] * pD[1+ps*2];
	w3 += pD[3+ps*2] * pD[1+ps*2];
	//
	pD[1+ps*3] *= tmp;
	w0 += pD[0+ps*3] * pD[1+ps*3]; //
	w2 += pD[2+ps*3] * pD[1+ps*3];
	w3 += pD[3+ps*3] * pD[1+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[1+ps*ii] *= tmp;
		w0 += pD[0+ps*ii] * pD[1+ps*ii]; //
		w2 += pD[2+ps*ii] * pD[1+ps*ii];
		w3 += pD[3+ps*ii] * pD[1+ps*ii];
		}
	//
	pT[0+ps*1] = - dD[1] * (w0*pT[0+ps*0]);
	w2 = - dD[1] * w2;
	w3 = - dD[1] * w3;
	//
	pD[2+ps*1] += w2;
	pD[3+ps*1] += w3;
	//
	pD[2+ps*2] += w2 * pD[1+ps*2];
	pD[3+ps*2] += w3 * pD[1+ps*2];
	//
	pD[2+ps*3] += w2 * pD[1+ps*3];
	pD[3+ps*3] += w3 * pD[1+ps*3];
	beta = pD[2+ps*3] * pD[2+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[2+ps*ii] += w2 * pD[1+ps*ii];
		pD[3+ps*ii] += w3 * pD[1+ps*ii];
		beta += pD[2+ps*ii] * pD[2+ps*ii];
		}
	// third column
col3:
	if(beta==0.0)
		{
		dD[2] = 0.0;
		tmp = 0.0;
		goto col4;
		}
	alpha = pD[2+ps*2];
	beta += alpha*alpha;
	beta = sqrt(beta);
	if(alpha>0)
		beta = -beta;
	dD[2] = (beta-alpha) / beta;
	pT[2+ps*2] = - dD[2];
	tmp = 1.0 / (alpha-beta);
	//
	pD[2+ps*2] = beta;
	w0 = pD[0+ps*2];
	w1 = pD[1+ps*2];
	w3 = pD[3+ps*2];
	//
	pD[2+ps*3] *= tmp;
	w0 += pD[0+ps*3] * pD[2+ps*3];
	w1 += pD[1+ps*3] * pD[2+ps*3];
	w3 += pD[3+ps*3] * pD[2+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[2+ps*ii] *= tmp;
		w0 += pD[0+ps*ii] * pD[2+ps*ii];
		w1 += pD[1+ps*ii] * pD[2+ps*ii];
		w3 += pD[3+ps*ii] * pD[2+ps*ii];
		}
	//
	pT[0+ps*2] = - dD[2] * (w0*pT[0+ps*0] + w1*pT[0+ps*1]);
	pT[1+ps*2] = - dD[2] * (w1*pT[1+ps*1]);
	w3 = - dD[2] * w3;
//printf("\n%f %f %f\n", pT[0+ps*2], pT[1+ps*2], w3);
//return;
	//
	pD[3+ps*2] += w3;
	//
	pD[3+ps*3] += w3 * pD[2+ps*3];
	//
	beta = 0.0;
	for(ii=4; ii<n; ii++)
		{
		pD[3+ps*ii] += w3 * pD[2+ps*ii];
		beta += pD[3+ps*ii] * pD[3+ps*ii];
		}
	// fourth column
col4:
	if(beta==0.0)
		{
		dD[3] = 0.0;
		tmp = 0.0;
		return;
		}
	alpha = pD[3+ps*3];
	beta += alpha*alpha;
	beta = sqrt(beta);
	if(alpha>0)
		beta = -beta;
	dD[3] = (beta-alpha) / beta;
	pT[3+ps*3] = - dD[3];
	tmp = 1.0 / (alpha-beta);
	//
	pD[3+ps*3] = beta;
	w0 =  pD[0+ps*3];
	w1 =  pD[1+ps*3];
	w2 =  pD[2+ps*3];
	//
	for(ii=4; ii<n; ii++)
		{
		pD[3+ps*ii] *= tmp;
		w0 += pD[0+ps*ii] * pD[3+ps*ii];
		w1 += pD[1+ps*ii] * pD[3+ps*ii];
		w2 += pD[2+ps*ii] * pD[3+ps*ii];
		}
	//
	pT[0+ps*3] = - dD[3] * (w0*pT[0+ps*0] + w1*pT[0+ps*1] + w2*pT[0+ps*2]);
	pT[1+ps*3] = - dD[3] * (w1*pT[1+ps*1] + w2*pT[1+ps*2]);
	pT[2+ps*3] = - dD[3] * (w2*pT[2+ps*2]);
	return;
	}




