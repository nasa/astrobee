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
#include <stdio.h>
#include <math.h>

#include "../include/blasfeo_common.h"



#if defined(LA_HIGH_PERFORMANCE)




void blasfeo_cvt_d2s_vec(int m, struct blasfeo_dvec *vd, int vdi, struct blasfeo_svec *vs, int vsi)
	{
	double *pd = vd->pa+vdi;
	float *ps = vs->pa+vsi;
	int ii;
	for(ii=0; ii<m; ii++)
		{
		ps[ii] = (float) pd[ii];
		}
	return;
	}



void blasfeo_cvt_s2d_vec(int m, struct blasfeo_svec *vs, int vsi, struct blasfeo_dvec *vd, int vdi)
	{
	double *pd = vd->pa+vdi;
	float *ps = vs->pa+vsi;
	int ii;
	for(ii=0; ii<m; ii++)
		{
		pd[ii] = (double) ps[ii];
		}
	return;
	}



void blasfeo_cvt_d2s_mat(int m, int n, struct blasfeo_dmat *Md, int mid, int nid, struct blasfeo_smat *Ms, int mis, int nis)
	{
	if(mid!=0 | mis!=0)
		{
		printf("\nblasfeo_cvt_d2s_mat: feature not implmeneted yet: mid=%d, mis=%d\n\n", mid, mis);
		exit(1);
		}
	const int psd = 4;
	const int pss = 4;
	const int sdd = Md->cn;
	double *D0 = Md->pA + nid*psd;
	double *D1;
	const int sds = Ms->cn;
	float *S = Ms->pA + nis*pss;
	int ii, jj, ll;
	for(ii=0; ii<m-3; ii+=4)
		{
		D1 = D0 + psd*sdd;
		for(jj=0; jj<n; jj++)
			{
			S[0+jj*pss] = (float) D0[0+jj*psd];
			S[1+jj*pss] = (float) D0[1+jj*psd];
			S[2+jj*pss] = (float) D0[2+jj*psd];
			S[3+jj*pss] = (float) D0[3+jj*psd];
			}
		D0 += 4*sdd;
		S  += 4*sds;
		}
	if(m-ii>0)
		{
		for(jj=0; jj<n; jj++)
			{
			for(ll=0; ll<m-ii; ll++)
				{
				S[ll+jj*pss] = (float) D0[ll+jj*psd];
				}
			}
		}
	return;
	}



void blasfeo_cvt_s2d_mat(int m, int n, struct blasfeo_smat *Ms, int mis, int nis, struct blasfeo_dmat *Md, int mid, int nid)
	{
	printf("\nblasfeo_cvt_s2d_mat: feature not implmeneted yet\n\n");
	exit(1);
	return;
	}



#else

#error : wrong LA choice

#endif

