/**************************************************************************************************
*                                                                                                 *
* This file is part of HPMPC.                                                                     *
*                                                                                                 *
* HPMPC -- Library for High-Performance implementation of solvers for MPC.                        *
* Copyright (C) 2014-2015 by Technical University of Denmark. All rights reserved.                *
*                                                                                                 *
* HPMPC is free software; you can redistribute it and/or                                          *
* modify it under the terms of the GNU Lesser General Public                                      *
* License as published by the Free Software Foundation; either                                    *
* version 2.1 of the License, or (at your option) any later version.                              *
*                                                                                                 *
* HPMPC is distributed in the hope that it will be useful,                                        *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                                  *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.                                            *
* See the GNU Lesser General Public License for more details.                                     *
*                                                                                                 *
* You should have received a copy of the GNU Lesser General Public                                *
* License along with HPMPC; if not, write to the Free Software                                    *
* Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA                  *
*                                                                                                 *
* Author: Gianluca Frison, giaf (at) dtu.dk                                                       *
*                                                                                                 *
**************************************************************************************************/

#include "../include/kernel_s_lib8.h"
#include "../include/block_size.h"



/* preforms                                          */
/* C  = A * B' (alg== 0)                             */
/* C += A * B' (alg== 1)                             */
/* C -= A * B' (alg==-1)                             */
/* where A, B and C are packed with block size 4     */
void sgemm_nt_lib(int m, int n, int k, float *pA, int sda, float *pB, int sdb, int alg, float *pC, int sdc)
	{

	const int bs = 8;

	int i, j, jj;
	
	i = 0;
#if defined(TARGET_X64_AVX2)
	for(; i<m-16; i+=24)
		{
		j = 0;
		for(; j<n-7; j+=8)
			{
			kernel_sgemm_nt_24x4_lib8(k, &pA[0+i*sda], &pA[0+(i+8)*sda], &pA[0+(i+16)*sda], &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+8)*sdc], &pC[0+(j+0)*bs+(i+16)*sdc], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+8)*sdc], &pC[0+(j+0)*bs+(i+16)*sdc], alg);
			kernel_sgemm_nt_24x4_lib8(k, &pA[0+i*sda], &pA[0+(i+8)*sda], &pA[0+(i+16)*sda], &pB[4+j*sdb], &pC[0+(j+4)*bs+i*sdc], &pC[0+(j+4)*bs+(i+8)*sdc], &pC[0+(j+4)*bs+(i+16)*sdc], &pC[0+(j+4)*bs+i*sdc], &pC[0+(j+4)*bs+(i+8)*sdc], &pC[0+(j+4)*bs+(i+16)*sdc], alg);
			}
		jj = 0;
		for(; jj<n-j-3; jj+=4)
			{
			kernel_sgemm_nt_24x4_lib8(k, &pA[0+i*sda], &pA[0+(i+8)*sda], &pA[0+(i+16)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+8)*sdc], &pC[0+(j+jj)*bs+(i+16)*sdc], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+8)*sdc], &pC[0+(j+jj)*bs+(i+16)*sdc], alg);
			}
/*		for(; jj<n-j-1; jj+=2)*/
/*			{*/
/*			kernel_sgemm_nt_16x2_lib8(k, &pA[0+i*sda], &pA[0+(i+8)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+8)*sdc], alg);*/
/*			}*/
/*		for(; jj<n-j; jj++)*/
/*			{*/
/*			kernel_sgemm_nt_16x1_lib8(k, &pA[0+i*sda], &pA[0+(i+8)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+8)*sdc], alg);*/
/*			}*/
		}
#endif
	for(; i<m-8; i+=16)
		{
		j = 0;
		for(; j<n-7; j+=8)
			{
			kernel_sgemm_nt_16x4_lib8(k, &pA[0+i*sda], &pA[0+(i+8)*sda], &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+8)*sdc], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+8)*sdc], alg);
			kernel_sgemm_nt_16x4_lib8(k, &pA[0+i*sda], &pA[0+(i+8)*sda], &pB[4+j*sdb], &pC[0+(j+4)*bs+i*sdc], &pC[0+(j+4)*bs+(i+8)*sdc], &pC[0+(j+4)*bs+i*sdc], &pC[0+(j+4)*bs+(i+8)*sdc], alg);
			}
		jj = 0;
		for(; jj<n-j-3; jj+=4)
			{
			kernel_sgemm_nt_16x4_lib8(k, &pA[0+i*sda], &pA[0+(i+8)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+8)*sdc], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+8)*sdc], alg);
			}
/*		for(; jj<n-j-1; jj+=2)*/
/*			{*/
/*			kernel_sgemm_nt_16x2_lib8(k, &pA[0+i*sda], &pA[0+(i+8)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+8)*sdc], alg);*/
/*			}*/
/*		for(; jj<n-j; jj++)*/
/*			{*/
/*			kernel_sgemm_nt_16x1_lib8(k, &pA[0+i*sda], &pA[0+(i+8)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+8)*sdc], alg);*/
/*			}*/
		}
	for(; i<m-4; i+=8)
		{
		j = 0;
		for(; j<n-7; j+=8)
			{
			kernel_sgemm_nt_8x8_lib8(k, &pA[0+i*sda], &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+i*sdc], alg);
/*			kernel_sgemm_nt_8x4_lib8(k, &pA[0+i*sda], &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], alg);*/
/*			kernel_sgemm_nt_8x4_lib8(k, &pA[0+i*sda], &pB[4+j*sdb], &pC[0+(j+4)*bs+i*sdc], alg);*/
			}
		jj = 0;
		for(; jj<n-j-3; jj+=4)
			{
			kernel_sgemm_nt_8x4_lib8(k, &pA[0+i*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+i*sdc], alg);
			}
/*		for(; jj<n-j-1; jj+=2)*/
/*			{*/
/*			kernel_sgemm_nt_8x2_lib8(k, &pA[0+i*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], alg);*/
/*			}*/
/*		for(; jj<n-j; jj++)*/
/*			{*/
/*			kernel_sgemm_nt_8x1_lib8(k, &pA[0+i*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], alg);*/
/*			}*/
		}
	for(; i<m; i+=4)
		{
		j = 0;
		for(; j<n-7; j+=8)
			{
			kernel_sgemm_nt_4x8_lib8(k, &pA[0+i*sda], &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+i*sdc], alg);
/*			kernel_sgemm_nt_4x4_lib8(k, &pA[0+i*sda], &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+i*sdc], alg);*/
/*			kernel_sgemm_nt_4x4_lib8(k, &pA[0+i*sda], &pB[4+j*sdb], &pC[0+(j+4)*bs+i*sdc], &pC[0+(j+4)*bs+i*sdc], alg);*/
			}
		jj = 0;
		for(; jj<n-j-3; jj+=4)
			{
			kernel_sgemm_nt_4x4_lib8(k, &pA[0+i*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+i*sdc], alg);
			}
/*		for(; jj<n-j-1; jj+=2)*/
/*			{*/
/*			kernel_sgemm_nt_4x2_lib8(k, &pA[0+i*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], alg);*/
/*			}*/
/*		for(; jj<n-j; jj++)*/
/*			{*/
/*			kernel_sgemm_nt_4x1_lib8(k, &pA[0+i*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], alg);*/
/*			}*/
		}
	
	}



/* preforms                                          */
/* C  = A * B'                                       */
/* where A, B and C are packed with block size 4,    */
/* and B is upper triangular                         */
void strmm_lib(int m, int n, float *pA, int sda, float *pB, int sdb, float *pC, int sdc)
	{
	
	const int bs = 8;
	
	int i, j;
	
	i = 0;
	for(; i<m-8; i+=16)
		{
		j = 0;
		for(; j<n-7; j+=8)
			{
			kernel_strmm_nt_16x4_lib8(n-j-0, &pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+8)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+8)*sdc]);
			kernel_strmm_nt_16x4_lib8(n-j-4, &pA[0+(j+4)*bs+i*sda], &pA[0+(j+4)*bs+(i+8)*sda], &pB[4+(j+4)*bs+j*sdb], &pC[0+(j+4)*bs+i*sdc], &pC[0+(j+4)*bs+(i+8)*sdc]);
			}
		if(n-j>3)
			{
			kernel_strmm_nt_16x4_lib8(n-j-0, &pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+8)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+8)*sdc]);
			if(n-j==5)
				{
				corner_strmm_nt_16x1_lib8(&pA[0+(j+4)*bs+i*sda], &pA[0+(j+4)*bs+(i+8)*sda], &pB[4+(j+4)*bs+j*sdb], &pC[0+(j+4)*bs+i*sdc], &pC[0+(j+4)*bs+(i+8)*sdc]);
				}
			else if(n-j==6)
				{
				corner_strmm_nt_16x2_lib8(&pA[0+(j+4)*bs+i*sda], &pA[0+(j+4)*bs+(i+8)*sda], &pB[4+(j+4)*bs+j*sdb], &pC[0+(j+4)*bs+i*sdc], &pC[0+(j+4)*bs+(i+8)*sdc]);
				}
			else if(n-j==7)
				{
				corner_strmm_nt_16x3_lib8(&pA[0+(j+4)*bs+i*sda], &pA[0+(j+4)*bs+(i+8)*sda], &pB[4+(j+4)*bs+j*sdb], &pC[0+(j+4)*bs+i*sdc], &pC[0+(j+4)*bs+(i+8)*sdc]);
				}
			j+=4;
			}
		else
			{
			if(n-j==1)
				{
				corner_strmm_nt_16x1_lib8(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+8)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+8)*sdc]);
				}
			else if(n-j==2)
				{
				corner_strmm_nt_16x2_lib8(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+8)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+8)*sdc]);
				}
			else if(n-j==3)
				{
				corner_strmm_nt_16x3_lib8(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+8)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+8)*sdc]);
				}
			}
		}
	for(; i<m; i+=8)
		{
		j = 0;
		for(; j<n-7; j+=8)
			{
			kernel_strmm_nt_8x4_lib8(n-j-0, &pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			kernel_strmm_nt_8x4_lib8(n-j-4, &pA[0+(j+4)*bs+i*sda], &pB[4+(j+4)*bs+j*sdb], &pC[0+(j+4)*bs+i*sdc]);
			}
		if(n-j>3)
			{
			kernel_strmm_nt_8x4_lib8(n-j-0, &pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			if(n-j==5)
				{
				corner_strmm_nt_8x1_lib8(&pA[0+(j+4)*bs+i*sda], &pB[4+(j+4)*bs+j*sdb], &pC[0+(j+4)*bs+i*sdc]);
				}
			else if(n-j==6)
				{
				corner_strmm_nt_8x2_lib8(&pA[0+(j+4)*bs+i*sda], &pB[4+(j+4)*bs+j*sdb], &pC[0+(j+4)*bs+i*sdc]);
				}
			else if(n-j==7)
				{
				corner_strmm_nt_8x3_lib8(&pA[0+(j+4)*bs+i*sda], &pB[4+(j+4)*bs+j*sdb], &pC[0+(j+4)*bs+i*sdc]);
				}
			j+=4;
			}
		else
			{
			if(n-j==1)
				{
				corner_strmm_nt_8x1_lib8(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
				}
			else if(n-j==2)
				{
				corner_strmm_nt_8x2_lib8(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
				}
			else if(n-j==3)
				{
				corner_strmm_nt_8x3_lib8(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
				}
			}
		}

	}



void ssyrk_spotrf_lib(int m, int n, int k, float *pA, int sda, float *pC, int sdc, float *diag)
	{
	const int bs = 8;
	const int d_ncl = S_NCL;
	const int k0 = (d_ncl-k%d_ncl)%d_ncl;
	
	int i, j;
	
/*	int n = m;*/
	
	float fact0[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	float fact1[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	
/*	j=0;*/
/*	i=0;*/
/*	kernel_ssyrk_spotrf_nt_16x4_lib8(k, j, &pA[i*sda], &pA[(i+8)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+8)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+8)*sda], fact);*/
/*	kernel_ssyrk_spotrf_nt_12x4_lib8(k, j+4, &pA[i*sda], &pA[(i+8)*sda], &pA[4+j*sda], &pC[(j+4)*bs+i*sdc], &pC[(j+4)*bs+(i+8)*sdc], &pA[(k0+k+j+4)*bs+i*sda], &pA[(k0+k+j+4)*bs+(i+8)*sda], fact);*/
/*	j=8;*/
/*	i = j;*/
/*	kernel_ssyrk_spotrf_nt_8x4_lib8(k, j, &pA[i*sda], &pA[j*sda], &pC[(j+0)*bs+i*sdc], &pA[(k0+k+j+0)*bs+i*sda], fact);*/
/*	kernel_ssyrk_spotrf_nt_4x4_lib8(k, j+4, &pA[i*sda], &pA[4+j*sda], &pC[(j+4)*bs+i*sdc], &pA[(k0+k+j+4)*bs+i*sda], fact);*/

	j = 0;
	for(; j<n-4; j+=8)
		{
		i = j;
		if(i<m-8)
			{
			kernel_ssyrk_spotrf_nt_16x4_lib8(k, j, &pA[i*sda], &pA[(i+8)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+8)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+8)*sda], fact0);
			diag[j+0] = fact0[0];
			diag[j+1] = fact0[2];
			diag[j+2] = fact0[5];
			diag[j+3] = fact0[9];
			kernel_ssyrk_spotrf_nt_12x4_lib8(k, j+4, &pA[i*sda], &pA[(i+8)*sda], &pA[4+j*sda], &pC[(j+4)*bs+i*sdc], &pC[(j+4)*bs+(i+8)*sdc], &pA[(k0+k+j+4)*bs+i*sda], &pA[(k0+k+j+4)*bs+(i+8)*sda], fact1);
			diag[j+4] = fact1[0];
			diag[j+5] = fact1[2];
			diag[j+6] = fact1[5];
			diag[j+7] = fact1[9];
			i += 16;
#if defined(TARGET_X64_AVX2)
			for(; i<m-16; i+=24)
				{
				kernel_sgemm_strsm_nt_24x4_lib8(k, j, &pA[i*sda], &pA[(i+8)*sda], &pA[(i+16)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+8)*sdc], &pC[j*bs+(i+16)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+8)*sda], &pA[(k0+k+j)*bs+(i+16)*sda], fact0);
				kernel_sgemm_strsm_nt_24x4_lib8(k, j+4, &pA[i*sda], &pA[(i+8)*sda], &pA[(i+16)*sda], &pA[4+j*sda], &pC[(j+4)*bs+i*sdc], &pC[(j+4)*bs+(i+8)*sdc], &pC[(j+4)*bs+(i+16)*sdc], &pA[(k0+k+j+4)*bs+i*sda], &pA[(k0+k+j+4)*bs+(i+8)*sda], &pA[(k0+k+j+4)*bs+(i+16)*sda], fact1);
				}
#endif
			for(; i<m-8; i+=16)
				{
				kernel_sgemm_strsm_nt_16x4_lib8(k, j, &pA[i*sda], &pA[(i+8)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+8)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+8)*sda], fact0);
				kernel_sgemm_strsm_nt_16x4_lib8(k, j+4, &pA[i*sda], &pA[(i+8)*sda], &pA[4+j*sda], &pC[(j+4)*bs+i*sdc], &pC[(j+4)*bs+(i+8)*sdc], &pA[(k0+k+j+4)*bs+i*sda], &pA[(k0+k+j+4)*bs+(i+8)*sda], fact1);
				}
			for(; i<m; i+=8)
				{
				kernel_sgemm_strsm_nt_8x4_lib8(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact0);
				kernel_sgemm_strsm_nt_8x4_lib8(k, j+4, &pA[i*sda], &pA[4+j*sda], &pC[(j+4)*bs+i*sdc], &pA[(k0+k+j+4)*bs+i*sda], fact1);
				}
			}
		else //if(i<m-2)
			{
			kernel_ssyrk_spotrf_nt_8x4_lib8(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact0);
			diag[j+0] = fact0[0];
			diag[j+1] = fact0[2];
			diag[j+2] = fact0[5];
			diag[j+3] = fact0[9];
			if(j<n-6)
				{
				kernel_ssyrk_spotrf_nt_4x4_lib8(k, j+4, &pA[4+i*sda], &pA[4+j*sda], &pC[4+(j+4)*bs+i*sdc], &pA[4+(k0+k+j+4)*bs+i*sda], fact1);
				diag[j+4] = fact1[0];
				diag[j+5] = fact1[2];
				diag[j+6] = fact1[5];
				diag[j+7] = fact1[9];
				}
			else
				{
				kernel_ssyrk_spotrf_nt_4x2_lib8(k, j+4, &pA[4+i*sda], &pA[4+j*sda], &pC[4+(j+4)*bs+i*sdc], &pA[4+(k0+k+j+4)*bs+i*sda], fact1);
				diag[j+4] = fact1[0];
				diag[j+5] = fact1[2];
				diag[j+6] = fact1[5];
				diag[j+7] = fact1[9];
				}
			}
		}
	if(j<n)
		{
		i = j;
/*		kernel_ssyrk_spotrf_nt_4x4_lib8(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact0);*/
/*		diag[j+0] = fact0[0];*/
/*		diag[j+1] = fact0[2];*/
/*		diag[j+2] = fact0[5];*/
/*		diag[j+3] = fact0[9];*/
		if(i<m-8)
			{
			kernel_ssyrk_spotrf_nt_16x4_lib8(k, j, &pA[i*sda], &pA[(i+8)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+8)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+8)*sda], fact0);
			diag[j+0] = fact0[0];
			diag[j+1] = fact0[2];
			diag[j+2] = fact0[5];
			diag[j+3] = fact0[9];
/*			kernel_ssyrk_spotrf_nt_12x4_lib8(k, j+4, &pA[i*sda], &pA[(i+8)*sda], &pA[4+j*sda], &pC[(j+4)*bs+i*sdc], &pC[(j+4)*bs+(i+8)*sdc], &pA[(k0+k+j+4)*bs+i*sda], &pA[(k0+k+j+4)*bs+(i+8)*sda], fact1);*/
/*			diag[j+4] = fact1[0];*/
/*			diag[j+5] = fact1[2];*/
/*			diag[j+6] = fact1[5];*/
/*			diag[j+7] = fact1[9];*/
			i += 16;
#if defined(TARGET_X64_AVX2)
			for(; i<m-16; i+=24)
				{
				kernel_sgemm_strsm_nt_24x4_lib8(k, j, &pA[i*sda], &pA[(i+8)*sda], &pA[(i+16)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+8)*sdc], &pC[j*bs+(i+16)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+8)*sda], &pA[(k0+k+j)*bs+(i+16)*sda], fact0);
/*				kernel_sgemm_strsm_nt_24x4_lib8(k, j+4, &pA[i*sda], &pA[(i+8)*sda], &pA[(i+16)*sda], &pA[4+j*sda], &pC[(j+4)*bs+i*sdc], &pC[(j+4)*bs+(i+8)*sdc], &pC[(j+4)*bs+(i+16)*sdc], &pA[(k0+k+j+4)*bs+i*sda], &pA[(k0+k+j+4)*bs+(i+8)*sda], &pA[(k0+k+j+4)*bs+(i+16)*sda], fact1);*/
				}
#endif
			for(; i<m-8; i+=16)
				{
				kernel_sgemm_strsm_nt_16x4_lib8(k, j, &pA[i*sda], &pA[(i+8)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+8)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+8)*sda], fact0);
/*				kernel_sgemm_strsm_nt_16x4_lib8(k, j+4, &pA[i*sda], &pA[(i+8)*sda], &pA[4+j*sda], &pC[(j+4)*bs+i*sdc], &pC[(j+4)*bs+(i+8)*sdc], &pA[(k0+k+j+4)*bs+i*sda], &pA[(k0+k+j+4)*bs+(i+8)*sda], fact1);*/
				}
			for(; i<m; i+=8)
				{
				kernel_sgemm_strsm_nt_8x4_lib8(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact0);
/*				kernel_sgemm_strsm_nt_8x4_lib8(k, j+4, &pA[i*sda], &pA[4+j*sda], &pC[(j+4)*bs+i*sdc], &pA[(k0+k+j+4)*bs+i*sda], fact1);*/
				}
			}
		else //if(i<m-2)
			{
			if(j<n-2)
				{
				kernel_ssyrk_spotrf_nt_8x4_lib8(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact0);
				diag[j+0] = fact0[0];
				diag[j+1] = fact0[2];
				diag[j+2] = fact0[5];
				diag[j+3] = fact0[9];
				}
			else
				{
				kernel_ssyrk_spotrf_nt_8x2_lib8(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact0);
				diag[j+0] = fact0[0];
				diag[j+1] = fact0[2];
				diag[j+2] = fact0[5];
				diag[j+3] = fact0[9];
				}
/*			kernel_ssyrk_spotrf_nt_4x4_lib8(k, j+4, &pA[i*sda], &pA[4+j*sda], &pC[4+(j+4)*bs+i*sdc], &pA[4+(k0+k+j+4)*bs+i*sda], fact1);*/
/*			diag[j+4] = fact1[0];*/
/*			diag[j+5] = fact1[2];*/
/*			diag[j+6] = fact1[5];*/
/*			diag[j+7] = fact1[9];*/
			}
		j+=4;
		}



/*	for(; j<n; j+=2)*/
/*		{*/
/*		i = j;*/
/*		if(i<m-2)*/
/*			{*/
/*			kernel_ssyrk_spotrf_nt_4x2_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);*/
/*			diag[j+0] = fact[0];*/
/*			diag[j+1] = fact[2];*/
/*			i += 4;*/
/*#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)*/
/*			for(; i<m-4; i+=8)*/
/*				{*/
/*				kernel_sgemm_strsm_nt_8x2_lib4(k, j, &pA[i*sda], &pA[(i+4)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+4)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+4)*sda], fact);*/
/*				}*/
/*#endif*/
/*			for(; i<m-2; i+=4)*/
/*				{*/
/*				kernel_sgemm_strsm_nt_4x2_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);*/
/*				}*/
/*			for(; i<m; i+=2)*/
/*				{*/
/*				kernel_sgemm_strsm_nt_2x2_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);*/
/*				}*/
/*			}*/
/*		else //if(i<m)*/
/*			{*/
/*			kernel_ssyrk_spotrf_nt_2x2_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);*/
/*			diag[j+0] = fact[0];*/
/*			diag[j+1] = fact[2];*/
/*			}*/
/*		}*/

	}



// transpose & align lower triangular matrix
void strtr_l_lib(int m, int offset, float *pA, int sda, float *pC, int sdc)
	{
	
	const int bs = 8;
	
	int mna = (bs-offset%bs)%bs;
	
	int j;
	
	j=0;
	for(; j<m; j+=8)
		{
		kernel_stran_8_lib8(m-j, mna, pA, sda, pC);
		pA += bs*(sda+bs);
		pC += bs*(sdc+bs);
		}

/*	j=0;*/
/*	for(; j<m-3; j+=4)*/
/*		{*/
/*		kernel_dtran_4_lib4(m-j, mna, pA, sda, pC);*/
/*		pA += bs*(sda+bs);*/
/*		pC += bs*(sdc+bs);*/
/*		}*/
/*	if(j==m)*/
/*		{*/
/*		return;*/
/*		}*/
/*	else if(m-j==1)*/
/*		{*/
/*		pC[0] = pA[0];*/
/*		}*/
/*	else if(m-j==2)*/
/*		{*/
/*		corner_dtran_2_lib4(mna, pA, sda, pC);*/
/*		}*/
/*	else // if(m-j==3)*/
/*		{*/
/*		corner_dtran_3_lib4(mna, pA, sda, pC);*/
/*		}*/
	
	}



void sgemv_n_lib(int m, int n, float *pA, int sda, float *x, float *y, int alg) // pA has to be aligned !!!
	{
	
	const int bs = 8;
	
	int j;

	j=0;
/*	for(; j<n-7; j+=8)*/
/*	for(; j<m-4; j+=8)*/
	for(; j<m-8; j+=16)
		{
		kernel_sgemv_n_16_lib8(n, pA, pA+sda*bs, x, y, alg);
		pA += 2*sda*bs;
		y  += 2*bs;
		}
	for(; j<m; j+=8)
		{
		kernel_sgemv_n_8_lib8(n, pA, x, y, alg);
		pA += sda*bs;
		y  += bs;
		}
/*	for(; j<n-3; j+=4)*/
/*for(; j<m; j+=4)*/
/*	{*/
/*	kernel_sgemv_n_4_lib4(n, pA, x, y, alg);*/
/*	pA += sda*bs;*/
/*	y  += bs;*/
/*	}*/
/*	for(; j<m-1; j+=2)*/
/*		{*/
/*		kernel_sgemv_n_2_lib4(n, pA, x, y, alg);*/
/*		pA += 2;*/
/*		y  += 2;*/
/*		}*/
/*	for(; j<m; j++)*/
/*		{*/
/*		kernel_sgemv_n_1_lib4(n, pA, x, y, alg);*/
/*		pA += 1;*/
/*		y  += 1;*/
/*		}*/

	}



void sgemv_t_lib(int m, int n, int offset, float *pA, int sda, float *x, float *y, int alg)
	{
	
	const int bs = 8;
	
	int mna = (bs-offset%bs)%bs;
	
	int j;
	
	j=0;
	for(; j<n-7; j+=8)
		{
		kernel_sgemv_t_8_lib8(m, mna, pA+j*bs, sda, x, y+j, alg);
		}
	for(; j<n-3; j+=4)
		{
		kernel_sgemv_t_4_lib8(m, mna, pA+j*bs, sda, x, y+j, alg);
		}
	for(; j<n-2; j+=3)
		{
		kernel_sgemv_t_3_lib8(m, mna, pA+j*bs, sda, x, y+j, alg);
		}
	for(; j<n-1; j+=2)
		{
		kernel_sgemv_t_2_lib8(m, mna, pA+j*bs, sda, x, y+j, alg);
		}
	for(; j<n; j++)
		{
		kernel_sgemv_t_1_lib8(m, mna, pA+j*bs, sda, x, y+j, alg);
		}

	}



void strmv_u_n_lib(int m, float *pA, int sda, float *x, float *y, int alg)
	{

	const int bs = 8;
	
	int j;
	
	j=0;
	for(; j<m-7; j+=8)
		{
		kernel_strmv_u_n_8_lib8(m-j, pA, x, y, alg);
		pA += sda*bs + 8*bs;
		x  += bs;
		y  += bs;
		}
	for(; j<m-3; j+=4)
		{
		kernel_strmv_u_n_4_lib8(m-j, pA, x, y, alg);
		pA += 4 + 4*bs;
		x  += 4;
		y  += 4;
		}
	for(; j<m-1; j+=2)
		{
		kernel_strmv_u_n_2_lib8(m-j, pA, x, y, alg);
		pA += 2 + 2*bs;
		x  += 2;
		y  += 2;
		}
	if(j<m)
		{
		if(alg==0)
			y[0] = pA[0+bs*0]*x[0];
		else if(alg==1)
			y[0] += pA[0+bs*0]*x[0];
		else
			y[0] -= pA[0+bs*0]*x[0];
		}

	}



void strmv_u_t_lib(int m, float *pA, int sda, float *x, float *y, int alg)
	{

	const int bs = 8;
	
	int j;
	
	float *ptrA;
	
	j=0;
	for(; j<m-7; j+=8)
		{
		kernel_strmv_u_t_8_lib8(j, pA, sda, x, y, alg);
		pA += 8*bs;
		y  += 8;
		}
	for(; j<m-3; j+=4)
		{
		kernel_strmv_u_t_4_lib8(j, pA, sda, x, y, alg);
		pA += 4*bs;
		y  += 4;
		}
	for(; j<m-1; j+=2) // keep for !!!
		{
		kernel_strmv_u_t_2_lib8(j, pA, sda, x, y, alg);
		pA += 2*bs;
		y  += 2;
		}
	if(j<m)
		{
		kernel_strmv_u_t_1_lib8(j, pA, sda, x, y, alg);
		}

	}



// it moves vertically across block
void ssymv_lib(int m, int offset, float *pA, int sda, float *x, float *y, int alg)
	{
	
	const int bs = 8;
	
	int mna = (bs-offset%bs)%bs;
	int ma = m - mna;

	int j, j0;
	
	if(alg==0)
		{
		for(j=0; j<m; j++)
			y[j] = 0.0;
		alg = 1;
		}
	
	if(mna>0)
		{
		j=0;
		for(; j<mna; j++)
			{
/*printf("\n1\n");*/
			kernel_ssymv_1_lib8(m-j, mna-j, pA+j+j*bs, sda, x+j, y+j, x+j, y+j, 1, alg);
			}
		pA += j + (sda-1)*bs + j*bs;
		x += j;
		y += j;
		}
	j=0;
	for(; j<ma-7; j+=8)
		{
/*printf("\n8 %d\n", ma-j);*/
		kernel_ssymv_4_lib8(ma-j-0, 8, pA+0+j*sda+(j+0)*bs, sda, x+(j+0), y+(j+0), x+(j+0), y+(j+0), 1, alg);
		kernel_ssymv_4_lib8(ma-j-4, 4, pA+4+j*sda+(j+4)*bs, sda, x+(j+4), y+(j+4), x+(j+4), y+(j+4), 1, alg);
		}
/*return;*/
	j0 = j;
	for(; j<ma-3; j+=4)
		{
/*printf("\n4 %d\n", ma-j);*/
		kernel_ssymv_4_lib8(ma-j, ma-j, pA+(j-j0)+j0*sda+j*bs, sda, x+j, y+j, x+j, y+j, 1, alg);
		}
	for(; j<ma-1; j+=2)
		{
/*printf("\n2\n");*/
		kernel_ssymv_2_lib8(ma-j, ma-j, pA+(j-j0)+j0*sda+j*bs, sda, x+j, y+j, x+j, y+j, 1, alg);
		}
	for(; j<ma; j++)
		{
/*printf("\n1\n");*/
		kernel_ssymv_1_lib8(ma-j, ma-j, pA+(j-j0)+j0*sda+j*bs, sda, x+j, y+j, x+j, y+j, 1, alg);
		}

	}



// it moves vertically across block
void smvmv_lib(int m, int n, int offset, float *pA, int sda, float *x_n, float *y_n, float *x_t, float *y_t, int alg)
	{
	
	const int bs = 8;

	int mna = (bs-offset%bs)%bs;

	int j;
	
	if(alg==0)
		{
		for(j=0; j<m; j++)
			y_n[j] = 0.0;
		for(j=0; j<n; j++)
			y_t[j] = 0.0;
		alg = 1;
		}
	
	j=0;
	for(; j<n-3; j+=4)
		{
		kernel_ssymv_4_lib8(m, mna, pA+j*bs, sda, x_n+j, y_n, x_t, y_t+j, 0, alg);
		}
	for(; j<n-1; j+=2)
		{
		kernel_ssymv_2_lib8(m, mna, pA+j*bs, sda, x_n+j, y_n, x_t, y_t+j, 0, alg);
		}
	for(; j<n; j++)
		{
		kernel_ssymv_1_lib8(m, mna, pA+j*bs, sda, x_n+j, y_n, x_t, y_t+j, 0, alg);
		}

	}



// the diagonal is inverted !!!
void strsv_sgemv_n_lib(int m, int n, float *pA, int sda, float *x)
	{
	
	const int bs = 8;
	
	int j;
	
	float *y;

	// blocks of 4 (pA is supposed to be properly aligned)
	y  = x;

	j = 0;
	for(; j<m-7; j+=8)
		{

		kernel_strsv_n_8_lib8(j, 8, pA, x, y); // j+8 !!!

		pA += bs*sda;
		y  += bs;

		}
	if(j<m) // !!! suppose that there are enough nx after !!! => x padded with enough zeros at the end !!!
		{

		kernel_strsv_n_8_lib8(j, m-j, pA, x, y); // j+4 !!!

		pA += bs*sda;
		y  += bs;
		j+=8;

		}
/*	for(; j<n-7; j+=8)*/
	for(; j<n; j+=8)
		{

		kernel_sgemv_n_8_lib8(m, pA, x, y, -1);

		pA += sda*bs;
		y  += bs;

		}
/*	for(; j<n-3; j+=4)*/
/*	for(; j<n; j+=4)*/
/*		{*/

/*		kernel_sgemv_n_4_lib4(m, pA, x, y, -1);*/

/*		pA += sda*bs;*/
/*		y  += bs;*/

/*		}*/
/*	for(; j<n-1; j+=2)*/
/*		{*/

/*		kernel_sgemv_n_2_lib4(m, pA, x, y, -1);*/

/*		pA += 2;*/
/*		y  += 2;*/

/*		}*/
/*	for(; j<n; j+=1)*/
/*		{*/

/*		kernel_sgemv_n_1_lib4(m, pA, x, y, -1);*/

/*		pA += 1;*/
/*		y  += 1;*/

/*		}*/

	}



// the diagonal is inverted !!!
void strsv_sgemv_t_lib(int n, int m, float *pA, int sda, float *x)
	{

	const int bs = 8;
	
	int j;
	
/*	float *y;*/
	
	j=0;
	if(n%8==1)
		{
		kernel_strsv_t_1_lib8(m-n+j+1, 4, pA+0+(n/bs)*bs*sda+(n-j-1)*bs, sda, x+n-j-1);
		j++;
		}
	else if(n%8==2)
		{
		kernel_strsv_t_2_lib8(m-n+j+2, 4, pA+0+(n/bs)*bs*sda+(n-j-2)*bs, sda, x+n-j-2);
		j+=2;
		}
	else if(n%8==3)
		{
		kernel_strsv_t_3_lib8(m-n+j+3, 4, pA+0+(n/bs)*bs*sda+(n-j-3)*bs, sda, x+n-j-3);
		j+=3;
		}
	else if(n%8==4)
		{
		kernel_strsv_t_4_lib8(m-n+j+4, 4, pA+0+(n/bs)*bs*sda+(n-j-4)*bs, sda, x+n-j-4);
		j+=4;
		}
	else if(n%8==5)
		{
		kernel_strsv_t_1_lib8(m-n+j+1, 0, pA+4+(n/bs)*bs*sda+(n-j-1)*bs, sda, x+n-j-1);
		j++;
		kernel_strsv_t_4_lib8(m-n+j+4, 4, pA+0+(n/bs)*bs*sda+(n-j-4)*bs, sda, x+n-j-4);
		j+=4;
		}
	else if(n%8==6)
		{
		kernel_strsv_t_2_lib8(m-n+j+2, 0, pA+4+(n/bs)*bs*sda+(n-j-2)*bs, sda, x+n-j-2);
		j+=2;
		kernel_strsv_t_4_lib8(m-n+j+4, 4, pA+0+(n/bs)*bs*sda+(n-j-4)*bs, sda, x+n-j-4);
		j+=4;
		}
	else if(n%8==7)
		{
		kernel_strsv_t_3_lib8(m-n+j+3, 0, pA+4+(n/bs)*bs*sda+(n-j-3)*bs, sda, x+n-j-3);
		j+=3;
		kernel_strsv_t_4_lib8(m-n+j+4, 4, pA+0+(n/bs)*bs*sda+(n-j-4)*bs, sda, x+n-j-4);
		j+=4;
		}
	for(; j<n-7; j+=8)
		{
		kernel_strsv_t_4_lib8(m-n+j+4, 0, pA+4+((n-j-8)/bs)*bs*sda+(n-j-4)*bs, sda, x+n-j-4);
		kernel_strsv_t_4_lib8(m-n+j+8, 4, pA+0+((n-j-8)/bs)*bs*sda+(n-j-8)*bs, sda, x+n-j-8);
		}

	}


