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

#include "../include/kernel_s_lib4.h"
#include "../include/block_size.h"



/* preforms                                          */
/* C  = A * B' (alg== 0)                             */
/* C += A * B' (alg== 1)                             */
/* C -= A * B' (alg==-1)                             */
/* where A, B and C are packed with block size 4     */
void sgemm_nt_lib(int m, int n, int k, float *pA, int sda, float *pB, int sdb, int alg, float *pC, int sdc)
	{

	const int bs = 4;

	int i, j, jj;
	
	i = 0;
#if defined(TARGET_CORTEX_A15) || defined(TARGET_CORTEX_A57)
	for(; i<m-16; i+=12)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
#if defined(TARGET_CORTEX_A57)
			kernel_sgemm_nt_12x4_lib4_new(k, &pA[0+i*sda], sda, &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], sdc, &pC[0+(j+0)*bs+i*sdc], sdc, alg, 0, 0);//return;
#else
			kernel_sgemm_nt_12x4_lib4(k, &pA[0+i*sda], &pA[0+(i+4)*sda], &pA[0+(i+8)*sda], &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc], &pC[0+(j+0)*bs+(i+8)*sdc], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc], &pC[0+(j+0)*bs+(i+8)*sdc], alg);//return;
#endif
			}
		jj = 0;
/*		for(; jj<n-j-1; jj+=2)*/
/*		for(; jj<n-j; jj+=2)*/
/*			{*/
/*			kernel_sgemm_nt_8x2_lib4(k, &pA[0+i*sda], &pA[0+(i+4)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+4)*sdc], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+4)*sdc], alg);*/
/*			}*/
/*		for(; jj<n-j; jj++)*/
/*			{*/
/*			kernel_sgemm_nt_8x1_lib4(k, &pA[0+i*sda], &pA[0+(i+4)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+4)*sdc], alg);*/
/*			}*/
		}
	if(m-i>12)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
#if defined(TARGET_CORTEX_A57)
			kernel_sgemm_nt_8x4_lib4_new(k, &pA[0+i*sda], sda, &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], sdc, &pC[0+(j+0)*bs+i*sdc], sdc, alg, 0, 0);
#else
			kernel_sgemm_nt_8x4_lib4(k, &pA[0+i*sda], &pA[0+(i+4)*sda], &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc], alg);
#endif
			}
		jj = 0;
/*		for(; jj<n-j-1; jj+=2)*/
/*		for(; jj<n-j; jj+=2)*/
/*			{*/
/*			kernel_sgemm_nt_8x2_lib4(k, &pA[0+i*sda], &pA[0+(i+4)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+4)*sdc], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+4)*sdc], alg);*/
/*			}*/
/*		for(; jj<n-j; jj++)*/
/*			{*/
/*			kernel_sgemm_nt_8x1_lib4(k, &pA[0+i*sda], &pA[0+(i+4)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+4)*sdc]s, alg);*/
/*			}*/
		i+=8;
		j = 0;
		for(; j<n-3; j+=4)
			{
#if defined(TARGET_CORTEX_A57)
			kernel_sgemm_nt_8x4_lib4_new(k, &pA[0+i*sda], sda, &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], sdc, &pC[0+(j+0)*bs+i*sdc], sdc, alg, 0, 0);
#else
			kernel_sgemm_nt_8x4_lib4(k, &pA[0+i*sda], &pA[0+(i+4)*sda], &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc], alg);
#endif
			}
		jj = 0;
/*		for(; jj<n-j-1; jj+=2)*/
/*		for(; jj<n-j; jj+=2)*/
/*			{*/
/*			kernel_sgemm_nt_8x2_lib4(k, &pA[0+i*sda], &pA[0+(i+4)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+4)*sdc], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+4)*sdc], alg);*/
/*			}*/
/*		for(; jj<n-j; jj++)*/
/*			{*/
/*			kernel_sgemm_nt_8x1_lib4(k, &pA[0+i*sda], &pA[0+(i+4)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+4)*sdc], alg);*/
/*			}*/
		i+=8;
		}
	if(i<m-8)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
#if defined(TARGET_CORTEX_A57)
			kernel_sgemm_nt_12x4_lib4_new(k, &pA[0+i*sda], sda, &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], sdc, &pC[0+(j+0)*bs+i*sdc], sdc, alg, 0, 0);//return;
#else
			kernel_sgemm_nt_12x4_lib4(k, &pA[0+i*sda], &pA[0+(i+4)*sda], &pA[0+(i+8)*sda], &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc], &pC[0+(j+0)*bs+(i+8)*sdc], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc], &pC[0+(j+0)*bs+(i+8)*sdc], alg);//return;
#endif
			}
		jj = 0;
/*		for(; jj<n-j-1; jj+=2)*/
/*		for(; jj<n-j; jj+=2)*/
/*			{*/
/*			kernel_sgemm_nt_8x2_lib4(k, &pA[0+i*sda], &pA[0+(i+4)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+4)*sdc], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+4)*sdc], alg);*/
/*			}*/
/*		for(; jj<n-j; jj++)*/
/*			{*/
/*			kernel_sgemm_nt_8x1_lib4(k, &pA[0+i*sda], &pA[0+(i+4)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+4)*sdc], alg);*/
/*			}*/
		i+=12;
		}
#endif
#if defined(TARGET_CORTEX_A15) || defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7) || defined(TARGET_X64_SSE3) || defined(TARGET_CORTEX_A57)
	for(; i<m-4; i+=8)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
#if defined(TARGET_CORTEX_A57)
			kernel_sgemm_nt_8x4_lib4_new(k, &pA[0+i*sda], sda, &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], sdc, &pC[0+(j+0)*bs+i*sdc], sdc, alg, 0, 0);
#else
			kernel_sgemm_nt_8x4_lib4(k, &pA[0+i*sda], &pA[0+(i+4)*sda], &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc], alg);
#endif
			}
		jj = 0;
/*		for(; jj<n-j-1; jj+=2)*/
/*		for(; jj<n-j; jj+=2)*/
/*			{*/
/*			kernel_sgemm_nt_8x2_lib4(k, &pA[0+i*sda], &pA[0+(i+4)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+4)*sdc], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+4)*sdc], alg);*/
/*			}*/
/*		for(; jj<n-j; jj++)*/
/*			{*/
/*			kernel_sgemm_nt_8x1_lib4(k, &pA[0+i*sda], &pA[0+(i+4)*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+jj)*bs+(i+4)*sdc], alg);*/
/*			}*/
		}
#endif
	for(; i<m-2; i+=4)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
#if defined(TARGET_CORTEX_A57)
			kernel_sgemm_nt_4x4_lib4_new(k, &pA[0+i*sda], &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+i*sdc], alg, 0, 0);
#else
			kernel_sgemm_nt_4x4_lib4(k, &pA[0+i*sda], &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+i*sdc], alg);
#endif
			}
		jj = 0;
/*		for(; jj<n-j-1; jj+=2)*/
/*		for(; jj<n-j; jj+=2)*/
/*			{*/
/*			kernel_sgemm_nt_4x2_lib4(k, &pA[0+i*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+0)*bs+i*sdc], alg);*/
/*			}*/
/*		for(; jj<n-j; jj++)*/
/*			{*/
/*			kernel_sgemm_nt_4x1_lib4(k, &pA[0+i*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], alg);*/
/*			}*/
		}
/*	for(; i<m; i+=2)*/
/*		{*/
/*		j = 0;*/
/*		for(; j<n-3; j+=4)*/
/*			{*/
/*			kernel_sgemm_nt_2x4_lib4(k, &pA[0+i*sda], &pB[0+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+i*sdc], alg);*/
/*			}*/
/*		jj = 0;*/
/*//		for(; jj<n-j-1; jj+=2)*/
/*		for(; jj<n-j; jj+=2)*/
/*			{*/
/*			kernel_sgemm_nt_2x2_lib4(k, &pA[0+i*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], &pC[0+(j+0)*bs+i*sdc], alg);*/
/*			}*/
/*		for(; jj<n-j; jj++)*/
/*			{*/
/*			kernel_sgemm_nt_4x1_lib4(k, &pA[0+i*sda], &pB[jj+j*sdb], &pC[0+(j+jj)*bs+i*sdc], alg);*/
/*			}*/
/*		}*/

	}



/* preforms                                          */
/* C  = A * B'                                       */
/* where A, B and C are packed with block size 4,    */
/* and B is upper triangular                         */
void strmm_lib(int m, int n, float *pA, int sda, float *pB, int sdb, float *pC, int sdc)
	{
	
	const int bs = 4;
	
	int i, j;
	
	i = 0;
#if defined(TARGET_CORTEX_A15)
	for(; i<m-16; i+=12)
/*	for(; i<m-8; i+=12)*/
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
/*printf("\ncazzo %d %d %d\n", m, n, n-j);*/
			kernel_strmm_nt_12x4_lib4(n-j-0, &pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pA[0+(j+0)*bs+(i+8)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc], &pC[0+(j+0)*bs+(i+8)*sdc]);
			}
		if(n-j==1)
			{
/*			corner_strmm_nt_8x1_lib4(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			corner_strmm_nt_4x1_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			corner_strmm_nt_4x1_lib4(&pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);
			corner_strmm_nt_4x1_lib4(&pA[0+(j+0)*bs+(i+8)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+8)*sdc]);
			}
		else if(n-j==2)
			{
/*			corner_strmm_nt_8x2_lib4(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			corner_strmm_nt_4x2_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			corner_strmm_nt_4x2_lib4(&pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);
			corner_strmm_nt_4x2_lib4(&pA[0+(j+0)*bs+(i+8)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+8)*sdc]);
			}
		else if(n-j==3)
			{
/*			corner_strmm_nt_8x3_lib4(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			corner_strmm_nt_4x3_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			corner_strmm_nt_4x3_lib4(&pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);
			corner_strmm_nt_4x3_lib4(&pA[0+(j+0)*bs+(i+8)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+8)*sdc]);
			}
		}
	if(m-i>12)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_strmm_nt_8x4_lib4(n-j-0, &pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);
			}
		if(n-j==1)
			{
/*			corner_strmm_nt_8x1_lib4(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			corner_strmm_nt_4x1_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			corner_strmm_nt_4x1_lib4(&pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);
			}
		else if(n-j==2)
			{
/*			corner_strmm_nt_8x2_lib4(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			corner_strmm_nt_4x2_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			corner_strmm_nt_4x2_lib4(&pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);
			}
		else if(n-j==3)
			{
/*			corner_strmm_nt_8x3_lib4(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			corner_strmm_nt_4x3_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			corner_strmm_nt_4x3_lib4(&pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);
			}
		i+=8;
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_strmm_nt_8x4_lib4(n-j-0, &pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);
			}
		if(n-j==1)
			{
/*			corner_strmm_nt_8x1_lib4(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			corner_strmm_nt_4x1_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			corner_strmm_nt_4x1_lib4(&pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);
			}
		else if(n-j==2)
			{
/*			corner_strmm_nt_8x2_lib4(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			corner_strmm_nt_4x2_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			corner_strmm_nt_4x2_lib4(&pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);
			}
		else if(n-j==3)
			{
/*			corner_strmm_nt_8x3_lib4(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			corner_strmm_nt_4x3_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			corner_strmm_nt_4x3_lib4(&pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);
			}
		i+=8;
		}
	if(i<m-8)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_strmm_nt_12x4_lib4(n-j-0, &pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pA[0+(j+0)*bs+(i+8)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc], &pC[0+(j+0)*bs+(i+8)*sdc]);
			}
		if(n-j==1)
			{
/*			corner_strmm_nt_8x1_lib4(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			corner_strmm_nt_4x1_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			corner_strmm_nt_4x1_lib4(&pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);
			corner_strmm_nt_4x1_lib4(&pA[0+(j+0)*bs+(i+8)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+8)*sdc]);
			}
		else if(n-j==2)
			{
/*			corner_strmm_nt_8x2_lib4(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			corner_strmm_nt_4x2_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			corner_strmm_nt_4x2_lib4(&pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);
			corner_strmm_nt_4x2_lib4(&pA[0+(j+0)*bs+(i+8)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+8)*sdc]);
			}
		else if(n-j==3)
			{
/*			corner_strmm_nt_8x3_lib4(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			corner_strmm_nt_4x3_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			corner_strmm_nt_4x3_lib4(&pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);
			corner_strmm_nt_4x3_lib4(&pA[0+(j+0)*bs+(i+8)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+8)*sdc]);
			}
		i+=12;
		}
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7) || defined(TARGET_CORTEX_A15) || defined(TARGET_X64_SSE3)
	for(; i<m-4; i+=8)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
/*printf("\ncazzo %d %d %d\n", m, n, n-j);*/
			kernel_strmm_nt_8x4_lib4(n-j-0, &pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);
/*			kernel_strmm_nt_4x4_lib4(n-j-0, &pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);*/
/*			kernel_strmm_nt_4x4_lib4(n-j-0, &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			}
		if(n-j==1)
			{
/*			corner_strmm_nt_8x1_lib4(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			corner_strmm_nt_4x1_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			corner_strmm_nt_4x1_lib4(&pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);
			}
		else if(n-j==2)
			{
/*			corner_strmm_nt_8x2_lib4(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			corner_strmm_nt_4x2_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			corner_strmm_nt_4x2_lib4(&pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);
			}
		else if(n-j==3)
			{
/*			corner_strmm_nt_8x3_lib4(&pA[0+(j+0)*bs+i*sda], &pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], &pC[0+(j+0)*bs+(i+4)*sdc]);*/
			corner_strmm_nt_4x3_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			corner_strmm_nt_4x3_lib4(&pA[0+(j+0)*bs+(i+4)*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+(i+4)*sdc]);
			}
		}
#endif
	for(; i<m; i+=4)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
/*printf("\ncazzo %d %d %d\n", m, n, n-j);*/
			kernel_strmm_nt_4x4_lib4(n-j-0, &pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			}
		if(n-j==1)
			{
			corner_strmm_nt_4x1_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			}
		else if(n-j==2)
			{
			corner_strmm_nt_4x2_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			}
		else if(n-j==3)
			{
			corner_strmm_nt_4x3_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			}
		}

	}



void ssyrk_spotrf_lib(int m, int n, int k, float *pA, int sda, float *pC, int sdc, float *diag)
	{
	const int bs = 4;
	const int d_ncl = S_NCL;
	const int k0 = (d_ncl-k%d_ncl)%d_ncl;
	
	int i, j;
	
/*	int n = m;*/
	
	float fact[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	j = 0;
	for(; j<n-2; j+=4)
		{
		i = j;
#if defined(TARGET_CORTEX_A15)
		if(i<m-8)
			{
/*			exit(1);*/
			kernel_ssyrk_spotrf_nt_12x4_lib4(k, j, &pA[i*sda], &pA[(i+4)*sda], &pA[(i+8)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+4)*sdc], &pC[j*bs+(i+8)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+4)*sda], &pA[(k0+k+j)*bs+(i+8)*sda], fact);
			i += 12;
			diag[j+0] = fact[0];
			diag[j+1] = fact[2];
			diag[j+2] = fact[5];
			diag[j+3] = fact[9];
			for(; i<m-16; i+=12)
				{
/*printf("\n12 l %d %d\n", m, i);*/
				kernel_sgemm_strsm_nt_12x4_lib4(k, j, &pA[i*sda], &pA[(i+4)*sda], &pA[(i+8)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+4)*sdc], &pC[j*bs+(i+8)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+4)*sda], &pA[(k0+k+j)*bs+(i+8)*sda], fact);
				}
			if(m-i>12)
				{
/*printf("\n8 8 %d %d\n", m, i);*/
				kernel_sgemm_strsm_nt_8x4_lib4(k, j, &pA[i*sda], &pA[(i+4)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+4)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+4)*sda], fact);
				i+=8;
				kernel_sgemm_strsm_nt_8x4_lib4(k, j, &pA[i*sda], &pA[(i+4)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+4)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+4)*sda], fact);
				i+=8;
				}
			if(i<m-8)
				{
/*printf("\n12 %d %d\n", m, i);*/
				kernel_sgemm_strsm_nt_12x4_lib4(k, j, &pA[i*sda], &pA[(i+4)*sda], &pA[(i+8)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+4)*sdc], &pC[j*bs+(i+8)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+4)*sda], &pA[(k0+k+j)*bs+(i+8)*sda], fact);
				i+=12;
				}
/*exit(1);*/
			for(; i<m-4; i+=8)
				{
				kernel_sgemm_strsm_nt_8x4_lib4(k, j, &pA[i*sda], &pA[(i+4)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+4)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+4)*sda], fact);
/*				kernel_sgemm_strsm_nt_4x4_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);*/
/*				kernel_sgemm_strsm_nt_4x4_lib4(k, j, &pA[(i+4)*sda], &pA[j*sda], &pC[j*bs+(i+4)*sdc], &pA[(k0+k+j)*bs+(i+4)*sda], fact);*/
				}
			for(; i<m; i+=4)
				{
				kernel_sgemm_strsm_nt_4x4_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);
				}
			}
		else if(i<m-4)
#else
		if(i<m-4)
#endif
			{
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7) || defined(TARGET_CORTEX_A15) || defined(TARGET_X64_SSE3)
			kernel_ssyrk_spotrf_nt_8x4_lib4(k, j, &pA[i*sda], &pA[(i+4)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+4)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+4)*sda], fact);
			i += 8;
#else
			kernel_ssyrk_spotrf_nt_4x4_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);
			i += 4;
#endif
			diag[j+0] = fact[0];
			diag[j+1] = fact[2];
			diag[j+2] = fact[5];
			diag[j+3] = fact[9];
#if defined(TARGET_CORTEX_A15)
#if 0
			for(; i<m-8; i+=12)
				{
				kernel_sgemm_strsm_nt_12x4_lib4(k, j, &pA[i*sda], &pA[(i+4)*sda], &pA[(i+8)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+4)*sdc], &pC[j*bs+(i+8)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+4)*sda], &pA[(k0+k+j)*bs+(i+8)*sda], fact);
/*				kernel_sgemm_strsm_nt_4x4_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);*/
/*				kernel_sgemm_strsm_nt_4x4_lib4(k, j, &pA[(i+4)*sda], &pA[j*sda], &pC[j*bs+(i+4)*sdc], &pA[(k0+k+j)*bs+(i+4)*sda], fact);*/
/*				kernel_sgemm_strsm_nt_4x4_lib4(k, j, &pA[(i+8)*sda], &pA[j*sda], &pC[j*bs+(i+8)*sdc], &pA[(k0+k+j)*bs+(i+8)*sda], fact);*/
				}
#endif
			for(; i<m-16; i+=12)
				{
/*printf("\n12 l %d %d\n", m, i);*/
				kernel_sgemm_strsm_nt_12x4_lib4(k, j, &pA[i*sda], &pA[(i+4)*sda], &pA[(i+8)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+4)*sdc], &pC[j*bs+(i+8)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+4)*sda], &pA[(k0+k+j)*bs+(i+8)*sda], fact);
				}
			if(m-i>12)
				{
/*printf("\n8 8 %d %d\n", m, i);*/
				kernel_sgemm_strsm_nt_8x4_lib4(k, j, &pA[i*sda], &pA[(i+4)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+4)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+4)*sda], fact);
				i+=8;
				kernel_sgemm_strsm_nt_8x4_lib4(k, j, &pA[i*sda], &pA[(i+4)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+4)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+4)*sda], fact);
				i+=8;
				}
			if(i<m-8)
				{
/*printf("\n12 %d %d\n", m, i);*/
				kernel_sgemm_strsm_nt_12x4_lib4(k, j, &pA[i*sda], &pA[(i+4)*sda], &pA[(i+8)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+4)*sdc], &pC[j*bs+(i+8)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+4)*sda], &pA[(k0+k+j)*bs+(i+8)*sda], fact);
				i+=12;
				}
/*exit(1);*/
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7) || defined(TARGET_CORTEX_A15) || defined(TARGET_X64_SSE3)
			for(; i<m-4; i+=8)
				{
				kernel_sgemm_strsm_nt_8x4_lib4(k, j, &pA[i*sda], &pA[(i+4)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+4)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+4)*sda], fact);
/*				kernel_sgemm_strsm_nt_4x4_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);*/
/*				kernel_sgemm_strsm_nt_4x4_lib4(k, j, &pA[(i+4)*sda], &pA[j*sda], &pC[j*bs+(i+4)*sdc], &pA[(k0+k+j)*bs+(i+4)*sda], fact);*/
				}
#endif
#if defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7) || defined(TARGET_CORTEX_A15) || defined(TARGET_X64_SSE3)
			for(; i<m; i+=4)
				{
				kernel_sgemm_strsm_nt_4x4_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);
				}
#else
			for(; i<m-2; i+=4)
				{
				kernel_sgemm_strsm_nt_4x4_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);
				}
			for(; i<m; i+=2)
				{
				kernel_sgemm_strsm_nt_2x4_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);
				}
#endif
			}
		else //if(i<m-2)
			{
			kernel_ssyrk_spotrf_nt_4x4_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);
			diag[j+0] = fact[0];
			diag[j+1] = fact[2];
			diag[j+2] = fact[5];
			diag[j+3] = fact[9];
/*s_print_mat(4, 4, &pA[(k0+k+j)*bs+i*sda], 4);*/
/*exit(1);*/
			}
		}
	for(; j<n; j+=2)
		{
		i = j;
		if(i<m-2)
			{
			kernel_ssyrk_spotrf_nt_4x2_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);
			diag[j+0] = fact[0];
			diag[j+1] = fact[2];
			i += 4;
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
			for(; i<m-4; i+=8)
				{
				kernel_sgemm_strsm_nt_8x2_lib4(k, j, &pA[i*sda], &pA[(i+4)*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pC[j*bs+(i+4)*sdc], &pA[(k0+k+j)*bs+i*sda], &pA[(k0+k+j)*bs+(i+4)*sda], fact);
				}
#endif
			for(; i<m-2; i+=4)
				{
				kernel_sgemm_strsm_nt_4x2_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);
				}
			for(; i<m; i+=2)
				{
				kernel_sgemm_strsm_nt_2x2_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);
				}
			}
		else //if(i<m)
			{
			kernel_ssyrk_spotrf_nt_2x2_lib4(k, j, &pA[i*sda], &pA[j*sda], &pC[j*bs+i*sdc], &pA[(k0+k+j)*bs+i*sda], fact);
			diag[j+0] = fact[0];
			diag[j+1] = fact[2];
			}
		}

	}



void sgemv_n_lib(int m, int n, float *pA, int sda, float *x, float *y, int alg) // pA has to be aligned !!!
	{
	
	const int bs = 4;
	
	int j;

	j=0;
/*	for(; j<n-7; j+=8)*/
	for(; j<m-4; j+=8)
		{
		kernel_sgemv_n_8_lib4(n, pA, pA+sda*bs, x, y, alg);
		pA += 2*sda*bs;
		y  += 2*bs;
		}
/*	for(; j<n-3; j+=4)*/
	for(; j<m; j+=4)
		{
		kernel_sgemv_n_4_lib4(n, pA, x, y, alg);
		pA += sda*bs;
		y  += bs;
		}
	// TODO small cases using mask !!!!!!!!!!!!!!!!!!!!!!!!!!!!
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
	
	const int bs = 4;
	
	int mna = (bs-offset%bs)%bs;
	
	int j;
	
	j=0;
	for(; j<n-7; j+=8)
		{
		kernel_sgemv_t_8_lib4(m, mna, pA+j*bs, sda, x, y+j, alg);
		}
	for(; j<n-3; j+=4)
		{
		kernel_sgemv_t_4_lib4(m, mna, pA+j*bs, sda, x, y+j, alg);
		}
	if(j<n)
		{
		if(j==n-1)
			{
			kernel_sgemv_t_1_lib4(m, mna, pA+j*bs, sda, x, y+j, alg);
			}
		if(j==n-2)
			{
			kernel_sgemv_t_2_lib4(m, mna, pA+j*bs, sda, x, y+j, alg);
			}
		if(j==n-3)
			{
			kernel_sgemv_t_3_lib4(m, mna, pA+j*bs, sda, x, y+j, alg);
			}
		}

	}



void strmv_u_n_lib(int m, float *pA, int sda, float *x, float *y, int alg)
	{

	const int bs = 4;
	
	int j;
	
	j=0;
	for(; j<m-7; j+=8)
		{
		kernel_strmv_u_n_8_lib4(m-j, pA, pA+sda*bs, x, y, alg);
		pA += 2*sda*bs + 2*4*bs;
		x  += 2*bs;
		y  += 2*bs;
		}
	for(; j<m-3; j+=4)
		{
		kernel_strmv_u_n_4_lib4(m-j, pA, x, y, alg);
		pA += sda*bs + 4*bs;
		x  += bs;
		y  += bs;
		}
/*	for(; j<m-1; j+=2)*/
/*		{*/
/*		kernel_strmv_u_n_2_lib4(m-j, pA, x, y, alg);*/
/*		pA += 2 + 2*bs;*/
/*		x  += 2;*/
/*		y  += 2;*/
/*		}*/
	if(j<m)
		{
		if(j==m-1)
			{
			if(alg==0)
				y[0] = pA[0+bs*0]*x[0];
			else if(alg==1)
				y[0] += pA[0+bs*0]*x[0];
			else
				y[0] -= pA[0+bs*0]*x[0];
			}
		else if(j==m-2)
			{
			if(alg==0)
				{
				y[0] = pA[0+bs*0]*x[0] + pA[0+bs*1]*x[1];
				y[1] = pA[1+bs*1]*x[1];
				}
			else if(alg==1)
				{
				y[0] += pA[0+bs*0]*x[0] + pA[0+bs*1]*x[1];
				y[1] += pA[1+bs*1]*x[1];
				}
			else
				{
				y[0] -= pA[0+bs*0]*x[0] + pA[0+bs*1]*x[1];
				y[1] -= pA[1+bs*1]*x[1];
				}
			}
		else if(j==m-3)
			{
			if(alg==0)
				{
				y[0] = pA[0+bs*0]*x[0] + pA[0+bs*1]*x[1] + pA[0+bs*2]*x[2];
				y[1] = pA[1+bs*1]*x[1] + pA[1+bs*2]*x[2];
				y[2] = pA[2+bs*2]*x[2];
				}
			else if(alg==1)
				{
				y[0] += pA[0+bs*0]*x[0] + pA[0+bs*1]*x[1] + pA[0+bs*2]*x[2];
				y[1] += pA[1+bs*1]*x[1] + pA[1+bs*2]*x[2];
				y[2] += pA[2+bs*2]*x[2];
				}
			else
				{
				y[0] -= pA[0+bs*0]*x[0] + pA[0+bs*1]*x[1] + pA[0+bs*2]*x[2];
				y[1] -= pA[1+bs*1]*x[1] + pA[1+bs*2]*x[2];
				y[2] -= pA[2+bs*2]*x[2];
				}
			}
		}

	}



void strmv_u_t_lib(int m, float *pA, int sda, float *x, float *y, int alg)
	{

	const int bs = 4;
	
	int j;
	
	float *ptrA;
	
	j=0;
	for(; j<m-7; j+=8)
		{
		kernel_strmv_u_t_8_lib4(j, pA, sda, x, y, alg);
		pA += 2*4*bs;
		y  += 2*bs;
		}
	for(; j<m-3; j+=4)
		{
		kernel_strmv_u_t_4_lib4(j, pA, sda, x, y, alg);
		pA += 4*bs;
		y  += bs;
		}
/*	for(; j<m-1; j+=2) // keep for !!!*/
/*		{*/
/*		kernel_strmv_u_t_2_lib4(j, pA, sda, x, y, alg);*/
/*		pA += 2*bs;*/
/*		y  += 2;*/
/*		}*/
	if(j<m)
		{
		if(j==m-1)
			{
			kernel_strmv_u_t_1_lib4(j, pA, sda, x, y, alg);
			}
		else if(j==m-2)
			{
			kernel_strmv_u_t_2_lib4(j, pA, sda, x, y, alg);
			}
		else if(j==m-3)
			{
			kernel_strmv_u_t_3_lib4(j, pA, sda, x, y, alg);
			}
		}

	}



// it moves vertically across block
void ssymv_lib(int m, int offset, float *pA, int sda, float *x, float *y, int alg)
	{
	
	const int bs = 4;
	
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
			kernel_ssymv_1_lib4(m-j, mna-j, pA+j+j*bs, sda, x+j, y+j, x+j, y+j, 1, alg);
			}
		pA += j + (sda-1)*bs + j*bs;
		x += j;
		y += j;
		}
	j=0;
	for(; j<ma-3; j+=4)
		{
		kernel_ssymv_4_lib4(ma-j, 0, pA+j*sda+j*bs, sda, x+j, y+j, x+j, y+j, 1, alg);
		}
	j0 = j;
	for(; j<ma-1; j+=2)
		{
		kernel_ssymv_2_lib4(ma-j, ma-j, pA+(j-j0)+j0*sda+j*bs, sda, x+j, y+j, x+j, y+j, 1, alg);
		}
	for(; j<ma; j++)
		{
		kernel_ssymv_1_lib4(ma-j, ma-j, pA+(j-j0)+j0*sda+j*bs, sda, x+j, y+j, x+j, y+j, 1, alg);
		}

	}



// it moves vertically across block
void smvmv_lib(int m, int n, int offset, float *pA, int sda, float *x_n, float *y_n, float *x_t, float *y_t, int alg)
	{
	
	const int bs = 4;

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
		kernel_ssymv_4_lib4(m, mna, pA+j*bs, sda, x_n+j, y_n, x_t, y_t+j, 0, alg);
		}
	for(; j<n-1; j+=2)
		{
		kernel_ssymv_2_lib4(m, mna, pA+j*bs, sda, x_n+j, y_n, x_t, y_t+j, 0, alg);
		}
	for(; j<n; j++)
		{
		kernel_ssymv_1_lib4(m, mna, pA+j*bs, sda, x_n+j, y_n, x_t, y_t+j, 0, alg);
		}

	}



// the diagonal is inverted !!!
void strsv_sgemv_n_lib(int m, int n, float *pA, int sda, float *x)
	{
	
	const int bs = 4;
	
	int j;
	
	float *y;

	// blocks of 4 (pA is supposed to be properly aligned)
	y  = x;

	j = 0;
	for(; j<m-7; j+=8)
		{

		kernel_strsv_n_8_lib4(j, pA, pA+bs*sda, x, y); // j+8 !!!

		pA += 2*bs*sda;
		y  += 2*bs;

		}
	if(j<m-3)
		{

		kernel_strsv_n_4_lib4(j, 4, pA, x, y); // j+4 !!!

		pA += bs*sda;
		y  += bs;
		j+=4;

		}
	if(j<m) // !!! suppose that there are enough nx after !!! => x padded with enough zeros at the end !!!
		{

		kernel_strsv_n_4_lib4(j, m-j, pA, x, y); // j+4 !!!

		pA += bs*sda;
		y  += bs;
		j+=4;

		}
/*	for(; j<n-7; j+=8)*/
	for(; j<n-4; j+=8)
		{

		kernel_sgemv_n_8_lib4(m, pA, pA+sda*bs, x, y, -1);

		pA += 2*sda*bs;
		y  += 2*bs;

		}
/*	for(; j<n-3; j+=4)*/
	for(; j<n; j+=4)
		{

		kernel_sgemv_n_4_lib4(m, pA, x, y, -1);

		pA += sda*bs;
		y  += bs;

		}
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
	
	const int bs = 4;
	
	int j;
	
/*	float *y;*/
	
	j=0;
	if(n%4==1)
		{
		kernel_strsv_t_1_lib4(m-n+j+1, pA+(n/bs)*bs*sda+(n-1)*bs, sda, x+n-j-1);
		j++;
		}
	else if(n%4==2)
		{
		kernel_strsv_t_2_lib4(m-n+j+2, pA+(n/bs)*bs*sda+(n-j-2)*bs, sda, x+n-j-2);
		j+=2;
		}
	else if(n%4==3)
		{
		kernel_strsv_t_3_lib4(m-n+j+3, pA+(n/bs)*bs*sda+(n-j-3)*bs, sda, x+n-j-3);
		j+=3;
		}
	for(; j<n-3; j+=4)
		{
		kernel_strsv_t_4_lib4(m-n+j+4, pA+((n-j-4)/bs)*bs*sda+(n-j-4)*bs, sda, x+n-j-4);
		}

	}



// transpose & align lower triangular matrix
void strtr_l_lib(int m, int offset, float *pA, int sda, float *pC, int sdc)
	{
	
	const int bs = 4;
	
	int mna = (bs-offset%bs)%bs;
	
	int j;
	
	j=0;
	for(; j<m-3; j+=4)
		{
		kernel_stran_4_lib4(m-j, mna, pA, sda, pC);
		pA += bs*(sda+bs);
		pC += bs*(sdc+bs);
		}
	if(j==m)
		{
		return;
		}
	else if(m-j==1)
		{
		pC[0] = pA[0];
		}
	else if(m-j==2)
		{
		corner_stran_2_lib4(mna, pA, sda, pC);
		}
	else // if(m-j==3)
		{
		corner_stran_3_lib4(mna, pA, sda, pC);
		}
	
	}

