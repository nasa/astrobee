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

#include <math.h>

#include "../include/kernel_d_lib4.h"
#include "../include/block_size.h"

#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
#include <mmintrin.h>
#include <xmmintrin.h>  // SSE
#include <emmintrin.h>  // SSE2
#include <pmmintrin.h>  // SSE3
#include <smmintrin.h>  // SSE4
#include <immintrin.h>  // AVX
#endif



#if ! defined(BLASFEO)
// test for the performance of the dgemm kernel
void dgemm_kernel_nt_lib(int m, int n, int k, double *pA, int sda, double *pB, int sdb, int alg, double *pC, int sdc, double *pD, int sdd, int tc, int td)
	{

	const int bs = 4;

	int i, j, jj;
	
	i = 0;
#if defined(TARGET_X64_AVX2)
	for(; i<m-8; i+=12)
		{
		j = 0;
		for(; j<n; j+=4)
			{
			kernel_dgemm_nt_12x4_lib4(k, &pA[0], sda, &pB[0], &pC[0], sdc, &pD[0], sdd, alg, tc, td);
			}
		}
	for(; i<m-4; i+=8)
		{
		j = 0;
		for(; j<n; j+=4)
			{
			kernel_dgemm_nt_8x4_vs_lib4(8, 4, k, &pA[0], sda, &pB[0], &pC[0], sdc, &pD[0], sdd, alg, tc, td);
			}
		}
#endif
#if defined(TARGET_X64_AVX)
	for(; i<m-4; i+=8)
		{
		j = 0;
		for(; j<n; j+=4)
			{
			kernel_dgemm_nt_8x4_lib4(k, &pA[0], sda, &pB[0], &pC[0], sdc, &pD[0], sdd, alg, tc, td);
			}
		}
#endif
	for(; i<m; i+=4)
		{
		j = 0;
		for(; j<n; j+=4)
			{
			kernel_dgemm_nt_4x4_vs_lib4(4, 4, k, &pA[0], &pB[0], &pC[0], &pD[0], alg, tc, td);
			}
		}


	}
#endif



#if ! defined(BLASFEO)
/* preforms                                          */
/* C  = A * B' (alg== 0)                             */
/* C += A * B' (alg== 1)                             */
/* C -= A * B' (alg==-1)                             */
/* where A, B and C are packed with block size 4     */
void dgemm_nt_lib(int m, int n, int k, double *pA, int sda, double *pB, int sdb, int alg, double *pC, int sdc, double *pD, int sdd, int tc, int td)
	{

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	int i, j, l;

#if 0
	i = 0;
//	for( ; i<m-4; i+=8)
//		{
//		for(j=0; j<n; j+=4)
//			{
//			kernel_dgemm_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, tc, td);
//			}
//		}
	for( ; i<m; i+=4)
		{
		j = 0;
		for( ; j<n-2; j+=4)
			{
			kernel_dgemm_nt_4x4_vs_lib4(4, 4, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, tc, td);
			//kernel_dgemm_nt_4x3_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, tc, td);
			//kernel_dgemm_nt_4x2_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, tc, td);
			//kernel_dgemm_nt_4x1_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, tc, td);
			}
		}
	return;
#endif
	
	if(tc==0)
		{
		if(td==0) // tc==0, td==0
			{

#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
	// low rank updates
	if (k<=4)
		{

		int alg2 = alg;
		
		double *pC2 = pC;
	
		if(k>0)
			{
			l = 0;
			for(; l<k-3; l+=4)
				{
				// rank 4 updates
				i = 0;
//				for(; i<m-7; i+=8)
//					{
//					kernel_dsyr4_8_lib4(n, n, pA+i*sda+l*bs, sda, pB+l*bs, sdb, alg2, pC2+i*sdc, sdc, pD+i*sdd, sdd);
//					}
				for(; i<m-3; i+=4)
					{
					kernel_dsyr4_4_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
					}
				if(m-i>0)
					{
					if(m-i==1)
						kernel_dsyr4_1_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
					else if(m-i==2)
						kernel_dsyr4_2_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
					else //if(m-i==3)
						kernel_dsyr4_3_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
					}
				pC2 = pD;
				if(alg2==0)
					{
					alg2=1;
					}
				}
			// clean up lower ranks
			if(k-l>0)
				{
				if(k-l==1)
					{
					// rank 1 update
					for(i=0; i<m-3; i+=4)
						{
						kernel_dsyr1_4_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						}
					if(m-i>0)
						{
						if(m-i==1)
							kernel_dsyr1_1_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						else if(m-i==2)
							kernel_dsyr1_2_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						else //if(m-i==3)
							kernel_dsyr1_3_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						}
					}
				else if(k-l==2)
					{
					// rank 2 update
					for(i=0; i<m-3; i+=4)
						{
						kernel_dsyr2_4_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						}
					if(m-i>0)
						{
						if(m-i==1)
							kernel_dsyr2_1_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						else if(m-i==2)
							kernel_dsyr2_2_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						else //if(m-i==3)
							kernel_dsyr2_3_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						}
					}
				else //if(k-l==3)
					{
					// rank 3 update
					for(i=0; i<m-3; i+=4)
						{
						kernel_dsyr3_4_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						}
					if(m-i>0)
						{
						if(m-i==1)
							kernel_dsyr3_1_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						else if(m-i==2)
							kernel_dsyr3_2_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						else //if(m-i==3)
							kernel_dsyr3_3_lib4(n, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						}
					}
				}
			}
		else // rank 0 update
			{
			for(i=0; i<m-3; i+=4)
				{
				kernel_dsyr0_4_lib4(n, n, alg, pC+i*sdc, pD+i*sdd);
				}
			if(m-i>0)
				{
				if(m-i==1)
					kernel_dsyr0_1_lib4(n, n, alg, pC+i*sdc, pD+i*sdd);
				else if(m-i==2)
					kernel_dsyr0_2_lib4(n, n, alg, pC+i*sdc, pD+i*sdd);
				else //if(m-i==3)
					kernel_dsyr0_3_lib4(n, n, alg, pC+i*sdc, pD+i*sdd);
				}
			}
		return;
		}
#endif

			i = 0;
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2) || defined(TARGET_CORTEX_A57)
#if defined(TARGET_X64_AVX2) //|| defined(TARGET_X64_AVX)
			for(; i<m-11; i+=12)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_12x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_12x4_vs_lib4(12, 3, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
						}
					else // n-j==1 || n-j==2
						{
						kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[(i+8)*sda], &pB[j*sdb], &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], alg, 0, 0);
						}
					}
				}
			if(i<m-8)
				{
				goto left_00_12;
				}
			if(i<m-4)
				{
				goto left_00_8;
				}
			if(i<m-2)
				{
				goto left_00_4;
				}
			if(i<m)
				{
				goto left_00_2;
				}

			//exit(1);
			return;
#endif
#if defined(TARGET_X64_AVX)
			for(; i<m-10; i+=8)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_8x4_vs_lib4(8, 3, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
						}
					else // n-j==1 || n-j==2
						{
						kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
						}
					}
				}
			if(m-i<3)
				{
				if(m-i==0)
					return;
				else
					goto left_00_2;
				}
			else
				{
				if(m-i<7)
					{
					if(m-i<5)
						goto left_00_4;
					else
						goto left_00_6;
					}
				else
					{
					if(m-i<9)
						goto left_00_8;
					else
						goto left_00_10;
					}
				}
#endif
#if defined(TARGET_CORTEX_A57)
			for(; i<m-7; i+=8)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						//kernel_dgemm_nt_8x4_vs_lib4(8, 3, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[(i+4)*sda], &pB[j*sdb], &pC[j*bs+(i+4)*sdc], &pD[j*bs+(i+4)*sdd], alg, 0, 0);
						}
					else // n-j==1 || n-j==2
						{
						//kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[(i+4)*sda], &pB[j*sdb], &pC[j*bs+(i+4)*sdc], &pD[j*bs+(i+4)*sdd], alg, 0, 0);
						}
					}
				}
			for(; i<m-3; i+=4)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
						}
					else
						{
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
						}
					}
				}
			if(m-i==0)
				{
				return;
				}
			else
				{
				if(m-i<3)
					{
					goto left_00_2;
					}
				else
					{
					goto left_00_4;
					}
				}
#endif
#else
			for(; i<m-3; i+=4)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
#if defined(TARGET_CORTEX_A15) || defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7) //|| defined(TARGET_CORTEX_A57)
					kernel_dgemm_nt_4x4_nn_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
#else
					kernel_dgemm_nt_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
#endif
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
						}
					else
						{
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
						}
					}
				}
			if(m>i)
				{
				if(m-i==3)
					{
					goto left_00_4;
					}
				else // m-i==2 || m-i==1
					{
					goto left_00_2;
					}
				}
#endif

			// common return if i==m
			return;

			// clean up loops definitions
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
			left_00_12:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_12x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				kernel_dgemm_nt_4x2_vs_lib4(m-i-8, n-j, k, &pA[(i+8)*sda], &pB[j*sdb], &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], alg, 0, 0);
				}
			return;
#endif

#if defined(TARGET_X64_AVX)
			left_00_10:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_10x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				kernel_dgemm_nt_2x2_vs_lib4(m-i-8, n-j, k, &pA[(i+8)*sda], &pB[j*sdb], &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], alg, 0, 0);
				}
			return;
#endif

#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
			left_00_8:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_8x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				}
			return;
#endif

#if defined(TARGET_X64_AVX)
			left_00_6:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_6x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				}
			return;
#endif

			left_00_4:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_4x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nt_4x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
				}
			return;

			left_00_2:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_4x2_vs_lib4(n-j, m-i, k, &pB[j*sdb], &pA[i*sda], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 1, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nt_2x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
				}
			return;



			}
		else // tc==0, td==1
			{



			i = 0;
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2) || defined(TARGET_CORTEX_A57)
#if defined(TARGET_X64_AVX2)
			for(; i<m-11; i+=12)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_12x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_12x4_vs_lib4(12, 3, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
						}
					else // n-j==1 || n-j==2
						{
						kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[(i+8)*sda], &pB[j*sdb], &pC[j*bs+(i+8)*sdc], &pD[(i+8)*bs+j*sdd], alg, 0, 1);
						}
					}
				}
			if(i<m-8)
				{
				goto left_01_12;
				}
			if(i<m-4)
				{
				goto left_01_8;
				}
			if(i<m)
				{
				goto left_01_4;
				}
#endif
#if defined(TARGET_X64_AVX)
			for(; i<m-10; i+=8)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_8x4_vs_lib4(8, 3, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
						}
					else
						{
						kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
						}
					}
				}
			if(m-i<3)
				{
				if(m-i==0)
					return;
				else
					goto left_01_2;
				}
			else
				{
				if(m-i<7)
					{
					if(m-i<5)
						goto left_01_4;
					else
						goto left_01_6;
					}
				else
					{
					if(m-i<9)
						goto left_01_8;
					else
						goto left_01_10;
					}
				}
#endif
#if defined(TARGET_CORTEX_A57)
			for(; i<m-7; i+=8)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						//kernel_dgemm_nt_8x4_vs_lib4(8, 3, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], alg, 0, 1);
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[(i+4)*sda], &pB[j*sdb], &pC[j*bs+(i+4)*sdc], &pD[(i+4)*bs+j*sdd], alg, 0, 1);
						}
					else
						{
						//kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], alg, 0, 1);
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[(i+4)*sda], &pB[j*sdb], &pC[j*bs+(i+4)*sdc], &pD[(i+4)*bs+j*sdd], alg, 0, 1);
						}
					}
				}
			for(; i<m-3; i+=4)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], alg, 0, 1);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], alg, 0, 1);
						}
					else
						{
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], alg, 0, 1);
						}
					}
				}
			if(m-i==0)
				{
				return;
				}
			else
				{
				if(m-i<3)
					{
					goto left_01_2;
					}
				else
					{
					goto left_01_4;
					}
				}
#endif
#else
			for(; i<m-3; i+=4)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], alg, 0, 1);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], alg, 0, 1);
						}
					else
						{
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], alg, 0, 1);
						}
					}
				}
			if(m>i)
				{
				if(m-i==3)
					{
					goto left_01_4;
					}
				else // m-i==2 || m-i==1
					{
					goto left_01_2;
					}
				}
#endif

			// common return if i==m
			return;

			// clean up loops definitions
#if defined(TARGET_X64_AVX2)
			left_01_12:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_12x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
				kernel_dgemm_nt_4x2_vs_lib4(m-i-8, n-j, k, &pA[(i+8)*sda], &pB[j*sdb], &pC[j*bs+(i+8)*sdc], &pD[(i+8)*bs+j*sdd], alg, 0, 1);
				}
			return;
#endif

#if defined(TARGET_X64_AVX)
			left_01_10:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_10x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
				kernel_dgemm_nt_2x2_vs_lib4(m-i-8, n-j, k, &pA[(i+8)*sda], &pB[j*sdb], &pC[j*bs+(i+8)*sdc], &pD[(i+8)*bs+j*sdd], alg, 0, 1);
				}
			return;
#endif

#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
			left_01_8:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_8x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
				}
			return;
#endif

#if defined(TARGET_X64_AVX)
			left_01_6:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_6x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 0, 1);
				}
			return;
#endif

			left_01_4:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_4x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], alg, 0, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nt_4x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], alg, 0, 1);
				}
			return;

			left_01_2:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_4x2_vs_lib4(n-j, m-i, k, &pB[j*sdb], &pA[i*sda], &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], alg, 1, 0); 
				}
			if(j<n)
				{
				kernel_dgemm_nt_2x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], alg, 0, 1);
				}
			return;



			}
		}
	else // tc==1
		{
		if(td==0) // tc==1, td==0
			{



			i = 0;
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2) || defined(TARGET_CORTEX_A57)
#if defined(TARGET_X64_AVX2)
			for(; i<m-11; i+=12)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_12x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_12x4_vs_lib4(12, 3, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
						}
					else // n-j==1 || n-j==2
						{
						kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[(i+8)*sda], &pB[j*sdb], &pC[(i+8)*bs+j*sdc], &pD[j*bs+(i+8)*sdd], alg, 1, 0);
						}
					}
				}
			if(i<m-8)
				{
				goto left_10_12;
				}
			if(i<m-4)
				{
				goto left_10_8;
				}
			if(i<m-2)
				{
				goto left_10_4;
				}
			if(i<m)
				{
				goto left_10_2;
				}
#endif
#if defined(TARGET_X64_AVX)
			for(; i<m-7; i+=8)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_8x4_vs_lib4(8, 3, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
						}
					else // n-j==2 || n-j==1
						{
						kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
						}
					}
				}
			if(m-i<3)
				{
				if(m-i==0)
					return;
				else
					goto left_10_2;
				}
			else
				{
				if(m-i<7)
					{
					if(m-i<5)
						goto left_10_4;
					else
						goto left_10_6;
					}
				else
					{
					if(m-i<9)
						goto left_10_8;
					else
						goto left_10_10;
					}
				}
#endif
#if defined(TARGET_CORTEX_A57)
			for(; i<m-7; i+=8)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						//kernel_dgemm_nt_8x4_vs_lib4(8, 3, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], alg, 1, 0);
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[(i+4)*sda], &pB[j*sdb], &pC[(i+4)*bs+j*sdc], &pD[j*bs+(i+4)*sdd], alg, 1, 0);
						}
					else // n-j==2 || n-j==1
						{
						//kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], alg, 1, 0);
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[(i+4)*sda], &pB[j*sdb], &pC[(i+4)*bs+j*sdc], &pD[j*bs+(i+4)*sdd], alg, 1, 0);
						}
					}
				}
			for(; i<m-3; i+=4)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], alg, 1, 0);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], alg, 1, 0);
						}
					else
						{
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], alg, 1, 0);
						}
					}
				}
			if(m-i==0)
				{
				return;
				}
			else
				{
				if(m-i<3)
					{
					goto left_10_2;
					}
				else
					{
					goto left_10_4;
					}
				}
#endif
#else
			for(; i<m-3; i+=4)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], alg, 1, 0);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], alg, 1, 0);
						}
					else
						{
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], alg, 1, 0);
						}
					}
				}
			if(m>i)
				{
				if(m-i==3)
					{
					goto left_10_4;
					}
				else // m-i==2 || m-i==1
					{
					goto left_10_2;
					}
				}
#endif

			// common return if i==m
			return;

			// clean up loops definitions
#if defined(TARGET_X64_AVX2)
			left_10_12:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_12x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
				kernel_dgemm_nt_4x2_vs_lib4(m-i-8, n-j, k, &pA[(i+8)*sda], &pB[j*sdb], &pC[(i+8)*bs+j*sdc], &pD[j*bs+(i+8)*sdd], alg, 1, 0);
				}
			return;
#endif

#if defined(TARGET_X64_AVX)
			left_10_10:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_10x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
				kernel_dgemm_nt_2x2_vs_lib4(m-i-8, n-j, k, &pA[(i+8)*sda], &pB[j*sdb], &pC[(i+8)*bs+j*sdc], &pD[j*bs+(i+8)*sdd], alg, 1, 0);
				}
			return;
#endif

#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
			left_10_8:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_8x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
				}
			return;
#endif

#if defined(TARGET_X64_AVX)
			left_10_6:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_6x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 1, 0);
				}
			return;
#endif

			left_10_4:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_4x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], alg, 1, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nt_4x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], alg, 1, 0);
				}
			return;

			left_10_2:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_4x2_vs_lib4(n-j, m-i, k, &pB[j*sdb], &pA[i*sda], &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], alg, 0, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nt_2x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], alg, 1, 0);
				}
			return;



			}
		else // tc==1, td==1
			{



			i = 0;
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2) || defined(TARGET_CORTEX_A57)
#if defined(TARGET_X64_AVX2)
			for(; i<m-11; i+=12)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_12x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_12x4_vs_lib4(12, 3, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
						}
					else // n-j==1 || n-j==2
						{
						kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[(i+8)*sda], &pB[j*sdb], &pC[(i+8)*bs+j*sdc], &pD[(i+8)*bs+j*sdd], alg, 1, 1);
						}
					}
				}
			if(i<m-8)
				{
				goto left_11_12;
				}
			if(i<m-4)
				{
				goto left_11_8;
				}
			if(i<m-2)
				{
				goto left_11_4;
				}
			if(i<m)
				{
				goto left_11_2;
				}
#endif
#if defined(TARGET_X64_AVX)
			for(; i<m-7; i+=8)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_8x4_vs_lib4(8, 3, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
						}
					else // n-j==1 || n-j==2
						{
						kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
						}
					}
				}
			if(m-i<3)
				{
				if(m-i==0)
					return;
				else
					goto left_11_2;
				}
			else
				{
				if(m-i<7)
					{
					if(m-i<5)
						goto left_11_4;
					else
						goto left_11_6;
					}
				else
					{
					if(m-i<9)
						goto left_11_8;
					else
						goto left_11_10;
					}
				}
#endif
#if defined(TARGET_CORTEX_A57)
			for(; i<m-7; i+=8)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						//kernel_dgemm_nt_8x4_vs_lib4(8, 3, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], alg, 1, 1);
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[(i+4)*sda], &pB[j*sdb], &pC[(i+4)*bs+j*sdc], &pD[(i+4)*bs+j*sdd], alg, 1, 1);
						}
					else // n-j==1 || n-j==2
						{
						//kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], alg, 1, 1);
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[(i+4)*sda], &pB[j*sdb], &pC[(i+4)*bs+j*sdc], &pD[(i+4)*bs+j*sdd], alg, 1, 1);
						}
					}
				}
			for(; i<m-3; i+=4)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], alg, 1, 1);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], alg, 1, 1);
						}
					else
						{
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], alg, 1, 1);
						}
					}
				}
			if(m-i==0)
				{
				return;
				}
			else
				{
				if(m-i<3)
					{
					goto left_11_2;
					}
				else
					{
					goto left_11_4;
					}
				}
#endif
#else
			for(; i<m-3; i+=4)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nt_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], alg, 1, 1);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], alg, 1, 1);
						}
					else
						{
						kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], alg, 1, 1);
						}
					}
				}
			if(m>i)
				{
				if(m-i==3)
					{
					goto left_11_4;
					}
				else // m-i==2 || m-i==1
					{
					goto left_11_2;
					}
				}
#endif

			// common return if i==m
			return;

			// clean up loops definitions
#if defined(TARGET_X64_AVX2)
			left_11_12:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_12x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
				kernel_dgemm_nt_4x2_vs_lib4(m-i-8, n-j, k, &pA[(i+8)*sda], &pB[j*sdb], &pC[(i+8)*bs+j*sdc], &pD[(i+8)*bs+j*sdd], alg, 1, 1);
				}
			return;
#endif

#if defined(TARGET_X64_AVX)
			left_11_10:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_10x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
				kernel_dgemm_nt_2x2_vs_lib4(m-i-8, n-j, k, &pA[(i+8)*sda], &pB[j*sdb], &pC[(i+8)*bs+j*sdc], &pD[(i+8)*bs+j*sdd], alg, 1, 1);
				}
			return;
#endif

#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
			left_11_8:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_8x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
				}
			return;
#endif

#if defined(TARGET_X64_AVX)
			left_11_6:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_6x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, alg, 1, 1);
				}
			return;
#endif

			left_11_4:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_4x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], alg, 1, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nt_4x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], alg, 1, 1);
				}
			return;

			left_11_2:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nt_4x2_vs_lib4(n-j, m-i, k, &pB[j*sdb], &pA[i*sda], &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], alg, 0, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nt_2x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], alg, 1, 1);
				}
			return;



			}
		}
	}
#endif



#if ! defined(BLASFEO)
/* preforms                                          */
/* C  = A * B (alg== 0)                             */
/* C += A * B (alg== 1)                             */
/* C -= A * B (alg==-1)                             */
/* where A, B and C are packed with block size 4     */
void dgemm_nn_lib(int m, int n, int k, double *pA, int sda, double *pB, int sdb, int alg, double *pC, int sdc, double *pD, int sdd, int tc, int td)
	{

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	int i, j, jj;
	
	if(tc==0)
		{
		if(td==0) // not transpose D
			{
			i = 0;
#if defined(TARGET_X64_AVX2) ||  defined(TARGET_X64_AVX)
#if defined(TARGET_X64_AVX2)
			for(; i<m-11; i+=12)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nn_12x4_lib4(k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, 0, 0);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nn_12x4_vs_lib4(12, 3, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, 0, 0);
						}
					else // n-j==1 || n-j==2
						{
						kernel_dgemm_nn_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, 0, 0);
						kernel_dgemm_nn_4x2_vs_lib4(4, n-j, k, &pA[(i+8)*sda], &pB[j*bs], sdb, alg, &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], 0, 0);
						}
					}
				}
			if(m-i<5)
				{
				if(m-i==0)
					return;
				else
					goto left_00_4;
				}
			else
				{
				if(m-i<9)
					goto left_00_8;
				else
					goto left_00_12;
				}

#elif defined(TARGET_X64_AVX)
			for(; i<m-7; i+=8)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nn_8x4_lib4(k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, 0, 0);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nn_8x4_vs_lib4(8, 3, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, 0, 0);
						}
					else // n-j==1 || n-j==2
						{
						kernel_dgemm_nn_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, 0, 0);
						}
					}
				}
			if(m-i==0)
				return;
			else
				{
				if(m-i<5)
					goto left_00_4;
				else
					goto left_00_8;
				}
#endif

			// common return if i==m
			return;

			// clean up loops definitions

#if defined(TARGET_X64_AVX2)
			left_00_12:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nn_12x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, 0, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nn_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, 0, 0);
				kernel_dgemm_nn_4x2_vs_lib4(m-i-8, n-j, k, &pA[(i+8)*sda], &pB[j*bs], sdb, alg, &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], 0, 0);
				}
			return;
#endif

			left_00_8:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nn_8x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, 0, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nn_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, 0, 0);
				}
			return;

			left_00_4:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nn_4x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], 0, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nn_4x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], 0, 0);
				}
			return;


#else
			for(; i<m-3; i+=4)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nn_4x4_lib4(k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], tc, td);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nn_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], tc, td);
						}
					else
						{
						kernel_dgemm_nn_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], tc, td);
						}
					}
				}
			if(m>i)
				{
				if(m-i==3)
					{
					j = 0;
					for(; j<n-2; j+=4)
						{
						kernel_dgemm_nn_4x4_vs_lib4(3, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], tc, td);
						}
					if(j<n)
						{
						kernel_dgemm_nn_4x2_vs_lib4(3, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], tc, td);
						}
					}
				else // m-i==2 || m-i==1
					{
					j = 0;
					for(; j<n-2; j+=4)
						{
						kernel_dgemm_nn_2x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], tc, td);
						}
					if(j<n)
						{
						kernel_dgemm_nn_2x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], tc, td);
						}
					}
				}
#endif
			}
		else // tc==0, td==1
			{
			i = 0;
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
#if defined(TARGET_X64_AVX2)
			for(; i<m-11; i+=12)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nn_12x4_lib4(k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, 0, 1);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nn_12x4_vs_lib4(12, 3, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, 0, 1);
						}
					else // n-j==1 || n-j==2
						{
						kernel_dgemm_nn_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, 0, 1);
						kernel_dgemm_nn_4x2_vs_lib4(4, n-j, k, &pA[(i+8)*sda], &pB[j*bs], sdb, alg, &pC[j*bs+(i+8)*sdc], &pD[(i+8)*bs+j*sdd], 0, 1);
						}
					}
				}
			if(m-i<5)
				{
				if(m-i==0)
					return;
				else
					goto left_10_4;
				}
			else
				{
				if(m-i<9)
					goto left_10_8;
				else
					goto left_10_12;
				}

#elif defined(TARGET_X64_AVX)
			for(; i<m-7; i+=8)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nn_8x4_lib4(k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, 0, 1);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nn_8x4_vs_lib4(8, 3, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, 0, 1);
						}
					else // n-j==1 || n-j==2
						{
						kernel_dgemm_nn_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, 0, 1);
						}
					}
				}
			if(m-i==0)
				return;
			else
				{
				if(m-i<5)
					goto left_01_4;
				else
					goto left_01_8;
				}

#endif

			// common return if i==m
			return;

			// clean up loops definitions
#if defined(TARGET_X64_AVX2)
			left_01_12:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nn_12x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, 0, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nn_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, 0, 1);
				kernel_dgemm_nn_4x2_vs_lib4(m-i-8, n-j, k, &pA[(i+8)*sda], &pB[j*bs], sdb, alg, &pC[j*bs+(i+8)*sdc], &pD[(i+8)*bs+j*sdd], 0, 1);
				}
			return;
#endif

			left_01_8:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nn_8x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, 0, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nn_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], sdc, &pD[i*bs+j*sdd], sdd, 0, 1);
				}
			return;

			left_01_4:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nn_4x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], 0, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nn_4x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], 0, 1);
				}
			return;

#else
			for(; i<m-3; i+=4)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nn_4x4_lib4(k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], tc, td);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nn_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], tc, td);
						}
					else
						{
						kernel_dgemm_nn_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], tc, td);
						}
					}
				}
			if(m>i)
				{
				if(m-i==3)
					{
					j = 0;
					for(; j<n-2; j+=4)
						{
						kernel_dgemm_nn_4x4_vs_lib4(3, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], tc, td);
						}
					if(j<n)
						{
						kernel_dgemm_nn_4x2_vs_lib4(3, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], tc, td);
						}
					}
				else // m-i==2 || m-i==1
					{
					j = 0;
					for(; j<n-2; j+=4)
						{
						kernel_dgemm_nn_2x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], tc, td);
						}
					if(j<n)
						{
						kernel_dgemm_nn_2x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[i*bs+j*sdd], tc, td);
						}
					}
				}
#endif
			}
		}
	else // tc==1
		{
		if(td==0) // not transpose D
			{
			i = 0;
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
#if defined(TARGET_X64_AVX2)
			for(; i<m-11; i+=12)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nn_12x4_lib4(k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, 1, 0);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nn_12x4_vs_lib4(12, 3, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, 1, 0);
						}
					else // n-j==1 || n-j==2
						{
						kernel_dgemm_nn_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, 1, 0);
						kernel_dgemm_nn_4x2_vs_lib4(4, n-j, k, &pA[(i+8)*sda], &pB[j*bs], sdb, alg, &pC[(i+8)*bs+j*sdc], &pD[j*bs+(i+8)*sdd], 1, 0);
						}
					}
				}
			if(m-i<5)
				{
				if(m-i==0)
					return;
				else
					goto left_01_4;
				}
			else
				{
				if(m-i<9)
					goto left_01_8;
				else
					goto left_01_12;
				}

#elif defined(TARGET_X64_AVX)
			for(; i<m-7; i+=8)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nn_8x4_lib4(k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, 1, 0);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nn_8x4_vs_lib4(8, 3, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, 1, 0);
						}
					else // n-j==1 || n-j==2
						{
						kernel_dgemm_nn_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, 1, 0);
						}
					}
				}
			if(m-i==0)
				return;
			else
				{
				if(m-i<5)
					goto left_10_4;
				else
					goto left_10_8;
				}

#endif

			// common return if i==m
			return;

			// clean up loops definitions

#if defined(TARGET_X64_AVX2)
			left_10_12:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nn_12x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, 1, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nn_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, 1, 0);
				kernel_dgemm_nn_4x2_vs_lib4(m-i-8, n-j, k, &pA[(i+8)*sda], &pB[j*bs], sdb, alg, &pC[(i+8)*bs+j*sdc], &pD[j*bs+(i+8)*sdd], 1, 0);
				}
			return;
#endif

			left_10_8:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nn_8x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, 1, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nn_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[j*bs+i*sdd], sdd, 1, 0);
				}
			return;

			left_10_4:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nn_4x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], 1, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nn_4x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], 1, 0);
				}
			return;

#else
			for(; i<m-3; i+=4)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nn_4x4_lib4(k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], tc, td);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nn_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], tc, td);
						}
					else
						{
						kernel_dgemm_nn_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], tc, td);
						}
					}
				}
			if(m>i)
				{
				if(m-i==3)
					{
					j = 0;
					for(; j<n-2; j+=4)
						{
						kernel_dgemm_nn_4x4_vs_lib4(3, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], tc, td);
						}
					if(j<n)
						{
						kernel_dgemm_nn_4x2_vs_lib4(3, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], tc, td);
						}
					}
				else // m-i==2 || m-i==1
					{
					j = 0;
					for(; j<n-2; j+=4)
						{
						kernel_dgemm_nn_2x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], tc, td);
						}
					if(j<n)
						{
						kernel_dgemm_nn_2x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[j*bs+i*sdd], tc, td);
						}
					}
				}
#endif
			}
		else // td==1
			{
			i = 0;
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
#if defined(TARGET_X64_AVX2)
			for(; i<m-11; i+=12)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nn_12x4_lib4(k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, 1, 1);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nn_12x4_vs_lib4(12, 3, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, 1, 1);
						}
					else // n-j==1 || n-j==2
						{
						kernel_dgemm_nn_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, 1, 1);
						kernel_dgemm_nn_4x2_vs_lib4(4, n-j, k, &pA[(i+8)*sda], &pB[j*bs], sdb, alg, &pC[(i+8)*bs+j*sdc], &pD[(i+8)*bs+j*sdd], 1, 1);
						}
					}
				}
			if(m-i<5)
				{
				if(m-i==0)
					return;
				else
					goto left_11_4;
				}
			else
				{
				if(m-i<9)
					goto left_11_8;
				else
					goto left_11_12;
				}

#elif defined(TARGET_X64_AVX)
			for(; i<m-7; i+=8)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nn_8x4_lib4(k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, 1, 1);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nn_8x4_vs_lib4(8, 3, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, 1, 1);
						}
					else // n-j==1 || n-j==2
						{
						kernel_dgemm_nn_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, 1, 1);
						}
					}
				}
			if(m-i==0)
				return;
			else
				{
				if(m-i<5)
					goto left_11_4;
				else
					goto left_11_8;
				}

#endif

			// common return if i==m
			return;

			// clean up loops definitions

#if defined(TARGET_X64_AVX2)
			left_11_12:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nn_12x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, 1, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nn_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, 1, 1);
				kernel_dgemm_nn_4x2_vs_lib4(m-i-8, n-j, k, &pA[(i+8)*sda], &pB[j*bs], sdb, alg, &pC[(i+8)*bs+j*sdc], &pD[(i+8)*bs+j*sdd], 1, 1);
				}
			return;
#endif

			left_11_8:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nn_8x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, 1, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nn_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], sdc, &pD[i*bs+j*sdd], sdd, 1, 1);
				}
			return;

			left_11_4:
			j = 0;
			for(; j<n-2; j+=4)
				{
				kernel_dgemm_nn_4x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], 1, 1);
				}
			if(j<n)
				{
				kernel_dgemm_nn_4x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], 1, 1);
				}
			return;

#else
			for(; i<m-3; i+=4)
				{
				j = 0;
				for(; j<n-3; j+=4)
					{
					kernel_dgemm_nn_4x4_lib4(k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], tc, td);
					}
				if(j<n)
					{
					if(n-j==3)
						{
						kernel_dgemm_nn_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], tc, td);
						}
					else
						{
						kernel_dgemm_nn_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], tc, td);
						}
					}
				}
			if(m>i)
				{
				if(m-i==3)
					{
					j = 0;
					for(; j<n-2; j+=4)
						{
						kernel_dgemm_nn_4x4_vs_lib4(3, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], tc, td);
						}
					if(j<n)
						{
						kernel_dgemm_nn_4x2_vs_lib4(3, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], tc, td);
						}
					}
				else // m-i==2 || m-i==1
					{
					j = 0;
					for(; j<n-2; j+=4)
						{
						kernel_dgemm_nn_2x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], tc, td);
						}
					if(j<n)
						{
						kernel_dgemm_nn_2x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[i*bs+j*sdc], &pD[i*bs+j*sdd], tc, td);
						}
					}
				}
#endif
			}
		}
	}
#endif



#if ! defined(BLASFEO)
/* preforms                                          */
/* C  = A * B'                                       */
/* where A, B and C are packed with block size 4,    */
/* and B is upper triangular                         */
void dtrmm_nt_u_lib(int m, int n, double *pA, int sda, double *pB, int sdb, double *pC, int sdc)
	{

	if(m<=0 || n<=0)
		return;
	
	const int bs = 4;
	
	int i, j;
	
	i = 0;
#if defined(TARGET_X64_AVX)
	for(; i<m-10; i+=8)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dtrmm_nt_u_8x4_lib4(n-j, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
			}
		if(n-j==1)
			{
			corner_dtrmm_nt_u_8x1_vs_lib4(8, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
			}
		else if(n-j==2)
			{
			corner_dtrmm_nt_u_8x2_vs_lib4(8, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
			}
		else if(n-j==3)
			{
			corner_dtrmm_nt_u_8x3_vs_lib4(8, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
			}
		}
	if(i<m)
		{
		if(m-i<7)
			{
			if(m-i<5)
				goto left_4;
			else
				goto left_6;
			}
		else
			{
			if(m-i<9)
				goto left_8;
			else
				goto left_10;
			}
		}

	left_10:
	j = 0;
	for(; j<n-3; j+=4)
		{
		kernel_dtrmm_nt_u_10x4_vs_lib4(m-i, n-j, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
		}
	if(n-j==1)
		{
		corner_dtrmm_nt_u_8x1_vs_lib4(8, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
		corner_dtrmm_nt_u_4x1_vs_lib4(m-i-8, &pA[j*bs+(i+8)*sda], &pB[j*bs+j*sdb], &pC[j*bs+(i+8)*sdc]);
		}
	else if(n-j==2)
		{
		corner_dtrmm_nt_u_8x2_vs_lib4(8, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
		corner_dtrmm_nt_u_4x2_vs_lib4(m-i-8, &pA[j*bs+(i+8)*sda], &pB[j*bs+j*sdb], &pC[j*bs+(i+8)*sdc]);
		}
	else if(n-j==3)
		{
		corner_dtrmm_nt_u_8x3_vs_lib4(8, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
		corner_dtrmm_nt_u_4x3_vs_lib4(m-i-8, &pA[j*bs+(i+8)*sda], &pB[j*bs+j*sdb], &pC[j*bs+(i+8)*sdc]);
		}
	return;

	left_8:
	j = 0;
	for(; j<n-3; j+=4)
		{
		kernel_dtrmm_nt_u_8x4_vs_lib4(m-i, n-j, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
		}
	if(n-j==1)
		{
		corner_dtrmm_nt_u_8x1_vs_lib4(m-i, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
		}
	else if(n-j==2)
		{
		corner_dtrmm_nt_u_8x2_vs_lib4(m-i, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
		}
	else if(n-j==3)
		{
		corner_dtrmm_nt_u_8x3_vs_lib4(m-i, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
		}
	return;

	left_6:
	j = 0;
	for(; j<n-3; j+=4)
		{
		kernel_dtrmm_nt_u_6x4_vs_lib4(m-i, n-j, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
		}
	if(n-j==1)
		{
		corner_dtrmm_nt_u_8x1_vs_lib4(m-i, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
		}
	else if(n-j==2)
		{
		corner_dtrmm_nt_u_8x2_vs_lib4(m-i, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
		}
	else if(n-j==3)
		{
		corner_dtrmm_nt_u_8x3_vs_lib4(m-i, &pA[j*bs+i*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+i*sdc], sdc);
		}
	return;

	left_4:
	j = 0;
	for(; j<n-3; j+=4)
		{
		kernel_dtrmm_nt_u_4x4_vs_lib4(m-i, n-j, &pA[j*bs+i*sda], &pB[j*bs+j*sdb], &pC[j*bs+i*sdc]);
		}
	if(n-j==1)
		{
		corner_dtrmm_nt_u_4x1_vs_lib4(m-i, &pA[j*bs+i*sda], &pB[j*bs+j*sdb], &pC[j*bs+i*sdc]);
		}
	else if(n-j==2)
		{
		corner_dtrmm_nt_u_4x2_vs_lib4(m-i, &pA[j*bs+i*sda], &pB[j*bs+j*sdb], &pC[j*bs+i*sdc]);
		}
	else if(n-j==3)
		{
		corner_dtrmm_nt_u_4x3_vs_lib4(m-i, &pA[j*bs+i*sda], &pB[j*bs+j*sdb], &pC[j*bs+i*sdc]);
		}
	return;

#else
#if defined(TARGET_X64_AVX2)
	for(; i<m-8; i+=12)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dtrmm_nt_u_12x4_lib4(n-j-0, &pA[0+(j+0)*bs+i*sda], sda, &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], sdc);
			}
		if(n-j==1)
			{
			corner_dtrmm_nt_u_12x1_lib4(&pA[j*bs+(i+0)*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+(i+0)*sdc], sdc);
			}
		else if(n-j==2)
			{
			corner_dtrmm_nt_u_12x2_lib4(&pA[j*bs+(i+0)*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+(i+0)*sdc], sdc);
			}
		else if(n-j==3)
			{
			corner_dtrmm_nt_u_12x3_lib4(&pA[j*bs+(i+0)*sda], sda, &pB[j*bs+j*sdb], &pC[j*bs+(i+0)*sdc], sdc);
			}
		}
#endif
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
	for(; i<m-4; i+=8)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dtrmm_nt_u_8x4_lib4(n-j-0, &pA[0+(j+0)*bs+i*sda], sda, &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], sdc);
			}
		if(n-j==1)
			{
			corner_dtrmm_nt_u_8x1_lib4(&pA[0+(j+0)*bs+i*sda], sda, &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], sdc);
			}
		else if(n-j==2)
			{
			corner_dtrmm_nt_u_8x2_lib4(&pA[0+(j+0)*bs+i*sda], sda, &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], sdc);
			}
		else if(n-j==3)
			{
			corner_dtrmm_nt_u_8x3_lib4(&pA[0+(j+0)*bs+i*sda], sda, &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc], sdc);
			}
		}
#endif
	for(; i<m; i+=4)
		{
		j = 0;
		for(; j<n-3; j+=4)
			{
			kernel_dtrmm_nt_u_4x4_lib4(n-j-0, &pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			}
		if(n-j==1)
			{
			corner_dtrmm_nt_u_4x1_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			}
		else if(n-j==2)
			{
			corner_dtrmm_nt_u_4x2_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			}
		else if(n-j==3)
			{
			corner_dtrmm_nt_u_4x3_lib4(&pA[0+(j+0)*bs+i*sda], &pB[0+(j+0)*bs+j*sdb], &pC[0+(j+0)*bs+i*sdc]);
			}
		}
#endif

	}
#endif



#if ! defined(BLASFEO)
/* preforms                                          */
/* C  = A * B'                                       */
/* where A, B and C are packed with block size 4,    */
/* and B is lower triangular                         */
void dtrmm_nt_l_lib(int m, int n, double *pA, int sda, double *pB, int sdb, double *pC, int sdc)
	{

	if(m<=0 || n<=0)
		return;
	
	const int bs = 4;
	
	int i, j;
	
	i=0;
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
	for( ; i<m-4; i+=8)
		{
		j=0;
		for( ; j<n-2; j+=4)
			{
			kernel_dtrmm_nt_l_8x4_lib4(j+4, &pA[i*sda], sda, &pB[j*sdb], &pC[i*sdc+j*bs], sdc);
			}
		if(j<n)
			{
			kernel_dtrmm_nt_l_8x2_lib4(j+2, &pA[i*sda], sda, &pB[j*sdb], &pC[i*sdc+j*bs], sdc);
			}
		}
	for( ; i<m; i+=4)
		{
		j=0;
		for( ; j<n-2; j+=4)
			{
			kernel_dtrmm_nt_l_4x4_lib4(j+4, &pA[i*sda], &pB[j*sdb], &pC[i*sdc+j*bs]);
			}
		if(j<n)
			{
			kernel_dtrmm_nt_l_4x2_lib4(j+2, &pA[i*sda], &pB[j*sdb], &pC[i*sdc+j*bs]);
			}
		}
#else
	for( ; i<m-2; i+=4)
		{
		j=0;
		for( ; j<n-2; j+=4)
			{
			kernel_dtrmm_nt_l_4x4_lib4(j+4, &pA[i*sda], &pB[j*sdb], &pC[i*sdc+j*bs]);
			}
		if(j<n)
			{
			kernel_dtrmm_nt_l_4x2_lib4(j+2, &pA[i*sda], &pB[j*sdb], &pC[i*sdc+j*bs]);
			}
		}
	if(i<m)
		{
		j=0;
		for( ; j<n-2; j+=4)
			{
			kernel_dtrmm_nt_l_2x4_lib4(j+4, &pA[i*sda], &pB[j*sdb], &pC[i*sdc+j*bs]);
			}
		if(j<n)
			{
			kernel_dtrmm_nt_l_2x2_lib4(j+2, &pA[i*sda], &pB[j*sdb], &pC[i*sdc+j*bs]);
			}
		}
#endif

	}
#endif



#if ! defined(BLASFEO)
void dsyrk_nt_lib(int m, int n, int k, double *pA, int sda, double *pB, int sdb, int alg, double *pC, int sdc, double *pD, int sdd)
	{

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	int i, j, l;

#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
	// low rank updates
	if (k<=4)
		{

		int alg2 = alg;
		
		double *pC2 = pC;
	
		if(k>0)
			{
			l = 0;
			for(; l<k-3; l+=4)
				{
				// rank 4 updates
				i = 0;
				for(; i<m-3; i+=4)
					{
					kernel_dsyr4_4_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
					}
				if(m-i>0)
					{
					if(m-i==1)
						kernel_dsyr4_1_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
					else if(m-i==2)
						kernel_dsyr4_2_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
					else //if(m-i==3)
						kernel_dsyr4_3_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
					}
				pC2 = pD;
				if(alg2==0)
					{
					alg2=1;
					}
				}
			// clean up lower ranks
			if(k-l>0)
				{
				if(k-l==1)
					{
					// rank 1 update
					for(i=0; i<m-3; i+=4)
						{
						kernel_dsyr1_4_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						}
					if(m-i>0)
						{
						if(m-i==1)
							kernel_dsyr1_1_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						else if(m-i==2)
							kernel_dsyr1_2_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						else //if(m-i==3)
							kernel_dsyr1_3_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						}
					}
				else if(k-l==2)
					{
					// rank 2 update
					for(i=0; i<m-3; i+=4)
						{
						kernel_dsyr2_4_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						}
					if(m-i>0)
						{
						if(m-i==1)
							kernel_dsyr2_1_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						else if(m-i==2)
							kernel_dsyr2_2_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						else //if(m-i==3)
							kernel_dsyr2_3_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						}
					}
				else //if(k-l==3)
					{
					// rank 3 update
					for(i=0; i<m-3; i+=4)
						{
						kernel_dsyr3_4_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						}
					if(m-i>0)
						{
						if(m-i==1)
							kernel_dsyr3_1_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						else if(m-i==2)
							kernel_dsyr3_2_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						else //if(m-i==3)
							kernel_dsyr3_3_lib4(i, n, pA+i*sda+l*bs, pB+l*bs, sdb, alg2, pC2+i*sdc, pD+i*sdd);
						}
					}
				}
			}
		else // rank 0 update
			{
			for(i=0; i<m-3; i+=4)
				{
				kernel_dsyr0_4_lib4(i, n, alg, pC+i*sdc, pD+i*sdd);
				}
			if(m-i>0)
				{
				if(m-i==1)
					kernel_dsyr0_1_lib4(i, n, alg, pC+i*sdc, pD+i*sdd);
				else if(m-i==2)
					kernel_dsyr0_2_lib4(i, n, alg, pC+i*sdc, pD+i*sdd);
				else //if(m-i==3)
					kernel_dsyr0_3_lib4(i, n, alg, pC+i*sdc, pD+i*sdd);
				}
			}
		return;
		}

#if 0
	if(k<=4)
		{
		if(k==0)
			{
			for(i=0; i<m-3; i+=4)
				{
				kernel_dsyr0_4_lib4(i, n, alg, pC+i*sdc, pD+i*sdd);
				}
			if(m-i>0)
				{
				if(m-i==1)
					kernel_dsyr0_1_lib4(i, n, alg, pC+i*sdc, pD+i*sdd);
				else if(m-i==2)
					kernel_dsyr0_2_lib4(i, n, alg, pC+i*sdc, pD+i*sdd);
				else //if(m-i==3)
					kernel_dsyr0_3_lib4(i, n, alg, pC+i*sdc, pD+i*sdd);
				}
			}
		else if(k==1)
			{
			for(i=0; i<m-3; i+=4)
				{
				kernel_dsyr1_4_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				}
			if(m-i>0)
				{
				if(m-i==1)
					kernel_dsyr1_1_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				else if(m-i==2)
					kernel_dsyr1_2_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				else //if(m-i==3)
					kernel_dsyr1_3_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				}
			}
		else if(k==2)
			{
			for(i=0; i<m-3; i+=4)
				{
				kernel_dsyr2_4_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				}
			if(m-i>0)
				{
				if(m-i==1)
					kernel_dsyr2_1_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				else if(m-i==2)
					kernel_dsyr2_2_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				else //if(m-i==3)
					kernel_dsyr2_3_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				}
			}
		else if(k==3)
			{
			for(i=0; i<m-3; i+=4)
				{
				kernel_dsyr3_4_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				}
			if(m-i>0)
				{
				if(m-i==1)
					kernel_dsyr3_1_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				else if(m-i==2)
					kernel_dsyr3_2_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				else //if(m-i==3)
					kernel_dsyr3_3_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				}
			}
		else // if(k==4)
			{
			for(i=0; i<m-3; i+=4)
				{
				kernel_dsyr3_4_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				}
			if(m-i>0)
				{
				if(m-i==1)
					kernel_dsyr3_1_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				else if(m-i==2)
					kernel_dsyr3_2_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				else //if(m-i==3)
					kernel_dsyr3_3_lib4(i, n, pA+i*sda, pB, sdb, alg, pC+i*sdc, pD+i*sdd);
				}
			}
		return;
		}
#endif

#endif

	i = 0;
#if defined(TARGET_X64_AVX2)
	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i && j<n-3; j+=4)
			{
			kernel_dgemm_nt_12x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
			}
		if(j<i) // dgemm
			{
			if(n-j==3)
				{
				kernel_dgemm_nt_12x4_vs_lib4(12, 3, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				}
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[(i+8)*sda], &pB[j*sdb], &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], alg, 0, 0);
				}
			}
		else // dsyrk
			{
			if(j<n-2)
				{
				kernel_dsyrk_nt_12x4_vs_lib4(12, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg);
				if(j<n-10)
					{
					kernel_dsyrk_nt_8x8_vs_lib4(8, n-j-4, k, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], sdb, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, alg);
					}
				else if(j<n-6)
					{
					kernel_dsyrk_nt_8x4_vs_lib4(8, n-j-4, k, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, alg);
					//if(j<n-10)
					//	{
					//	kernel_dsyrk_nt_4x4_lib4(k, &pA[(i+8)*sda], &pB[(j+8)*sdb], &pC[(j+8)*bs+(i+8)*sdc], &pD[(j+8)*bs+(i+8)*sdd], alg);
					//	}
					//else 
					if(j<n-8)
						{
						kernel_dsyrk_nt_4x2_vs_lib4(4, n-j-8, k, &pA[(i+8)*sda], &pB[(j+8)*sdb], &pC[(j+8)*bs+(i+8)*sdc], &pD[(j+8)*bs+(i+8)*sdd], alg);
						}
					}
				else if(j<n-4)
					{
					kernel_dsyrk_nt_8x2_vs_lib4(8, n-j-4, k, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, alg);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg);
				kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[(i+4)*sda], sda, &pB[j*sdb], &pC[j*bs+(i+4)*sdc], sdc, &pD[j*bs+(i+4)*sdd], sdd, alg, 0, 0);
				}
			}
		}
	if(i<m-8)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_nt_12x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
			}
		if(j<i) // dgemm
			{
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				kernel_dgemm_nt_4x2_vs_lib4(m-i-8, n-j, k, &pA[(i+8)*sda], &pB[j*sdb], &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], alg, 0, 0);
				}
			}
		else // dsyrk
			{
			if(j<n-2)
				{
				kernel_dsyrk_nt_12x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg);
				if(j<n-10)
					{
					kernel_dsyrk_nt_8x8_vs_lib4(m-i-4, n-j-4, k, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], sdb, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, alg);
					}
				else if(j<n-6)
					{
					kernel_dsyrk_nt_8x4_vs_lib4(m-i-4, n-j-4, k, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, alg);
					//if(j<n-10)
					//	{
					//	kernel_dsyrk_nt_4x4_lib4(k, &pA[(i+8)*sda], &pB[(j+8)*sdb], &pC[(j+8)*bs+(i+8)*sdc], &pD[(j+8)*bs+(i+8)*sdd], alg);
					//	}
					//else 
					if(j<n-8)
						{
						kernel_dsyrk_nt_4x2_vs_lib4(m-i-8, n-j-8, k, &pA[(i+8)*sda], &pB[(j+8)*sdb], &pC[(j+8)*bs+(i+8)*sdc], &pD[(j+8)*bs+(i+8)*sdd], alg);
						}
					}
				else if(j<n-4)
					{
					kernel_dsyrk_nt_8x2_vs_lib4(m-i-4, n-j-4, k, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, alg);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg);
				kernel_dgemm_nt_8x2_vs_lib4(m-i-4, n-j, k, &pA[(i+4)*sda], sda, &pB[j*sdb], &pC[j*bs+(i+4)*sdc], sdc, &pD[j*bs+(i+4)*sdd], sdd, alg, 0, 0);
				}
			}
		i += 12;
		}
#endif
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i && j<n-3; j+=4)
			{
#if defined(TARGET_X64_AVX)
			kernel_dgemm_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
#endif
#if defined(TARGET_X64_AVX2)
			kernel_dgemm_nt_8x4_vs_lib4(8, 4, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
#endif
			}
		if(j<i) // dgemm
			{
			if(j<n)
				{
				if(j==n-3)
					{
					kernel_dgemm_nt_8x4_vs_lib4(8, 3, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
					}
				else
					{
					kernel_dgemm_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
					}
				}
			}
		else // dsyrk
			{
			if(j<n-2)
				{
				kernel_dsyrk_nt_8x4_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg);
				if(j<n-6)
					{
					kernel_dsyrk_nt_4x4_vs_lib4(4, n-j-4, k, &pA[(i+4)*sda], &pB[(j+4)*sdb], &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], alg);
					}
				else if(j<n-4)
					{
					kernel_dsyrk_nt_4x2_vs_lib4(4, n-j-4, k, &pA[(i+4)*sda], &pB[(j+4)*sdb], &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], alg);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_nt_8x2_vs_lib4(8, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg);
				}
			}
		}
	if(i<m-4)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_nt_8x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
			}
		if(j<i) // dgemm
			{
			if(j<n)
				{
				kernel_dgemm_nt_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				}
			}
		else // dsyrk
			{
			if(j<n-2)
				{
				kernel_dsyrk_nt_8x4_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg);
				if(j<n-6)
					{
					kernel_dsyrk_nt_4x4_vs_lib4(m-i-4, n-j-4, k, &pA[(i+4)*sda], &pB[(j+4)*sdb], &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], alg);
					}
				else if(j<n-4)
					{
					kernel_dsyrk_nt_4x2_vs_lib4(m-i-4, n-j-4, k, &pA[(i+4)*sda], &pB[(j+4)*sdb], &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], alg);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_nt_8x2_vs_lib4(m-i, n-j, k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg);
				}
			}
		i += 8;
		}
	if(i<m)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_nt_4x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
			}
		if(j<i) // dgemm
			{
			if(j<n)
				{
				kernel_dgemm_nt_4x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
				}
			}
		else // dsyrk
			{
			if(j<n-2)
				{
				kernel_dsyrk_nt_4x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg);
				}
			else if(j<n)
				{
				kernel_dsyrk_nt_4x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg);
				}
			}
		//i += 4;
		}

#else
	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<i && j<n-3; j+=4)
			{
			kernel_dgemm_nt_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
			}
		if(j<i) // dgemm
			{
			if(j<n)
				{
				if(n-j==3)
					{
					kernel_dgemm_nt_4x4_vs_lib4(4, 3, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
					}
				else
					{
					kernel_dgemm_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
					}
				}
			}
		else // dsyrk
			{
			if(j<n-2)
				{
				kernel_dsyrk_nt_4x4_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg);
				}
			else if(j<n)
				{
				kernel_dsyrk_nt_4x2_vs_lib4(4, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg);
				}
			}
		}
	if(i<m)
		{
		if(m-i==3)
			{
			j = 0;
			for(; j<i && j<n-2; j+=4)
				{
				kernel_dgemm_nt_4x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
				}
			if(j<i) // dgemm
				{
				if(j<n)
					{
					kernel_dgemm_nt_4x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
					}
				}
			else // dsyrk
				{
				if(j<n-2)
					{
					kernel_dsyrk_nt_4x4_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg);
					}
				else if(j<n)
					{
					kernel_dsyrk_nt_4x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg);
					}
				}
			// i += 4;
			}
		else // m-i==2 || m-i==1
			{
			j = 0;
			for(; j<i && j<n-2; j+=4)
				{
				kernel_dgemm_nt_4x2_vs_lib4(n-j, m-i, k, &pB[j*sdb], &pA[i*sda], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 1, 1);
				}
			if(j<i) // dgemm
				{
				if(j<n)
					{
					kernel_dgemm_nt_2x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
					}
				}
			else // dsyrk
				{
				if(j<n)
					{
					kernel_dsyrk_nt_2x2_vs_lib4(m-i, n-j, k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg);
					}
				}
			// i += 2;
			}
		}
#endif

	}
#endif



#if ! defined(BLASFEO)
void dsyrk_nn_lib(int m, int n, int k, double *pA, int sda, double *pB, int sdb, int alg, double *pC, int sdc, double *pD, int sdd)
	{

	if(m<=0 || n<=0)
		return;

	const int bs = 4;
	
	int i, j;
	
/*	int n = m;*/
	
#if 1
	i = 0;
#if 0 && defined(TARGET_X64_AVX2)
	for(; i<m-8; i+=12)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_nt_12x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
			}
		if(j<i) // dgemm
			{
			if(j<n)
				{
				kernel_dgemm_nt_8x2_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				kernel_dgemm_nt_4x2_lib4(k, &pA[(i+8)*sda], &pB[j*sdb], &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], alg, 0, 0);
				j += 2;
				}
			}
		else // dsyrk
			{
			if(j<n-2)
				{
				kernel_dsyrk_nt_12x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg);
				if(j<n-10)
					{
					kernel_dsyrk_nt_8x8_lib4(k, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], sdb, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, alg);
					}
				else if(j<n-6)
					{
					kernel_dsyrk_nt_8x4_lib4(k, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, alg);
					//if(j<n-10)
					//	{
					//	kernel_dsyrk_nt_4x4_lib4(k, &pA[(i+8)*sda], &pB[(j+8)*sdb], &pC[(j+8)*bs+(i+8)*sdc], &pD[(j+8)*bs+(i+8)*sdd], alg);
					//	}
					//else 
					if(j<n-8)
						{
						kernel_dsyrk_nt_4x2_lib4(k, &pA[(i+8)*sda], &pB[(j+8)*sdb], &pC[(j+8)*bs+(i+8)*sdc], &pD[(j+8)*bs+(i+8)*sdd], alg);
						}
					}
				else if(j<n-4)
					{
					kernel_dsyrk_nt_8x2_lib4(k, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, alg);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_nt_4x2_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg);
				kernel_dgemm_nt_8x2_lib4(k, &pA[(i+4)*sda], sda, &pB[j*sdb], &pC[j*bs+(i+4)*sdc], sdc, &pD[j*bs+(i+4)*sdd], sdd, alg, 0, 0);
				}
			}
		}
#endif
#if 0 // defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
	for(; i<m-4; i+=8)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
			}
		if(j<i) // dgemm
			{
			if(j<n)
				{
				kernel_dgemm_nt_8x2_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				j += 2;
				}
			}
		else // dsyrk
			{
			if(j<n-2)
				{
				kernel_dsyrk_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg);
				if(j<n-6)
					{
					kernel_dsyrk_nt_4x4_lib4(k, &pA[(i+4)*sda], &pB[(j+4)*sdb], &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], alg);
					}
				else if(j<n-4)
					{
					kernel_dsyrk_nt_4x2_lib4(k, &pA[(i+4)*sda], &pB[(j+4)*sdb], &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], alg);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_nt_8x2_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg);
				}
			}
		}
#endif
	for(; i<m; i+=4)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_nn_4x4_lib4(k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], 0, 0);
			}
		if(j<i) // dgemm
			{
			if(j<n)
				{
				kernel_dgemm_nn_4x2_lib4(k, &pA[i*sda], &pB[j*bs], sdb, alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], 0, 0);
				j += 2;
				}
			}
		else // dsyrk
			{
			if(j<n-2)
				{
				kernel_dsyrk_nn_4x4_lib4(k, &pA[i*sda], &pB[j*bs], sdb, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg);
				}
			else if(j<n)
				{
				kernel_dsyrk_nn_4x2_lib4(k, &pA[i*sda], &pB[j*bs], sdb, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg);
				}
			}
		}
#else
	j = 0;
	for(; j<n-2; j+=4)
		{
		i = j;
#if defined(TARGET_X64_AVX2)
		if(i<m-8)
			{
			kernel_dsyrk_nt_12x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg);
			i += 12;
			for(; i<m-8; i+=12)
				{
				kernel_dgemm_nt_12x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				}
			for(; i<m-4; i+=8)
				{
				kernel_dgemm_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				}
			for(; i<m-2; i+=4)
				{
				kernel_dgemm_nt_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
				}
			for(; i<m; i+=2)
				{
				kernel_dgemm_nt_2x4_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
				}
			}
		else if(i<m-4)
			{
			kernel_dsyrk_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg);
			i += 8;
			}
#else
		if(i<m-4)
			{
#if defined(TARGET_X64_AVX)
			kernel_dsyrk_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg);
			i += 8;
#else
			kernel_dsyrk_nt_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdD], alg);
			i += 4;
#endif
#if defined(TARGET_X64_AVX)
			for(; i<m-4; i+=8)
				{
				kernel_dgemm_nt_8x4_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				}
#endif
			for(; i<m-2; i+=4)
				{
				kernel_dgemm_nt_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
				}
			for(; i<m; i+=2)
				{
				kernel_dgemm_nt_2x4_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
				}
			}
#endif
		else //if(i<m)
			{
			kernel_dsyrk_nt_4x4_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg);
			//i += 4;
			}
		}
	for(; j<n; j+=2)
		{
		i = j;
		if(i<m-2)
			{
			kernel_dsyrk_nt_4x2_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg);
			i += 4;
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
			for(; i<m-4; i+=8)
				{
				kernel_dgemm_nt_8x2_lib4(k, &pA[i*sda], sda, &pB[j*sdb], &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, alg, 0, 0);
				}
#endif
			for(; i<m-2; i+=4)
				{
				kernel_dgemm_nt_4x2_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
				}
			for(; i<m; i+=2)
				{
				kernel_dgemm_nt_2x2_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg, 0, 0);
				}
			}
		else //if(i<m)
			{
			kernel_dsyrk_nt_2x2_lib4(k, &pA[i*sda], &pB[j*sdb], &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], alg);
			}
		}
#endif

	}
#endif



#if ! defined(BLASFEO)
void dpotrf_lib(int m, int n, double *pC, int sdc, double *pD, int sdd, double *inv_diag_D)
	{

	if(m<=0 || n<=0)
		return;

	if(m<n)
		n = m;

	const int bs = 4;
	
	int i, j;
	
	double *dummy;

#if 1 // inner loop over columns

	i = 0;
#if defined(TARGET_X64_AVX2)
	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i && j<n-3; j+=4)
			{
			kernel_dtrsm_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				if(n-j==3)
					{
					kernel_dgemm_dtrsm_nt_12x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//					j += 3;
					}
				else
					{
					kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
					kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j, 0, 0, dummy, dummy, j, &pD[(i+8)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//					j += 2;
					}
				}
			}
		else // dpotrf
			{
			if(j<n-11)
				{
				kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dpotrf_nt_8x8_lib4_new(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				}
			else if(j<n-10)
				{
				kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x8_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, 0, dummy, 0, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				}
			else if(j<n-6)
				{
				kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, 0, dummy, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				if(j<n-8)
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-8, n-j-8, 0, 0, dummy, dummy, j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], 1, &pC[(j+8)*bs+(i+8)*sdc], &pD[(j+8)*bs+(i+8)*sdd], &inv_diag_D[j+8]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+8)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+8)*sdc], &pD[(j+4)*bs+(i+8)*sdd], &pD[(j+4)*bs+(j+4)*sdd], 1, &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i-4, n-j, 0, 0, dummy, 0, dummy, j, &pD[(i+4)*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], sdc, &pD[j*bs+(i+4)*sdd], sdd,  &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		}
//	for(; i<m-8; i+=12)
	if(i<m-8)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_12x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j, 0, 0, dummy, dummy, j, &pD[(i+8)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-10)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x8_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, 0, dummy, 0, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				}
			else if(j<n-6)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, 0, dummy, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				if(j<n-8)
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-8, n-j-8, 0, 0, dummy, dummy, j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], 1, &pC[(j+8)*bs+(i+8)*sdc], &pD[(j+8)*bs+(i+8)*sdd], &inv_diag_D[j+8]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+8)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+8)*sdc], &pD[(j+4)*bs+(i+8)*sdd], &pD[(j+4)*bs+(j+4)*sdd], 1, &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i-4, n-j, 0, 0, dummy, 0, dummy, j, &pD[(i+4)*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], sdc, &pD[j*bs+(i+4)*sdd], sdd,  &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		i += 12;
		}
//	for(; i<m-4; i+=8)
	if(i<m-4)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-6)
				{
				kernel_dsyrk_dpotrf_nt_8x8_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, 0, j, &pD[i*sdd], sdd, &pD[j*sdd], sdd, 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-4, n-j, 0, 0, dummy, dummy, j, &pD[(i+4)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], &pD[j*bs+(i+4)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		i += 8;
		}
//	for(; i<m; i+=4)
	if(i<m)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			}
		i += 4;
		}
#endif
#if defined(TARGET_X64_AVX)
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i && j<n-3; j+=4)
			{
			kernel_dtrsm_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				if(n-j==3)
					{
					kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//					j += 3;
					}
				else
					{
					kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//					j += 2;
					}
				}
			}
		else // dpotrf
			{
			if(j<n-6)
				{
				kernel_dpotrf_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-4, n-j, 0, 0, dummy, dummy, j, &pD[(i+4)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], &pD[j*bs+(i+4)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		}
	for(; i<m-4; i+=8)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-6)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-4, n-j, 0, 0, dummy, dummy, j, &pD[(i+4)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], &pD[j*bs+(i+4)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		}
	for(; i<m; i+=4)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			}
		}
#endif
#if defined(TARGET_C99_4X4) || defined(TARGET_X64_SSE3) || defined(TARGET_CORTEX_A15) || defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7) || defined(TARGET_CORTEX_A57)
	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dtrsm_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				if(n-j==3)
					{
					kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
					j += 3;
					}
				else
					{
					kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
					j += 2;
					}
				}
			}
		else // dpotrf
			{
			if(j<n-3)
				{
				kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			else if(j<n)
				{
				if(n-j==3)
					{
					kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
					}
				else
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
					}
				}
			}
		}
	if(i<m-2)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			}
		i += 4;
		}
	if(i<m)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_2x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_2x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				j += 2;
				}
			}
		else // dpotrf
			{
			kernel_dsyrk_dpotrf_nt_2x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			}
		i += 2;
		}

#endif

#else // inner loop over rows

#if defined(TARGET_X64_AVX)
	j = 0;
	for(; j<n-3; j+=4) // then i<n-3 !!!
		{
		i = j;
		if(i<m-7)
			{
			kernel_dpotrf_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
			i += 8;
			for(; i<m-7; i+=8)
				{
				kernel_dtrsm_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			if(i<m-4)
				{
				kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				i += 8;
				}
			else if(i<m)
				{
				kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				i += 4;
				}
			}
		else
			{
			if(i<m-4)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				}
			else //if(i<m)
				{
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			}
		}
	if(j<n-2) // then i<m-2 !!!
		{
		i = j;
		if(i<m-4)
			{
			kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
			i += 8;
			for(; i<m-4; i+=8)
				{
				kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			if(i<m)
				{
				kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				i += 4;
				}
			}
		else //if(i<m)
			{
			kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			}
//		j += 4;
		}
	else if(j<n) // then i<m !!!
		{
		i = j;
		if(i<m-2)
			{
			kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			i += 4;
			for(; i<m-4; i+=8)
				{
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			if(i<m)
				{
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				i += 4;
				}
			}
		else //if(i<m)
			{
			kernel_dsyrk_dpotrf_nt_2x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			}
//		j += 2;
		}

#endif
#if defined(TARGET_C99_4X4) || defined(TARGET_X64_SSE3) || defined(TARGET_CORTEX_A15) || defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7) || defined(TARGET_CORTEX_A57)
	j = 0;
	for(; j<n-3; j+=4) // then i<m-3 !!!
		{
		i = j;
		kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		i += 4;
		for(; i<m-3; i+=4)
			{
			kernel_dtrsm_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(i<m-2)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//			i += 4;
			}
		else if(i<m)
			{
			kernel_dgemm_dtrsm_nt_2x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//			i += 2;
			}
		}
	if(j<n-2) // then i<m-2 !!!
		{
		i = j;
		kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		i += 4;
		for(; i<m-2; i+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(i<m)
			{
			kernel_dgemm_dtrsm_nt_2x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//			i += 2;
			}
//		j += 4;
		}
	else if(j<n) // then i<m !!!
		{
		i = j;
		if(i<m-2)
			{
			kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			i += 4;
			for(; i<m-2; i+=4)
				{
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			if(i<m)
				{
				kernel_dgemm_dtrsm_nt_2x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				i += 2;
				}
			}
		else //if(i<m)
			{
			kernel_dsyrk_dpotrf_nt_2x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			}
//		j += 2;
		}

#endif

#endif

	}
#endif



#if ! defined(BLASFEO)
void dsyrk_dpotrf_lib(int m, int n, int k, double *pA, int sda, double *pB, int sdb, int alg, double *pC, int sdc, double *pD, int sdd, double *inv_diag_D)
	{

	if(m<=0 || n<=0)
		return;

	if(m<n)
		n = m;

	const int bs = 4;
	
	int i, j;

#if 1 // inner loop over columns

	i = 0;
#if defined(TARGET_X64_AVX2)
	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i && j<n-3; j+=4)
			{
			kernel_dgemm_dtrsm_nt_12x4_lib4_new(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				if(n-j==3)
					{
					kernel_dgemm_dtrsm_nt_12x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//					j += 3;
					}
				else
					{
					kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
					kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j, k, 0, &pA[(i+8)*sda], &pB[j*sdb], j, &pD[(i+8)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//					j += 2;
					}
				}
			}
		else // dpotrf
			{
			if(j<n-11)
				{
				kernel_dsyrk_dpotrf_nt_12x4_lib4_new(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x8_lib4_new(k, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], sdb, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				//kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i-4, n-j-4, k, 0, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				//kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-8, n-j-8, k, 0, &pA[(i+8)*sda], &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], 1, &pC[(j+8)*bs+(i+8)*sdc], &pD[(j+8)*bs+(i+8)*sdd], &inv_diag_D[j+8]);
				}
			else if(j<n-10)
				{
				kernel_dsyrk_dpotrf_nt_12x4_lib4_new(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x8_vs_lib4_new(m-i-4, n-j-4, k, 0, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], sdb, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				}
			else if(j<n-6)
				{
				kernel_dsyrk_dpotrf_nt_12x4_lib4_new(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i-4, n-j-4, k, 0, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				if(j<n-8)
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-8, n-j-8, k, 0, &pA[(i+8)*sda], &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], 1, &pC[(j+8)*bs+(i+8)*sdc], &pD[(j+8)*bs+(i+8)*sdd], &inv_diag_D[j+8]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, k, 0, &pA[(i+4)*sda], &pB[(j+4)*sdb], j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j-4, k, 0, &pA[(i+8)*sda], &pB[(j+4)*sdb], j+4, &pD[(i+8)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+8)*sdc], &pD[(j+4)*bs+(i+8)*sdd], &pD[(j+4)*bs+(j+4)*sdd], 1, &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i-4, n-j, k, 0, &pA[(i+4)*sda], sda, &pB[j*sdb], j, &pD[(i+4)*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], sdc, &pD[j*bs+(i+4)*sdd], sdd,  &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		}
//	for(; i<m-8; i+=12)
	if(i<m-8)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_12x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j, k, 0, &pA[(i+8)*sda], &pB[j*sdb], j, &pD[(i+8)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-10)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x8_vs_lib4_new(m-i-4, n-j-4, k, 0, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], sdb, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				}
			else if(j<n-6)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i-4, n-j-4, k, 0, &pA[(i+4)*sda], sda, &pB[(j+4)*sdb], j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				if(j<n-8)
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-8, n-j-8, k, 0, &pA[(i+8)*sda], &pB[(j+8)*sdb], j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], 1, &pC[(j+8)*bs+(i+8)*sdc], &pD[(j+8)*bs+(i+8)*sdd], &inv_diag_D[j+8]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, k, 0, &pA[(i+4)*sda], &pB[(j+4)*sdb], j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j-4, k, 0, &pA[(i+8)*sda], &pB[(j+4)*sdb], j+4, &pD[(i+8)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+8)*sdc], &pD[(j+4)*bs+(i+8)*sdd], &pD[(j+4)*bs+(j+4)*sdd], 1, &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i-4, n-j, k, 0, &pA[(i+4)*sda], sda, &pB[j*sdb], j, &pD[(i+4)*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], sdc, &pD[j*bs+(i+4)*sdd], sdd,  &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		i += 12;
		}
//	for(; i<m-4; i+=8)
	if(i<m-4)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-6)
				{
				kernel_dsyrk_dpotrf_nt_8x8_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], sdb, j, &pD[i*sdd], sdd, &pD[j*sdd], sdd, 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, k, 0, &pA[(i+4)*sda], &pB[(j+4)*sdb], j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-4, n-j, k, 0, &pA[(i+4)*sda], &pB[j*sdb], j, &pD[(i+4)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], &pD[j*bs+(i+4)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		i += 8;
		}
//	for(; i<m; i+=4)
	if(i<m)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			}
		i += 4;
		}
#endif
#if defined(TARGET_X64_AVX)
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i && j<n-3; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_lib4_new(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				if(n-j==3)
					{
					kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//					j += 3;
					}
				else
					{
					kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//					j += 2;
					}
				}
			}
		else // dpotrf
			{
			if(j<n-6)
				{
				kernel_dsyrk_dpotrf_nt_8x4_lib4_new(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i-4, n-j-4, k, 0, &pA[(i+4)*sda], &pB[(j+4)*sdb], j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], alg, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, k, 0, &pA[(i+4)*sda], &pB[(j+4)*sdb], j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], alg, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-4, n-j, k, 0, &pA[(i+4)*sda], &pB[j*sdb], j, &pD[(i+4)*sdd], &pD[j*sdd], alg, &pC[j*bs+(i+4)*sdc], &pD[j*bs+(i+4)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		}
	for(; i<m-4; i+=8)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-6)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i-4, n-j-4, k, 0, &pA[(i+4)*sda], &pB[(j+4)*sdb], j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], alg, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, k, 0, &pA[(i+4)*sda], &pB[(j+4)*sdb], j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], alg, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-4, n-j, k, 0, &pA[(i+4)*sda], &pB[j*sdb], j, &pD[(i+4)*sdd], &pD[j*sdd], alg, &pC[j*bs+(i+4)*sdc], &pD[j*bs+(i+4)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		}
	for(; i<m; i+=4)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			}
		}
#endif
#if defined(TARGET_C99_4X4) || defined(TARGET_X64_SSE3) || defined(TARGET_CORTEX_A15) || defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7) || defined(TARGET_CORTEX_A57)
	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_lib4_new(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				if(n-j==3)
					{
					kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
					j += 3;
					}
				else
					{
					kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
					j += 2;
					}
				}
			}
		else // dpotrf
			{
			if(j<n-3)
				{
				kernel_dsyrk_dpotrf_nt_4x4_lib4_new(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			else if(j<n)
				{
				if(n-j==3)
					{
					kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
					}
				else
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
					}
				}
			}
		}
	if(i<m-2)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			}
		i += 4;
		}
	if(i<m)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_2x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_2x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				j += 2;
				}
			}
		else // dpotrf
			{
			kernel_dsyrk_dpotrf_nt_2x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			}
		i += 2;
		}

#endif

#else // inner loop over rows

#if defined(TARGET_X64_AVX2)
	j = 0;
	for(; j<n-3; j+=4) // then i<n-3 !!!
		{
		i = j;
		if(i<m-11)
			{
			kernel_dsyrk_dpotrf_nt_12x4_lib4_new(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
			i += 12;
			for(; i<m-11; i+=12)
				{
				kernel_dgemm_dtrsm_nt_12x4_lib4_new(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			if(i<m-8)
				{
				kernel_dgemm_dtrsm_nt_12x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				i += 12;
				}
			else if(i<m-4)
				{
				kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				i += 8;
				}
			else if(i<m)
				{
				kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				i += 4;
				}
			}
		else
			{
			if(i<m-8)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				}
			else if(i<m-4)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				}
			else //if(i<m)
				{
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			}
		}
	if(j<n-2) // then i<m-2 !!!
		{
		i = j;
		if(i<m-4)
			{
			kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
			i += 8;
			for(; i<m-4; i+=8)
				{
				kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			if(i<m)
				{
				kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				i += 4;
				}
			}
		else //if(i<m)
			{
			kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			}
//		j += 4;
		}
	else if(j<n) // then i<m !!!
		{
		i = j;
		if(i<m-2)
			{
			kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			i += 4;
			for(; i<m-4; i+=8)
				{
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			if(i<m)
				{
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				i += 4;
				}
			}
		else //if(i<m)
			{
			kernel_dsyrk_dpotrf_nt_2x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			}
//		j += 2;
		}

#endif
#if defined(TARGET_X64_AVX)
	j = 0;
	for(; j<n-3; j+=4) // then i<n-3 !!!
		{
		i = j;
		if(i<m-7)
			{
			kernel_dsyrk_dpotrf_nt_8x4_lib4_new(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
			i += 8;
			for(; i<m-7; i+=8)
				{
				kernel_dgemm_dtrsm_nt_8x4_lib4_new(k, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			if(i<m-4)
				{
				kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				i += 8;
				}
			else if(i<m)
				{
				kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				i += 4;
				}
			}
		else
			{
			if(i<m-4)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				}
			else //if(i<m)
				{
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			}
		}
	if(j<n-2) // then i<m-2 !!!
		{
		i = j;
		if(i<m-4)
			{
			kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
			i += 8;
			for(; i<m-4; i+=8)
				{
				kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			if(i<m)
				{
				kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				i += 4;
				}
			}
		else //if(i<m)
			{
			kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			}
//		j += 4;
		}
	else if(j<n) // then i<m !!!
		{
		i = j;
		if(i<m-2)
			{
			kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			i += 4;
			for(; i<m-4; i+=8)
				{
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], sda, &pB[j*sdb], j, &pD[i*sdd], sdd, &pD[j*sdd], alg, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			if(i<m)
				{
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				i += 4;
				}
			}
		else //if(i<m)
			{
			kernel_dsyrk_dpotrf_nt_2x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			}
//		j += 2;
		}

#endif
#if defined(TARGET_C99_4X4) || defined(TARGET_X64_SSE3) || defined(TARGET_CORTEX_A15) || defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7) || defined(TARGET_CORTEX_A57)
	j = 0;
	for(; j<n-3; j+=4) // then i<m-3 !!!
		{
		i = j;
		kernel_dsyrk_dpotrf_nt_4x4_lib4_new(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		i += 4;
		for(; i<m-3; i+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_lib4_new(k, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(i<m-2)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//			i += 4;
			}
		else if(i<m)
			{
			kernel_dgemm_dtrsm_nt_2x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//			i += 2;
			}
		}
	if(j<n-2) // then i<m-2 !!!
		{
		i = j;
		kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
		i += 4;
		for(; i<m-2; i+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(i<m)
			{
			kernel_dgemm_dtrsm_nt_2x4_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//			i += 2;
			}
//		j += 4;
		}
	else if(j<n) // then i<m !!!
		{
		i = j;
		if(i<m-2)
			{
			kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			i += 4;
			for(; i<m-2; i+=4)
				{
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			if(i<m)
				{
				kernel_dgemm_dtrsm_nt_2x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				i += 2;
				}
			}
		else //if(i<m)
			{
			kernel_dsyrk_dpotrf_nt_2x2_vs_lib4_new(m-i, n-j, k, 0, &pA[i*sda], &pB[j*sdb], j, &pD[i*sdd], &pD[j*sdd], alg, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			}
//		j += 2;
		}

#endif

#endif

	}
#endif



void dlauum_dpotrf_lib(int m, int n, int k, double *pA, int sda, double *pB, int sdb, int alg, double *pC, int sdc, double *pD, int sdd, double *inv_diag_D)
	{

	if(m<=0 || n<=0)
		return;

	if(m<n)
		n = m;

	const int bs = 4;
	
	int i, j, ii, jj;
	
	double *dummy;

	i = 0;
#if defined(TARGET_X64_AVX2)
	// dlauum_dpotrf
	for(; i<k-11; i+=12)
		{
		j = 0;
		for(; j<i; j+=4)
			{
			kernel_dtrmm_dtrsm_nt_12x4_lib4_new(k-i, pA+i*sda+i*bs, sda, pB+j*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], alg, pC+i*sdc+j*bs, sdc, pD+i*sdd+j*bs, sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//			kernel_dtrmm_l_u_nt_12x4_lib4(k-i, pA+i*sda+i*bs, sda, pB+j*sdb+i*bs, pC+i*sdc+j*bs, sdc, pD+i*sdd+j*bs, sdd, alg);
//			kernel_dtrsm_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pD[j*bs+i*sdd], sdd, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		kernel_dlauum_dpotrf_nt_12x4_lib4_new(k-i, pA+i*sda+i*bs, sda, pB+i*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], alg, pC+i*sdc+i*bs, sdc, pD+i*sdd+i*bs, sdd, &inv_diag_D[j]);
//		kernel_dlauum_nt_12x4_lib4(k-i, pA+i*sda+i*bs, sda, pB+i*sdb+i*bs, alg, pC+i*sdc+i*bs, sdc, pD+i*sdd+i*bs, sdd);
//		kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pD[j*bs+i*sdd], sdd, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
		kernel_dlauum_dpotrf_nt_8x8_lib4_new(k-i-4, pA+(i+4)*sda+(i+4)*bs, sda, pB+(i+4)*sdb+(i+4)*bs, sdb, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, alg, pC+(i+4)*sdc+(i+4)*bs, sdc, pD+(i+4)*sdd+(i+4)*bs, sdd, &inv_diag_D[j+4]);
//		kernel_dlauum_nt_8x8_lib4(k-i-4, pA+(i+4)*sda+(i+4)*bs, sda, pB+(i+4)*sdb+(i+4)*bs, sdb, alg, pC+(i+4)*sdc+(i+4)*bs, sdc, pD+(i+4)*sdd+(i+4)*bs, sdd);
//		kernel_dpotrf_nt_8x8_lib4_new(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pD[(j+4)*bs+(i+4)*sdd], sdd, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
		}
	
	if(m-i<12)
		goto dlauum_dpotrf;
	
	// clean dlauum for k-i<12 and the continue with dpotrf only
	if(1)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_12x4_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, sda, pB+j*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_12x2_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, sda, pB+j*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, sda, pB+j*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j, k-i-8, 1, pA+(i+8)*sda+(i+8)*bs, pB+j*sdb+(i+8)*bs, j, &pD[(i+8)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-10)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, sda, pB+i*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x8_vs_lib4_new(m-i-4, n-j-4, k-i-4, 1, pA+(i+4)*sda+(i+4)*bs, sda, pB+(i+4)*sdb+(i+4)*bs, sdb, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				}
			else if(j<n-6)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, sda, pB+i*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i-4, n-j-4, k-i-4, 1, pA+(i+4)*sda+(i+4)*bs, sda, pB+(i+4)*sdb+(i+4)*bs, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				if(j<n-8)
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-8, n-j-8, k-i-8, 1, pA+(i+8)*sda+(i+8)*bs, pB+(i+8)*sdb+(i+8)*bs, j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], 1, &pC[(j+8)*bs+(i+8)*sdc], &pD[(j+8)*bs+(i+8)*sdd], &inv_diag_D[j+8]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, sda, pB+i*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, k-i-4, 1, pA+(i+4)*sda+(i+4)*bs, pB+(i+4)*sdb+(i+4)*bs, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j-4, k-i-8, 1, pA+(i+8)*sda+(i+8)*bs, pB+(j+4)*sdb+(i+8)*bs, j+4, &pD[(i+8)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+8)*sdc], &pD[(j+4)*bs+(i+8)*sdd], &pD[(j+4)*bs+(j+4)*sdd], 1, &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, pB+i*sdb+i*bs, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i-4, n-j, k-i-4, 1, pA+(i+4)*sda+(i+4)*bs, sda, pB+j*sdb+(i+4)*bs, j, &pD[(i+4)*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], sdc, &pD[j*bs+(i+4)*sdd], sdd,  &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		i += 12;
		}

	// dpotrf only
	for(; i<m-11; i+=12)
		{
		j = 0;
		for(; j<i && j<n-3; j+=4)
			{
			kernel_dtrsm_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				if(n-j==3)
					{
					kernel_dgemm_dtrsm_nt_12x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//					j += 3;
					}
				else
					{
					kernel_dgemm_dtrsm_nt_12x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//					kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//					kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j, 0, 0, dummy, dummy, j, &pD[(i+8)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//					j += 2;
					}
				}
			}
		else // dpotrf
			{
			if(j<n-11)
				{
				kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dpotrf_nt_8x8_lib4_new(j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				}
			else if(j<n-10)
				{
				kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x8_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, 0, dummy, 0, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				}
			else if(j<n-6)
				{
				kernel_dpotrf_nt_12x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, 0, dummy, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				if(j<n-8)
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-8, n-j-8, 0, 0, dummy, dummy, j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], 1, &pC[(j+8)*bs+(i+8)*sdc], &pD[(j+8)*bs+(i+8)*sdd], &inv_diag_D[j+8]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+8)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+8)*sdc], &pD[(j+4)*bs+(i+8)*sdd], &pD[(j+4)*bs+(j+4)*sdd], 1, &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i-4, n-j, 0, 0, dummy, 0, dummy, j, &pD[(i+4)*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], sdc, &pD[j*bs+(i+4)*sdd], sdd,  &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		}
//	for(; i<m-8; i+=12)
	if(i<m-8)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_12x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_12x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j, 0, 0, dummy, dummy, j, &pD[(i+8)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-10)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x8_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, 0, dummy, 0, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				}
			else if(j<n-6)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, 0, dummy, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				if(j<n-8)
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-8, n-j-8, 0, 0, dummy, dummy, j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], 1, &pC[(j+8)*bs+(i+8)*sdc], &pD[(j+8)*bs+(i+8)*sdd], &inv_diag_D[j+8]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+8)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+8)*sdc], &pD[(j+4)*bs+(i+8)*sdd], &pD[(j+4)*bs+(j+4)*sdd], 1, &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i-4, n-j, 0, 0, dummy, 0, dummy, j, &pD[(i+4)*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], sdc, &pD[j*bs+(i+4)*sdd], sdd,  &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		}
//	for(; i<m-4; i+=8)
	else if(i<m-4)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-6)
				{
				kernel_dsyrk_dpotrf_nt_8x8_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, 0, j, &pD[i*sdd], sdd, &pD[j*sdd], sdd, 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-4, n-j, 0, 0, dummy, dummy, j, &pD[(i+4)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], &pD[j*bs+(i+4)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		}
//	for(; i<m; i+=4)
	else if(i<m)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			}
		}
	
	return;

	dlauum_dpotrf:

	for(; i<m-8; i+=12)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_12x4_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, sda, pB+j*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, sda, pB+j*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j, k-i-8, 1, pA+(i+8)*sda+(i+8)*bs, pB+j*sdb+(i+8)*bs, j, &pD[(i+8)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+8)*sdc], &pD[j*bs+(i+8)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-10)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, sda, pB+i*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x8_vs_lib4_new(m-i-4, n-j-4, k-i-4, 1, pA+(i+4)*sda+(i+4)*bs, sda, pB+(i+4)*sdb+(i+4)*bs, sdb, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], sdd, 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				}
			else if(j<n-6)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, sda, pB+i*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i-4, n-j-4, k-i-4, 1, pA+(i+4)*sda+(i+4)*bs, sda, pB+(i+4)*sdb+(i+4)*bs, j+4, &pD[(i+4)*sdd], sdd, &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], sdc, &pD[(j+4)*bs+(i+4)*sdd], sdd, &inv_diag_D[j+4]);
				if(j<n-8)
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-8, n-j-8, k-i-8, 1, pA+(i+8)*sda+(i+8)*bs, pB+(i+8)*sdb+(i+8)*bs, j+8, &pD[(i+8)*sdd], &pD[(j+8)*sdd], 1, &pC[(j+8)*bs+(i+8)*sdc], &pD[(j+8)*bs+(i+8)*sdd], &inv_diag_D[j+8]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_12x4_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, sda, pB+i*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, k-i-4, 1, pA+(i+4)*sda+(i+4)*bs, pB+(i+4)*sdb+(i+4)*bs, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-8, n-j-4, k-i-8, 1, pA+(i+8)*sda+(i+8)*bs, pB+(j+4)*sdb+(i+8)*bs, j+4, &pD[(i+8)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+8)*sdc], &pD[(j+4)*bs+(i+8)*sdd], &pD[(j+4)*bs+(j+4)*sdd], 1, &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, pB+i*sdb+i*bs, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i-4, n-j, k-i-4, 1, pA+(i+4)*sda+(i+4)*bs, sda, pB+j*sdb+(i+4)*bs, j, &pD[(i+4)*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], sdc, &pD[j*bs+(i+4)*sdd], sdd,  &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		}
	for(; i<m-4; i+=8)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, sda, pB+j*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, sda, pB+j*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-6)
				{
				kernel_dsyrk_dpotrf_nt_8x8_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, sda, pB+i*sdb+i*bs, sdb, j, &pD[i*sdd], sdd, &pD[j*sdd], sdd, 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, sda, pB+i*sdb+i*bs, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, k-i-4, 1, pA+(i+4)*sda+(i+4)*bs, pB+(i+4)*sdb+(i+4)*bs, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, pB+i*sdb+i*bs, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-4, n-j, k-i-4, 1, pA+(i+4)*sda+(i+4)*bs, pB+(i+4)*sdb+(i+4)*bs, j, &pD[(i+4)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], &pD[j*bs+(i+4)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		}
	for(; i<m; i+=4)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, pB+j*sdb+i*bs, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, pB+j*sdb+i*bs, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, pB+i*sdb+i*bs, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, k-i, 1, pA+i*sda+i*bs, pB+i*sdb+i*bs, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			}
		}
	
	return;

#endif
#if defined(TARGET_X64_AVX)
	for(; i<m-7; i+=8)
		{
		j = 0;
		for(; j<i && j<n-3; j+=4)
			{
			kernel_dtrsm_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				if(n-j==3)
					{
					kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//					j += 3;
					}
				else
					{
					kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//					j += 2;
					}
				}
			}
		else // dpotrf
			{
			if(j<n-6)
				{
				kernel_dpotrf_nt_8x4_lib4_new(j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-4, n-j, 0, 0, dummy, dummy, j, &pD[(i+4)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], &pD[j*bs+(i+4)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		}
	for(; i<m-4; i+=8)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_8x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-6)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
				}
			else if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_8x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, 0, dummy, j, &pD[i*sdd], sdd, &pD[j*sdd], 1, &pC[j*bs+i*sdc], sdc, &pD[j*bs+i*sdd], sdd, &inv_diag_D[j]);
				if(j<n-4)
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i-4, n-j-4, 0, 0, dummy, dummy, j+4, &pD[(i+4)*sdd], &pD[(j+4)*sdd], 1, &pC[(j+4)*bs+(i+4)*sdc], &pD[(j+4)*bs+(i+4)*sdd], &inv_diag_D[j+4]);
					}
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i-4, n-j, 0, 0, dummy, dummy, j, &pD[(i+4)*sdd], &pD[j*sdd], 1, &pC[j*bs+(i+4)*sdc], &pD[j*bs+(i+4)*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				}
			}
		}
	for(; i<m; i+=4)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
//				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			}
		}
#endif
#if defined(TARGET_C99_4X4) || defined(TARGET_X64_SSE3) || defined(TARGET_CORTEX_A15) || defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7) || defined(TARGET_CORTEX_A57)
	for(; i<m-3; i+=4)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dtrsm_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				if(n-j==3)
					{
					kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
					j += 3;
					}
				else
					{
					kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
					j += 2;
					}
				}
			}
		else // dpotrf
			{
			if(j<n-3)
				{
				kernel_dpotrf_nt_4x4_lib4_new(j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			else if(j<n)
				{
				if(n-j==3)
					{
					kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
					}
				else
					{
					kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
					}
				}
			}
		}
	if(i<m-2)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				j += 2;
				}
			}
		else // dpotrf
			{
			if(j<n-2)
				{
				kernel_dsyrk_dpotrf_nt_4x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			else if(j<n)
				{
				kernel_dsyrk_dpotrf_nt_4x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
				}
			}
		i += 4;
		}
	if(i<m)
		{
		j = 0;
		for(; j<i && j<n-2; j+=4)
			{
			kernel_dgemm_dtrsm_nt_2x4_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
			}
		if(j<i) // dtrsm
			{
			if(j<n)
				{
				kernel_dgemm_dtrsm_nt_2x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &pD[j*bs+j*sdd], 1, &inv_diag_D[j]);
				j += 2;
				}
			}
		else // dpotrf
			{
			kernel_dsyrk_dpotrf_nt_2x2_vs_lib4_new(m-i, n-j, 0, 0, dummy, dummy, j, &pD[i*sdd], &pD[j*sdd], 1, &pC[j*bs+i*sdc], &pD[j*bs+i*sdd], &inv_diag_D[j]);
			}
		i += 2;
		}

#endif

	}



#if ! defined(BLASFEO)
// TODO modify kernels instead
void dgemv_n_lib(int m, int n, double *pA, int sda, double *x, int alg, double *y, double *z) // pA has to be aligned !!!
	{

	// early return
	if(m<=0  || n<=0)
		return;
	
	const int bs = 4;
	
	int i, j;

	j=0;
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
	for(; j<m-11; j+=12)
		{
		kernel_dgemv_n_12_lib4(n, pA, sda, x, y, z, alg);
		pA += 3*sda*bs;
		y  += 3*bs;
		z  += 3*bs;
		}
	if(j<m-8)
		{
		kernel_dgemv_n_12_vs_lib4(m-j, n, pA, sda, x, y, z, alg);
		pA += 3*sda*bs;
		y  += 3*bs;
		z  += 3*bs;
		//j  += 12;
		return;
		}
#else
	for(; j<m-7; j+=8)
		{
		kernel_dgemv_n_8_lib4(n, pA, sda, x, y, z, alg);
		pA += 2*sda*bs;
		y  += 2*bs;
		z  += 2*bs;
		}
#endif
	if(j<m-4)
		{
		kernel_dgemv_n_8_vs_lib4(m-j, n, pA, sda, x, y, z, alg);
		pA += 2*sda*bs;
		y  += 2*bs;
		z  += 2*bs;
		//j  += 8;
		return;
		}
	if(j<m)
		{
		kernel_dgemv_n_4_vs_lib4(m-j, n, pA, x, y, z, alg);
		pA += sda*bs;
		y  += bs;
		z  += bs;
		//j  += 4;
		return;
		}

	}
#endif



#if ! defined(BLASFEO)
void dgemv_t_lib(int m, int n, double *pA, int sda, double *x, int alg, double *y, double *z)
	{
	
	// early return
	if(m<=0  || n<=0)
		return;
	
	const int bs = 4;
	
	int j;
	
	j=0;
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
	for(; j<n-11; j+=12)
		{
		kernel_dgemv_t_12_lib4(m, pA+j*bs, sda, x, y+j, z+j, alg);
		}
#endif
	for(; j<n-7; j+=8)
		{
		kernel_dgemv_t_8_lib4(m, pA+j*bs, sda, x, y+j, z+j, alg);
		}
	for(; j<n-3; j+=4)
		{
		kernel_dgemv_t_4_lib4(m, pA+j*bs, sda, x, y+j, z+j, alg);
		}
	if(n>j)
		{
		if(n-j==1)
			kernel_dgemv_t_1_lib4(m, pA+j*bs, sda, x, y+j, z+j, alg);
		else if(n-j==2)
			kernel_dgemv_t_2_lib4(m, pA+j*bs, sda, x, y+j, z+j, alg);
		else if(n-j==3)
			kernel_dgemv_t_3_lib4(m, pA+j*bs, sda, x, y+j, z+j, alg);
		}

	}
#endif



#if ! defined(BLASFEO)
void dtrmv_u_n_lib(int m, double *pA, int sda, double *x, int alg, double *y)
	{

	if(m<=0)
		return;

	const int bs = 4;
	
	int j;
	
	j=0;
#if defined(TARGET_X64_AVX2)
	for(; j<m-11; j+=12)
		{
		kernel_dtrmv_u_n_12_lib4(m-j, pA, sda, x, y, alg);
		pA += 3*sda*bs + 3*4*bs;
		x  += 3*bs;
		y  += 3*bs;
		}
#endif
	for(; j<m-7; j+=8)
		{
		kernel_dtrmv_u_n_8_lib4(m-j, pA, sda, x, y, alg);
		pA += 2*sda*bs + 2*4*bs;
		x  += 2*bs;
		y  += 2*bs;
		}
	for(; j<m-3; j+=4)
		{
		kernel_dtrmv_u_n_4_lib4(m-j, pA, x, y, alg);
		pA += sda*bs + 4*bs;
		x  += bs;
		y  += bs;
		}
	for(; j<m-1; j+=2)
		{
		kernel_dtrmv_u_n_2_lib4(m-j, pA, x, y, alg);
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
#endif



#if ! defined(BLASFEO)
void dtrmv_u_t_lib(int m, double *pA, int sda, double *x, int alg, double *y)
	{

	if(m<=0)
		return;

	const int bs = 4;
	
	int j;
	
	double *ptrA;
	
	j=0;
#if defined(TARGET_X64_AVX2)
	for(; j<m-11; j+=12)
		{
		kernel_dtrmv_u_t_12_lib4(j, pA, sda, x, y, alg);
		pA += 3*4*bs;
		y  += 3*bs;
		}
#endif
	for(; j<m-7; j+=8)
		{
		kernel_dtrmv_u_t_8_lib4(j, pA, sda, x, y, alg);
		pA += 2*4*bs;
		y  += 2*bs;
		}
	for(; j<m-3; j+=4)
		{
		kernel_dtrmv_u_t_4_lib4(j, pA, sda, x, y, alg);
		pA += 4*bs;
		y  += bs;
		}
	for(; j<m-1; j+=2) // keep for !!!
		{
		kernel_dtrmv_u_t_2_lib4(j, pA, sda, x, y, alg);
		pA += 2*bs;
		y  += 2;
		}
	if(j<m)
		{
		kernel_dtrmv_u_t_1_lib4(j, pA, sda, x, y, alg);
		}

	}
#endif



#if ! defined(BLASFEO)
// it moves vertically across block // TODO allow rectangular matrices
void dsymv_lib(int m, int n, double *pA, int sda, double *x, int alg, double *y, double *z)
	{

	if(m<=0 || n<=0)
		return;

	// TODO better way to do 4-ways ???
	
	const int bs = 4;
	
	if(m<n)
		n = m;
	
	int j, j0;
	
	if(alg==0)
		{
		for(j=0; j<m; j++)
			z[j] = 0.0;
		alg = 1;
		}
	else
		{
		if(y!=z) // not same vector in memory
			for(j=0; j<m; j++)
				z[j] = y[j];
		}
	
	j=0;
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
	for(; j<n-11; j+=12)
		{
		kernel_dsymv_6_lib4(m-j, pA+j*sda+j*bs, sda, x+j, z+j, z+j, x+j, z+j, z+j, 1, alg, alg);
		kernel_dsymv_6_lib4(m-(j+6), pA+2+(j+4)*sda+(j+6)*bs, sda, x+(j+6), z+(j+6), z+(j+6), x+(j+6), z+(j+6), z+(j+6), 2, alg, alg);
		}
#endif
	for(; j<n-3; j+=4)
		{
		kernel_dsymv_4_lib4(m-j, pA+j*sda+j*bs, sda, x+j, z+j, z+j, x+j, z+j, z+j, 1, alg, alg);
		}
	if(j<n)
		{
		if(n-j==1)
			{
			kernel_dsymv_1_lib4(m-j, pA+j*sda+j*bs, sda, x+j, z+j, z+j, x+j, z+j, z+j, 1, alg, alg);
			}
		else if(n-j==2)
			{
			kernel_dsymv_2_lib4(m-j, pA+j*sda+j*bs, sda, x+j, z+j, z+j, x+j, z+j, z+j, 1, alg, alg);
			}
		else // if(n-j==3)
			{
			kernel_dsymv_3_lib4(m-j, pA+j*sda+j*bs, sda, x+j, z+j, z+j, x+j, z+j, z+j, 1, alg, alg);
			}
		}

	}
#endif



#if ! defined(BLASFEO)
// it moves vertically across block
void dgemv_nt_lib(int m, int n, double *pA, int sda, double *x_n, double *x_t, int alg_n, int alg_t, double *y_n, double *y_t, double *z_n, double *z_t)
	{

	if(m<=0 || n<=0)
		return;

	// TODO better way to do 4-ways ???
	
	const int bs = 4;

	int j;
	
	if(alg_n==0)
		{
		for(j=0; j<m; j++)
			z_n[j] = 0.0;
		alg_n = 1;
		}
	else
		{
		if(y_n!=z_n)
			for(j=0; j<m; j++)
				z_n[j] = y_n[j];
		}

	if(alg_t==0)
		{
		for(j=0; j<n; j++)
			z_t[j] = 0.0; // TODO the t part can work as in dgemv_t, i.e. move this in the kernel !!!!!
		alg_t = 1;
		}
	else
		{
		if(y_t!=z_t)
			for(j=0; j<n; j++)
				z_t[j] = y_t[j]; // TODO the t part can work as in dgemv_t, i.e. move this in the kernel !!!!!
		}
	
	j=0;
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
	for(; j<n-5; j+=6)
		{
		kernel_dsymv_6_lib4(m, pA+j*bs, sda, x_n+j, z_n, z_n, x_t, z_t+j, z_t+j, 0, alg_n, alg_t);
		}
#endif
	for(; j<n-3; j+=4)
		{
		kernel_dsymv_4_lib4(m, pA+j*bs, sda, x_n+j, z_n, z_n, x_t, z_t+j, z_t+j, 0, alg_n, alg_t);
		}
	for(; j<n-1; j+=2)
		{
		kernel_dsymv_2_lib4(m, pA+j*bs, sda, x_n+j, z_n, z_n, x_t, z_t+j, z_t+j, 0, alg_n, alg_t);
		}
	for(; j<n; j++)
		{
		kernel_dsymv_1_lib4(m, pA+j*bs, sda, x_n+j, z_n, z_n, x_t, z_t+j, z_t+j, 0, alg_n, alg_t);
		}

	}
#endif



#if ! defined(BLASFEO)
void dtrsv_n_lib(int m, int n, double *pA, int sda, int use_inv_diag_A, double *inv_diag_A, double *x, double *y)
	{

	if(m<=0 || n<=0)
		return;

	// suppose m>=n
	if(m<n)
		m = n;
	
	const int bs = 4;
	
	int i, j;
	
	if(x!=y)
		for(i=0; i<m; i++)
			y[i] = x[i];

	j = 0;
#if defined(TARGET_X64_AVX2)
	for(; j<n-11; j+=12)
		{
		kernel_dtrsv_n_12_lib4_new(j, &pA[j*sda], sda, use_inv_diag_A, &inv_diag_A[j], x, &y[j]);
		}
#endif
	for(; j<n-7; j+=8)
		{
		kernel_dtrsv_n_8_lib4_new(j, &pA[j*sda], sda, use_inv_diag_A, &inv_diag_A[j], x, &y[j]);
		}
	if(j<n-3)
		{
		kernel_dtrsv_n_4_lib4_new(j, &pA[j*sda], use_inv_diag_A, &inv_diag_A[j], x, &y[j]);
		j += 4;
		}
	if(j<n)
		{
		kernel_dtrsv_n_4_vs_lib4_new(m-j, n-j, j, &pA[j*sda], use_inv_diag_A, &inv_diag_A[j], x, &y[j]);
		j += 4;
		}
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
	for(; j<m-11; j+=12)
		{
		kernel_dgemv_n_12_lib4(n, &pA[j*sda], sda, x, &y[j], &y[j], -1);
		}
	if(j<m-8)
		{
		kernel_dgemv_n_12_vs_lib4(m-j, n, &pA[j*sda], sda, x, &y[j], &y[j], -1);
		j += 12;
		}
#else
	for(; j<m-7; j+=8)
		{
		kernel_dgemv_n_8_lib4(n, &pA[j*sda], sda, x, &y[j], &y[j], -1);
		}
#endif
	if(j<m-4)
		{
		kernel_dgemv_n_8_vs_lib4(m-j, n, &pA[j*sda], sda, x, &y[j], &y[j], -1);
		j += 8;
		}
	if(j<m)
		{
		kernel_dgemv_n_4_vs_lib4(m-j, n, &pA[j*sda], x, &y[j], &y[j], -1);
		j += 4;
		}

	}
#endif



#if ! defined(BLASFEO)
void dtrsv_t_lib(int m, int n, double *pA, int sda, int use_inv_diag_A, double *inv_diag_A, double *x, double *y)
	{

	if(m<=0 || n<=0)
		return;
	
	if(n>m)
		n = m;
	
	const int bs = 4;
	
	int i, j;
	
	if(x!=y)
		for(i=0; i<m; i++)
			y[i] = x[i];
			
	j=0;
	if(n%4==1)
		{
		kernel_dtrsv_t_1_lib4_new(m-n+j+1, pA+(n/bs)*bs*sda+(n-1)*bs, sda, use_inv_diag_A, inv_diag_A+n-j-1, y+n-j-1);
		j++;
		}
	else if(n%4==2)
		{
		kernel_dtrsv_t_2_lib4_new(m-n+j+2, pA+(n/bs)*bs*sda+(n-j-2)*bs, sda, use_inv_diag_A, inv_diag_A+n-j-2, y+n-j-2);
		j+=2;
		}
	else if(n%4==3)
		{
		kernel_dtrsv_t_3_lib4_new(m-n+j+3, pA+(n/bs)*bs*sda+(n-j-3)*bs, sda, use_inv_diag_A, inv_diag_A+n-j-3, y+n-j-3);
		j+=3;
		}
	for(; j<n-3; j+=4)
		{
		kernel_dtrsv_t_4_lib4_new(m-n+j+4, pA+((n-j-4)/bs)*bs*sda+(n-j-4)*bs, sda, use_inv_diag_A, inv_diag_A+n-j-4, y+n-j-4);
		}

	}
#endif



#if ! defined(BLASFEO)
// y = y + alpha*x, with increments equal to 1
void daxpy_lib(int kmax, double alpha, double *x, double *y)
	{

	int ii;

#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
	__m256d
		v_alpha, v_tmp,
		v_x0, v_y0,
		v_x1, v_y1;
#endif

	ii = 0;
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
	v_alpha = _mm256_broadcast_sd( &alpha );
	for( ; ii<kmax-7; ii+=8)
		{
		v_x0  = _mm256_load_pd( &x[ii+0] );
		v_x1  = _mm256_load_pd( &x[ii+4] );
		v_y0  = _mm256_load_pd( &y[ii+0] );
		v_y1  = _mm256_load_pd( &y[ii+4] );
#if defined(TARGET_X64_AVX2)
		v_y0  = _mm256_fmadd_pd( v_alpha, v_x0, v_y0 );
		v_y1  = _mm256_fmadd_pd( v_alpha, v_x1, v_y1 );
#else // AVX
		v_tmp = _mm256_mul_pd( v_alpha, v_x0 );
		v_y0  = _mm256_add_pd( v_tmp, v_y0 );
		v_tmp = _mm256_mul_pd( v_alpha, v_x1 );
		v_y1  = _mm256_add_pd( v_tmp, v_y1 );
#endif
		_mm256_store_pd( &y[ii+0], v_y0 );
		_mm256_store_pd( &y[ii+4], v_y1 );
		}
	for( ; ii<kmax-3; ii+=4)
		{
		v_x0  = _mm256_load_pd( &x[ii] );
		v_y0  = _mm256_load_pd( &y[ii] );
#if defined(TARGET_X64_AVX2)
		v_y0  = _mm256_fmadd_pd( v_alpha, v_x0, v_y0 );
#else
		v_tmp = _mm256_mul_pd( v_alpha, v_x0 );
		v_y0  = _mm256_add_pd( v_tmp, v_y0 );
#endif
		_mm256_store_pd( &y[ii], v_y0 );
		}
#else
	for( ; ii<kmax-3; ii+=4)
		{
		y[ii+0] = y[ii+0] + alpha*x[ii+0];
		y[ii+1] = y[ii+1] + alpha*x[ii+1];
		y[ii+2] = y[ii+2] + alpha*x[ii+2];
		y[ii+3] = y[ii+3] + alpha*x[ii+3];
		}
#endif
	for( ; ii<kmax; ii++)
		{
		y[ii+0] = y[ii+0] + alpha*x[ii+0];
		}

	return;

	}
#endif



#if ! defined(BLASFEO)
// z = y, y = y + alpha*x, with increments equal to 1
void daxpy_bkp_lib(int kmax, double alpha, double *x, double *y, double *z)
	{

	int ii;

#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
	__m256d
		v_alpha, v_tmp,
		v_x0, v_y0,
		v_x1, v_y1;
#endif

	ii = 0;
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
	v_alpha = _mm256_broadcast_sd( &alpha );
	for( ; ii<kmax-7; ii+=8)
		{
		v_y0  = _mm256_loadu_pd( &y[ii+0] );
		v_y1  = _mm256_loadu_pd( &y[ii+4] );
		v_x0  = _mm256_loadu_pd( &x[ii+0] );
		v_x1  = _mm256_loadu_pd( &x[ii+4] );
		_mm256_storeu_pd( &z[ii+0], v_y0 );
		_mm256_storeu_pd( &z[ii+4], v_y1 );
#if defined(TARGET_X64_AVX2)
		v_y0  = _mm256_fmadd_pd( v_alpha, v_x0, v_y0 );
		v_y1  = _mm256_fmadd_pd( v_alpha, v_x1, v_y1 );
#else // AVX
		v_tmp = _mm256_mul_pd( v_alpha, v_x0 );
		v_y0  = _mm256_add_pd( v_tmp, v_y0 );
		v_tmp = _mm256_mul_pd( v_alpha, v_x1 );
		v_y1  = _mm256_add_pd( v_tmp, v_y1 );
#endif
		_mm256_storeu_pd( &y[ii+0], v_y0 );
		_mm256_storeu_pd( &y[ii+4], v_y1 );
		}
	for( ; ii<kmax-3; ii+=4)
		{
		v_y0  = _mm256_loadu_pd( &y[ii] );
		v_x0  = _mm256_loadu_pd( &x[ii] );
		_mm256_storeu_pd( &z[ii], v_y0 );
#if defined(TARGET_X64_AVX2)
		v_y0  = _mm256_fmadd_pd( v_alpha, v_x0, v_y0 );
#else
		v_tmp = _mm256_mul_pd( v_alpha, v_x0 );
		v_y0  = _mm256_add_pd( v_tmp, v_y0 );
#endif
		_mm256_storeu_pd( &y[ii], v_y0 );
		}
#else
	for( ; ii<kmax-3; ii+=4)
		{
		z[ii+0] = y[ii+0];
		y[ii+0] = y[ii+0] + alpha*x[ii+0];
		z[ii+1] = y[ii+1];
		y[ii+1] = y[ii+1] + alpha*x[ii+1];
		z[ii+2] = y[ii+2];
		y[ii+2] = y[ii+2] + alpha*x[ii+2];
		z[ii+3] = y[ii+3];
		y[ii+3] = y[ii+3] + alpha*x[ii+3];
		}
#endif
	for( ; ii<kmax; ii++)
		{
		z[ii+0] = y[ii+0];
		y[ii+0] = y[ii+0] + alpha*x[ii+0];
		}

	return;

	}
#endif



#if ! defined(BLASFEO)
// regularize diagonal 
void ddiareg_lib(int kmax, double reg, int offset, double *pD, int sdd)
	{

	const int bs = 4;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pD[ll+bs*ll] += reg;
			}
		pD += kna + bs*(sdd-1) + kna*bs;
		kmax -= kna;
		}
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[jj*sdd+(jj+0)*bs+0] += reg;
		pD[jj*sdd+(jj+1)*bs+1] += reg;
		pD[jj*sdd+(jj+2)*bs+2] += reg;
		pD[jj*sdd+(jj+3)*bs+3] += reg;
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+(jj+ll)*bs+ll] += reg;
		}
	
	}
#endif



#if ! defined(BLASFEO)
// insert vector to diagonal 
void ddiain_lib(int kmax, double *x, int offset, double *pD, int sdd)
	{

	const int bs = 4;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pD[ll+bs*ll] = x[ll];
			}
		pD += kna + bs*(sdd-1) + kna*bs;
		x  += kna;
		kmax -= kna;
		}
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[jj*sdd+(jj+0)*bs+0] = x[jj+0];
		pD[jj*sdd+(jj+1)*bs+1] = x[jj+1];
		pD[jj*sdd+(jj+2)*bs+2] = x[jj+2];
		pD[jj*sdd+(jj+3)*bs+3] = x[jj+3];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+(jj+ll)*bs+ll] = x[jj+ll];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// insert sqrt of vector to diagonal 
void ddiain_sqrt_lib(int kmax, double *x, int offset, double *pD, int sdd)
	{

	const int bs = 4;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pD[ll+bs*ll] = sqrt(x[ll]);
			}
		pD += kna + bs*(sdd-1) + kna*bs;
		x  += kna;
		kmax -= kna;
		}
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[jj*sdd+(jj+0)*bs+0] = sqrt(x[jj+0]);
		pD[jj*sdd+(jj+1)*bs+1] = sqrt(x[jj+1]);
		pD[jj*sdd+(jj+2)*bs+2] = sqrt(x[jj+2]);
		pD[jj*sdd+(jj+3)*bs+3] = sqrt(x[jj+3]);
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+(jj+ll)*bs+ll] = sqrt(x[jj+ll]);
		}
	
	}
#endif



#if ! defined(BLASFEO)
// extract diagonal to vector 
void ddiaex_lib(int kmax, int offset, double *pD, int sdd, double *x)
	{

	const int bs = 4;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			x[ll] = pD[ll+bs*ll];
			}
		pD += kna + bs*(sdd-1) + kna*bs;
		x  += kna;
		kmax -= kna;
		}
	for(jj=0; jj<kmax-3; jj+=4)
		{
		x[jj+0] = pD[jj*sdd+(jj+0)*bs+0];
		x[jj+1] = pD[jj*sdd+(jj+1)*bs+1];
		x[jj+2] = pD[jj*sdd+(jj+2)*bs+2];
		x[jj+3] = pD[jj*sdd+(jj+3)*bs+3];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		x[jj+ll] = pD[jj*sdd+(jj+ll)*bs+ll];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// add scaled vector to diagonal 
void ddiaad_lib(int kmax, double alpha, double *x, int offset, double *pD, int sdd)
	{

	const int bs = 4;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pD[ll+bs*ll] += alpha * x[ll];
			}
		pD += kna + bs*(sdd-1) + kna*bs;
		x  += kna;
		kmax -= kna;
		}
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[jj*sdd+(jj+0)*bs+0] += alpha * x[jj+0];
		pD[jj*sdd+(jj+1)*bs+1] += alpha * x[jj+1];
		pD[jj*sdd+(jj+2)*bs+2] += alpha * x[jj+2];
		pD[jj*sdd+(jj+3)*bs+3] += alpha * x[jj+3];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+(jj+ll)*bs+ll] += alpha * x[jj+ll];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// insert vector to diagonal, sparse formulation 
void ddiain_libsp(int kmax, int *idx, double *x, double *pD, int sdd)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii/bs*bs*sdd+ii%bs+ii*bs] = x[jj];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// add scaled vector to diagonal, sparse formulation 
void ddiaad_libsp(int kmax, int *idx, double alpha, double *x, double *pD, int sdd)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii/bs*bs*sdd+ii%bs+ii*bs] += alpha * x[jj];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// add scaled vector to another vector and insert to diagonal, sparse formulation 
void ddiaadin_libsp(int kmax, int *idx, double alpha, double *x, double *y, double *pD, int sdd)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii/bs*bs*sdd+ii%bs+ii*bs] = y[jj] + alpha * x[jj];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// insert vector to row 
void drowin_lib(int kmax, double *x, double *pD)
	{
	
	const int bs = 4;

	int jj, ll;

	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[(jj+0)*bs] = x[jj+0];
		pD[(jj+1)*bs] = x[jj+1];
		pD[(jj+2)*bs] = x[jj+2];
		pD[(jj+3)*bs] = x[jj+3];
		}
	for(; jj<kmax; jj++)
		{
		pD[(jj)*bs] = x[jj];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// extract row to vector
void drowex_lib(int kmax, double *pD, double *x)
	{
	
	const int bs = 4;

	int jj, ll;

	for(jj=0; jj<kmax-3; jj+=4)
		{
		x[jj+0] = pD[(jj+0)*bs];
		x[jj+1] = pD[(jj+1)*bs];
		x[jj+2] = pD[(jj+2)*bs];
		x[jj+3] = pD[(jj+3)*bs];
		}
	for(; jj<kmax; jj++)
		{
		x[jj] = pD[(jj)*bs];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// add scaled vector to row 
void drowad_lib(int kmax, double alpha, double *x, double *pD)
	{

	const int bs = 4;

	int jj, ll;

	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[(jj+0)*bs] += alpha * x[jj+0];
		pD[(jj+1)*bs] += alpha * x[jj+1];
		pD[(jj+2)*bs] += alpha * x[jj+2];
		pD[(jj+3)*bs] += alpha * x[jj+3];
		}
	for(; jj<kmax; jj++)
		{
		pD[(jj)*bs] += alpha * x[jj];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// insert vector to row, sparse formulation 
void drowin_libsp(int kmax, int *idx, double *x, double *pD)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii*bs] = x[jj];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// add scaled vector to row, sparse formulation 
void drowad_libsp(int kmax, int *idx, double alpha, double *x, double *pD)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii*bs] += alpha * x[jj];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// add scaled vector to another vector and insert to row, sparse formulation 
void drowadin_libsp(int kmax, int *idx, double alpha, double *x, double *y, double *pD)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii*bs] = y[jj] + alpha * x[jj];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// swap two rows
void drowsw_lib(int kmax, double *pA, double *pC)
	{

	const int bs = 4;

	int ii;
	double tmp;

	for(ii=0; ii<kmax-3; ii+=4)
		{
		tmp = pA[0+bs*0];
		pA[0+bs*0] = pC[0+bs*0];
		pC[0+bs*0] = tmp;
		tmp = pA[0+bs*1];
		pA[0+bs*1] = pC[0+bs*1];
		pC[0+bs*1] = tmp;
		tmp = pA[0+bs*2];
		pA[0+bs*2] = pC[0+bs*2];
		pC[0+bs*2] = tmp;
		tmp = pA[0+bs*3];
		pA[0+bs*3] = pC[0+bs*3];
		pC[0+bs*3] = tmp;
		pA += 4*bs;
		pC += 4*bs;
		}
	for( ; ii<kmax; ii++)
		{
		tmp = pA[0+bs*0];
		pA[0+bs*0] = pC[0+bs*0];
		pC[0+bs*0] = tmp;
		pA += 1*bs;
		pC += 1*bs;
		}
	
	}
#endif



#if ! defined(BLASFEO)
// insert vector to column 
void dcolin_lib(int kmax, double *x, int offset, double *pD, int sdd)
	{

	const int bs = 4;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pD[ll] = x[ll];
			}
		pD += kna + bs*(sdd-1);
		x  += kna;
		kmax -= kna;
		}
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[jj*sdd+0] = x[jj+0];
		pD[jj*sdd+1] = x[jj+1];
		pD[jj*sdd+2] = x[jj+2];
		pD[jj*sdd+3] = x[jj+3];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+ll] = x[jj+ll];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// add scaled vector to column 
void dcolad_lib(int kmax, double alpha, double *x, int offset, double *pD, int sdd)
	{

	const int bs = 4;

	int kna = (bs-offset%bs)%bs;
	kna = kmax<kna ? kmax : kna;

	int jj, ll;

	if(kna>0)
		{
		for(ll=0; ll<kna; ll++)
			{
			pD[ll] += alpha * x[ll];
			}
		pD += kna + bs*(sdd-1);
		x  += kna;
		kmax -= kna;
		}
	for(jj=0; jj<kmax-3; jj+=4)
		{
		pD[jj*sdd+0] += alpha * x[jj+0];
		pD[jj*sdd+1] += alpha * x[jj+1];
		pD[jj*sdd+2] += alpha * x[jj+2];
		pD[jj*sdd+3] += alpha * x[jj+3];
		}
	for(ll=0; ll<kmax-jj; ll++)
		{
		pD[jj*sdd+ll] += alpha * x[jj+ll];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// insert vector to diagonal, sparse formulation 
void dcolin_libsp(int kmax, int *idx, double *x, double *pD, int sdd)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii/bs*bs*sdd+ii%bs] = x[jj];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// add scaled vector to diagonal, sparse formulation 
void dcolad_libsp(int kmax, double alpha, int *idx, double *x, double *pD, int sdd)
	{

	const int bs = 4;

	int ii, jj;

	for(jj=0; jj<kmax; jj++)
		{
		ii = idx[jj];
		pD[ii/bs*bs*sdd+ii%bs] += alpha * x[jj];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// insert vector to vector, sparse formulation
void dvecin_libsp(int kmax, int *idx, double *x, double *y)
	{

	int jj;

	for(jj=0; jj<kmax; jj++)
		{
		y[idx[jj]] = x[jj];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// adds vector to vector, sparse formulation
void dvecad_libsp(int kmax, int *idx, double alpha, double *x, double *y)
	{

	int jj;

	for(jj=0; jj<kmax; jj++)
		{
		y[idx[jj]] += alpha * x[jj];
		}
	
	}
#endif



#if ! defined(BLASFEO)
// copies a packed matrix into a packed matrix
void dgecp_lib(int m, int n, int offsetA, double *A, int sda, int offsetB, double *B, int sdb)
	{

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	int mna, ii;

	int offA = offsetA%bs;
	int offB = offsetB%bs;

	// A at the beginning of the block
	A -= offA;

	// A at the beginning of the block
	B -= offB;

	// same alignment
	if(offA==offB)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna) // mna<=3  ==>  m = { 1, 2 }
				{
				if(m==1)
					{
					kernel_dgecp_1_0_lib4(0, n, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgecp_2_0_lib4(0, n, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecp_1_0_lib4(0, n, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecp_2_0_lib4(0, n, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecp_3_0_lib4(0, n, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecp_8_0_lib4(0, n, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecp_4_0_lib4(0, n, A, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecp_1_0_lib4(0, n, A, B);
			else if(m-ii==2)
				kernel_dgecp_2_0_lib4(0, n, A, B);
			else // if(m-ii==3)
				kernel_dgecp_3_0_lib4(0, n, A, B);
			}
		}
	// skip one element of A
	else if(offA==(offB+1)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna) // mna<=3  ==>  m = { 1, 2 }
				{
				if(m==1)
					{
					kernel_dgecp_1_0_lib4(0, n, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgecp_2_0_lib4(0, n, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecp_1_0_lib4(0, n, A+offA, B+offB);
				//A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecp_2_3_lib4(0, n, A, sda, B+2);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecp_3_2_lib4(0, n, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
		for( ; ii<m-7; ii+=8)
			{
			kernel_dgecp_8_1_lib4(0, n, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for( ; ii<m-3; ii+=4)
			{
			kernel_dgecp_4_1_lib4(0, n, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecp_1_0_lib4(0, n, A+1, B);
			else if(m-ii==2)
				kernel_dgecp_2_0_lib4(0, n, A+1, B);
			else // if(m-ii==3)
				kernel_dgecp_3_0_lib4(0, n, A+1, B);
			}
		}
	// skip 2 elements of A
	else if(offA==(offB+2)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna)
				{
				if(m==1)
					{
					kernel_dgecp_1_0_lib4(0, n, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgecp_2_3_lib4(0, n, A, sda, B+1);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecp_1_0_lib4(0, n, A+1, B+3);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecp_2_0_lib4(0, n, A, B+2);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecp_3_3_lib4(0, n, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecp_8_2_lib4(0, n, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecp_4_2_lib4(0, n, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecp_1_0_lib4(0, n, A+2, B);
			else if(m-ii==2)
				kernel_dgecp_2_0_lib4(0, n, A+2, B);
			else // if(m-ii==3)
				kernel_dgecp_3_2_lib4(0, n, A, sda, B);
			}
		}
	// skip 3 elements of A
	else // if(offA==(offB+3)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna)
				{
				if(m==1)
					{
					kernel_dgecp_1_0_lib4(0, n, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgecp_2_0_lib4(0, n, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecp_1_0_lib4(0, n, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecp_2_0_lib4(0, n, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecp_3_0_lib4(0, n, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecp_8_3_lib4(0, n, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecp_4_3_lib4(0, n, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecp_1_0_lib4(0, n, A+3, B);
			else if(m-ii==2)
				kernel_dgecp_2_3_lib4(0, n, A, sda, B);
			else // if(m-ii==3)
				kernel_dgecp_3_3_lib4(0, n, A, sda, B);
			}
		}

	}
#endif



#if ! defined(BLASFEO)
// copies a lower triangular packed matrix into a lower triangular packed matrix
void dtrcp_l_lib(int m, int offsetA, double *A, int sda, int offsetB, double *B, int sdb)
	{

	if(m<=0)
		return;
	
	int n = m;

	const int bs = 4;

	int mna, ii;

	int offA = offsetA%bs;
	int offB = offsetB%bs;

	// A at the beginning of the block
	A -= offA;

	// A at the beginning of the block
	B -= offB;

	// same alignment
	if(offA==offB)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna) // mna<=3  ==>  m = { 1, 2 }
				{
				if(m==1)
					{
					kernel_dgecp_1_0_lib4(1, ii, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgecp_2_0_lib4(1, ii, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecp_1_0_lib4(1, ii, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecp_2_0_lib4(1, ii, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecp_3_0_lib4(1, ii, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecp_8_0_lib4(1, ii, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecp_4_0_lib4(1, ii, A, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecp_1_0_lib4(1, ii, A, B);
			else if(m-ii==2)
				kernel_dgecp_2_0_lib4(1, ii, A, B);
			else // if(m-ii==3)
				kernel_dgecp_3_0_lib4(1, ii, A, B);
			}
		}
	// skip one element of A
	else if(offA==(offB+1)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna) // mna<=3  ==>  m = { 1, 2 }
				{
				if(m==1)
					{
					kernel_dgecp_1_0_lib4(1, ii, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgecp_2_0_lib4(1, ii, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecp_1_0_lib4(1, ii, A+offA, B+offB);
				//A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecp_2_3_lib4(1, ii, A, sda, B+2);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecp_3_2_lib4(1, ii, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
		for( ; ii<m-7; ii+=8)
			{
			kernel_dgecp_8_1_lib4(1, ii, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for( ; ii<m-3; ii+=4)
			{
			kernel_dgecp_4_1_lib4(1, ii, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecp_1_0_lib4(1, ii, A+1, B);
			else if(m-ii==2)
				kernel_dgecp_2_0_lib4(1, ii, A+1, B);
			else // if(m-ii==3)
				kernel_dgecp_3_0_lib4(1, ii, A+1, B);
			}
		}
	// skip 2 elements of A
	else if(offA==(offB+2)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna)
				{
				if(m==1)
					{
					kernel_dgecp_1_0_lib4(1, ii, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgecp_2_3_lib4(1, ii, A, sda, B+1);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecp_1_0_lib4(1, ii, A+1, B+3);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecp_2_0_lib4(1, ii, A, B+2);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecp_3_3_lib4(1, ii, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecp_8_2_lib4(1, ii, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecp_4_2_lib4(1, ii, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecp_1_0_lib4(1, ii, A+2, B);
			else if(m-ii==2)
				kernel_dgecp_2_0_lib4(1, ii, A+2, B);
			else // if(m-ii==3)
				kernel_dgecp_3_2_lib4(1, ii, A, sda, B);
			}
		}
	// skip 3 elements of A
	else // if(offA==(offB+3)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna)
				{
				if(m==1)
					{
					kernel_dgecp_1_0_lib4(1, ii, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgecp_2_0_lib4(1, ii, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgecp_1_0_lib4(1, ii, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgecp_2_0_lib4(1, ii, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgecp_3_0_lib4(1, ii, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgecp_8_3_lib4(1, ii, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgecp_4_3_lib4(1, ii, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgecp_1_0_lib4(1, ii, A+3, B);
			else if(m-ii==2)
				kernel_dgecp_2_3_lib4(1, ii, A, sda, B);
			else // if(m-ii==3)
				kernel_dgecp_3_3_lib4(1, ii, A, sda, B);
			}
		}

	}
#endif



#if ! defined(BLASFEO)
// scaled and adds a packed matrix into a packed matrix: B = B + alpha*A
void dgead_lib(int m, int n, double alpha, int offsetA, double *A, int sda, int offsetB, double *B, int sdb)
	{

	if(m<=0 || n<=0)
		return;

	const int bs = D_MR;

	int mna, ii;

	int offA = offsetA%bs;
	int offB = offsetB%bs;

	// A at the beginning of the block
	A -= offA;

	// A at the beginning of the block
	B -= offB;

	// same alignment
	if(offA==offB)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna) // mna<=3  ==>  m = { 1, 2 }
				{
				if(m==1)
					{
					kernel_dgead_1_0_lib4(n, alpha, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgead_2_0_lib4(n, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgead_1_0_lib4(n, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgead_2_0_lib4(n, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgead_3_0_lib4(n, alpha, A+offA, B+offB);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgead_8_0_lib4(n, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgead_4_0_lib4(n, alpha, A, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgead_1_0_lib4(n, alpha, A, B);
			else if(m-ii==2)
				kernel_dgead_2_0_lib4(n, alpha, A, B);
			else // if(m-ii==3)
				kernel_dgead_3_0_lib4(n, alpha, A, B);
			}
		}
	// skip one element of A
	else if(offA==(offB+1)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna) // mna<=3  ==>  m = { 1, 2 }
				{
				if(m==1)
					{
					kernel_dgead_1_0_lib4(n, alpha, A+offA, B+offB);
					return;
					}
				else //if(m==2 && mna==3)
					{
					kernel_dgead_2_0_lib4(n, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgead_1_0_lib4(n, alpha, A+offA, B+offB);
				//A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgead_2_3_lib4(n, alpha, A, sda, B+2);
				A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgead_3_2_lib4(n, alpha, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
		for( ; ii<m-7; ii+=8)
			{
			kernel_dgead_8_1_lib4(n, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for( ; ii<m-3; ii+=4)
			{
			kernel_dgead_4_1_lib4(n, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgead_1_0_lib4(n, alpha, A+1, B);
			else if(m-ii==2)
				kernel_dgead_2_0_lib4(n, alpha, A+1, B);
			else // if(m-ii==3)
				kernel_dgead_3_0_lib4(n, alpha, A+1, B);
			}
		}
	// skip 2 elements of A
	else if(offA==(offB+2)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna)
				{
				if(m==1)
					{
					kernel_dgead_1_0_lib4(n, alpha, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgead_2_3_lib4(n, alpha, A, sda, B+1);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgead_1_0_lib4(n, alpha, A+1, B+3);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgead_2_0_lib4(n, alpha, A, B+2);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgead_3_3_lib4(n, alpha, A, sda, B+1);
				A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgead_8_2_lib4(n, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgead_4_2_lib4(n, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgead_1_0_lib4(n, alpha, A+2, B);
			else if(m-ii==2)
				kernel_dgead_2_0_lib4(n, alpha, A+2, B);
			else // if(m-ii==3)
				kernel_dgead_3_2_lib4(n, alpha, A, sda, B);
			}
		}
	// skip 3 elements of A
	else // if(offA==(offB+3)%bs)
		{
		ii = 0;
		// clean up at the beginning
		mna = (4-offB)%bs;
		if(mna>0)
			{
			if(m<mna)
				{
				if(m==1)
					{
					kernel_dgead_1_0_lib4(n, alpha, A+offA, B+offB);
					return;
					}
				else // if(m==2 && mna==3)
					{
					kernel_dgead_2_0_lib4(n, alpha, A+offA, B+offB);
					return;
					}
				}
			if(mna==1)
				{
				kernel_dgead_1_0_lib4(n, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 1;
				}
			else if(mna==2)
				{
				kernel_dgead_2_0_lib4(n, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 2;
				}
			else // if(mna==3)
				{
				kernel_dgead_3_0_lib4(n, alpha, A+offA, B+offB);
				// A += 4*sda;
				B += 4*sdb;
				ii += 3;
				}
			}
		// main loop
#if defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX2)
		for(; ii<m-7; ii+=8)
			{
			kernel_dgead_8_3_lib4(n, alpha, A, sda, B, sdb);
			A += 8*sda;
			B += 8*sdb;
			}
#endif
		for(; ii<m-3; ii+=4)
			{
			kernel_dgead_4_3_lib4(n, alpha, A, sda, B);
			A += 4*sda;
			B += 4*sdb;
			}
		// clean up at the end
		if(ii<m)
			{
			if(m-ii==1)
				kernel_dgead_1_0_lib4(n, alpha, A+3, B);
			else if(m-ii==2)
				kernel_dgead_2_3_lib4(n, alpha, A, sda, B);
			else // if(m-ii==3)
				kernel_dgead_3_3_lib4(n, alpha, A, sda, B);
			}
		}

	}
#endif



#if ! defined(BLASFEO)
// transpose general matrix; m and n are referred to the original matrix
void dgetr_lib(int m, int n, int offsetA, double *pA, int sda, int offsetC, double *pC, int sdc)
	{

/*

m = 5
n = 3
offsetA = 1
offsetC = 2

A = 
 x x x
 -
 x x x
 x x x
 x x x
 x x x

C =
 x x x x x
 x x x x x
 -
 x x x x x

*/

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	int mna = (bs-offsetA%bs)%bs;
	mna = m<mna ? m : mna;
	int nna = (bs-offsetC%bs)%bs;
	nna = n<nna ? n : nna;
	
	int ii;

	ii = 0;

	if(mna>0)
		{
		if(mna==1)
			kernel_dgetr_1_lib4(0, n, nna, pA, pC, sdc);
		else if(mna==2)
			kernel_dgetr_2_lib4(0, n, nna, pA, pC, sdc);
		else //if(mna==3)
			kernel_dgetr_3_lib4(0, n, nna, pA, pC, sdc);
		ii += mna;
		pA += mna + bs*(sda-1);
		pC += mna*bs;
		}
#if defined(TARGET_X64_AVX2)
	for( ; ii<m-7; ii+=8)
		{
		kernel_dgetr_8_lib4(0, n, nna, pA, sda, pC, sdc);
		pA += 2*bs*sda;
		pC += 2*bs*bs;
		}
#endif
	for( ; ii<m-3; ii+=4)
//	for( ; ii<m; ii+=4)
		{
		kernel_dgetr_4_lib4(0, n, nna, pA, pC, sdc);
		pA += bs*sda;
		pC += bs*bs;
		}

	// clean-up at the end using smaller kernels
	if(ii==m)
		return;
	
	if(m-ii==1)
		kernel_dgetr_1_lib4(0, n, nna, pA, pC, sdc);
	else if(m-ii==2)
		kernel_dgetr_2_lib4(0, n, nna, pA, pC, sdc);
	else if(m-ii==3)
		kernel_dgetr_3_lib4(0, n, nna, pA, pC, sdc);
		
	return;
	
	}	
#endif



#if ! defined(BLASFEO)
// transpose lower triangular matrix
void dtrtr_l_lib(int m, int offsetA, double *pA, int sda, int offsetC, double *pC, int sdc)
	{

/*

A = 
 x
 x x
 x x x
 x x x x
  
 x x x x x
 x x x x x x
 x x x x x x x
 x x x x x x x x

C =
 x x x x x x x x
  
   x x x x x x x
     x x x x x x
	   x x x x x
	     x x x x

	       x x x
	         x x
	           x

*/

	int n = m;

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	int mna = (bs-offsetA%bs)%bs;
	mna = m<mna ? m : mna;
	int nna = (bs-offsetC%bs)%bs;
	nna = n<nna ? n : nna;
	
	int ii;

	ii = 0;

	if(mna>0)
		{
		if(mna==1)
			{
			pC[0] = pA[0];
			}
		else if(mna==2)
			{
			if(nna==1)
				{
				pC[0+bs*0] = pA[0+bs*0];
				pC[0+bs*1] = pA[1+bs*0];
				pC[1+bs*(0+sdc)] = pA[1+bs*1];
				}
			else
				{
				pC[0+bs*0] = pA[0+bs*0];
				pC[0+bs*1] = pA[1+bs*0];
				pC[1+bs*1] = pA[1+bs*1];
				}
			}
		else //if(mna==3)
			{
			if(nna==1)
				{
				pC[0+bs*0] = pA[0+bs*0];
				pC[0+bs*1] = pA[1+bs*0];
				pC[0+bs*2] = pA[2+bs*0];
				pC[1+bs*(0+sdc)] = pA[1+bs*1];
				pC[1+bs*(1+sdc)] = pA[2+bs*1];
				pC[2+bs*(1+sdc)] = pA[2+bs*2];
				}
			else if(nna==2)
				{
				pC[0+bs*0] = pA[0+bs*0];
				pC[0+bs*1] = pA[1+bs*0];
				pC[0+bs*2] = pA[2+bs*0];
				pC[1+bs*1] = pA[1+bs*1];
				pC[1+bs*2] = pA[2+bs*1];
				pC[2+bs*(1+sdc)] = pA[2+bs*2];
				}
			else
				{
				pC[0+bs*0] = pA[0+bs*0];
				pC[0+bs*1] = pA[1+bs*0];
				pC[0+bs*2] = pA[2+bs*0];
				pC[1+bs*1] = pA[1+bs*1];
				pC[1+bs*2] = pA[2+bs*1];
				pC[2+bs*2] = pA[2+bs*2];
				}
			}
		ii += mna;
		pA += mna + bs*(sda-1);
		pC += mna*bs;
		}
#if 0 //defined(TARGET_X64_AVX2)
	for( ; ii<m-7; ii+=8)
		{
		kernel_dgetr_8_lib4(1, n, nna, pA, sda, pC, sdc);
		pA += 2*bs*sda;
		pC += 2*bs*bs;
		}
#endif
	for( ; ii<m-3; ii+=4)
		{
		kernel_dgetr_4_lib4(1, ii, nna, pA, pC, sdc);
		pA += bs*sda;
		pC += bs*bs;
		}
	
	// clean-up at the end using smaller kernels
	if(ii==m)
		return;
	
	if(m-ii==1)
		kernel_dgetr_1_lib4(1, ii, nna, pA, pC, sdc);
	else if(m-ii==2)
		kernel_dgetr_2_lib4(1, ii, nna, pA, pC, sdc);
	else if(m-ii==3)
		kernel_dgetr_3_lib4(1, ii, nna, pA, pC, sdc);
		
	return;

	}
#endif



#if ! defined(BLASFEO)
// transpose an aligned upper triangular matrix into an aligned lower triangular matrix
void dtrtr_u_lib(int m, int offsetA, double *pA, int sda, int offsetC, double *pC, int sdc)
	{

/*

A = 
 x x x x x x x x
   x x x x x x x

     x x x x x x
       x x x x x
         x x x x
           x x x
             x x
               x

C = 
 x

 x x
 x x x
 x x x x
 x x x x x
 x x x x x x
 x x x x x x x
 x x x x x x x x

*/

	int n = m;

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	int mna = (bs-offsetA%bs)%bs;
	mna = m<mna ? m : mna;
	int nna = (bs-offsetC%bs)%bs;
	nna = n<nna ? n : nna;
	int tna = nna;
	
	int ii;

	ii = 0;

	if(mna>0)
		{
		if(mna==1)
			{
			kernel_dgetr_1_lib4(0, n, nna, pA, pC, sdc);
			if(nna!=1)
				{
//				pC[0+bs*0] = pA[0+bs*0];
				pA += 1*bs;
				pC += 1;
				tna = (bs-(offsetC+1)%bs)%bs;
				}
			else //if(nna==1)
				{
//				pC[0+bs*0] = pA[0+bs*0];
				pA += 1*bs;
				pC += 1 + (sdc-1)*bs;
				tna = 0; //(bs-(offsetC+1)%bs)%bs;
				}
//			kernel_dgetr_1_lib4(0, n-1, tna, pA, pC, sdc);
			}
		else if(mna==2)
			{
			if(nna==0 || nna==3)
				{
				pC[0+bs*0] = pA[0+bs*0];
				pC[1+bs*0] = pA[0+bs*1];
				pC[1+bs*1] = pA[1+bs*1];
				pA += 2*bs;
				pC += 2;
				tna = (bs-(offsetC+2)%bs)%bs;
				kernel_dgetr_2_lib4(0, n-2, tna, pA, pC, sdc);
				}
			else if(nna==1)
				{
				pC[0+bs*0] = pA[0+bs*0];
				pA += 1*bs;
				pC += 1 + (sdc-1)*bs;
//				pC[0+bs*0] = pA[0+bs*0];
//				pC[0+bs*1] = pA[1+bs*0];
				kernel_dgetr_2_lib4(0, n-1, 0, pA, pC, sdc);
				pA += 1*bs;
				pC += 1;
				tna = 3; //(bs-(offsetC+2)%bs)%bs;
//				kernel_dgetr_2_lib4(0, n-2, tna, pA, pC, sdc);
				}
			else if(nna==2)
				{
				pC[0+bs*0] = pA[0+bs*0];
				pC[1+bs*0] = pA[0+bs*1];
				pC[1+bs*1] = pA[1+bs*1];
				pA += 2*bs;
				pC += 2 + (sdc-1)*bs;
				tna = 0; //(bs-(offsetC+2)%bs)%bs;
				kernel_dgetr_2_lib4(0, n-2, tna, pA, pC, sdc);
				}
			}
		else //if(mna==3)
			{
			if(nna==0)
				{
				pC[0+bs*0] = pA[0+bs*0];
				pC[1+bs*0] = pA[0+bs*1];
				pC[1+bs*1] = pA[1+bs*1];
				pC[2+bs*0] = pA[0+bs*2];
				pC[2+bs*1] = pA[1+bs*2];
				pC[2+bs*2] = pA[2+bs*2];
				pA += 3*bs;
				pC += 3;
				tna = 1;
				kernel_dgetr_3_lib4(0, n-3, tna, pA, pC, sdc);
				}
			else if(nna==1)
				{
				pC[0+bs*0] = pA[0+bs*0];
				pA += bs;
				pC += 1 + (sdc-1)*bs;
				pC[0+bs*0] = pA[0+bs*0];
				pC[0+bs*1] = pA[1+bs*0];
				pC[1+bs*0] = pA[0+bs*1];
				pC[1+bs*1] = pA[1+bs*1];
				pC[1+bs*2] = pA[2+bs*1];
				pA += 2*bs;
				pC += 2;
				tna = 2;
				kernel_dgetr_3_lib4(0, n-3, tna, pA, pC, sdc);
				}
			else if(nna==2)
				{
				pC[0+bs*0] = pA[0+bs*0];
				pC[1+bs*0] = pA[0+bs*1];
				pC[1+bs*1] = pA[1+bs*1];
				pA += 2*bs;
				pC += 2 + (sdc-1)*bs;
//				pC[0+bs*0] = pA[0+bs*0];
//				pC[0+bs*1] = pA[1+bs*0];
//				pC[0+bs*2] = pA[2+bs*0];
				kernel_dgetr_3_lib4(0, n-2, 0, pA, pC, sdc);
				pA += 1*bs;
				pC += 1;
				tna = 3;
//				kernel_dgetr_3_lib4(0, n-3, tna, pA, pC, sdc);
				}
			else //if(nna==3)
				{
				pC[0+bs*0] = pA[0+bs*0];
				pC[1+bs*0] = pA[0+bs*1];
				pC[1+bs*1] = pA[1+bs*1];
				pC[2+bs*0] = pA[0+bs*2];
				pC[2+bs*1] = pA[1+bs*2];
				pC[2+bs*2] = pA[2+bs*2];
				pA += 3*bs;
				pC += 3 + (sdc-1)*bs;
				tna = 0;
				kernel_dgetr_3_lib4(0, n-3, tna, pA, pC, sdc);
				}
			}
		ii += mna;
		pA += mna + bs*(sda-1);
		pC += mna*bs;
		}
#if 0 //defined(TARGET_X64_AVX2)
	for( ; ii<m-7; ii+=8)
		{
		kernel_dgetr_8_lib4(0, n, nna, pA, sda, pC, sdc);
		pA += 2*bs*sda;
		pC += 2*bs*bs;
		}
#endif
	for( ; ii<m-3; ii+=4)
		{
		if(tna==0)
			{
			pC[0+bs*0] = pA[0+bs*0];
			pC[1+bs*0] = pA[0+bs*1];
			pC[1+bs*1] = pA[1+bs*1];
			pC[2+bs*0] = pA[0+bs*2];
			pC[2+bs*1] = pA[1+bs*2];
			pC[2+bs*2] = pA[2+bs*2];
			pC[3+bs*0] = pA[0+bs*3];
			pC[3+bs*1] = pA[1+bs*3];
			pC[3+bs*2] = pA[2+bs*3];
			pC[3+bs*3] = pA[3+bs*3];
			pA += 4*bs;
			pC += sdc*bs;
			kernel_dgetr_4_lib4(0, n-ii-4, 0, pA, pC, sdc);
			}
		else if(tna==1)
			{
			pC[0+bs*0] = pA[0+bs*0];
			pA += bs;
			pC += 1 + (sdc-1)*bs;
			pC[0+bs*0] = pA[0+bs*0];
			pC[0+bs*1] = pA[1+bs*0];
			pC[1+bs*0] = pA[0+bs*1];
			pC[1+bs*1] = pA[1+bs*1];
			pC[1+bs*2] = pA[2+bs*1];
			pC[2+bs*0] = pA[0+bs*2];
			pC[2+bs*1] = pA[1+bs*2];
			pC[2+bs*2] = pA[2+bs*2];
			pC[2+bs*3] = pA[3+bs*2];
			pA += 3*bs;
			pC += 3;
			kernel_dgetr_4_lib4(0, n-ii-4, 1, pA, pC, sdc);
			}
		else if(tna==2)
			{
			pC[0+bs*0] = pA[0+bs*0];
			pC[1+bs*0] = pA[0+bs*1];
			pC[1+bs*1] = pA[1+bs*1];
			pA += 2*bs;
			pC += 2 + (sdc-1)*bs;
			pC[0+bs*0] = pA[0+bs*0];
			pC[0+bs*1] = pA[1+bs*0];
			pC[0+bs*2] = pA[2+bs*0];
			pC[1+bs*0] = pA[0+bs*1];
			pC[1+bs*1] = pA[1+bs*1];
			pC[1+bs*2] = pA[2+bs*1];
			pC[1+bs*3] = pA[3+bs*1];
			pA += 2*bs;
			pC += 2;
			kernel_dgetr_4_lib4(0, n-ii-4, 2, pA, pC, sdc);
			}
		else //if(tna==3)
			{
			pC[0+bs*0] = pA[0+bs*0];
			pC[1+bs*0] = pA[0+bs*1];
			pC[1+bs*1] = pA[1+bs*1];
			pC[2+bs*0] = pA[0+bs*2];
			pC[2+bs*1] = pA[1+bs*2];
			pC[2+bs*2] = pA[2+bs*2];
			pA += 3*bs;
			pC += 3 + (sdc-1)*bs;
			kernel_dgetr_4_lib4(0, n-ii-3, 0, pA, pC, sdc);
//			pC[0+bs*0] = pA[0+bs*0];
//			pC[0+bs*1] = pA[1+bs*0];
//			pC[0+bs*2] = pA[2+bs*0];
//			pC[0+bs*3] = pA[3+bs*0];
			pA += bs;
			pC += 1;
//			kernel_dgetr_4_lib4(0, n-ii-4, tna, pA, pC, sdc);
			}
		pA += bs*sda;
		pC += bs*bs;
		}

	// clean-up at the end
	if(ii==m)
		return;
	
	if(m-ii==1)
		{
		pC[0+bs*0] = pA[0+bs*0];
		}
	else if(m-ii==2)
		{
		if(tna!=1)
			{
			pC[0+bs*0] = pA[0+bs*0];
			pC[1+bs*0] = pA[0+bs*1];
			pC[1+bs*1] = pA[1+bs*1];
			}
		else //if(tna==1)
			{
			pC[0+bs*0] = pA[0+bs*0];
			pA += bs;
			pC += 1 + (sdc-1)*bs;
			pC[0+bs*0] = pA[0+bs*0];
			pC[0+bs*1] = pA[1+bs*0];
			}
		}
	else if(m-ii==3)
		{
		if(tna==0 || tna==3)
			{
			pC[0+bs*0] = pA[0+bs*0];
			pC[1+bs*0] = pA[0+bs*1];
			pC[1+bs*1] = pA[1+bs*1];
			pC[2+bs*0] = pA[0+bs*2];
			pC[2+bs*1] = pA[1+bs*2];
			pC[2+bs*2] = pA[2+bs*2];
			}
		else if(tna==1)
			{
			pC[0+bs*0] = pA[0+bs*0];
			pA += bs;
			pC += 1 + (sdc-1)*bs;
			pC[0+bs*0] = pA[0+bs*0];
			pC[0+bs*1] = pA[1+bs*0];
			pC[1+bs*0] = pA[0+bs*0];
			pC[1+bs*1] = pA[1+bs*1];
			pC[1+bs*2] = pA[2+bs*1];
			}
		else //if(tna==2)
			{
			pC[0+bs*0] = pA[0+bs*0];
			pC[1+bs*0] = pA[0+bs*1];
			pC[1+bs*1] = pA[1+bs*1];
			pA += 2*bs;
			pC += 2 + (sdc-1)*bs;
			pC[0+bs*0] = pA[0+bs*0];
			pC[0+bs*1] = pA[1+bs*0];
			pC[0+bs*2] = pA[2+bs*0];
			}
		}
		
	return;

	}
#endif



void dlauum_lib(int m, double *pA, int sda, double *pB, int sdb, int alg, double *pC, int sdc, double *pD, int sdd)
	{

	if(m<=0)
		return;

	const int bs = 4;

	int ii, jj;
	
	ii = 0;
// TODO unify kernels for dtrmm_l_u and dlauum using a flag !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
#if defined(TARGET_X64_AVX2)
	for( ; ii<m-11; ii+=12)
		{
		// off-diagonal blocks
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dtrmm_l_u_nt_12x4_lib4(m-ii, pA+ii*sda+ii*bs, sda, pB+jj*sdb+ii*bs, pC+ii*sdc+jj*bs, sdc, pD+ii*sdd+jj*bs, sdd, alg);
			}
		// diagonal block
		kernel_dlauum_nt_12x4_lib4(m-ii, pA+ii*sda+ii*bs, sda, pB+ii*sdb+ii*bs, alg, pC+ii*sdc+ii*bs, sdc, pD+ii*sdd+ii*bs, sdd);
		kernel_dlauum_nt_8x8_lib4(m-ii-4, pA+(ii+4)*sda+(ii+4)*bs, sda, pB+(ii+4)*sdb+(ii+4)*bs, sdb, alg, pC+(ii+4)*sdc+(ii+4)*bs, sdc, pD+(ii+4)*sdd+(ii+4)*bs, sdd);
//		kernel_dlauum_nt_8x4_lib4(m-ii-4, pA+(ii+4)*sda+(ii+4)*bs, sda, pB+(ii+4)*sdb+(ii+4)*bs, alg, pC+(ii+4)*sdc+(ii+4)*bs, sdc, pD+(ii+4)*sdd+(ii+4)*bs, sdd);
//		kernel_dlauum_nt_4x4_lib4(m-ii-8, pA+(ii+8)*sda+(ii+8)*bs, pB+(ii+8)*sdb+(ii+8)*bs, alg, pC+(ii+8)*sdc+(ii+8)*bs, pD+(ii+8)*sdd+(ii+8)*bs);
		}
	for( ; ii<m-7; ii+=8)
		{
		// off-diagonal blocks
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dtrmm_l_u_nt_8x4_lib4(m-ii, pA+ii*sda+ii*bs, sda, pB+jj*sdb+ii*bs, pC+ii*sdc+jj*bs, sdc, pD+ii*sdd+jj*bs, sdd, alg);
			}
		// diagonal block
		kernel_dlauum_nt_8x8_lib4(m-ii, pA+ii*sda+ii*bs, sda, pB+ii*sdb+ii*bs, sdb, alg, pC+ii*sdc+ii*bs, sdc, pD+ii*sdd+ii*bs, sdd);
//		kernel_dlauum_nt_8x4_lib4(m-ii, pA+ii*sda+ii*bs, sda, pB+ii*sdb+ii*bs, alg, pC+ii*sdc+ii*bs, sdc, pD+ii*sdd+ii*bs, sdd);
//		kernel_dlauum_nt_4x4_lib4(m-ii-4, pA+(ii+4)*sda+(ii+4)*bs, pB+(ii+4)*sdb+(ii+4)*bs, alg, pC+(ii+4)*sdc+(ii+4)*bs, pD+(ii+4)*sdd+(ii+4)*bs);
		}
#endif
#if defined(TARGET_X64_AVX)
	for( ; ii<m-7; ii+=8)
		{
		// off-diagonal blocks
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dtrmm_l_u_nt_8x4_lib4(m-ii, pA+ii*sda+ii*bs, sda, pB+jj*sdb+ii*bs, pC+ii*sdc+jj*bs, sdc, pD+ii*sdd+jj*bs, sdd, alg);
			}
		// diagonal block
		kernel_dlauum_nt_8x4_lib4(m-ii, pA+ii*sda+ii*bs, sda, pB+ii*sdb+ii*bs, alg, pC+ii*sdc+ii*bs, sdc, pD+ii*sdd+ii*bs, sdd);
		kernel_dlauum_nt_4x4_lib4(m-ii-4, pA+(ii+4)*sda+(ii+4)*bs, pB+(ii+4)*sdb+(ii+4)*bs, alg, pC+(ii+4)*sdc+(ii+4)*bs, pD+(ii+4)*sdd+(ii+4)*bs);
		}
#endif
	for( ; ii<m-3; ii+=4)
		{
		// off-diagonal blocks
		for(jj=0; jj<ii; jj+=4)
			{
			kernel_dtrmm_l_u_nt_4x4_lib4(m-ii, pA+ii*sda+ii*bs, pB+jj*sdb+ii*bs, pC+ii*sdc+jj*bs, pD+ii*sdd+jj*bs, alg);
			}
		// diagonal block
		kernel_dlauum_nt_4x4_lib4(m-ii, pA+ii*sda+ii*bs, pB+ii*sdb+ii*bs, alg, pC+ii*sdc+ii*bs, pD+ii*sdd+ii*bs);
		}
	
	if(m-ii>1)
		{
		if(m-ii==3) // 3
			{
			// off-diagonal blocks
			for(jj=0; jj<ii; jj+=4)
				{
				//kernel_dtrmm_l_u_nt_4x4_vs_lib4(3, 3, pA+ii*sda+ii*bs, pB+jj*sdb+ii*bs, pC+ii*sdc+jj*bs, pD+ii*sdd+jj*bs, alg);
				corner_dtrmm_l_u_nt_3x4_lib4(pA+ii*sda+ii*bs, pB+jj*sdb+ii*bs, alg, pC+ii*sdc+jj*bs, pD+ii*sdd+jj*bs);
				}
			// diagonal block
			corner_dlauum_nt_3x3_lib4(pA+ii*sda+ii*bs, pB+ii*sdb+ii*bs, alg, pC+ii*sdc+ii*bs, pD+ii*sdd+ii*bs);
			}
		else // 2
			{
			// off-diagonal blocks
			for(jj=0; jj<ii; jj+=4)
				{
				//kernel_dtrmm_l_u_nt_4x4_vs_lib4(2, 2, pA+ii*sda+ii*bs, pB+jj*sdb+ii*bs, pC+ii*sdc+jj*bs, pD+ii*sdd+jj*bs, alg);
				corner_dtrmm_l_u_nt_2x4_lib4(pA+ii*sda+ii*bs, pB+jj*sdb+ii*bs, alg, pC+ii*sdc+jj*bs, pD+ii*sdd+jj*bs);
				}
			// diagonal block
			corner_dlauum_nt_2x2_lib4(pA+ii*sda+ii*bs, pB+ii*sdb+ii*bs, alg, pC+ii*sdc+ii*bs, pD+ii*sdd+ii*bs);
			}
		}
	else
		{
		if(m-ii==1) // 1
			{
			// off-diagonal blocks
			for(jj=0; jj<ii; jj+=4)
				{
				//kernel_dtrmm_l_u_nt_4x4_vs_lib4(1, 1, pA+ii*sda+ii*bs, pB+jj*sdb+ii*bs, pC+ii*sdc+jj*bs, pD+ii*sdd+jj*bs, alg);
				corner_dtrmm_l_u_nt_1x4_lib4(pA+ii*sda+ii*bs, pB+jj*sdb+ii*bs, alg, pC+ii*sdc+jj*bs, pD+ii*sdd+jj*bs);
				}
			// diagonal block
			corner_dlauum_nt_1x1_lib4(pA+ii*sda+ii*bs, pB+ii*sdb+ii*bs, alg, pC+ii*sdc+ii*bs, pD+ii*sdd+ii*bs);
			}
		}
	
	return;

	}



void dtrtri_lib(int m, double *pA, int sda, int use_inv_diag_A, double *inv_diag_A, double *pC, int sdc)
	{

	if(m<=0)
		return;
	
	const int bs = 4;

	int ii, jj;

	int n = m; // just to distinguish between rows and colscan be removed

	ii = 0;
#if defined(TARGET_X64_AVX2)
	for( ; ii<m-11; ii+=12)
		{
		jj = ii;
		corner_dtrtri_8x8_lib4(&pA[jj*sda+jj*bs], sda, use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs], sdc);
		jj += 8;
		corner_dtrtri_12x4_lib4(&pC[ii*sdc+ii*bs], sdc, &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], sdc, &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
		jj += 4;
		for( ; jj<n-3; jj+=4)
			{
			kernel_dtrtri_12x4_lib4(jj-ii, &pC[ii*sdc+ii*bs], sdc, &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], sdc, &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
			}
		if(n-jj>1)
			{
			if(n-jj==3) // 3
				{
				kernel_dtrtri_12x3_lib4(jj-ii, &pC[ii*sdc+ii*bs], sdc, &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], sdc, &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
				}
			else // 2
				{
				kernel_dtrtri_12x2_lib4(jj-ii, &pC[ii*sdc+ii*bs], sdc, &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], sdc, &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
				}
			}
		else
			{
			if(n-jj==1)
				{
				kernel_dtrtri_12x1_lib4(jj-ii, &pC[ii*sdc+ii*bs], sdc, &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], sdc, &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
				}
			}
		}
	jj = ii;
	if(m-ii>=8)
		{
		corner_dtrtri_8x8_lib4(&pA[jj*sda+jj*bs], sda, use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs], sdc);
		jj += 8;
		if(m-ii>9)
			{
			if(m-ii==11)
				{
				corner_dtrtri_11x3_lib4(&pC[ii*sdc+ii*bs], sdc, &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], sdc, &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
				}
			else // 10
				{
				corner_dtrtri_10x2_lib4(&pC[ii*sdc+ii*bs], sdc, &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], sdc, &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
				}
			}
		else
			{
			if(m-ii==9)
				{
				corner_dtrtri_9x1_lib4(&pC[ii*sdc+ii*bs], sdc, &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], sdc, &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
				}
			}
		return;
		}
	if(m-ii>3)
		{
		if(m-ii>5)
			{
			if(m-ii==7) // 7
				{
				corner_dtrtri_7x7_lib4(&pA[jj*sda+jj*bs], sda, use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs], sdc);
				}
			else // 6
				{
				corner_dtrtri_6x6_lib4(&pA[jj*sda+jj*bs], sda, use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs], sdc);
				}
			}
		else
			{
			if(m-ii==5) // 5
				{
				corner_dtrtri_5x5_lib4(&pA[jj*sda+jj*bs], sda, use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs], sdc);
				}
			else // 4
				{
				corner_dtrtri_4x4_lib4(&pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs]);
				}
			}
		}
	else
		{
		if(m-ii>1)
			{
			if(m-ii==3) // 3
				{
				corner_dtrtri_3x3_lib4(&pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs]);
				}
			else // 2
				{
				corner_dtrtri_2x2_lib4(&pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs]);
				}
			}
		else
			{
			if(m-ii==1) // 1
				{
				corner_dtrtri_1x1_lib4(&pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs]);
				}
			}
		}
#endif
#if defined(TARGET_X64_AVX)
	for( ; ii<m-7; ii+=8)
		{
		jj = ii;
		corner_dtrtri_8x8_lib4(&pA[jj*sda+jj*bs], sda, use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs], sdc);
		jj += 8;
		for( ; jj<n-3; jj+=4)
			{
			kernel_dtrtri_8x4_lib4(jj-ii, &pC[ii*sdc+ii*bs], sdc, &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], sdc, &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
			}
		if(n-jj>1)
			{
			if(n-jj==3) // 3
				{
				kernel_dtrtri_8x3_lib4(jj-ii, &pC[ii*sdc+ii*bs], sdc, &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], sdc, &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
				}
			else // 2
				{
				kernel_dtrtri_8x2_lib4(jj-ii, &pC[ii*sdc+ii*bs], sdc, &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], sdc, &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
				}
			}
		else
			{
			if(n-jj==1)
				{
				kernel_dtrtri_8x1_lib4(jj-ii, &pC[ii*sdc+ii*bs], sdc, &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], sdc, &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
				}
			}
		}
	jj = ii;
	if(m-ii>3)
		{
		if(m-ii>5)
			{
			if(m-ii==7) // 7
				{
				corner_dtrtri_7x7_lib4(&pA[jj*sda+jj*bs], sda, use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs], sdc);
				}
			else // 6
				{
				corner_dtrtri_6x6_lib4(&pA[jj*sda+jj*bs], sda, use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs], sdc);
				}
			}
		else
			{
			if(m-ii==5) // 5
				{
				corner_dtrtri_5x5_lib4(&pA[jj*sda+jj*bs], sda, use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs], sdc);
				}
			else // 4
				{
				corner_dtrtri_4x4_lib4(&pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs]);
				}
			}
		}
	else
		{
		if(m-ii>1)
			{
			if(m-ii==3) // 3
				{
				corner_dtrtri_3x3_lib4(&pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs]);
				}
			else // 2
				{
				corner_dtrtri_2x2_lib4(&pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs]);
				}
			}
		else
			{
			if(m-ii==1) // 1
				{
				corner_dtrtri_1x1_lib4(&pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs]);
				}
			}
		}
#endif
#if defined(TARGET_X64_SSE3) || defined(TARGET_C99_4X4) || defined(TARGET_CORTEX_A15) || defined(TARGET_CORTEX_A9) || defined(TARGET_CORTEX_A7) || defined(TARGET_CORTEX_A57)
	for( ; ii<m-3; ii+=4)
		{
		jj = ii;
		corner_dtrtri_4x4_lib4(&pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs]);
		jj += 4;
		for( ; jj<n-3; jj+=4)
			{
			kernel_dtrtri_4x4_lib4(jj-ii, &pC[ii*sdc+ii*bs], &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
			}
		if(n-jj>1)
			{
			if(n-jj==3) // 3
				{
				kernel_dtrtri_4x3_lib4(jj-ii, &pC[ii*sdc+ii*bs], &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
				}
			else // 2
				{
				kernel_dtrtri_4x2_lib4(jj-ii, &pC[ii*sdc+ii*bs], &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
				}
			}
		else
			{
			if(n-jj==1)
				{
				kernel_dtrtri_4x1_lib4(jj-ii, &pC[ii*sdc+ii*bs], &pA[jj*sda+ii*bs], &pC[ii*sdc+jj*bs], &pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj]);
				}
			}
		}
	jj = ii;
	if(m-ii>1)
		{
		if(m-ii==3) // 3
			{
			corner_dtrtri_3x3_lib4(&pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs]);
			}
		else // 2
			{
			corner_dtrtri_2x2_lib4(&pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs]);
			}
		}
	else
		{
		if(m-ii==1) // 1
			{
			corner_dtrtri_1x1_lib4(&pA[jj*sda+jj*bs], use_inv_diag_A, &inv_diag_A[jj], &pC[jj*sdc+jj*bs]);
			}
		}
#endif
	
	return;

	}



#if ! defined(BLASFEO)
void dgemm_diag_left_lib(int m, int n, double *dA, double *pB, int sdb, int alg, double *pC, int sdc, double *pD, int sdd)
	{

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	int ii;

	ii = 0;
	for( ; ii<m-3; ii+=4)
		{
		kernel_dgemm_diag_left_4_lib4(n, &dA[ii], &pB[ii*sdb], &pC[ii*sdc], &pD[ii*sdd], alg);
		}
	if(m-ii>0)
		{
		if(m-ii==1)
			kernel_dgemm_diag_left_1_lib4(n, &dA[ii], &pB[ii*sdb], &pC[ii*sdc], &pD[ii*sdd], alg);
		else if(m-ii==2)
			kernel_dgemm_diag_left_2_lib4(n, &dA[ii], &pB[ii*sdb], &pC[ii*sdc], &pD[ii*sdd], alg);
		else // if(m-ii==3)
			kernel_dgemm_diag_left_3_lib4(n, &dA[ii], &pB[ii*sdb], &pC[ii*sdc], &pD[ii*sdd], alg);
		}
	
	}
#endif



#if ! defined(BLASFEO)
void dgemm_diag_right_lib(int m, int n, double *pA, int sda, double *dB, int alg, double *pC, int sdc, double *pD, int sdd)
	{

	if(m<=0 || n<=0)
		return;

	const int bs = 4;

	int ii;

	ii = 0;
	for( ; ii<n-3; ii+=4)
		{
		kernel_dgemm_diag_right_4_lib4(m, &pA[ii*bs], sda, &dB[ii], &pC[ii*bs], sdc, &pD[ii*bs], sdd, alg);
		}
	if(n-ii>0)
		{
		if(n-ii==1)
			kernel_dgemm_diag_right_1_lib4(m, &pA[ii*bs], sda, &dB[ii], &pC[ii*bs], sdc, &pD[ii*bs], sdd, alg);
		else if(n-ii==2)
			kernel_dgemm_diag_right_2_lib4(m, &pA[ii*bs], sda, &dB[ii], &pC[ii*bs], sdc, &pD[ii*bs], sdd, alg);
		else // if(n-ii==3)
			kernel_dgemm_diag_right_3_lib4(m, &pA[ii*bs], sda, &dB[ii], &pC[ii*bs], sdc, &pD[ii*bs], sdd, alg);
		}
	
	}
#endif



void dsyrk_diag_left_right_lib(int m, double *dAl, double *dAr, double *pB, int sdb, int alg, double *pC, int sdc, double *pD, int sdd)
	{

	if(m<=0)
		return;

	const int bs = 4;

	int ii;

	for(ii=0; ii<m-3; ii+=4)
		{
		kernel_dsyrk_diag_left_right_4_lib4(ii+4, &dAl[ii], &dAr[0], &pB[ii*sdb], &pC[ii*sdc], &pD[ii*sdd], alg);
		}
	if(m-ii>0)
		{
		if(m-ii==1)
			kernel_dsyrk_diag_left_right_1_lib4(ii+1, &dAl[ii], &dAr[0], &pB[ii*sdb], &pC[ii*sdc], &pD[ii*sdd], alg);
		else if(m-ii==2)
			kernel_dsyrk_diag_left_right_2_lib4(ii+2, &dAl[ii], &dAr[0], &pB[ii*sdb], &pC[ii*sdc], &pD[ii*sdd], alg);
		else // if(m-ii==3)
			kernel_dsyrk_diag_left_right_3_lib4(ii+3, &dAl[ii], &dAr[0], &pB[ii*sdb], &pC[ii*sdc], &pD[ii*sdd], alg);
		}
	
	}
			


void dgemv_diag_lib(int m, double *dA, double *x, int alg, double *y, double *z)
	{

	if(m<=0)
		return;
	
	kernel_dgemv_diag_lib4(m, dA, x, y, z, alg);

	}

	

#if ! defined(BLASFEO)
void dgetrf_lib(int m, int n, double *pC, int sdc, double *pD, int sdd, double *inv_diag_D)
	{

	if(m<=0 || n<=0)
		return;
	
	const int bs = 4;

	int ii, jj, ie;

	// main loop
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
	ii = 0;
	for( ; ii<m-7; ii+=8)
		{
		jj = 0;
		// solve lower
		ie = n<ii ? n : ii; // ii is multiple of 4
		for( ; jj<ie-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_8x4_lib4(jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &pD[jj*bs+jj*sdd], 1, &inv_diag_D[jj]);
			}
		if(jj<ie)
			{
			if(ie-jj==3)
				{
				kernel_dtrsm_nn_ru_8x4_vs_lib4(8, 3, jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &pD[jj*bs+jj*sdd], 1, &inv_diag_D[jj]);
				jj+=4;
				}
			else
				{
				kernel_dtrsm_nn_ru_8x2_vs_lib4(8, ie-jj, jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &pD[jj*bs+jj*sdd], 1, &inv_diag_D[jj]);
				jj+=2;
				}
			}
		// factorize
		if(jj<n-3)
			{
			kernel_dgetrf_l_nn_8x4_lib4(jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &inv_diag_D[jj]);
			jj+=4;
			}
		else if(jj<n)
			{
			if(n-jj==3)
				{
				kernel_dgetrf_l_nn_8x4_vs_lib4(8, 3, jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &inv_diag_D[jj]);
				jj+=4;
				}
			else
				{
				kernel_dgetrf_l_nn_8x2_vs_lib4(8, n-jj, jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &inv_diag_D[jj]);
				jj+=2;
				}
			}
		if(jj<n-3)
			{
			kernel_dgetrf_r_nn_8x4_lib4(ii, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &inv_diag_D[jj], &pD[ii*bs+ii*sdd], sdd);
			jj+=4;
			}
		else if(jj<n)
			{
			if(n-jj==3)
				{
				kernel_dgetrf_r_nn_8x4_vs_lib4(8, 3, ii, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &inv_diag_D[jj], &pD[ii*bs+ii*sdd], sdd);
				jj+=4;
				}
			else
				{
				kernel_dgetrf_r_nn_8x2_vs_lib4(8, n-jj, ii, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &inv_diag_D[jj], &pD[ii*bs+ii*sdd], sdd);
				jj+=2;
				}
			}
		// solve upper 
		for( ; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ll_diag_8x4_lib4(ii, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &pD[ii*bs+ii*sdd], sdd);
			}
		if(jj<n)
			{
			if(n-jj==3)
				{
				kernel_dtrsm_nn_ll_diag_8x4_vs_lib4(8, 3, ii, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &pD[ii*bs+ii*sdd], sdd);
				}
			else
				{
				kernel_dtrsm_nn_ll_diag_8x2_vs_lib4(8, n-jj, ii, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &pD[ii*bs+ii*sdd], sdd);
				}
			}
		}
	if(m>ii)
		{
		if(m-ii>4)
			goto left_8;
		else
			goto left_4;
		}
#else
	ii = 0;
	for( ; ii<m-3; ii+=4)
		{
		jj = 0;
		// solve lower
		ie = n<ii ? n : ii; // ie is multiple of 4
		for( ; jj<ie-3; jj+=4)
			{
			kernel_dtrsm_nn_ru_4x4_lib4(jj, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &pD[jj*bs+jj*sdd], 1, &inv_diag_D[jj]);
			}
		if(jj<ie)
			{
			if(ie-jj==3)
				{
				kernel_dtrsm_nn_ru_4x4_vs_lib4(4, 3, jj, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &pD[jj*bs+jj*sdd], 1, &inv_diag_D[jj]);
				jj+=4;
				}
			else
				{
				kernel_dtrsm_nn_ru_4x2_vs_lib4(4, ie-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &pD[jj*bs+jj*sdd], 1, &inv_diag_D[jj]);
				jj+=2;
				}
			}
		// factorize
		if(jj<n-3)
			{
			kernel_dgetrf_nn_4x4_lib4(jj, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &inv_diag_D[jj]);
			jj+=4;
			}
		else if(jj<n)
			{
			if(n-jj==3)
				{
				kernel_dgetrf_nn_4x4_vs_lib4(4, 3, jj, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &inv_diag_D[jj]);
				jj+=4;
				}
			else
				{
				kernel_dgetrf_nn_4x2_vs_lib4(4, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &inv_diag_D[jj]);
				jj+=2;
				}
			}
		// solve upper 
		for( ; jj<n-3; jj+=4)
			{
			kernel_dtrsm_nn_ll_diag_4x4_lib4(ii, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &pD[ii*bs+ii*sdd]);
			}
		if(jj<n)
			{
			if(n-jj==3)
				{
				kernel_dtrsm_nn_ll_diag_4x4_vs_lib4(4, 3, ii, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &pD[ii*bs+ii*sdd]);
				}
			else
				{
				kernel_dtrsm_nn_ll_diag_4x2_vs_lib4(m-ii, n-jj, ii, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &pD[ii*bs+ii*sdd]);
				}
			}
		}
	if(m>ii)
		{
		if(n-ii>2)
			goto left_4;
		else
			goto left_2;
		}

#endif

	// common return if i==m
	return;

#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
	left_8:
	jj = 0;
	// solve lower
	ie = n<ii ? n : ii; // ii is multiple of 4
	for( ; jj<ie-2; jj+=4)
		{
		kernel_dtrsm_nn_ru_8x4_vs_lib4(m-ii, ie-jj, jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &pD[jj*bs+jj*sdd], 1, &inv_diag_D[jj]);
		}
	if(jj<ie)
		{
		kernel_dtrsm_nn_ru_8x2_vs_lib4(m-ii, ie-jj, jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &pD[jj*bs+jj*sdd], 1, &inv_diag_D[jj]);
		jj+=2;
		}
	// factorize
	if(jj<n-2)
		{
		kernel_dgetrf_l_nn_8x4_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &inv_diag_D[jj]);
		jj+=4;
		}
	else if(jj<n)
		{
		kernel_dgetrf_l_nn_8x2_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &inv_diag_D[jj]);
		}
	if(jj<n-2)
		{
		kernel_dgetrf_r_nn_8x4_vs_lib4(m-ii, n-jj, ii, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &inv_diag_D[jj], &pD[ii*bs+ii*sdd], sdd);
		jj+=4;
		}
	else if(jj<n)
		{
		kernel_dgetrf_r_nn_8x2_vs_lib4(m-ii, n-jj, ii, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &inv_diag_D[jj], &pD[ii*bs+ii*sdd], sdd);
		jj+=2;
		}
	// solve upper 
	for( ; jj<n-2; jj+=4)
		{
		kernel_dtrsm_nn_ll_diag_8x4_vs_lib4(m-ii, n-jj, ii, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &pD[ii*bs+ii*sdd], sdd);
		}
	if(jj<n)
		{
		kernel_dtrsm_nn_ll_diag_8x2_vs_lib4(m-ii, n-jj, ii, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], sdc, &pD[jj*bs+ii*sdd], sdd, &pD[ii*bs+ii*sdd], sdd);
		//jj+=2;
		}
	return;
#endif

	left_4:
	jj = 0;
	// solve lower
	ie = n<ii ? n : ii; // ie is multiple of 4
	for( ; jj<ie-2; jj+=4)
		{
		kernel_dtrsm_nn_ru_4x4_vs_lib4(m-ii, ie-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &pD[jj*bs+jj*sdd], 1, &inv_diag_D[jj]);
		}
	if(jj<ie)
		{
		kernel_dtrsm_nn_ru_4x2_vs_lib4(m-ii, ie-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &pD[jj*bs+jj*sdd], 1, &inv_diag_D[jj]);
		jj+=2;
		}
	// factorize
	if(jj<n-2)
		{
		kernel_dgetrf_nn_4x4_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &inv_diag_D[jj]);
		jj+=4;
		}
	else if(jj<n)
		{
		kernel_dgetrf_nn_4x2_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &inv_diag_D[jj]);
		jj+=2;
		}
	// solve upper 
	for( ; jj<n-2; jj+=4)
		{
		kernel_dtrsm_nn_ll_diag_4x4_vs_lib4(m-ii, n-jj, ii, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &pD[ii*bs+ii*sdd]);
		}
	if(jj<n)
		{
		kernel_dtrsm_nn_ll_diag_4x2_vs_lib4(m-ii, n-jj, ii, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &pD[ii*bs+ii*sdd]);
		//jj+=2;
		}
	return;

#if ! ( defined(TARGET_X64_AVX) || defined(TARGET_X64_AVX) )
	left_2:
	jj = 0;
	// solve lower
	ie = n<ii ? n : ii; // ie is multiple of 4
	for( ; jj<ie-2; jj+=4)
		{
		kernel_dtrsm_nn_ru_2x4_vs_lib4(m-ii, ie-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &pD[jj*bs+jj*sdd], 1, &inv_diag_D[jj]);
		}
	if(jj<ie)
		{
		kernel_dtrsm_nn_ru_2x2_vs_lib4(m-ii, ie-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &pD[jj*bs+jj*sdd], 1, &inv_diag_D[jj]);
		jj+=2;
		}
	// factorize
	if(jj<n-2)
		{
		kernel_dgetrf_nn_2x4_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &inv_diag_D[jj]);
		jj+=4;
		}
	else if(jj<n)
		{
		kernel_dgetrf_nn_2x2_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &inv_diag_D[jj]);
		jj+=2;
		}
	// solve upper 
	for( ; jj<n-2; jj+=4)
		{
		kernel_dtrsm_nn_ll_diag_2x4_vs_lib4(m-ii, n-jj, ii, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &pD[ii*bs+ii*sdd]);
		}
	if(jj<n)
		{
		kernel_dtrsm_nn_ll_diag_2x2_vs_lib4(m-ii, n-jj, ii, &pD[ii*sdd], &pD[jj*bs], sdd, 1, &pC[jj*bs+ii*sdc], &pD[jj*bs+ii*sdd], &pD[ii*bs+ii*sdd]);
		//jj+=2;
		}
	return;
#endif

	}
#endif



#if ! defined(BLASFEO)
void dgetrf_pivot_lib(int m, int n, double *pC, int sdc, double *pD, int sdd, double *inv_diag_D, int *ipiv)
	{

	if(m<=0)
		return;
	
	const int bs = 4;

	int ii, jj, i0, j0, ll, p;

	// needs to perform row-excanges on the yet-to-be-factorized matrix too
	if(pC!=pD)
		dgecp_lib(m, n, 0, pC, sdc, 0, pD, sdd);

	// minimum matrix size
	p = n<m ? n : m; // XXX

	// main loop
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
	// 8 columns at a time
	jj = 0;
	for(; jj<p-7; jj+=8)
		{
		// pivot & factorize & solve lower
		// left block-column
		ii = jj;
		i0 = ii;
		for( ; ii<m-7; ii+=8)
			{
			kernel_dgemm_nn_8x4_lib4(jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], sdd, &pD[jj*bs+ii*sdd], sdd, 0, 0);
			}
		if(m-ii>0)
			{
			if(m-ii>4)
				{
				kernel_dgemm_nn_8x4_vs_lib4(m-ii, 4, jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], sdd, &pD[jj*bs+ii*sdd], sdd, 0, 0);
				}
			else
				{
				kernel_dgemm_nn_4x4_vs_lib4(m-ii, 4, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
				}
			}
		kernel_dgetrf_pivot_4_lib4(m-i0, &pD[jj*bs+i0*sdd], sdd, &inv_diag_D[jj], &ipiv[i0]);
		ipiv[i0+0] += i0;
		if(ipiv[i0+0]!=i0+0)
			{
			drowsw_lib(jj, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs);
			drowsw_lib(n-jj-4, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs+(jj+4)*bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs+(jj+4)*bs);
			}
		ipiv[i0+1] += i0;
		if(ipiv[i0+1]!=i0+1)
			{
			drowsw_lib(jj, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs);
			drowsw_lib(n-jj-4, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs+(jj+4)*bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs+(jj+4)*bs);
			}
		ipiv[i0+2] += i0;
		if(ipiv[i0+2]!=i0+2)
			{
			drowsw_lib(jj, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs);
			drowsw_lib(n-jj-4, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs+(jj+4)*bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs+(jj+4)*bs);
			}
		ipiv[i0+3] += i0;
		if(ipiv[i0+3]!=i0+3)
			{
			drowsw_lib(jj, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs);
			drowsw_lib(n-jj-4, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs+(jj+4)*bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs+(jj+4)*bs);
			}
		// right block-column
		ii = i0;
		kernel_dtrsm_nn_ll_diag_4x4_lib4(ii, &pD[ii*sdd], &pD[(jj+4)*bs], sdd, 1, &pD[(jj+4)*bs+ii*sdd], &pD[(jj+4)*bs+ii*sdd], &pD[ii*bs+ii*sdd]);
		ii += 4;
		i0 = ii;
		for( ; ii<m-7; ii+=8)
			{
			kernel_dgemm_nn_8x4_lib4((jj+4), &pD[ii*sdd], sdd, &pD[(jj+4)*bs], sdd, -1, &pD[(jj+4)*bs+ii*sdd], sdd, &pD[(jj+4)*bs+ii*sdd], sdd, 0, 0);
			}
		if(m-ii>0)
			{
			if(m-ii>4)
				{
				kernel_dgemm_nn_8x4_vs_lib4(m-ii, 4, (jj+4), &pD[ii*sdd], sdd, &pD[(jj+4)*bs], sdd, -1, &pD[(jj+4)*bs+ii*sdd], sdd, &pD[(jj+4)*bs+ii*sdd], sdd, 0, 0);
				}
			else
				{
				kernel_dgemm_nn_4x4_vs_lib4(m-ii, 4, (jj+4), &pD[ii*sdd], &pD[(jj+4)*bs], sdd, -1, &pD[(jj+4)*bs+ii*sdd], &pD[(jj+4)*bs+ii*sdd], 0, 0);
				}
			}
		kernel_dgetrf_pivot_4_lib4(m-i0, &pD[(jj+4)*bs+i0*sdd], sdd, &inv_diag_D[(jj+4)], &ipiv[i0]);
		ipiv[i0+0] += i0;
		if(ipiv[i0+0]!=i0+0)
			{
			drowsw_lib(jj+4, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs);
			drowsw_lib(n-jj-8, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs+(jj+8)*bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs+(jj+8)*bs);
			}
		ipiv[i0+1] += i0;
		if(ipiv[i0+1]!=i0+1)
			{
			drowsw_lib(jj+4, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs);
			drowsw_lib(n-jj-8, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs+(jj+8)*bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs+(jj+8)*bs);
			}
		ipiv[i0+2] += i0;
		if(ipiv[i0+2]!=i0+2)
			{
			drowsw_lib(jj+4, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs);
			drowsw_lib(n-jj-8, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs+(jj+8)*bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs+(jj+8)*bs);
			}
		ipiv[i0+3] += i0;
		if(ipiv[i0+3]!=i0+3)
			{
			drowsw_lib(jj+4, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs);
			drowsw_lib(n-jj-8, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs+(jj+8)*bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs+(jj+8)*bs);
			}

		// solve upper 
		i0 -= 4;
		ll = jj+8;
		for( ; ll<n-3; ll+=4)
			{
			kernel_dtrsm_nn_ll_diag_8x4_lib4(i0, &pD[i0*sdd], sdd, &pD[ll*bs], sdd, 1, &pD[ll*bs+i0*sdd], sdd, &pD[ll*bs+i0*sdd], sdd, &pD[i0*bs+i0*sdd], sdd);
			}
		if(ll<n)
			{
			if(n-ll==3)
				{
				kernel_dtrsm_nn_ll_diag_8x4_vs_lib4(8, 3, i0, &pD[i0*sdd], sdd, &pD[ll*bs], sdd, 1, &pD[ll*bs+i0*sdd], sdd, &pD[ll*bs+i0*sdd], sdd, &pD[i0*bs+i0*sdd], sdd);
				}
			else
				{
				kernel_dtrsm_nn_ll_diag_8x2_vs_lib4(8, n-ll, i0, &pD[i0*sdd], sdd, &pD[ll*bs], sdd, 1, &pD[ll*bs+i0*sdd], sdd, &pD[ll*bs+i0*sdd], sdd, &pD[i0*bs+i0*sdd], sdd);
				}
			}
		}
	if(m>=n)
		{
		if(n-jj>0)
			{
			if(n-jj<=4)
				{
				if(n-jj<=2) // (m>=1 && n==1) || (m>=2 && n==2)
					goto left_n_2;
				else // m>=3 && n==3
					goto left_n_4;
				}
			else // (m>=5 && n==5) || (m>=6 && n==6) || (m>=7 && n==7)
				goto left_n_8;
			}
		}
	else // n>m
		{
		if(m-jj>0)
			{
			if(m-jj<=4) // (m==1 && n>=2) || (m==2 && n>=3) || (m==3 && n>=4) || (m==4 && n>=5)
				goto left_m_4;
			else // (m==5 && n>=6) || (m==6 && n>=7) || (m==7 && n>=8)
				{
				goto left_m_8;
				}
			}
		}
#else
	// 4 columns at a time
	jj = 0;
	for(; jj<p-3; jj+=4) // XXX
		{
		// pivot & factorize & solve lower
		ii = jj;
		i0 = ii;
		for( ; ii<m-3; ii+=4)
			{
			kernel_dgemm_nn_4x4_lib4(jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
			}
		if(m-ii>0)
			{
			if(m-ii==3)
				{
				kernel_dgemm_nn_4x4_vs_lib4(3, 4, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
//				ii += 4;
				}
			else
				{
				kernel_dgemm_nn_2x4_vs_lib4(m-ii, 4, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
//				ii += 2;
				}
			}
		kernel_dgetrf_pivot_4_lib4(m-i0, &pD[jj*bs+i0*sdd], sdd, &inv_diag_D[jj], &ipiv[i0]);
		ipiv[i0+0] += i0;
		if(ipiv[i0+0]!=i0+0)
			{
			drowsw_lib(jj, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs);
			drowsw_lib(n-jj-4, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs+(jj+4)*bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs+(jj+4)*bs);
			}
		ipiv[i0+1] += i0;
		if(ipiv[i0+1]!=i0+1)
			{
			drowsw_lib(jj, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs);
			drowsw_lib(n-jj-4, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs+(jj+4)*bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs+(jj+4)*bs);
			}
		ipiv[i0+2] += i0;
		if(ipiv[i0+2]!=i0+2)
			{
			drowsw_lib(jj, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs);
			drowsw_lib(n-jj-4, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs+(jj+4)*bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs+(jj+4)*bs);
			}
		ipiv[i0+3] += i0;
		if(ipiv[i0+3]!=i0+3)
			{
			drowsw_lib(jj, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs);
			drowsw_lib(n-jj-4, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs+(jj+4)*bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs+(jj+4)*bs);
			}

		// solve upper
		ll = jj+4;
		for( ; ll<n-3; ll+=4)
			{
			kernel_dtrsm_nn_ll_diag_4x4_lib4(i0, &pD[i0*sdd], &pD[ll*bs], sdd, 1, &pD[ll*bs+i0*sdd], &pD[ll*bs+i0*sdd], &pD[i0*bs+i0*sdd]);
			}
		if(n-ll>0)
			{
			if(n-ll==3)
				{
				kernel_dtrsm_nn_ll_diag_4x4_vs_lib4(4, 3, i0, &pD[i0*sdd], &pD[ll*bs], sdd, 1, &pD[ll*bs+i0*sdd], &pD[ll*bs+i0*sdd], &pD[i0*bs+i0*sdd]);
//				ll += 4;
				}
			else
				{
				kernel_dtrsm_nn_ll_diag_4x2_vs_lib4(4, n-ll, i0, &pD[i0*sdd], &pD[ll*bs], sdd, 1, &pD[ll*bs+i0*sdd], &pD[ll*bs+i0*sdd], &pD[i0*bs+i0*sdd]);
//				ll += 2;
				}
			}
		}
	if(m>=n)
		{
		if(n-jj>0)
			{
			if(n-jj<=2) // (m>=1 && n==1) || (m>=2 && n==2)
				goto left_n_2;
			else // m>=3 && n==3
				goto left_n_4;
			}
		}
	else
		{
		if(m-jj>0)
			{
			if(m-jj<=2) // (m==1 && n>=2) || (m==2 && n>=3)
				goto left_m_2;
			else // m==3 and n>=4
				goto left_m_4;
			}
		}
#endif

	// common return if jj==n
	return;


	// clean up
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
	left_n_8:
	// 5-8 columns at a time
	// pivot & factorize & solve lower
	// left block-column
	ii = jj;
	i0 = ii;
	for( ; ii<m-4; ii+=8)
		{
		kernel_dgemm_nn_8x4_vs_lib4(m-ii, 4, jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], sdd, &pD[jj*bs+ii*sdd], sdd, 0, 0);
		}
	if(m-ii>0)
		{
		kernel_dgemm_nn_4x4_vs_lib4(m-ii, 4, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
//		ii+=4;
		}
	kernel_dgetrf_pivot_4_lib4(m-i0, &pD[jj*bs+i0*sdd], sdd, &inv_diag_D[jj], &ipiv[i0]);
	ipiv[i0+0] += i0;
	if(ipiv[i0+0]!=i0+0)
		{
		drowsw_lib(jj, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs);
		drowsw_lib(n-jj-4, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs+(jj+4)*bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs+(jj+4)*bs);
		}
	ipiv[i0+1] += i0;
	if(ipiv[i0+1]!=i0+1)
		{
		drowsw_lib(jj, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs);
		drowsw_lib(n-jj-4, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs+(jj+4)*bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs+(jj+4)*bs);
		}
	ipiv[i0+2] += i0;
	if(ipiv[i0+2]!=i0+2)
		{
		drowsw_lib(jj, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs);
		drowsw_lib(n-jj-4, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs+(jj+4)*bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs+(jj+4)*bs);
		}
	ipiv[i0+3] += i0;
	if(ipiv[i0+3]!=i0+3)
		{
		drowsw_lib(jj, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs);
		drowsw_lib(n-jj-4, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs+(jj+4)*bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs+(jj+4)*bs);
		}
	// right block-column
	ii = i0;
	if(n-jj-4>2)
		{
		kernel_dtrsm_nn_ll_diag_4x4_vs_lib4(4, n-jj-4, ii, &pD[ii*sdd], &pD[(jj+4)*bs], sdd, 1, &pD[(jj+4)*bs+ii*sdd], &pD[(jj+4)*bs+ii*sdd], &pD[ii*bs+ii*sdd]);
		ii += 4;
		i0 = ii;
		for( ; ii<m-4; ii+=8)
			{
			kernel_dgemm_nn_8x4_vs_lib4(m-ii, n-jj-4, (jj+4), &pD[ii*sdd], sdd, &pD[(jj+4)*bs], sdd, -1, &pD[(jj+4)*bs+ii*sdd], sdd, &pD[(jj+4)*bs+ii*sdd], sdd, 0, 0);
			}
		if(m-ii>0)
			{
			kernel_dgemm_nn_4x4_vs_lib4(m-ii, n-jj-4, (jj+4), &pD[ii*sdd], &pD[(jj+4)*bs], sdd, -1, &pD[(jj+4)*bs+ii*sdd], &pD[(jj+4)*bs+ii*sdd], 0, 0);
			}
		}
	else
		{
		kernel_dtrsm_nn_ll_diag_4x2_vs_lib4(4, n-jj-4, ii, &pD[ii*sdd], &pD[(jj+4)*bs], sdd, 1, &pD[(jj+4)*bs+ii*sdd], &pD[(jj+4)*bs+ii*sdd], &pD[ii*bs+ii*sdd]);
		ii += 4;
		i0 = ii;
		for( ; ii<m-4; ii+=8)
			{
			kernel_dgemm_nn_8x2_vs_lib4(m-ii, n-jj-4, (jj+4), &pD[ii*sdd], sdd, &pD[(jj+4)*bs], sdd, -1, &pD[(jj+4)*bs+ii*sdd], sdd, &pD[(jj+4)*bs+ii*sdd], sdd, 0, 0);
			}
		if(m-ii>0)
			{
			kernel_dgemm_nn_4x2_vs_lib4(m-ii, n-jj-4, (jj+4), &pD[ii*sdd], &pD[(jj+4)*bs], sdd, -1, &pD[(jj+4)*bs+ii*sdd], &pD[(jj+4)*bs+ii*sdd], 0, 0);
			}
		}
	kernel_dgetrf_pivot_4_vs_lib4(m-i0, n-jj-4, &pD[(jj+4)*bs+i0*sdd], sdd, &inv_diag_D[(jj+4)], &ipiv[i0]);
	ipiv[i0+0] += i0;
	if(ipiv[i0+0]!=i0+0)
		{
		drowsw_lib(jj+4, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs);
		drowsw_lib(n-jj-8, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs+(jj+8)*bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs+(jj+8)*bs);
		}
	if(n-jj-4>1)
		{
		ipiv[i0+1] += i0;
		if(ipiv[i0+1]!=i0+1)
			{
			drowsw_lib(jj+4, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs);
			drowsw_lib(n-jj-8, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs+(jj+8)*bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs+(jj+8)*bs);
			}
		if(n-jj-4>2)
			{
			ipiv[i0+2] += i0;
			if(ipiv[i0+2]!=i0+2)
				{
				drowsw_lib(jj+4, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs);
				drowsw_lib(n-jj-8, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs+(jj+8)*bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs+(jj+8)*bs);
				}
			if(n-jj-4>3)
				{
				ipiv[i0+3] += i0;
				if(ipiv[i0+3]!=i0+3)
					{
					drowsw_lib(jj+4, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs);
					drowsw_lib(n-jj-8, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs+(jj+8)*bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs+(jj+8)*bs);
					}
				}
			}
		}

	// solve upper 
	// there is no upper
	return;
#endif


#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
	left_m_8:
	// 5-8 rows at a time
	// pivot & factorize & solve lower
	// left block-column
	ii = jj;
	i0 = ii;
	kernel_dgemm_nn_8x4_vs_lib4(m-ii, 4, jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], sdd, &pD[jj*bs+ii*sdd], sdd, 0, 0);
	kernel_dgetrf_pivot_4_lib4(m-i0, &pD[jj*bs+i0*sdd], sdd, &inv_diag_D[jj], &ipiv[i0]);
	ipiv[i0+0] += i0;
	if(ipiv[i0+0]!=i0+0)
		{
		drowsw_lib(jj, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs);
		drowsw_lib(n-jj-4, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs+(jj+4)*bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs+(jj+4)*bs);
		}
	ipiv[i0+1] += i0;
	if(ipiv[i0+1]!=i0+1)
		{
		drowsw_lib(jj, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs);
		drowsw_lib(n-jj-4, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs+(jj+4)*bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs+(jj+4)*bs);
		}
	ipiv[i0+2] += i0;
	if(ipiv[i0+2]!=i0+2)
		{
		drowsw_lib(jj, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs);
		drowsw_lib(n-jj-4, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs+(jj+4)*bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs+(jj+4)*bs);
		}
	ipiv[i0+3] += i0;
	if(ipiv[i0+3]!=i0+3)
		{
		drowsw_lib(jj, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs);
		drowsw_lib(n-jj-4, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs+(jj+4)*bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs+(jj+4)*bs);
		}
	// right block-column
	ii = i0;
	if(n-jj-4>2)
		kernel_dtrsm_nn_ll_diag_4x4_vs_lib4(4, n-jj-4, ii, &pD[ii*sdd], &pD[(jj+4)*bs], sdd, 1, &pD[(jj+4)*bs+ii*sdd], &pD[(jj+4)*bs+ii*sdd], &pD[ii*bs+ii*sdd]);
	else
		kernel_dtrsm_nn_ll_diag_4x2_vs_lib4(4, n-jj-4, ii, &pD[ii*sdd], &pD[(jj+4)*bs], sdd, 1, &pD[(jj+4)*bs+ii*sdd], &pD[(jj+4)*bs+ii*sdd], &pD[ii*bs+ii*sdd]);
	ii += 4;
	i0 = ii;
	if(n-jj-4>2)
		kernel_dgemm_nn_4x4_vs_lib4(m-ii, n-jj-4, (jj+4), &pD[ii*sdd], &pD[(jj+4)*bs], sdd, -1, &pD[(jj+4)*bs+ii*sdd], &pD[(jj+4)*bs+ii*sdd], 0, 0);
	else
		kernel_dgemm_nn_4x2_vs_lib4(m-ii, n-jj-4, (jj+4), &pD[ii*sdd], &pD[(jj+4)*bs], sdd, -1, &pD[(jj+4)*bs+ii*sdd], &pD[(jj+4)*bs+ii*sdd], 0, 0);
	kernel_dgetrf_pivot_4_vs_lib4(m-i0, n-jj-4, &pD[(jj+4)*bs+i0*sdd], sdd, &inv_diag_D[(jj+4)], &ipiv[i0]);
	ipiv[i0+0] += i0;
	if(ipiv[i0+0]!=i0+0)
		{
		drowsw_lib(jj+4, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs);
		drowsw_lib(n-jj-8, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs+(jj+8)*bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs+(jj+8)*bs);
		}
	if(m-jj-4>1)
		{
		ipiv[i0+1] += i0;
		if(ipiv[i0+1]!=i0+1)
			{
			drowsw_lib(jj+4, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs);
			drowsw_lib(n-jj-8, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs+(jj+8)*bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs+(jj+8)*bs);
			}
		if(m-jj-4>2)
			{
			ipiv[i0+2] += i0;
			if(ipiv[i0+2]!=i0+2)
				{
				drowsw_lib(jj+4, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs);
				drowsw_lib(n-jj-8, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs+(jj+8)*bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs+(jj+8)*bs);
				}
			if(m-jj-4>3)
				{
				ipiv[i0+3] += i0;
				if(ipiv[i0+3]!=i0+3)
					{
					drowsw_lib(jj+4, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs);
					drowsw_lib(n-jj-8, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs+(jj+8)*bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs+(jj+8)*bs);
					}
				}
			}
		}

	// solve upper 
	i0 -= 4;
	ll = jj+8;
	for( ; ll<n-2; ll+=4)
		{
		kernel_dtrsm_nn_ll_diag_8x4_vs_lib4(m-i0, n-ll, i0, &pD[i0*sdd], sdd, &pD[ll*bs], sdd, 1, &pD[ll*bs+i0*sdd], sdd, &pD[ll*bs+i0*sdd], sdd, &pD[i0*bs+i0*sdd], sdd);
		}
	if(ll<n)
		{
		kernel_dtrsm_nn_ll_diag_8x2_vs_lib4(m-i0, n-ll, i0, &pD[i0*sdd], sdd, &pD[ll*bs], sdd, 1, &pD[ll*bs+i0*sdd], sdd, &pD[ll*bs+i0*sdd], sdd, &pD[i0*bs+i0*sdd], sdd);
//			ll+=2;
		}
	return;
#endif


	left_n_4:
	// 3-4 columns at a time
	// pivot & factorize & solve lower
	ii = jj;
	i0 = ii;
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
	for( ; ii<m-4; ii+=8)
		{
		kernel_dgemm_nn_8x4_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], sdd, &pD[jj*bs+ii*sdd], sdd, 0, 0);
		}
	if(m-ii>0)
		{
		kernel_dgemm_nn_4x4_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
//		ii+=4;
		}
#else
	for( ; ii<m-2; ii+=4)
		{
		kernel_dgemm_nn_4x4_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
		}
	if(m-ii>0)
		{
		kernel_dgemm_nn_2x4_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
//		ii+=2;
		}
#endif
	kernel_dgetrf_pivot_4_vs_lib4(m-i0, n-jj, &pD[jj*bs+i0*sdd], sdd, &inv_diag_D[jj], &ipiv[i0]);
	ipiv[i0+0] += i0;
	if(ipiv[i0+0]!=i0+0)
		{
		drowsw_lib(jj, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs);
		drowsw_lib(n-jj-4, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs+(jj+4)*bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs+(jj+4)*bs);
		}
	ipiv[i0+1] += i0;
	if(ipiv[i0+1]!=i0+1)
		{
		drowsw_lib(jj, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs);
		drowsw_lib(n-jj-4, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs+(jj+4)*bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs+(jj+4)*bs);
		}
	ipiv[i0+2] += i0;
	if(ipiv[i0+2]!=i0+2)
		{
		drowsw_lib(jj, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs);
		drowsw_lib(n-jj-4, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs+(jj+4)*bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs+(jj+4)*bs);
		}
	if(n-jj==4)
		{
		ipiv[i0+3] += i0;
		if(ipiv[i0+3]!=i0+3)
			{
			drowsw_lib(jj, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs);
			drowsw_lib(n-jj-4, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs+(jj+4)*bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs+(jj+4)*bs);
			}
		}

	// solve upper
	if(0) // there is no upper
		{
		ll = jj+4;
		for( ; ll<n-2; ll+=4)
			{
			kernel_dtrsm_nn_ll_diag_4x4_vs_lib4(m-i0, n-ll, i0, &pD[i0*sdd], &pD[ll*bs], sdd, 1, &pD[ll*bs+i0*sdd], &pD[ll*bs+i0*sdd], &pD[i0*bs+i0*sdd]);
			}
		if(n-ll>0)
			{
			kernel_dtrsm_nn_ll_diag_4x2_vs_lib4(m-i0, n-ll, i0, &pD[i0*sdd], &pD[ll*bs], sdd, 1, &pD[ll*bs+i0*sdd], &pD[ll*bs+i0*sdd], &pD[i0*bs+i0*sdd]);
//			ll+=2;
			}
		}
	return;


	left_m_4:
	// 3-4 rows at a time (c99) or 1-4 rows at a time (avx)
	// pivot & factorize & solve lower
	ii = jj;
	i0 = ii;
	if(n-jj>2)
		{
		kernel_dgemm_nn_4x4_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
		}
	else
		{
		kernel_dgemm_nn_4x2_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
		}
	kernel_dgetrf_pivot_4_vs_lib4(m-i0, n-jj, &pD[jj*bs+i0*sdd], sdd, &inv_diag_D[jj], &ipiv[i0]);
	ipiv[i0+0] += i0;
	if(ipiv[i0+0]!=i0+0)
		{
		drowsw_lib(jj, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs);
		drowsw_lib(n-jj-4, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs+(jj+4)*bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs+(jj+4)*bs);
		}
	if(m-i0>1)
		{
		ipiv[i0+1] += i0;
		if(ipiv[i0+1]!=i0+1)
			{
			drowsw_lib(jj, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs);
			drowsw_lib(n-jj-4, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs+(jj+4)*bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs+(jj+4)*bs);
			}
		if(m-i0>2)
			{
			ipiv[i0+2] += i0;
			if(ipiv[i0+2]!=i0+2)
				{
				drowsw_lib(jj, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs);
				drowsw_lib(n-jj-4, pD+(i0+2)/bs*bs*sdd+(i0+2)%bs+(jj+4)*bs, pD+(ipiv[i0+2])/bs*bs*sdd+(ipiv[i0+2])%bs+(jj+4)*bs);
				}
			if(m-i0>3)
				{
				ipiv[i0+3] += i0;
				if(ipiv[i0+3]!=i0+3)
					{
					drowsw_lib(jj, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs);
					drowsw_lib(n-jj-4, pD+(i0+3)/bs*bs*sdd+(i0+3)%bs+(jj+4)*bs, pD+(ipiv[i0+3])/bs*bs*sdd+(ipiv[i0+3])%bs+(jj+4)*bs);
					}
				}
			}
		}

	// solve upper
	ll = jj+4;
	for( ; ll<n-2; ll+=4)
		{
		kernel_dtrsm_nn_ll_diag_4x4_vs_lib4(m-i0, n-ll, i0, &pD[i0*sdd], &pD[ll*bs], sdd, 1, &pD[ll*bs+i0*sdd], &pD[ll*bs+i0*sdd], &pD[i0*bs+i0*sdd]);
		}
	if(n-ll>0)
		{
		kernel_dtrsm_nn_ll_diag_4x2_vs_lib4(m-i0, n-ll, i0, &pD[i0*sdd], &pD[ll*bs], sdd, 1, &pD[ll*bs+i0*sdd], &pD[ll*bs+i0*sdd], &pD[i0*bs+i0*sdd]);
//		ll+=2;
		}
	return;


	left_n_2:
	// 1-2 columns at a time
	// pivot & factorize & solve lower
	ii = jj;
	i0 = ii;
#if defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX)
	for( ; ii<m-4; ii+=8)
		{
		kernel_dgemm_nn_8x2_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], sdd, &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], sdd, &pD[jj*bs+ii*sdd], sdd, 0, 0);
		}
	if(m-ii>0)
		{
		kernel_dgemm_nn_4x2_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
//		ii+=4;
		}
#else
	for( ; ii<m-2; ii+=4)
		{
		kernel_dgemm_nn_4x2_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
		}
	if(m-ii>0)
		{
		kernel_dgemm_nn_2x2_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
//		ii+=2;
		}
#endif
	kernel_dgetrf_pivot_4_vs_lib4(m-i0, n-jj, &pD[jj*bs+i0*sdd], sdd, &inv_diag_D[jj], &ipiv[i0]);
	ipiv[i0+0] += i0;
	if(ipiv[i0+0]!=i0+0)
		{
		drowsw_lib(jj, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs);
		drowsw_lib(n-jj-4, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs+(jj+4)*bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs+(jj+4)*bs);
		}
	if(n-jj==4)
		{
		ipiv[i0+1] += i0;
		if(ipiv[i0+1]!=i0+1)
			{
			drowsw_lib(jj, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs);
			drowsw_lib(n-jj-4, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs+(jj+4)*bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs+(jj+4)*bs);
			}
		}

	// solve upper
	// there is no upper
	return;


#if ! ( defined(TARGET_X64_AVX2) || defined(TARGET_X64_AVX) )
	left_m_2:
	// 1-2 rows at a time
	// pivot & factorize & solve lower
	ii = jj;
	i0 = ii;
	if(n-jj>2)
		{
//		for( ; ii<m-2; ii+=4)
//			{
//			kernel_dgemm_nn_4x4_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
//			}
//		if(m-ii>0)
//			{
		kernel_dgemm_nn_2x4_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
	//		ii+=2;
//			}
		}
	else
		{
//		for( ; ii<m-2; ii+=4)
//			{
//			kernel_dgemm_nn_4x2_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
//			}
//		if(m-ii>0)
//			{
		kernel_dgemm_nn_2x2_vs_lib4(m-ii, n-jj, jj, &pD[ii*sdd], &pD[jj*bs], sdd, -1, &pD[jj*bs+ii*sdd], &pD[jj*bs+ii*sdd], 0, 0);
	//		ii+=2;
//			}
		}
	kernel_dgetrf_pivot_4_vs_lib4(m-i0, n-jj, &pD[jj*bs+i0*sdd], sdd, &inv_diag_D[jj], &ipiv[i0]);
	ipiv[i0+0] += i0;
	if(ipiv[i0+0]!=i0+0)
		{
		drowsw_lib(jj, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs);
		drowsw_lib(n-jj-4, pD+(i0+0)/bs*bs*sdd+(i0+0)%bs+(jj+4)*bs, pD+(ipiv[i0+0])/bs*bs*sdd+(ipiv[i0+0])%bs+(jj+4)*bs);
		}
	if(m-i0>1)
		{
		ipiv[i0+1] += i0;
		if(ipiv[i0+1]!=i0+1)
			{
			drowsw_lib(jj, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs);
			drowsw_lib(n-jj-4, pD+(i0+1)/bs*bs*sdd+(i0+1)%bs+(jj+4)*bs, pD+(ipiv[i0+1])/bs*bs*sdd+(ipiv[i0+1])%bs+(jj+4)*bs);
			}
		}

	// solve upper
	ll = jj+4;
	for( ; ll<n-2; ll+=4)
		{
		kernel_dtrsm_nn_ll_diag_2x4_vs_lib4(m-i0, n-ll, i0, &pD[i0*sdd], &pD[ll*bs], sdd, 1, &pD[ll*bs+i0*sdd], &pD[ll*bs+i0*sdd], &pD[i0*bs+i0*sdd]);
		}
	if(n-ll>0)
		{
		kernel_dtrsm_nn_ll_diag_2x2_vs_lib4(m-i0, n-ll, i0, &pD[i0*sdd], &pD[ll*bs], sdd, 1, &pD[ll*bs+i0*sdd], &pD[ll*bs+i0*sdd], &pD[i0*bs+i0*sdd]);
//		ll += 2;
		}
	return;
#endif

	}
#endif



