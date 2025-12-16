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

#include <stdlib.h>
#include <stdio.h>

int posix_memalign(void **memptr, size_t alignment, size_t size);



/* creates a zero matrix aligned */
void s_zeros(float **pA, int row, int col)
	{
	void *temp = malloc((row*col)*sizeof(float));
	*pA = temp;
	float *A = *pA;
	int i;
	for(i=0; i<row*col; i++) A[i] = 0.0;
	}



/* creates a zero matrix aligned to a cache line */
void s_zeros_align(float **pA, int row, int col)
	{
	void *temp;
	int err = posix_memalign(&temp, 64, (row*col)*sizeof(float));
	if(err!=0)
		{
		printf("Memory allocation error");
		exit(1);
		}
	*pA = temp;
	float *A = *pA;
	int i;
	for(i=0; i<row*col; i++) A[i] = 0.0;
	}



/* creates a zero matrix aligned */
void s_eye(float **pA, int row)
	{
	void *temp = malloc((row*row)*sizeof(float));
	*pA = temp;
	float *A = *pA;
	int i;
	for(i=0; i<row*row; i++) A[i] = 0.0;
	for(i=0; i<row; i++) A[i*(row+1)] = 1.0;
	}



/* copies a matrix */
void s_copy_mat(int row, int col, float *A, int lda, float *B, int ldb)
	{
	
	int i, j;
	
	for(j=0; j<col; j++)
		{
		for(i=0; i<row; i++)
			{
			B[i+j*ldb] = A[i+j*lda];
			}
		}
	
	}



/* copies a packed matrix */
void s_copy_pmat(int row, int col, int bs_dummy, float *A, int sda, float *B, int sdb)
	{
	
	const int bs = 8;
	
	int i, ii, j, row2;
	
	ii = 0;
	for(; ii<row-7; ii+=8)
		{
		for(j=0; j<col; j++)
			{
			B[0+j*bs+ii*sdb] = A[0+j*bs+ii*sda];
			B[1+j*bs+ii*sdb] = A[1+j*bs+ii*sda];
			B[2+j*bs+ii*sdb] = A[2+j*bs+ii*sda];
			B[3+j*bs+ii*sdb] = A[3+j*bs+ii*sda];
			B[4+j*bs+ii*sdb] = A[4+j*bs+ii*sda];
			B[5+j*bs+ii*sdb] = A[5+j*bs+ii*sda];
			B[6+j*bs+ii*sdb] = A[6+j*bs+ii*sda];
			B[7+j*bs+ii*sdb] = A[7+j*bs+ii*sda];
			}
		}
	if(ii<row)
		{
		row2 = row-ii;
		for(j=0; j<col; j++)
			{
			for(i=0; i<row2; i++)
				{
				B[i+j*bs+ii*sdb] = A[i+j*bs+ii*sda];
				}
			}
		}
	
	}



/* copies a lower triangular packed matrix */
/*void s_copy_pmat_lo(int row, int bs_dummy, float *A, int sda, float *B, int sdb)*/
/*	{*/
/*	*/
/*	const int bs = 4;*/

/*	int i, ii, j, row2, row0;*/
/*	*/
/*	ii = 0;*/
/*	for(; ii<row-3; ii+=bs)*/
/*		{*/
/*		j = 0;*/
/*		for(; j<ii; j++)*/
/*			{*/
/*			B[0+j*bs+ii*sdb] = A[0+j*bs+ii*sda];*/
/*			B[1+j*bs+ii*sdb] = A[1+j*bs+ii*sda];*/
/*			B[2+j*bs+ii*sdb] = A[2+j*bs+ii*sda];*/
/*			B[3+j*bs+ii*sdb] = A[3+j*bs+ii*sda];*/
/*			}*/
/*		for(; j<ii+bs; j++)*/
/*			{*/
/*			row0 = j-ii;*/
/*			if(row0<0) row0=0;*/
/*			for(i=row0; i<bs; i++)*/
/*				{*/
/*				B[i+j*bs+ii*sdb] = A[i+j*bs+ii*sda];*/
/*				}*/
/*			}*/
/*		}*/
/*	for(; ii<row; ii+=bs)*/
/*		{*/
/*		row2 = row-ii;*/
/*		if(bs<row2) row2 = bs;*/
/*		for(j=0; j<ii+row2; j++)*/
/*			{*/
/*			row0 = j-ii;*/
/*			if(row0<0) row0=0;*/
/*			for(i=row0; i<row2; i++)*/
/*				{*/
/*				B[i+j*bs+ii*sdb] = A[i+j*bs+ii*sda];*/
/*				}*/
/*			}*/
/*		}*/
/*	*/
/*	}*/



/* transposes a lower triangular packed matrix */
/*void s_transpose_pmat_lo(int row, int offset, float *A, int sda, float *B, int sdb)*/
/*	{*/
/*	*/
/*	const int bs = 4;*/

/*	int i, j, jj;*/
/*	*/
/*	int row0, row1, row2, row3;*/
/*	row0 = (bs-offset%bs)%bs; // row2 < bs !!!*/
/*	*/
/*	float *pA, *pB;*/

/*	jj = 0;*/
/*	for(; jj<row-3; jj+=4)*/
/*		{*/
/*		row1 = row - jj;*/
/*		pA = A + jj*bs + jj*sda;*/
/*		pB = B + jj*bs + jj*sdb;*/
/*		row2 = row0; // row2 < bs !!!*/
/*		if(row1<row2)*/
/*			row2 = row1;*/
/*		i = 0;*/
/*		if(row2>0)*/
/*			{*/
/*			for(; i<row2; i++)*/
/*				{*/
/*				for(j=0; j<=i; j++)*/
/*					{*/
/*					pB[j] = pA[j*bs];*/
/*					}*/
/*				pA += 1;*/
/*				pB += bs;*/
/*				}*/
/*			pA += (sda-1)*bs;*/
/*			}*/
/*		row3 = row2 + 4;*/
/*		if(row1<row3)*/
/*			row3 = row1;*/
/*		row2 = 4;*/
/*		if(row1<row2)*/
/*			row2 = row1;*/
/*		for(; i<row2; i++)*/
/*			{*/
/*			for(j=0; j<=i; j++)*/
/*				{*/
/*				pB[j] = pA[j*bs];*/
/*				}*/
/*			pA += 1;*/
/*			pB += bs;*/
/*			}*/
/*		for(; i<row3; i++)*/
/*			{*/
/*			pB[0] = pA[0*bs];*/
/*			pB[1] = pA[1*bs];*/
/*			pB[2] = pA[2*bs];*/
/*			pB[3] = pA[3*bs];*/
/*			pA += 1;*/
/*			pB += bs;*/
/*			}*/
/*		pA += (sda-1)*bs;*/
/*		for(; i<row1-3; i+=4)*/
/*			{*/
/*			// buildin_prefetch*/
/*			// unroll 0*/
/*			pB[0+0*bs] = pA[0+0*bs];*/
/*			pB[1+0*bs] = pA[0+1*bs];*/
/*			pB[2+0*bs] = pA[0+2*bs];*/
/*			pB[3+0*bs] = pA[0+3*bs];*/
/*			// unroll 1*/
/*			pB[0+1*bs] = pA[1+0*bs];*/
/*			pB[1+1*bs] = pA[1+1*bs];*/
/*			pB[2+1*bs] = pA[1+2*bs];*/
/*			pB[3+1*bs] = pA[1+3*bs];*/
/*			// unroll 2*/
/*			pB[0+2*bs] = pA[2+0*bs];*/
/*			pB[1+2*bs] = pA[2+1*bs];*/
/*			pB[2+2*bs] = pA[2+2*bs];*/
/*			pB[3+2*bs] = pA[2+3*bs];*/
/*			// unroll 3*/
/*			pB[0+3*bs] = pA[3+0*bs];*/
/*			pB[1+3*bs] = pA[3+1*bs];*/
/*			pB[2+3*bs] = pA[3+2*bs];*/
/*			pB[3+3*bs] = pA[3+3*bs];*/
/*			pA += sda*bs;*/
/*			pB += 4*bs;*/
/*			}*/
/*		for(; i<row1; i++)*/
/*			{*/
/*			pB[0] = pA[0*bs];*/
/*			pB[1] = pA[1*bs];*/
/*			pB[2] = pA[2*bs];*/
/*			pB[3] = pA[3*bs];*/
/*			pA += 1;*/
/*			pB += bs;*/
/*			}*/
/*		}*/
/*	if(jj<row)*/
/*		{*/
/*		row1 = row - jj;*/
/*		pA = A + jj*bs + jj*sda;*/
/*		pB = B + jj*bs + jj*sdb;*/
/*		row2 = row0; // row2 < bs !!!*/
/*		if(row1<row2)*/
/*			row2 = row1;*/
/*		i = 0;*/
/*		if(row2>0)*/
/*			{*/
/*			for(; i<row2; i++)*/
/*				{*/
/*				for(j=0; j<=i; j++)*/
/*					{*/
/*					pB[j] = pA[j*bs];*/
/*					}*/
/*				pA += 1;*/
/*				pB += bs;*/
/*				}*/
/*			pA += (sda-1)*bs;*/
/*			}*/
/*		row2 = 4;*/
/*		if(row1<row2)*/
/*			row2 = row1;*/
/*		for(; i<row2; i++)*/
/*			{*/
/*			for(j=0; j<=i; j++)*/
/*				{*/
/*				pB[j] = pA[j*bs];*/
/*				}*/
/*			pA += 1;*/
/*			pB += bs;*/
/*			}*/
/*		}*/
/*	*/
/*	}*/



/* copies a packed matrix */
void s_cvt_d2s_pmat(int row, int col, int dbs, double *A, int sda, int sbs, float *B, int sdb)
	{
	
	int i, ii, j, row2, ll;
	
	int bs_ratio = sbs/dbs; // the block size in single is supposed to be a multiple of the block size in double
	
	for(ii=0; ii<row; ii+=sbs)
		{
		
		for(ll=0; ll<bs_ratio; ll++)
			{

			row2 = row-ii;
			if(dbs<row2) row2 = dbs;

			for(j=0; j<col; j++)
				{
				for(i=0; i<row2; i++)
					{
					B[i+ll*dbs+j*sbs+ii*sdb] = (float) A[i+j*dbs+(ii+ll*dbs)*sda];
					}
				}
			
			}


		}
	
	}



/* copies a lower triangular packed matrix */
/*void s_cvt_d2s_pmat_lo(int row, int bs, double *A, int sda, float *B, int sdb)*/
/*	{*/
/*	*/
/*//	TODO*/
/*	*/
/*	}*/



/* copies a packed matrix into an aligned packed matrix ; A has to be aligned at the beginning of the current block : the offset takes care of the row to be copied */
/*void s_align_pmat(int row, int col, int offset, int bs, float *A, int sda, float *B, int sdb)*/
/*	{*/
/*	*/
/*	int i, j;*/
/*	*/
/*	float *ptrA, *ptrB;*/
/*	*/
/*	for(i=0; i<row; i++)*/
/*		{*/
/*		ptrA = A + ((offset+i)/bs)*bs*sda + ((offset+i)%bs);*/
/*		ptrB = B + (i/bs)*bs*sdb + (i%bs);*/
/*		for(j=0; j<col; j++)*/
/*			{*/
/*			ptrB[j*bs] = ptrA[j*bs];*/
/*			}*/
/*		}*/
/*	*/
/*	}*/



/* converts a double matrix into a single packed matrix */
void cvt_d2s_mat2pmat(int row, int col, int offset, int bs_dummy, double *A, int lda, float *pA, int sda)
	{
	
	const int bs = 8;

	int i, ii, j, row0, row1, row2;
	
	row0 = (bs-offset%bs)%bs;
	if(row0>row)
		row0 = row;
	row1 = row - row0;
	
	ii = 0;
	if(row0>0)
		{
		for(j=0; j<col; j++)
			{
			for(i=0; i<row0; i++)
				{
				pA[i+j*bs+ii*sda] = (float) A[i+ii+j*lda];
				}
			}
	
		A  += row0;
		pA += row0 + bs*(sda-1);
		}
	
	ii = 0;
	for(; ii<row1-7; ii+=bs)
		{
/*		for(j=0; j<col; j++)*/
/*			{*/
/*			pA[0+j*bs+ii*sda] = (float) A[0+ii+j*lda];*/
/*			pA[1+j*bs+ii*sda] = (float) A[1+ii+j*lda];*/
/*			pA[2+j*bs+ii*sda] = (float) A[2+ii+j*lda];*/
/*			pA[3+j*bs+ii*sda] = (float) A[3+ii+j*lda];*/
/*			pA[4+j*bs+ii*sda] = (float) A[4+ii+j*lda];*/
/*			pA[5+j*bs+ii*sda] = (float) A[5+ii+j*lda];*/
/*			pA[6+j*bs+ii*sda] = (float) A[6+ii+j*lda];*/
/*			pA[7+j*bs+ii*sda] = (float) A[7+ii+j*lda];*/
/*			}*/
		j=0;
		for(; j<col-7; j+=8)
			{
			// unroll 0
			pA[0+(j+0)*bs+ii*sda] = (float) A[ii+0+(j+0)*lda];
			pA[1+(j+0)*bs+ii*sda] = (float) A[ii+1+(j+0)*lda];
			pA[2+(j+0)*bs+ii*sda] = (float) A[ii+2+(j+0)*lda];
			pA[3+(j+0)*bs+ii*sda] = (float) A[ii+3+(j+0)*lda];
			pA[4+(j+0)*bs+ii*sda] = (float) A[ii+4+(j+0)*lda];
			pA[5+(j+0)*bs+ii*sda] = (float) A[ii+5+(j+0)*lda];
			pA[6+(j+0)*bs+ii*sda] = (float) A[ii+6+(j+0)*lda];
			pA[7+(j+0)*bs+ii*sda] = (float) A[ii+7+(j+0)*lda];
			// unroll 1
			pA[0+(j+1)*bs+ii*sda] = (float) A[ii+0+(j+1)*lda];
			pA[1+(j+1)*bs+ii*sda] = (float) A[ii+1+(j+1)*lda];
			pA[2+(j+1)*bs+ii*sda] = (float) A[ii+2+(j+1)*lda];
			pA[3+(j+1)*bs+ii*sda] = (float) A[ii+3+(j+1)*lda];
			pA[4+(j+1)*bs+ii*sda] = (float) A[ii+4+(j+1)*lda];
			pA[5+(j+1)*bs+ii*sda] = (float) A[ii+5+(j+1)*lda];
			pA[6+(j+1)*bs+ii*sda] = (float) A[ii+6+(j+1)*lda];
			pA[7+(j+1)*bs+ii*sda] = (float) A[ii+7+(j+1)*lda];
			// unroll 2
			pA[0+(j+2)*bs+ii*sda] = (float) A[ii+0+(j+2)*lda];
			pA[1+(j+2)*bs+ii*sda] = (float) A[ii+1+(j+2)*lda];
			pA[2+(j+2)*bs+ii*sda] = (float) A[ii+2+(j+2)*lda];
			pA[3+(j+2)*bs+ii*sda] = (float) A[ii+3+(j+2)*lda];
			pA[4+(j+2)*bs+ii*sda] = (float) A[ii+4+(j+2)*lda];
			pA[5+(j+2)*bs+ii*sda] = (float) A[ii+5+(j+2)*lda];
			pA[6+(j+2)*bs+ii*sda] = (float) A[ii+6+(j+2)*lda];
			pA[7+(j+2)*bs+ii*sda] = (float) A[ii+7+(j+2)*lda];
			// unroll 3
			pA[0+(j+3)*bs+ii*sda] = (float) A[ii+0+(j+3)*lda];
			pA[1+(j+3)*bs+ii*sda] = (float) A[ii+1+(j+3)*lda];
			pA[2+(j+3)*bs+ii*sda] = (float) A[ii+2+(j+3)*lda];
			pA[3+(j+3)*bs+ii*sda] = (float) A[ii+3+(j+3)*lda];
			pA[4+(j+3)*bs+ii*sda] = (float) A[ii+4+(j+3)*lda];
			pA[5+(j+3)*bs+ii*sda] = (float) A[ii+5+(j+3)*lda];
			pA[6+(j+3)*bs+ii*sda] = (float) A[ii+6+(j+3)*lda];
			pA[7+(j+3)*bs+ii*sda] = (float) A[ii+7+(j+3)*lda];
			// unroll 4
			pA[0+(j+4)*bs+ii*sda] = (float) A[ii+0+(j+4)*lda];
			pA[1+(j+4)*bs+ii*sda] = (float) A[ii+1+(j+4)*lda];
			pA[2+(j+4)*bs+ii*sda] = (float) A[ii+2+(j+4)*lda];
			pA[3+(j+4)*bs+ii*sda] = (float) A[ii+3+(j+4)*lda];
			pA[4+(j+4)*bs+ii*sda] = (float) A[ii+4+(j+4)*lda];
			pA[5+(j+4)*bs+ii*sda] = (float) A[ii+5+(j+4)*lda];
			pA[6+(j+4)*bs+ii*sda] = (float) A[ii+6+(j+4)*lda];
			pA[7+(j+4)*bs+ii*sda] = (float) A[ii+7+(j+4)*lda];
			// unroll 5
			pA[0+(j+5)*bs+ii*sda] = (float) A[ii+0+(j+5)*lda];
			pA[1+(j+5)*bs+ii*sda] = (float) A[ii+1+(j+5)*lda];
			pA[2+(j+5)*bs+ii*sda] = (float) A[ii+2+(j+5)*lda];
			pA[3+(j+5)*bs+ii*sda] = (float) A[ii+3+(j+5)*lda];
			pA[4+(j+5)*bs+ii*sda] = (float) A[ii+4+(j+5)*lda];
			pA[5+(j+5)*bs+ii*sda] = (float) A[ii+5+(j+5)*lda];
			pA[6+(j+5)*bs+ii*sda] = (float) A[ii+6+(j+5)*lda];
			pA[7+(j+5)*bs+ii*sda] = (float) A[ii+7+(j+5)*lda];
			// unroll 6
			pA[0+(j+6)*bs+ii*sda] = (float) A[ii+0+(j+6)*lda];
			pA[1+(j+6)*bs+ii*sda] = (float) A[ii+1+(j+6)*lda];
			pA[2+(j+6)*bs+ii*sda] = (float) A[ii+2+(j+6)*lda];
			pA[3+(j+6)*bs+ii*sda] = (float) A[ii+3+(j+6)*lda];
			pA[4+(j+6)*bs+ii*sda] = (float) A[ii+4+(j+6)*lda];
			pA[5+(j+6)*bs+ii*sda] = (float) A[ii+5+(j+6)*lda];
			pA[6+(j+6)*bs+ii*sda] = (float) A[ii+6+(j+6)*lda];
			pA[7+(j+6)*bs+ii*sda] = (float) A[ii+7+(j+6)*lda];
			// unroll 7
			pA[0+(j+7)*bs+ii*sda] = (float) A[ii+0+(j+7)*lda];
			pA[1+(j+7)*bs+ii*sda] = (float) A[ii+1+(j+7)*lda];
			pA[2+(j+7)*bs+ii*sda] = (float) A[ii+2+(j+7)*lda];
			pA[3+(j+7)*bs+ii*sda] = (float) A[ii+3+(j+7)*lda];
			pA[4+(j+7)*bs+ii*sda] = (float) A[ii+4+(j+7)*lda];
			pA[5+(j+7)*bs+ii*sda] = (float) A[ii+5+(j+7)*lda];
			pA[6+(j+7)*bs+ii*sda] = (float) A[ii+6+(j+7)*lda];
			pA[7+(j+7)*bs+ii*sda] = (float) A[ii+7+(j+7)*lda];
			}
		for(; j<col; j++)
			{
			pA[0+(j+0)*bs+ii*sda] = (float) A[ii+0+(j+0)*lda];
			pA[1+(j+0)*bs+ii*sda] = (float) A[ii+1+(j+0)*lda];
			pA[2+(j+0)*bs+ii*sda] = (float) A[ii+2+(j+0)*lda];
			pA[3+(j+0)*bs+ii*sda] = (float) A[ii+3+(j+0)*lda];
			pA[4+(j+0)*bs+ii*sda] = (float) A[ii+4+(j+0)*lda];
			pA[5+(j+0)*bs+ii*sda] = (float) A[ii+5+(j+0)*lda];
			pA[6+(j+0)*bs+ii*sda] = (float) A[ii+6+(j+0)*lda];
			pA[7+(j+0)*bs+ii*sda] = (float) A[ii+7+(j+0)*lda];
			}
		}
	if(ii<row1)
		{
		row2 = row1-ii;
		if(bs<row2) row2 = bs;
		for(j=0; j<col; j++)
			{
			for(i=0; i<row2; i++)
				{
				pA[i+j*bs+ii*sda] = (float) A[i+ii+j*lda];
				}
			}
		}
	
	}



/* converts a matrix into a packed matrix */
void s_cvt_mat2pmat(int row, int col, float *A, int lda, int offset, float *pA, int sda)
	{
	
	const int bs = 8;

	int i, ii, j, row0, row1, row2;
	
	row0 = (bs-offset%bs)%bs;
	if(row0>row)
		row0 = row;
	row1 = row - row0;
	
	ii = 0;
	if(row0>0)
		{
		for(j=0; j<col; j++)
			{
			for(i=0; i<row0; i++)
				{
				pA[i+j*bs+ii*sda] = A[i+ii+j*lda];
				}
			}
	
		A  += row0;
		pA += row0 + bs*(sda-1);
		}
	
	ii = 0;
	for(; ii<row1-7; ii+=bs)
		{
/*		for(j=0; j<col; j++)*/
/*			{*/
/*			pA[0+j*bs+ii*sda] = A[0+ii+j*lda];*/
/*			pA[1+j*bs+ii*sda] = A[1+ii+j*lda];*/
/*			pA[2+j*bs+ii*sda] = A[2+ii+j*lda];*/
/*			pA[3+j*bs+ii*sda] = A[3+ii+j*lda];*/
/*			pA[4+j*bs+ii*sda] = A[4+ii+j*lda];*/
/*			pA[5+j*bs+ii*sda] = A[5+ii+j*lda];*/
/*			pA[6+j*bs+ii*sda] = A[6+ii+j*lda];*/
/*			pA[7+j*bs+ii*sda] = A[7+ii+j*lda];*/
/*			}*/
		j=0;
		for(; j<col-7; j+=8)
			{
			// unroll 0
			pA[0+(j+0)*bs+ii*sda] = A[ii+0+(j+0)*lda];
			pA[1+(j+0)*bs+ii*sda] = A[ii+1+(j+0)*lda];
			pA[2+(j+0)*bs+ii*sda] = A[ii+2+(j+0)*lda];
			pA[3+(j+0)*bs+ii*sda] = A[ii+3+(j+0)*lda];
			pA[4+(j+0)*bs+ii*sda] = A[ii+4+(j+0)*lda];
			pA[5+(j+0)*bs+ii*sda] = A[ii+5+(j+0)*lda];
			pA[6+(j+0)*bs+ii*sda] = A[ii+6+(j+0)*lda];
			pA[7+(j+0)*bs+ii*sda] = A[ii+7+(j+0)*lda];
			// unroll 1
			pA[0+(j+1)*bs+ii*sda] = A[ii+0+(j+1)*lda];
			pA[1+(j+1)*bs+ii*sda] = A[ii+1+(j+1)*lda];
			pA[2+(j+1)*bs+ii*sda] = A[ii+2+(j+1)*lda];
			pA[3+(j+1)*bs+ii*sda] = A[ii+3+(j+1)*lda];
			pA[4+(j+1)*bs+ii*sda] = A[ii+4+(j+1)*lda];
			pA[5+(j+1)*bs+ii*sda] = A[ii+5+(j+1)*lda];
			pA[6+(j+1)*bs+ii*sda] = A[ii+6+(j+1)*lda];
			pA[7+(j+1)*bs+ii*sda] = A[ii+7+(j+1)*lda];
			// unroll 2
			pA[0+(j+2)*bs+ii*sda] = A[ii+0+(j+2)*lda];
			pA[1+(j+2)*bs+ii*sda] = A[ii+1+(j+2)*lda];
			pA[2+(j+2)*bs+ii*sda] = A[ii+2+(j+2)*lda];
			pA[3+(j+2)*bs+ii*sda] = A[ii+3+(j+2)*lda];
			pA[4+(j+2)*bs+ii*sda] = A[ii+4+(j+2)*lda];
			pA[5+(j+2)*bs+ii*sda] = A[ii+5+(j+2)*lda];
			pA[6+(j+2)*bs+ii*sda] = A[ii+6+(j+2)*lda];
			pA[7+(j+2)*bs+ii*sda] = A[ii+7+(j+2)*lda];
			// unroll 3
			pA[0+(j+3)*bs+ii*sda] = A[ii+0+(j+3)*lda];
			pA[1+(j+3)*bs+ii*sda] = A[ii+1+(j+3)*lda];
			pA[2+(j+3)*bs+ii*sda] = A[ii+2+(j+3)*lda];
			pA[3+(j+3)*bs+ii*sda] = A[ii+3+(j+3)*lda];
			pA[4+(j+3)*bs+ii*sda] = A[ii+4+(j+3)*lda];
			pA[5+(j+3)*bs+ii*sda] = A[ii+5+(j+3)*lda];
			pA[6+(j+3)*bs+ii*sda] = A[ii+6+(j+3)*lda];
			pA[7+(j+3)*bs+ii*sda] = A[ii+7+(j+3)*lda];
			// unroll 4
			pA[0+(j+4)*bs+ii*sda] = A[ii+0+(j+4)*lda];
			pA[1+(j+4)*bs+ii*sda] = A[ii+1+(j+4)*lda];
			pA[2+(j+4)*bs+ii*sda] = A[ii+2+(j+4)*lda];
			pA[3+(j+4)*bs+ii*sda] = A[ii+3+(j+4)*lda];
			pA[4+(j+4)*bs+ii*sda] = A[ii+4+(j+4)*lda];
			pA[5+(j+4)*bs+ii*sda] = A[ii+5+(j+4)*lda];
			pA[6+(j+4)*bs+ii*sda] = A[ii+6+(j+4)*lda];
			pA[7+(j+4)*bs+ii*sda] = A[ii+7+(j+4)*lda];
			// unroll 5
			pA[0+(j+5)*bs+ii*sda] = A[ii+0+(j+5)*lda];
			pA[1+(j+5)*bs+ii*sda] = A[ii+1+(j+5)*lda];
			pA[2+(j+5)*bs+ii*sda] = A[ii+2+(j+5)*lda];
			pA[3+(j+5)*bs+ii*sda] = A[ii+3+(j+5)*lda];
			pA[4+(j+5)*bs+ii*sda] = A[ii+4+(j+5)*lda];
			pA[5+(j+5)*bs+ii*sda] = A[ii+5+(j+5)*lda];
			pA[6+(j+5)*bs+ii*sda] = A[ii+6+(j+5)*lda];
			pA[7+(j+5)*bs+ii*sda] = A[ii+7+(j+5)*lda];
			// unroll 6
			pA[0+(j+6)*bs+ii*sda] = A[ii+0+(j+6)*lda];
			pA[1+(j+6)*bs+ii*sda] = A[ii+1+(j+6)*lda];
			pA[2+(j+6)*bs+ii*sda] = A[ii+2+(j+6)*lda];
			pA[3+(j+6)*bs+ii*sda] = A[ii+3+(j+6)*lda];
			pA[4+(j+6)*bs+ii*sda] = A[ii+4+(j+6)*lda];
			pA[5+(j+6)*bs+ii*sda] = A[ii+5+(j+6)*lda];
			pA[6+(j+6)*bs+ii*sda] = A[ii+6+(j+6)*lda];
			pA[7+(j+6)*bs+ii*sda] = A[ii+7+(j+6)*lda];
			// unroll 7
			pA[0+(j+7)*bs+ii*sda] = A[ii+0+(j+7)*lda];
			pA[1+(j+7)*bs+ii*sda] = A[ii+1+(j+7)*lda];
			pA[2+(j+7)*bs+ii*sda] = A[ii+2+(j+7)*lda];
			pA[3+(j+7)*bs+ii*sda] = A[ii+3+(j+7)*lda];
			pA[4+(j+7)*bs+ii*sda] = A[ii+4+(j+7)*lda];
			pA[5+(j+7)*bs+ii*sda] = A[ii+5+(j+7)*lda];
			pA[6+(j+7)*bs+ii*sda] = A[ii+6+(j+7)*lda];
			pA[7+(j+7)*bs+ii*sda] = A[ii+7+(j+7)*lda];
			}
		for(; j<col; j++)
			{
			pA[0+(j+0)*bs+ii*sda] = A[ii+0+(j+0)*lda];
			pA[1+(j+0)*bs+ii*sda] = A[ii+1+(j+0)*lda];
			pA[2+(j+0)*bs+ii*sda] = A[ii+2+(j+0)*lda];
			pA[3+(j+0)*bs+ii*sda] = A[ii+3+(j+0)*lda];
			pA[4+(j+0)*bs+ii*sda] = A[ii+4+(j+0)*lda];
			pA[5+(j+0)*bs+ii*sda] = A[ii+5+(j+0)*lda];
			pA[6+(j+0)*bs+ii*sda] = A[ii+6+(j+0)*lda];
			pA[7+(j+0)*bs+ii*sda] = A[ii+7+(j+0)*lda];
			}
		}
	if(ii<row1)
		{
		row2 = row1-ii;
		if(bs<row2) row2 = bs;
		for(j=0; j<col; j++)
			{
			for(i=0; i<row2; i++)
				{
				pA[i+j*bs+ii*sda] = A[i+ii+j*lda];
				}
			}
		}
	
	}



/* converts a matrix into a packed matrix */
// row and col of the source matrix, offsett in the destination matrix
void s_cvt_tran_mat2pmat(int row, int col, int offset, int bs_dummy, float *A, int lda, float *pA, int sda)
	{
	
	const int bs = 8;

	int i, ii, j, row0, row1, row2;
	
	row0 = (bs-offset%bs)%bs;
	if(row0>col)
		row0 = col;
	row1 = col - row0;
	
	ii = 0;
	if(row0>0)
		{
		for(j=0; j<row; j++)
			{
			for(i=0; i<row0; i++)
				{
				pA[i+j*bs+ii*sda] = A[j+(i+ii)*lda];
				}
			}
	
		A  += row0*lda;
		pA += row0 + bs*(sda-1);
		}
	
	ii = 0;
	for(; ii<row1-7; ii+=bs)
		{
		j=0;
		for(; j<row-7; j+=8)
			{
			// unroll 0
			pA[0+(j+0)*bs+ii*sda] = A[j+0+(ii+0)*lda];
			pA[1+(j+0)*bs+ii*sda] = A[j+0+(ii+1)*lda];
			pA[2+(j+0)*bs+ii*sda] = A[j+0+(ii+2)*lda];
			pA[3+(j+0)*bs+ii*sda] = A[j+0+(ii+3)*lda];
			pA[4+(j+0)*bs+ii*sda] = A[j+0+(ii+4)*lda];
			pA[5+(j+0)*bs+ii*sda] = A[j+0+(ii+5)*lda];
			pA[6+(j+0)*bs+ii*sda] = A[j+0+(ii+6)*lda];
			pA[7+(j+0)*bs+ii*sda] = A[j+0+(ii+7)*lda];
			// unroll 1
			pA[0+(j+1)*bs+ii*sda] = A[j+1+(ii+0)*lda];
			pA[1+(j+1)*bs+ii*sda] = A[j+1+(ii+1)*lda];
			pA[2+(j+1)*bs+ii*sda] = A[j+1+(ii+2)*lda];
			pA[3+(j+1)*bs+ii*sda] = A[j+1+(ii+3)*lda];
			pA[4+(j+1)*bs+ii*sda] = A[j+1+(ii+4)*lda];
			pA[5+(j+1)*bs+ii*sda] = A[j+1+(ii+5)*lda];
			pA[6+(j+1)*bs+ii*sda] = A[j+1+(ii+6)*lda];
			pA[7+(j+1)*bs+ii*sda] = A[j+1+(ii+7)*lda];
			// unroll 2
			pA[0+(j+2)*bs+ii*sda] = A[j+2+(ii+0)*lda];
			pA[1+(j+2)*bs+ii*sda] = A[j+2+(ii+1)*lda];
			pA[2+(j+2)*bs+ii*sda] = A[j+2+(ii+2)*lda];
			pA[3+(j+2)*bs+ii*sda] = A[j+2+(ii+3)*lda];
			pA[4+(j+2)*bs+ii*sda] = A[j+2+(ii+4)*lda];
			pA[5+(j+2)*bs+ii*sda] = A[j+2+(ii+5)*lda];
			pA[6+(j+2)*bs+ii*sda] = A[j+2+(ii+6)*lda];
			pA[7+(j+2)*bs+ii*sda] = A[j+2+(ii+7)*lda];
			// unroll 3
			pA[0+(j+3)*bs+ii*sda] = A[j+3+(ii+0)*lda];
			pA[1+(j+3)*bs+ii*sda] = A[j+3+(ii+1)*lda];
			pA[2+(j+3)*bs+ii*sda] = A[j+3+(ii+2)*lda];
			pA[3+(j+3)*bs+ii*sda] = A[j+3+(ii+3)*lda];
			pA[4+(j+3)*bs+ii*sda] = A[j+3+(ii+4)*lda];
			pA[5+(j+3)*bs+ii*sda] = A[j+3+(ii+5)*lda];
			pA[6+(j+3)*bs+ii*sda] = A[j+3+(ii+6)*lda];
			pA[7+(j+3)*bs+ii*sda] = A[j+3+(ii+7)*lda];
			// unroll 4
			pA[0+(j+4)*bs+ii*sda] = A[j+4+(ii+0)*lda];
			pA[1+(j+4)*bs+ii*sda] = A[j+4+(ii+1)*lda];
			pA[2+(j+4)*bs+ii*sda] = A[j+4+(ii+2)*lda];
			pA[3+(j+4)*bs+ii*sda] = A[j+4+(ii+3)*lda];
			pA[4+(j+4)*bs+ii*sda] = A[j+4+(ii+4)*lda];
			pA[5+(j+4)*bs+ii*sda] = A[j+4+(ii+5)*lda];
			pA[6+(j+4)*bs+ii*sda] = A[j+4+(ii+6)*lda];
			pA[7+(j+4)*bs+ii*sda] = A[j+4+(ii+7)*lda];
			// unroll 5
			pA[0+(j+5)*bs+ii*sda] = A[j+5+(ii+0)*lda];
			pA[1+(j+5)*bs+ii*sda] = A[j+5+(ii+1)*lda];
			pA[2+(j+5)*bs+ii*sda] = A[j+5+(ii+2)*lda];
			pA[3+(j+5)*bs+ii*sda] = A[j+5+(ii+3)*lda];
			pA[4+(j+5)*bs+ii*sda] = A[j+5+(ii+4)*lda];
			pA[5+(j+5)*bs+ii*sda] = A[j+5+(ii+5)*lda];
			pA[6+(j+5)*bs+ii*sda] = A[j+5+(ii+6)*lda];
			pA[7+(j+5)*bs+ii*sda] = A[j+5+(ii+7)*lda];
			// unroll 6
			pA[0+(j+6)*bs+ii*sda] = A[j+6+(ii+0)*lda];
			pA[1+(j+6)*bs+ii*sda] = A[j+6+(ii+1)*lda];
			pA[2+(j+6)*bs+ii*sda] = A[j+6+(ii+2)*lda];
			pA[3+(j+6)*bs+ii*sda] = A[j+6+(ii+3)*lda];
			pA[4+(j+6)*bs+ii*sda] = A[j+6+(ii+4)*lda];
			pA[5+(j+6)*bs+ii*sda] = A[j+6+(ii+5)*lda];
			pA[6+(j+6)*bs+ii*sda] = A[j+6+(ii+6)*lda];
			pA[7+(j+6)*bs+ii*sda] = A[j+6+(ii+7)*lda];
			// unroll 7
			pA[0+(j+7)*bs+ii*sda] = A[j+7+(ii+0)*lda];
			pA[1+(j+7)*bs+ii*sda] = A[j+7+(ii+1)*lda];
			pA[2+(j+7)*bs+ii*sda] = A[j+7+(ii+2)*lda];
			pA[3+(j+7)*bs+ii*sda] = A[j+7+(ii+3)*lda];
			pA[4+(j+7)*bs+ii*sda] = A[j+7+(ii+4)*lda];
			pA[5+(j+7)*bs+ii*sda] = A[j+7+(ii+5)*lda];
			pA[6+(j+7)*bs+ii*sda] = A[j+7+(ii+6)*lda];
			pA[7+(j+7)*bs+ii*sda] = A[j+7+(ii+7)*lda];
			}
		for(; j<row; j++)
			{
			pA[0+(j+0)*bs+ii*sda] = A[j+0+(ii+0)*lda];
			pA[1+(j+0)*bs+ii*sda] = A[j+0+(ii+1)*lda];
			pA[2+(j+0)*bs+ii*sda] = A[j+0+(ii+2)*lda];
			pA[3+(j+0)*bs+ii*sda] = A[j+0+(ii+3)*lda];
			pA[4+(j+0)*bs+ii*sda] = A[j+0+(ii+4)*lda];
			pA[5+(j+0)*bs+ii*sda] = A[j+0+(ii+5)*lda];
			pA[6+(j+0)*bs+ii*sda] = A[j+0+(ii+6)*lda];
			pA[7+(j+0)*bs+ii*sda] = A[j+0+(ii+7)*lda];
			}
		}
	if(ii<row1)
		{
		row2 = row1-ii;
		if(bs<row2) row2 = bs;
		for(j=0; j<row; j++)
			{
			for(i=0; i<row2; i++)
				{
				pA[i+j*bs+ii*sda] = A[j+(i+ii)*lda];
				}
			}
		}
	
	}



/* converts a matrix into a packed matrix */
// row and col of the source matrix, offsett in the destination matrix
void cvt_tran_d2s_mat2pmat(int row, int col, int offset, int bs_dummy, double *A, int lda, float *pA, int sda)
	{
	
	const int bs = 8;

	int i, ii, j, row0, row1, row2;
	
	row0 = (bs-offset%bs)%bs;
	if(row0>col)
		row0 = col;
	row1 = col - row0;
	
	ii = 0;
	if(row0>0)
		{
		for(j=0; j<row; j++)
			{
			for(i=0; i<row0; i++)
				{
				pA[i+j*bs+ii*sda] = (float) A[j+(i+ii)*lda];
				}
			}
	
		A  += row0*lda;
		pA += row0 + bs*(sda-1);
		}
	
	ii = 0;
	for(; ii<row1-7; ii+=bs)
		{
		j=0;
		for(; j<row-7; j+=8)
			{
			// unroll 0
			pA[0+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+0)*lda];
			pA[1+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+1)*lda];
			pA[2+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+2)*lda];
			pA[3+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+3)*lda];
			pA[4+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+4)*lda];
			pA[5+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+5)*lda];
			pA[6+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+6)*lda];
			pA[7+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+7)*lda];
			// unroll 1
			pA[0+(j+1)*bs+ii*sda] = (float) A[j+1+(ii+0)*lda];
			pA[1+(j+1)*bs+ii*sda] = (float) A[j+1+(ii+1)*lda];
			pA[2+(j+1)*bs+ii*sda] = (float) A[j+1+(ii+2)*lda];
			pA[3+(j+1)*bs+ii*sda] = (float) A[j+1+(ii+3)*lda];
			pA[4+(j+1)*bs+ii*sda] = (float) A[j+1+(ii+4)*lda];
			pA[5+(j+1)*bs+ii*sda] = (float) A[j+1+(ii+5)*lda];
			pA[6+(j+1)*bs+ii*sda] = (float) A[j+1+(ii+6)*lda];
			pA[7+(j+1)*bs+ii*sda] = (float) A[j+1+(ii+7)*lda];
			// unroll 2
			pA[0+(j+2)*bs+ii*sda] = (float) A[j+2+(ii+0)*lda];
			pA[1+(j+2)*bs+ii*sda] = (float) A[j+2+(ii+1)*lda];
			pA[2+(j+2)*bs+ii*sda] = (float) A[j+2+(ii+2)*lda];
			pA[3+(j+2)*bs+ii*sda] = (float) A[j+2+(ii+3)*lda];
			pA[4+(j+2)*bs+ii*sda] = (float) A[j+2+(ii+4)*lda];
			pA[5+(j+2)*bs+ii*sda] = (float) A[j+2+(ii+5)*lda];
			pA[6+(j+2)*bs+ii*sda] = (float) A[j+2+(ii+6)*lda];
			pA[7+(j+2)*bs+ii*sda] = (float) A[j+2+(ii+7)*lda];
			// unroll 3
			pA[0+(j+3)*bs+ii*sda] = (float) A[j+3+(ii+0)*lda];
			pA[1+(j+3)*bs+ii*sda] = (float) A[j+3+(ii+1)*lda];
			pA[2+(j+3)*bs+ii*sda] = (float) A[j+3+(ii+2)*lda];
			pA[3+(j+3)*bs+ii*sda] = (float) A[j+3+(ii+3)*lda];
			pA[4+(j+3)*bs+ii*sda] = (float) A[j+3+(ii+4)*lda];
			pA[5+(j+3)*bs+ii*sda] = (float) A[j+3+(ii+5)*lda];
			pA[6+(j+3)*bs+ii*sda] = (float) A[j+3+(ii+6)*lda];
			pA[7+(j+3)*bs+ii*sda] = (float) A[j+3+(ii+7)*lda];
			// unroll 4
			pA[0+(j+4)*bs+ii*sda] = (float) A[j+4+(ii+0)*lda];
			pA[1+(j+4)*bs+ii*sda] = (float) A[j+4+(ii+1)*lda];
			pA[2+(j+4)*bs+ii*sda] = (float) A[j+4+(ii+2)*lda];
			pA[3+(j+4)*bs+ii*sda] = (float) A[j+4+(ii+3)*lda];
			pA[4+(j+4)*bs+ii*sda] = (float) A[j+4+(ii+4)*lda];
			pA[5+(j+4)*bs+ii*sda] = (float) A[j+4+(ii+5)*lda];
			pA[6+(j+4)*bs+ii*sda] = (float) A[j+4+(ii+6)*lda];
			pA[7+(j+4)*bs+ii*sda] = (float) A[j+4+(ii+7)*lda];
			// unroll 5
			pA[0+(j+5)*bs+ii*sda] = (float) A[j+5+(ii+0)*lda];
			pA[1+(j+5)*bs+ii*sda] = (float) A[j+5+(ii+1)*lda];
			pA[2+(j+5)*bs+ii*sda] = (float) A[j+5+(ii+2)*lda];
			pA[3+(j+5)*bs+ii*sda] = (float) A[j+5+(ii+3)*lda];
			pA[4+(j+5)*bs+ii*sda] = (float) A[j+5+(ii+4)*lda];
			pA[5+(j+5)*bs+ii*sda] = (float) A[j+5+(ii+5)*lda];
			pA[6+(j+5)*bs+ii*sda] = (float) A[j+5+(ii+6)*lda];
			pA[7+(j+5)*bs+ii*sda] = (float) A[j+5+(ii+7)*lda];
			// unroll 6
			pA[0+(j+6)*bs+ii*sda] = (float) A[j+6+(ii+0)*lda];
			pA[1+(j+6)*bs+ii*sda] = (float) A[j+6+(ii+1)*lda];
			pA[2+(j+6)*bs+ii*sda] = (float) A[j+6+(ii+2)*lda];
			pA[3+(j+6)*bs+ii*sda] = (float) A[j+6+(ii+3)*lda];
			pA[4+(j+6)*bs+ii*sda] = (float) A[j+6+(ii+4)*lda];
			pA[5+(j+6)*bs+ii*sda] = (float) A[j+6+(ii+5)*lda];
			pA[6+(j+6)*bs+ii*sda] = (float) A[j+6+(ii+6)*lda];
			pA[7+(j+6)*bs+ii*sda] = (float) A[j+6+(ii+7)*lda];
			// unroll 7
			pA[0+(j+7)*bs+ii*sda] = (float) A[j+7+(ii+0)*lda];
			pA[1+(j+7)*bs+ii*sda] = (float) A[j+7+(ii+1)*lda];
			pA[2+(j+7)*bs+ii*sda] = (float) A[j+7+(ii+2)*lda];
			pA[3+(j+7)*bs+ii*sda] = (float) A[j+7+(ii+3)*lda];
			pA[4+(j+7)*bs+ii*sda] = (float) A[j+7+(ii+4)*lda];
			pA[5+(j+7)*bs+ii*sda] = (float) A[j+7+(ii+5)*lda];
			pA[6+(j+7)*bs+ii*sda] = (float) A[j+7+(ii+6)*lda];
			pA[7+(j+7)*bs+ii*sda] = (float) A[j+7+(ii+7)*lda];
			}
		for(; j<row; j++)
			{
			pA[0+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+0)*lda];
			pA[1+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+1)*lda];
			pA[2+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+2)*lda];
			pA[3+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+3)*lda];
			pA[4+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+4)*lda];
			pA[5+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+5)*lda];
			pA[6+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+6)*lda];
			pA[7+(j+0)*bs+ii*sda] = (float) A[j+0+(ii+7)*lda];
			}
		}
	if(ii<row1)
		{
		row2 = row1-ii;
		if(bs<row2) row2 = bs;
		for(j=0; j<row; j++)
			{
			for(i=0; i<row2; i++)
				{
				pA[i+j*bs+ii*sda] = (float) A[j+(i+ii)*lda];
				}
			}
		}
	
	}



/* converts a packed matrix into a matrix */
void s_cvt_pmat2mat(int row, int col, int offset, int bs_dummy, float *pA, int sda, float *A, int lda)
	{
	
	const int bs = 8;

	int i, ii, jj;
	
	int row0 = (bs-offset%bs)%bs;
	
	float *ptr_pA;
	

	jj=0;
	for(; jj<col; jj++)
		{
		ptr_pA = pA + jj*bs;
		ii = 0;
		if(row0>0)
			{
			for(; ii<row0; ii++)
				{
				A[ii+lda*jj] = ptr_pA[0];
				ptr_pA++;
				}
			ptr_pA += (sda-1)*bs;
			}
		for(; ii<row-bs+1; ii+=bs)
			{
			i=0;
			for(; i<bs; i++)
				{
				A[i+ii+lda*jj] = ptr_pA[0];
				ptr_pA++;
				}
			ptr_pA += (sda-1)*bs;
			}
		for(; ii<row; ii++)
			{
			A[ii+lda*jj] = ptr_pA[0];
			ptr_pA++;
			}
		}

	}



/* prints a matrix */
void s_print_mat(int row, int col, float *A, int lda)
	{
	int i, j;
	for(i=0; i<row; i++)
		{
		for(j=0; j<col; j++)
			{
//			printf("%5.2f ", *(A+i+j*lda));
//			printf("%7.3f ", *(A+i+j*lda));
			printf("%9.5f ", *(A+i+j*lda));
/*			printf("%19.15f ", *(A+i+j*lda));*/
//			printf("%e\t", *(A+i+j*lda));
			}
		printf("\n");
		}
	printf("\n");
	}	

void s_print_mat_e(int row, int col, float *A, int lda)
	{
	int i, j;
	for(i=0; i<row; i++)
		{
		for(j=0; j<col; j++)
			{
			printf("%e\t", *(A+i+j*lda));
			}
		printf("\n");
		}
	printf("\n");
	}	



/* prints a packed matrix */
void s_print_pmat(int row, int col, int bs, float *A, int sda)
	{

	int ii, i, j, row2;

	for(ii=0; ii<row; ii+=bs)
		{
		row2 = row-ii; if(bs<row2) row2=bs;
		for(i=0; i<row2; i++)
			{
			for(j=0; j<col; j++)
				{
//				printf("%5.2f ", *(A+i+j*lda));
//				printf("%7.3f ", *(A+i+j*bs+ii*sda));
				printf("%9.5f ", *(A+i+j*bs+ii*sda));
//				printf("%19.15f ", *(A+i+j*lda));
//				printf("%e\t", *(A+i+j*lda));
				}
			printf("\n");
			}
//		printf("\n");
		}
	printf("\n");

	}	

