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
void s_copy_pmat(int row, int col, int bs, float *A, int sda, float *B, int sdb)
	{
	
	int i, ii, j, row2;
	
	for(ii=0; ii<row; ii+=bs)
		{
		row2 = row-ii;
		if(bs<row2) row2 = bs;
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
void s_copy_pmat_lo(int row, int bs, float *A, int sda, float *B, int sdb)
	{
	
	int i, ii, j, row2, row0;
	
	for(ii=0; ii<row; ii+=bs)
		{
		row2 = row-ii;
		if(bs<row2) row2 = bs;
		for(j=0; j<ii+row2; j++)
			{
			row0 = j-ii;
			if(row0<0) row0=0;
			for(i=row0; i<row2; i++)
				{
				B[i+j*bs+ii*sdb] = A[i+j*bs+ii*sda];
				}
			}
		}
	
	}



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
void s_align_pmat(int row, int col, int offset, int bs, float *A, int sda, float *B, int sdb)
	{
	
	int i, j;
	
	float *ptrA, *ptrB;
	
	for(i=0; i<row; i++)
		{
		ptrA = A + ((offset+i)/bs)*bs*sda + ((offset+i)%bs);
		ptrB = B + (i/bs)*bs*sdb + (i%bs);
		for(j=0; j<col; j++)
			{
			ptrB[j*bs] = ptrA[j*bs];
			}
		}
	
	}



/* converts a double matrix into a single packed matrix */
void cvt_d2s_mat2pmat(int row, int col, int offset, int bs, double *A, int lda, float *pA, int sda)
	{
	
	int i, ii, j, row0, row1, row2;

	row0 = (bs-offset%bs)%bs;
	if(row0>row)
		row0 = row;
	row1 = row - row0;
	
	if(row0>0)
		{
		for(j=0; j<col; j++)
			{
			for(i=0; i<row0; i++)
				{
				pA[i+j*bs] = (float) A[i+j*lda];
				}
			}
	
		A += row0;
		pA += row0 + bs*(sda-1);
		}

	for(ii=0; ii<row1; ii+=bs)
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
void s_cvt_mat2pmat(int row, int col, int offset, int bs, float *A, int lda, float *pA, int sda)
	{
	
	int i, ii, j, row0, row1, row2;

	row0 = (bs-offset%bs)%bs;
	if(row0>row)
		row0 = row;
	row1 = row - row0;
	
	if(row0>0)
		{
		for(j=0; j<col; j++)
			{
			for(i=0; i<row0; i++)
				{
				pA[i+j*bs] = A[i+j*lda];
				}
			}
	
		A += row0;
		pA += row0 + bs*(sda-1);
		}

	for(ii=0; ii<row1; ii+=bs)
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



/* converts a packed matrix into a matrix */
void s_cvt_pmat2mat(int row, int col, int offset, int bs, float *pA, int sda, float *A, int lda)
	{
	
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

