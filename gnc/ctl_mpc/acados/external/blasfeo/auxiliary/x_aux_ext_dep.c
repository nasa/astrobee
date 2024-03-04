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



// create a matrix structure for a matrix of size m*n by dynamically allocating the memory
void ALLOCATE_MAT(int m, int n, struct MAT *sA)
	{
	size_t size = MEMSIZE_MAT(m, n);
	void *mem;
	blasfeo_malloc_align(&mem, size);
	CREATE_MAT(m, n, sA, mem);
	return;
	}



// free memory of a matrix structure
void FREE_MAT(struct MAT *sA)
	{
	blasfeo_free_align(sA->mem);
	return;
	}



// create a vector structure for a vector of size m by dynamically allocating the memory
void ALLOCATE_VEC(int m, struct VEC *sa)
	{
	size_t size = MEMSIZE_VEC(m);
	void *mem;
	blasfeo_malloc_align(&mem, size);
	CREATE_VEC(m, sa, mem);
	return;
	}



// free memory of a matrix structure
void FREE_VEC(struct VEC *sa)
	{
	blasfeo_free_align(sa->mem);
	return;
	}



// print a matrix structure
void PRINT_MAT(int m, int n, struct MAT *sA, int ai, int aj)
	{
	int ii, jj;
	for(ii=0; ii<m; ii++)
		{
		for(jj=0; jj<n; jj++)
			{
			printf("%9.5f ", MATEL(sA, ai+ii, aj+jj));
			}
		printf("\n");
		}
	printf("\n");
	return;
	}



// print the transposed of a matrix structure
void PRINT_TRAN_MAT(int m, int n, struct MAT *sA, int ai, int aj)
	{
	int ii, jj;
	for(jj=0; jj<n; jj++)
		{
		for(ii=0; ii<m; ii++)
			{
			printf("%9.5f ", MATEL(sA, ai+ii, aj+jj));
			}
		printf("\n");
		}
	printf("\n");
	return;
	}



// print a vector structure
void PRINT_VEC(int m, struct VEC *sa, int ai)
	{
	int ii;
	for(ii=0; ii<m; ii++)
		{
		printf("%9.5f\n", VECEL(sa, ai+ii));
		}
	printf("\n");
	return;
	}



// print the transposed of a vector structure
void PRINT_TRAN_VEC(int m, struct VEC *sa, int ai)
	{
	int ii;
	for(ii=0; ii<m; ii++)
		{
		printf("%9.5f ", VECEL(sa, ai+ii));
		}
	printf("\n\n");
	return;
	}


// print a matrix structure
void PRINT_TO_FILE_MAT(FILE * file, int m, int n, struct MAT *sA, int ai, int aj)
	{
	int ii, jj;
	for(ii=0; ii<m; ii++)
		{
		for(jj=0; jj<n; jj++)
			{
			fprintf(file, "%9.5f ", MATEL(sA, ai+ii, aj+jj));
			}
		fprintf(file, "\n");
		}
	fprintf(file, "\n");
	return;
	}



// print a matrix structure
void PRINT_TO_FILE_EXP_MAT(FILE * file, int m, int n, struct MAT *sA, int ai, int aj)
	{
	int ii, jj;
	for(ii=0; ii<m; ii++)
		{
		for(jj=0; jj<n; jj++)
			{
			fprintf(file, "%9.5e ", MATEL(sA, ai+ii, aj+jj));
			}
		fprintf(file, "\n");
		}
	fprintf(file, "\n");
	return;
	}


// print a matrix structure
void PRINT_TO_STRING_MAT(char **buf_out, int m, int n, struct MAT *sA, int ai, int aj)
	{
	int ii, jj;
	for(ii=0; ii<m; ii++)
		{
		for(jj=0; jj<n; jj++)
			{
			*buf_out += sprintf(*buf_out, "%9.5f ", MATEL(sA, ai+ii, aj+jj));
			}
		*buf_out += sprintf(*buf_out, "\n");
		}
	*buf_out += sprintf(*buf_out, "\n");
	return;
	}


// print a vector structure
void PRINT_TO_FILE_VEC(FILE * file, int m, struct VEC *sa, int ai)
	{
	int ii;
	for(ii=0; ii<m; ii++)
		{
		fprintf(file, "%9.5f\n", VECEL(sa, ai+ii));
		}
	fprintf(file, "\n");
	return;
	}



// print a vector structure
void PRINT_TO_STRING_VEC(char **buf_out, int m, struct VEC *sa, int ai)
	{
	int ii;
	for(ii=0; ii<m; ii++)
		{
		*buf_out += sprintf(*buf_out, "%9.5f\n", VECEL(sa, ai+ii));
		}
	*buf_out += sprintf(*buf_out, "\n");
	return;
	}


// print the transposed of a vector structure
void PRINT_TO_FILE_TRAN_VEC(FILE * file, int m, struct VEC *sa, int ai)
	{
	int ii;
	for(ii=0; ii<m; ii++)
		{
		fprintf(file, "%9.5f ", VECEL(sa, ai+ii));
		}
	fprintf(file, "\n\n");
	return;
	}



// print the transposed of a vector structure
void PRINT_TO_STRING_TRAN_VEC(char **buf_out, int m, struct VEC *sa, int ai)
	{
	int ii;
	for(ii=0; ii<m; ii++)
		{
		*buf_out += sprintf(*buf_out, "%9.5f ", VECEL(sa, ai+ii));
		}
	*buf_out += sprintf(*buf_out, "\n\n");
	return;
	}


// print a matrix structure
void PRINT_EXP_MAT(int m, int n, struct MAT *sA, int ai, int aj)
	{
	int ii, jj;
	for(ii=0; ii<m; ii++)
		{
		for(jj=0; jj<n; jj++)
			{
			printf("%9.5e ", MATEL(sA, ai+ii, aj+jj));
			}
		printf("\n");
		}
	printf("\n");
	return;
	}



// print the transposed of a matrix structure
void PRINT_EXP_TRAN_MAT(int m, int n, struct MAT *sA, int ai, int aj)
	{
	int ii, jj;
	for(jj=0; jj<n; jj++)
		{
		for(ii=0; ii<m; ii++)
			{
			printf("%9.5e ", MATEL(sA, ai+ii, aj+jj));
			}
		printf("\n");
		}
	printf("\n");
	return;
	}



// print a vector structure
void PRINT_EXP_VEC(int m, struct VEC *sa, int ai)
	{
	int ii;
	for(ii=0; ii<m; ii++)
		{
		printf("%9.5e\n", VECEL(sa, ai+ii));
		}
	printf("\n");
	return;
	}



// print the transposed of a vector structure
void PRINT_EXP_TRAN_VEC(int m, struct VEC *sa, int ai)
	{
	int ii;
	for(ii=0; ii<m; ii++)
		{
		printf("%9.5e ", VECEL(sa, ai+ii));
		}
	printf("\n\n");
	return;
	}


