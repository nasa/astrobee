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



hpipm_size_t DENSE_QP_DIM_MEMSIZE()
	{

	hpipm_size_t size = 0;

	size = (size+8-1)/8*8;

	return size;

	}



void DENSE_QP_DIM_CREATE(struct DENSE_QP_DIM *dim, void *mem)
	{

	// zero memory (to avoid corrupted memory like e.g. NaN)
//	hpipm_size_t memsize = DENSE_QP_DIM_MEMSIZE();
//	hpipm_zero_memset(memsize, mem);

	dim->memsize = DENSE_QP_DIM_MEMSIZE();

	// initialize dims to zero by default

	dim->nv = 0;
	dim->ne = 0;
	dim->nb = 0;
	dim->ng = 0;
	dim->ns = 0;
	dim->nsb = 0;
	dim->nsg = 0;

	return;

	}


void DENSE_QP_DIM_SET_ALL(int nv, int ne, int nb, int ng, int nsb, int nsg, struct DENSE_QP_DIM *dim)
	{

	dim->nv = nv;
	dim->ne = ne;
	dim->nb = nb;
	dim->ng = ng;
	dim->ns = nsb+nsg;
	dim->nsb = nsb;
	dim->nsg = nsg;

	return;

	}



void DENSE_QP_DIM_SET(char *field_name, int value, struct DENSE_QP_DIM *dim)
	{
	if(hpipm_strcmp(field_name, "nv"))
		{ 
		DENSE_QP_DIM_SET_NV(value, dim);
		}
	else if(hpipm_strcmp(field_name, "ne"))
		{ 
		DENSE_QP_DIM_SET_NE(value, dim);
		}
	else if(hpipm_strcmp(field_name, "nb"))
		{
		DENSE_QP_DIM_SET_NB(value, dim);
		}
	else if(hpipm_strcmp(field_name, "ng"))
		{
		DENSE_QP_DIM_SET_NG(value, dim);
		}
	else if(hpipm_strcmp(field_name, "nsb"))
		{
		DENSE_QP_DIM_SET_NSB(value, dim);
		}
	else if(hpipm_strcmp(field_name, "nsg"))
		{
		DENSE_QP_DIM_SET_NSG(value, dim);
		}
	else if(hpipm_strcmp(field_name, "ns"))
		{
		DENSE_QP_DIM_SET_NS(value, dim);
		}
	else 
		{
		printf("error: SET_OCP_QP_DIM: wrong field %s\n", field_name);
		exit(1);
		}
	return;
	}



void DENSE_QP_DIM_SET_NV(int value, struct DENSE_QP_DIM *dim)
	{
	dim->nv = value;

	return;
	}



void DENSE_QP_DIM_SET_NE(int value, struct DENSE_QP_DIM *dim)
	{
	dim->ne = value;

	return;
	}



void DENSE_QP_DIM_SET_NB(int value, struct DENSE_QP_DIM *dim)
	{
	dim->nb = value;

	return;
	}



void DENSE_QP_DIM_SET_NG(int value, struct DENSE_QP_DIM *dim)
	{
	dim->ng = value;

	return;
	}



void DENSE_QP_DIM_SET_NSB(int value, struct DENSE_QP_DIM *dim)
	{
	dim->nsb = value;
	dim->ns = dim->nsb + dim->nsg;

	return;
	}



void DENSE_QP_DIM_SET_NSG(int value, struct DENSE_QP_DIM *dim)
	{
	dim->nsg = value;
	dim->ns = dim->nsb + dim->nsg;

	return;
	}



void DENSE_QP_DIM_SET_NS(int value, struct DENSE_QP_DIM *dim)
	{
	dim->ns = value;

	return;
	}


