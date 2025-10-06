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



hpipm_size_t OCP_QP_DIM_STRSIZE()
	{
	return sizeof(struct OCP_QP_DIM);
	}



hpipm_size_t OCP_QP_DIM_MEMSIZE(int N)
	{

	hpipm_size_t size = 0;

	size += 13*(N+1)*sizeof(int);

	size = (size+8-1)/8*8; // make multiple of 8 bytes

	return size;

	}



void OCP_QP_DIM_CREATE(int N, struct OCP_QP_DIM *dim, void *mem)
	{

	// loop index
	int ii;

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = OCP_QP_DIM_MEMSIZE(N);
	hpipm_zero_memset(memsize, mem);

	char *c_ptr = mem;

	// nx
	dim->nx = (int *) c_ptr;
	c_ptr += (N+1)*sizeof(int);
	for(ii=0; ii<=N; ii++)
		dim->nx[ii] = 0;
	// nu
	dim->nu = (int *) c_ptr;
	c_ptr += (N+1)*sizeof(int);
	for(ii=0; ii<=N; ii++)
		dim->nu[ii] = 0;
	// nb
	dim->nb = (int *) c_ptr;
	c_ptr += (N+1)*sizeof(int);
	for(ii=0; ii<=N; ii++)
		dim->nb[ii] = 0;
	// nbx
	dim->nbx = (int *) c_ptr;
	c_ptr += (N+1)*sizeof(int);
	for(ii=0; ii<=N; ii++)
		dim->nbx[ii] = 0;
	// nbu
	dim->nbu = (int *) c_ptr;
	c_ptr += (N+1)*sizeof(int);
	for(ii=0; ii<=N; ii++)
		dim->nbu[ii] = 0;
	// ng
	dim->ng = (int *) c_ptr;
	c_ptr += (N+1)*sizeof(int);
	for(ii=0; ii<=N; ii++)
		dim->ng[ii] = 0;
	// ns
	dim->ns = (int *) c_ptr;
	c_ptr += (N+1)*sizeof(int);
	for(ii=0; ii<=N; ii++)
		dim->ns[ii] = 0;
	// nsbx
	dim->nsbx = (int *) c_ptr;
	c_ptr += (N+1)*sizeof(int);
	for(ii=0; ii<=N; ii++)
		dim->nsbx[ii] = 0;
	// nsbu
	dim->nsbu = (int *) c_ptr;
	c_ptr += (N+1)*sizeof(int);
	for(ii=0; ii<=N; ii++)
		dim->nsbu[ii] = 0;
	// nsg
	dim->nsg = (int *) c_ptr;
	c_ptr += (N+1)*sizeof(int);
	for(ii=0; ii<=N; ii++)
		dim->nsg[ii] = 0;
	// nbxe
	dim->nbxe = (int *) c_ptr;
	c_ptr += (N+1)*sizeof(int);
	for(ii=0; ii<=N; ii++)
		dim->nbxe[ii] = 0;
	// nbue
	dim->nbue = (int *) c_ptr;
	c_ptr += (N+1)*sizeof(int);
	for(ii=0; ii<=N; ii++)
		dim->nbue[ii] = 0;
	// nge
	dim->nge = (int *) c_ptr;
	c_ptr += (N+1)*sizeof(int);
	for(ii=0; ii<=N; ii++)
		dim->nge[ii] = 0;

	// N
	dim->N = N;

	dim->memsize = OCP_QP_DIM_MEMSIZE(N);

	return;

	}


void OCP_QP_DIM_COPY_ALL(struct OCP_QP_DIM *dim_orig, struct OCP_QP_DIM *dim_dest)
	{

#if defined(RUNTIME_CHECKS)
	if(dim_orig->N!=dim_dest->N)
		{
		printf("\nerror: OCP_QP_DIM_COPY_ALL: dim_orig->N != dim_dest->N\n");
		exit(1);
		}
#endif

	// loop index
	int ii;

	// N
	int N = dim_orig->N;

	// copy qp dim
	for(ii=0; ii<=N; ii++)
		dim_dest->nx[ii] = dim_orig->nx[ii];
	for(ii=0; ii<=N; ii++)
		dim_dest->nu[ii] = dim_orig->nu[ii];
	for(ii=0; ii<=N; ii++)
		dim_dest->nb[ii] = dim_orig->nb[ii];
	for(ii=0; ii<=N; ii++)
		dim_dest->nbx[ii] = dim_orig->nbx[ii];
	for(ii=0; ii<=N; ii++)
		dim_dest->nbu[ii] = dim_orig->nbu[ii];
	for(ii=0; ii<=N; ii++)
		dim_dest->ng[ii] = dim_orig->ng[ii];
	for(ii=0; ii<=N; ii++)
		dim_dest->ns[ii] = dim_orig->ns[ii];
	for(ii=0; ii<=N; ii++)
		dim_dest->nsbx[ii] = dim_orig->nsbx[ii];
	for(ii=0; ii<=N; ii++)
		dim_dest->nsbu[ii] = dim_orig->nsbu[ii];
	for(ii=0; ii<=N; ii++)
		dim_dest->nsg[ii] = dim_orig->nsg[ii];
	for(ii=0; ii<=N; ii++)
		dim_dest->nbxe[ii] = dim_orig->nbxe[ii];
	for(ii=0; ii<=N; ii++)
		dim_dest->nbue[ii] = dim_orig->nbue[ii];
	for(ii=0; ii<=N; ii++)
		dim_dest->nge[ii] = dim_orig->nge[ii];

	return;

	}



// TODO deprecated and remove ???
void OCP_QP_DIM_SET_ALL(int *nx, int *nu, int *nbx, int *nbu, int *ng, int *nsbx, int *nsbu, int *nsg, struct OCP_QP_DIM *dim)
	{

	// loop index
	int ii;

	// N
	int N = dim->N;

	// copy qp dim
	for(ii=0; ii<=N; ii++)
		dim->nx[ii] = nx[ii];
	for(ii=0; ii<=N; ii++)
		dim->nu[ii] = nu[ii];
	for(ii=0; ii<=N; ii++)
		dim->nb[ii] = nbx[ii]+nbu[ii];
	for(ii=0; ii<=N; ii++)
		dim->nbx[ii] = nbx[ii];
	for(ii=0; ii<=N; ii++)
		dim->nbu[ii] = nbu[ii];
	for(ii=0; ii<=N; ii++)
		dim->ng[ii] = ng[ii];
	for(ii=0; ii<=N; ii++)
		dim->ns[ii] = nsbx[ii]+nsbu[ii]+nsg[ii];
	for(ii=0; ii<=N; ii++)
		dim->nsbx[ii] = nsbx[ii];
	for(ii=0; ii<=N; ii++)
		dim->nsbu[ii] = nsbu[ii];
	for(ii=0; ii<=N; ii++)
		dim->nsg[ii] = nsg[ii];

	return;

	}



void OCP_QP_DIM_SET(char *field_name, int stage, int value, struct OCP_QP_DIM *dim)
	{
	if(hpipm_strcmp(field_name, "nx"))
		{ 
		OCP_QP_DIM_SET_NX(stage, value, dim);
		}
	else if(hpipm_strcmp(field_name, "nu"))
		{ 
		OCP_QP_DIM_SET_NU(stage, value, dim);
		}
	else if(hpipm_strcmp(field_name, "nbx"))
		{
		OCP_QP_DIM_SET_NBX(stage, value, dim);
		}
	else if(hpipm_strcmp(field_name, "nbu"))
		{
		OCP_QP_DIM_SET_NBU(stage, value, dim);
		}
	else if(hpipm_strcmp(field_name, "ng"))
		{
		OCP_QP_DIM_SET_NG(stage, value, dim);
		}
	else if(hpipm_strcmp(field_name, "ns"))
		{
		OCP_QP_DIM_SET_NS(stage, value, dim);
		}
	else if(hpipm_strcmp(field_name, "nsbx"))
		{
		OCP_QP_DIM_SET_NSBX(stage, value, dim);
		}
	else if(hpipm_strcmp(field_name, "nsbu"))
		{
		OCP_QP_DIM_SET_NSBU(stage, value, dim);
		}
	else if(hpipm_strcmp(field_name, "nsg"))
		{
		OCP_QP_DIM_SET_NSG(stage, value, dim);
		}
	else if(hpipm_strcmp(field_name, "nbxe"))
		{
		OCP_QP_DIM_SET_NBXE(stage, value, dim);
		}
	else if(hpipm_strcmp(field_name, "nbue"))
		{
		OCP_QP_DIM_SET_NBUE(stage, value, dim);
		}
	else if(hpipm_strcmp(field_name, "nge"))
		{
		OCP_QP_DIM_SET_NGE(stage, value, dim);
		}
	else 
		{
		printf("error: OCP_QP_DIM_SET: wrong field %s\n", field_name);
		exit(1);
		}
	return;
	}



void OCP_QP_DIM_SET_NX(int stage, int value, struct OCP_QP_DIM *dim)
	{
	dim->nx[stage] = value;
	return;
	}



void OCP_QP_DIM_SET_NU(int stage, int value, struct OCP_QP_DIM *dim)
	{
	dim->nu[stage] = value;
	return;
	}



void OCP_QP_DIM_SET_NBX(int stage, int value, struct OCP_QP_DIM *dim)
	{
	dim->nbx[stage] = value;
	dim->nb[stage] = dim->nbx[stage] + dim->nbu[stage];
	return;
	}



void OCP_QP_DIM_SET_NBU(int stage, int value, struct OCP_QP_DIM *dim)
	{
	dim->nbu[stage] = value;
	dim->nb[stage] = dim->nbx[stage] + dim->nbu[stage];
	return;
	}



void OCP_QP_DIM_SET_NG(int stage, int value, struct OCP_QP_DIM *dim)
	{
	dim->ng[stage] = value;
	return;
	}



void OCP_QP_DIM_SET_NS(int stage, int value, struct OCP_QP_DIM *dim)
	{
	dim->ns[stage] = value;
	return;
	}



void OCP_QP_DIM_SET_NSBX(int stage, int value, struct OCP_QP_DIM *dim)
	{
	dim->nsbx[stage] = value;
	dim->ns[stage] = dim->nsbx[stage] + dim->nsbu[stage] + dim->nsg[stage];
	return;
	}



void OCP_QP_DIM_SET_NSBU(int stage, int value, struct OCP_QP_DIM *dim)
	{
	dim->nsbu[stage] = value;
	dim->ns[stage] = dim->nsbx[stage] + dim->nsbu[stage] + dim->nsg[stage];
	return;
	}



void OCP_QP_DIM_SET_NSG(int stage, int value, struct OCP_QP_DIM *dim)
	{
	dim->nsg[stage] = value;
	dim->ns[stage] = dim->nsbx[stage] + dim->nsbu[stage] + dim->nsg[stage];
	return;
	}



void OCP_QP_DIM_SET_NBXE(int stage, int value, struct OCP_QP_DIM *dim)
	{
	dim->nbxe[stage] = value;
	return;
	}



void OCP_QP_DIM_SET_NBUE(int stage, int value, struct OCP_QP_DIM *dim)
	{
	dim->nbue[stage] = value;
	return;
	}



void OCP_QP_DIM_SET_NGE(int stage, int value, struct OCP_QP_DIM *dim)
	{
	dim->nge[stage] = value;
	return;
	}



void OCP_QP_DIM_GET(struct OCP_QP_DIM *dim, char *field_name, int stage, int *value)
	{
	if(hpipm_strcmp(field_name, "nx"))
		{
		OCP_QP_DIM_GET_NX(dim, stage, value);
		}
	else if(hpipm_strcmp(field_name, "nu"))
		{
		OCP_QP_DIM_GET_NU(dim, stage, value);
		}
	else if(hpipm_strcmp(field_name, "nbx"))
		{
		OCP_QP_DIM_GET_NBX(dim, stage, value);
		}
	else if(hpipm_strcmp(field_name, "nbu"))
		{
		OCP_QP_DIM_GET_NBU(dim, stage, value);
		}
	else if(hpipm_strcmp(field_name, "ng"))
		{
		OCP_QP_DIM_GET_NG(dim, stage, value);
		}
	else if(hpipm_strcmp(field_name, "ns"))
		{
		OCP_QP_DIM_GET_NS(dim, stage, value);
		}
	else if(hpipm_strcmp(field_name, "nsbx"))
		{
		OCP_QP_DIM_GET_NSBX(dim, stage, value);
		}
	else if(hpipm_strcmp(field_name, "nsbu"))
		{
		OCP_QP_DIM_GET_NSBU(dim, stage, value);
		}
	else if(hpipm_strcmp(field_name, "nsg"))
		{
		OCP_QP_DIM_GET_NSG(dim, stage, value);
		}
	else if(hpipm_strcmp(field_name, "nbxe"))
		{
		OCP_QP_DIM_GET_NBXE(dim, stage, value);
		}
	else if(hpipm_strcmp(field_name, "nbue"))
		{
		OCP_QP_DIM_GET_NBUE(dim, stage, value);
		}
	else if(hpipm_strcmp(field_name, "nge"))
		{
		OCP_QP_DIM_GET_NGE(dim, stage, value);
		}
	else
		{
		printf("error: OCP_QP_DIM_GET: wrong field %s\n", field_name);
		exit(1);
		}
	return;
	}



void OCP_QP_DIM_GET_N(struct OCP_QP_DIM *dim, int *value)
	{
	*value = dim->N;
	return;
	}



void OCP_QP_DIM_GET_NX(struct OCP_QP_DIM *dim, int stage, int *value)
	{
	*value = dim->nx[stage];
	return;
	}



void OCP_QP_DIM_GET_NU(struct OCP_QP_DIM *dim, int stage, int *value)
	{
	*value = dim->nu[stage];
	return;
	}



void OCP_QP_DIM_GET_NBX(struct OCP_QP_DIM *dim, int stage, int *value)
	{
	*value = dim->nbx[stage];
	return;
	}



void OCP_QP_DIM_GET_NBU(struct OCP_QP_DIM *dim, int stage, int *value)
	{
	*value = dim->nbu[stage];
	return;
	}



void OCP_QP_DIM_GET_NG(struct OCP_QP_DIM *dim, int stage, int *value)
	{
	*value = dim->ng[stage];
	return;
	}


void OCP_QP_DIM_GET_NS(struct OCP_QP_DIM *dim, int stage, int *value)
	{
	*value = dim->ns[stage];
	return;
	}



void OCP_QP_DIM_GET_NSBX(struct OCP_QP_DIM *dim, int stage, int *value)
	{
	*value = dim->nsbx[stage];
	return;
	}



void OCP_QP_DIM_GET_NSBU(struct OCP_QP_DIM *dim, int stage, int *value)
	{
	*value = dim->nsbu[stage];
	return;
	}



void OCP_QP_DIM_GET_NSG(struct OCP_QP_DIM *dim, int stage, int *value)
	{
	*value = dim->nsg[stage];
	return;
	}



void OCP_QP_DIM_GET_NBXE(struct OCP_QP_DIM *dim, int stage, int *value)
	{
	*value = dim->nbxe[stage];
	return;
	}



void OCP_QP_DIM_GET_NBUE(struct OCP_QP_DIM *dim, int stage, int *value)
	{
	*value = dim->nbue[stage];
	return;
	}



void OCP_QP_DIM_GET_NGE(struct OCP_QP_DIM *dim, int stage, int *value)
	{
	*value = dim->nge[stage];
	return;
	}
