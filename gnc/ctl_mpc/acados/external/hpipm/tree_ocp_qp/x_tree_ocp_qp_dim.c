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



hpipm_size_t TREE_OCP_QP_DIM_STRSIZE()
	{
	return sizeof(struct TREE_OCP_QP_DIM);
	}



hpipm_size_t TREE_OCP_QP_DIM_MEMSIZE(int Nn)
	{

	hpipm_size_t size = 0;

	size += 10*Nn*sizeof(int);

	size = (size+8-1)/8*8;

	return size;

	}



void TREE_OCP_QP_DIM_CREATE(int Nn, struct TREE_OCP_QP_DIM *dim, void *memory)
	{

	// loop index
	int ii;

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = TREE_OCP_QP_DIM_MEMSIZE(Nn);
	hpipm_zero_memset(memsize, memory);

	char *c_ptr = memory;

	// nx
	dim->nx = (int *) c_ptr;
	c_ptr += Nn*sizeof(int);
	for(ii=0; ii<Nn; ii++)
		dim->nx[ii] = 0;
	// nu
	dim->nu = (int *) c_ptr;
	c_ptr += Nn*sizeof(int);
	for(ii=0; ii<Nn; ii++)
		dim->nu[ii] = 0;
	// nb
	dim->nb = (int *) c_ptr;
	c_ptr += Nn*sizeof(int);
	for(ii=0; ii<Nn; ii++)
		dim->nb[ii] = 0;
	// nbx
	dim->nbx = (int *) c_ptr;
	c_ptr += Nn*sizeof(int);
	for(ii=0; ii<Nn; ii++)
		dim->nbx[ii] = 0;
	// nbu
	dim->nbu = (int *) c_ptr;
	c_ptr += Nn*sizeof(int);
	for(ii=0; ii<Nn; ii++)
		dim->nbu[ii] = 0;
	// ng
	dim->ng = (int *) c_ptr;
	c_ptr += Nn*sizeof(int);
	for(ii=0; ii<Nn; ii++)
		dim->ng[ii] = 0;
	// ns
	dim->ns = (int *) c_ptr;
	c_ptr += Nn*sizeof(int);
	for(ii=0; ii<Nn; ii++)
		dim->ns[ii] = 0;
	// nsbx
	dim->nsbx = (int *) c_ptr;
	c_ptr += Nn*sizeof(int);
	for(ii=0; ii<Nn; ii++)
		dim->nsbx[ii] = 0;
	// nsbu
	dim->nsbu = (int *) c_ptr;
	c_ptr += Nn*sizeof(int);
	for(ii=0; ii<Nn; ii++)
		dim->nsbu[ii] = 0;
	// nsg
	dim->nsg = (int *) c_ptr;
	c_ptr += Nn*sizeof(int);
	for(ii=0; ii<Nn; ii++)
		dim->nsg[ii] = 0;

	// Nn
	dim->Nn = Nn;

	dim->memsize = TREE_OCP_QP_DIM_MEMSIZE(Nn);

	return;

	}


void TREE_OCP_QP_DIM_SET_ALL(struct tree *ttree, int *nx, int *nu, int *nbx, int *nbu, int *ng, int *nsbx, int *nsbu, int *nsg, struct TREE_OCP_QP_DIM *dim)
	{

	// loop index
	int ii;

	// tree
	dim->ttree = ttree;

	// Nn
	int Nn = ttree->Nn;
	dim->Nn = ttree->Nn;

	// copy qp dim
	for(ii=0; ii<Nn; ii++)
		dim->nx[ii] = nx[ii];
	for(ii=0; ii<Nn; ii++)
		dim->nu[ii] = nu[ii];
	for(ii=0; ii<Nn; ii++)
		dim->nb[ii] = nbx[ii]+nbu[ii];
	for(ii=0; ii<Nn; ii++)
		dim->nbx[ii] = nbx[ii];
	for(ii=0; ii<Nn; ii++)
		dim->nbu[ii] = nbu[ii];
	for(ii=0; ii<Nn; ii++)
		dim->ng[ii] = ng[ii];
	for(ii=0; ii<Nn; ii++)
		dim->ns[ii] = nsbx[ii]+nsbu[ii]+nsg[ii];
	for(ii=0; ii<Nn; ii++)
		dim->nsbx[ii] = nsbx[ii];
	for(ii=0; ii<Nn; ii++)
		dim->nsbu[ii] = nsbu[ii];
	for(ii=0; ii<Nn; ii++)
		dim->nsg[ii] = nsg[ii];

	return;

	}



void TREE_OCP_QP_DIM_SET_TREE(struct tree *ttree, struct TREE_OCP_QP_DIM *dim)
	{
	dim->ttree = ttree;
	return;
	}



void TREE_OCP_QP_DIM_SET(char *field_name, int node, int value, struct TREE_OCP_QP_DIM *dim)
	{
	if(hpipm_strcmp(field_name, "nx"))
		{ 
		TREE_OCP_QP_DIM_SET_NX(node, value, dim);
		}
	else if(hpipm_strcmp(field_name, "nu"))
		{ 
		TREE_OCP_QP_DIM_SET_NU(node, value, dim);
		}
	else if(hpipm_strcmp(field_name, "nbx"))
		{
		TREE_OCP_QP_DIM_SET_NBX(node, value, dim);
		}
	else if(hpipm_strcmp(field_name, "nbu"))
		{
		TREE_OCP_QP_DIM_SET_NBU(node, value, dim);
		}
	else if(hpipm_strcmp(field_name, "ng"))
		{
		TREE_OCP_QP_DIM_SET_NG(node, value, dim);
		}
	else if(hpipm_strcmp(field_name, "ns"))
		{
		TREE_OCP_QP_DIM_SET_NS(node, value, dim);
		}
	else if(hpipm_strcmp(field_name, "nsbx"))
		{
		TREE_OCP_QP_DIM_SET_NSBX(node, value, dim);
		}
	else if(hpipm_strcmp(field_name, "nsbu"))
		{
		TREE_OCP_QP_DIM_SET_NSBU(node, value, dim);
		}
	else if(hpipm_strcmp(field_name, "nsg"))
		{
		TREE_OCP_QP_DIM_SET_NSG(node, value, dim);
		}
//	else if(hpipm_strcmp(field_name, "nbxe"))
//		{
//		TREE_OCP_QP_DIM_SET_NBXE(node, value, dim);
//		}
//	else if(hpipm_strcmp(field_name, "nbue"))
//		{
//		TREE_OCP_QP_DIM_SET_NBUE(node, value, dim);
//		}
//	else if(hpipm_strcmp(field_name, "nge"))
//		{
//		TREE_OCP_QP_DIM_SET_NGE(node, value, dim);
//		}
	else 
		{
		printf("error: TREE_OCP_QP_DIM_SET: wrong field %s\n", field_name);
		exit(1);
		}
	return;
	}



void TREE_OCP_QP_DIM_SET_NX(int node, int value, struct TREE_OCP_QP_DIM *dim)
	{
	dim->nx[node] = value;
	return;
	}



void TREE_OCP_QP_DIM_SET_NU(int node, int value, struct TREE_OCP_QP_DIM *dim)
	{
	dim->nu[node] = value;
	return;
	}



void TREE_OCP_QP_DIM_SET_NBX(int node, int value, struct TREE_OCP_QP_DIM *dim)
	{
	dim->nbx[node] = value;
	dim->nb[node] = dim->nbx[node] + dim->nbu[node];
	return;
	}



void TREE_OCP_QP_DIM_SET_NBU(int node, int value, struct TREE_OCP_QP_DIM *dim)
	{
	dim->nbu[node] = value;
	dim->nb[node] = dim->nbx[node] + dim->nbu[node];
	return;
	}



void TREE_OCP_QP_DIM_SET_NG(int node, int value, struct TREE_OCP_QP_DIM *dim)
	{
	dim->ng[node] = value;
	return;
	}



void TREE_OCP_QP_DIM_SET_NS(int node, int value, struct TREE_OCP_QP_DIM *dim)
	{
	dim->ns[node] = value;
	return;
	}



void TREE_OCP_QP_DIM_SET_NSBX(int node, int value, struct TREE_OCP_QP_DIM *dim)
	{
	dim->nsbx[node] = value;
	dim->ns[node] = dim->nsbx[node] + dim->nsbu[node] + dim->nsg[node];
	return;
	}



void TREE_OCP_QP_DIM_SET_NSBU(int node, int value, struct TREE_OCP_QP_DIM *dim)
	{
	dim->nsbu[node] = value;
	dim->ns[node] = dim->nsbx[node] + dim->nsbu[node] + dim->nsg[node];
	return;
	}



void TREE_OCP_QP_DIM_SET_NSG(int node, int value, struct TREE_OCP_QP_DIM *dim)
	{
	dim->nsg[node] = value;
	dim->ns[node] = dim->nsbx[node] + dim->nsbu[node] + dim->nsg[node];
	return;
	}



//void TREE_OCP_QP_DIM_SET_NBXE(int node, int value, struct TREE_OCP_QP_DIM *dim)
//	{
//	dim->nbxe[node] = value;
//	return;
//	}



//void TREE_OCP_QP_DIM_SET_NBUE(int node, int value, struct TREE_OCP_QP_DIM *dim)
//	{
//	dim->nbue[node] = value;
//	return;
//	}



//void TREE_OCP_QP_DIM_SET_NGE(int node, int value, struct TREE_OCP_QP_DIM *dim)
//	{
//	dim->nge[node] = value;
//	return;
//	}




