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



hpipm_size_t MEMSIZE_CORE_QP_IPM(int nv, int ne, int nc)
	{

	hpipm_size_t size;

	int nv0 = nv;
	int ne0 = ne;
	int nc0 = nc;
// if target avx
// nv0 = ...

	size = 0;

	size += 3*nv0*sizeof(REAL); // v_bkp dv res_g
	size += 3*ne0*sizeof(REAL); // pi_bkp dpi res_b
	size += 10*nc0*sizeof(REAL); // lam_bkp t_bkp dlam dt res_d res_m res_m_bkp t_inv Gamma gamma

	size = (size+63)/64*64; // make multiple of cache line size

	return size;

	}



void CREATE_CORE_QP_IPM(int nv, int ne, int nc, struct CORE_QP_IPM_WORKSPACE *workspace, void *mem)
	{

	workspace->nv = nv;
	workspace->ne = ne;
	workspace->nc = nc;

	int nv0 = nv;
	int ne0 = ne;
	int nc0 = nc;
// if target avx NO!!!!
// nv0 = ...

	REAL *d_ptr = (REAL *) mem;

	workspace->t_inv = d_ptr; // t_inv
	d_ptr += nc0;

	workspace->v_bkp = d_ptr; // v_bkp
	d_ptr += nv0;

	workspace->pi_bkp = d_ptr; // pi_bkp
	d_ptr += ne0;

	workspace->lam_bkp = d_ptr; // lam_bkp
	d_ptr += nc0;

	workspace->t_bkp = d_ptr; // t_bkp
	d_ptr += nc0;

	workspace->dv = d_ptr; // dv
	d_ptr += nv0;

	workspace->dpi = d_ptr; // dpi
	d_ptr += ne0;

	workspace->dlam = d_ptr; // dlam
	d_ptr += nc0;

	workspace->dt = d_ptr; // dt
	d_ptr += nc0;

	workspace->res_g = d_ptr; // res_g
	d_ptr += nv0;

	workspace->res_b = d_ptr; // res_b
	d_ptr += ne0;

	workspace->res_d = d_ptr; // res_d
	d_ptr += nc0;

	workspace->res_m = d_ptr; // res_m
	d_ptr += nc0;

	workspace->res_m_bkp = d_ptr; // res_m_bkp
	d_ptr += nc0;

	workspace->Gamma = d_ptr; // Gamma
	d_ptr += nc0;

	workspace->gamma = d_ptr; // gamma
	d_ptr += nc0;

	// by default no constr masking
	workspace->nc_mask = nc;

	if(nc>0)
		{
		workspace->nc_inv = 1.0/nc;
		workspace->nc_mask_inv = workspace->nc_inv;
		}
	else
		{
		workspace->nc_inv = 0.0;
		workspace->nc_mask_inv = 0.0;
		}


	workspace->lam_min = 0.0;
	workspace->t_min = 0.0;
	workspace->t_min_inv = 1e30;
	workspace->tau_min = 0.0;
	workspace->split_step = 0;
	workspace->t_lam_min = 2;


	workspace->memsize = MEMSIZE_CORE_QP_IPM(nv, ne, nc);


	char * c_ptr = (char *) d_ptr;


	#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + workspace->memsize)
		{
		printf("\nCreate_core_qp_ipm: outsize memory bounds!\n\n");
		exit(1);
		}
#endif


return;

	}


