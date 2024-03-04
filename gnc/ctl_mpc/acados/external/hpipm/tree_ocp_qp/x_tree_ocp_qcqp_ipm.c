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



hpipm_size_t TREE_OCP_QCQP_IPM_ARG_STRSIZE()
	{
	return sizeof(struct TREE_OCP_QCQP_IPM_ARG);
	}



hpipm_size_t TREE_OCP_QCQP_IPM_ARG_MEMSIZE(struct TREE_OCP_QCQP_DIM *dim)
	{

	hpipm_size_t size = 0;

	size += 1*sizeof(struct TREE_OCP_QP_IPM_ARG);
	size += 1*TREE_OCP_QP_IPM_ARG_MEMSIZE(dim->qp_dim);

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 1*64; // align once to typical cache line size

	return size;


	}



void TREE_OCP_QCQP_IPM_ARG_CREATE(struct TREE_OCP_QCQP_DIM *dim, struct TREE_OCP_QCQP_IPM_ARG *arg, void *mem)
	{

	// loop index
	int ii;

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = TREE_OCP_QCQP_IPM_ARG_MEMSIZE(dim);
	hpipm_zero_memset(memsize, mem);

	// qp_dim struct
	struct TREE_OCP_QP_IPM_ARG *arg_ptr = mem;

	arg->qp_arg = arg_ptr;
	arg_ptr += 1;

	// align to typical cache line size
	hpipm_size_t s_ptr = (hpipm_size_t) arg_ptr;
	s_ptr = (s_ptr+63)/64*64;

	// void
	char *c_ptr = (char *) s_ptr;

	TREE_OCP_QP_IPM_ARG_CREATE(dim->qp_dim, arg->qp_arg, c_ptr);
	c_ptr += arg->qp_arg->memsize;


	arg->memsize = memsize; //TREE_OCP_QCQP_IPM_ARG_MEMSIZE(dim);

#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + arg->memsize)
		{
		printf("\nerror: TREE_OCP_QCQP_IPM_ARG_CREATE: outside memory bounds!\n\n");
		exit(1);
		}
#endif

	return;

	}



void TREE_OCP_QCQP_IPM_ARG_SET_DEFAULT(enum HPIPM_MODE mode, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{

	TREE_OCP_QP_IPM_ARG_SET_DEFAULT(mode, arg->qp_arg);

	REAL mu0, alpha_min, res_g_max, res_b_max, res_d_max, res_m_max, reg_prim, lam_min, t_min;
	int iter_max, stat_max, pred_corr, cond_pred_corr, itref_pred_max, itref_corr_max, lq_fact, warm_start, abs_form, comp_res_exit, comp_res_pred, square_root_alg, comp_dual_sol_eq, split_step, t_lam_min;

	if(mode==SPEED_ABS)
		{
		mu0 = 1e1;
		alpha_min = 1e-12;
		res_g_max = 1e0; // not used
		res_b_max = 1e0; // not used
		res_d_max = 1e0; // not used
		res_m_max = 1e-8;
		iter_max = 15;
		stat_max = 15;
		pred_corr = 1;
		cond_pred_corr = 1;
		itref_pred_max = 0; // not used
		itref_corr_max = 0; // not used
		reg_prim = 1e-15;
//		square_root_alg = 1;
		lq_fact = 0; // not used
		lam_min = 1e-16;
		t_min = 1e-16;
		warm_start = 0;
		abs_form = 1;
		comp_dual_sol_eq = 0;
		comp_res_exit = 0;
//		comp_res_pred = 0;
		split_step = 1;
		t_lam_min = 2;
		}
	else if(mode==SPEED)
		{
		mu0 = 1e1;
		alpha_min = 1e-12;
		res_g_max = 1e-6;
		res_b_max = 1e-8;
		res_d_max = 1e-8;
		res_m_max = 1e-8;
		iter_max = 15;
		stat_max = 15;
		pred_corr = 1;
		cond_pred_corr = 1;
		itref_pred_max = 0;
		itref_corr_max = 0;
		reg_prim = 1e-15;
//		square_root_alg = 1;
		lq_fact = 0;
		lam_min = 1e-16;
		t_min = 1e-16;
		warm_start = 0;
		abs_form = 0;
		comp_dual_sol_eq = 1;
		comp_res_exit = 1;
//		comp_res_pred = 1;
		split_step = 1;
		t_lam_min = 2;
		}
	else if(mode==BALANCE)
		{
		mu0 = 1e1;
		alpha_min = 1e-12;
		res_g_max = 1e-6;
		res_b_max = 1e-8;
		res_d_max = 1e-8;
		res_m_max = 1e-8;
		iter_max = 30;
		stat_max = 30;
		pred_corr = 1;
		cond_pred_corr = 1;
		itref_pred_max = 0;
		itref_corr_max = 2;
		reg_prim = 1e-15;
//		square_root_alg = 1;
		lq_fact = 1;
		lam_min = 1e-16;
		t_min = 1e-16;
		warm_start = 0;
		abs_form = 0;
		comp_dual_sol_eq = 1;
		comp_res_exit = 1;
//		comp_res_pred = 1;
		split_step = 0;
		t_lam_min = 2;
		}
	else if(mode==ROBUST)
		{
		mu0 = 1e2;
		alpha_min = 1e-12;
		res_g_max = 1e-6;
		res_b_max = 1e-8;
		res_d_max = 1e-8;
		res_m_max = 1e-8;
		iter_max = 100;
		stat_max = 100;
		pred_corr = 1;
		cond_pred_corr = 1;
		itref_pred_max = 0;
		itref_corr_max = 4;
		reg_prim = 1e-15;
//		square_root_alg = 1;
		lq_fact = 2;
		lam_min = 1e-16;
		t_min = 1e-16;
		warm_start = 0;
		abs_form = 0;
		comp_dual_sol_eq = 1;
		comp_res_exit = 1;
//		comp_res_pred = 1;
		split_step = 0;
		t_lam_min = 2;
		}
	else
		{
		printf("\nerror: TREE_OCP_QCQP_IPM_ARG_SET_DEFAULT: wrong set default mode\n");
		exit(1);
		}

	// use individual setters when available
	TREE_OCP_QCQP_IPM_ARG_SET_MU0(&mu0, arg);
	TREE_OCP_QCQP_IPM_ARG_SET_ALPHA_MIN(&alpha_min, arg);
	TREE_OCP_QCQP_IPM_ARG_SET_TOL_STAT(&res_g_max, arg);
	TREE_OCP_QCQP_IPM_ARG_SET_TOL_EQ(&res_b_max, arg);
	TREE_OCP_QCQP_IPM_ARG_SET_TOL_INEQ(&res_d_max, arg);
	TREE_OCP_QCQP_IPM_ARG_SET_TOL_COMP(&res_m_max, arg);
	TREE_OCP_QCQP_IPM_ARG_SET_ITER_MAX(&iter_max, arg);
	arg->stat_max = stat_max;
	TREE_OCP_QCQP_IPM_ARG_SET_PRED_CORR(&pred_corr, arg);
	TREE_OCP_QCQP_IPM_ARG_SET_COND_PRED_CORR(&cond_pred_corr, arg);
//	TREE_OCP_QCQP_IPM_ARG_SET_RIC_ALG(&square_root_alg, arg);
	arg->itref_pred_max = itref_pred_max;
	arg->itref_corr_max = itref_corr_max;
	TREE_OCP_QCQP_IPM_ARG_SET_REG_PRIM(&reg_prim, arg);
	arg->lq_fact = lq_fact;
	TREE_OCP_QCQP_IPM_ARG_SET_LAM_MIN(&lam_min, arg);
	TREE_OCP_QCQP_IPM_ARG_SET_T_MIN(&t_min, arg);
	TREE_OCP_QCQP_IPM_ARG_SET_WARM_START(&warm_start, arg);
	arg->abs_form = abs_form;
//	TREE_OCP_QCQP_IPM_ARG_SET_COMP_RES_PRED(&comp_res_pred, arg);
	TREE_OCP_QCQP_IPM_ARG_SET_COMP_RES_EXIT(&comp_res_pred, arg);
	TREE_OCP_QCQP_IPM_ARG_SET_SPLIT_STEP(&split_step, arg);
	TREE_OCP_QCQP_IPM_ARG_SET_T_LAM_MIN(&t_lam_min, arg);
	arg->mode = mode;

	return;

	}



void TREE_OCP_QCQP_IPM_ARG_SET(char *field, void *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	if(hpipm_strcmp(field, "iter_max")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_ITER_MAX(value, arg);
		}
	else if(hpipm_strcmp(field, "alpha_min")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_ALPHA_MIN(value, arg);
		}
	else if(hpipm_strcmp(field, "mu0")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_MU0(value, arg);
		}
	else if(hpipm_strcmp(field, "tol_stat")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_TOL_STAT(value, arg);
		}
	else if(hpipm_strcmp(field, "tol_eq")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_TOL_EQ(value, arg);
		}
	else if(hpipm_strcmp(field, "tol_ineq")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_TOL_INEQ(value, arg);
		}
	else if(hpipm_strcmp(field, "tol_comp")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_TOL_COMP(value, arg);
		}
	else if(hpipm_strcmp(field, "reg_prim")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_REG_PRIM(value, arg);
		}
	else if(hpipm_strcmp(field, "warm_start")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_WARM_START(value, arg);
		}
	else if(hpipm_strcmp(field, "pred_corr")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_PRED_CORR(value, arg);
		}
	else if(hpipm_strcmp(field, "cond_pred_corr")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_COND_PRED_CORR(value, arg);
		}
//	else if(hpipm_strcmp(field, "ric_alg")) 
//		{
//		TREE_OCP_QCQP_IPM_ARG_SET_RIC_ALG(value, arg);
//		}
	else if(hpipm_strcmp(field, "comp_res_exit")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_COMP_RES_EXIT(value, arg);
		}
//	else if(hpipm_strcmp(field, "comp_res_pred")) 
//		{
//		TREE_OCP_QCQP_IPM_ARG_SET_COMP_RES_PRED(value, arg);
//		}
	else if(hpipm_strcmp(field, "lam_min")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_LAM_MIN(value, arg);
		}
	else if(hpipm_strcmp(field, "t_min")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_T_MIN(value, arg);
		}
	else if(hpipm_strcmp(field, "split_step")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_SPLIT_STEP(value, arg);
		}
	else if(hpipm_strcmp(field, "t_lam_min")) 
		{
		TREE_OCP_QCQP_IPM_ARG_SET_T_LAM_MIN(value, arg);
		}
	else
		{
		printf("error: TREE_OCP_QCQP_IPM_ARG_SET: wrong field %s\n", field);
		exit(1);	
		}
	return;
	}



void TREE_OCP_QCQP_IPM_ARG_SET_ITER_MAX(int *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->iter_max = *value;
	TREE_OCP_QP_IPM_ARG_SET_ITER_MAX(value, arg->qp_arg);
	return;
	}



void TREE_OCP_QCQP_IPM_ARG_SET_ALPHA_MIN(REAL *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->alpha_min = *value;
	TREE_OCP_QP_IPM_ARG_SET_ALPHA_MIN(value, arg->qp_arg);
	return;
	}



void TREE_OCP_QCQP_IPM_ARG_SET_MU0(REAL *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->mu0 = *value;
	TREE_OCP_QP_IPM_ARG_SET_MU0(value, arg->qp_arg);
	return;
	}



void TREE_OCP_QCQP_IPM_ARG_SET_TOL_STAT(REAL *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->res_g_max = *value;
	TREE_OCP_QP_IPM_ARG_SET_TOL_STAT(value, arg->qp_arg);
	return;
	}



void TREE_OCP_QCQP_IPM_ARG_SET_TOL_EQ(REAL *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->res_b_max = *value;
	TREE_OCP_QP_IPM_ARG_SET_TOL_STAT(value, arg->qp_arg);
	return;
	}



void TREE_OCP_QCQP_IPM_ARG_SET_TOL_INEQ(REAL *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->res_d_max = *value;
	TREE_OCP_QP_IPM_ARG_SET_TOL_STAT(value, arg->qp_arg);
	return;
	}



void TREE_OCP_QCQP_IPM_ARG_SET_TOL_COMP(REAL *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->res_m_max = *value;
	TREE_OCP_QP_IPM_ARG_SET_TOL_COMP(value, arg->qp_arg);
	return;
	}



void TREE_OCP_QCQP_IPM_ARG_SET_REG_PRIM(REAL *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->reg_prim = *value;
	TREE_OCP_QP_IPM_ARG_SET_REG_PRIM(value, arg->qp_arg);
	return;
	}



void TREE_OCP_QCQP_IPM_ARG_SET_WARM_START(int *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->warm_start = *value;
	TREE_OCP_QP_IPM_ARG_SET_WARM_START(value, arg->qp_arg);
	return;
	}



void TREE_OCP_QCQP_IPM_ARG_SET_PRED_CORR(int *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->pred_corr = *value;
	TREE_OCP_QP_IPM_ARG_SET_PRED_CORR(value, arg->qp_arg);
	return;
	}



void TREE_OCP_QCQP_IPM_ARG_SET_COND_PRED_CORR(int *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->cond_pred_corr = *value;
	TREE_OCP_QP_IPM_ARG_SET_COND_PRED_CORR(value, arg->qp_arg);
	return;
	}



//void TREE_OCP_QCQP_IPM_ARG_SET_RIC_ALG(int *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
//	{
//	arg->square_root_alg = *value;
//	TREE_OCP_QP_IPM_ARG_SET_RIC_ALG(value, arg->qp_arg);
//	return;
//	}



void TREE_OCP_QCQP_IPM_ARG_SET_COMP_RES_EXIT(int *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->comp_res_exit = *value;
	if(*value!=0)
		arg->comp_dual_sol_eq = 1;
	TREE_OCP_QP_IPM_ARG_SET_COMP_RES_EXIT(value, arg->qp_arg);
	return;
	}



//void TREE_OCP_QCQP_IPM_ARG_SET_COMP_RES_PRED(int *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
//	{
//	arg->comp_res_pred = *value;
//	TREE_OCP_QP_IPM_ARG_SET_COMP_RES_PRED(value, arg->qp_arg);
//	return;
//	}



void TREE_OCP_QCQP_IPM_ARG_SET_LAM_MIN(REAL *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->lam_min = *value;
	TREE_OCP_QP_IPM_ARG_SET_LAM_MIN(value, arg->qp_arg);
	return;
	}



void TREE_OCP_QCQP_IPM_ARG_SET_T_MIN(REAL *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->t_min = *value;
	TREE_OCP_QP_IPM_ARG_SET_T_MIN(value, arg->qp_arg);
	return;
	}



void TREE_OCP_QCQP_IPM_ARG_SET_SPLIT_STEP(int *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->split_step = *value;
	TREE_OCP_QP_IPM_ARG_SET_SPLIT_STEP(value, arg->qp_arg);
	return;
	}



void TREE_OCP_QCQP_IPM_ARG_SET_T_LAM_MIN(int *value, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{
	arg->t_lam_min = *value;
	TREE_OCP_QP_IPM_ARG_SET_T_LAM_MIN(value, arg->qp_arg);
	return;
	}



hpipm_size_t TREE_OCP_QCQP_IPM_WS_STRSIZE()
	{
	return sizeof(struct TREE_OCP_QCQP_IPM_WS);
	}



hpipm_size_t TREE_OCP_QCQP_IPM_WS_MEMSIZE(struct TREE_OCP_QCQP_DIM *dim, struct TREE_OCP_QCQP_IPM_ARG *arg)
	{

	int Nn = dim->Nn;
	int *nu = dim->nu;
	int *nx = dim->nx;

	int ii;
	
	int nuM = 0;
	int nxM = 0;
	for(ii=0; ii<Nn; ii++)
		{
		nuM = nu[ii]>nuM ? nu[ii] : nuM;
		nxM = nx[ii]>nxM ? nx[ii] : nxM;
		}

	hpipm_size_t size = 0;

	size += 1*sizeof(struct TREE_OCP_QP_IPM_WS);
	size += 1*TREE_OCP_QP_IPM_WS_MEMSIZE(dim->qp_dim, arg->qp_arg);

	size += 1*sizeof(struct TREE_OCP_QCQP_RES_WS); // qcqp_res_ws
	size += 1*TREE_OCP_QCQP_RES_WS_MEMSIZE(dim); // qcqp_res_ws

	size += 1*sizeof(struct TREE_OCP_QCQP_RES); // qcqp_res
	size += 1*TREE_OCP_QCQP_RES_MEMSIZE(dim); // qcqp_res

	size += 1*sizeof(struct TREE_OCP_QP); // qp
	size += 1*TREE_OCP_QP_MEMSIZE(dim->qp_dim); // qp

	size += 1*sizeof(struct TREE_OCP_QP_SOL); // qp_sol
	size += 1*TREE_OCP_QP_SOL_MEMSIZE(dim->qp_dim); // qp_sol

	size += 2*sizeof(struct STRVEC); // tmp_nuxM
	size += 2*SIZE_STRVEC(nuM+nxM); // tmp_nuxM

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 1*64; // align once to typical cache line size

	return size;

	}



void TREE_OCP_QCQP_IPM_WS_CREATE(struct TREE_OCP_QCQP_DIM *dim, struct TREE_OCP_QCQP_IPM_ARG *arg, struct TREE_OCP_QCQP_IPM_WS *workspace, void *mem)
	{

	// loop index
	int ii;

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = TREE_OCP_QCQP_IPM_WS_MEMSIZE(dim, arg);
	hpipm_zero_memset(memsize, mem);

	int Nn = dim->Nn;
	int *nu = dim->nu;
	int *nx = dim->nx;

	int nuM = 0;
	int nxM = 0;
	for(ii=0; ii<Nn; ii++)
		{
		nuM = nu[ii]>nuM ? nu[ii] : nuM;
		nxM = nx[ii]>nxM ? nx[ii] : nxM;
		}


	char *c_ptr = mem;


	// structures
	workspace->qp_ws = (struct TREE_OCP_QP_IPM_WS *) c_ptr;
	c_ptr += sizeof(struct TREE_OCP_QP_IPM_WS);

	workspace->qp = (struct TREE_OCP_QP *) c_ptr;
	c_ptr += sizeof(struct TREE_OCP_QP);

	workspace->qp_sol = (struct TREE_OCP_QP_SOL *) c_ptr;
	c_ptr += sizeof(struct TREE_OCP_QP_SOL);

	workspace->qcqp_res_ws = (struct TREE_OCP_QCQP_RES_WS *) c_ptr;
	c_ptr += sizeof(struct TREE_OCP_QCQP_RES_WS);

	workspace->qcqp_res = (struct TREE_OCP_QCQP_RES *) c_ptr;
	c_ptr += sizeof(struct TREE_OCP_QCQP_RES);


	// vector struct
	struct STRVEC *sv_ptr = (struct STRVEC *) c_ptr;

	workspace->tmp_nuxM = sv_ptr;
	sv_ptr += 2;


	// align to typical cache line size
	hpipm_size_t s_ptr = (hpipm_size_t) sv_ptr;
	s_ptr = (s_ptr+63)/64*64;


	// memory of structures
	c_ptr = (char *) s_ptr;

	TREE_OCP_QP_IPM_WS_CREATE(dim->qp_dim, arg->qp_arg, workspace->qp_ws, c_ptr);
	c_ptr += workspace->qp_ws->memsize;

	TREE_OCP_QP_CREATE(dim->qp_dim, workspace->qp, c_ptr);
	c_ptr += workspace->qp->memsize;

	TREE_OCP_QP_SOL_CREATE(dim->qp_dim, workspace->qp_sol, c_ptr);
	c_ptr += workspace->qp_sol->memsize;

	TREE_OCP_QCQP_RES_WS_CREATE(dim, workspace->qcqp_res_ws, c_ptr);
	c_ptr += workspace->qcqp_res_ws->memsize;

	TREE_OCP_QCQP_RES_CREATE(dim, workspace->qcqp_res, c_ptr);
	c_ptr += workspace->qcqp_res->memsize;

	CREATE_STRVEC(nuM+nxM, workspace->tmp_nuxM+0, c_ptr);
	c_ptr += (workspace->tmp_nuxM+0)->memsize;
	CREATE_STRVEC(nuM+nxM, workspace->tmp_nuxM+1, c_ptr);
	c_ptr += (workspace->tmp_nuxM+1)->memsize;


	//
	workspace->memsize = memsize; //TREE_OCP_QCQP_IPM_WS_MEMSIZE(dim, arg);


#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + workspace->memsize)
		{
		printf("\nCreate_dense_qp_ipm: outside memory bounds!\n\n");
		exit(1);
		}
#endif

	return;

	}



// TODO
void TREE_OCP_QCQP_IPM_GET(char *field, struct TREE_OCP_QCQP_IPM_WS *ws, void *value)
	{
	if(hpipm_strcmp(field, "status"))
		{ 
		TREE_OCP_QCQP_IPM_GET_STATUS(ws, value);
		}
	else if(hpipm_strcmp(field, "iter"))
		{ 
		TREE_OCP_QCQP_IPM_GET_ITER(ws, value);
		}
	else if(hpipm_strcmp(field, "max_res_stat"))
		{ 
		TREE_OCP_QCQP_IPM_GET_MAX_RES_STAT(ws, value);
		}
	else if(hpipm_strcmp(field, "max_res_eq"))
		{ 
		TREE_OCP_QCQP_IPM_GET_MAX_RES_EQ(ws, value);
		}
	else if(hpipm_strcmp(field, "max_res_ineq"))
		{ 
		TREE_OCP_QCQP_IPM_GET_MAX_RES_INEQ(ws, value);
		}
	else if(hpipm_strcmp(field, "max_res_comp"))
		{ 
		TREE_OCP_QCQP_IPM_GET_MAX_RES_COMP(ws, value);
		}
	else if(hpipm_strcmp(field, "stat"))
		{ 
		TREE_OCP_QCQP_IPM_GET_STAT(ws, value);
		}
	else if(hpipm_strcmp(field, "stat_m"))
		{ 
		TREE_OCP_QCQP_IPM_GET_STAT_M(ws, value);
		}
	else 
		{
		printf("error: TREE_OCP_QCQP_IPM_GET: wrong field %s\n", field);
		exit(1);
		}
	return;
	}



void TREE_OCP_QCQP_IPM_GET_STATUS(struct TREE_OCP_QCQP_IPM_WS *ws, int *status)
	{
	*status = ws->status;
	return;
	}



void TREE_OCP_QCQP_IPM_GET_ITER(struct TREE_OCP_QCQP_IPM_WS *ws, int *iter)
	{
	*iter = ws->iter;
	return;
	}



void TREE_OCP_QCQP_IPM_GET_MAX_RES_STAT(struct TREE_OCP_QCQP_IPM_WS *ws, REAL *res_stat)
	{
	*res_stat = ws->qcqp_res->res_max[0];
	return;
	}



void TREE_OCP_QCQP_IPM_GET_MAX_RES_EQ(struct TREE_OCP_QCQP_IPM_WS *ws, REAL *res_eq)
	{
	*res_eq = ws->qcqp_res->res_max[1];
	return;
	}



void TREE_OCP_QCQP_IPM_GET_MAX_RES_INEQ(struct TREE_OCP_QCQP_IPM_WS *ws, REAL *res_ineq)
	{
	*res_ineq = ws->qcqp_res->res_max[2];
	return;
	}



void TREE_OCP_QCQP_IPM_GET_MAX_RES_COMP(struct TREE_OCP_QCQP_IPM_WS *ws, REAL *res_comp)
	{
	*res_comp = ws->qcqp_res->res_max[3];
	return;
	}



void TREE_OCP_QCQP_IPM_GET_STAT(struct TREE_OCP_QCQP_IPM_WS *ws, REAL **stat)
	{
	TREE_OCP_QP_IPM_GET_STAT(ws->qp_ws, stat);
	}



void TREE_OCP_QCQP_IPM_GET_STAT_M(struct TREE_OCP_QCQP_IPM_WS *ws, int *stat_m)
	{
	TREE_OCP_QP_IPM_GET_STAT_M(ws->qp_ws, stat_m);
	}



// with warm_start==2 also init dual variables
void TREE_OCP_QCQP_INIT_VAR(struct TREE_OCP_QCQP *qp, struct TREE_OCP_QCQP_SOL *qp_sol, struct TREE_OCP_QCQP_IPM_ARG *arg, struct TREE_OCP_QCQP_IPM_WS *ws)
	{

//	struct CORE_QCQP_IPM_WORKSPACE *cws = ws->core_workspace;
	
	// loop index
	int ii, jj;

	//
	int Nn = qp->dim->Nn;
	int *nx = qp->dim->nx;
	int *nu = qp->dim->nu;
	int *nb = qp->dim->nb;
	int *ng = qp->dim->ng;
	int *nq = qp->dim->nq;
	int *ns = qp->dim->ns;

	REAL mu0 = arg->mu0;
	REAL tmp;

	//
	REAL *ux, *s, *pi, *d_lb, *d_ub, *d_lg, *d_ug, *d_ls, *d_uq, *lam_lb, *lam_ub, *lam_lg, *lam_ug, *lam_ls, *lam_lq, *lam_uq, *t_lb, *t_ub, *t_lg, *t_ug, *t_ls, *t_lq, *t_uq;
	int *idxb, *idxs_rev;
	int idx;

	REAL thr0 = 1e-1;


	// primal and dual variables
	if(arg->warm_start==2)
		{

		thr0 = 1e-1;

		for(ii=0; ii<Nn; ii++)
			{
			lam_lb = qp_sol->lam[ii].pa+0;
			t_lb = qp_sol->t[ii].pa+0;

			for(jj=0; jj<2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii]; jj++)
				{
				if(lam_lb[jj]<thr0)
					lam_lb[jj] = thr0;
				if(t_lb[jj]<thr0)
					t_lb[jj] = thr0;
				}
			}

		return;
		}


	// ux
	if(arg->warm_start==0)
		{

		// cold start
		for(ii=0; ii<Nn; ii++)
			{
			ux = qp_sol->ux[ii].pa;
			for(jj=0; jj<nu[ii]+nx[ii]+2*ns[ii]; jj++)
				{
				ux[jj] = 0.0;
				}
			}

		}
//	else
//		{
//
//		// warm start (keep u and x in solution)
//		for(ii=0; ii<Nn; ii++)
//			{
//			ux = qp_sol->ux[ii].pa;
//			for(jj=nu[ii]+nx[ii]; jj<nu[ii]+nx[ii]+2*ns[ii]; jj++)
//				{
//				ux[jj] = 0.0;
//				}
//			}
//
//		}
	
	// pi
	for(ii=0; ii<Nn-1; ii++)
		{
		idx = ii+1;
		pi = qp_sol->pi[ii].pa;
		for(jj=0; jj<nx[idx]; jj++)
			{
			pi[jj] = 0.0;
			}
		}

#if 1 // old version



	// box constraints
	for(ii=0; ii<Nn; ii++)
		{
		ux = qp_sol->ux[ii].pa;
		d_lb = qp->d[ii].pa+0;
		d_ub = qp->d[ii].pa+nb[ii]+ng[ii]+nq[ii];
		lam_lb = qp_sol->lam[ii].pa+0;
		lam_ub = qp_sol->lam[ii].pa+nb[ii]+ng[ii]+nq[ii];
		t_lb = qp_sol->t[ii].pa+0;
		t_ub = qp_sol->t[ii].pa+nb[ii]+ng[ii]+nq[ii];
		idxb = qp->idxb[ii];
		for(jj=0; jj<nb[ii]; jj++)
			{
#if 1
			t_lb[jj] = - d_lb[jj] + ux[idxb[jj]];
			t_ub[jj] = - d_ub[jj] - ux[idxb[jj]];
//			printf("\n%d %f %f\n", jj, t_lb[jj], t_ub[jj]);
			if(t_lb[jj]<thr0)
				{
				if(t_ub[jj]<thr0)
					{
//					ux[idxb[jj]] = 0.5*(d_lb[jj] + d_ub[jj]);
					ux[idxb[jj]] = 0.5*(d_lb[jj] - d_ub[jj]);
					t_lb[jj] = thr0;
					t_ub[jj] = thr0;
					}
				else
					{
					t_lb[jj] = thr0;
					ux[idxb[jj]] = d_lb[jj] + thr0;
					}
				}
			else if(t_ub[jj]<thr0)
				{
				t_ub[jj] = thr0;
				ux[idxb[jj]] = - d_ub[jj] - thr0;
				}
#else
			t_lb[jj] = 1.0;
			t_ub[jj] = 1.0;
#endif
			lam_lb[jj] = mu0/t_lb[jj];
			lam_ub[jj] = mu0/t_ub[jj];
			}
//		blasfeo_print_tran_dvec(nb[ii], qp->d+ii, 0);
//		blasfeo_print_tran_dvec(nb[ii], qp->d+ii, nb[ii]+ng[ii]);
//		blasfeo_print_tran_dvec(nu[ii]+nx[ii], qp_sol->ux+ii, 0);
//		blasfeo_print_tran_dvec(nb[ii], qp_sol->t+ii, 0);
//		blasfeo_print_tran_dvec(nb[ii], qp_sol->t+ii, nb[ii]+ng[ii]);
//		exit(1);
		}
	// general constraints
	for(ii=0; ii<Nn; ii++)
		{
		t_lg = qp_sol->t[ii].pa+nb[ii];
		t_ug = qp_sol->t[ii].pa+2*nb[ii]+ng[ii]+nq[ii];
		lam_lg = qp_sol->lam[ii].pa+nb[ii];
		lam_ug = qp_sol->lam[ii].pa+2*nb[ii]+ng[ii]+nq[ii];
		d_lg = qp->d[ii].pa+nb[ii];
		d_ug = qp->d[ii].pa+2*nb[ii]+ng[ii]+nq[ii];
		ux = qp_sol->ux[ii].pa;
		GEMV_T(nu[ii]+nx[ii], ng[ii], 1.0, qp->DCt+ii, 0, 0, qp_sol->ux+ii, 0, 0.0, qp_sol->t+ii, nb[ii], qp_sol->t+ii, nb[ii]);
		for(jj=0; jj<ng[ii]; jj++)
			{
#if 1
			t_ug[jj] = - t_lg[jj];
			t_lg[jj] -= d_lg[jj];
			t_ug[jj] -= d_ug[jj];
//			t_lg[jj] = fmax(thr0, t_lg[jj]);
//			t_ug[jj] = fmax(thr0, t_ug[jj]);
			t_lg[jj] = thr0>t_lg[jj] ? thr0 : t_lg[jj];
			t_ug[jj] = thr0>t_ug[jj] ? thr0 : t_ug[jj];
#else
			t_lg[jj] = 1.0;
			t_ug[jj] = 1.0;
#endif
			lam_lg[jj] = mu0/t_lg[jj];
			lam_ug[jj] = mu0/t_ug[jj];
			}
		}

	// soft constraints
	for(ii=0; ii<Nn; ii++)
		{
		lam_ls = qp_sol->lam[ii].pa+2*nb[ii]+2*ng[ii]+2*nq[ii];
		t_ls = qp_sol->t[ii].pa+2*nb[ii]+2*ng[ii]+2*nq[ii];
		for(jj=0; jj<2*ns[ii]; jj++)
			{
			t_ls[jj] = 1.0; // thr0;
//			t_ls[jj] = sqrt(mu0); // thr0;
			lam_ls[jj] = mu0/t_ls[jj];
			}
		}

	//  quadratic constraints
	REAL sqrt_mu0 = sqrt(mu0);
	sqrt_mu0 = thr0>sqrt_mu0 ? thr0 : sqrt_mu0;
	REAL mu0_div_sqrt_mu0 = mu0 / sqrt_mu0;

	for(ii=0; ii<Nn; ii++)
		{
		lam_lq = qp_sol->lam[ii].pa+nb[ii]+ng[ii];
		t_lq = qp_sol->t[ii].pa+nb[ii]+ng[ii];
		lam_uq = qp_sol->lam[ii].pa+2*nb[ii]+2*ng[ii]+nq[ii];
		t_uq = qp_sol->t[ii].pa+2*nb[ii]+2*ng[ii]+nq[ii];
		d_uq = qp->d[ii].pa+2*nb[ii]+2*ng[ii]+nq[ii];
		for(jj=0; jj<nq[ii]; jj++)
			{
			// disregard lower
			lam_lq[jj] = 0.0;
			t_lq[jj]   = 1.0;
			// upper
#if 1
			t_uq[jj]   = sqrt_mu0;
			lam_uq[jj] = mu0_div_sqrt_mu0;
#else
	//		t[2*nb+2*ng+nq+jj] = 1.0; // thr0;
			COLEX(nu[ii]+nx[ii], qp->DCt+ii, 0, ng[ii]+jj, ws->tmp_nuxM, 0);
			SYMV_L(nu[ii]+nx[ii], 0.5, &qp->Hq[ii][jj], 0, 0, qp_sol->ux+ii, 0, 1.0, ws->tmp_nuxM, 0, ws->tmp_nuxM, 0);
			tmp = DOT(nu[ii]+nx[ii], ws->tmp_nuxM, 0, qp_sol->ux+ii, 0);
			tmp = - d_uq[jj] - tmp;
			t_uq[jj] = thr0>tmp ? thr0 : tmp;
			lam_uq[jj]  = mu0/t_uq[jj];
#endif
			}
		}



#else // new version



	for(ii=0; ii<Nn; ii++)
		{

//		printf("\nii = %d\n", ii);

		ux = qp_sol->ux[ii].pa;
		s = qp_sol->ux[ii].pa+nu[ii]+nx[ii];
		d_lb = qp->d[ii].pa+0;
		d_ub = qp->d[ii].pa+nb[ii]+ng[ii]+nq[ii];
		d_lg = qp->d[ii].pa+nb[ii];
		d_ug = qp->d[ii].pa+2*nb[ii]+ng[ii]+nq[ii];
		d_ls = qp->d[ii].pa+2*nb[ii]+2*ng[ii]+2*nq[ii];
		lam_lb = qp_sol->lam[ii].pa+0;
		lam_ub = qp_sol->lam[ii].pa+nb[ii]+ng[ii]+nq[ii];
		lam_lg = qp_sol->lam[ii].pa+nb[ii];
		lam_ug = qp_sol->lam[ii].pa+2*nb[ii]+ng[ii]+nq[ii];
		lam_ls = qp_sol->lam[ii].pa+2*nb[ii]+2*ng[ii]+2*nq[ii];
		t_lb = qp_sol->t[ii].pa+0;
		t_ub = qp_sol->t[ii].pa+nb[ii]+ng[ii]+nq[ii];
		t_lg = qp_sol->t[ii].pa+nb[ii];
		t_ug = qp_sol->t[ii].pa+2*nb[ii]+ng[ii]+nq[ii];
		t_ls = qp_sol->t[ii].pa+2*nb[ii]+2*ng[ii]+2*nq[ii];
		idxb = qp->idxb[ii];
		idxs_rev = qp->idxs_rev[ii];

		// lower bound on slacks
		AXPY(2*ns[ii], -1.0, qp->d+ii, 2*nb[ii]+2*ng[ii]+2*nq[ii], qp_sol->ux+ii, nu[ii]+nx[ii], qp_sol->t+ii, 2*nb[ii]+2*ng[ii]+2*nq[ii]);
		for(jj=0; jj<2*ns[ii]; jj++)
			{
#if 1
			if(t_ls[jj]<thr0)
				{
				t_ls[jj] = thr0; //1.0;
				s[jj] = d_ls[jj] + t_ls[jj];
				}
#else
			t_ls[jj] = 1.0;
//			t_ls[jj] = sqrt(mu0);
#endif
			}
//		blasfeo_print_tran_dvec(2*ns[ii], qp_sol->ux+ii, nu[ii]+nx[ii]);
//		blasfeo_print_tran_dvec(2*ns[ii], qp_sol->t+ii, 2*nb[ii]+2*ng[ii]+2*nq[ii]);

		// upper and lower bounds on inputs and states
		VECEX_SP(nb[ii], 1.0, qp->idxb[ii], qp_sol->ux+ii, 0, qp_sol->t+ii, 0);
		VECCPSC(nb[ii], -1.0, qp_sol->t+ii, 0, qp_sol->t+ii, nb[ii]+ng[ii]+nq[ii]);
		for(jj=0; jj<nb[ii]; jj++)
			{
			idx = idxs_rev[jj];
			if(idx!=-1)
				{
				// softed bound
				t_lb[jj] += s[idx];
				t_ub[jj] += s[ns[ii]+idx];
				}
			}
		AXPY(nb[ii], -1.0, qp->d+ii, 0, qp_sol->t+ii, 0, qp_sol->t+ii, 0);
		AXPY(nb[ii], -1.0, qp->d+ii, nb[ii]+ng[ii]+nq[ii], qp_sol->t+ii, nb[ii]+ng[ii]+nq[ii], qp_sol->t+ii, nb[ii]+ng[ii]+nq[ii]);
//		blasfeo_print_tran_dvec(nb[ii], qp_sol->t+ii, 0);
//		blasfeo_print_tran_dvec(nb[ii], qp_sol->t+ii, nb[ii]+ng[ii]+nq[ii]);
		for(jj=0; jj<nb[ii]; jj++)
			{
#if 1
			if(t_lb[jj]<thr0)
				{
				if(t_ub[jj]<thr0)
					{
//					ux[idxb[jj]] = 0.5*(d_lb[jj] + d_ub[jj]);
					ux[idxb[jj]] = 0.5*(d_lb[jj] - d_ub[jj]);
					t_lb[jj] = thr0;
					t_ub[jj] = thr0;
					}
				else
					{
					t_lb[jj] = thr0;
					ux[idxb[jj]] = d_lb[jj] + thr0;
					}
				}
			else if(t_ub[jj]<thr0)
				{
				t_ub[jj] = thr0;
				ux[idxb[jj]] = - d_ub[jj] - thr0;
				}
#else
			t_lb[jj] = 1.0;
			t_ub[jj] = 1.0;
#endif
			}
//		blasfeo_print_tran_dvec(nu[ii]+nx[ii], qp_sol->ux+ii, 0);
//		blasfeo_print_tran_dvec(nb[ii], qp_sol->t+ii, 0);
//		blasfeo_print_tran_dvec(nb[ii], qp_sol->t+ii, nb[ii]+ng[ii]+nq[ii]);

		// upper and lower general constaints
		GEMV_T(nu[ii]+nx[ii], ng[ii], 1.0, qp->DCt+ii, 0, 0, qp_sol->ux+ii, 0, 0.0, qp_sol->t+ii, nb[ii], qp_sol->t+ii, nb[ii]);
		VECCPSC(ng[ii], -1.0, qp_sol->t+ii, nb[ii], qp_sol->t+ii, 2*nb[ii]+ng[ii]+nq[ii]);
//		blasfeo_print_tran_dvec(ng[ii], qp_sol->t+ii, nb[ii]);
//		blasfeo_print_tran_dvec(ng[ii], qp_sol->t+ii, 2*nb[ii]+ng[ii]+nq[ii]);
		for(jj=0; jj<ng[ii]; jj++)
			{
			idx = idxs_rev[nb[ii]+jj];
			if(idx!=-1)
				{
				// softed general constraint
				t_lb[nb[ii]+jj] += s[idx];
				t_ub[nb[ii]+jj] += s[ns[ii]+idx];
				}
			}
//		blasfeo_print_tran_dvec(ng[ii], qp_sol->t+ii, nb[ii]);
//		blasfeo_print_tran_dvec(ng[ii], qp_sol->t+ii, 2*nb[ii]+ng[ii]+nq[ii]);
		AXPY(ng[ii], -1.0, qp->d+ii, nb[ii], qp_sol->t+ii, nb[ii], qp_sol->t+ii, nb[ii]);
		AXPY(ng[ii], -1.0, qp->d+ii, 2*nb[ii]+ng[ii]+nq[ii], qp_sol->t+ii, 2*nb[ii]+ng[ii]+nq[ii], qp_sol->t+ii, 2*nb[ii]+ng[ii]+nq[ii]);
		for(jj=0; jj<ng[ii]; jj++)
			{
#if 1
			t_lg[jj] = thr0>t_lg[jj] ? thr0 : t_lg[jj];
			t_ug[jj] = thr0>t_ug[jj] ? thr0 : t_ug[jj];
#else
			t_lg[jj] = 1.0;
			t_ug[jj] = 1.0;
#endif
			}
//		blasfeo_print_tran_dvec(ng[ii], qp_sol->t+ii, nb[ii]);
//		blasfeo_print_tran_dvec(ng[ii], qp_sol->t+ii, 2*nb[ii]+ng[ii]+nq[ii]);

		// multipliers
		for(jj=0; jj<2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii]; jj++)
			lam_lb[jj] = mu0/t_lb[jj];

		}

	// TODO nq



#endif // new version


	return;

	}



void TREE_OCP_QCQP_APPROX_QP(struct TREE_OCP_QCQP *qcqp, struct TREE_OCP_QCQP_SOL *qcqp_sol, struct TREE_OCP_QP *qp, struct TREE_OCP_QCQP_IPM_WS *ws)
	{

	struct tree *ttree = qcqp->dim->ttree;
	int Nn = qcqp->dim->Nn;
	int *nu = qcqp->dim->nu;
	int *nx = qcqp->dim->nx;
	int *nb = qcqp->dim->nb;
	int *ng = qcqp->dim->ng;
	int *nq = qcqp->dim->nq;
	int *ns = qcqp->dim->ns;

	REAL tmp;

	int ii, jj, idx, idxdad;


	for(ii=0; ii<Nn; ii++)
		{

		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qcqp->d+ii, 0, qp->d+ii, 0);

		GECP(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], qcqp->RSQrq+ii, 0, 0, qp->RSQrq+ii, 0, 0);

		VECSE(nu[ii]+nx[ii], 0.0, ws->qcqp_res_ws->q_adj+ii, 0);

		for(jj=0; jj<nq[ii]; jj++)
			{
			tmp = - BLASFEO_VECEL(qcqp_sol->lam+ii, nb[ii]+ng[ii]+jj) + BLASFEO_VECEL(qcqp_sol->lam+ii, 2*nb[ii]+2*ng[ii]+nq[ii]+jj);
			GEAD(nu[ii]+nx[ii], nu[ii]+nx[ii], tmp, &qcqp->Hq[ii][jj], 0, 0, qp->RSQrq+ii, 0, 0);

			SYMV_L(nu[ii]+nx[ii], 1.0, &qcqp->Hq[ii][jj], 0, 0, qcqp_sol->ux+ii, 0, 0.0, ws->tmp_nuxM+0, 0, ws->tmp_nuxM+0, 0);
			COLEX(nu[ii]+nx[ii], qcqp->DCt+ii, 0, ng[ii]+jj, ws->tmp_nuxM+1, 0);
			AXPY(nu[ii]+nx[ii], 1.0, ws->tmp_nuxM+0, 0, ws->tmp_nuxM+1, 0, ws->tmp_nuxM+1, 0);
			COLIN(nu[ii]+nx[ii], ws->tmp_nuxM+1, 0, qp->DCt+ii, 0, ng[ii]+jj);
			AXPY(nu[ii]+nx[ii], tmp, ws->tmp_nuxM+1, 0, ws->qcqp_res_ws->q_adj+ii, 0, ws->qcqp_res_ws->q_adj+ii, 0);

			COLEX(nu[ii]+nx[ii], qcqp->DCt+ii, 0, ng[ii]+jj, ws->tmp_nuxM+1, 0);
			AXPY(nu[ii]+nx[ii], 0.5, ws->tmp_nuxM+0, 0, ws->tmp_nuxM+1, 0, ws->tmp_nuxM+1, 0);
			tmp = DOT(nu[ii]+nx[ii], ws->tmp_nuxM+1, 0, qcqp_sol->ux+ii, 0);
			// TODO maybe swap signs?
			BLASFEO_VECEL(qp->d+ii, nb[ii]+ng[ii]+jj) += - tmp;
			BLASFEO_VECEL(qp->d+ii, 2*nb[ii]+2*ng[ii]+nq[ii]+jj) += + tmp;
			BLASFEO_VECEL(ws->qcqp_res_ws->q_fun+ii, jj) = tmp;
			}

		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qcqp->d_mask+ii, 0, qp->d_mask+ii, 0);

		GECP(nu[ii]+nx[ii], ng[ii], qcqp->DCt+ii, 0, 0, qp->DCt+ii, 0, 0);

		VECCP(nu[ii]+nx[ii]+2*ns[ii], qcqp->rqz+ii, 0, qp->rqz, 0);

		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qcqp->m+ii, 0, qp->m+ii, 0);

		VECCP(2*ns[ii], qcqp->Z+ii, 0, qp->Z+ii, 0);

		for(jj=0; jj<nb[ii]; jj++)
			qp->idxb[ii][jj] = qcqp->idxb[ii][jj];

		for(jj=0; jj<nb[ii]+ng[ii]+nq[ii]; jj++)
			qp->idxs_rev[ii][jj] = qcqp->idxs_rev[ii][jj];

		}

	for(ii=0; ii<Nn-1; ii++)
		{
		idx = ii+1;
		idxdad = (ttree->root+idx)->dad;
		GECP(nu[idxdad]+nx[idxdad]+1, nx[idx], qcqp->BAbt+ii, 0, 0, qp->BAbt+ii, 0, 0);

		VECCP(nx[idxdad], qcqp->b+ii, 0, qp->b+ii, 0);
		}

	return;

	}



void TREE_OCP_QCQP_UPDATE_QP(struct TREE_OCP_QCQP *qcqp, struct TREE_OCP_QCQP_SOL *qcqp_sol, struct TREE_OCP_QP *qp, struct TREE_OCP_QCQP_IPM_WS *ws)
	{

	int Nn = qcqp->dim->Nn;
	int *nu = qcqp->dim->nu;
	int *nx = qcqp->dim->nx;
	int *nb = qcqp->dim->nb;
	int *ng = qcqp->dim->ng;
	int *nq = qcqp->dim->nq;
	int *ns = qcqp->dim->ns;

	REAL tmp;

	int ii, jj, idx;


	for(ii=0; ii<Nn; ii++)
		{

		// TODO only the 2*nq part needed !!!!!
		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qcqp->d+ii, 0, qp->d+ii, 0);

		GECP(nu[ii]+nx[ii]+1, nu[ii]+nx[ii], qcqp->RSQrq+ii, 0, 0, qp->RSQrq+ii, 0, 0);

		VECSE(nu[ii]+nx[ii], 0.0, ws->qcqp_res_ws->q_adj+ii, 0);

		for(jj=0; jj<nq[ii]; jj++)
			{
			tmp = - BLASFEO_VECEL(qcqp_sol->lam+ii, nb[ii]+ng[ii]+jj) + BLASFEO_VECEL(qcqp_sol->lam+ii, 2*nb[ii]+2*ng[ii]+nq[ii]+jj);
			GEAD(nu[ii]+nx[ii], nu[ii]+nx[ii], tmp, &qcqp->Hq[ii][jj], 0, 0, qp->RSQrq+ii, 0, 0);

			SYMV_L(nu[ii]+nx[ii], 1.0, &qcqp->Hq[ii][jj], 0, 0, qcqp_sol->ux+ii, 0, 0.0, ws->tmp_nuxM+0, 0, ws->tmp_nuxM+0, 0);
			COLEX(nu[ii]+nx[ii], qcqp->DCt+ii, 0, ng[ii]+jj, ws->tmp_nuxM+1, 0);
			AXPY(nu[ii]+nx[ii], 1.0, ws->tmp_nuxM+0, 0, ws->tmp_nuxM+1, 0, ws->tmp_nuxM+1, 0);
			COLIN(nu[ii]+nx[ii], ws->tmp_nuxM+1, 0, qp->DCt+ii, 0, ng[ii]+jj);
			AXPY(nu[ii]+nx[ii], tmp, ws->tmp_nuxM+1, 0, ws->qcqp_res_ws->q_adj+ii, 0, ws->qcqp_res_ws->q_adj+ii, 0);

			COLEX(nu[ii]+nx[ii], qcqp->DCt+ii, 0, ng[ii]+jj, ws->tmp_nuxM+1, 0);
			AXPY(nu[ii]+nx[ii], 0.5, ws->tmp_nuxM+0, 0, ws->tmp_nuxM+1, 0, ws->tmp_nuxM+1, 0);
			tmp = DOT(nu[ii]+nx[ii], ws->tmp_nuxM+1, 0, qcqp_sol->ux+ii, 0);
			// TODO maybe swap signs?
			BLASFEO_VECEL(qp->d+ii, nb[ii]+ng[ii]+jj) += - tmp;
			BLASFEO_VECEL(qp->d+ii, 2*nb[ii]+2*ng[ii]+nq[ii]+jj) += + tmp;
			BLASFEO_VECEL(ws->qcqp_res_ws->q_fun+ii, jj) = tmp;
			}

		// TODO needed ?????
		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qcqp->m+ii, 0, qp->m+ii, 0);

		}

	return;

	}



void TREE_OCP_QCQP_UPDATE_QP_ABS_STEP(struct TREE_OCP_QCQP *qcqp, struct TREE_OCP_QCQP_SOL *qcqp_sol, struct TREE_OCP_QP *qp, struct TREE_OCP_QCQP_IPM_WS *ws)
	{

	int Nn = qcqp->dim->Nn;
	int *nu = qcqp->dim->nu;
	int *nx = qcqp->dim->nx;
	int *nb = qcqp->dim->nb;
	int *ng = qcqp->dim->ng;
	int *nq = qcqp->dim->nq;
	int *ns = qcqp->dim->ns;

	REAL tmp;

	int ii, jj, idx;


	for(ii=0; ii<Nn; ii++)
		{

		// TODO only the 2*nq part needed !!!!!
		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qcqp->d+ii, 0, qp->d+ii, 0);

		GECP(nu[ii]+nx[ii]+1, nu[ii]+ns[ii], qcqp->RSQrq+ii, 0, 0, qp->RSQrq+ii, 0, 0);

		VECSE(nu[ii]+nx[ii], 0.0, ws->qcqp_res_ws->q_adj+ii, 0);

		for(jj=0; jj<nq[ii]; jj++)
			{
			tmp = - BLASFEO_VECEL(qcqp_sol->lam+ii, nb[ii]+ng[ii]+jj) + BLASFEO_VECEL(qcqp_sol->lam+ii, 2*nb[ii]+2*ng[ii]+nq[ii]+jj);
			GEAD(nu[ii]+nx[ii], nu[ii]+nx[ii], tmp, &qcqp->Hq[ii][jj], 0, 0, qp->RSQrq+ii, 0, 0);

			SYMV_L(nu[ii]+nx[ii], 1.0, &qcqp->Hq[ii][jj], 0, 0, qcqp_sol->ux+ii, 0, 0.0, ws->tmp_nuxM+0, 0, ws->tmp_nuxM+0, 0);
			COLEX(nu[ii]+nx[ii], qcqp->DCt+ii, 0, ng[ii]+jj, ws->tmp_nuxM+1, 0);
			AXPY(nu[ii]+nx[ii], 1.0, ws->tmp_nuxM+0, 0, ws->tmp_nuxM+1, 0, ws->tmp_nuxM+1, 0);
			COLIN(nu[ii]+nx[ii], ws->tmp_nuxM+1, 0, qp->DCt+ii, 0, ng[ii]+jj);
			AXPY(nu[ii]+nx[ii], tmp, ws->tmp_nuxM+1, 0, ws->qcqp_res_ws->q_adj+ii, 0, ws->qcqp_res_ws->q_adj+ii, 0);

//	//		AXPY(nu[ii]+nx[ii], 0.5, ws->tmp_nuxM+0, 0, &qcqp->gq[ii][jj], 0, ws->tmp_nuxM+1, 0);
//			AXPY(nu[ii]+nx[ii], 0.5, ws->tmp_nuxM+0, 0, &qcqp->gq[ii][jj], 0, ws->tmp_nuxM+0, 0);
//			AXPY(nu[ii]+nx[ii], -1.0, ws->tmp_nuxM+1, 0, ws->tmp_nuxM+0, 0, ws->tmp_nuxM+1, 0);
			AXPBY(nu[ii]+nx[ii], -1.0, ws->tmp_nuxM+1, 0, 0.5, ws->tmp_nuxM+0, 0, ws->tmp_nuxM+1, 0);
			COLEX(nu[ii]+nx[ii], qcqp->DCt+ii, 0, ng[ii]+jj, ws->tmp_nuxM+0, 0);
			AXPY(nu[ii]+nx[ii], 1.0, ws->tmp_nuxM+0, 0, ws->tmp_nuxM+1, 0, ws->tmp_nuxM+1, 0);
			tmp = DOT(nu[ii]+nx[ii], ws->tmp_nuxM+1, 0, qcqp_sol->ux+ii, 0);
			// TODO maybe swap signs?
			BLASFEO_VECEL(qp->d+ii, nb[ii]+ng[ii]+jj) += - tmp;
			BLASFEO_VECEL(qp->d+ii, 2*nb[ii]+2*ng[ii]+nq[ii]+jj) += + tmp;
			BLASFEO_VECEL(ws->qcqp_res_ws->q_fun+ii, jj) = tmp;
			}

		// TODO needed ?????
		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qcqp->m+ii, 0, qp->m+ii, 0);

		}

	return;

	}



void TREE_OCP_QCQP_SOL_CONV_QP_SOL(struct TREE_OCP_QCQP_SOL *qcqp_sol, struct TREE_OCP_QP_SOL *qp_sol)
	{

	int Nn = qcqp_sol->dim->Nn;
	int *nu = qcqp_sol->dim->nu;
	int *nx = qcqp_sol->dim->nx;
	int *nb = qcqp_sol->dim->nb;
	int *ng = qcqp_sol->dim->ng;
	int *nq = qcqp_sol->dim->nq;
	int *ns = qcqp_sol->dim->ns;

	int ii;

	for(ii=0; ii<Nn; ii++)
		VECCP(nu[ii]+nx[ii]+2*ns[ii], qcqp_sol->ux+ii, 0, qp_sol->ux+ii, 0);

	for(ii=0; ii<Nn-1; ii++)
		VECCP(nx[ii+1], qcqp_sol->pi+ii, 0, qp_sol->pi+ii, 0);

	for(ii=0; ii<Nn; ii++)
		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qcqp_sol->lam+ii, 0, qp_sol->lam+ii, 0);

	for(ii=0; ii<Nn; ii++)
		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qcqp_sol->t+ii, 0, qp_sol->t+ii, 0);

	return;

	}



void TREE_OCP_QP_SOL_CONV_QCQP_SOL(struct TREE_OCP_QP_SOL *qp_sol, struct TREE_OCP_QCQP_SOL *qcqp_sol)
	{

	int Nn = qcqp_sol->dim->Nn;
	int *nu = qcqp_sol->dim->nu;
	int *nx = qcqp_sol->dim->nx;
	int *nb = qcqp_sol->dim->nb;
	int *ng = qcqp_sol->dim->ng;
	int *nq = qcqp_sol->dim->nq;
	int *ns = qcqp_sol->dim->ns;

	int ii;

	for(ii=0; ii<Nn; ii++)
		VECCP(nu[ii]+nx[ii]+2*ns[ii], qp_sol->ux+ii, 0, qcqp_sol->ux+ii, 0);

	for(ii=0; ii<Nn-1; ii++)
		VECCP(nx[ii+1], qp_sol->pi+ii, 0, qcqp_sol->pi+ii, 0);

	for(ii=0; ii<Nn; ii++)
		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp_sol->lam+ii, 0, qcqp_sol->lam+ii, 0);

	for(ii=0; ii<Nn; ii++)
		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qp_sol->t+ii, 0, qcqp_sol->t+ii, 0);

	return;

	}



void TREE_OCP_QCQP_RES_CONV_QP_RES(struct TREE_OCP_QCQP_RES *qcqp_res, struct TREE_OCP_QP_RES *qp_res)
	{

	int Nn = qcqp_res->dim->Nn;
	int *nu = qcqp_res->dim->nu;
	int *nx = qcqp_res->dim->nx;
	int *nb = qcqp_res->dim->nb;
	int *ng = qcqp_res->dim->ng;
	int *nq = qcqp_res->dim->nq;
	int *ns = qcqp_res->dim->ns;

	int ii;

	for(ii=0; ii<Nn; ii++)
		VECCP(nu[ii]+nx[ii]+2*ns[ii], qcqp_res->res_g+ii, 0, qp_res->res_g+ii, 0);

	for(ii=0; ii<Nn-1; ii++)
		VECCP(nx[ii+1], qcqp_res->res_b+ii, 0, qp_res->res_b+ii, 0);

	for(ii=0; ii<Nn; ii++)
		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qcqp_res->res_d+ii, 0, qp_res->res_d+ii, 0);

	for(ii=0; ii<Nn; ii++)
		VECCP(2*nb[ii]+2*ng[ii]+2*nq[ii]+2*ns[ii], qcqp_res->res_m+ii, 0, qp_res->res_m+ii, 0);

	qp_res->res_mu = qcqp_res->res_mu;

	return;

	}



void TREE_OCP_QCQP_IPM_SOLVE(struct TREE_OCP_QCQP *qcqp, struct TREE_OCP_QCQP_SOL *qcqp_sol, struct TREE_OCP_QCQP_IPM_ARG *qcqp_arg, struct TREE_OCP_QCQP_IPM_WS *qcqp_ws)
	{

	// dim
	int Nn = qcqp->dim->Nn;
	int *nx = qcqp->dim->nx;
	int *nu = qcqp->dim->nu;
	int *nb = qcqp->dim->nb;
	int *ng = qcqp->dim->ng;
	int *nq = qcqp->dim->nq;
	int *ns = qcqp->dim->ns;

	// extract stuff

	struct TREE_OCP_QP *qp = qcqp_ws->qp;
	struct TREE_OCP_QP_SOL *qp_sol = qcqp_ws->qp_sol;
	struct TREE_OCP_QP_IPM_WS *qp_ws = qcqp_ws->qp_ws;
	struct TREE_OCP_QP_IPM_ARG *qp_arg = qcqp_arg->qp_arg;

	struct TREE_OCP_QCQP_DIM *qcqp_dim = qcqp->dim;
	struct TREE_OCP_QCQP_RES *qcqp_res = qcqp_ws->qcqp_res;
	struct TREE_OCP_QCQP_RES_WS *qcqp_res_ws = qcqp_ws->qcqp_res_ws;

	struct CORE_QP_IPM_WORKSPACE *cws = qp_ws->core_workspace;

	int kk, ii, jj, idx;
	REAL mu;

	REAL *stat = qp_ws->stat;
	int stat_m = qp_ws->stat_m;
	int stat_max = qp_ws->stat_max;

//	int qcqp_nv = qcqp->dim->nv + 2*qcqp->dim->ns;
//	int qcqp_ne = qcqp->dim->ne;
//	int qcqp_nc = 2*qcqp->dim->nb + 2*qcqp->dim->ng + qcqp->dim->nq + 2*qcqp->dim->ns;


	// qp_arg to core workspace
	cws->lam_min = qp_arg->lam_min;
	cws->t_min = qp_arg->t_min;
	cws->t_min_inv = qp_arg->t_min>0 ? 1.0/qp_arg->t_min : 1e30;
	cws->split_step = qp_arg->split_step;
	cws->t_lam_min = qp_arg->t_lam_min;

	// alias qp vectors into qp_sol
	cws->v = qp_sol->ux->pa;
	cws->pi = qp_sol->pi->pa;
	cws->lam = qp_sol->lam->pa;
	cws->t = qp_sol->t->pa;

	// alias members of qp_step
	qp_ws->qp_step->dim = qp->dim;
	qp_ws->qp_step->RSQrq = qp->RSQrq;
	qp_ws->qp_step->BAbt = qp->BAbt;
	qp_ws->qp_step->DCt = qp->DCt;
	qp_ws->qp_step->Z = qp->Z;
	qp_ws->qp_step->idxb = qp->idxb;
	qp_ws->qp_step->idxs_rev = qp->idxs_rev;
	qp_ws->qp_step->rqz = qp_ws->res->res_g;
	qp_ws->qp_step->b = qp_ws->res->res_b;
	qp_ws->qp_step->d = qp_ws->res->res_d;
	qp_ws->qp_step->m = qp_ws->res->res_m;
	qp_ws->qp_step->d_mask = qp->d_mask;

	// alias members of qp_itref
	qp_ws->qp_itref->dim = qp->dim;
	qp_ws->qp_itref->RSQrq = qp->RSQrq;
	qp_ws->qp_itref->BAbt = qp->BAbt;
	qp_ws->qp_itref->DCt = qp->DCt;
	qp_ws->qp_itref->Z = qp->Z;
	qp_ws->qp_itref->idxb = qp->idxb;
	qp_ws->qp_itref->idxs_rev = qp->idxs_rev;
	qp_ws->qp_itref->rqz = qp_ws->res_itref->res_g;
	qp_ws->qp_itref->b = qp_ws->res_itref->res_b;
	qp_ws->qp_itref->d = qp_ws->res_itref->res_d;
	qp_ws->qp_itref->m = qp_ws->res_itref->res_m;
	qp_ws->qp_itref->d_mask = qp->d_mask;

	REAL *qcqp_res_max = qcqp_res->res_max;


	// cache q_fun & q_adj from approx/update for res
	qcqp_ws->qcqp_res_ws->use_q_fun = 1;
	qcqp_ws->qcqp_res_ws->use_q_adj = 1;


	// disregard soft constr on (disregarded) lower quard constr
	for(ii=0; ii<Nn; ii++)
		{
		VECSE(nq[ii], 0.0, qcqp->d_mask+ii, nb[ii]+ng[ii]); // TODO needed ???
		// TODO probably remove when using only idxs_rev, as the same slack may be associated with other constraints !!!!!
		for(jj=0; jj<nq[ii]; jj++)
			{
			idx = qcqp->idxs_rev[ii][nb[ii]+ng[ii]+jj];
			if(idx!=-1)
				{
				BLASFEO_VECEL(qcqp->d_mask+ii, 2*nb[ii]+2*ng[ii]+2*nq[ii]+idx) = 0.0;
				}
			}
		}


	// initialize qcqp & qp
	TREE_OCP_QCQP_INIT_VAR(qcqp, qcqp_sol, qcqp_arg, qcqp_ws);
	// mask out disregarded constraints
	VECMUL(cws->nc, qcqp->d_mask, 0, qcqp_sol->lam, 0, qcqp_sol->lam, 0);
	TREE_OCP_QCQP_SOL_CONV_QP_SOL(qcqp_sol, qp_sol);

	// approximate qcqp with a qp
	TREE_OCP_QCQP_APPROX_QP(qcqp, qcqp_sol, qp, qcqp_ws);


	// detect constr mask
	int mask_unconstr;
	int nc_mask = 0;
	for(ii=0; ii<cws->nc; ii++)
		{
		if(qp->d_mask->pa[ii]!=0.0)
			nc_mask++;
		}
	if(nc_mask==0)
		{
		mask_unconstr = 1;
		cws->nc_mask = 0;
		cws->nc_mask_inv = 0.0;
		}
	else
		{
		mask_unconstr = 0;
		cws->nc_mask = nc_mask;
		cws->nc_mask_inv = 1.0/nc_mask;
		}
	// always mask lower quadratic constr
	qp_ws->mask_constr = 1;


	// no constraints
	if(cws->nc==0 | mask_unconstr==1)
		{
		TREE_OCP_QP_FACT_SOLVE_KKT_UNCONSTR(qp, qp_sol, qp_arg, qp_ws);
		TREE_OCP_QP_SOL_CONV_QCQP_SOL(qp_sol, qcqp_sol);
		if(qp_arg->comp_res_exit)
			{
			// compute residuals
			TREE_OCP_QCQP_RES_COMPUTE(qcqp, qcqp_sol, qcqp_res, qcqp_res_ws);
			// XXX no constraints, so no mask
			TREE_OCP_QCQP_RES_COMPUTE_INF_NORM(qcqp_res);
			// save infinity norm of residuals
			if(0<stat_max)
				{
				stat[6] = qcqp_res_max[0];
				stat[7] = qcqp_res_max[1];
				stat[8] = qcqp_res_max[2];
				stat[9] = qcqp_res_max[3];
				}
			cws->mu = qcqp_res->res_mu;
			}
		// save info before return
		qcqp_ws->iter = 0;
		qcqp_ws->status = 0;
		return;
		}
	

	cws->alpha = 1.0;


	// absolute IPM formulation
	if(qp_arg->abs_form)
		{

		// alias members of qp_step
		qp_ws->qp_step->dim = qp->dim;
		qp_ws->qp_step->RSQrq = qp->RSQrq;
		qp_ws->qp_step->BAbt = qp->BAbt;
		qp_ws->qp_step->DCt = qp->DCt;
		qp_ws->qp_step->Z = qp->Z;
		qp_ws->qp_step->idxb = qp->idxb;
//		qp_ws->qp_step->idxe = qp->idxe;
		qp_ws->qp_step->idxs_rev = qp->idxs_rev;
		qp_ws->qp_step->rqz = qp->rqz;
		qp_ws->qp_step->b = qp->b;
		qp_ws->qp_step->d = qp->d;
		qp_ws->qp_step->d_mask = qp->d_mask;
		qp_ws->qp_step->m = qp_ws->tmp_m;

		// alias core workspace
		cws->res_m = qp_ws->qp_step->m->pa;

		// update approximation of qcqp as qp for absolute step
		TREE_OCP_QCQP_UPDATE_QP_ABS_STEP(qcqp, qcqp_sol, qp, qcqp_ws);

		// compute mu
		mu = VECMULDOT(cws->nc, qp_sol->lam, 0, qp_sol->t, 0, qp_ws->tmp_m, 0);
		mu /= cws->nc;
		cws->mu = mu;

		// IPM loop (absolute formulation)
		for(kk=0; \
				kk < qcqp_arg->iter_max & \
				cws->alpha > qcqp_arg->alpha_min & \
				mu > qcqp_arg->res_m_max; kk++)
			{

			// compute delta step
			TREE_OCP_QP_IPM_ABS_STEP(kk, qp, qp_sol, qp_arg, qp_ws);
//blasfeo_print_exp_tran_dvec(cws->nc, qp_sol->lam, 0);
			TREE_OCP_QP_SOL_CONV_QCQP_SOL(qp_sol, qcqp_sol);

			// update approximation of qcqp as qp for absolute step
			TREE_OCP_QCQP_UPDATE_QP_ABS_STEP(qcqp, qcqp_sol, qp, qcqp_ws);

			// compute mu
			mu = VECMULDOT(cws->nc, qcqp_sol->lam, 0, qcqp_sol->t, 0, qp_ws->tmp_m, 0);
			mu /= cws->nc;
			cws->mu = mu;
			if(kk+1<stat_max)
				stat[stat_m*(kk+1)+5] = mu;

			}

		if(qp_arg->comp_res_exit & qp_arg->comp_dual_sol_eq)
			{
			// compute residuals
			TREE_OCP_QCQP_RES_COMPUTE(qcqp, qcqp_sol, qcqp_res, qcqp_res_ws);
			if(qp_ws->mask_constr)
				{
				// mask out disregarded constraints
				for(ii=0; ii<Nn; ii++)
					VECMUL(2*ns[ii], qp->d_mask+ii, 2*nb[ii]+2*ng[ii]+2*nq[ii], qcqp_res->res_g+ii, nu[ii]+nx[ii], qcqp_res->res_g+ii, nu[ii]+nx[ii]);
				VECMUL(cws->nc, qp->d_mask, 0, qcqp_res->res_d, 0, qcqp_res->res_d, 0);
				VECMUL(cws->nc, qp->d_mask, 0, qcqp_res->res_m, 0, qcqp_res->res_m, 0);
				}
			TREE_OCP_QCQP_RES_COMPUTE_INF_NORM(qcqp_res);
			// save infinity norm of residuals
			// XXX it is already kk+1
			if(kk<stat_max)
				{
				stat[stat_m*(kk+0)+6] = qcqp_res_max[0];
				stat[stat_m*(kk+0)+7] = qcqp_res_max[1];
				stat[stat_m*(kk+0)+8] = qcqp_res_max[2];
				stat[stat_m*(kk+0)+9] = qcqp_res_max[3];
				}
			}

		goto set_status;

		}


	// compute residuals
	TREE_OCP_QCQP_RES_COMPUTE(qcqp, qcqp_sol, qcqp_res, qcqp_res_ws);
	if(qp_ws->mask_constr)
		{
		// mask out disregarded constraints
		for(ii=0; ii<Nn; ii++)
			VECMUL(2*ns[ii], qp->d_mask+ii, 2*nb[ii]+2*ng[ii]+2*nq[ii], qcqp_res->res_g+ii, nu[ii]+nx[ii], qcqp_res->res_g+ii, nu[ii]+nx[ii]);
		VECMUL(cws->nc, qp->d_mask, 0, qcqp_res->res_d, 0, qcqp_res->res_d, 0);
		VECMUL(cws->nc, qp->d_mask, 0, qcqp_res->res_m, 0, qcqp_res->res_m, 0);
		}
	TREE_OCP_QCQP_RES_COMPUTE_INF_NORM(qcqp_res);
	TREE_OCP_QCQP_RES_CONV_QP_RES(qcqp_res, qp_ws->res);
	cws->mu = qcqp_res->res_mu;
	// save infinity norm of residuals
	if(0<stat_max)
		{
		stat[stat_m*(0)+6] = qcqp_res_max[0];
		stat[stat_m*(0)+7] = qcqp_res_max[1];
		stat[stat_m*(0)+8] = qcqp_res_max[2];
		stat[stat_m*(0)+9] = qcqp_res_max[3];
		}


	// relative (delta) IPM formulation
	for(kk=0; \
			kk < qcqp_arg->iter_max & \
			cws->alpha > qcqp_arg->alpha_min & \
			(qcqp_res_max[0] > qcqp_arg->res_g_max | \
			qcqp_res_max[1] > qcqp_arg->res_b_max | \
			qcqp_res_max[2] > qcqp_arg->res_d_max | \
			qcqp_res_max[3] > qcqp_arg->res_m_max); kk++)
		{

		// hessian is updated with quad constr: can not reuse hessian factorization !!!
		// XXX is it in ocp ws ?????
		for(ii=0; ii<Nn; ii++)
			qp_ws->use_hess_fact[ii] = 0;

		// compute delta step
		TREE_OCP_QP_IPM_DELTA_STEP(kk, qp, qp_sol, qp_arg, qp_ws);
//blasfeo_print_exp_tran_dvec(cws->nc, qp_sol->lam, 0);
		TREE_OCP_QP_SOL_CONV_QCQP_SOL(qp_sol, qcqp_sol);
		// XXX maybe not needed
		if(qp_ws->mask_constr)
			{
			// mask out disregarded constraints
			VECMUL(cws->nc, qp->d_mask, 0, qcqp_sol->lam, 0, qcqp_sol->lam, 0);
			}

		// update approximation of qcqp as qp
		TREE_OCP_QCQP_UPDATE_QP(qcqp, qcqp_sol, qp, qcqp_ws);

		// compute residuals
		TREE_OCP_QCQP_RES_COMPUTE(qcqp, qcqp_sol, qcqp_res, qcqp_res_ws);
		if(qp_ws->mask_constr)
			{
			// mask out disregarded constraints
			for(ii=0; ii<Nn; ii++)
				VECMUL(2*ns[ii], qp->d_mask+ii, 2*nb[ii]+2*ng[ii]+2*nq[ii], qcqp_res->res_g+ii, nu[ii]+nx[ii], qcqp_res->res_g+ii, nu[ii]+nx[ii]);
			VECMUL(cws->nc, qp->d_mask, 0, qcqp_res->res_d, 0, qcqp_res->res_d, 0);
			VECMUL(cws->nc, qp->d_mask, 0, qcqp_res->res_m, 0, qcqp_res->res_m, 0);
			}
		TREE_OCP_QCQP_RES_COMPUTE_INF_NORM(qcqp_res);
		TREE_OCP_QCQP_RES_CONV_QP_RES(qcqp_res, qp_ws->res);
		cws->mu = qcqp_res->res_mu;
		// save infinity norm of residuals
		if(kk+1<stat_max)
			{
			stat[stat_m*(kk+1)+5] = qcqp_res->res_mu;
			stat[stat_m*(kk+1)+6] = qcqp_res_max[0];
			stat[stat_m*(kk+1)+7] = qcqp_res_max[1];
			stat[stat_m*(kk+1)+8] = qcqp_res_max[2];
			stat[stat_m*(kk+1)+9] = qcqp_res_max[3];
			}

		}

set_status:

	// save info before return
	qcqp_ws->iter = kk;

	if(kk == qcqp_arg->iter_max)
		{
		// max iteration number reached
		qcqp_ws->status = MAX_ITER;
		}
	else if(cws->alpha <= qcqp_arg->alpha_min)
		{
		// min step lenght
		qcqp_ws->status = MIN_STEP;
		}
#ifdef USE_C99_MATH
	else if(isnan(cws->mu))
		{
		// NaN in the solution
		qcqp_ws->status = NAN_SOL;
		}
#else
	else if(cws->mu != cws->mu)
		{
		// NaN in the solution
		qcqp_ws->status = NAN_SOL;
		}
#endif
	else
		{
		// normal return
		qcqp_ws->status = SUCCESS;
		}

	return;

	}



