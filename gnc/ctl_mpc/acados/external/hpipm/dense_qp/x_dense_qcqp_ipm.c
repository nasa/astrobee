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



hpipm_size_t DENSE_QCQP_IPM_ARG_MEMSIZE(struct DENSE_QCQP_DIM *dim)
	{

	hpipm_size_t size = 0;

	size += 1*sizeof(struct DENSE_QP_IPM_ARG);
	size += 1*DENSE_QP_IPM_ARG_MEMSIZE(dim->qp_dim);

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 1*64; // align once to typical cache line size

	return size;

	}



void DENSE_QCQP_IPM_ARG_CREATE(struct DENSE_QCQP_DIM *dim, struct DENSE_QCQP_IPM_ARG *arg, void *mem)
	{

	// loop index
	int ii;

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = DENSE_QCQP_IPM_ARG_MEMSIZE(dim);
	hpipm_zero_memset(memsize, mem);

	// qp_dim struct
	struct DENSE_QP_IPM_ARG *arg_ptr = mem;

	arg->qp_arg = arg_ptr;
	arg_ptr += 1;

	// align to typical cache line size
	hpipm_size_t s_ptr = (hpipm_size_t) arg_ptr;
	s_ptr = (s_ptr+63)/64*64;

	// void
	char *c_ptr = (char *) s_ptr;

	DENSE_QP_IPM_ARG_CREATE(dim->qp_dim, arg->qp_arg, c_ptr);
	c_ptr += arg->qp_arg->memsize;


	arg->memsize = DENSE_QCQP_IPM_ARG_MEMSIZE(dim);

#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + arg->memsize)
		{
		printf("\nerror: DENSE_QCQP_IPM_ARG_CREATE: outside memory bounds!\n\n");
		exit(1);
		}
#endif

	return;

	}



void DENSE_QCQP_IPM_ARG_SET_DEFAULT(enum HPIPM_MODE mode, struct DENSE_QCQP_IPM_ARG *arg)
	{

	DENSE_QP_IPM_ARG_SET_DEFAULT(mode, arg->qp_arg);

	REAL mu0, alpha_min, res_g, res_b, res_d, res_m, reg_prim, reg_dual, lam_min, t_min;
	int iter_max, stat_max, pred_corr, cond_pred_corr, itref_pred_max, itref_corr_max, lq_fact, scale, warm_start, abs_form, comp_res_exit, comp_res_pred, split_step, t_lam_min;

	if(mode==SPEED_ABS)
		{
		mu0 = 1e1;
		alpha_min = 1e-12;
		res_g = 1e0;
		res_b = 1e0;
		res_d = 1e0;
		res_m = 1e-8;
		iter_max = 15;
		stat_max = 15;
		pred_corr = 1;
		cond_pred_corr = 1;
		itref_pred_max = 0;
		itref_corr_max = 0;
		reg_prim = 1e-15;
		reg_dual = 1e-15;
		lq_fact = 0;
		scale = 0;
		lam_min = 1e-16;
		t_min = 1e-16;
		warm_start = 0;
		abs_form = 1;
		comp_res_exit = 0;
		comp_res_pred = 0;
		split_step = 1;
		t_lam_min = 2;
		}
	else if(mode==SPEED)
		{
		mu0 = 1e1;
		alpha_min = 1e-12;
		res_g = 1e-6;
		res_b = 1e-8;
		res_d = 1e-8;
		res_m = 1e-8;
		iter_max = 15;
		stat_max = 15;
		pred_corr = 1;
		cond_pred_corr = 1;
		itref_pred_max = 0;
		itref_corr_max = 0;
		reg_prim = 1e-15;
		reg_dual = 1e-15;
		lq_fact = 0;
		scale = 0;
		lam_min = 1e-16;
		t_min = 1e-16;
		warm_start = 0;
		abs_form = 0;
		comp_res_exit = 1;
		comp_res_pred = 0;
		split_step = 1;
		t_lam_min = 2;
		}
	else if(mode==BALANCE)
		{
		mu0 = 1e1;
		alpha_min = 1e-12;
		res_g = 1e-6;
		res_b = 1e-8;
		res_d = 1e-8;
		res_m = 1e-8;
		iter_max = 30;
		stat_max = 30;
		pred_corr = 1;
		cond_pred_corr = 1;
		itref_pred_max = 0;
		itref_corr_max = 2;
		reg_prim = 1e-15;
		reg_dual = 1e-15;
		lq_fact = 1;
		scale = 0;
		lam_min = 1e-16;
		t_min = 1e-16;
		warm_start = 0;
		abs_form = 0;
		comp_res_exit = 1;
		comp_res_pred = 0;
		split_step = 0;
		t_lam_min = 2;
		}
	else if(mode==ROBUST)
		{
		mu0 = 1e2;
		alpha_min = 1e-12;
		res_g = 1e-6;
		res_b = 1e-8;
		res_d = 1e-8;
		res_m = 1e-8;
		iter_max = 100;
		stat_max = 100;
		pred_corr = 1;
		cond_pred_corr = 1;
		itref_pred_max = 0;
		itref_corr_max = 4;
		reg_prim = 1e-15;
		reg_dual = 1e-15;
		lq_fact = 2;
		scale = 0;
		lam_min = 1e-16;
		t_min = 1e-16;
		warm_start = 0;
		abs_form = 0;
		comp_res_exit = 1;
		comp_res_pred = 0;
		split_step = 0;
		t_lam_min = 2;
		}
	else
		{
		printf("\nerror: DENSE_QP_IPM_ARG_SET_DEFAULT: wrong set default mode\n");
		exit(1);
		}

	// use individual setters when available
	DENSE_QCQP_IPM_ARG_SET_MU0(&mu0, arg);
	DENSE_QCQP_IPM_ARG_SET_ALPHA_MIN(&alpha_min, arg);
	DENSE_QCQP_IPM_ARG_SET_TOL_STAT(&res_g, arg);
	DENSE_QCQP_IPM_ARG_SET_TOL_EQ(&res_b, arg);
	DENSE_QCQP_IPM_ARG_SET_TOL_INEQ(&res_d, arg);
	DENSE_QCQP_IPM_ARG_SET_TOL_COMP(&res_m, arg);
	DENSE_QCQP_IPM_ARG_SET_ITER_MAX(&iter_max, arg);
	arg->stat_max = stat_max;
	DENSE_QCQP_IPM_ARG_SET_PRED_CORR(&pred_corr, arg);
	DENSE_QCQP_IPM_ARG_SET_COND_PRED_CORR(&cond_pred_corr, arg);
	arg->itref_pred_max = itref_pred_max;
	arg->itref_corr_max = itref_corr_max;
	DENSE_QCQP_IPM_ARG_SET_REG_PRIM(&reg_prim, arg);
	DENSE_QCQP_IPM_ARG_SET_REG_DUAL(&reg_prim, arg);
	arg->lq_fact = lq_fact;
	arg->scale = scale;
	arg->lam_min = lam_min;
	arg->t_min = t_min;
	DENSE_QCQP_IPM_ARG_SET_LAM_MIN(&lam_min, arg);
	DENSE_QCQP_IPM_ARG_SET_T_MIN(&t_min, arg);
	DENSE_QCQP_IPM_ARG_SET_WARM_START(&warm_start, arg);
	arg->abs_form = abs_form;
	DENSE_QCQP_IPM_ARG_SET_COMP_RES_PRED(&comp_res_pred, arg);
	DENSE_QCQP_IPM_ARG_SET_COMP_RES_EXIT(&comp_res_pred, arg);
	DENSE_QCQP_IPM_ARG_SET_SPLIT_STEP(&split_step, arg);
	DENSE_QCQP_IPM_ARG_SET_T_LAM_MIN(&t_lam_min, arg);
	arg->mode = mode;

	return;

	}



void DENSE_QCQP_IPM_ARG_SET(char *field, void *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	if(hpipm_strcmp(field, "iter_max")) 
		{
		DENSE_QCQP_IPM_ARG_SET_ITER_MAX(value, arg);
		}
	else if(hpipm_strcmp(field, "alpha_min")) 
		{
		DENSE_QCQP_IPM_ARG_SET_ALPHA_MIN(value, arg);
		}
	else if(hpipm_strcmp(field, "mu0")) 
		{
		DENSE_QCQP_IPM_ARG_SET_MU0(value, arg);
		}
	else if(hpipm_strcmp(field, "tol_stat")) 
		{
		DENSE_QCQP_IPM_ARG_SET_TOL_STAT(value, arg);
		}
	else if(hpipm_strcmp(field, "tol_eq")) 
		{
		DENSE_QCQP_IPM_ARG_SET_TOL_EQ(value, arg);
		}
	else if(hpipm_strcmp(field, "tol_ineq")) 
		{
		DENSE_QCQP_IPM_ARG_SET_TOL_INEQ(value, arg);
		}
	else if(hpipm_strcmp(field, "tol_comp")) 
		{
		DENSE_QCQP_IPM_ARG_SET_TOL_COMP(value, arg);
		}
	else if(hpipm_strcmp(field, "reg_prim")) 
		{
		DENSE_QCQP_IPM_ARG_SET_REG_PRIM(value, arg);
		}
	else if(hpipm_strcmp(field, "reg_dual")) 
		{
		DENSE_QCQP_IPM_ARG_SET_REG_DUAL(value, arg);
		}
	else if(hpipm_strcmp(field, "warm_start")) 
		{
		DENSE_QCQP_IPM_ARG_SET_WARM_START(value, arg);
		}
	else if(hpipm_strcmp(field, "pred_corr")) 
		{
		DENSE_QCQP_IPM_ARG_SET_PRED_CORR(value, arg);
		}
	else if(hpipm_strcmp(field, "cond_pred_corr")) 
		{
		DENSE_QCQP_IPM_ARG_SET_COND_PRED_CORR(value, arg);
		}
	else if(hpipm_strcmp(field, "comp_res_pred")) 
		{
		DENSE_QCQP_IPM_ARG_SET_COMP_RES_PRED(value, arg);
		}
	else if(hpipm_strcmp(field, "comp_res_exit")) 
		{
		DENSE_QCQP_IPM_ARG_SET_COMP_RES_EXIT(value, arg);
		}
	else if(hpipm_strcmp(field, "lam_min")) 
		{
		DENSE_QCQP_IPM_ARG_SET_LAM_MIN(value, arg);
		}
	else if(hpipm_strcmp(field, "t_min")) 
		{
		DENSE_QCQP_IPM_ARG_SET_T_MIN(value, arg);
		}
	else if(hpipm_strcmp(field, "split_step")) 
		{
		DENSE_QCQP_IPM_ARG_SET_SPLIT_STEP(value, arg);
		}
	else if(hpipm_strcmp(field, "t_lam_min")) 
		{
		DENSE_QCQP_IPM_ARG_SET_T_LAM_MIN(value, arg);
		}
	else
		{
		printf("error: DENSE_QCQP_IPM_ARG_SET: wrong field %s\n", field);
		exit(1);	
		}
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_ITER_MAX(int *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->iter_max = *value;
	DENSE_QP_IPM_ARG_SET_ITER_MAX(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_ALPHA_MIN(REAL *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->alpha_min = *value;
	DENSE_QP_IPM_ARG_SET_ALPHA_MIN(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_MU0(REAL *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->mu0 = *value;
	DENSE_QP_IPM_ARG_SET_MU0(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_TOL_STAT(REAL *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->res_g_max = *value;
	DENSE_QP_IPM_ARG_SET_TOL_STAT(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_TOL_EQ(REAL *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->res_b_max = *value;
	DENSE_QP_IPM_ARG_SET_TOL_EQ(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_TOL_INEQ(REAL *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->res_d_max = *value;
	DENSE_QP_IPM_ARG_SET_TOL_INEQ(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_TOL_COMP(REAL *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->res_m_max = *value;
	DENSE_QP_IPM_ARG_SET_TOL_COMP(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_REG_PRIM(REAL *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->reg_prim = *value;
	DENSE_QP_IPM_ARG_SET_REG_PRIM(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_REG_DUAL(REAL *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->reg_dual = *value;
	DENSE_QP_IPM_ARG_SET_REG_DUAL(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_WARM_START(int *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->warm_start = *value;
	DENSE_QP_IPM_ARG_SET_WARM_START(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_PRED_CORR(int *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->pred_corr = *value;
	DENSE_QP_IPM_ARG_SET_PRED_CORR(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_COND_PRED_CORR(int *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->cond_pred_corr = *value;
	DENSE_QP_IPM_ARG_SET_COND_PRED_CORR(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_COMP_RES_EXIT(int *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->comp_res_exit = *value;
	DENSE_QP_IPM_ARG_SET_COMP_RES_EXIT(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_COMP_RES_PRED(int *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->comp_res_pred = *value;
	DENSE_QP_IPM_ARG_SET_COMP_RES_PRED(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_LAM_MIN(REAL *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->lam_min = *value;
	DENSE_QP_IPM_ARG_SET_LAM_MIN(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_T_MIN(REAL *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->t_min = *value;
	DENSE_QP_IPM_ARG_SET_T_MIN(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_SPLIT_STEP(int *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->split_step = *value;
	DENSE_QP_IPM_ARG_SET_SPLIT_STEP(value, arg->qp_arg);
	return;
	}



void DENSE_QCQP_IPM_ARG_SET_T_LAM_MIN(int *value, struct DENSE_QCQP_IPM_ARG *arg)
	{
	arg->t_lam_min = *value;
	DENSE_QP_IPM_ARG_SET_T_LAM_MIN(value, arg->qp_arg);
	return;
	}



hpipm_size_t DENSE_QCQP_IPM_WS_MEMSIZE(struct DENSE_QCQP_DIM *dim, struct DENSE_QCQP_IPM_ARG *arg)
	{

	int nv = dim->nv;

	hpipm_size_t size = 0;

	size += 1*sizeof(struct DENSE_QP_IPM_WS);
	size += 1*DENSE_QP_IPM_WS_MEMSIZE(dim->qp_dim, arg->qp_arg);

	size += 1*sizeof(struct DENSE_QCQP_RES_WS); // qcqp_res_ws
	size += 1*DENSE_QCQP_RES_WS_MEMSIZE(dim); // qcqp_res_ws

	size += 1*sizeof(struct DENSE_QCQP_RES); // qcqp_res
	size += 1*DENSE_QCQP_RES_MEMSIZE(dim); // qcqp_res

	size += 1*sizeof(struct DENSE_QP); // qp
	size += 1*DENSE_QP_MEMSIZE(dim->qp_dim); // qp

	size += 1*sizeof(struct DENSE_QP_SOL); // qp_sol
	size += 1*DENSE_QP_SOL_MEMSIZE(dim->qp_dim); // qp_sol

	size += 2*sizeof(struct STRVEC); // tmp_nv
	size += 2*SIZE_STRVEC(nv); // tmp_nv

	size = (size+63)/64*64; // make multiple of typical cache line size
	size += 1*64; // align once to typical cache line size

	return size;

	}



void DENSE_QCQP_IPM_WS_CREATE(struct DENSE_QCQP_DIM *dim, struct DENSE_QCQP_IPM_ARG *arg, struct DENSE_QCQP_IPM_WS *workspace, void *mem)
	{

	int ii;

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = DENSE_QCQP_IPM_WS_MEMSIZE(dim, arg);
	hpipm_zero_memset(memsize, mem);

	int nv = dim->nv;

	char *c_ptr = mem;


	// structures
	workspace->qp_ws = (struct DENSE_QP_IPM_WS *) c_ptr;
	c_ptr += sizeof(struct DENSE_QP_IPM_WS);

	workspace->qp = (struct DENSE_QP *) c_ptr;
	c_ptr += sizeof(struct DENSE_QP);

	workspace->qp_sol = (struct DENSE_QP_SOL *) c_ptr;
	c_ptr += sizeof(struct DENSE_QP_SOL);

	workspace->qcqp_res_ws = (struct DENSE_QCQP_RES_WS *) c_ptr;
	c_ptr += sizeof(struct DENSE_QCQP_RES_WS);

	workspace->qcqp_res = (struct DENSE_QCQP_RES *) c_ptr;
	c_ptr += sizeof(struct DENSE_QCQP_RES);


	// vector struct
	struct STRVEC *sv_ptr = (struct STRVEC *) c_ptr;

	workspace->tmp_nv = sv_ptr;
	sv_ptr += 2;


	// align to typical cache line size
	hpipm_size_t s_ptr = (hpipm_size_t) sv_ptr;
	s_ptr = (s_ptr+63)/64*64;


	// memory of structures
	c_ptr = (char *) s_ptr;

	DENSE_QP_IPM_WS_CREATE(dim->qp_dim, arg->qp_arg, workspace->qp_ws, c_ptr);
	c_ptr += workspace->qp_ws->memsize;

	DENSE_QP_CREATE(dim->qp_dim, workspace->qp, c_ptr);
	c_ptr += workspace->qp->memsize;

	DENSE_QP_SOL_CREATE(dim->qp_dim, workspace->qp_sol, c_ptr);
	c_ptr += workspace->qp_sol->memsize;

	DENSE_QCQP_RES_WS_CREATE(dim, workspace->qcqp_res_ws, c_ptr);
	c_ptr += workspace->qcqp_res_ws->memsize;

	DENSE_QCQP_RES_CREATE(dim, workspace->qcqp_res, c_ptr);
	c_ptr += workspace->qcqp_res->memsize;

	CREATE_STRVEC(nv, workspace->tmp_nv+0, c_ptr);
	c_ptr += (workspace->tmp_nv+0)->memsize;
	CREATE_STRVEC(nv, workspace->tmp_nv+1, c_ptr);
	c_ptr += (workspace->tmp_nv+1)->memsize;


	//
	workspace->memsize = DENSE_QCQP_IPM_WS_MEMSIZE(dim, arg);


#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + workspace->memsize)
		{
		printf("\nCreate_dense_qp_ipm: outside memory bounds!\n\n");
		exit(1);
		}
#endif

	return;

	}



void DENSE_QCQP_IPM_GET(char *field, struct DENSE_QCQP_IPM_WS *ws, void *value)
	{
	if(hpipm_strcmp(field, "status"))
		{ 
		DENSE_QCQP_IPM_GET_STATUS(ws, value);
		}
	else if(hpipm_strcmp(field, "iter"))
		{ 
		DENSE_QCQP_IPM_GET_ITER(ws, value);
		}
	else if(hpipm_strcmp(field, "max_res_stat"))
		{ 
		DENSE_QCQP_IPM_GET_MAX_RES_STAT(ws, value);
		}
	else if(hpipm_strcmp(field, "max_res_eq"))
		{ 
		DENSE_QCQP_IPM_GET_MAX_RES_EQ(ws, value);
		}
	else if(hpipm_strcmp(field, "max_res_ineq"))
		{ 
		DENSE_QCQP_IPM_GET_MAX_RES_INEQ(ws, value);
		}
	else if(hpipm_strcmp(field, "max_res_comp"))
		{ 
		DENSE_QCQP_IPM_GET_MAX_RES_COMP(ws, value);
		}
	else if(hpipm_strcmp(field, "stat"))
		{ 
		DENSE_QCQP_IPM_GET_STAT(ws, value);
		}
	else if(hpipm_strcmp(field, "stat_m"))
		{ 
		DENSE_QCQP_IPM_GET_STAT_M(ws, value);
		}
	else 
		{
		printf("error: DENSE_QCQP_IPM_GET: wrong field %s\n", field);
		exit(1);
		}
	return;
	}



void DENSE_QCQP_IPM_GET_STATUS(struct DENSE_QCQP_IPM_WS *ws, int *status)
	{
	*status = ws->status;
	return;
	}



void DENSE_QCQP_IPM_GET_ITER(struct DENSE_QCQP_IPM_WS *ws, int *iter)
	{
	*iter = ws->iter;
	return;
	}



void DENSE_QCQP_IPM_GET_MAX_RES_STAT(struct DENSE_QCQP_IPM_WS *ws, REAL *res_stat)
	{
	*res_stat = ws->qcqp_res->res_max[0];
	return;
	}



void DENSE_QCQP_IPM_GET_MAX_RES_EQ(struct DENSE_QCQP_IPM_WS *ws, REAL *res_eq)
	{
	*res_eq = ws->qcqp_res->res_max[1];
	return;
	}



void DENSE_QCQP_IPM_GET_MAX_RES_INEQ(struct DENSE_QCQP_IPM_WS *ws, REAL *res_ineq)
	{
	*res_ineq = ws->qcqp_res->res_max[2];
	return;
	}



void DENSE_QCQP_IPM_GET_MAX_RES_COMP(struct DENSE_QCQP_IPM_WS *ws, REAL *res_comp)
	{
	*res_comp = ws->qcqp_res->res_max[3];
	return;
	}



void DENSE_QCQP_IPM_GET_STAT(struct DENSE_QCQP_IPM_WS *ws, REAL **stat)
	{
	DENSE_QP_IPM_GET_STAT(ws->qp_ws, stat);
	}



void DENSE_QCQP_IPM_GET_STAT_M(struct DENSE_QCQP_IPM_WS *ws, int *stat_m)
	{
	DENSE_QP_IPM_GET_STAT_M(ws->qp_ws, stat_m);
	}



void DENSE_QCQP_INIT_VAR(struct DENSE_QCQP *qcqp, struct DENSE_QCQP_SOL *qcqp_sol, struct DENSE_QCQP_IPM_ARG *arg, struct DENSE_QCQP_IPM_WS *ws)
	{

//	struct CORE_QCQP_IPM_WORKSPACE *cws = ws->core_workspace;

	// extract cws members
	int nv = qcqp->dim->nv;
	int ne = qcqp->dim->ne;
	int nb = qcqp->dim->nb;
	int ng = qcqp->dim->ng;
	int nq = qcqp->dim->nq;
	int ns = qcqp->dim->ns;

	REAL *d = qcqp->d->pa;
	int *idxb = qcqp->idxb;

	REAL *v = qcqp_sol->v->pa;
	REAL *pi = qcqp_sol->pi->pa;
	REAL *lam = qcqp_sol->lam->pa;
	REAL *t = qcqp_sol->t->pa;

	REAL mu0 = arg->mu0;

	// local variables
	int ii;
	int idxb0;
	REAL tmp;

	// TODO move to args ???
	REAL thr0 = 0.5;


	// primal and dual variables
	if(arg->warm_start==2)
		{

		thr0 = 1e-1;

		for(ii=0; ii<2*nb+2*ng+2*nq+2*ns; ii++)
			{
			if(lam[ii]<thr0)
				lam[ii] = thr0;
			if(t[ii]<thr0)
				t[ii] = thr0;
			}


		return;

		}


	// primal variables
	if(arg->warm_start==0)
		{
		// cold start
		for(ii=0; ii<nv+2*ns; ii++)
			{
			v[ii] = 0.0;
			}
		}
		
	// equality constraints
	for(ii=0; ii<ne; ii++)
		{
		pi[ii] = 0.0;
		}
	
	// box constraints
	for(ii=0; ii<nb; ii++)
		{
#if 1
		idxb0 = idxb[ii];
		t[0+ii]        = - d[0+ii]        + v[idxb0];
		t[nb+ng+nq+ii] = - d[nb+ng+nq+ii] - v[idxb0];
		if(t[0+ii]<thr0)
			{
			if(t[nb+ng+nq+ii]<thr0)
				{
				v[idxb0] = 0.5*(d[0+ii] + d[nb+ng+nq+ii]);
				t[0+ii]        = thr0;
				t[nb+ng+nq+ii] = thr0;
				}
			else
				{
				t[0+ii] = thr0;
				v[idxb0] = d[0+ii] + thr0;
				}
			}
		else if(t[nb+ng+nq+ii]<thr0)
			{
			t[nb+ng+nq+ii] = thr0;
			v[idxb0] = - d[nb+ng+nq+ii] - thr0;
			}
#else
		t[0+ii]     = 1.0;
		t[nb+ng+nq+ii] = 1.0;
#endif
		lam[0+ii]        = mu0/t[0+ii];
		lam[nb+ng+nq+ii] = mu0/t[nb+ng+nq+ii];
		}
	
	// general constraints
	if(ng>0)
		{
		GEMV_T(nv, ng, 1.0, qcqp->Ct, 0, 0, qcqp_sol->v, 0, 0.0, qcqp_sol->t, nb, qcqp_sol->t, nb);
		for(ii=0; ii<ng; ii++)
			{
#if 1
			t[2*nb+ng+nq+ii] = t[nb+ii];
			t[nb+ii]      -= d[nb+ii];
			t[2*nb+ng+nq+ii] -= d[2*nb+ng+nq+ii];
	//		t[nb+ii]      = fmax( thr0, t[nb+ii] );
	//		t[2*nb+ng+nq+ii] = fmax( thr0, t[2*nb+ng+nq+ii] );
			t[nb+ii]      = thr0>t[nb+ii]      ? thr0 : t[nb+ii];
			t[2*nb+ng+nq+ii] = thr0>t[2*nb+ng+nq+ii] ? thr0 : t[2*nb+ng+nq+ii];
#else
			t[nb+ii]      = 1.0;
			t[2*nb+ng+nq+ii] = 1.0;
#endif
			lam[nb+ii]      = mu0/t[nb+ii];
			lam[2*nb+ng+nq+ii] = mu0/t[2*nb+ng+nq+ii];
			}
		}

	// soft constraints
	for(ii=0; ii<ns; ii++)
		{
		t[2*nb+2*ng+2*nq+ii]    = 1.0; // thr0;
		t[2*nb+2*ng+2*nq+ns+ii] = 1.0; // thr0;
		lam[2*nb+2*ng+2*nq+ii]    = mu0/t[2*nb+2*ng+2*nq+ii];
		lam[2*nb+2*ng+2*nq+ns+ii] = mu0/t[2*nb+2*ng+2*nq+ns+ii];
		}

	//  quadratic constraints
	REAL sqrt_mu0 = sqrt(mu0);
	sqrt_mu0 = thr0>sqrt_mu0 ? thr0 : sqrt_mu0;
	REAL mu0_div_sqrt_mu0 = mu0 / sqrt_mu0;

	for(ii=0; ii<nq; ii++)
		{
		// disregard lower
		lam[nb+ng+ii] = 0.0;
		t[nb+ng+ii]   = 1.0;
		// upper
#if 1
		t[2*nb+2*ng+nq+ii]   = sqrt_mu0;
		lam[2*nb+2*ng+nq+ii] = mu0_div_sqrt_mu0;
#else
//		t[2*nb+2*ng+nq+ii] = 1.0; // thr0;
		COLEX(nv, qcqp->Ct, 0, ng+ii, ws->tmp_nv, 0);
		SYMV_L(nv, 0.5, qcqp->Hq+ii, 0, 0, qcqp_sol->v, 0, 1.0, ws->tmp_nv, 0, ws->tmp_nv, 0);
		tmp = DOT(nv, ws->tmp_nv, 0, qcqp_sol->v, 0);
		tmp = - d[2*nb+2*ng+nq+ii] - tmp;
		t[2*nb+2*ng+nq+ii] = thr0>tmp ? thr0 : tmp;
		lam[2*nb+2*ng+nq+ii]  = mu0/t[2*nb+2*ng+nq+ii];
#endif
		}

	// TODO rewrite all the above taking some pointers to key parts, e.g. lam_lb, lam_ub, and make relative to them

	return;

	}



void DENSE_QCQP_APPROX_QP(struct DENSE_QCQP *qcqp, struct DENSE_QCQP_SOL *qcqp_sol, struct DENSE_QP *qp, struct DENSE_QCQP_IPM_WS *ws)
	{

	int nv = qcqp->dim->nv;
	int ne = qcqp->dim->ne;
	int nb = qcqp->dim->nb;
	int ng = qcqp->dim->ng;
	int nq = qcqp->dim->nq;
	int ns = qcqp->dim->ns;

	REAL tmp;

	int ii;


	VECCP(2*nb+2*ng+2*nq+2*ns, qcqp->d, 0, qp->d, 0);

	GECP(nv, nv, qcqp->Hv, 0, 0, qp->Hv, 0, 0);

	VECSE(nv, 0.0, ws->qcqp_res_ws->q_adj, 0);

	for(ii=0; ii<nq; ii++)
		{
#ifdef DOUBLE_PRECISION
		tmp = - BLASFEO_DVECEL(qcqp_sol->lam, nb+ng+ii) + BLASFEO_DVECEL(qcqp_sol->lam, 2*nb+2*ng+nq+ii);
#else
		tmp = - BLASFEO_SVECEL(qcqp_sol->lam, nb+ng+ii) + BLASFEO_SVECEL(qcqp_sol->lam, 2*nb+2*ng+nq+ii);
#endif
		GEAD(nv, nv, tmp, qcqp->Hq+ii, 0, 0, qp->Hv, 0, 0);

		SYMV_L(nv, 1.0, qcqp->Hq+ii, 0, 0, qcqp_sol->v, 0, 0.0, ws->tmp_nv+0, 0, ws->tmp_nv+0, 0);
		COLEX(nv, qcqp->Ct, 0, ng+ii, ws->tmp_nv+1, 0);
		AXPY(nv, 1.0, ws->tmp_nv+0, 0, ws->tmp_nv+1, 0, ws->tmp_nv+1, 0);
		COLIN(nv, ws->tmp_nv+1, 0, qp->Ct, 0, ng+ii);
		AXPY(nv, tmp, ws->tmp_nv+1, 0, ws->qcqp_res_ws->q_adj, 0, ws->qcqp_res_ws->q_adj, 0);

		COLEX(nv, qcqp->Ct, 0, ng+ii, ws->tmp_nv+1, 0);
		AXPY(nv, 0.5, ws->tmp_nv+0, 0, ws->tmp_nv+1, 0, ws->tmp_nv+1, 0);
		tmp = DOT(nv, ws->tmp_nv+1, 0, qcqp_sol->v, 0);
#ifdef DOUBLE_PRECISION
		// TODO maybe swap signs?
		BLASFEO_DVECEL(qp->d, nb+ng+ii) += - tmp;
		BLASFEO_DVECEL(qp->d, 2*nb+2*ng+nq+ii) += + tmp;
		BLASFEO_DVECEL(ws->qcqp_res_ws->q_fun, ii) = tmp;
#else
		BLASFEO_SVECEL(qp->d, nb+ng+ii) += - tmp;
		BLASFEO_SVECEL(qp->d, 2*nb+2*ng+nq+ii) += + tmp;
		BLASFEO_SVECEL(ws->qcqp_res_ws->q_fun, ii) = tmp;
#endif
		}

	VECCP(2*nb+2*ng+2*nq+2*ns, qcqp->d_mask, 0, qp->d_mask, 0);

	GECP(ne, nv, qcqp->A, 0, 0, qp->A, 0, 0);

	GECP(nv, ng, qcqp->Ct, 0, 0, qp->Ct, 0, 0);

	VECCP(nv+2*ns, qcqp->gz, 0, qp->gz, 0);

	VECCP(ne, qcqp->b, 0, qp->b, 0);

	VECCP(2*nb+2*ng+2*nq+2*ns, qcqp->m, 0, qp->m, 0);

	VECCP(2*ns, qcqp->Z, 0, qp->Z, 0);

	for(ii=0; ii<nb; ii++)
		qp->idxb[ii] = qcqp->idxb[ii];

	for(ii=0; ii<nb+ng+nq; ii++)
		qp->idxs_rev[ii] = qcqp->idxs_rev[ii];

	return;

	}



void DENSE_QCQP_UPDATE_QP(struct DENSE_QCQP *qcqp, struct DENSE_QCQP_SOL *qcqp_sol, struct DENSE_QP *qp, struct DENSE_QCQP_IPM_WS *ws)
	{

	int nv = qcqp->dim->nv;
	int ne = qcqp->dim->ne;
	int nb = qcqp->dim->nb;
	int ng = qcqp->dim->ng;
	int nq = qcqp->dim->nq;
	int ns = qcqp->dim->ns;

	REAL tmp;

	int ii;

	// TODO only the 2*nq part needed !!!!!
	VECCP(2*nb+2*ng+2*nq+2*ns, qcqp->d, 0, qp->d, 0);

	GECP(nv, nv, qcqp->Hv, 0, 0, qp->Hv, 0, 0);

	VECSE(nv, 0.0, ws->qcqp_res_ws->q_adj, 0);

	for(ii=0; ii<nq; ii++)
		{
#ifdef DOUBLE_PRECISION
		tmp = - BLASFEO_DVECEL(qcqp_sol->lam, nb+ng+ii) + BLASFEO_DVECEL(qcqp_sol->lam, 2*nb+2*ng+nq+ii);
#else
		tmp = - BLASFEO_SVECEL(qcqp_sol->lam, nb+ng+ii) + BLASFEO_SVECEL(qcqp_sol->lam, 2*nb+2*ng+nq+ii);
#endif
		GEAD(nv, nv, tmp, qcqp->Hq+ii, 0, 0, qp->Hv, 0, 0);

		SYMV_L(nv, 1.0, qcqp->Hq+ii, 0, 0, qcqp_sol->v, 0, 0.0, ws->tmp_nv+0, 0, ws->tmp_nv+0, 0);
		COLEX(nv, qcqp->Ct, 0, ng+ii, ws->tmp_nv+1, 0);
		AXPY(nv, 1.0, ws->tmp_nv+0, 0, ws->tmp_nv+1, 0, ws->tmp_nv+1, 0);
		COLIN(nv, ws->tmp_nv+1, 0, qp->Ct, 0, ng+ii);
		AXPY(nv, tmp, ws->tmp_nv+1, 0, ws->qcqp_res_ws->q_adj, 0, ws->qcqp_res_ws->q_adj, 0);

		COLEX(nv, qcqp->Ct, 0, ng+ii, ws->tmp_nv+1, 0);
		AXPY(nv, 0.5, ws->tmp_nv+0, 0, ws->tmp_nv+1, 0, ws->tmp_nv+1, 0);
		tmp = DOT(nv, ws->tmp_nv+1, 0, qcqp_sol->v, 0);
#ifdef DOUBLE_PRECISION
		// TODO maybe swap signs?
		BLASFEO_DVECEL(qp->d, nb+ng+ii) += - tmp;
		BLASFEO_DVECEL(qp->d, 2*nb+2*ng+nq+ii) += + tmp;
		BLASFEO_DVECEL(ws->qcqp_res_ws->q_fun, ii) = tmp;
#else
		BLASFEO_SVECEL(qp->d, nb+ng+ii) += - tmp;
		BLASFEO_SVECEL(qp->d, 2*nb+2*ng+nq+ii) += + tmp;
		BLASFEO_DVECEL(ws->qcqp_res_ws->q_fun, ii) = tmp;
#endif
		}

	// TODO needed ?????
	VECCP(2*nb+2*ng+2*nq+2*ns, qcqp->m, 0, qp->m, 0);

	return;

	}



void DENSE_QCQP_UPDATE_QP_ABS_STEP(struct DENSE_QCQP *qcqp, struct DENSE_QCQP_SOL *qcqp_sol, struct DENSE_QP *qp, struct DENSE_QCQP_IPM_WS *ws)
	{

	int nv = qcqp->dim->nv;
	int ne = qcqp->dim->ne;
	int nb = qcqp->dim->nb;
	int ng = qcqp->dim->ng;
	int nq = qcqp->dim->nq;
	int ns = qcqp->dim->ns;

	REAL tmp;

	int ii;

	VECCP(2*nb+2*ng+2*nq+2*ns, qcqp->d, 0, qp->d, 0);

	GECP(nv, nv, qcqp->Hv, 0, 0, qp->Hv, 0, 0);

	VECSE(nv, 0.0, ws->qcqp_res_ws->q_adj, 0);

	for(ii=0; ii<nq; ii++)
		{
#ifdef DOUBLE_PRECISION
		tmp = - BLASFEO_DVECEL(qcqp_sol->lam, nb+ng+ii) + BLASFEO_DVECEL(qcqp_sol->lam, 2*nb+2*ng+nq+ii);
#else
		tmp = - BLASFEO_SVECEL(qcqp_sol->lam, nb+ng+ii) + BLASFEO_SVECEL(qcqp_sol->lam, 2*nb+2*ng+nq+ii);
#endif
		GEAD(nv, nv, tmp, qcqp->Hq+ii, 0, 0, qp->Hv, 0, 0);

		SYMV_L(nv, 1.0, qcqp->Hq+ii, 0, 0, qcqp_sol->v, 0, 0.0, ws->tmp_nv+0, 0, ws->tmp_nv+0, 0);
		COLEX(nv, qcqp->Ct, 0, ng+ii, ws->tmp_nv+1, 0);
		AXPY(nv, 1.0, ws->tmp_nv+0, 0, ws->tmp_nv+1, 0, ws->tmp_nv+1, 0);
		COLIN(nv, ws->tmp_nv+1, 0, qp->Ct, 0, ng+ii);
		AXPY(nv, tmp, ws->tmp_nv+1, 0, ws->qcqp_res_ws->q_adj, 0, ws->qcqp_res_ws->q_adj, 0);

////		AXPY(nv, 0.5, ws->tmp_nv+0, 0, qcqp->gq+ii, 0, ws->tmp_nv+1, 0);
//		AXPY(nv, 0.5, ws->tmp_nv+0, 0, qcqp->gq+ii, 0, ws->tmp_nv+0, 0);
//		AXPY(nv, -1.0, ws->tmp_nv+1, 0, ws->tmp_nv+0, 0, ws->tmp_nv+1, 0);
		AXPBY(nv, -1.0, ws->tmp_nv+1, 0, 0.5, ws->tmp_nv+0, 0, ws->tmp_nv+1, 0);
		COLEX(nv, qcqp->Ct, 0, ng+ii, ws->tmp_nv+0, 0);
		AXPY(nv, 1.0, ws->tmp_nv+0, 0, ws->tmp_nv+1, 0, ws->tmp_nv+1, 0);
		tmp = DOT(nv, ws->tmp_nv+1, 0, qcqp_sol->v, 0);
#ifdef DOUBLE_PRECISION
		// TODO maybe swap signs?
		BLASFEO_DVECEL(qp->d, nb+ng+ii) += - tmp;
		BLASFEO_DVECEL(qp->d, 2*nb+2*ng+nq+ii) += + tmp;
		BLASFEO_DVECEL(ws->qcqp_res_ws->q_fun, ii) = tmp;
#else
		BLASFEO_SVECEL(qp->d, nb+ng+ii) += - tmp;
		BLASFEO_SVECEL(qp->d, 2*nb+2*ng+nq+ii) += + tmp;
		BLASFEO_SVECEL(ws->qcqp_res_ws->q_fun, ii) = tmp;
#endif
		}

	VECCP(2*nb+2*ng+2*nq+2*ns, qcqp->m, 0, qp->m, 0);

	return;

	}



void DENSE_QCQP_SOL_CONV_QP_SOL(struct DENSE_QCQP_SOL *qcqp_sol, struct DENSE_QP_SOL *qp_sol)
	{

	int nv = qcqp_sol->dim->nv;
	int ne = qcqp_sol->dim->ne;
	int nb = qcqp_sol->dim->nb;
	int ng = qcqp_sol->dim->ng;
	int nq = qcqp_sol->dim->nq;
	int ns = qcqp_sol->dim->ns;

	VECCP(nv+2*ns, qcqp_sol->v, 0, qp_sol->v, 0);

	VECCP(ne, qcqp_sol->pi, 0, qp_sol->pi, 0);

	VECCP(2*nb+2*ng+2*nq+2*ns, qcqp_sol->lam, 0, qp_sol->lam, 0);

	VECCP(2*nb+2*ng+2*nq+2*ns, qcqp_sol->t, 0, qp_sol->t, 0);

	return;

	}



void DENSE_QP_SOL_CONV_QCQP_SOL(struct DENSE_QP_SOL *qp_sol, struct DENSE_QCQP_SOL *qcqp_sol)
	{

	int nv = qcqp_sol->dim->nv;
	int ne = qcqp_sol->dim->ne;
	int nb = qcqp_sol->dim->nb;
	int ng = qcqp_sol->dim->ng;
	int nq = qcqp_sol->dim->nq;
	int ns = qcqp_sol->dim->ns;

	VECCP(nv+2*ns, qp_sol->v, 0, qcqp_sol->v, 0);

	VECCP(ne, qp_sol->pi, 0, qcqp_sol->pi, 0);

	VECCP(2*nb+2*ng+2*nq+2*ns, qp_sol->lam, 0, qcqp_sol->lam, 0);

	VECCP(2*nb+2*ng+2*nq+2*ns, qp_sol->t, 0, qcqp_sol->t, 0);

	return;

	}



void DENSE_QCQP_RES_CONV_QP_RES(struct DENSE_QCQP_RES *qcqp_res, struct DENSE_QP_RES *qp_res)
	{

	int nv = qcqp_res->dim->nv;
	int ne = qcqp_res->dim->ne;
	int nb = qcqp_res->dim->nb;
	int ng = qcqp_res->dim->ng;
	int nq = qcqp_res->dim->nq;
	int ns = qcqp_res->dim->ns;

	VECCP(nv+2*ns, qcqp_res->res_g, 0, qp_res->res_g, 0);

	VECCP(ne, qcqp_res->res_b, 0, qp_res->res_b, 0);

	VECCP(2*nb+2*ng+2*nq+2*ns, qcqp_res->res_d, 0, qp_res->res_d, 0);

	VECCP(2*nb+2*ng+2*nq+2*ns, qcqp_res->res_m, 0, qp_res->res_m, 0);

	qp_res->res_mu = qcqp_res->res_mu;

	return;

	}



void DENSE_QCQP_IPM_SOLVE(struct DENSE_QCQP *qcqp, struct DENSE_QCQP_SOL *qcqp_sol, struct DENSE_QCQP_IPM_ARG *qcqp_arg, struct DENSE_QCQP_IPM_WS *qcqp_ws)
	{

	int nv = qcqp->dim->nv;
	int ne = qcqp->dim->ne;
	int nb = qcqp->dim->nb;
	int ng = qcqp->dim->ng;
	int nq = qcqp->dim->nq;
	int ns = qcqp->dim->ns;

	// extract stuff

	struct DENSE_QP *qp = qcqp_ws->qp;
	struct DENSE_QP_SOL *qp_sol = qcqp_ws->qp_sol;
	struct DENSE_QP_IPM_WS *qp_ws = qcqp_ws->qp_ws;
	struct DENSE_QP_IPM_ARG *qp_arg = qcqp_arg->qp_arg;

	struct DENSE_QCQP_DIM *qcqp_dim = qcqp->dim;
	struct DENSE_QCQP_RES *qcqp_res = qcqp_ws->qcqp_res;
	struct DENSE_QCQP_RES_WS *qcqp_res_ws = qcqp_ws->qcqp_res_ws;

	struct CORE_QP_IPM_WORKSPACE *cws = qp_ws->core_workspace;

	int kk, ii, idx;
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
	cws->t_min_inv = qp_arg->t_min>0.0 ? 1.0/qp_arg->t_min : 1e30;
	cws->split_step = qp_arg->split_step;
	cws->t_lam_min = qp_arg->t_lam_min;

	// alias qp vectors into qp_sol
	cws->v = qp_sol->v->pa;
	cws->pi = qp_sol->pi->pa;
	cws->lam = qp_sol->lam->pa;
	cws->t = qp_sol->t->pa;

	// alias members of qp_step
	qp_ws->qp_step->dim = qp->dim;
	qp_ws->qp_step->Hv = qp->Hv;
	qp_ws->qp_step->A = qp->A;
	qp_ws->qp_step->Ct = qp->Ct;
	qp_ws->qp_step->Z = qp->Z;
	qp_ws->qp_step->idxb = qp->idxb;
	qp_ws->qp_step->idxs_rev = qp->idxs_rev;
	qp_ws->qp_step->gz = qp_ws->res->res_g;
	qp_ws->qp_step->b = qp_ws->res->res_b;
	qp_ws->qp_step->d = qp_ws->res->res_d;
	qp_ws->qp_step->m = qp_ws->res->res_m;
	qp_ws->qp_step->d_mask = qp->d_mask;

	// alias members of qp_itref
	qp_ws->qp_itref->dim = qp->dim;
	qp_ws->qp_itref->Hv = qp->Hv;
	qp_ws->qp_itref->A = qp->A;
	qp_ws->qp_itref->Ct = qp->Ct;
	qp_ws->qp_itref->Z = qp->Z;
	qp_ws->qp_itref->idxb = qp->idxb;
	qp_ws->qp_itref->idxs_rev = qp->idxs_rev;
	qp_ws->qp_itref->gz = qp_ws->res_itref->res_g;
	qp_ws->qp_itref->b = qp_ws->res_itref->res_b;
	qp_ws->qp_itref->d = qp_ws->res_itref->res_d;
	qp_ws->qp_itref->m = qp_ws->res_itref->res_m;
	qp_ws->qp_itref->d_mask = qp->d_mask;

	REAL *qcqp_res_max = qcqp_res->res_max;

	qp_ws->use_A_fact = 0;

	// cache q_fun & q_adj from approx/update for res
	qcqp_ws->qcqp_res_ws->use_q_fun = 1;
	qcqp_ws->qcqp_res_ws->use_q_adj = 1;


	// disregard soft constr on (disregarded) lower quard constr
	VECSE(nq, 0.0, qcqp->d_mask, nb+ng); // TODO needed ???
	// TODO probably remove when using only idxs_rev, as the same slack may be associated with other constraints !!!!!
	for(ii=0; ii<nq; ii++)
		{
		idx = qcqp->idxs_rev[nb+ng+ii];
		if(idx!=-1)
			{
#ifdef DOUBLE_PRECISION
			BLASFEO_DVECEL(qcqp->d_mask, 2*nb+2*ng+2*nq+idx) = 0.0;
#else
			BLASFEO_DVECEL(qcqp->d_mask, 2*nb+2*ng+2*nq+idx) = 0.0;
#endif
			}
		}


	// initialize qcqp & qp
	DENSE_QCQP_INIT_VAR(qcqp, qcqp_sol, qcqp_arg, qcqp_ws);
	DENSE_QCQP_SOL_CONV_QP_SOL(qcqp_sol, qp_sol);

	// approximate qcqp with a qp
	DENSE_QCQP_APPROX_QP(qcqp, qcqp_sol, qp, qcqp_ws);


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
		FACT_SOLVE_KKT_UNCONSTR_DENSE_QP(qp, qp_sol, qp_arg, qp_ws);
		DENSE_QP_SOL_CONV_QCQP_SOL(qp_sol, qcqp_sol);
		if(qp_arg->comp_res_exit)
			{
			// compute residuals
			DENSE_QCQP_RES_COMPUTE(qcqp, qcqp_sol, qcqp_res, qcqp_res_ws);
			// XXX no constraints, so no mask
			DENSE_QCQP_RES_COMPUTE_INF_NORM(qcqp_res);
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
		qp_ws->qp_step->Hv = qp->Hv;
		qp_ws->qp_step->A = qp->A;
		qp_ws->qp_step->Ct = qp->Ct;
		qp_ws->qp_step->Z = qp->Z;
		qp_ws->qp_step->idxb = qp->idxb;
		qp_ws->qp_step->idxs_rev = qp->idxs_rev;
		qp_ws->qp_step->gz = qp->gz;
		qp_ws->qp_step->b = qp->b;
		qp_ws->qp_step->d = qp->d;
		qp_ws->qp_step->m = qp_ws->tmp_m;

		// alias core workspace
		cws->res_m = qp_ws->qp_step->m->pa;

		// update approximation of qcqp as qp for absolute step
		DENSE_QCQP_UPDATE_QP_ABS_STEP(qcqp, qcqp_sol, qp, qcqp_ws);

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
			DENSE_QP_IPM_ABS_STEP(kk, qp, qp_sol, qp_arg, qp_ws);
//blasfeo_print_exp_tran_dvec(cws->nc, qp_sol->lam, 0);
			DENSE_QP_SOL_CONV_QCQP_SOL(qp_sol, qcqp_sol);

			// update approximation of qcqp as qp for absolute step
			DENSE_QCQP_UPDATE_QP_ABS_STEP(qcqp, qcqp_sol, qp, qcqp_ws);

			// compute mu
			mu = VECMULDOT(cws->nc, qcqp_sol->lam, 0, qcqp_sol->t, 0, qp_ws->tmp_m, 0);
			mu /= cws->nc;
			cws->mu = mu;
			if(kk+1<stat_max)
				stat[stat_m*(kk+1)+5] = mu;

			}

		if(qp_arg->comp_res_exit)
			{
			// compute residuals
			DENSE_QCQP_RES_COMPUTE(qcqp, qcqp_sol, qcqp_res, qcqp_res_ws);
			if(qp_ws->mask_constr)
				{
				// mask out disregarded constraints
				VECMUL(2*ns, qp->d_mask, 2*nb+2*ng+2*nq, qcqp_res->res_g, nv, qcqp_res->res_g, nv);
				VECMUL(cws->nc, qp->d_mask, 0, qcqp_res->res_d, 0, qcqp_res->res_d, 0);
				VECMUL(cws->nc, qp->d_mask, 0, qcqp_res->res_m, 0, qcqp_res->res_m, 0);
				}
			DENSE_QCQP_RES_COMPUTE_INF_NORM(qcqp_res);
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
	DENSE_QCQP_RES_COMPUTE(qcqp, qcqp_sol, qcqp_res, qcqp_res_ws);
	if(qp_ws->mask_constr)
		{
		// mask out disregarded constraints
		VECMUL(2*ns, qp->d_mask, 2*nb+2*ng+2*nq, qcqp_res->res_g, nv, qcqp_res->res_g, nv);
		VECMUL(cws->nc, qp->d_mask, 0, qcqp_res->res_d, 0, qcqp_res->res_d, 0);
		VECMUL(cws->nc, qp->d_mask, 0, qcqp_res->res_m, 0, qcqp_res->res_m, 0);
		}
	DENSE_QCQP_RES_COMPUTE_INF_NORM(qcqp_res);
	DENSE_QCQP_RES_CONV_QP_RES(qcqp_res, qp_ws->res);
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
		qp_ws->use_hess_fact = 0;

		// compute delta step
		DENSE_QP_IPM_DELTA_STEP(kk, qp, qp_sol, qp_arg, qp_ws);
//blasfeo_print_exp_tran_dvec(cws->nc, qp_sol->lam, 0);
		DENSE_QP_SOL_CONV_QCQP_SOL(qp_sol, qcqp_sol);
		// XXX maybe not needed
		if(qp_ws->mask_constr)
			{
			// mask out disregarded constraints
			VECMUL(cws->nc, qp->d_mask, 0, qcqp_sol->lam, 0, qcqp_sol->lam, 0);
			}

		// update approximation of qcqp as qp
		DENSE_QCQP_UPDATE_QP(qcqp, qcqp_sol, qp, qcqp_ws);

		// compute residuals
		DENSE_QCQP_RES_COMPUTE(qcqp, qcqp_sol, qcqp_res, qcqp_res_ws);
		if(qp_ws->mask_constr)
			{
			// mask out disregarded constraints
			VECMUL(2*ns, qp->d_mask, 2*nb+2*ng+2*nq, qcqp_res->res_g, nv, qcqp_res->res_g, nv);
			VECMUL(cws->nc, qp->d_mask, 0, qcqp_res->res_d, 0, qcqp_res->res_d, 0);
			VECMUL(cws->nc, qp->d_mask, 0, qcqp_res->res_m, 0, qcqp_res->res_m, 0);
			}
		DENSE_QCQP_RES_COMPUTE_INF_NORM(qcqp_res);
		DENSE_QCQP_RES_CONV_QP_RES(qcqp_res, qp_ws->res);
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

