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



hpipm_size_t SIM_ERK_ARG_MEMSIZE()
	{
	return 0;
	}



void SIM_ERK_ARG_CREATE(struct SIM_ERK_ARG *erk_arg, void *mem)
	{

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = SIM_ERK_ARG_MEMSIZE();
	hpipm_zero_memset(memsize, mem);

	erk_arg->memsize = memsize;

	return;

	}



void SIM_ERK_ARG_SET_ALL(struct SIM_RK_DATA *rk_data, REAL h, int steps, struct SIM_ERK_ARG *erk_arg)
	{

	erk_arg->rk_data = rk_data;
	erk_arg->h = h;
	erk_arg->steps = steps;
//	erk_arg->for_sens = for_sens;
//	erk_arg->adj_sens = adj_sens;

	return;

	}



hpipm_size_t SIM_ERK_WS_MEMSIZE(struct SIM_ERK_ARG *erk_arg, int nx, int np, int nf_max, int na_max)
	{

	int ns = erk_arg->rk_data->ns;

	int nX = nx*(1+nf_max);

	int steps = erk_arg->steps;

	hpipm_size_t size = 0;

	size += 1*np*sizeof(REAL); // p
	size += 1*nX*sizeof(REAL); // x_for
	size += 1*ns*nX*sizeof(REAL); // K
	size += 1*nX*sizeof(REAL); // x_tmp
	if(na_max>0)
		{
//		size += 1*nX*(steps+1)*sizeof(REAL); // x_traj XXX
		size += 1*nx*(ns*steps+1)*sizeof(REAL); // x_traj
//		size += 1*ns*nX*steps*sizeof(REAL); // K
//		size += 1*nX*ns*sizeof(REAL); // x_tmp
		size += 1*nf_max*(steps+1)*sizeof(REAL); // l // XXX *na_max ???
		size += 1*(nx+nf_max)*sizeof(REAL); // adj_in // XXX *na_max ???
		size += 1*nf_max*ns*sizeof(REAL); // adj_tmp // XXX *na_max ???
		}

	return size;

	}



void SIM_ERK_WS_CREATE(struct SIM_ERK_ARG *erk_arg, int nx, int np, int nf_max, int na_max, struct SIM_ERK_WS *work, void *mem)
	{

	// zero memory (to avoid corrupted memory like e.g. NaN)
	hpipm_size_t memsize = SIM_ERK_WS_MEMSIZE(erk_arg, nx, np, nf_max, na_max);
	hpipm_zero_memset(memsize, mem);

	work->erk_arg = erk_arg;
	work->nx = nx;
	work->np = np;
	work->nf_max = nf_max;
	work->na_max = na_max;

	int ns = erk_arg->rk_data->ns;

	int nX = nx*(1+nf_max);

	int steps = erk_arg->steps;

	REAL *d_ptr = mem;

	//
	work->p = d_ptr;
	d_ptr += np;
	//
	work->x_for = d_ptr;
	d_ptr += nX;
	//
	work->K = d_ptr;
	d_ptr += ns*nX;
	//
	work->x_tmp = d_ptr;
	d_ptr += nX;
	//
	if(na_max>0)
		{
		//
//		work->x_for = d_ptr;
//		d_ptr += nX*(steps+1);
		//
		work->x_traj = d_ptr;
		d_ptr += nx*(ns*steps+1);
		//
//		work->K = d_ptr;
//		d_ptr += ns*nX*steps;
		//
//		work->x_tmp = d_ptr;
//		d_ptr += nX*ns;
		//
		work->l = d_ptr;
		d_ptr += nf_max*(steps+1);
		//
		work->adj_in = d_ptr;
		d_ptr += nx+nf_max;
		//
		work->adj_tmp = d_ptr;
		d_ptr += nf_max*ns;
		}
	
	// default init
	work->nf = 0;
	work->na = 0;


	work->memsize = memsize;


	char *c_ptr = (char *) d_ptr;


#if defined(RUNTIME_CHECKS)
	if(c_ptr > ((char *) mem) + work->memsize)
		{
		printf("\nSIM_ERK_WS_CREATE: outsize memory bounds! %p %p\n\n", c_ptr, ((char *) mem) + work->memsize);
		exit(1);
		}
#endif


	return;

	}



void SIM_ERK_WS_SET_ALL(int nf, int na, REAL *x, REAL *fs, REAL *bs, REAL *p, void (*ode)(int t, REAL *x, REAL *p, void *ode_args, REAL *xdot), void (*vde_for)(int t, REAL *x, REAL *p, void *ode_args, REAL *xdot), void (*vde_adj)(int t, REAL *adj_in, void *ode_args, REAL *adj_out), void *ode_args, struct SIM_ERK_WS *work)
	{

	int ii;

	// TODO check against nf_max and na_max
	if(nf>work->nf_max)
		{
		printf("\nerror: SIM_ERK_WS_SET_ALL: nf>nf_max: %d > %d\n", nf, work->nf_max);
		exit(1);
		}
	if(nf>work->nf_max | na>work->na_max)
		{
		printf("\nerror: SIM_ERK_WS_SET_ALL: na>na_max: %d > %d\n", na, work->na_max);
		exit(1);
		}

	work->nf = nf;
	work->na = na;

	int nx = work->nx;
	int np = work->np;

	int nX = nx*(1+nf);
	int nA = np+nx; // XXX

	int steps = work->erk_arg->steps;

	REAL *x_for = work->x_for;
	REAL *p_ws = work->p;
	REAL *l = work->l;

	for(ii=0; ii<nx; ii++)
		x_for[ii] = x[ii];

	for(ii=0; ii<nx*nf; ii++)
		x_for[nx+ii] = fs[ii];

	for(ii=0; ii<np; ii++)
		p_ws[ii] = p[ii];
	
	if(na>0) // TODO what if na>1 !!!
		{
		for(ii=0; ii<np; ii++)
			l[nA*steps+ii] = 0.0;
		for(ii=0; ii<nx; ii++)
			l[nA*steps+np+ii] = bs[ii];
		}
	
	work->ode = ode;
	work->vde_for = vde_for;
	work->vde_adj = vde_adj;

	work->ode_args = ode_args;

//	d_print_mat(1, nx*nf, x, 1);
//	d_print_mat(1, np, p, 1);
//	printf("\n%p %p\n", ode, ode_args);

	return;

	}



void SIM_ERK_WS_SET_NF(int *nf, struct SIM_ERK_WS *work)
	{

	work->nf = *nf;
	
	return;

	}



// state
void SIM_ERK_WS_SET_X(REAL *x, struct SIM_ERK_WS *work)
	{

	int ii;

	int nx = work->nx;

	REAL *x_ws = work->x_for;

	for(ii=0; ii<nx; ii++)
		x_ws[ii] = x[ii];
	
	return;

	}



// forward sensitivities
void SIM_ERK_WS_SET_FS(REAL *fs, struct SIM_ERK_WS *work)
	{

	int ii;

	int nx = work->nx;
	int nf = work->nf;

	REAL *fs_ws = work->x_for+nx;

	for(ii=0; ii<nx*nf; ii++)
		fs_ws[ii] = fs[ii];
	
	return;

	}



// state
void SIM_ERK_WS_GET_X(struct SIM_ERK_WS *work, REAL *x)
	{

	int ii;

	int nx = work->nx;

	REAL *x_ws = work->x_for;

	for(ii=0; ii<nx; ii++)
		x[ii] = x_ws[ii];
	
	return;

	}



void SIM_ERK_WS_SET_P(REAL *p, struct SIM_ERK_WS *work)
	{

	int ii;

	int np = work->np;

	REAL *p_ws = work->p;

	for(ii=0; ii<np; ii++)
		p_ws[ii] = p[ii];
	
	return;

	}



void SIM_ERK_WS_SET_ODE(void (*ode)(int t, REAL *x, REAL *p, void *ode_args, REAL *xdot), struct SIM_ERK_WS *work)
	{

	work->ode = ode;
	
	return;

	}



void SIM_ERK_WS_SET_VDE_FOR(void (*vde_for)(int t, REAL *x, REAL *p, void *ode_args, REAL *xdot), struct SIM_ERK_WS *work)
	{

	work->vde_for = vde_for;
	
	return;

	}



void SIM_ERK_WS_SET_ODE_ARGS(void *ode_args, struct SIM_ERK_WS *work)
	{

	work->ode_args = ode_args;

	return;

	}



// forward sensitivities
void SIM_ERK_WS_GET_FS(struct SIM_ERK_WS *work, REAL *fs)
	{

	int ii;

	int nx = work->nx;
	int nf = work->nf;

	REAL *fs_ws = work->x_for+nx;

	for(ii=0; ii<nx*nf; ii++)
		fs[ii] = fs_ws[ii];
	
	return;

	}



void SIM_ERK_SOLVE(struct SIM_ERK_ARG *erk_arg, struct SIM_ERK_WS *work)
	{

	int steps = work->erk_arg->steps;
	REAL h = work->erk_arg->h;

	struct SIM_RK_DATA *rk_data = erk_arg->rk_data;
	int nx = work->nx;
	int np = work->np;
	int nf = work->nf;
	int na = work->na;
	REAL *K0 = work->K;
	REAL *x0 = work->x_for;
	REAL *x1 = work->x_for;
	REAL *x_traj = work->x_traj;
	REAL *p = work->p;
	REAL *x_tmp = work->x_tmp;
	REAL *adj_in = work->adj_in;
	REAL *adj_tmp = work->adj_tmp;

	REAL *l0, *l1;

	int ns = rk_data->ns;
	REAL *A_rk = rk_data->A_rk;
	REAL *B_rk = rk_data->B_rk;
	REAL *C_rk = rk_data->C_rk;

	struct BLASFEO_VEC sxt; // XXX
	struct BLASFEO_VEC sK; // XXX
	sxt.pa = x_tmp; // XXX

	int ii, jj, step, ss;
	REAL t, a, b;

	int nX = nx*(1+nf);
	int nA = nx+np; // XXX

	if(na>0)
		{
		printf("\nSIM_ERK_SOLVE: na>0 not implemented yet!\n");
		exit(1);
		}

//printf("\nnf %d na %d nX %d nA %d\n", nf, na, nX, nA);
	// forward sweep

	// TODO no need to save the entire [x Su Sx] & sens, but only [x] & sens !!!

	t = 0.0; // TODO plus time of multiple-shooting stage !!!
	if(na>0)
		{
		x_traj = work->x_traj;
		for(ii=0; ii<nx; ii++)
			x_traj[ii] = x0[ii];
		x_traj += nx;
		}
	for(step=0; step<steps; step++)
		{
//		if(na>0)
//			{
//			x0 = work->x_for + step*nX;
//			x1 = work->x_for + (step+1)*nX;
//			for(ii=0; ii<nX; ii++)
//				x1[ii] = x0[ii];
//			K0 = work->K + ns*step*nX;
//			}
		for(ss=0; ss<ns; ss++)
			{
			for(ii=0; ii<nX; ii++)
				x_tmp[ii] = x0[ii];
			for(ii=0; ii<ss; ii++)
				{
				a = A_rk[ss+ns*ii];
				if(a!=0)
					{
					a *= h;
#if 0
					sK.pa = K0+ii*nX; // XXX
					BLASFEO_AXPY(nX, a, &sK, 0, &sxt, 0, &sxt, 0); // XXX
#else
					for(jj=0; jj<nX; jj++)
						x_tmp[jj] += a*K0[jj+ii*(nX)];
#endif
					}
				}
			if(na>0)
				{
				for(ii=0; ii<nx; ii++)
					x_traj[ii] = x_tmp[ii];
				x_traj += nx;
				}
			if(nf>0)
				{
				work->vde_for(t+h*C_rk[ss], x_tmp, p, work->ode_args, K0+ss*(nX));
				}
			else
				{
				work->ode(t+h*C_rk[ss], x_tmp, p, work->ode_args, K0+ss*(nX));
				}
			}
		for(ss=0; ss<ns; ss++)
			{
			b = h*B_rk[ss];
			for(ii=0; ii<nX; ii++)
				x1[ii] += b*K0[ii+ss*(nX)];
			}
		t += h;
		}
	
	// adjoint sweep

	if(na>0)
		{
		x_traj = work->x_traj + nx*ns*steps;
		t = steps*h; // TODO plus time of multiple-shooting stage !!!
		for(step=steps-1; step>=0; step--)
			{
			l0 = work->l + step*nA;
			l1 = work->l + (step+1)*nA;
			x0 = work->x_for + step*nX;
			K0 = work->K + ns*step*nX; // XXX save all x insead !!!
			// TODO save all x instead of K !!!
			for(ss=ns-1; ss>=0; ss--)
				{
				// x
				for(ii=0; ii<nx; ii++)
					adj_in[0+ii] = x_traj[ii];
				x_traj -= nx;
//				for(ii=0; ii<nx; ii++)
//					adj_in[0+ii] = x0[ii];
//				for(ii=0; ii<ss; ii++)
//					{
//					a = A_rk[ss+ns*ii];
//					if(a!=0)
//						{
//						a *= h;
//						for(jj=0; jj<nx; jj++)
//							adj_in[0+jj] += a*K0[jj+ii*(nX)];
//						}
//					}
				// l
				b = h*B_rk[ss];
				for(ii=0; ii<nx; ii++)
					adj_in[nx+ii] = b*l1[np+ii];
				for(ii=ss+1; ii<ns; ii++)
					{
					a = A_rk[ii+ns*ss];
					if(a!=0)
						{
						a *= h;
						for(jj=0; jj<nx; jj++)
							adj_in[nx+jj] += a*adj_tmp[np+jj+ii*nA];
						}
					}
				// p
				for(ii=0; ii<np; ii++)
					adj_in[nx+nx+ii] = p[ii];
				// adj_vde
				work->vde_adj(t+h*C_rk[ss], adj_in, work->ode_args, adj_tmp+ss*nA);
				}
			// erk step
			for(ii=0; ii<nA; ii++) // TODO move in the erk step !!!
				l0[ii] = l1[ii];
			for(ss=0; ss<ns; ss++)
				for(ii=0; ii<nA; ii++)
					l0[ii] += adj_tmp[ii+ss*nA];
			t -= h;
			}
		}

	return;

	}

