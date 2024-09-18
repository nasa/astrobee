#define S_FUNCTION_NAME hpipm_solver_sfunction
#define S_FUNCTION_LEVEL  2

#define MDL_START

// HPIPM
#include <blasfeo_d_aux_ext_dep.h>

#include "hpipm_d_ocp_qp_ipm.h"
#include "hpipm_d_ocp_qp_dim.h"
#include "hpipm_d_ocp_qp.h"
#include "hpipm_d_ocp_qp_sol.h"
#include "hpipm_timing.h"

#include "simstruc.h"

#define SAMPLINGTIME -1

// memory as global data
void *dim_mem;
void *qp_mem;
void *qp_sol_mem;
void *ipm_arg_mem;
void *ipm_mem; 

// hpipm structs as global data
struct d_ocp_qp *qp;
struct d_ocp_qp_sol *qp_sol;
struct d_ocp_qp_dim *dim;
struct d_ocp_qp_ipm_arg *arg;
struct d_ocp_qp_ipm_ws *workspace;

// timing structures
struct timeval tv0, tv1;

// auxiliary dimensions
int nx_total;
int nu_total;

// qp data as global data
// qp data as global data
extern int N;
extern int *nx;
extern int *nu;
extern int *nbu;
extern int *nbx;
extern int *ng;
extern int *nsbx;
extern int *nsbu;
extern int *nsg;
extern double **hA;
extern double **hB;
extern double **hb;
extern double **hQ;
extern double **hR;
extern double **hS;
extern double **hq;
extern double **hr;
extern int **hidxbx;
extern double **hlbx;
extern double **hubx;
extern int **hidxbu;
extern double **hlbu;
extern double **hubu;
extern double **hC;
extern double **hD;
extern double **hlg;
extern double **hug;
extern double **hZl;
extern double **hZu;
extern double **hzl;
extern double **hzu;
extern int **hidxs;
extern double **hlls;
extern double **hlus;


// arg
extern int mode;
extern int iter_max;
extern double alpha_min;
extern double mu0;
extern double tol_stat;
extern double tol_eq;
extern double tol_ineq;
extern double tol_comp;
extern double reg_prim;
extern int warm_start;
extern int pred_corr;
extern int ric_alg;

static void mdlInitializeSizes (SimStruct *S)
{
    // specify the number of continuous and discrete states 
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);

    // specify the number of input ports 
    if ( !ssSetNumInputPorts(S, 1) )
        return;

    // specify the number of output ports 
    if ( !ssSetNumOutputPorts(S, 4) )
        return;

    // specify dimension information for the input ports 
    ssSetInputPortVectorDimension(S, 0, nx[0]);

    // specify dimension information for the output ports 
	// compute sum of dimensions
	nx_total = 0;
	nu_total = 0;
	for(int i = 0; i <= N; i++) nu_total += nu[i];
    for(int i = 0; i <= N; i++) nx_total += nx[i];

    ssSetOutputPortVectorDimension(S, 0, nu_total);  // optimal input trajectory
    ssSetOutputPortVectorDimension(S, 1, nx_total);  // optimal state trajectory
    ssSetOutputPortVectorDimension(S, 2, 1 );  // solver status
    ssSetOutputPortVectorDimension(S, 3, 1);   // computation times

    // specify the direct feedthrough status 
    ssSetInputPortDirectFeedThrough(S, 0, 1); // current state x0

    // one sample time 
    ssSetNumSampleTimes(S, 1);
    }


#if defined(MATLAB_MEX_FILE)

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO

static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetInputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port, const DimsInfo_T *dimsInfo)
{
    if ( !ssSetOutputPortDimensionInfo(S, port, dimsInfo) )
         return;
}

    #endif /* MATLAB_MEX_FILE */


static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, SAMPLINGTIME);
    ssSetOffsetTime(S, 0, 0.0);
}


static void mdlStart(SimStruct *S)
{
	int ii;

	int rep, nrep=10;


/************************************************
* ocp qp dim
************************************************/

	hpipm_size_t dim_size = d_ocp_qp_dim_memsize(N);
	dim_mem = malloc(dim_size);
	if(dim_mem == NULL) printf("Error allocating memory for dim_mem. Exiting.\n\n");

	dim = malloc(sizeof(struct d_ocp_qp_dim));
	if(dim == NULL) printf("Error allocating memory for dim. Exiting.\n\n");
	d_ocp_qp_dim_create(N, dim, dim_mem);

	d_ocp_qp_dim_set_all(nx, nu, nbx, nbu, ng, nsbx, nsbu, nsg, dim);

/************************************************
* ocp qp
************************************************/

	hpipm_size_t qp_size = d_ocp_qp_memsize(dim);
	qp_mem = malloc(qp_size);
	if(qp_mem == NULL) printf("Error allocating memory for qp_mem. Exiting.\n\n");

	qp = malloc(sizeof(struct d_ocp_qp));
	if(qp == NULL) printf("Error allocating memory for qp. Exiting.\n\n");
	d_ocp_qp_create(dim, qp, qp_mem);

	d_ocp_qp_set_all(hA, hB, hb, hQ, hS, hR, hq, hr, hidxbx, hlbx, hubx, hidxbu, hlbu, hubu, hC, hD, hlg, hug, hZl, hZu, hzl, hzu, hidxs, hlls, hlus, qp);

/************************************************
* ocp qp sol
************************************************/

	hpipm_size_t qp_sol_size = d_ocp_qp_sol_memsize(dim);
	qp_sol_mem = malloc(qp_sol_size);
	if(qp_sol_mem == NULL) printf("Error allocating memory for qp_sol_mem. Exiting.\n\n");

	qp_sol = malloc(sizeof(struct d_ocp_qp_sol));
	if(qp_sol == NULL) printf("Error allocating memory for qp_sol. Exiting.\n\n");
	d_ocp_qp_sol_create(dim, qp_sol, qp_sol_mem);

/************************************************
* ipm arg
************************************************/

	hpipm_size_t ipm_arg_size = d_ocp_qp_ipm_arg_memsize(dim);
	ipm_arg_mem = malloc(ipm_arg_size);
	if(ipm_arg_mem == NULL) printf("Error allocating memory for ipm_arg_mem. Exiting.\n\n");

	arg = malloc(sizeof(struct d_ocp_qp_ipm_arg));
	if(arg == NULL) printf("Error allocating memory for arg. Exiting.\n\n");
	d_ocp_qp_ipm_arg_create(dim, arg, ipm_arg_mem);

	d_ocp_qp_ipm_arg_set_default(mode, arg);

	d_ocp_qp_ipm_arg_set_mu0(&mu0, arg);
	d_ocp_qp_ipm_arg_set_iter_max(&iter_max, arg);
	d_ocp_qp_ipm_arg_set_tol_stat(&tol_stat, arg);
	d_ocp_qp_ipm_arg_set_tol_eq(&tol_eq, arg);
	d_ocp_qp_ipm_arg_set_tol_ineq(&tol_ineq, arg);
	d_ocp_qp_ipm_arg_set_tol_comp(&tol_comp, arg);
	d_ocp_qp_ipm_arg_set_reg_prim(&reg_prim, arg);
	d_ocp_qp_ipm_arg_set_warm_start(&warm_start, arg);
//	d_ocp_qp_ipm_arg_set_ric_alg(&ric_alg, arg);

/************************************************
* ipm workspace
************************************************/

	hpipm_size_t ipm_size = d_ocp_qp_ipm_ws_memsize(dim, arg);
	ipm_mem = malloc(ipm_size);

	workspace = malloc(sizeof(struct d_ocp_qp_ipm_ws));
	if(workspace == NULL) printf("Error allocating memory for workspace. Exiting.\n\n");
	d_ocp_qp_ipm_ws_create(dim, arg, workspace, ipm_mem);

}

static void mdlOutputs(SimStruct *S, int_T tid)
{
    int hpipm_status;
    int ii; 
    
    // get input signals
    InputRealPtrsType in_x0_sign;
    // InputRealPtrsType in_y_ref_sign;
    // InputRealPtrsType in_y_ref_e_sign;
    
    // local buffers
    double in_x0[nx[0]];
    // double in_y_ref[{{ ocp.dims.ny }}];
    // double in_y_ref_e[{{ ocp.dims.ny_e }}];

    in_x0_sign = ssGetInputPortRealSignalPtrs(S, 0);
    // in_y_ref_sign = ssGetInputPortRealSignalPtrs(S, 1);
    // in_y_ref_e_sign = ssGetInputPortRealSignalPtrs(S, 2);

    // copy signals into local buffers
    for (int i = 0; i < nx[0]; i++) in_x0[i] = (double)(*in_x0_sign[i]);
    // for (int i = 0; i < {{ ocp.dims.ny }}; i++) in_y_ref[i] = (double)(*in_y_ref_sign[i]);
    // for (int i = 0; i < {{ ocp.dims.ny_e }}; i++) in_y_ref_e[i] = (double)(*in_y_ref_e_sign[i]);

    // for (int i = 0; i < 4; i++) ssPrintf("x0[%d] = %f\n", i, in_x0[i]);
    // ssPrintf("\n");

    // set initial condition
    d_ocp_qp_set_lbx(0, in_x0, qp);
    d_ocp_qp_set_ubx(0, in_x0, qp);
    
    // update reference
    // for (int ii = 0; ii < {{ocp.dims.N}}; ii++)
        // ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, ii, "yref", (void *) in_y_ref);

    // assign pointers to output signals 
    double *out_u, *out_x, *out_status, *out_time;

    out_u          = ssGetOutputPortRealSignal(S, 0);
    out_x          = ssGetOutputPortRealSignal(S, 1);
    out_status     = ssGetOutputPortRealSignal(S, 2);
    out_time       = ssGetOutputPortRealSignal(S, 3);
    
	hpipm_timer timer;
	hpipm_tic(&timer);
	
	// call solver
	d_ocp_qp_ipm_solve(qp, qp_sol, arg, workspace);
	d_ocp_qp_ipm_get_status(workspace, &hpipm_status);

	*out_time = hpipm_toc(&timer);
    printf("time = %f\n", out_time);

    *out_status   = (double) hpipm_status;
    
    // get solution
	// u
    int offset = 0;
	for(ii=0; ii<=N; ii++) {
        
		d_ocp_qp_sol_get_u(ii, qp_sol, out_u+offset);
		// d_print_mat(1, nu[ii], out_u+offset, 1);
        offset+=nu[ii];
    }

	// x
    offset = 0;
	for(ii=0; ii<=N; ii++) {
		d_ocp_qp_sol_get_x(ii, qp_sol, out_x+offset);
  		// d_print_mat(1, nx[ii], out_x+offset, 1);
        offset+=nx[ii];

    }
}

static void mdlTerminate(SimStruct *S)
{
    free(dim_mem);
    free(qp_mem);
	free(qp_sol_mem);
	free(ipm_arg_mem);
	free(ipm_mem); 

    free(qp);
    free(qp_sol);
    free(dim);
    free(arg);
    free(workspace);

}


#ifdef  MATLAB_MEX_FILE
#include "simulink.c"
#else
#include "cg_sfun.h"
#endif
