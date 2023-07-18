#include "mex.h"
#include "api.h"
#include "utils.h"
#include "codegen.h"
#include <string.h>

const char* INFO_FIELDS[] = {
  "lambda",
  "setup_time",           
  "solve_time",           
  "iter",           
  "nodes",
  "soft_slack"}; 

const char* SETTINGS_FIELDS[] = {
  "primal_tol",           
  "dual_tol",           
  "zero_tol",           
  "pivot_tol",
  "progress_tol",
  "cycle_tol",
  "iter_limit",
  "fval_bound",
  "eps_prox",
  "eta_prox",
  "rho_soft",
  "abs_subopt",
  "rel_subopt"};


/* The gateway function */
void mexFunction( int nlhs, mxArray *plhs[],
				  int nrhs, const mxArray *prhs[])
{

  // RHS 

  // Extract command
  char cmd[64];
  mxGetString(prhs[0], cmd, sizeof(cmd));
  
  // Extract workspace pointer 
  DAQPWorkspace *work;
  long long *work_i;
  union{long long i; void *ptr;} work_ptr; // Used for int64 & pointer juggling..
  if(nrhs>1){// Pointer always second argument (stored as int64)
	work_i = (long long *)mxGetData(prhs[1]);
	work_ptr.i  = *work_i; 
	work = work_ptr.ptr;
  }

	if (!strcmp("new", cmd)) {
	  // Allocate new workspace and return pointer to it
	  work_ptr.ptr = calloc(1,sizeof(DAQPWorkspace));
	  // Return pointer as int64
	  plhs[0] = mxCreateNumericMatrix(1, 1, mxINT64_CLASS, mxREAL);
	  work_i = (long long *) mxGetData(plhs[0]);
	  *work_i = work_ptr.i;
	  return;
	}
	else if (!strcmp("delete", cmd)) {
	  // Free workspace 
	  free_daqp_workspace(work);
	  free_daqp_ldp(work);
	  if(work->qp) free(work->qp);
	  free(work);
	  return;
	}
	else if (!strcmp("setup", cmd)) {

	  if(work->qp) mexErrMsgTxt("Setup already completed.");
	  DAQPProblem *qp = calloc(1,sizeof(DAQPProblem));
	  // Extract data
	  int error_flag;
	  
	  // Get dimensions 
	  int n = mxGetM(prhs[4]);
	  int m = mxGetM(prhs[5]);
	  int ms = m-mxGetN(prhs[4]);
	  int nb = mxGetM(prhs[8]);
	  
	  // Setup QP struct
	  qp->n = n;
	  qp->m = m;
	  qp->ms = ms;
	  qp->H= mxIsEmpty(prhs[2]) ? NULL : (c_float *)mxGetPr(prhs[2]);
	  qp->f= mxIsEmpty(prhs[3]) ? NULL : (c_float *)mxGetPr(prhs[3]);
	  qp->A= (c_float *)mxGetPr(prhs[4]);
	  qp->bupper= (c_float *)mxGetPr(prhs[5]);
	  qp->blower= (c_float *)mxGetPr(prhs[6]);
	  qp->sense= (int *)mxGetPr(prhs[7]);
	  qp->bin_ids= (int *)mxGetPr(prhs[8]);
	  qp->nb=nb; 
	  
	  c_float solve_time;
	  error_flag = setup_daqp(qp,work,&solve_time);
	  if(error_flag < 0){
		free(work->qp);
		work->qp = NULL;
	  }


	  plhs[0] = mxCreateDoubleScalar(error_flag);
	  plhs[1] = mxCreateDoubleScalar(solve_time);
	}
	else if (!strcmp("solve", cmd)) {
	  int *exitflag;

	  DAQPResult result;
	  if(work->qp == NULL) mexErrMsgTxt("No problem to solve");
	  // Update QP pointers 
	  work->qp->H= mxIsEmpty(prhs[2]) ? NULL : (c_float *)mxGetPr(prhs[2]);
	  work->qp->f= mxIsEmpty(prhs[3]) ? NULL : (c_float *)mxGetPr(prhs[3]);
	  work->qp->A= (c_float *)mxGetPr(prhs[4]);
	  work->qp->bupper= (c_float *)mxGetPr(prhs[5]);
	  work->qp->blower= (c_float *)mxGetPr(prhs[6]);
	  work->qp->sense= (int *)mxGetPr(prhs[7]);
	  // Setup output 
#ifdef DAQP_SINGLE_PRECISION
	  plhs[0] = mxCreateNumericMatrix((mwSize)work->n,1,mxSINGLE,mxREAL); // x_star
	  mxArray* lam = mxCreateNumericMatrix((mwSize)work->m,1,mxSINGLE,mxREAL); // lambda 
#else
	  plhs[0] = mxCreateNumericMatrix((mwSize)work->n,1,mxDOUBLE_CLASS,mxREAL); // x_star
	  mxArray* lam = mxCreateNumericMatrix((mwSize)work->m,1,mxDOUBLE_CLASS,mxREAL); // lambda 
#endif
	  plhs[2] = mxCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL); //Exit flag

	  result.x = (c_float *) mxGetPr(plhs[0]);
	  result.lam = (c_float *) mxGetPr(lam);
	  exitflag = (int *)mxGetPr(plhs[2]);
	  
	  // Solve problem
	  daqp_solve(&result,work); 
	  // Extract solution information
	  exitflag[0] = result.exitflag; 
	  plhs[1] = mxCreateDoubleScalar(result.fval);

	  // Package info struct
	  int n_info = sizeof(INFO_FIELDS)/sizeof(INFO_FIELDS[0]);
	  mxArray* info_struct = mxCreateStructMatrix(1,1,n_info,INFO_FIELDS);
	  mxSetField(info_struct, 0, "lambda", lam);
	  mxSetField(info_struct, 0, "solve_time", mxCreateDoubleScalar(result.solve_time));
	  mxSetField(info_struct, 0, "setup_time", mxCreateDoubleScalar(result.setup_time));
	  mxSetField(info_struct, 0, "iter", mxCreateDoubleScalar(result.iter));
	  mxSetField(info_struct, 0, "nodes", mxCreateDoubleScalar(result.nodes));
	  mxSetField(info_struct, 0, "soft_slack", mxCreateDoubleScalar(result.soft_slack));
	  plhs[3] = info_struct;
	}
	else if (!strcmp("set_default_settings", cmd)){
	  if(work->settings == NULL) work->settings = malloc(sizeof(DAQPSettings));
	  daqp_default_settings(work->settings);
	}
	else if (!strcmp("get_settings", cmd)) {
	  if(work->settings != NULL){
		int n_settings = sizeof(SETTINGS_FIELDS)/sizeof(SETTINGS_FIELDS[0]);
		mxArray* s = mxCreateStructMatrix(1,1,n_settings,SETTINGS_FIELDS);
		mxSetField(s, 0, "primal_tol", mxCreateDoubleScalar(work->settings->primal_tol));
		mxSetField(s, 0, "dual_tol", mxCreateDoubleScalar(work->settings->dual_tol));
		mxSetField(s, 0, "zero_tol", mxCreateDoubleScalar(work->settings->zero_tol));
		mxSetField(s, 0, "pivot_tol", mxCreateDoubleScalar(work->settings->pivot_tol));
		mxSetField(s, 0, "progress_tol", mxCreateDoubleScalar(work->settings->progress_tol));
		mxSetField(s, 0, "cycle_tol", mxCreateDoubleScalar(work->settings->cycle_tol));
		mxSetField(s, 0, "iter_limit", mxCreateDoubleScalar(work->settings->iter_limit));
		mxSetField(s, 0, "fval_bound", mxCreateDoubleScalar(work->settings->fval_bound));
		mxSetField(s, 0, "eps_prox", mxCreateDoubleScalar(work->settings->eps_prox));
		mxSetField(s, 0, "eta_prox", mxCreateDoubleScalar(work->settings->eta_prox));
		mxSetField(s, 0, "rho_soft", mxCreateDoubleScalar(work->settings->rho_soft));
		mxSetField(s, 0, "abs_subopt", mxCreateDoubleScalar(work->settings->abs_subopt));
		mxSetField(s, 0, "rel_subopt", mxCreateDoubleScalar(work->settings->rel_subopt));
		plhs[0] = s;
	  }
	}
	else if (!strcmp("set_settings", cmd)) {
	  const mxArray* s = prhs[2];
	  work->settings->primal_tol = (c_float)mxGetScalar(mxGetField(s, 0, "primal_tol"));
	  work->settings->dual_tol =  (c_float)mxGetScalar(mxGetField(s, 0, "dual_tol"));
	  work->settings->zero_tol = (c_float)mxGetScalar(mxGetField(s, 0, "zero_tol"));
	  work->settings->pivot_tol = (c_float)mxGetScalar(mxGetField(s, 0, "pivot_tol"));
	  work->settings->progress_tol = (c_float)mxGetScalar(mxGetField(s, 0, "progress_tol"));
	  work->settings->cycle_tol = (int)mxGetScalar(mxGetField(s, 0, "cycle_tol"));
	  work->settings->iter_limit= (int)mxGetScalar(mxGetField(s, 0, "iter_limit"));
	  work->settings->fval_bound= (c_float)mxGetScalar(mxGetField(s, 0, "fval_bound"));
	  work->settings->eps_prox = (c_float)mxGetScalar(mxGetField(s, 0, "eps_prox"));
	  work->settings->eta_prox= (c_float)mxGetScalar(mxGetField(s, 0, "eta_prox"));
	  work->settings->rho_soft= (c_float)mxGetScalar(mxGetField(s, 0, "rho_soft"));
	  work->settings->abs_subopt= (c_float)mxGetScalar(mxGetField(s, 0, "abs_subopt"));
	  work->settings->rel_subopt= (c_float)mxGetScalar(mxGetField(s, 0, "rel_subopt"));
	}
	else if (!strcmp("update", cmd)) {
	  if(work->qp == NULL) mexErrMsgTxt("No problem to update");
	  // Update QP pointers 
	  work->qp->H= mxIsEmpty(prhs[2]) ? NULL : (c_float *)mxGetPr(prhs[2]);
	  work->qp->f= mxIsEmpty(prhs[3]) ? NULL : (c_float *)mxGetPr(prhs[3]);
	  work->qp->A= (c_float *)mxGetPr(prhs[4]);
	  work->qp->bupper= (c_float *)mxGetPr(prhs[5]);
	  work->qp->blower= (c_float *)mxGetPr(prhs[6]);
	  work->qp->sense= (int *)mxGetPr(prhs[7]);
	  // Update LDP with new QP data
	  const int update_mask = (int)mxGetScalar(prhs[8]);
	  update_ldp(update_mask,work);
	}
    else if (!strcmp("codegen", cmd)) {
        char fname[64];
        char dir[128];
        mxGetString(prhs[2], fname, sizeof(fname));
        mxGetString(prhs[3], dir, sizeof(dir));
        if(work->qp == NULL) mexErrMsgTxt("Setup is required before code generation");
        render_daqp_workspace(work,fname,dir);
    }
    else if (!strcmp("isdouble", cmd)) {
        plhs[0] = mxCreateNumericMatrix(1, 1, mxINT32_CLASS, mxREAL); // is_double 
        int* isdouble_ptr = (int *)mxGetPr(plhs[0]);
#ifdef DAQP_SINGLE_PRECISION
        isdouble_ptr[0] = 0;
#else
        isdouble_ptr[0] = 1;
#endif
    }

  // RHS
}
