#include "api.h" 
#include "utils.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

// Solve problem from a given workspace and measure setup and solve time 
void daqp_solve(DAQPResult *res, DAQPWorkspace *work){
#ifdef PROFILING
    DAQPtimer timer;
    tic(&timer);
#endif
    // Select algorithm
    if(work->settings->eps_prox==0){
        if(work->bnb != NULL)
            res->exitflag = daqp_bnb(work);
        else
            res->exitflag = daqp_ldp(work);

        if(res->exitflag > 0) ldp2qp_solution(work); // Retrieve qp solution 
    }
    else{//Prox
        res->exitflag = daqp_prox(work);
    }
#ifdef PROFILING
    toc(&timer);
#endif

    // Package result
    daqp_extract_result(res,work);
    // Add time to result
#ifdef PROFILING
    res->solve_time = get_time(&timer);
#else
    res->solve_time = 0; 
#endif
    res->setup_time = 0; 
}

// Setup and solve problem
void daqp_quadprog(DAQPResult *res, DAQPProblem* qp, DAQPSettings *settings){
    int setup_flag;
    c_float setup_time=0;

    DAQPWorkspace work;
    work.settings = NULL;
    setup_flag = setup_daqp(qp,&work,&setup_time);
    if(settings!=NULL) 
        *(work.settings) = *settings; 

    if(setup_flag >= 0)
        daqp_solve(res,&work);
    else
        res->exitflag = setup_flag;

    // Add setup time to result 
    res->setup_time = setup_time; 
    // Free memory
    free_daqp_workspace(&work);
    free_daqp_ldp(&work);
}

// Setup workspace and transform QP to LDP
int setup_daqp(DAQPProblem* qp, DAQPWorkspace *work, c_float* setup_time){
    int errorflag;
#ifdef PROFILING
    DAQPtimer timer;
    if(setup_time != NULL){
        *setup_time = 0; // in case setup fails 
        tic(&timer);
    }
#endif
    // Check if QP is well-posed
    //validate_QP(qp);


    //
    int ns = 0;
    for(int i = 0; i < qp->m ; i++)
        if(qp->sense[i] & SOFT) ns++;
    // Setup workspace
    allocate_daqp_settings(work);
    allocate_daqp_workspace(work,qp->n,ns);
    errorflag = setup_daqp_ldp(work,qp);
    if(errorflag < 0){
        free_daqp_workspace(work);
        return errorflag;
    }
    errorflag = setup_daqp_bnb(work,qp->bin_ids,qp->nb, ns);
    if(errorflag < 0){
        free_daqp_workspace(work);
        return errorflag;
    }
    errorflag = activate_constraints(work);
    if(errorflag < 0){
        free_daqp_workspace(work);
        return errorflag;
    }
#ifdef PROFILING
    if(setup_time != NULL){
        toc(&timer);
        *setup_time = get_time(&timer);
    }
#endif
    return 1;
}

//  Setup LDP from QP  
int setup_daqp_ldp(DAQPWorkspace *work, DAQPProblem *qp){
    int error_flag,update_mask=0;

    work->n = qp->n;
    work->m = qp->m;
    work->ms = qp->ms;
    work->qp = qp;

    // Allocate memory for Rinv and M 
    if(qp->H!=NULL){ 
        work->Rinv = malloc(((qp->n+1)*qp->n/2)*sizeof(c_float));
        work->M = malloc(qp->n*(qp->m-qp->ms)*sizeof(c_float));
        work->scaling= malloc(qp->m*sizeof(c_float));
        update_mask += UPDATE_Rinv+UPDATE_M;
    }
    else{// H = I =>  no need to transform H->Rinv and M->A 
        work->Rinv = NULL;
        work->M = qp->A;
        work->scaling = NULL;
    }

    // Allocate memory for d and v 
    if(qp->f!=NULL || work->settings->eps_prox != 0){
        work->dupper = malloc(qp->m*sizeof(c_float));
        work->dlower = malloc(qp->m*sizeof(c_float));
        work->v = malloc(qp->n*sizeof(c_float));
        update_mask+=UPDATE_v+UPDATE_d;
    }
    else{ // f = 0 => no need to transform f->v and b->d 
        work->v= NULL;
        work->dupper = qp->bupper; 
        work->dlower = qp->blower; 
    }

    // Allocate memory for local constraint states
    work->sense = malloc(qp->m*sizeof(int));
    update_mask += UPDATE_sense;


#ifdef SOFT_WEIGHTS
    // Allocate memory for soft weights
    work->d_ls = malloc(qp->m*sizeof(c_float));
    work->d_us = malloc(qp->m*sizeof(c_float));
    work->rho_ls= malloc(qp->m*sizeof(c_float));
    work->rho_us= malloc(qp->m*sizeof(c_float));
    for(int i = 0; i< qp->m; i++){
        work->d_ls[i] = 0;
        work->d_us[i] = 0;
        work->rho_ls[i] = DEFAULT_RHO_SOFT;
        work->rho_us[i] = DEFAULT_RHO_SOFT;
    }
#endif

    // Form LDP
    error_flag = update_ldp(update_mask, work);
    if(error_flag<0){
        free_daqp_ldp(work);
        return error_flag;
    }
    return 1;
}

int setup_daqp_bnb(DAQPWorkspace* work, int* bin_ids, int nb, int ns){
    if(nb > work->n) return EXIT_OVERDETERMINED_INITIAL;
    if((work->bnb == NULL) && (nb >0)){
        work->bnb= malloc(sizeof(DAQPBnB));

        work->bnb->nb = nb;
        work->bnb->bin_ids = bin_ids;

        // Setup tree
        work->bnb->tree= malloc((work->bnb->nb+1)*sizeof(DAQPNode));
        work->bnb->tree_WS= malloc((work->n+ns+1)*(work->bnb->nb+1)*sizeof(int));
        work->bnb->n_nodes = 0; 
        work->bnb->nWS= 0; 
        work->bnb->fixed_ids= malloc((work->bnb->nb+1)*sizeof(int));
    }
    return 1;
}

// Free data for LDP 
void free_daqp_ldp(DAQPWorkspace *work){
    if(work->sense==NULL) return; // Already freed
    free(work->sense);
    if(work->Rinv != NULL){
        free(work->Rinv);
        free(work->scaling);
        free(work->M);
    }
    if(work->v != NULL){
        free(work->v);
        free(work->dupper);
        free(work->dlower);
    }

#ifdef SOFT_WEIGHTS
    if(work->d_ls != NULL){
        free(work->d_ls);
        free(work->d_us);
        free(work->rho_ls);
        free(work->rho_us);
    }
#endif

    work->sense = NULL;
}

void allocate_daqp_settings(DAQPWorkspace *work){
    if(work->settings == NULL){
        work->settings = malloc(sizeof(DAQPSettings));
        daqp_default_settings(work->settings);
    }
}

void free_daqp_bnb(DAQPWorkspace* work){
    if(work->bnb != NULL){
        free(work->bnb->tree);
        free(work->bnb->tree_WS);
        free(work->bnb->fixed_ids);
        free(work->bnb);
        work->bnb = NULL;
    }
}

// Allocate memory for iterates  
void allocate_daqp_workspace(DAQPWorkspace *work, int n, int ns){
    work->n = n;
    n = n + ns; //To account for soft_constraints
    work->Rinv = NULL;
    work->v = NULL;
    work->scaling = NULL;

    work->lam = malloc((n+1)*sizeof(c_float));
    work->lam_star = malloc((n+1)*sizeof(c_float));

    work->WS= malloc((n+1)*sizeof(int));

    work->D= malloc((n+1)*sizeof(c_float));
    work->xldl= malloc((n+1)*sizeof(c_float));
    work->zldl= malloc((n+1)*sizeof(c_float));
    work->L= malloc(((n+1)*(n+2)/2)*sizeof(c_float));



    work->u= malloc(work->n*sizeof(c_float));
    work->x = work->u; 

    work->xold= malloc(work->n*sizeof(c_float));

#ifdef SOFT_WEIGHTS
    work->d_ls= NULL;
    work->d_us= NULL;
    work->rho_ls= NULL;
    work->rho_us= NULL;
#endif

    work->bnb = NULL;
    reset_daqp_workspace(work);
}


// Free memory for iterates
void free_daqp_workspace(DAQPWorkspace *work){
    if(work->lam != NULL){
        free(work->lam);
        free(work->lam_star);

        free(work->WS);

        free(work->L);
        free(work->D);

        free(work->xldl);
        free(work->zldl);

        free(work->u);

        free(work->xold);

        work->lam = NULL;
    }

    if(work->settings != NULL){ 
        free(work->settings);
        work->settings = NULL;
    }

    free_daqp_bnb(work);
}

// Extract solution information from workspace 
void daqp_extract_result(DAQPResult* res, DAQPWorkspace* work){
    int i; 
    // Extract primal solution
    for(i=0;i<work->n;i++) res->x[i] = work->x[i];

    // Extract dual solution
    if(res->lam != NULL){
        for(i=0;i<work->m;i++) 
            res->lam[i] = 0; 
        for(i=0;i<work->n_active;i++)
            res->lam[work->WS[i]] = work->lam_star[i];
    }

    // Shift back function value
    if(work->v != NULL && (work->settings->eps_prox == 0 || work->Rinv != NULL)){ // Normal QP
        res->fval = work->fval;
        for(i=0;i<work->n;i++) res->fval-=work->v[i]*work->v[i];
        res->fval *=0.5;
        if(work->settings->eps_prox != 0)
        for(i=0;i<work->n;i++) // compensate for proximal iterations
            res->fval+= work->settings->eps_prox*work->x[i]*work->x[i];
    }
    else if(work->qp != NULL && work->qp->f != NULL ){ // LP
        res->fval = 0;
        for(i=0;i<work->n;i++) res->fval+=work->qp->f[i]*work->x[i];
    }

    // info
    res->soft_slack = work->soft_slack;
    res->iter = work->iterations;
    res->nodes = (work->bnb == NULL) ? 1 : work->bnb->nodecount;
}

void daqp_default_settings(DAQPSettings* settings){
    settings->primal_tol = DEFAULT_PRIM_TOL;
    settings->dual_tol = DEFAULT_DUAL_TOL; 
    settings->zero_tol = DEFAULT_ZERO_TOL;
    settings->pivot_tol = DEFAULT_PIVOT_TOL;
    settings->progress_tol= DEFAULT_PROG_TOL;

    settings->cycle_tol = DEFAULT_CYCLE_TOL;
    settings->iter_limit = DEFAULT_ITER_LIMIT;
    settings->fval_bound = DAQP_INF; 

    settings->eps_prox = 0;
    settings->eta_prox = DEFAULT_ETA;

    settings->rho_soft = DEFAULT_RHO_SOFT; 

    settings->rel_subopt = DEFAULT_REL_SUBOPT;
    settings->abs_subopt = DEFAULT_ABS_SUBOPT;
}
