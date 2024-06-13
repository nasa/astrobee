#include "daqp_prox.h"
#include "utils.h"

static int gradient_step(DAQPWorkspace* work);

int daqp_prox(DAQPWorkspace *work){
    int i,total_iter=0;
    const int nx=NX;
    int exitflag;
    c_float *swp_ptr;
    c_float diff,eps=work->settings->eps_prox;
    for(i=0;i < NX;i++) work->x[i] = 0; // TODO add option for user to set x0

    while(total_iter  <  work->settings->iter_limit){
        // xold <-- x
        swp_ptr = work->xold; work->xold = work->x; work->x = swp_ptr;

        // ** Solve least-distance problem **
        work->u = work->x;
        exitflag = daqp_ldp(work);

        total_iter += work->iterations;
        if(exitflag<0) 
            return exitflag; // Could not solve LDP -> return
        else 
            ldp2qp_solution(work); // Get qp solution 

        if(eps==0) break; // No regularization -> no outer iterations

        // ** Check convergence **
        if(work->iterations==1){ // No changes to the working set 
            for(i=0, diff= 0;i<nx;i++){ // ||x_old - x|| > eta  ?
                diff= work->x[i] - work->xold[i];
                if((diff> work->settings->eta_prox) || (diff< -work->settings->eta_prox)) break;
            }
            if(i==nx){
                exitflag = EXIT_OPTIMAL; // Fix point reached
                break;
            }
            // Take gradient step if LP (and the iterate is not constrained to a vertex)
            if((work->Rinv == NULL)&&(work->n_active != NX)){
                if(gradient_step(work)==EMPTY_IND){
                    exitflag= EXIT_UNBOUNDED;
                    break;
                }
            }
        }

        // ** Perturb problem **
        // Compute v = R'\(f-eps*x) (FWS Skipped if LP since R = I) 
        if(work->Rinv== NULL){ 
            eps*= (work->iterations==1) ? 10 : 0.9; // Adapt epsilon TODO: add to settings 
            for(i = 0; i<nx;i++) 
                work->v[i] = work->qp->f[i]*eps-work->x[i];
        }
        else{
            for(i = 0; i<nx;i++) 
                work->v[i] = work->qp->f[i]-eps*work->x[i];
            update_v(work->v,work); // 
        }
        // Perturb RHS of constraints 
        update_d(work);
    }
    // Finalize results
    if(total_iter >= work->settings->iter_limit) exitflag = EXIT_ITERLIMIT; 
    if(work->Rinv == NULL)
        for(i = 0; i<work->n_active;i++) work->lam_star[i]/=eps;// Rescale dual variables
    work->iterations = total_iter;
    return exitflag;
}
// Gradient step
// TODO: could probably reuse code from daqp
static int gradient_step(DAQPWorkspace* work){
    int j,k,disp,add_ind=EMPTY_IND;
    const int nx=NX;
    const int m=N_CONSTR;
    const int ms=N_SIMPLE;
    c_float Ax,delta_s, min_alpha=DAQP_INF;
    // Find constraint j to add: j =  argmin_j s_j
    // Simple bounds
    for(j=0, disp=0;j<ms;j++){
        if(work->sense[j]&(ACTIVE+IMMUTABLE)) continue;
        delta_s = work->x[j]-work->xold[j];
        if(delta_s>0 && //Feasible descent direction
                work->qp->bupper[j]<DAQP_INF && // Not single-sided
                work->qp->bupper[j]-work->x[j]<min_alpha*delta_s){
            add_ind = j;
            min_alpha = (work->qp->bupper[j]-work->x[j])/delta_s;
        }
        else if(delta_s < 0 && //Feasible descent direction
                work->qp->blower[j]>-DAQP_INF && // Not single-sided
                work->qp->blower[j]-work->x[j]>min_alpha*delta_s){
            add_ind = j;
            min_alpha = (work->qp->blower[j]-work->x[j])/delta_s;
        }
    }
    //General bounds
    for(j=ms, disp=0;j<m;j++){
        if(work->sense[j]&(ACTIVE+IMMUTABLE)){
            disp+=nx;// Skip ahead in A
            continue;
        }
        //delta_s[j] = A[j,:]*delta_x
        for(k=0,delta_s=0,Ax=0;k<nx;k++){ // compute s = A(x-xold) and Ax
            Ax += work->M[disp]*work->x[k];
            delta_s-=work->M[disp++]*work->xold[k];
        }
        delta_s +=Ax;

        if(delta_s>0 && // Feasible descent direction
                work->qp->bupper[j]<DAQP_INF && // Not single-sided
                work->qp->bupper[j]-Ax < delta_s*min_alpha){
            add_ind = j;
            min_alpha=(work->qp->bupper[j]-Ax)/delta_s;
        }
        else if(delta_s<0 && // Feasible descent direction
                work->qp->blower[j]>-DAQP_INF && // Not single-sided
                work->qp->blower[j]-Ax > delta_s*min_alpha){
            add_ind = j;
            min_alpha=(work->qp->blower[j]-Ax)/delta_s;
        }
    }
    // update iterate
    if(add_ind != EMPTY_IND)
        for(k=0;k<nx;k++) // x <-- x+alpha deltax
            work->x[k]+=min_alpha*(work->x[k]-work->xold[k]);

    return add_ind;
}
