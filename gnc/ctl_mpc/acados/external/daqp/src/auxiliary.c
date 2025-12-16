#include "auxiliary.h"
#include "factorization.h"
void remove_constraint(DAQPWorkspace* work, const int rm_ind){
    int i;
    // Update data structures
    SET_INACTIVE(work->WS[rm_ind]); 
    update_LDL_remove(work,rm_ind);
    (work->n_active)--;
    for(i=rm_ind;i<work->n_active;i++){
        work->WS[i] = work->WS[i+1]; 
        work->lam[i] = work->lam[i+1]; 
    }
    // Can only reuse work less than the ind that was removed 
    if(rm_ind < work->reuse_ind)
        work->reuse_ind = rm_ind;

    // Pivot for improved numerics
    pivot_last(work);
}
// Maybe take add_ind as input instead?
void add_constraint(DAQPWorkspace *work, const int add_ind, c_float lam){
    // Update data structures  
    SET_ACTIVE(add_ind);
#ifdef SOFT_WEIGHTS
    if((IS_LOWER(add_ind) && lam <= -work->d_ls[add_ind])||
            (IS_LOWER(add_ind)==0 && lam >=  work->d_us[add_ind]))
        SET_SLACK_FREE(add_ind);
    else
        SET_SLACK_FIXED(add_ind);
#endif
    update_LDL_add(work, add_ind);
    work->WS[work->n_active] = add_ind;
    work->lam[work->n_active] = lam;
    work->n_active++;

    // Pivot for improved numerics
    pivot_last(work);
}

void compute_primal_and_fval(DAQPWorkspace *work){
    int i,j,disp;
    c_float fval=0;
    // Reset u & soft slack
    for(j=0;j<NX;j++)
        work->u[j]=0;
    work->soft_slack = 0;
    //u[m] <-- Mk'*lam_star (zero if empty set)
    for(i=0;i<work->n_active;i++){
        if(IS_SIMPLE(work->WS[i])){
            // Simple constraint 
            if(work->Rinv!=NULL){ // Hessian is not identity
                for(j=work->WS[i], disp=R_OFFSET(work->WS[i],NX);j<NX;++j)
                    work->u[j]-=work->Rinv[disp+j]*work->lam_star[i];
            }
            else work->u[work->WS[i]]-=work->lam_star[i]; // Hessian is identity
        }
        else{ // General constraint
            for(j=0,disp=NX*(work->WS[i]-N_SIMPLE);j<NX;j++)
                work->u[j]-=work->M[disp++]*work->lam_star[i];
        }
        if(IS_SOFT(work->WS[i])){
#ifdef SOFT_WEIGHTS
            if(IS_LOWER(work->WS[i]))
                fval+=SQUARE(work->lam_star[i])*work->rho_ls[work->WS[i]];
            else
                fval+=SQUARE(work->lam_star[i])*work->rho_us[work->WS[i]];
#else
            fval+= SQUARE(work->lam_star[i])/SQUARE(work->scaling[work->WS[i]]);
#endif
        }
    }
    // Check for progress 
#ifndef SOFT_WEIGHTS
    fval=fval*work->settings->rho_soft;
#endif
    work->soft_slack=fval;// XXX: keep this for now to return SOFT_OPTIMAL
    for(j=0;j<NX;j++)
        fval+=work->u[j]*work->u[j];
    work->fval = fval;
}
int add_infeasible(DAQPWorkspace *work){
    int j,k,disp;
    c_float min_val = -work->settings->primal_tol;
    c_float Mu;
    int isupper=0, add_ind=EMPTY_IND;
    // Simple bounds 
    for(j=0, disp=0;j<N_SIMPLE;j++){
        // Never activate immutable or already active constraints 
        if(work->sense[j]&(ACTIVE+IMMUTABLE)){ 
            disp+=NX-j;
            continue;
        }
        if(work->Rinv==NULL){// Hessian is identify
            Mu=work->u[j]; 
            disp+=NX-j;
        }
        else{
            for(k=j,Mu=0;k<NX;k++) // 
                Mu+=work->Rinv[disp++]*work->u[k];
        }
        if((work->dupper[j]-Mu)<min_val){
            add_ind = j; isupper = 1;
            min_val = work->dupper[j]-Mu;
        }
        else if(-(work->dlower[j]-Mu)<min_val){
            add_ind = j; isupper = 0;
            min_val = -(work->dlower[j]-Mu);
        }
    }
    /* General two-sided constraints */
    for(j=N_SIMPLE, disp=0;j<N_CONSTR;j++){
        // Never activate immutable or already active constraints 
        if(work->sense[j]&(ACTIVE+IMMUTABLE)){ 
            disp+=NX;// Skip ahead in M
            continue;
        }
        for(k=0,Mu=0;k<NX;k++) 
            Mu+=work->M[disp++]*work->u[k];

        //TODO: check correct sign for slack!
        if((work->dupper[j]-Mu)<min_val){
            add_ind = j; isupper = 1;
            min_val = work->dupper[j]-Mu;
        }
        else if(-(work->dlower[j]-Mu)<min_val){
            add_ind = j; isupper = 0;
            min_val = -(work->dlower[j]-Mu);
        }
    }
    // No constraint is infeasible => return
    if(add_ind == EMPTY_IND) return 0;
    // Otherwise add infeasible constraint to working set 
    if(isupper)
        SET_UPPER(add_ind);
    else
        SET_LOWER(add_ind);
    // Set lam = lam_star
    c_float *swp_ptr;
    swp_ptr=work->lam; work->lam = work->lam_star; work->lam_star=swp_ptr;
    // Add the constraint
    if(isupper)
        add_constraint(work,add_ind,1);
    else
        add_constraint(work,add_ind,-1);
    return 1;
}
#ifdef SOFT_WEIGHTS
int remove_blocking(DAQPWorkspace *work){
    int i, ind, rm_ind = EMPTY_IND;
    c_float alpha=DAQP_INF;
    c_float alpha_cand, lam_slack;
    const c_float dual_tol = work->settings->dual_tol;
    c_float p;
    for(int i=0;i<work->n_active;i++){
        ind = work->WS[i];
        if(IS_IMMUTABLE(ind)) continue;
        lam_slack = work->lam[i];
        p = (work->sing_ind == EMPTY_IND) ? work->lam_star[i]-work->lam[i] : work->lam_star[i];
        if(IS_LOWER(ind)){
            if(IS_SLACK_FREE(ind)){ // lam <= -d_ls
                lam_slack += work->d_ls[ind];
                // lam* <= -d_ls for lower -> nothing to do
                if(p < dual_tol || work->lam_star[i] <= -work->d_ls[ind]+dual_tol) continue;
            }
            else{ // slack bound active (implying that -d_ls <= lam <= 0)
                // Remaining within the bound -d_ls <=lam* <= 0 -> nothing to do
                if(work->lam_star[i] <= dual_tol && (work->lam_star[i]+dual_tol >= -work->d_ls[ind]) &&
                        work->sing_ind == EMPTY_IND) continue;
                if(p < 0){ // lam* < -d_ls
                    lam_slack += work->d_ls[ind];
                }
            }
        }
        else{ // IS_UPPER
            if(IS_SLACK_FREE(ind)){ // lam >= d_us
                lam_slack -= work->d_us[ind];
                //lam* >= d_us for upper -> nothing to do
                if(p > -dual_tol || work->lam_star[i] >= work->d_us[ind]) continue;
            }
            else{ // slack bound active (implying that 0 <= lam <=d_us)
                // Remaining within the bound 0 <=lam* <=d_us -> nothing to do
                if(work->lam_star[i] >= -dual_tol && (work->lam_star[i] <= dual_tol+work->d_us[ind])
                        && work->sing_ind == EMPTY_IND) continue;
                if(p > 0) // lam* > d_us
                    lam_slack -= work->d_us[ind];
            }
        }

        alpha_cand = -lam_slack/p;
        if(alpha_cand < alpha){
            alpha = alpha_cand;
            rm_ind = i;
        }
    }
    if(rm_ind == EMPTY_IND) return 0; // Either dual feasible or primal infeasible
    // If blocking constraint -> update lambda
    alpha *= 1.001;
    if(work->sing_ind == EMPTY_IND)
        for(i=0;i<work->n_active;i++){
            work->lam[i]+=alpha*(work->lam_star[i]-work->lam[i]);
        }
    else
        for(i=0;i<work->n_active;i++){
            work->lam[i]+=alpha*work->lam_star[i];
        }



    // Remove the constraint from the working set and update LDL
    work->sing_ind=EMPTY_IND;
    int abs_rm_id = work->WS[rm_ind];
    lam_slack = work->lam[rm_ind];

    remove_constraint(work,rm_ind);
    if(IS_SOFT(abs_rm_id)==0 || work->sing_ind != EMPTY_IND) return 1;

    // Reactivate
    if((IS_LOWER(abs_rm_id))){
        if(lam_slack  > 0) return 1;
    }
    else{//IS_UPPER
        if(lam_slack  < 0) return 1;
    }
    // Reactive and toggle sense to correctly update factorization
    add_constraint(work,abs_rm_id,lam_slack);

    return 1;
}
#else // not SOFT_WEIGHTS
int remove_blocking(DAQPWorkspace *work){
    int i,rm_ind = EMPTY_IND; 
    c_float alpha=DAQP_INF;
    c_float alpha_cand;
    const c_float dual_tol = work->settings->dual_tol;
    for(int i=0;i<work->n_active;i++){
        if(IS_IMMUTABLE(work->WS[i])) continue;
        if(IS_LOWER(work->WS[i])){
            if(work->lam_star[i]<dual_tol) continue; //lam <= 0 for lower -> dual feasible
        }
        else if(work->lam_star[i]>-dual_tol) continue; //lam* >= 0 for upper-> dual feasible

        if(work->sing_ind == EMPTY_IND)
            alpha_cand= -work->lam[i]/(work->lam_star[i]-work->lam[i]);
        else
            alpha_cand= -work->lam[i]/work->lam_star[i];
        if(alpha_cand < alpha){
            alpha = alpha_cand; 
            rm_ind = i;
        }
    }
    if(rm_ind == EMPTY_IND) return 0; // Either dual feasible or primal infeasible
    // If blocking constraint -> update lambda
    if(work->sing_ind == EMPTY_IND)
        for(i=0;i<work->n_active;i++)
            work->lam[i]+=alpha*(work->lam_star[i]-work->lam[i]);
    else
        for(i=0;i<work->n_active;i++)
            work->lam[i]+=alpha*work->lam_star[i];

    // Remove the constraint from the working set and update LDL
    work->sing_ind=EMPTY_IND;
    remove_constraint(work,rm_ind);
    return 1;
}
#endif // SOFT_WEIGHTS

void compute_CSP(DAQPWorkspace *work){
    int i,j,disp,start_disp;
    c_float sum;
    // Forward substitution (xi <-- L\d)
    for(i=work->reuse_ind,disp=ARSUM(work->reuse_ind); i<work->n_active; i++){
        // Setup RHS
        if(IS_LOWER(work->WS[i])){
            sum = -work->dlower[work->WS[i]];
#ifdef SOFT_WEIGHTS
            if(IS_SOFT(work->WS[i]) && IS_SLACK_FREE(work->WS[i])) 
                sum-= work->d_ls[work->WS[i]]*work->rho_ls[work->WS[i]]; 
#endif
        }
        else{
            sum = -work->dupper[work->WS[i]];
#ifdef SOFT_WEIGHTS
            if(IS_SOFT(work->WS[i]) && IS_SLACK_FREE(work->WS[i])) 
                sum+= work->d_us[work->WS[i]]*work->rho_us[work->WS[i]]; 
#endif
        }
        for(j=0; j<i; j++)
            sum -= work->L[disp++]*work->xldl[j];
        disp++; //Skip 1 in L 
        work->xldl[i] = sum;
    }
    // Scale with D  (zi = xi/di)
    for(i=work->reuse_ind; i<work->n_active; i++)
        work->zldl[i] = work->xldl[i]/work->D[i];
    //Backward substitution  (lam_star <-- L'\z)
    start_disp = ARSUM(work->n_active)-1;
    for(i = work->n_active-1;i>=0;i--){
        sum=work->zldl[i];
        disp = start_disp--;
        for(j=work->n_active-1;j>i;j--){
            sum-=work->lam_star[j]*work->L[disp];
            disp-=j;
        } 
        work->lam_star[i] = sum;
    }
    work->reuse_ind = work->n_active; // Save forward substitution information 
}

//TODO this could probably be directly calculated in L
void compute_singular_direction(DAQPWorkspace *work){
    // Step direction is stored in lam_star
    int i,j,disp,offset_L= ARSUM(work->sing_ind);
    int start_disp= offset_L-1;

    // Backwards substitution (p_tidle <-- L'\(-l))
    for(i = work->sing_ind-1;i>=0;i--){
        work->lam_star[i] = -work->L[offset_L+i];
        disp = start_disp--;
        for(j=work->sing_ind-1;j>i;j--){
            work->lam_star[i]-=work->lam_star[j]*work->L[disp];
            disp-=j;
        } 
    }
    work->lam_star[work->sing_ind]=1;

    if(IS_LOWER(work->WS[work->sing_ind])) //Flip to ensure descent direction 
        for(i=0;i<=work->sing_ind;i++)
            work->lam_star[i] =-work->lam_star[i];
}


void reorder_LDL(DAQPWorkspace *work){
    // Extract first column l1,: of  L 
    // and store l1,:^2 in the beginning of L (since L will be overwritten anyways...)  
    // (a large value of l^2 signify linear dependence with the first constraint)
    int i,j,disp;
    c_float swp;
    for(i = 1, disp = 1; i < work->n_active; i++){
        work->L[i] = work->L[disp]*work->L[disp];  
        disp+=i+1;
    }
    // Sort l1,:^2 elements (and reorder the working set accordingly)
    // Bubble sort (use disp for swapping int)
    for(i=work->n_active-1; i>0; i--){
        for(j=1; j<i; j++){

            if(work->L[j] > work->L[j+1]){
                // Swap
                swp= work->L[j];
                disp = work->WS[j]; 
                work->L[j] = work->L[j+1];
                work->WS[j] = work->WS[j+1];
                work->L[j+1]= swp; 
                work->WS[j+1]= disp; 
            }
        }
    }
}

void pivot_last(DAQPWorkspace *work){
    const int rm_ind = work->n_active-2; 
    if(work->n_active > 1 && 
            work->D[rm_ind] < work->settings->pivot_tol && // element in D small enough
            work->D[rm_ind] < work->D[work->n_active-1]){ // element in D smallar than neighbor
        const int ind_old = work->WS[rm_ind];
        // Ensure that binaries never swap order (since this order is exploited) 
        if(IS_BINARY(ind_old) && IS_BINARY(work->WS[work->n_active-1])) return; 
        if(work->bnb != NULL && rm_ind < work->bnb->n_clean) return;

        c_float lam_old = work->lam[rm_ind];
        remove_constraint(work,rm_ind); // pivot_last might be recursively called here 

        if(work->sing_ind!=EMPTY_IND) return; // Abort if D becomes singular

        add_constraint(work,ind_old,lam_old);
    }	
}

// Activate constrainte that are marked active in sense
int activate_constraints(DAQPWorkspace *work){
    //TODO prioritize inequalities?
    int i;
    for(i =0;i<N_CONSTR;i++){
        if(IS_ACTIVE(i)){
#ifdef SOFT_WEIGHTS
            if(IS_LOWER(i)){
                if(IS_SLACK_FREE(i))
                    add_constraint(work,i, -(work->d_ls[i]+1));
                else
                    add_constraint(work,i, -0.9*work->d_ls[i]);
            }
            else{ //IS Upper
                if(IS_SLACK_FREE(i))
                    add_constraint(work,i, work->d_us[i]+1);
                else
                    add_constraint(work,i, 0.9*work->d_us[i]);
            }
#else
            if(IS_LOWER(i))
                add_constraint(work,i, -1.0);
            else
                add_constraint(work,i, 1.0);
#endif
        }
        if(work->sing_ind != EMPTY_IND){
            for(;i<N_CONSTR;i++) SET_INACTIVE(i); // correct sense for unadded constraints
            return EXIT_OVERDETERMINED_INITIAL;
        }
    }
    return 1;
}

// Deactivate all active constraints that are mutable (i.e., not equality constraints)
void deactivate_constraints(DAQPWorkspace *work){
    for(int i =0;i<work->n_active;i++){
        if(IS_IMMUTABLE(work->WS[i])) continue; 
        SET_INACTIVE(work->WS[i]); 
    }
}
