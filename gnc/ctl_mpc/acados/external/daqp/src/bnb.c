#include "bnb.h"

int daqp_bnb(DAQPWorkspace* work){
    int branch_id, exitflag;
    DAQPNode* node;
    c_float *swp_ptr = NULL;

    // Modify upper bound based on absolute/relavtive suboptimality tolerance
    c_float fval_bound0 = work->settings->fval_bound;
    c_float eps_r = 1/(1+work->settings->rel_subopt);
    work->settings->fval_bound = (fval_bound0 - work->settings->abs_subopt)*eps_r;

    work->bnb->neq = work->n_active; 
    work->bnb->itercount=0;
    work->bnb->nodecount=0;
    // Setup root node
    work->bnb->tree[0].depth=-1;
    work->bnb->tree[0].WS_start=0;
    work->bnb->tree[0].WS_end=0;
    work->bnb->tree[0].bin_id=0;
    work->bnb->n_nodes=1;
    work->bnb->n_clean=work->bnb->neq;

    // Start tree exploration
    while( work->bnb->n_nodes > 0 ){
        node = work->bnb->tree+(--work->bnb->n_nodes);
        exitflag = process_node(node,work); // Solve relaxation
        // Cut conditions
        if(exitflag==EXIT_INFEASIBLE) continue; // Dominance cut
        if(exitflag<0) return exitflag; // Inner solver failed => abort

        // Find index to branch over 
        branch_id = get_branch_id(work); 
        if(branch_id==-1){// Nothing to branch over => integer feasible
            work->settings->fval_bound = (work->fval - work->settings->abs_subopt)*eps_r;
            swp_ptr=work->xold; work->xold= work->u; work->u=swp_ptr; // Store feasible sol
        }
        else{
            spawn_children(node,branch_id, work);
        }
    }

    // Exploration completed 
    work->iterations = work->bnb->itercount;
    // Correct fval
    work->fval = work->settings->fval_bound/eps_r+work->settings->abs_subopt;
    work->settings->fval_bound = fval_bound0;
    if(swp_ptr==NULL)
        return EXIT_INFEASIBLE;
    else{
        // Let work->u point to the best feasible solution 
        swp_ptr=work->u; work->u= work->xold; work->xold=swp_ptr;
        return EXIT_OPTIMAL;
    }
}

int process_node(DAQPNode* node, DAQPWorkspace* work){
    int exitflag;
    work->bnb->nodecount+=1;
    if(node->depth >=0){
        // Fix a binary constraints
        work->bnb->fixed_ids[node->depth] = node->bin_id;
        // Setup relaxation 
        if(work->bnb->n_nodes==0 || (node-1)->depth!=node->depth){ 
            // Sibling has been processed => need to fix workspace state
            work->bnb->n_clean += (node->depth-(node+1)->depth);
            node_cleanup_workspace(work->bnb->n_clean,work);
            warmstart_node(node,work);
        }
        else{
            //work->bnb->n_clean = node->depth;
            //node_cleanup_workspace(work->bnb->n_clean,work);
            //warmstart_node(node,work);
            add_upper_lower(node->bin_id,work);
            work->sense[REMOVE_LOWER_FLAG(node->bin_id)] |= IMMUTABLE; // Make equality
        }
        // Add binary constraint 
    }
    // Solve relaxation
    exitflag = daqp_ldp(work);
    work->bnb->itercount += work->iterations;
    
    if(exitflag == EXIT_CYCLE){// Try to repair (cold start)
        node_cleanup_workspace(work->bnb->n_clean,work);
        work->sing_ind=EMPTY_IND;
        work->n_active=work->bnb->n_clean;
        work->reuse_ind=work->bnb->n_clean;
        for(int i=work->bnb->n_clean - work->bnb->neq; i< node->depth+1;i++){
            add_upper_lower(work->bnb->fixed_ids[i],work);
            SET_IMMUTABLE(REMOVE_LOWER_FLAG(work->bnb->fixed_ids[i]));
        }
        work->bnb->n_clean = work->bnb->neq+node->depth;
        exitflag = daqp_ldp(work);
        work->bnb->itercount += work->iterations;
    }

    return exitflag;
}

int get_branch_id(DAQPWorkspace* work){
    int i,disp;
    int branch_id = EMPTY_IND;
    for(i=0; i < work->bnb->nb; i++){
        // Branch on first inactive constraint 
        if(IS_ACTIVE(work->bnb->bin_ids[i])) continue;
        branch_id = work->bnb->bin_ids[i];
        break;
    }

    if(branch_id == EMPTY_IND) return EMPTY_IND; // Nothing to branch over (=>integer feasible)

    // Determine if upper or lower child should be processed first 
    // by computing whether the upper or lower bound is closer to be activated
    c_float diff = 0.5*(work->dupper[branch_id]+work->dlower[branch_id]);
    if(IS_SIMPLE(branch_id)){//Simple bound
        if(work->Rinv==NULL) diff-=work->u[branch_id]; //Hessian is identify 
        else{
            for(i=branch_id,disp=branch_id+R_OFFSET(branch_id,NX);i<NX;i++) // 
                diff-=work->Rinv[disp++]*work->u[i];
        }
    }
    else{//General bound
        for(i=0,disp=NX*(branch_id-N_SIMPLE);i<NX;i++) 
            diff-=work->M[disp++]*work->u[i];
    }
    branch_id = diff<0 ? branch_id : ADD_LOWER_FLAG(branch_id);
    return branch_id;
}

void spawn_children(DAQPNode* node, const int branch_id, DAQPWorkspace* work){

    save_warmstart(node,work);

    // Update child1 (reuse current node) 
    node->bin_id = TOGGLE_LOWER_FLAG(branch_id);
    node->depth +=1;

    // Update child2
    (node+1)->bin_id = branch_id;
    (node+1)->depth = node->depth; 
    (node+1)->WS_start = node->WS_start;
    (node+1)->WS_end= node->WS_end;

    work->bnb->n_nodes+=2;
}

void node_cleanup_workspace(int n_clean, DAQPWorkspace* work){
    // Cleanup sense 
    for(int i=n_clean; i<work->n_active; i++)
        work->sense[work->WS[i]]&= IS_BINARY(work->WS[i]) ? ~(ACTIVE+IMMUTABLE): ~ACTIVE;
    // Reset workspace
    work->sing_ind=EMPTY_IND;
    work->n_active=n_clean;
    work->reuse_ind=n_clean; 
}


void warmstart_node(DAQPNode* node, DAQPWorkspace* work){
    int i;
    // Add fixed constraints
    for(i=work->bnb->n_clean - work->bnb->neq; i< node->depth+1;i++){ 
        add_upper_lower(work->bnb->fixed_ids[i],work);
        SET_IMMUTABLE(REMOVE_LOWER_FLAG(work->bnb->fixed_ids[i]));
    }
    work->bnb->n_clean = work->bnb->neq+node->depth; 
    // Add free constraints
    for(i=node->WS_start; i < node->WS_end; i++){
        add_upper_lower(work->bnb->tree_WS[i],work);
        if(work->sing_ind != EMPTY_IND) {
            work->n_active--;
            SET_INACTIVE(work->WS[work->n_active]);
            work->sing_ind = EMPTY_IND;
            break; // Abort warm start if singular basis 
        }
    }
    work->bnb->nWS = node->WS_start; // always move up tree after warmstart 
}

void save_warmstart(DAQPNode* node, DAQPWorkspace* work){
    // Save warmstart 
    node->WS_start = work->bnb->nWS;

    int id_to_add;
    for(int i =work->bnb->neq; i<work->n_active;i++){
        id_to_add = (work->WS[i]+(IS_LOWER(work->WS[i]) << (LOWER_BIT-1)));
        if((work->sense[work->WS[i]]&(IMMUTABLE+BINARY))!=IMMUTABLE+BINARY)
            work->bnb->tree_WS[work->bnb->nWS++]= id_to_add;
    }
    node->WS_end = work->bnb->nWS;
}

int add_upper_lower(const int add_id, DAQPWorkspace* work){
    int true_add_id = REMOVE_LOWER_FLAG(add_id);
    // Setup new binary constraint
    if(EXTRACT_LOWER_FLAG(add_id)){
        SET_LOWER(true_add_id);
        add_constraint(work,true_add_id,-1.0);
    }
    else{
        SET_UPPER(true_add_id);
        add_constraint(work,true_add_id,1.0);
    }
    return 1;
}
