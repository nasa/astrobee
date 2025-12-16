/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

// standard
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific
#include "astrobee_model/astrobee_model.h"
#include "astrobee_constraints/astrobee_constraints.h"
#include "astrobee_cost/astrobee_cost.h"




#include "acados_solver_astrobee.h"

#define NX     ASTROBEE_NX
#define NZ     ASTROBEE_NZ
#define NU     ASTROBEE_NU
#define NP     ASTROBEE_NP
#define NBX    ASTROBEE_NBX
#define NBX0   ASTROBEE_NBX0
#define NBU    ASTROBEE_NBU
#define NSBX   ASTROBEE_NSBX
#define NSBU   ASTROBEE_NSBU
#define NSH    ASTROBEE_NSH
#define NSG    ASTROBEE_NSG
#define NSPHI  ASTROBEE_NSPHI
#define NSHN   ASTROBEE_NSHN
#define NSGN   ASTROBEE_NSGN
#define NSPHIN ASTROBEE_NSPHIN
#define NSBXN  ASTROBEE_NSBXN
#define NS     ASTROBEE_NS
#define NSN    ASTROBEE_NSN
#define NG     ASTROBEE_NG
#define NBXN   ASTROBEE_NBXN
#define NGN    ASTROBEE_NGN
#define NY0    ASTROBEE_NY0
#define NY     ASTROBEE_NY
#define NYN    ASTROBEE_NYN
// #define N      ASTROBEE_N
#define NH     ASTROBEE_NH
#define NPHI   ASTROBEE_NPHI
#define NHN    ASTROBEE_NHN
#define NPHIN  ASTROBEE_NPHIN
#define NR     ASTROBEE_NR


// ** solver data **

astrobee_solver_capsule * astrobee_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(astrobee_solver_capsule));
    astrobee_solver_capsule *capsule = (astrobee_solver_capsule *) capsule_mem;

    return capsule;
}


int astrobee_acados_free_capsule(astrobee_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int astrobee_acados_create(astrobee_solver_capsule* capsule)
{
    int N_shooting_intervals = ASTROBEE_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return astrobee_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int astrobee_acados_update_time_steps(astrobee_solver_capsule* capsule, int N, double* new_time_steps)
{
    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "astrobee_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;
}

/**
 * Internal function for astrobee_acados_create: step 1
 */
void astrobee_acados_create_1_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/
    nlp_solver_plan->nlp_solver = SQP_RTI;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_OSQP;

    nlp_solver_plan->nlp_cost[0] = EXTERNAL;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = EXTERNAL;

    nlp_solver_plan->nlp_cost[N] = EXTERNAL;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = CONTINUOUS_MODEL;
        nlp_solver_plan->sim_solver_plan[i].sim_solver = ERK;
    }

    for (int i = 0; i < N; i++)
    {nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;
}


/**
 * Internal function for astrobee_acados_create: step 2
 */
ocp_nlp_dims* astrobee_acados_create_2_create_and_set_dimensions(astrobee_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 17
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
    }

    // for initial state
    nbx[0]  = NBX0;
    nsbx[0] = 0;
    ns[0] = NS - NSBX;
    nbxe[0] = 12;
    ny[0] = NY0;

    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    free(intNp1mem);
return nlp_dims;
}


/**
 * Internal function for astrobee_acados_create: step 3
 */
void astrobee_acados_create_3_create_and_set_functions(astrobee_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_param_casadi_create(&capsule->__CAPSULE_FNC__ , 46); \
    }while(false)


    // constraints.constr_type == "BGH" and dims.nh > 0
    capsule->nl_constr_h_fun_jac = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(nl_constr_h_fun_jac[i], astrobee_constr_h_fun_jac_uxt_zt);
    }
    capsule->nl_constr_h_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(nl_constr_h_fun[i], astrobee_constr_h_fun);
    }
    
    capsule->nl_constr_h_fun_jac_hess = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(nl_constr_h_fun_jac_hess[i], astrobee_constr_h_fun_jac_uxt_zt_hess);
    }
    

    MAP_CASADI_FNC(nl_constr_h_e_fun_jac, astrobee_constr_h_e_fun_jac_uxt_zt);
    MAP_CASADI_FNC(nl_constr_h_e_fun, astrobee_constr_h_e_fun);
    MAP_CASADI_FNC(nl_constr_h_e_fun_jac_hess, astrobee_constr_h_e_fun_jac_uxt_zt_hess);
    


    // explicit ode
    capsule->forw_vde_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(forw_vde_casadi[i], astrobee_expl_vde_forw);
    }

    capsule->expl_ode_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(expl_ode_fun[i], astrobee_expl_ode_fun);
    }
    capsule->hess_vde_casadi = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N; i++) {
        MAP_CASADI_FNC(hess_vde_casadi[i], astrobee_expl_ode_hess);
    }


    // external cost
    MAP_CASADI_FNC(ext_cost_0_fun, astrobee_cost_ext_cost_0_fun);

    // external cost
    MAP_CASADI_FNC(ext_cost_0_fun_jac, astrobee_cost_ext_cost_0_fun_jac);

    // external cost
    MAP_CASADI_FNC(ext_cost_0_fun_jac_hess, astrobee_cost_ext_cost_0_fun_jac_hess);
    // external cost
    capsule->ext_cost_fun = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(ext_cost_fun[i], astrobee_cost_ext_cost_fun);
    }

    capsule->ext_cost_fun_jac = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(ext_cost_fun_jac[i], astrobee_cost_ext_cost_fun_jac);
    }

    capsule->ext_cost_fun_jac_hess = (external_function_param_casadi *) malloc(sizeof(external_function_param_casadi)*N);
    for (int i = 0; i < N-1; i++)
    {
        MAP_CASADI_FNC(ext_cost_fun_jac_hess[i], astrobee_cost_ext_cost_fun_jac_hess);
    }
    // external cost - function
    MAP_CASADI_FNC(ext_cost_e_fun, astrobee_cost_ext_cost_e_fun);
    

    // external cost - jacobian
    MAP_CASADI_FNC(ext_cost_e_fun_jac, astrobee_cost_ext_cost_e_fun_jac);

    // external cost - hessian
    MAP_CASADI_FNC(ext_cost_e_fun_jac_hess, astrobee_cost_ext_cost_e_fun_jac_hess);

#undef MAP_CASADI_FNC
}


/**
 * Internal function for astrobee_acados_create: step 4
 */
void astrobee_acados_create_4_set_default_parameters(astrobee_solver_capsule* capsule) {
    const int N = capsule->nlp_solver_plan->N;
    // initialize parameters to nominal value
    double* p = calloc(NP, sizeof(double));

    for (int i = 0; i <= N; i++) {
        astrobee_acados_update_params(capsule, i, p, NP);
    }
    free(p);
}


/**
 * Internal function for astrobee_acados_create: step 5
 */
void astrobee_acados_create_5_set_nlp_in(astrobee_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    /************************************************
    *  nlp_in
    ************************************************/
//    ocp_nlp_in * nlp_in = ocp_nlp_in_create(nlp_config, nlp_dims);
//    capsule->nlp_in = nlp_in;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    // set up time_steps
    

    if (new_time_steps) {
        astrobee_acados_update_time_steps(capsule, N, new_time_steps);
    } else {// all time_steps are identical
        double time_step = 0.1;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &time_step);
        }
    }

    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_vde_forw", &capsule->forw_vde_casadi[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_ode_fun", &capsule->expl_ode_fun[i]);
        ocp_nlp_dynamics_model_set(nlp_config, nlp_dims, nlp_in, i, "expl_ode_hess", &capsule->hess_vde_casadi[i]);
    
    }

    /**** Cost ****/
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun", &capsule->ext_cost_0_fun);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac", &capsule->ext_cost_0_fun_jac);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "ext_cost_fun_jac_hess", &capsule->ext_cost_0_fun_jac_hess);
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun", &capsule->ext_cost_fun[i-1]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac", &capsule->ext_cost_fun_jac[i-1]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "ext_cost_fun_jac_hess", &capsule->ext_cost_fun_jac_hess[i-1]);
    }
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun", &capsule->ext_cost_e_fun);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac", &capsule->ext_cost_e_fun_jac);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "ext_cost_fun_jac_hess", &capsule->ext_cost_e_fun_jac_hess);



    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(12 * sizeof(int));
    
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    idxbxe_0[4] = 4;
    idxbxe_0[5] = 5;
    idxbxe_0[6] = 6;
    idxbxe_0[7] = 7;
    idxbxe_0[8] = 8;
    idxbxe_0[9] = 9;
    idxbxe_0[10] = 10;
    idxbxe_0[11] = 11;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);

    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    
    idxbu[0] = 0;
    idxbu[1] = 1;
    idxbu[2] = 2;
    idxbu[3] = 3;
    idxbu[4] = 4;
    idxbu[5] = 5;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    
    lbu[0] = -0.85;
    ubu[0] = 0.85;
    lbu[1] = -0.41;
    ubu[1] = 0.41;
    lbu[2] = -0.41;
    ubu[2] = 0.41;
    lbu[3] = -0.085;
    ubu[3] = 0.085;
    lbu[4] = -0.041;
    ubu[4] = 0.041;
    lbu[5] = -0.041;
    ubu[5] = 0.041;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);












    // set up nonlinear constraints for stage 0 to N-1
    double* luh = calloc(2*NH, sizeof(double));
    double* lh = luh;
    double* uh = luh + NH;

    
    lh[12] = -1000000000000000;
    lh[13] = -1000000000000000;
    lh[14] = -1000000000000000;
    lh[15] = -1000000000000000;
    lh[16] = -1000000000000000;
    lh[17] = -1000000000000000;
    lh[18] = -1000000000000000;
    lh[19] = -1000000000000000;
    lh[20] = -1000000000000000;
    lh[21] = -1000000000000000;
    lh[22] = -1000000000000000;
    lh[23] = -1000000000000000;

    
    uh[0] = 1000000000000000;
    uh[1] = 1000000000000000;
    uh[2] = 1000000000000000;
    uh[3] = 1000000000000000;
    uh[4] = 1000000000000000;
    uh[5] = 1000000000000000;
    uh[6] = 1000000000000000;
    uh[7] = 1000000000000000;
    uh[8] = 1000000000000000;
    uh[9] = 1000000000000000;
    uh[10] = 1000000000000000;
    uh[11] = 1000000000000000;

    for (int i = 0; i < N; i++)
    {
        // nonlinear constraints for stages 0 to N-1
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun_jac",
                                      &capsule->nl_constr_h_fun_jac[i]);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun",
                                      &capsule->nl_constr_h_fun[i]);
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i,
                                      "nl_constr_h_fun_jac_hess", &capsule->nl_constr_h_fun_jac_hess[i]);
        
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, i, "uh", uh);
    }
    free(luh);



    /* terminal constraints */













    // set up nonlinear constraints for last stage
    double* luh_e = calloc(2*NHN, sizeof(double));
    double* lh_e = luh_e;
    double* uh_e = luh_e + NHN;
    
    lh_e[0] = -1000000000000000;
    lh_e[1] = -1000000000000000;
    lh_e[2] = -1000000000000000;
    lh_e[3] = -1000000000000000;
    lh_e[4] = -1000000000000000;
    lh_e[5] = -1000000000000000;
    lh_e[6] = -1000000000000000;
    lh_e[7] = -1000000000000000;
    lh_e[8] = -1000000000000000;
    lh_e[9] = -1000000000000000;
    lh_e[10] = -1000000000000000;
    lh_e[11] = -1000000000000000;
    lh_e[12] = -1000000000000000;
    lh_e[13] = -1000000000000000;
    lh_e[14] = -1000000000000000;
    lh_e[15] = -1000000000000000;
    lh_e[16] = -1000000000000000;
    lh_e[17] = -1000000000000000;
    lh_e[18] = -1000000000000000;
    lh_e[19] = -1000000000000000;
    lh_e[20] = -1000000000000000;
    lh_e[21] = -1000000000000000;
    lh_e[22] = -1000000000000000;
    lh_e[23] = -1000000000000000;
    lh_e[24] = -1000000000000000;
    lh_e[25] = -1000000000000000;
    lh_e[26] = -1000000000000000;
    lh_e[27] = -1000000000000000;
    lh_e[28] = -1000000000000000;
    lh_e[29] = -1000000000000000;
    lh_e[30] = -1000000000000000;
    lh_e[31] = -1000000000000000;
    lh_e[32] = -1000000000000000;
    lh_e[33] = -1000000000000000;
    lh_e[34] = -1000000000000000;
    lh_e[35] = -1000000000000000;
    lh_e[36] = -1000000000000000;
    lh_e[37] = -1000000000000000;
    lh_e[38] = -1000000000000000;
    lh_e[39] = -1000000000000000;
    lh_e[40] = -1000000000000000;
    lh_e[41] = -1000000000000000;
    lh_e[42] = -1000000000000000;
    lh_e[43] = -1000000000000000;
    lh_e[44] = -1000000000000000;
    lh_e[45] = -1000000000000000;
    lh_e[46] = -1000000000000000;
    lh_e[47] = -1000000000000000;
    lh_e[48] = -1000000000000000;
    lh_e[49] = -1000000000000000;
    lh_e[50] = -1000000000000000;
    lh_e[51] = -1000000000000000;
    lh_e[52] = -1000000000000000;
    lh_e[53] = -1000000000000000;
    lh_e[54] = -1000000000000000;
    lh_e[55] = -1000000000000000;
    lh_e[56] = -1000000000000000;
    lh_e[57] = -1000000000000000;
    lh_e[58] = -1000000000000000;
    lh_e[59] = -1000000000000000;
    lh_e[60] = -1000000000000000;
    lh_e[61] = -1000000000000000;
    lh_e[62] = -1000000000000000;
    lh_e[63] = -1000000000000000;
    lh_e[64] = -1000000000000000;
    lh_e[65] = -1000000000000000;
    lh_e[66] = -1000000000000000;
    lh_e[67] = -1000000000000000;
    lh_e[68] = -1000000000000000;
    lh_e[69] = -1000000000000000;
    lh_e[70] = -1000000000000000;
    lh_e[71] = -1000000000000000;
    lh_e[72] = -1000000000000000;
    lh_e[73] = -1000000000000000;
    lh_e[74] = -1000000000000000;
    lh_e[75] = -1000000000000000;
    lh_e[76] = -1000000000000000;
    lh_e[77] = -1000000000000000;
    lh_e[78] = -1000000000000000;
    lh_e[79] = -1000000000000000;
    lh_e[80] = -1000000000000000;
    lh_e[81] = -1000000000000000;
    lh_e[82] = -1000000000000000;
    lh_e[83] = -1000000000000000;
    lh_e[84] = -1000000000000000;
    lh_e[85] = -1000000000000000;
    lh_e[86] = -1000000000000000;
    lh_e[87] = -1000000000000000;
    lh_e[88] = -1000000000000000;
    lh_e[89] = -1000000000000000;
    lh_e[90] = -1000000000000000;
    lh_e[91] = -1000000000000000;
    lh_e[92] = -1000000000000000;
    lh_e[93] = -1000000000000000;
    lh_e[94] = -1000000000000000;
    lh_e[95] = -1000000000000000;
    lh_e[96] = -1000000000000000;
    lh_e[97] = -1000000000000000;
    lh_e[98] = -1000000000000000;
    lh_e[99] = -1000000000000000;
    lh_e[100] = -1000000000000000;
    lh_e[101] = -1000000000000000;
    lh_e[102] = -1000000000000000;
    lh_e[103] = -1000000000000000;
    lh_e[104] = -1000000000000000;
    lh_e[105] = -1000000000000000;
    lh_e[106] = -1000000000000000;
    lh_e[107] = -1000000000000000;
    lh_e[108] = -1000000000000000;
    lh_e[109] = -1000000000000000;
    lh_e[110] = -1000000000000000;
    lh_e[111] = -1000000000000000;
    lh_e[112] = -1000000000000000;
    lh_e[113] = -1000000000000000;
    lh_e[114] = -1000000000000000;
    lh_e[115] = -1000000000000000;
    lh_e[116] = -1000000000000000;
    lh_e[117] = -1000000000000000;
    lh_e[118] = -1000000000000000;
    lh_e[119] = -1000000000000000;
    lh_e[120] = -1000000000000000;
    lh_e[121] = -1000000000000000;
    lh_e[122] = -1000000000000000;
    lh_e[123] = -1000000000000000;
    lh_e[124] = -1000000000000000;
    lh_e[125] = -1000000000000000;
    lh_e[126] = -1000000000000000;
    lh_e[127] = -1000000000000000;

    
    uh_e[0] = 1.2;
    uh_e[1] = 0.1;
    uh_e[2] = 0.1;
    uh_e[3] = 1.2;
    uh_e[4] = 0.1;
    uh_e[5] = 0.1;
    uh_e[6] = 0.1893895542721036;
    uh_e[7] = 0.1893895542721036;
    uh_e[8] = 0.0984156608138769;
    uh_e[9] = 0.0984156608138769;
    uh_e[10] = 0.0984156608138769;
    uh_e[11] = 0.0984156608138769;
    uh_e[12] = 0.19972690630892312;
    uh_e[13] = 0.19972690630892312;
    uh_e[14] = 0.09461981464064924;
    uh_e[15] = 0.09461981464064924;
    uh_e[16] = 0.09461981464064924;
    uh_e[17] = 0.09461981464064924;
    uh_e[18] = 0.2110368465225312;
    uh_e[19] = 0.2110368465225312;
    uh_e[20] = 0.08991944513789338;
    uh_e[21] = 0.08991944513789338;
    uh_e[22] = 0.08991944513789338;
    uh_e[23] = 0.08991944513789338;
    uh_e[24] = 0.22342606782791527;
    uh_e[25] = 0.22342606782791527;
    uh_e[26] = 0.08518371852615683;
    uh_e[27] = 0.08518371852615683;
    uh_e[28] = 0.08518371852615683;
    uh_e[29] = 0.08518371852615683;
    uh_e[30] = 0.08085254317515395;
    uh_e[31] = 0.08085254317515395;
    uh_e[32] = 0.08085254317515395;
    uh_e[33] = 0.08085254317515395;
    uh_e[34] = 0.07709221895874693;
    uh_e[35] = 0.07709221895874693;
    uh_e[36] = 0.07709221895874693;
    uh_e[37] = 0.07709221895874693;
    uh_e[38] = 0.07392838238966074;
    uh_e[39] = 0.07392838238966074;
    uh_e[40] = 0.07392838238966074;
    uh_e[41] = 0.07392838238966074;
    uh_e[42] = 0.07132548589956735;
    uh_e[43] = 0.07132548589956735;
    uh_e[44] = 0.07132548589956735;
    uh_e[45] = 0.07132548589956735;
    uh_e[46] = 0.06922725464951465;
    uh_e[47] = 0.06922725464951465;
    uh_e[48] = 0.06922725464951465;
    uh_e[49] = 0.06922725464951465;
    uh_e[50] = 0.06757520158934696;
    uh_e[51] = 0.06757520158934696;
    uh_e[52] = 0.06757520158934696;
    uh_e[53] = 0.06757520158934696;
    uh_e[54] = 0.06631615544680305;
    uh_e[55] = 0.06631615544680305;
    uh_e[56] = 0.06631615544680305;
    uh_e[57] = 0.06631615544680305;
    uh_e[58] = 0.06540466666409214;
    uh_e[59] = 0.06540466666409214;
    uh_e[60] = 0.06540466666409214;
    uh_e[61] = 0.06540466666409214;
    uh_e[62] = 0.06480318686667315;
    uh_e[63] = 0.06480318686667315;
    uh_e[64] = 0.06480318686667315;
    uh_e[65] = 0.06480318686667315;
    uh_e[66] = 0.06448138500403758;
    uh_e[67] = 0.06448138500403758;
    uh_e[68] = 0.06448138500403758;
    uh_e[69] = 0.06448138500403758;
    uh_e[70] = 0.06441521526522961;
    uh_e[71] = 0.06441521526522961;
    uh_e[72] = 0.06441521526522961;
    uh_e[73] = 0.06441521526522961;
    uh_e[74] = 0.06458599803645261;
    uh_e[75] = 0.06458599803645261;
    uh_e[76] = 0.06458599803645261;
    uh_e[77] = 0.06458599803645261;
    uh_e[78] = 0.0649796121809427;
    uh_e[79] = 0.0649796121809427;
    uh_e[80] = 0.0649796121809427;
    uh_e[81] = 0.0649796121809427;
    uh_e[82] = 0.06558582451495656;
    uh_e[83] = 0.06558582451495656;
    uh_e[84] = 0.06558582451495656;
    uh_e[85] = 0.06558582451495656;
    uh_e[86] = 0.06639775227449232;
    uh_e[87] = 0.06639775227449232;
    uh_e[88] = 0.06639775227449232;
    uh_e[89] = 0.06639775227449232;
    uh_e[90] = 0.06741144366434573;
    uh_e[91] = 0.06741144366434573;
    uh_e[92] = 0.06741144366434573;
    uh_e[93] = 0.06741144366434573;
    uh_e[94] = 0.0686255594314776;
    uh_e[95] = 0.0686255594314776;
    uh_e[96] = 0.0686255594314776;
    uh_e[97] = 0.0686255594314776;
    uh_e[98] = 0.07004113976573303;
    uh_e[99] = 0.07004113976573303;
    uh_e[100] = 0.07004113976573303;
    uh_e[101] = 0.07004113976573303;
    uh_e[102] = 0.07166144340603295;
    uh_e[103] = 0.07166144340603295;
    uh_e[104] = 0.07166144340603295;
    uh_e[105] = 0.07166144340603295;
    uh_e[106] = 0.1;
    uh_e[107] = 0.1;
    uh_e[108] = 0.1;
    uh_e[109] = 0.1;
    uh_e[110] = 0.1;
    uh_e[111] = 0.1;
    uh_e[112] = 0.10625180277066076;
    uh_e[113] = 0.05358291646609077;
    uh_e[114] = 0.04955924582149085;
    uh_e[115] = 0.10625180277066076;
    uh_e[116] = 0.05358291646609077;
    uh_e[117] = 0.04955924582149085;
    uh_e[118] = 0.14950381769835486;
    uh_e[119] = 0.1462997056309771;
    uh_e[120] = 0.15200213079626354;
    uh_e[121] = 0.14950381769835486;
    uh_e[122] = 0.1462997056309771;
    uh_e[123] = 0.15200213079626354;
    uh_e[124] = 0.13890763045596186;
    uh_e[125] = 0.14028994721825228;
    uh_e[126] = 0.13890763045596186;
    uh_e[127] = 0.14028994721825228;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "nl_constr_h_fun_jac", &capsule->nl_constr_h_e_fun_jac);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "nl_constr_h_fun", &capsule->nl_constr_h_e_fun);
    
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "nl_constr_h_fun_jac_hess",
                                  &capsule->nl_constr_h_e_fun_jac_hess);
    
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "lh", lh_e);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, N, "uh", uh_e);
    free(luh_e);


}


/**
 * Internal function for astrobee_acados_create: step 6
 */
void astrobee_acados_create_6_set_opts(astrobee_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/


    bool nlp_solver_exact_hessian = true;
    // TODO: this if should not be needed! however, calling the setter with false leads to weird behavior. Investigate!
    if (nlp_solver_exact_hessian)
    {
        ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess", &nlp_solver_exact_hessian);
    }
    int exact_hess_dyn = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_dyn", &exact_hess_dyn);

    int exact_hess_cost = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_cost", &exact_hess_cost);

    int exact_hess_constr = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "exact_hess_constr", &exact_hess_constr);
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization", "fixed_step");int full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "full_step_dual", &full_step_dual);

    // set collocation type (relevant for implicit integrators)
    sim_collocation_type collocation_type = GAUSS_LEGENDRE;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_collocation_type", &collocation_type);

    // set up sim_method_num_steps
    // all sim_method_num_steps are identical
    int sim_method_num_steps = 1;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_steps", &sim_method_num_steps);

    // set up sim_method_num_stages
    // all sim_method_num_stages are identical
    int sim_method_num_stages = 4;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_num_stages", &sim_method_num_stages);

    int newton_iter_val = 3;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_newton_iter", &newton_iter_val);


    // set up sim_method_jac_reuse
    bool tmp_bool = (bool) 0;
    for (int i = 0; i < N; i++)
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "dynamics_jac_reuse", &tmp_bool);

    double nlp_solver_step_length = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "step_length", &nlp_solver_step_length);

    double levenberg_marquardt = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;

    
    // NOTE: there is no condensing happening here!
    qp_solver_cond_N = N;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);



    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);

int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);
    int qp_solver_cond_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_ric_alg", &qp_solver_cond_ric_alg);


    int ext_cost_num_hess = 0;
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, i, "cost_numerical_hessian", &ext_cost_num_hess);
    }
    ocp_nlp_solver_opts_set_at_stage(nlp_config, nlp_opts, N, "cost_numerical_hessian", &ext_cost_num_hess);
}


/**
 * Internal function for astrobee_acados_create: step 7
 */
void astrobee_acados_create_7_set_nlp_out(astrobee_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0
    


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for astrobee_acados_create: step 8
 */
//void astrobee_acados_create_8_create_solver(astrobee_solver_capsule* capsule)
//{
//    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
//}

/**
 * Internal function for astrobee_acados_create: step 9
 */
int astrobee_acados_create_9_precompute(astrobee_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int astrobee_acados_create_with_discretization(astrobee_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != ASTROBEE_N && !new_time_steps) {
        fprintf(stderr, "astrobee_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, ASTROBEE_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    astrobee_acados_create_1_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 3) create and set dimensions
    capsule->nlp_dims = astrobee_acados_create_2_create_and_set_dimensions(capsule);
    astrobee_acados_create_3_create_and_set_functions(capsule);

    // 4) set default parameters in functions
    astrobee_acados_create_4_set_default_parameters(capsule);

    // 5) create and set nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);
    astrobee_acados_create_5_set_nlp_in(capsule, N, new_time_steps);

    // 6) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    astrobee_acados_create_6_set_opts(capsule);

    // 7) create and set nlp_out
    // 7.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 7.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    astrobee_acados_create_7_set_nlp_out(capsule);

    // 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);
    //astrobee_acados_create_8_create_solver(capsule);

    // 9) do precomputations
    int status = astrobee_acados_create_9_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int astrobee_acados_update_qp_solver_cond_N(astrobee_solver_capsule* capsule, int qp_solver_cond_N)
{
    // 1) destroy solver
    ocp_nlp_solver_destroy(capsule->nlp_solver);

    // 2) set new value for "qp_cond_N"
    const int N = capsule->nlp_solver_plan->N;
    if(qp_solver_cond_N > N)
        printf("Warning: qp_solver_cond_N = %d > N = %d\n", qp_solver_cond_N, N);
    ocp_nlp_solver_opts_set(capsule->nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    // 3) continue with the remaining steps from astrobee_acados_create_with_discretization(...):
    // -> 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts);

    // -> 9) do precomputations
    int status = astrobee_acados_create_9_precompute(capsule);
    return status;
}


int astrobee_acados_reset(astrobee_solver_capsule* capsule, int reset_qp_solver_mem)
{

    // set initialization to all zeros

    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    int nx, nu, nv, ns, nz, ni, dim;

    double* buffer = calloc(NX+NU+NZ+2*NS+2*NSN+NBX+NBU+NG+NH+NPHI+NBX0+NBXN+NHN+NPHIN+NGN, sizeof(double));

    for(int i=0; i<N+1; i++)
    {
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "x", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "u", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "sl", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "su", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "lam", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "t", buffer);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "z", buffer);
        if (i<N)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "pi", buffer);
        }
    }

    free(buffer);
    return 0;
}




int astrobee_acados_update_params(astrobee_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 46;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }

    const int N = capsule->nlp_solver_plan->N;
    if (stage < N && stage >= 0)
    {
        capsule->forw_vde_casadi[stage].set_param(capsule->forw_vde_casadi+stage, p);
        capsule->expl_ode_fun[stage].set_param(capsule->expl_ode_fun+stage, p);
        capsule->hess_vde_casadi[stage].set_param(capsule->hess_vde_casadi+stage, p);
    

        // constraints
    
        capsule->nl_constr_h_fun_jac[stage].set_param(capsule->nl_constr_h_fun_jac+stage, p);
        capsule->nl_constr_h_fun[stage].set_param(capsule->nl_constr_h_fun+stage, p);
        capsule->nl_constr_h_fun_jac_hess[stage].set_param(capsule->nl_constr_h_fun_jac_hess+stage, p);

        // cost
        if (stage == 0)
        {
            capsule->ext_cost_0_fun.set_param(&capsule->ext_cost_0_fun, p);
            capsule->ext_cost_0_fun_jac.set_param(&capsule->ext_cost_0_fun_jac, p);
            capsule->ext_cost_0_fun_jac_hess.set_param(&capsule->ext_cost_0_fun_jac_hess, p);
        
        }
        else // 0 < stage < N
        {
            capsule->ext_cost_fun[stage-1].set_param(capsule->ext_cost_fun+stage-1, p);
            capsule->ext_cost_fun_jac[stage-1].set_param(capsule->ext_cost_fun_jac+stage-1, p);
            capsule->ext_cost_fun_jac_hess[stage-1].set_param(capsule->ext_cost_fun_jac_hess+stage-1, p);
        }
    }

    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        capsule->ext_cost_e_fun.set_param(&capsule->ext_cost_e_fun, p);
        capsule->ext_cost_e_fun_jac.set_param(&capsule->ext_cost_e_fun_jac, p);
        capsule->ext_cost_e_fun_jac_hess.set_param(&capsule->ext_cost_e_fun_jac_hess, p);
    
        // constraints
    
        capsule->nl_constr_h_e_fun_jac.set_param(&capsule->nl_constr_h_e_fun_jac, p);
        capsule->nl_constr_h_e_fun.set_param(&capsule->nl_constr_h_e_fun, p);
        capsule->nl_constr_h_e_fun_jac_hess.set_param(&capsule->nl_constr_h_e_fun_jac_hess, p);
    
    }

    return solver_status;
}


int astrobee_acados_update_params_sparse(astrobee_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    int solver_status = 0;

    int casadi_np = 46;
    if (casadi_np < n_update) {
        printf("astrobee_acados_update_params_sparse: trying to set %d parameters for external functions."
            " External function has %d parameters. Exiting.\n", n_update, casadi_np);
        exit(1);
    }
    // for (int i = 0; i < n_update; i++)
    // {
    //     if (idx[i] > casadi_np) {
    //         printf("astrobee_acados_update_params_sparse: attempt to set parameters with index %d, while"
    //             " external functions only has %d parameters. Exiting.\n", idx[i], casadi_np);
    //         exit(1);
    //     }
    //     printf("param %d value %e\n", idx[i], p[i]);
    // }
    const int N = capsule->nlp_solver_plan->N;
    if (stage < N && stage >= 0)
    {
        capsule->forw_vde_casadi[stage].set_param_sparse(capsule->forw_vde_casadi+stage, n_update, idx, p);
        capsule->expl_ode_fun[stage].set_param_sparse(capsule->expl_ode_fun+stage, n_update, idx, p);
        capsule->hess_vde_casadi[stage].set_param_sparse(capsule->hess_vde_casadi+stage, n_update, idx, p);
    

        // constraints
    
        capsule->nl_constr_h_fun_jac[stage].set_param_sparse(capsule->nl_constr_h_fun_jac+stage, n_update, idx, p);
        capsule->nl_constr_h_fun[stage].set_param_sparse(capsule->nl_constr_h_fun+stage, n_update, idx, p);
        capsule->nl_constr_h_fun_jac_hess[stage].set_param_sparse(capsule->nl_constr_h_fun_jac_hess+stage, n_update, idx, p);

        // cost
        if (stage == 0)
        {
            capsule->ext_cost_0_fun.set_param_sparse(&capsule->ext_cost_0_fun, n_update, idx, p);
            capsule->ext_cost_0_fun_jac.set_param_sparse(&capsule->ext_cost_0_fun_jac, n_update, idx, p);
            capsule->ext_cost_0_fun_jac_hess.set_param_sparse(&capsule->ext_cost_0_fun_jac_hess, n_update, idx, p);
        
        }
        else // 0 < stage < N
        {
            capsule->ext_cost_fun[stage-1].set_param_sparse(capsule->ext_cost_fun+stage-1, n_update, idx, p);
            capsule->ext_cost_fun_jac[stage-1].set_param_sparse(capsule->ext_cost_fun_jac+stage-1, n_update, idx, p);
            capsule->ext_cost_fun_jac_hess[stage-1].set_param_sparse(capsule->ext_cost_fun_jac_hess+stage-1, n_update, idx, p);
        }
    }

    else // stage == N
    {
        // terminal shooting node has no dynamics
        // cost
        capsule->ext_cost_e_fun.set_param_sparse(&capsule->ext_cost_e_fun, n_update, idx, p);
        capsule->ext_cost_e_fun_jac.set_param_sparse(&capsule->ext_cost_e_fun_jac, n_update, idx, p);
        capsule->ext_cost_e_fun_jac_hess.set_param_sparse(&capsule->ext_cost_e_fun_jac_hess, n_update, idx, p);
    
        // constraints
    
        capsule->nl_constr_h_e_fun_jac.set_param_sparse(&capsule->nl_constr_h_e_fun_jac, n_update, idx, p);
        capsule->nl_constr_h_e_fun.set_param_sparse(&capsule->nl_constr_h_e_fun, n_update, idx, p);
        capsule->nl_constr_h_e_fun_jac_hess.set_param_sparse(&capsule->nl_constr_h_e_fun_jac_hess, n_update, idx, p);
    
    }


    return 0;
}

int astrobee_acados_solve(astrobee_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}


int astrobee_acados_free(astrobee_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_param_casadi_free(&capsule->forw_vde_casadi[i]);
        external_function_param_casadi_free(&capsule->expl_ode_fun[i]);
        external_function_param_casadi_free(&capsule->hess_vde_casadi[i]);
    }
    free(capsule->forw_vde_casadi);
    free(capsule->expl_ode_fun);
    free(capsule->hess_vde_casadi);

    // cost
    external_function_param_casadi_free(&capsule->ext_cost_0_fun);
    external_function_param_casadi_free(&capsule->ext_cost_0_fun_jac);
    external_function_param_casadi_free(&capsule->ext_cost_0_fun_jac_hess);
    for (int i = 0; i < N - 1; i++)
    {
        external_function_param_casadi_free(&capsule->ext_cost_fun[i]);
        external_function_param_casadi_free(&capsule->ext_cost_fun_jac[i]);
        external_function_param_casadi_free(&capsule->ext_cost_fun_jac_hess[i]);
    }
    free(capsule->ext_cost_fun);
    free(capsule->ext_cost_fun_jac);
    free(capsule->ext_cost_fun_jac_hess);
    external_function_param_casadi_free(&capsule->ext_cost_e_fun);
    external_function_param_casadi_free(&capsule->ext_cost_e_fun_jac);
    external_function_param_casadi_free(&capsule->ext_cost_e_fun_jac_hess);

    // constraints
    for (int i = 0; i < N; i++)
    {
        external_function_param_casadi_free(&capsule->nl_constr_h_fun_jac[i]);
        external_function_param_casadi_free(&capsule->nl_constr_h_fun[i]);
    }
    for (int i = 0; i < N; i++)
    {
        external_function_param_casadi_free(&capsule->nl_constr_h_fun_jac_hess[i]);
    }
    free(capsule->nl_constr_h_fun_jac);
    free(capsule->nl_constr_h_fun);
    free(capsule->nl_constr_h_fun_jac_hess);
    external_function_param_casadi_free(&capsule->nl_constr_h_e_fun_jac);
    external_function_param_casadi_free(&capsule->nl_constr_h_e_fun);
    external_function_param_casadi_free(&capsule->nl_constr_h_e_fun_jac_hess);

    return 0;
}


void astrobee_acados_print_stats(astrobee_solver_capsule* capsule)
{
    int sqp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "sqp_iter", &sqp_iter);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "stat_m", &stat_m);

    
    double stat[1200];
    ocp_nlp_get(capsule->nlp_config, capsule->nlp_solver, "statistics", stat);

    int nrow = sqp_iter+1 < stat_m ? sqp_iter+1 : stat_m;

    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\talpha");
    if (stat_n > 8)
        printf("\t\tqp_res_stat\tqp_res_eq\tqp_res_ineq\tqp_res_comp");
    printf("\n");
    printf("iter\tqp_stat\tqp_iter\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            tmp_int = (int) stat[i + j * nrow];
            printf("%d\t", tmp_int);
        }
        printf("\n");
    }
}

int astrobee_acados_custom_update(astrobee_solver_capsule* capsule, double* data, int data_len)
{
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *astrobee_acados_get_nlp_in(astrobee_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *astrobee_acados_get_nlp_out(astrobee_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *astrobee_acados_get_sens_out(astrobee_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *astrobee_acados_get_nlp_solver(astrobee_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *astrobee_acados_get_nlp_config(astrobee_solver_capsule* capsule) { return capsule->nlp_config; }
void *astrobee_acados_get_nlp_opts(astrobee_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *astrobee_acados_get_nlp_dims(astrobee_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *astrobee_acados_get_nlp_plan(astrobee_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
