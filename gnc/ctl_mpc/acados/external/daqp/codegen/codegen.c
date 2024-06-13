#include "codegen.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include "types.h"
#include "api.h"


void render_daqp_workspace(DAQPWorkspace* work, const char *fname, const char *dir){
    char *hfname= malloc(strlen(dir)+strlen(fname) + 3); // two chars for .h and 1 for terminator
    char *hguard= malloc(strlen(fname) + 3); // two chars for _H and 1 for terminator
    char *cfname= malloc(strlen(dir)+strlen(fname) + 3); // two chars for .c and 1 for terminator

    // Header file
    strcpy(hfname, dir);
    strcat(hfname, fname);
    strcat(hfname, ".h");
    FILE* fh= fopen(hfname, "w");

    // Source file
    strcpy(cfname, dir);
    strcat(cfname, fname);
    strcat(cfname, ".c");
    FILE* fsrc= fopen(cfname, "w");

    // Header guard
    strcpy(hguard, fname);
    strcat(hguard, "_H");
    char *s = hguard;
    while(*s){
        *s = toupper((unsigned char) *s);
        s++;
    }
    fprintf(fh, "#ifndef %s\n",   hguard);
    fprintf(fh, "#define %s\n\n", hguard);

    // Include types and constants
    fprintf(fh, "#include \"types.h\"\n");
    fprintf(fh, "#include \"constants.h\"\n");

    fprintf(fsrc, "#include \"types.h\"\n");
    fprintf(fsrc, "#include \"constants.h\"\n");

    //Write settings
    fprintf(fh, "// Settings prototype\n");
    fprintf(fh, "extern DAQPSettings settings;\n\n");
    write_daqp_settings_src(fsrc,work->settings);

    // Write BnB struct
    if(work->bnb != NULL){
        write_daqp_bnb_h(fh,work->bnb,work->n);
        write_daqp_bnb_src(fsrc,work->bnb,work->n);
    }

    // TODO Check soft constraints 

    //Write workspace
    write_daqp_workspace_h(fh,work);
    write_daqp_workspace_src(fsrc,work);


    // Close header guard 
    fprintf(fh, "#endif // ifndef %s\n", hguard);

    fclose(fh);
    fclose(fsrc);

    free(hfname);
    free(cfname);
    free(hguard);
}

void write_daqp_workspace_h(FILE *f, DAQPWorkspace* work){

    const int n = work->n;
    const int m = work->m;
    const int ms = work->ms;
    int ntot = n;
    for(int i = 0; i < m ; i++) 
        if(work->sense[i] & SOFT) ntot++;

    // Refdefine NX, N_CONSTR and N_SIMPLE to static
    fprintf(f, "#undef NX\n");
    fprintf(f, "#define NX %d\n",n);
    fprintf(f, "#undef N_CONSTR\n");
    fprintf(f, "#define N_CONSTR %d\n",m);
    fprintf(f, "#undef N_SIMPLE\n");
    fprintf(f, "#define N_SIMPLE %d \n",ms);

    fprintf(f, "// Workspace prototypes\n");

    fprintf(f, "extern c_float M[%d];\n", (m-ms)*n);
    fprintf(f, "extern c_float dupper[%d];\n", m);
    fprintf(f, "extern c_float dlower[%d];\n", m);
    fprintf(f, "extern c_float Rinv[%d];\n", n*(n+1)/2);
    fprintf(f, "extern c_float v[%d];\n", n);
    fprintf(f, "extern int sense[%d];\n\n", m);
    fprintf(f, "extern c_float scaling[%d];\n\n", m);

    fprintf(f, "extern c_float x[%d];\n", n+1);
    fprintf(f, "extern c_float xold[%d];\n\n", n+1);

    fprintf(f, "extern c_float lam[%d];\n", ntot+1);
    fprintf(f, "extern c_float lam_star[%d];\n", ntot+1);
    fprintf(f, "extern c_float u[%d];\n\n", n+1);

    fprintf(f, "extern c_float L[%d];\n", (ntot+1)*(ntot+2)/2);
    fprintf(f, "extern c_float D[%d];\n", ntot+1);
    fprintf(f, "extern c_float xldl[%d];\n", ntot+1);
    fprintf(f, "extern c_float zldl[%d];\n\n", ntot+1);

    fprintf(f, "extern int WS[%d];\n\n", ntot+1);

    fprintf(f, "extern DAQPWorkspace daqp_work;\n\n");
}

void write_daqp_workspace_src(FILE* f, DAQPWorkspace* work){
    int n = work->n;
    int m = work->m;
    int ms = work->ms;
    int ntot = n;
    for(int i = 0; i < m ; i++) 
        if(work->sense[i] & SOFT) ntot++;

    fprintf(f, "// Workspace\n");
    // LDP data
    write_float_array(f,work->M,(m-ms)*n,"M");
    //write_float_array(f,work->dupper,m,"dupper");
    //write_float_array(f,work->dlower,m,"dlower");
    fprintf(f, "c_float dupper[%d];\n", m);
    fprintf(f, "c_float dlower[%d];\n", m);
    write_float_array(f,work->Rinv,n*(n+1)/2,"Rinv");
    //write_float_array(f,work->v,n, "v");
    write_int_array(f,work->sense, m,"sense");
    write_float_array(f,work->scaling, m,"scaling");

    // Iteratates
    fprintf(f, "c_float x[%d];\n", n+1);
    fprintf(f, "c_float xold[%d];\n\n", n+1);

    fprintf(f, "c_float lam[%d];\n", ntot+1);
    fprintf(f, "c_float lam_star[%d];\n", ntot+1);
    fprintf(f, "c_float u[%d];\n\n", n+1);

    fprintf(f, "c_float L[%d];\n", (ntot+1)*(ntot+2)/2);
    fprintf(f, "c_float D[%d];\n", ntot+1);
    fprintf(f, "c_float xldl[%d];\n", ntot+1);
    fprintf(f, "c_float zldl[%d];\n\n", ntot+1);

    fprintf(f, "int WS[%d];\n\n", ntot+1);

    //Workspace struct
    fprintf(f, "DAQPWorkspace daqp_work= {\n");
    fprintf(f, "NULL,\n"); // DAQPProblem
    fprintf(f, "%d, %d, %d,\n",n,m,ms); // dimensions 
    fprintf(f, "M, dupper, dlower, Rinv, NULL, sense,\n"); //LDP 
    fprintf(f, "scaling,\n"); // scaling
    fprintf(f, "x, xold,\n");
    fprintf(f, "lam, lam_star, u, %d,\n",-1); // fval
    fprintf(f, "L, D, xldl,zldl,%d,\n",0); // reuse_ind
    fprintf(f, "WS, %d,\n",0); //n_active
    fprintf(f, "%d,%d,\n",0,-1); //iterations + sing_id
    fprintf(f, "%f,\n",0.0); // Soft slack
    fprintf(f, "&settings, \n");
    //if(!has_binary)
    if(work->bnb == NULL)
        fprintf(f, "NULL};\n\n");
    else
        fprintf(f, "&daqp_bnb_work};\n\n");


}

void write_daqp_settings_src(FILE*  f, DAQPSettings* settings){

    fprintf(f, "// Settings\n");
    fprintf(f, "DAQPSettings settings = {");
    fprintf(f, "(c_float)%.20f, ", settings->primal_tol);
    fprintf(f, "(c_float)%.20f, ", settings->dual_tol);
    fprintf(f, "(c_float)%.20f, ", settings->zero_tol);
    fprintf(f, "(c_float)%.20f, ", settings->pivot_tol);
    fprintf(f, "(c_float)%.20f, ", settings->progress_tol);

    fprintf(f, "%d, ",             settings->cycle_tol);
    fprintf(f, "%d, ",             settings->iter_limit);
    fprintf(f, "(c_float)%.20f, ", settings->fval_bound);

    fprintf(f, "(c_float)%.20f, ", settings->eps_prox);
    fprintf(f, "(c_float)%.20f, ", settings->eta_prox);

    fprintf(f, "(c_float)%.20f,", settings->rho_soft);

    fprintf(f, "(c_float)%.20f,", settings->rel_subopt);
    fprintf(f, "(c_float)%.20f", settings->abs_subopt);

    fprintf(f, "};\n\n");
}

void write_daqp_bnb_h(FILE*  f, DAQPBnB* bnb, const int n){
    fprintf(f, "#define DAQP_BNB\n");
    fprintf(f, "extern int bin_ids[%d];\n", bnb->nb);
    fprintf(f, "extern DAQPNode tree[%d];\n", bnb->nb+1);
    fprintf(f, "extern int tree_WS[%d];\n", (n+1)*(bnb->nb+1));
    fprintf(f, "extern int fixed_ids[%d];\n", bnb->nb+1);
    fprintf(f, "extern DAQPBnB daqp_bnb_work;\n\n");
}
void write_daqp_bnb_src(FILE*  f, DAQPBnB* bnb, const int n){
    if(bnb==NULL) return;
    fprintf(f, "// BnB \n");

    write_int_array(f,bnb->bin_ids, bnb->nb,"bin_ids");
    fprintf(f, "DAQPNode tree[%d];\n", bnb->nb+1);
    fprintf(f, "int tree_WS[%d];\n", (n+1)*(bnb->nb+1));
    fprintf(f, "int fixed_ids[%d];\n", bnb->nb+1);

    fprintf(f, "DAQPBnB daqp_bnb_work= {");
    fprintf(f, "bin_ids, ");
    fprintf(f, "(int)%d, ", bnb->nb);
    fprintf(f, "(int)%d, ", bnb->neq);

    fprintf(f, "tree, ");
    fprintf(f, "(int)%d, ", 0); // n_nodes

    fprintf(f, "tree_WS, ");
    fprintf(f, "(int)%d, ", 0); // nWS
    fprintf(f, "(int)%d, ", 0); // n_clean
    fprintf(f, "fixed_ids, "); // n_clean

    fprintf(f, "(int)%d, ", 0); // nodecount
    fprintf(f, "(int)%d, ", 0); // itercount

    fprintf(f, "};\n\n");
}

void write_float_array(FILE *f, c_float* a, const int N, const char *name){

    if(a == NULL)
        fprintf(f, "c_float %s[%d];\n", name, N);
    else{

        fprintf(f, "c_float %s[%d] = {\n", name, N);
        for(int i = 0; i < N; i++)
            fprintf(f, "(c_float)%.20f,\n", a[i]);
        fprintf(f, "};\n");
    }
}

void write_int_array(FILE *f, int* a, const int N, const char *name){
    if(a == NULL)
        fprintf(f, "int %s[%d];\n", name, N);
    else{
        fprintf(f, "int %s[%d] = {\n", name, N);
        for(int i = 0; i < N; i++)
            fprintf(f, "(int)%i,\n", a[i]);
        fprintf(f, "};\n");
    }
}
