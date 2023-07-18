/***************
* dim
***************/
/* N */
int N = 3;
/* nx */
static int nnx[] = {2, 2, 2, 2, };
int *nx = nnx;
/* nu */
static int nnu[] = {2, 2, 2, 0, };
int *nu = nnu;
/* nbx */
static int nnbx[] = {2, 0, 0, 0, };
int *nbx = nnbx;
/* nbu */
static int nnbu[] = {0, 0, 0, 0, };
int *nbu = nnbu;
/* ng */
static int nng[] = {0, 0, 0, 0, };
int *ng = nng;
/* nq */
static int nnq[] = {1, 1, 1, 0, };
int *nq = nnq;
/* ns */
static int nns[] = {0, 0, 0, 0, };
int *ns = nns;
/* nsbx */
static int nnsbx[] = {0, 0, 0, 0, };
int *nsbx = nnsbx;
/* nsbu */
static int nnsbu[] = {0, 0, 0, 0, };
int *nsbu = nnsbu;
/* nsg */
static int nnsg[] = {0, 0, 0, 0, };
int *nsg = nnsg;
/* nsq */
static int nnsq[] = {0, 0, 0, 0, };
int *nsq = nnsq;
/***************
* qp
***************/
/* A */
static double A0[] = {9.978670666983173e-01, -2.613133967924819e-02, 3.762912913811738e-02, 9.981402421168042e-01, };
static double A1[] = {9.978670666983173e-01, -2.613133967924819e-02, 3.762912913811738e-02, 9.981402421168042e-01, };
static double A2[] = {9.978670666983173e-01, -2.613133967924819e-02, 3.762912913811738e-02, 9.981402421168042e-01, };
static double *AA[] = {A0, A1, A2, };
double **hA = AA;
/* B */
static double B0[] = {1.491067643872558e-02, -1.951239082455206e-04, 2.341486898946246e-04, 1.242726354338869e-02, };
static double B1[] = {1.491067643872558e-02, -1.951239082455206e-04, 2.341486898946246e-04, 1.242726354338869e-02, };
static double B2[] = {1.491067643872558e-02, -1.951239082455206e-04, 2.341486898946246e-04, 1.242726354338869e-02, };
static double *BB[] = {B0, B1, B2, };
double **hB = BB;
/* b */
static double b0[] = {-5.538021593241391e-02, -2.939262819670726e+00, };
static double b1[] = {-5.538021593241391e-02, -2.939262819670726e+00, };
static double b2[] = {-5.538021593241391e-02, -2.939262819670726e+00, };
static double *bb[] = {b0, b1, b2, };
double **hb = bb;
/* Q */
static double Q0[] = {5.000000000000000e-04, 0.000000000000000e+00, 0.000000000000000e+00, 5.000000000000000e-04, };
static double Q1[] = {5.000000000000000e-04, 0.000000000000000e+00, 0.000000000000000e+00, 5.000000000000000e-04, };
static double Q2[] = {5.000000000000000e-04, 0.000000000000000e+00, 0.000000000000000e+00, 5.000000000000000e-04, };
static double Q3[] = {1.000000000000000e-04, 0.000000000000000e+00, 0.000000000000000e+00, 1.000000000000000e-04, };
static double *QQ[] = {Q0, Q1, Q2, Q3, };
double **hQ = QQ;
/* S */
static double S0[] = {0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, };
static double S1[] = {0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, };
static double S2[] = {0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, };
static double S3[] = {};
static double *SS[] = {S0, S1, S2, S3, };
double **hS = SS;
/* R */
static double R0[] = {5.000000000000000e-08, 0.000000000000000e+00, 0.000000000000000e+00, 5.000000000000000e-08, };
static double R1[] = {5.000000000000000e-08, 0.000000000000000e+00, 0.000000000000000e+00, 5.000000000000000e-08, };
static double R2[] = {5.000000000000000e-08, 0.000000000000000e+00, 0.000000000000000e+00, 5.000000000000000e-08, };
static double R3[] = {};
static double *RR[] = {R0, R1, R2, R3, };
double **hR = RR;
/* r */
static double r0[] = {3.888688210122473e-07, -1.153156516231371e-05, };
static double r1[] = {3.888688210122473e-07, -1.153156516231371e-05, };
static double r2[] = {3.888688210122473e-07, -1.153156516231371e-05, };
static double r3[] = {};
static double *rr[] = {r0, r1, r2, r3, };
double **hr = rr;
/* q */
static double q0[] = {1.475579164822193e-03, -1.475579164822193e-03, };
static double q1[] = {1.475579164822193e-03, -1.475579164822193e-03, };
static double q2[] = {1.475579164822193e-03, -1.475579164822193e-03, };
static double q3[] = {2.951158329644385e-04, -2.951158329644386e-04, };
static double *qq[] = {q0, q1, q2, q3, };
double **hq = qq;
/* idxbu */
static int idxbu0[] = {};
static int idxbu1[] = {};
static int idxbu2[] = {};
static int idxbu3[] = {};
static int *iidxbu[] = {idxbu0, idxbu1, idxbu2, idxbu3, };
int **hidxbu = iidxbu;
/* lbu */
static double lbu0[] = {};
static double lbu1[] = {};
static double lbu2[] = {};
static double lbu3[] = {};
static double *llbu[] = {lbu0, lbu1, lbu2, lbu3, };
double **hlbu = llbu;
/* lbu_mask */
static double lbu_mask0[] = {};
static double lbu_mask1[] = {};
static double lbu_mask2[] = {};
static double lbu_mask3[] = {};
static double *llbu_mask[] = {lbu_mask0, lbu_mask1, lbu_mask2, lbu_mask3, };
double **hlbu_mask = llbu_mask;
/* ubu */
static double ubu0[] = {};
static double ubu1[] = {};
static double ubu2[] = {};
static double ubu3[] = {};
static double *uubu[] = {ubu0, ubu1, ubu2, ubu3, };
double **hubu = uubu;
/* ubu_mask */
static double ubu_mask0[] = {};
static double ubu_mask1[] = {};
static double ubu_mask2[] = {};
static double ubu_mask3[] = {};
static double *uubu_mask[] = {ubu_mask0, ubu_mask1, ubu_mask2, ubu_mask3, };
double **hubu_mask = uubu_mask;
/* idxbx */
static int idxbx0[] = {0, 1, };
static int idxbx1[] = {};
static int idxbx2[] = {};
static int idxbx3[] = {};
static int *iidxbx[] = {idxbx0, idxbx1, idxbx2, idxbx3, };
int **hidxbx = iidxbx;
/* lbx */
static double lbx0[] = {-1.000000000000000e+01, -5.000000000000000e+00, };
static double lbx1[] = {};
static double lbx2[] = {};
static double lbx3[] = {};
static double *llbx[] = {lbx0, lbx1, lbx2, lbx3, };
double **hlbx = llbx;
/* lbx_mask */
static double lbx_mask0[] = {1.000000000000000e+00, 1.000000000000000e+00, };
static double lbx_mask1[] = {};
static double lbx_mask2[] = {};
static double lbx_mask3[] = {};
static double *llbx_mask[] = {lbx_mask0, lbx_mask1, lbx_mask2, lbx_mask3, };
double **hlbx_mask = llbx_mask;
/* ubx */
static double ubx0[] = {-1.000000000000000e+01, -5.000000000000000e+00, };
static double ubx1[] = {};
static double ubx2[] = {};
static double ubx3[] = {};
static double *uubx[] = {ubx0, ubx1, ubx2, ubx3, };
double **hubx = uubx;
/* ubx_mask */
static double ubx_mask0[] = {1.000000000000000e+00, 1.000000000000000e+00, };
static double ubx_mask1[] = {};
static double ubx_mask2[] = {};
static double ubx_mask3[] = {};
static double *uubx_mask[] = {ubx_mask0, ubx_mask1, ubx_mask2, ubx_mask3, };
double **hubx_mask = uubx_mask;
/* C */
static double C0[] = {};
static double C1[] = {};
static double C2[] = {};
static double C3[] = {};
static double *CC[] = {C0, C1, C2, C3, };
double **hC = CC;
/* D */
static double D0[] = {};
static double D1[] = {};
static double D2[] = {};
static double D3[] = {};
static double *DD[] = {D0, D1, D2, D3, };
double **hD = DD;
/* lg */
static double lg0[] = {};
static double lg1[] = {};
static double lg2[] = {};
static double lg3[] = {};
static double *llg[] = {lg0, lg1, lg2, lg3, };
double **hlg = llg;
/* lg_mask */
static double lg_mask0[] = {};
static double lg_mask1[] = {};
static double lg_mask2[] = {};
static double lg_mask3[] = {};
static double *llg_mask[] = {lg_mask0, lg_mask1, lg_mask2, lg_mask3, };
double **hlg_mask = llg_mask;
/* ug */
static double ug0[] = {};
static double ug1[] = {};
static double ug2[] = {};
static double ug3[] = {};
static double *uug[] = {ug0, ug1, ug2, ug3, };
double **hug = uug;
/* ug_mask */
static double ug_mask0[] = {};
static double ug_mask1[] = {};
static double ug_mask2[] = {};
static double ug_mask3[] = {};
static double *uug_mask[] = {ug_mask0, ug_mask1, ug_mask2, ug_mask3, };
double **hug_mask = uug_mask;
/* Qq */
static double Qq0[] = {0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, };
static double Qq1[] = {0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, };
static double Qq2[] = {0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, };
static double Qq3[] = {};
static double *QQq[] = {Qq0, Qq1, Qq2, Qq3, };
double **hQq = QQq;
/* Sq */
static double Sq0[] = {0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, };
static double Sq1[] = {0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, };
static double Sq2[] = {0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, };
static double Sq3[] = {};
static double *SSq[] = {Sq0, Sq1, Sq2, Sq3, };
double **hSq = SSq;
/* Rq */
static double Rq0[] = {2.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 2.000000000000000e+00, };
static double Rq1[] = {2.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 2.000000000000000e+00, };
static double Rq2[] = {2.000000000000000e+00, 0.000000000000000e+00, 0.000000000000000e+00, 2.000000000000000e+00, };
static double Rq3[] = {};
static double *RRq[] = {Rq0, Rq1, Rq2, Rq3, };
double **hRq = RRq;
/* qq */
static double qq0[] = {};
static double qq1[] = {};
static double qq2[] = {};
static double qq3[] = {};
static double *qqq[] = {qq0, qq1, qq2, qq3, };
double **hqq = qqq;
/* rq */
static double rq0[] = {};
static double rq1[] = {};
static double rq2[] = {};
static double rq3[] = {};
static double *rrq[] = {rq0, rq1, rq2, rq3, };
double **hrq = rrq;
/* uq */
static double uq0[] = {8.410000000000000e+04, };
static double uq1[] = {8.410000000000000e+04, };
static double uq2[] = {8.410000000000000e+04, };
static double uq3[] = {};
static double *uuq[] = {uq0, uq1, uq2, uq3, };
double **huq = uuq;
/* uq_mask */
static double uq_mask0[] = {};
static double uq_mask1[] = {};
static double uq_mask2[] = {};
static double uq_mask3[] = {};
static double *uuq_mask[] = {uq_mask0, uq_mask1, uq_mask2, uq_mask3, };
double **huq_mask = uuq_mask;
/* Zl */
double **hZl;
/* Zu */
double **hZu;
/* zl */
double **hzl;
/* zu */
double **hzu;
/* idxs_rev */
int **hidxs_rev;
/* idxs */
int **hidxs;
/* lls */
double **hlls;
/* lus */
double **hlus;
/***************
* arg
***************/
/* mode */
int mode = 1;
/* iter_max */
int iter_max = 40;
/* alpha_min */
double alpha_min = 1.000000000000000e-12;
/* mu0 */
double mu0 = 1.000000000000000e+01;
/* tol_stat */
double tol_stat = 1.000000000000000e-08;
/* tol_eq */
double tol_eq = 1.000000000000000e-08;
/* tol_ineq */
double tol_ineq = 1.000000000000000e-08;
/* tol_comp */
double tol_comp = 1.000000000000000e-08;
/* reg_prim */
double reg_prim = 1.000000000000000e-12;
/* warm_start */
int warm_start = 0;
/* pred_corr */
int pred_corr = 1;
/* ric_alg */
int ric_alg = 1;
/* split_step */
int split_step = 1;
