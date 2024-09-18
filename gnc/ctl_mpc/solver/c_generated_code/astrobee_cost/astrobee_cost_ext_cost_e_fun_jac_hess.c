/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) astrobee_cost_ext_cost_e_fun_jac_hess_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_mtimes CASADI_PREFIX(mtimes)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_trans CASADI_PREFIX(trans)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

void casadi_mtimes(const casadi_real* x, const casadi_int* sp_x, const casadi_real* y, const casadi_int* sp_y, casadi_real* z, const casadi_int* sp_z, casadi_real* w, casadi_int tr) {
  casadi_int ncol_x, ncol_y, ncol_z, cc;
  const casadi_int *colind_x, *row_x, *colind_y, *row_y, *colind_z, *row_z;
  ncol_x = sp_x[1];
  colind_x = sp_x+2; row_x = sp_x + 2 + ncol_x+1;
  ncol_y = sp_y[1];
  colind_y = sp_y+2; row_y = sp_y + 2 + ncol_y+1;
  ncol_z = sp_z[1];
  colind_z = sp_z+2; row_z = sp_z + 2 + ncol_z+1;
  if (tr) {
    for (cc=0; cc<ncol_z; ++cc) {
      casadi_int kk;
      for (kk=colind_y[cc]; kk<colind_y[cc+1]; ++kk) {
        w[row_y[kk]] = y[kk];
      }
      for (kk=colind_z[cc]; kk<colind_z[cc+1]; ++kk) {
        casadi_int kk1;
        casadi_int rr = row_z[kk];
        for (kk1=colind_x[rr]; kk1<colind_x[rr+1]; ++kk1) {
          z[kk] += x[kk1] * w[row_x[kk1]];
        }
      }
    }
  } else {
    for (cc=0; cc<ncol_y; ++cc) {
      casadi_int kk;
      for (kk=colind_z[cc]; kk<colind_z[cc+1]; ++kk) {
        w[row_z[kk]] = z[kk];
      }
      for (kk=colind_y[cc]; kk<colind_y[cc+1]; ++kk) {
        casadi_int kk1;
        casadi_int rr = row_y[kk];
        for (kk1=colind_x[rr]; kk1<colind_x[rr+1]; ++kk1) {
          w[row_x[kk1]] += x[kk1]*y[kk];
        }
      }
      for (kk=colind_z[cc]; kk<colind_z[cc+1]; ++kk) {
        z[kk] = w[row_z[kk]];
      }
    }
  }
}

void casadi_trans(const casadi_real* x, const casadi_int* sp_x, casadi_real* y,
    const casadi_int* sp_y, casadi_int* tmp) {
  casadi_int ncol_x, nnz_x, ncol_y, k;
  const casadi_int* row_x, *colind_y;
  ncol_x = sp_x[1];
  nnz_x = sp_x[2 + ncol_x];
  row_x = sp_x + 2 + ncol_x+1;
  ncol_y = sp_y[1];
  colind_y = sp_y+2;
  for (k=0; k<ncol_y; ++k) tmp[k] = colind_y[k];
  for (k=0; k<nnz_x; ++k) {
    y[tmp[row_x[k]]++] = x[k];
  }
}

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

static const casadi_int casadi_s0[27] = {1, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static const casadi_int casadi_s1[27] = {12, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const casadi_int casadi_s2[16] = {12, 1, 0, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const casadi_int casadi_s3[3] = {0, 0, 0};
static const casadi_int casadi_s4[50] = {46, 1, 0, 46, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45};
static const casadi_int casadi_s5[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s6[15] = {0, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

/* astrobee_cost_ext_cost_e_fun_jac_hess:(i0[12],i1[],i2[],i3[46])->(o0,o1[12],o2[12x12,12nz],o3[],o4[0x12]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i, j, k;
  casadi_real *rr, *ss, *tt;
  const casadi_real *cs;
  casadi_real w0, *w1=w+2, *w2=w+14, *w3=w+26, *w4=w+38, *w5=w+50, w6, *w7=w+63;
  /* #0: @0 = 0 */
  w0 = 0.;
  /* #1: @1 = zeros(1x12) */
  casadi_clear(w1, 12);
  /* #2: @2 = input[0][0] */
  casadi_copy(arg[0], 12, w2);
  /* #3: @3 = @2' */
  casadi_copy(w2, 12, w3);
  /* #4: @4 = input[3][0] */
  casadi_copy(arg[3], 12, w4);
  /* #5: @5 = @4' */
  casadi_copy(w4, 12, w5);
  /* #6: @3 = (@3-@5) */
  for (i=0, rr=w3, cs=w5; i<12; ++i) (*rr++) -= (*cs++);
  /* #7: @6 = 100 */
  w6 = 100.;
  /* #8: @5 = input[3][4] */
  casadi_copy(arg[3] ? arg[3]+28 : 0, 12, w5);
  /* #9: @5 = (@6*@5) */
  for (i=0, rr=w5, cs=w5; i<12; ++i) (*rr++)  = (w6*(*cs++));
  /* #10: @7 = @5[:12] */
  for (rr=w7, ss=w5+0; ss!=w5+12; ss+=1) *rr++ = *ss;
  /* #11: @1 = mac(@3,@7,@1) */
  casadi_mtimes(w3, casadi_s0, w7, casadi_s1, w1, casadi_s0, w, 0);
  /* #12: @2 = (@2-@4) */
  for (i=0, rr=w2, cs=w4; i<12; ++i) (*rr++) -= (*cs++);
  /* #13: @0 = mac(@1,@2,@0) */
  for (i=0, rr=(&w0); i<1; ++i) for (j=0; j<1; ++j, ++rr) for (k=0, ss=w1+j, tt=w2+i*12; k<12; ++k) *rr += ss[k*1]**tt++;
  /* #14: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #15: @1 = @1' */
  /* #16: @4 = zeros(1x12) */
  casadi_clear(w4, 12);
  /* #17: @2 = @2' */
  /* #18: @3 = @7' */
  casadi_trans(w7,casadi_s1, w3, casadi_s1, iw);
  /* #19: @4 = mac(@2,@3,@4) */
  casadi_mtimes(w2, casadi_s0, w3, casadi_s1, w4, casadi_s0, w, 0);
  /* #20: @4 = @4' */
  /* #21: @1 = (@1+@4) */
  for (i=0, rr=w1, cs=w4; i<12; ++i) (*rr++) += (*cs++);
  /* #22: output[1][0] = @1 */
  casadi_copy(w1, 12, res[1]);
  /* #23: @1 = zeros(12x12,12nz) */
  casadi_clear(w1, 12);
  /* #24: @4 = zeros(1x12) */
  casadi_clear(w4, 12);
  /* #25: @2 = ones(1x12) */
  casadi_fill(w2, 12, 1.);
  /* #26: @4 = mac(@2,@7,@4) */
  casadi_mtimes(w2, casadi_s0, w7, casadi_s1, w4, casadi_s0, w, 0);
  /* #27: @4 = @4' */
  /* #28: @2 = zeros(1x12) */
  casadi_clear(w2, 12);
  /* #29: @7 = ones(1x12) */
  casadi_fill(w7, 12, 1.);
  /* #30: @2 = mac(@7,@3,@2) */
  casadi_mtimes(w7, casadi_s0, w3, casadi_s1, w2, casadi_s0, w, 0);
  /* #31: @2 = @2' */
  /* #32: @4 = (@4+@2) */
  for (i=0, rr=w4, cs=w2; i<12; ++i) (*rr++) += (*cs++);
  /* #33: (@1[:12] = @4) */
  for (rr=w1+0, ss=w4; rr!=w1+12; rr+=1) *rr = *ss++;
  /* #34: (@1[:12] = @4) */
  for (rr=w1+0, ss=w4; rr!=w1+12; rr+=1) *rr = *ss++;
  /* #35: @4 = @1' */
  casadi_trans(w1,casadi_s1, w4, casadi_s1, iw);
  /* #36: output[2][0] = @4 */
  casadi_copy(w4, 12, res[2]);
  return 0;
}

CASADI_SYMBOL_EXPORT int astrobee_cost_ext_cost_e_fun_jac_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int astrobee_cost_ext_cost_e_fun_jac_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int astrobee_cost_ext_cost_e_fun_jac_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void astrobee_cost_ext_cost_e_fun_jac_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int astrobee_cost_ext_cost_e_fun_jac_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void astrobee_cost_ext_cost_e_fun_jac_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void astrobee_cost_ext_cost_e_fun_jac_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void astrobee_cost_ext_cost_e_fun_jac_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int astrobee_cost_ext_cost_e_fun_jac_hess_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int astrobee_cost_ext_cost_e_fun_jac_hess_n_out(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_real astrobee_cost_ext_cost_e_fun_jac_hess_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* astrobee_cost_ext_cost_e_fun_jac_hess_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* astrobee_cost_ext_cost_e_fun_jac_hess_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    case 4: return "o4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* astrobee_cost_ext_cost_e_fun_jac_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s3;
    case 2: return casadi_s3;
    case 3: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* astrobee_cost_ext_cost_e_fun_jac_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s5;
    case 1: return casadi_s2;
    case 2: return casadi_s1;
    case 3: return casadi_s3;
    case 4: return casadi_s6;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int astrobee_cost_ext_cost_e_fun_jac_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 7;
  if (sz_res) *sz_res = 6;
  if (sz_iw) *sz_iw = 13;
  if (sz_w) *sz_w = 75;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
