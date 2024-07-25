/* This file was automatically generated by CasADi 3.6.5.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_ ## ID
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
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_sq CASADI_PREFIX(sq)

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

casadi_real casadi_sq(casadi_real x) { return x*x;}

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

static const casadi_int casadi_s0[13] = {9, 1, 0, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s4[14] = {12, 2, 0, 5, 9, 6, 7, 8, 10, 11, 6, 7, 8, 10};
static const casadi_int casadi_s5[3] = {2, 0, 0};

/* f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt:(i0[9],i1[3],i2[],i3[])->(o0[2],o1[12x2,9nz],o2[2x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real *rr, *ss;
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, *w17=w+17, w18;
  /* #0: @0 = 52.4282 */
  w0 = 5.2428199999999997e+01;
  /* #1: @1 = input[0][7] */
  w1 = arg[0] ? arg[0][7] : 0;
  /* #2: @1 = (@0*@1) */
  w1  = (w0*w1);
  /* #3: @2 = 3.5 */
  w2 = 3.5000000000000000e+00;
  /* #4: @3 = input[0][3] */
  w3 = arg[0] ? arg[0][3] : 0;
  /* #5: @4 = (@2*@3) */
  w4  = (w2*w3);
  /* #6: @1 = (@1-@4) */
  w1 -= w4;
  /* #7: @4 = 0.8 */
  w4 = 8.0000000000000004e-01;
  /* #8: @5 = (@4*@3) */
  w5  = (w4*w3);
  /* #9: @1 = (@1-@5) */
  w1 -= w5;
  /* #10: @5 = sq(@1) */
  w5 = casadi_sq( w1 );
  /* #11: @6 = 1000000 */
  w6 = 1000000.;
  /* #12: @5 = (@5/@6) */
  w5 /= w6;
  /* #13: @6 = 41.7372 */
  w6 = 4.1737200000000001e+01;
  /* #14: @7 = input[0][8] */
  w7 = arg[0] ? arg[0][8] : 0;
  /* #15: @8 = input[0][4] */
  w8 = arg[0] ? arg[0][4] : 0;
  /* #16: @9 = 0.163 */
  w9 = 1.6300000000000001e-01;
  /* #17: @10 = input[0][5] */
  w10 = arg[0] ? arg[0][5] : 0;
  /* #18: @11 = (@9*@10) */
  w11  = (w9*w10);
  /* #19: @11 = (@8+@11) */
  w11  = (w8+w11);
  /* #20: @12 = 0.001 */
  w12 = 1.0000000000000000e-03;
  /* #21: @12 = (@12+@3) */
  w12 += w3;
  /* #22: @13 = atan2(@11,@12) */
  w13  = atan2(w11,w12);
  /* #23: @7 = (@7-@13) */
  w7 -= w13;
  /* #24: @7 = (@6*@7) */
  w7  = (w6*w7);
  /* #25: @13 = sq(@7) */
  w13 = casadi_sq( w7 );
  /* #26: @14 = 2500 */
  w14 = 2500.;
  /* #27: @13 = (@13/@14) */
  w13 /= w14;
  /* #28: @5 = (@5+@13) */
  w5 += w13;
  /* #29: output[0][0] = @5 */
  if (res[0]) res[0][0] = w5;
  /* #30: @5 = sq(@1) */
  w5 = casadi_sq( w1 );
  /* #31: @13 = 1000000 */
  w13 = 1000000.;
  /* #32: @5 = (@5/@13) */
  w5 /= w13;
  /* #33: @13 = 29.4662 */
  w13 = 2.9466200000000001e+01;
  /* #34: @14 = 0.168 */
  w14 = 1.6800000000000001e-01;
  /* #35: @10 = (@14*@10) */
  w10  = (w14*w10);
  /* #36: @10 = (@10-@8) */
  w10 -= w8;
  /* #37: @8 = 0.001 */
  w8 = 1.0000000000000000e-03;
  /* #38: @8 = (@8+@3) */
  w8 += w3;
  /* #39: @3 = atan2(@10,@8) */
  w3  = atan2(w10,w8);
  /* #40: @3 = (@13*@3) */
  w3  = (w13*w3);
  /* #41: @15 = sq(@3) */
  w15 = casadi_sq( w3 );
  /* #42: @16 = 2500 */
  w16 = 2500.;
  /* #43: @15 = (@15/@16) */
  w15 /= w16;
  /* #44: @5 = (@5+@15) */
  w5 += w15;
  /* #45: output[0][1] = @5 */
  if (res[0]) res[0][1] = w5;
  /* #46: @17 = zeros(12x2,9nz) */
  casadi_clear(w17, 9);
  /* #47: @5 = sq(@11) */
  w5 = casadi_sq( w11 );
  /* #48: @15 = sq(@12) */
  w15 = casadi_sq( w12 );
  /* #49: @5 = (@5+@15) */
  w5 += w15;
  /* #50: @11 = (@11/@5) */
  w11 /= w5;
  /* #51: @7 = (2.*@7) */
  w7 = (2.* w7 );
  /* #52: @15 = 0.0004 */
  w15 = 4.0000000000000002e-04;
  /* #53: @16 = ones(2x1,1nz) */
  w16 = 1.;
  /* #54: {@18, NULL} = vertsplit(@16) */
  w18 = w16;
  /* #55: @15 = (@15*@18) */
  w15 *= w18;
  /* #56: @7 = (@7*@15) */
  w7 *= w15;
  /* #57: @6 = (@6*@7) */
  w6 *= w7;
  /* #58: @11 = (@11*@6) */
  w11 *= w6;
  /* #59: @7 = (2.*@1) */
  w7 = (2.* w1 );
  /* #60: @15 = 1e-06 */
  w15 = 9.9999999999999995e-07;
  /* #61: @15 = (@15*@18) */
  w15 *= w18;
  /* #62: @7 = (@7*@15) */
  w7 *= w15;
  /* #63: @15 = (@4*@7) */
  w15  = (w4*w7);
  /* #64: @11 = (@11-@15) */
  w11 -= w15;
  /* #65: @15 = (@2*@7) */
  w15  = (w2*w7);
  /* #66: @11 = (@11-@15) */
  w11 -= w15;
  /* #67: (@17[0] = @11) */
  for (rr=w17+0, ss=(&w11); rr!=w17+1; rr+=1) *rr = *ss++;
  /* #68: @12 = (@12/@5) */
  w12 /= w5;
  /* #69: @12 = (@12*@6) */
  w12 *= w6;
  /* #70: @5 = (-@12) */
  w5 = (- w12 );
  /* #71: (@17[1] = @5) */
  for (rr=w17+1, ss=(&w5); rr!=w17+2; rr+=1) *rr = *ss++;
  /* #72: @9 = (@9*@12) */
  w9 *= w12;
  /* #73: @9 = (-@9) */
  w9 = (- w9 );
  /* #74: (@17[2] = @9) */
  for (rr=w17+2, ss=(&w9); rr!=w17+3; rr+=1) *rr = *ss++;
  /* #75: @7 = (@0*@7) */
  w7  = (w0*w7);
  /* #76: (@17[3] = @7) */
  for (rr=w17+3, ss=(&w7); rr!=w17+4; rr+=1) *rr = *ss++;
  /* #77: (@17[4] = @6) */
  for (rr=w17+4, ss=(&w6); rr!=w17+5; rr+=1) *rr = *ss++;
  /* #78: @1 = (2.*@1) */
  w1 = (2.* w1 );
  /* #79: @6 = 1e-06 */
  w6 = 9.9999999999999995e-07;
  /* #80: @7 = ones(2x1,1nz) */
  w7 = 1.;
  /* #81: {NULL, @9} = vertsplit(@7) */
  w9 = w7;
  /* #82: @6 = (@6*@9) */
  w6 *= w9;
  /* #83: @1 = (@1*@6) */
  w1 *= w6;
  /* #84: @4 = (@4*@1) */
  w4 *= w1;
  /* #85: @4 = (-@4) */
  w4 = (- w4 );
  /* #86: @6 = sq(@10) */
  w6 = casadi_sq( w10 );
  /* #87: @7 = sq(@8) */
  w7 = casadi_sq( w8 );
  /* #88: @6 = (@6+@7) */
  w6 += w7;
  /* #89: @10 = (@10/@6) */
  w10 /= w6;
  /* #90: @3 = (2.*@3) */
  w3 = (2.* w3 );
  /* #91: @7 = 0.0004 */
  w7 = 4.0000000000000002e-04;
  /* #92: @7 = (@7*@9) */
  w7 *= w9;
  /* #93: @3 = (@3*@7) */
  w3 *= w7;
  /* #94: @13 = (@13*@3) */
  w13 *= w3;
  /* #95: @10 = (@10*@13) */
  w10 *= w13;
  /* #96: @4 = (@4-@10) */
  w4 -= w10;
  /* #97: @2 = (@2*@1) */
  w2 *= w1;
  /* #98: @4 = (@4-@2) */
  w4 -= w2;
  /* #99: (@17[5] = @4) */
  for (rr=w17+5, ss=(&w4); rr!=w17+6; rr+=1) *rr = *ss++;
  /* #100: @8 = (@8/@6) */
  w8 /= w6;
  /* #101: @8 = (@8*@13) */
  w8 *= w13;
  /* #102: @13 = (-@8) */
  w13 = (- w8 );
  /* #103: (@17[6] = @13) */
  for (rr=w17+6, ss=(&w13); rr!=w17+7; rr+=1) *rr = *ss++;
  /* #104: @14 = (@14*@8) */
  w14 *= w8;
  /* #105: (@17[7] = @14) */
  for (rr=w17+7, ss=(&w14); rr!=w17+8; rr+=1) *rr = *ss++;
  /* #106: @0 = (@0*@1) */
  w0 *= w1;
  /* #107: (@17[8] = @0) */
  for (rr=w17+8, ss=(&w0); rr!=w17+9; rr+=1) *rr = *ss++;
  /* #108: output[1][0] = @17 */
  casadi_copy(w17, 9, res[1]);
  return 0;
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_incref(void) {
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s4;
    case 2: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 5;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 27;
  return 0;
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 5*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 27*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
