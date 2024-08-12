/* This file was automatically generated by CasADi 3.6.6.
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
  #define CASADI_PREFIX(ID) f1tenth_bicycle_model_expl_ode_fun_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_sign CASADI_PREFIX(sign)

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

casadi_real casadi_sign(casadi_real x) { return x<0 ? -1 : x>0 ? 1 : x;}

static const casadi_int casadi_s0[13] = {9, 1, 0, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[3] = {0, 0, 0};

/* f1tenth_bicycle_model_expl_ode_fun:(i0[9],i1[3],i2[])->(o0[9]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7, w8, w9;
  /* #0: @0 = input[0][3] */
  w0 = arg[0] ? arg[0][3] : 0;
  /* #1: @1 = input[0][2] */
  w1 = arg[0] ? arg[0][2] : 0;
  /* #2: @2 = cos(@1) */
  w2 = cos( w1 );
  /* #3: @2 = (@0*@2) */
  w2  = (w0*w2);
  /* #4: @3 = input[0][4] */
  w3 = arg[0] ? arg[0][4] : 0;
  /* #5: @4 = sin(@1) */
  w4 = sin( w1 );
  /* #6: @4 = (@3*@4) */
  w4  = (w3*w4);
  /* #7: @2 = (@2-@4) */
  w2 -= w4;
  /* #8: output[0][0] = @2 */
  if (res[0]) res[0][0] = w2;
  /* #9: @2 = sin(@1) */
  w2 = sin( w1 );
  /* #10: @2 = (@0*@2) */
  w2  = (w0*w2);
  /* #11: @1 = cos(@1) */
  w1 = cos( w1 );
  /* #12: @1 = (@3*@1) */
  w1  = (w3*w1);
  /* #13: @2 = (@2+@1) */
  w2 += w1;
  /* #14: output[0][1] = @2 */
  if (res[0]) res[0][1] = w2;
  /* #15: @2 = input[0][5] */
  w2 = arg[0] ? arg[0][5] : 0;
  /* #16: output[0][2] = @2 */
  if (res[0]) res[0][2] = w2;
  /* #17: @1 = 0.344828 */
  w1 = 3.4482758620689657e-01;
  /* #18: @4 = 52.4282 */
  w4 = 5.2428199999999997e+01;
  /* #19: @5 = input[0][7] */
  w5 = arg[0] ? arg[0][7] : 0;
  /* #20: @4 = (@4*@5) */
  w4 *= w5;
  /* #21: @5 = 3 */
  w5 = 3.;
  /* #22: @5 = (@5*@0) */
  w5 *= w0;
  /* #23: @4 = (@4-@5) */
  w4 -= w5;
  /* #24: @5 = 0.8 */
  w5 = 8.0000000000000004e-01;
  /* #25: @6 = sign(@0) */
  w6 = casadi_sign( w0 );
  /* #26: @5 = (@5*@6) */
  w5 *= w6;
  /* #27: @4 = (@4-@5) */
  w4 -= w5;
  /* #28: @5 = input[0][8] */
  w5 = arg[0] ? arg[0][8] : 0;
  /* #29: @6 = cos(@5) */
  w6 = cos( w5 );
  /* #30: @6 = (@4*@6) */
  w6  = (w4*w6);
  /* #31: @6 = (@4+@6) */
  w6  = (w4+w6);
  /* #32: @7 = 41.7372 */
  w7 = 4.1737200000000001e+01;
  /* #33: @8 = 0.163 */
  w8 = 1.6300000000000001e-01;
  /* #34: @8 = (@8*@2) */
  w8 *= w2;
  /* #35: @8 = (@3+@8) */
  w8  = (w3+w8);
  /* #36: @9 = 0.001 */
  w9 = 1.0000000000000000e-03;
  /* #37: @9 = (@9+@0) */
  w9 += w0;
  /* #38: @8 = atan2(@8,@9) */
  w8  = atan2(w8,w9);
  /* #39: @8 = (@5-@8) */
  w8  = (w5-w8);
  /* #40: @7 = (@7*@8) */
  w7 *= w8;
  /* #41: @8 = sin(@5) */
  w8 = sin( w5 );
  /* #42: @8 = (@7*@8) */
  w8  = (w7*w8);
  /* #43: @6 = (@6-@8) */
  w6 -= w8;
  /* #44: @8 = 2.9 */
  w8 = 2.8999999999999999e+00;
  /* #45: @8 = (@8*@3) */
  w8 *= w3;
  /* #46: @8 = (@8*@2) */
  w8 *= w2;
  /* #47: @6 = (@6+@8) */
  w6 += w8;
  /* #48: @1 = (@1*@6) */
  w1 *= w6;
  /* #49: output[0][3] = @1 */
  if (res[0]) res[0][3] = w1;
  /* #50: @1 = 0.344828 */
  w1 = 3.4482758620689657e-01;
  /* #51: @6 = 29.4662 */
  w6 = 2.9466200000000001e+01;
  /* #52: @8 = 0.168 */
  w8 = 1.6800000000000001e-01;
  /* #53: @8 = (@8*@2) */
  w8 *= w2;
  /* #54: @8 = (@8-@3) */
  w8 -= w3;
  /* #55: @3 = 0.001 */
  w3 = 1.0000000000000000e-03;
  /* #56: @3 = (@3+@0) */
  w3 += w0;
  /* #57: @8 = atan2(@8,@3) */
  w8  = atan2(w8,w3);
  /* #58: @6 = (@6*@8) */
  w6 *= w8;
  /* #59: @8 = sin(@5) */
  w8 = sin( w5 );
  /* #60: @8 = (@4*@8) */
  w8  = (w4*w8);
  /* #61: @8 = (@6+@8) */
  w8  = (w6+w8);
  /* #62: @3 = cos(@5) */
  w3 = cos( w5 );
  /* #63: @3 = (@7*@3) */
  w3  = (w7*w3);
  /* #64: @8 = (@8+@3) */
  w8 += w3;
  /* #65: @3 = 2.9 */
  w3 = 2.8999999999999999e+00;
  /* #66: @3 = (@3*@0) */
  w3 *= w0;
  /* #67: @3 = (@3*@2) */
  w3 *= w2;
  /* #68: @8 = (@8-@3) */
  w8 -= w3;
  /* #69: @1 = (@1*@8) */
  w1 *= w8;
  /* #70: output[0][4] = @1 */
  if (res[0]) res[0][4] = w1;
  /* #71: @1 = 12.5628 */
  w1 = 1.2562814070351758e+01;
  /* #72: @8 = 0.163 */
  w8 = 1.6300000000000001e-01;
  /* #73: @8 = (@8*@7) */
  w8 *= w7;
  /* #74: @7 = cos(@5) */
  w7 = cos( w5 );
  /* #75: @8 = (@8*@7) */
  w8 *= w7;
  /* #76: @7 = 0.163 */
  w7 = 1.6300000000000001e-01;
  /* #77: @7 = (@7*@4) */
  w7 *= w4;
  /* #78: @5 = sin(@5) */
  w5 = sin( w5 );
  /* #79: @7 = (@7*@5) */
  w7 *= w5;
  /* #80: @8 = (@8+@7) */
  w8 += w7;
  /* #81: @7 = 0.168 */
  w7 = 1.6800000000000001e-01;
  /* #82: @7 = (@7*@6) */
  w7 *= w6;
  /* #83: @8 = (@8-@7) */
  w8 -= w7;
  /* #84: @1 = (@1*@8) */
  w1 *= w8;
  /* #85: output[0][5] = @1 */
  if (res[0]) res[0][5] = w1;
  /* #86: @1 = input[1][0] */
  w1 = arg[1] ? arg[1][0] : 0;
  /* #87: output[0][6] = @1 */
  if (res[0]) res[0][6] = w1;
  /* #88: @1 = input[1][1] */
  w1 = arg[1] ? arg[1][1] : 0;
  /* #89: output[0][7] = @1 */
  if (res[0]) res[0][7] = w1;
  /* #90: @1 = input[1][2] */
  w1 = arg[1] ? arg[1][2] : 0;
  /* #91: output[0][8] = @1 */
  if (res[0]) res[0][8] = w1;
  return 0;
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_expl_ode_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_expl_ode_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_expl_ode_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_expl_ode_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_expl_ode_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_expl_ode_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_expl_ode_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_expl_ode_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int f1tenth_bicycle_model_expl_ode_fun_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int f1tenth_bicycle_model_expl_ode_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real f1tenth_bicycle_model_expl_ode_fun_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* f1tenth_bicycle_model_expl_ode_fun_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* f1tenth_bicycle_model_expl_ode_fun_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* f1tenth_bicycle_model_expl_ode_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* f1tenth_bicycle_model_expl_ode_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_expl_ode_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 10;
  return 0;
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_expl_ode_fun_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 2*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 10*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
