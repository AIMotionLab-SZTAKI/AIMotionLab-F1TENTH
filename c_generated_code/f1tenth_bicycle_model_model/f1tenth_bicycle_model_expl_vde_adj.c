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
  #define CASADI_PREFIX(ID) f1tenth_bicycle_model_expl_vde_adj_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
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

casadi_real casadi_sq(casadi_real x) { return x*x;}

static const casadi_int casadi_s0[13] = {9, 1, 0, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8};
static const casadi_int casadi_s1[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[13] = {12, 1, 0, 9, 2, 3, 4, 5, 7, 8, 9, 10, 11};

/* f1tenth_bicycle_model_expl_vde_adj:(i0[9],i1[9],i2[3],i3[])->(o0[12x1,9nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1, w2, *w3=w+3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, w19, w20, w21, w22, w23, w24, w25, w26, w27, w28, w29, w30, w31, w32;
  /* #0: @0 = input[0][2] */
  w0 = arg[0] ? arg[0][2] : 0;
  /* #1: @1 = cos(@0) */
  w1 = cos( w0 );
  /* #2: @2 = input[0][3] */
  w2 = arg[0] ? arg[0][3] : 0;
  /* #3: @3 = input[1][0] */
  casadi_copy(arg[1], 9, w3);
  /* #4: {@4, @5, @6, @7, @8, @9, @10, @11, @12} = vertsplit(@3) */
  w4 = w3[0];
  w5 = w3[1];
  w6 = w3[2];
  w7 = w3[3];
  w8 = w3[4];
  w9 = w3[5];
  w10 = w3[6];
  w11 = w3[7];
  w12 = w3[8];
  /* #5: @13 = (@2*@5) */
  w13  = (w2*w5);
  /* #6: @1 = (@1*@13) */
  w1 *= w13;
  /* #7: @13 = sin(@0) */
  w13 = sin( w0 );
  /* #8: @14 = input[0][4] */
  w14 = arg[0] ? arg[0][4] : 0;
  /* #9: @15 = (@14*@5) */
  w15  = (w14*w5);
  /* #10: @13 = (@13*@15) */
  w13 *= w15;
  /* #11: @1 = (@1-@13) */
  w1 -= w13;
  /* #12: @13 = cos(@0) */
  w13 = cos( w0 );
  /* #13: @15 = (@14*@4) */
  w15  = (w14*w4);
  /* #14: @13 = (@13*@15) */
  w13 *= w15;
  /* #15: @1 = (@1-@13) */
  w1 -= w13;
  /* #16: @13 = sin(@0) */
  w13 = sin( w0 );
  /* #17: @15 = (@2*@4) */
  w15  = (w2*w4);
  /* #18: @13 = (@13*@15) */
  w13 *= w15;
  /* #19: @1 = (@1-@13) */
  w1 -= w13;
  /* #20: output[0][0] = @1 */
  if (res[0]) res[0][0] = w1;
  /* #21: @1 = 0.168 */
  w1 = 1.6800000000000001e-01;
  /* #22: @13 = input[0][5] */
  w13 = arg[0] ? arg[0][5] : 0;
  /* #23: @15 = (@1*@13) */
  w15  = (w1*w13);
  /* #24: @15 = (@15-@14) */
  w15 -= w14;
  /* #25: @16 = sq(@15) */
  w16 = casadi_sq( w15 );
  /* #26: @17 = 0.001 */
  w17 = 1.0000000000000000e-03;
  /* #27: @17 = (@17+@2) */
  w17 += w2;
  /* #28: @18 = sq(@17) */
  w18 = casadi_sq( w17 );
  /* #29: @16 = (@16+@18) */
  w16 += w18;
  /* #30: @15 = (@15/@16) */
  w15 /= w16;
  /* #31: @18 = 29.4662 */
  w18 = 2.9466200000000001e+01;
  /* #32: @19 = 0.344828 */
  w19 = 3.4482758620689657e-01;
  /* #33: @19 = (@19*@8) */
  w19 *= w8;
  /* #34: @8 = 0.168 */
  w8 = 1.6800000000000001e-01;
  /* #35: @20 = 12.5628 */
  w20 = 1.2562814070351758e+01;
  /* #36: @20 = (@20*@9) */
  w20 *= w9;
  /* #37: @8 = (@8*@20) */
  w8 *= w20;
  /* #38: @8 = (@19-@8) */
  w8  = (w19-w8);
  /* #39: @18 = (@18*@8) */
  w18 *= w8;
  /* #40: @15 = (@15*@18) */
  w15 *= w18;
  /* #41: @15 = (-@15) */
  w15 = (- w15 );
  /* #42: @8 = 2.9 */
  w8 = 2.8999999999999999e+00;
  /* #43: @9 = (@13*@19) */
  w9  = (w13*w19);
  /* #44: @9 = (@8*@9) */
  w9  = (w8*w9);
  /* #45: @15 = (@15-@9) */
  w15 -= w9;
  /* #46: @9 = 0.163 */
  w9 = 1.6300000000000001e-01;
  /* #47: @21 = (@9*@13) */
  w21  = (w9*w13);
  /* #48: @21 = (@14+@21) */
  w21  = (w14+w21);
  /* #49: @22 = sq(@21) */
  w22 = casadi_sq( w21 );
  /* #50: @23 = 0.001 */
  w23 = 1.0000000000000000e-03;
  /* #51: @23 = (@23+@2) */
  w23 += w2;
  /* #52: @24 = sq(@23) */
  w24 = casadi_sq( w23 );
  /* #53: @22 = (@22+@24) */
  w22 += w24;
  /* #54: @24 = (@21/@22) */
  w24  = (w21/w22);
  /* #55: @25 = 41.7372 */
  w25 = 4.1737200000000001e+01;
  /* #56: @26 = 0.163 */
  w26 = 1.6300000000000001e-01;
  /* #57: @27 = input[0][8] */
  w27 = arg[0] ? arg[0][8] : 0;
  /* #58: @28 = cos(@27) */
  w28 = cos( w27 );
  /* #59: @28 = (@28*@20) */
  w28 *= w20;
  /* #60: @28 = (@26*@28) */
  w28  = (w26*w28);
  /* #61: @29 = cos(@27) */
  w29 = cos( w27 );
  /* #62: @29 = (@29*@19) */
  w29 *= w19;
  /* #63: @28 = (@28+@29) */
  w28 += w29;
  /* #64: @29 = sin(@27) */
  w29 = sin( w27 );
  /* #65: @30 = 0.344828 */
  w30 = 3.4482758620689657e-01;
  /* #66: @30 = (@30*@7) */
  w30 *= w7;
  /* #67: @29 = (@29*@30) */
  w29 *= w30;
  /* #68: @28 = (@28-@29) */
  w28 -= w29;
  /* #69: @28 = (@25*@28) */
  w28  = (w25*w28);
  /* #70: @24 = (@24*@28) */
  w24 *= w28;
  /* #71: @15 = (@15+@24) */
  w15 += w24;
  /* #72: @24 = 0.8 */
  w24 = 8.0000000000000004e-01;
  /* #73: @29 = 0.163 */
  w29 = 1.6300000000000001e-01;
  /* #74: @7 = sin(@27) */
  w7 = sin( w27 );
  /* #75: @7 = (@7*@20) */
  w7 *= w20;
  /* #76: @7 = (@29*@7) */
  w7  = (w29*w7);
  /* #77: @31 = sin(@27) */
  w31 = sin( w27 );
  /* #78: @31 = (@31*@19) */
  w31 *= w19;
  /* #79: @7 = (@7+@31) */
  w7 += w31;
  /* #80: @7 = (@7+@30) */
  w7 += w30;
  /* #81: @31 = cos(@27) */
  w31 = cos( w27 );
  /* #82: @31 = (@31*@30) */
  w31 *= w30;
  /* #83: @7 = (@7+@31) */
  w7 += w31;
  /* #84: @31 = (@24*@7) */
  w31  = (w24*w7);
  /* #85: @15 = (@15-@31) */
  w15 -= w31;
  /* #86: @31 = 3 */
  w31 = 3.;
  /* #87: @32 = (@31*@7) */
  w32  = (w31*w7);
  /* #88: @15 = (@15-@32) */
  w15 -= w32;
  /* #89: @32 = sin(@0) */
  w32 = sin( w0 );
  /* #90: @32 = (@32*@5) */
  w32 *= w5;
  /* #91: @15 = (@15+@32) */
  w15 += w32;
  /* #92: @32 = cos(@0) */
  w32 = cos( w0 );
  /* #93: @32 = (@32*@4) */
  w32 *= w4;
  /* #94: @15 = (@15+@32) */
  w15 += w32;
  /* #95: output[0][1] = @15 */
  if (res[0]) res[0][1] = w15;
  /* #96: @15 = 2.9 */
  w15 = 2.8999999999999999e+00;
  /* #97: @13 = (@13*@30) */
  w13 *= w30;
  /* #98: @13 = (@15*@13) */
  w13  = (w15*w13);
  /* #99: @17 = (@17/@16) */
  w17 /= w16;
  /* #100: @17 = (@17*@18) */
  w17 *= w18;
  /* #101: @13 = (@13-@17) */
  w13 -= w17;
  /* #102: @22 = (@23/@22) */
  w22  = (w23/w22);
  /* #103: @22 = (@22*@28) */
  w22 *= w28;
  /* #104: @13 = (@13-@22) */
  w13 -= w22;
  /* #105: @18 = cos(@0) */
  w18 = cos( w0 );
  /* #106: @18 = (@18*@5) */
  w18 *= w5;
  /* #107: @13 = (@13+@18) */
  w13 += w18;
  /* #108: @0 = sin(@0) */
  w0 = sin( w0 );
  /* #109: @0 = (@0*@4) */
  w0 *= w4;
  /* #110: @13 = (@13-@0) */
  w13 -= w0;
  /* #111: output[0][2] = @13 */
  if (res[0]) res[0][2] = w13;
  /* #112: @1 = (@1*@17) */
  w1 *= w17;
  /* #113: @8 = (@8*@2) */
  w8 *= w2;
  /* #114: @8 = (@8*@19) */
  w8 *= w19;
  /* #115: @1 = (@1-@8) */
  w1 -= w8;
  /* #116: @15 = (@15*@14) */
  w15 *= w14;
  /* #117: @15 = (@15*@30) */
  w15 *= w30;
  /* #118: @1 = (@1+@15) */
  w1 += w15;
  /* #119: @9 = (@9*@22) */
  w9 *= w22;
  /* #120: @1 = (@1-@9) */
  w1 -= w9;
  /* #121: @1 = (@1+@6) */
  w1 += w6;
  /* #122: output[0][3] = @1 */
  if (res[0]) res[0][3] = w1;
  /* #123: @1 = 52.4282 */
  w1 = 5.2428199999999997e+01;
  /* #124: @7 = (@1*@7) */
  w7  = (w1*w7);
  /* #125: output[0][4] = @7 */
  if (res[0]) res[0][4] = w7;
  /* #126: @7 = cos(@27) */
  w7 = cos( w27 );
  /* #127: @6 = input[0][7] */
  w6 = arg[0] ? arg[0][7] : 0;
  /* #128: @1 = (@1*@6) */
  w1 *= w6;
  /* #129: @31 = (@31*@2) */
  w31 *= w2;
  /* #130: @1 = (@1-@31) */
  w1 -= w31;
  /* #131: @24 = (@24*@2) */
  w24 *= w2;
  /* #132: @1 = (@1-@24) */
  w1 -= w24;
  /* #133: @29 = (@29*@1) */
  w29 *= w1;
  /* #134: @29 = (@29*@20) */
  w29 *= w20;
  /* #135: @7 = (@7*@29) */
  w7 *= w29;
  /* #136: @29 = sin(@27) */
  w29 = sin( w27 );
  /* #137: @21 = atan2(@21,@23) */
  w21  = atan2(w21,w23);
  /* #138: @21 = (@27-@21) */
  w21  = (w27-w21);
  /* #139: @25 = (@25*@21) */
  w25 *= w21;
  /* #140: @26 = (@26*@25) */
  w26 *= w25;
  /* #141: @26 = (@26*@20) */
  w26 *= w20;
  /* #142: @29 = (@29*@26) */
  w29 *= w26;
  /* #143: @7 = (@7-@29) */
  w7 -= w29;
  /* #144: @29 = sin(@27) */
  w29 = sin( w27 );
  /* #145: @26 = (@25*@19) */
  w26  = (w25*w19);
  /* #146: @29 = (@29*@26) */
  w29 *= w26;
  /* #147: @7 = (@7-@29) */
  w7 -= w29;
  /* #148: @29 = cos(@27) */
  w29 = cos( w27 );
  /* #149: @19 = (@1*@19) */
  w19  = (w1*w19);
  /* #150: @29 = (@29*@19) */
  w29 *= w19;
  /* #151: @7 = (@7+@29) */
  w7 += w29;
  /* #152: @29 = cos(@27) */
  w29 = cos( w27 );
  /* #153: @25 = (@25*@30) */
  w25 *= w30;
  /* #154: @29 = (@29*@25) */
  w29 *= w25;
  /* #155: @7 = (@7-@29) */
  w7 -= w29;
  /* #156: @7 = (@7+@28) */
  w7 += w28;
  /* #157: @27 = sin(@27) */
  w27 = sin( w27 );
  /* #158: @1 = (@1*@30) */
  w1 *= w30;
  /* #159: @27 = (@27*@1) */
  w27 *= w1;
  /* #160: @7 = (@7-@27) */
  w7 -= w27;
  /* #161: output[0][5] = @7 */
  if (res[0]) res[0][5] = w7;
  /* #162: output[0][6] = @10 */
  if (res[0]) res[0][6] = w10;
  /* #163: output[0][7] = @11 */
  if (res[0]) res[0][7] = w11;
  /* #164: output[0][8] = @12 */
  if (res[0]) res[0][8] = w12;
  return 0;
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_expl_vde_adj(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_expl_vde_adj_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_expl_vde_adj_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_expl_vde_adj_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_expl_vde_adj_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_expl_vde_adj_release(int mem) {
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_expl_vde_adj_incref(void) {
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_expl_vde_adj_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int f1tenth_bicycle_model_expl_vde_adj_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int f1tenth_bicycle_model_expl_vde_adj_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real f1tenth_bicycle_model_expl_vde_adj_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* f1tenth_bicycle_model_expl_vde_adj_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* f1tenth_bicycle_model_expl_vde_adj_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* f1tenth_bicycle_model_expl_vde_adj_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* f1tenth_bicycle_model_expl_vde_adj_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_expl_vde_adj_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 10;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 41;
  return 0;
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_expl_vde_adj_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 10*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 41*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
