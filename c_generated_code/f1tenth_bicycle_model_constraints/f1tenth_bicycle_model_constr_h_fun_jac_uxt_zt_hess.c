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
  #define CASADI_PREFIX(ID) f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_ ## ID
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
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s10 CASADI_PREFIX(s10)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_s8 CASADI_PREFIX(s8)
#define casadi_s9 CASADI_PREFIX(s9)
#define casadi_sign CASADI_PREFIX(sign)
#define casadi_sq CASADI_PREFIX(sq)
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

casadi_real casadi_sign(casadi_real x) { return x<0 ? -1 : x>0 ? 1 : x;}

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

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
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

static const casadi_int casadi_s0[5] = {0, 5, 9, 13, 15};
static const casadi_int casadi_s1[4] = {1, 6, 10, 16};
static const casadi_int casadi_s2[4] = {2, 7, 11, 17};
static const casadi_int casadi_s3[4] = {4, 8, 12, 18};
static const casadi_int casadi_s4[34] = {12, 12, 0, 0, 0, 0, 0, 0, 0, 5, 9, 13, 13, 15, 19, 6, 7, 8, 10, 11, 6, 7, 8, 11, 6, 7, 8, 11, 6, 10, 6, 7, 8, 11};
static const casadi_int casadi_s5[13] = {9, 1, 0, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8};
static const casadi_int casadi_s6[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s7[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s8[3] = {0, 0, 0};
static const casadi_int casadi_s9[14] = {12, 2, 0, 5, 9, 6, 7, 8, 10, 11, 6, 7, 8, 10};
static const casadi_int casadi_s10[3] = {2, 0, 0};

/* f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess:(i0[9],i1[3],i2[2],i3[],i4[])->(o0[2],o1[12x2,9nz],o2[12x12,19nz],o3[2x0],o4[]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real *rr, *ss;
  const casadi_int *cii;
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, *w16=w+16, w17, w18, w19, *w20=w+28, *w27=w+47, *w28=w+49, w29, w30, w31, w32, w33, w34, w35, w36, w37, w38, w39, w40, w41, w42, w43, w44, *w46=w+73, *w47=w+78, *w49=w+83, *w50=w+87, *w53=w+91, *w54=w+93;
  /* #0: @0 = 45 */
  w0 = 45.;
  /* #1: @1 = input[0][7] */
  w1 = arg[0] ? arg[0][7] : 0;
  /* #2: @1 = (@0*@1) */
  w1  = (w0*w1);
  /* #3: @2 = 3 */
  w2 = 3.;
  /* #4: @3 = input[0][3] */
  w3 = arg[0] ? arg[0][3] : 0;
  /* #5: @4 = (@2*@3) */
  w4  = (w2*w3);
  /* #6: @1 = (@1-@4) */
  w1 -= w4;
  /* #7: @4 = 0.8 */
  w4 = 8.0000000000000004e-01;
  /* #8: @5 = sign(@3) */
  w5 = casadi_sign( w3 );
  /* #9: @4 = (@4*@5) */
  w4 *= w5;
  /* #10: @1 = (@1-@4) */
  w1 -= w4;
  /* #11: @4 = sq(@1) */
  w4 = casadi_sq( w1 );
  /* #12: @5 = 1000000 */
  w5 = 1000000.;
  /* #13: @4 = (@4/@5) */
  w4 /= w5;
  /* #14: @5 = 41.7372 */
  w5 = 4.1737200000000001e+01;
  /* #15: @6 = input[0][8] */
  w6 = arg[0] ? arg[0][8] : 0;
  /* #16: @7 = input[0][4] */
  w7 = arg[0] ? arg[0][4] : 0;
  /* #17: @8 = 0.163 */
  w8 = 1.6300000000000001e-01;
  /* #18: @9 = input[0][5] */
  w9 = arg[0] ? arg[0][5] : 0;
  /* #19: @10 = (@8*@9) */
  w10  = (w8*w9);
  /* #20: @10 = (@7+@10) */
  w10  = (w7+w10);
  /* #21: @11 = 0.001 */
  w11 = 1.0000000000000000e-03;
  /* #22: @11 = (@11+@3) */
  w11 += w3;
  /* #23: @12 = atan2(@10,@11) */
  w12  = atan2(w10,w11);
  /* #24: @6 = (@6-@12) */
  w6 -= w12;
  /* #25: @6 = (@5*@6) */
  w6  = (w5*w6);
  /* #26: @12 = sq(@6) */
  w12 = casadi_sq( w6 );
  /* #27: @13 = 10000 */
  w13 = 10000.;
  /* #28: @12 = (@12/@13) */
  w12 /= w13;
  /* #29: @4 = (@4+@12) */
  w4 += w12;
  /* #30: output[0][0] = @4 */
  if (res[0]) res[0][0] = w4;
  /* #31: @4 = sq(@1) */
  w4 = casadi_sq( w1 );
  /* #32: @12 = 1000000 */
  w12 = 1000000.;
  /* #33: @4 = (@4/@12) */
  w4 /= w12;
  /* #34: @12 = 29.4662 */
  w12 = 2.9466200000000001e+01;
  /* #35: @13 = 0.168 */
  w13 = 1.6800000000000001e-01;
  /* #36: @9 = (@13*@9) */
  w9  = (w13*w9);
  /* #37: @9 = (@9-@7) */
  w9 -= w7;
  /* #38: @7 = 0.001 */
  w7 = 1.0000000000000000e-03;
  /* #39: @7 = (@7+@3) */
  w7 += w3;
  /* #40: @3 = atan2(@9,@7) */
  w3  = atan2(w9,w7);
  /* #41: @3 = (@12*@3) */
  w3  = (w12*w3);
  /* #42: @14 = sq(@3) */
  w14 = casadi_sq( w3 );
  /* #43: @15 = 10000 */
  w15 = 10000.;
  /* #44: @14 = (@14/@15) */
  w14 /= w15;
  /* #45: @4 = (@4+@14) */
  w4 += w14;
  /* #46: output[0][1] = @4 */
  if (res[0]) res[0][1] = w4;
  /* #47: @16 = zeros(12x2,9nz) */
  casadi_clear(w16, 9);
  /* #48: @4 = sq(@10) */
  w4 = casadi_sq( w10 );
  /* #49: @14 = sq(@11) */
  w14 = casadi_sq( w11 );
  /* #50: @4 = (@4+@14) */
  w4 += w14;
  /* #51: @14 = (@10/@4) */
  w14  = (w10/w4);
  /* #52: @15 = (2.*@6) */
  w15 = (2.* w6 );
  /* #53: @17 = 0.0001 */
  w17 = 1.0000000000000000e-04;
  /* #54: @18 = ones(2x1,1nz) */
  w18 = 1.;
  /* #55: {@19, NULL} = vertsplit(@18) */
  w19 = w18;
  /* #56: @17 = (@17*@19) */
  w17 *= w19;
  /* #57: @15 = (@15*@17) */
  w15 *= w17;
  /* #58: @15 = (@5*@15) */
  w15  = (w5*w15);
  /* #59: @14 = (@14*@15) */
  w14 *= w15;
  /* #60: @17 = (2.*@1) */
  w17 = (2.* w1 );
  /* #61: @18 = 1e-06 */
  w18 = 9.9999999999999995e-07;
  /* #62: @18 = (@18*@19) */
  w18 *= w19;
  /* #63: @17 = (@17*@18) */
  w17 *= w18;
  /* #64: @18 = (@2*@17) */
  w18  = (w2*w17);
  /* #65: @14 = (@14-@18) */
  w14 -= w18;
  /* #66: (@16[0] = @14) */
  for (rr=w16+0, ss=(&w14); rr!=w16+1; rr+=1) *rr = *ss++;
  /* #67: @4 = (@11/@4) */
  w4  = (w11/w4);
  /* #68: @4 = (@4*@15) */
  w4 *= w15;
  /* #69: @14 = (-@4) */
  w14 = (- w4 );
  /* #70: (@16[1] = @14) */
  for (rr=w16+1, ss=(&w14); rr!=w16+2; rr+=1) *rr = *ss++;
  /* #71: @4 = (@8*@4) */
  w4  = (w8*w4);
  /* #72: @4 = (-@4) */
  w4 = (- w4 );
  /* #73: (@16[2] = @4) */
  for (rr=w16+2, ss=(&w4); rr!=w16+3; rr+=1) *rr = *ss++;
  /* #74: @17 = (@0*@17) */
  w17  = (w0*w17);
  /* #75: (@16[3] = @17) */
  for (rr=w16+3, ss=(&w17); rr!=w16+4; rr+=1) *rr = *ss++;
  /* #76: (@16[4] = @15) */
  for (rr=w16+4, ss=(&w15); rr!=w16+5; rr+=1) *rr = *ss++;
  /* #77: @1 = (2.*@1) */
  w1 = (2.* w1 );
  /* #78: @15 = 1e-06 */
  w15 = 9.9999999999999995e-07;
  /* #79: @17 = ones(2x1,1nz) */
  w17 = 1.;
  /* #80: {NULL, @4} = vertsplit(@17) */
  w4 = w17;
  /* #81: @15 = (@15*@4) */
  w15 *= w4;
  /* #82: @1 = (@1*@15) */
  w1 *= w15;
  /* #83: @15 = (@2*@1) */
  w15  = (w2*w1);
  /* #84: @15 = (-@15) */
  w15 = (- w15 );
  /* #85: @17 = sq(@9) */
  w17 = casadi_sq( w9 );
  /* #86: @14 = sq(@7) */
  w14 = casadi_sq( w7 );
  /* #87: @17 = (@17+@14) */
  w17 += w14;
  /* #88: @14 = (@9/@17) */
  w14  = (w9/w17);
  /* #89: @18 = (2.*@3) */
  w18 = (2.* w3 );
  /* #90: @19 = 0.0001 */
  w19 = 1.0000000000000000e-04;
  /* #91: @19 = (@19*@4) */
  w19 *= w4;
  /* #92: @18 = (@18*@19) */
  w18 *= w19;
  /* #93: @18 = (@12*@18) */
  w18  = (w12*w18);
  /* #94: @14 = (@14*@18) */
  w14 *= w18;
  /* #95: @15 = (@15-@14) */
  w15 -= w14;
  /* #96: (@16[5] = @15) */
  for (rr=w16+5, ss=(&w15); rr!=w16+6; rr+=1) *rr = *ss++;
  /* #97: @17 = (@7/@17) */
  w17  = (w7/w17);
  /* #98: @17 = (@17*@18) */
  w17 *= w18;
  /* #99: @18 = (-@17) */
  w18 = (- w17 );
  /* #100: (@16[6] = @18) */
  for (rr=w16+6, ss=(&w18); rr!=w16+7; rr+=1) *rr = *ss++;
  /* #101: @17 = (@13*@17) */
  w17  = (w13*w17);
  /* #102: (@16[7] = @17) */
  for (rr=w16+7, ss=(&w17); rr!=w16+8; rr+=1) *rr = *ss++;
  /* #103: @1 = (@0*@1) */
  w1  = (w0*w1);
  /* #104: (@16[8] = @1) */
  for (rr=w16+8, ss=(&w1); rr!=w16+9; rr+=1) *rr = *ss++;
  /* #105: output[1][0] = @16 */
  casadi_copy(w16, 9, res[1]);
  /* #106: @20 = zeros(12x12,19nz) */
  casadi_clear(w20, 19);
  /* #107: @21 = 00 */
  /* #108: @22 = 00 */
  /* #109: @23 = 00 */
  /* #110: @24 = 00 */
  /* #111: @25 = 00 */
  /* #112: @26 = 00 */
  /* #113: @1 = sq(@10) */
  w1 = casadi_sq( w10 );
  /* #114: @17 = sq(@11) */
  w17 = casadi_sq( w11 );
  /* #115: @1 = (@1+@17) */
  w1 += w17;
  /* #116: @17 = (@10/@1) */
  w17  = (w10/w1);
  /* #117: @18 = 0.0001 */
  w18 = 1.0000000000000000e-04;
  /* #118: @27 = input[2][0] */
  casadi_copy(arg[2], 2, w27);
  /* #119: {@15, @14} = vertsplit(@27) */
  w15 = w27[0];
  w14 = w27[1];
  /* #120: @18 = (@18*@15) */
  w18 *= w15;
  /* #121: @19 = sq(@10) */
  w19 = casadi_sq( w10 );
  /* #122: @4 = sq(@11) */
  w4 = casadi_sq( w11 );
  /* #123: @19 = (@19+@4) */
  w19 += w4;
  /* #124: @4 = (@10/@19) */
  w4  = (w10/w19);
  /* #125: @28 = ones(12x1,8nz) */
  casadi_fill(w28, 8, 1.);
  /* #126: {NULL, NULL, NULL, NULL, NULL, NULL, @29, NULL, NULL, NULL, NULL, NULL} = vertsplit(@28) */
  w29 = w28[6];
  /* #127: @4 = (@4*@29) */
  w4 *= w29;
  /* #128: @4 = (@5*@4) */
  w4  = (w5*w4);
  /* #129: @4 = (2.*@4) */
  w4 = (2.* w4 );
  /* #130: @4 = (@18*@4) */
  w4  = (w18*w4);
  /* #131: @4 = (@5*@4) */
  w4  = (w5*w4);
  /* #132: @30 = (@17*@4) */
  w30  = (w17*w4);
  /* #133: @6 = (2.*@6) */
  w6 = (2.* w6 );
  /* #134: @6 = (@6*@18) */
  w6 *= w18;
  /* #135: @6 = (@5*@6) */
  w6  = (w5*w6);
  /* #136: @31 = (@17/@1) */
  w31  = (w17/w1);
  /* #137: @32 = (2.*@11) */
  w32 = (2.* w11 );
  /* #138: @32 = (@32*@29) */
  w32 *= w29;
  /* #139: @33 = (@31*@32) */
  w33  = (w31*w32);
  /* #140: @33 = (@6*@33) */
  w33  = (w6*w33);
  /* #141: @30 = (@30-@33) */
  w30 -= w33;
  /* #142: @33 = sq(@9) */
  w33 = casadi_sq( w9 );
  /* #143: @34 = sq(@7) */
  w34 = casadi_sq( w7 );
  /* #144: @33 = (@33+@34) */
  w33 += w34;
  /* #145: @34 = (@9/@33) */
  w34  = (w9/w33);
  /* #146: @35 = 0.0001 */
  w35 = 1.0000000000000000e-04;
  /* #147: @35 = (@35*@14) */
  w35 *= w14;
  /* #148: @36 = sq(@9) */
  w36 = casadi_sq( w9 );
  /* #149: @37 = sq(@7) */
  w37 = casadi_sq( w7 );
  /* #150: @36 = (@36+@37) */
  w36 += w37;
  /* #151: @37 = (@9/@36) */
  w37  = (w9/w36);
  /* #152: @37 = (@37*@29) */
  w37 *= w29;
  /* #153: @37 = (@12*@37) */
  w37  = (w12*w37);
  /* #154: @37 = (-@37) */
  w37 = (- w37 );
  /* #155: @37 = (2.*@37) */
  w37 = (2.* w37 );
  /* #156: @37 = (@35*@37) */
  w37  = (w35*w37);
  /* #157: @37 = (@12*@37) */
  w37  = (w12*w37);
  /* #158: @38 = (@34*@37) */
  w38  = (w34*w37);
  /* #159: @3 = (2.*@3) */
  w3 = (2.* w3 );
  /* #160: @3 = (@3*@35) */
  w3 *= w35;
  /* #161: @3 = (@12*@3) */
  w3  = (w12*w3);
  /* #162: @39 = (@34/@33) */
  w39  = (w34/w33);
  /* #163: @40 = (2.*@7) */
  w40 = (2.* w7 );
  /* #164: @40 = (@40*@29) */
  w40 *= w29;
  /* #165: @41 = (@39*@40) */
  w41  = (w39*w40);
  /* #166: @41 = (@3*@41) */
  w41  = (w3*w41);
  /* #167: @38 = (@38-@41) */
  w38 -= w41;
  /* #168: @30 = (@30-@38) */
  w30 -= w38;
  /* #169: @38 = 1e-06 */
  w38 = 9.9999999999999995e-07;
  /* #170: @38 = (@38*@14) */
  w38 *= w14;
  /* #171: @14 = (@2*@29) */
  w14  = (w2*w29);
  /* #172: @14 = (-@14) */
  w14 = (- w14 );
  /* #173: @41 = (2.*@14) */
  w41 = (2.* w14 );
  /* #174: @41 = (@38*@41) */
  w41  = (w38*w41);
  /* #175: @42 = 1e-06 */
  w42 = 9.9999999999999995e-07;
  /* #176: @42 = (@42*@15) */
  w42 *= w15;
  /* #177: @14 = (2.*@14) */
  w14 = (2.* w14 );
  /* #178: @14 = (@42*@14) */
  w14  = (w42*w14);
  /* #179: @41 = (@41+@14) */
  w41 += w14;
  /* #180: @14 = (@2*@41) */
  w14  = (w2*w41);
  /* #181: @30 = (@30-@14) */
  w30 -= w14;
  /* #182: @14 = (@29/@33) */
  w14  = (w29/w33);
  /* #183: @15 = (@7/@33) */
  w15  = (w7/w33);
  /* #184: @43 = (@15/@33) */
  w43  = (w15/w33);
  /* #185: @40 = (@43*@40) */
  w40  = (w43*w40);
  /* #186: @14 = (@14-@40) */
  w14 -= w40;
  /* #187: @14 = (@3*@14) */
  w14  = (w3*w14);
  /* #188: @37 = (@15*@37) */
  w37  = (w15*w37);
  /* #189: @14 = (@14+@37) */
  w14 += w37;
  /* #190: @37 = (-@14) */
  w37 = (- w14 );
  /* #191: @29 = (@29/@1) */
  w29 /= w1;
  /* #192: @40 = (@11/@1) */
  w40  = (w11/w1);
  /* #193: @44 = (@40/@1) */
  w44  = (w40/w1);
  /* #194: @32 = (@44*@32) */
  w32  = (w44*w32);
  /* #195: @29 = (@29-@32) */
  w29 -= w32;
  /* #196: @29 = (@6*@29) */
  w29  = (w6*w29);
  /* #197: @32 = (@40*@4) */
  w32  = (w40*w4);
  /* #198: @29 = (@29+@32) */
  w29 += w32;
  /* #199: @37 = (@37-@29) */
  w37 -= w29;
  /* #200: @14 = (@13*@14) */
  w14  = (w13*w14);
  /* #201: @29 = (@8*@29) */
  w29  = (w8*w29);
  /* #202: @14 = (@14-@29) */
  w14 -= w29;
  /* #203: @45 = 00 */
  /* #204: @41 = (@0*@41) */
  w41  = (w0*w41);
  /* #205: @46 = vertcat(@21, @22, @23, @24, @25, @26, @30, @37, @14, @45, @41, @4) */
  rr=w46;
  *rr++ = w30;
  *rr++ = w37;
  *rr++ = w14;
  *rr++ = w41;
  *rr++ = w4;
  /* #206: @47 = @46[:5] */
  for (rr=w47, ss=w46+0; ss!=w46+5; ss+=1) *rr++ = *ss;
  /* #207: (@20[0, 5, 9, 13, 15] = @47) */
  for (cii=casadi_s0, rr=w20, ss=w47; cii!=casadi_s0+5; ++cii, ++ss) rr[*cii] = *ss;
  /* #208: @30 = ones(12x1,1nz) */
  w30 = 1.;
  /* #209: {NULL, NULL, NULL, NULL, NULL, NULL, NULL, @37, NULL, NULL, NULL, NULL} = vertsplit(@30) */
  w37 = w30;
  /* #210: @30 = (@37/@1) */
  w30  = (w37/w1);
  /* #211: @10 = (2.*@10) */
  w10 = (2.* w10 );
  /* #212: @14 = (@10*@37) */
  w14  = (w10*w37);
  /* #213: @41 = (@31*@14) */
  w41  = (w31*w14);
  /* #214: @30 = (@30-@41) */
  w30 -= w41;
  /* #215: @30 = (@6*@30) */
  w30  = (w6*w30);
  /* #216: @11 = (@11/@19) */
  w11 /= w19;
  /* #217: @19 = (@11*@37) */
  w19  = (w11*w37);
  /* #218: @19 = (@5*@19) */
  w19  = (w5*w19);
  /* #219: @19 = (-@19) */
  w19 = (- w19 );
  /* #220: @19 = (2.*@19) */
  w19 = (2.* w19 );
  /* #221: @19 = (@18*@19) */
  w19  = (w18*w19);
  /* #222: @19 = (@5*@19) */
  w19  = (w5*w19);
  /* #223: @41 = (@17*@19) */
  w41  = (w17*w19);
  /* #224: @30 = (@30+@41) */
  w30 += w41;
  /* #225: @9 = (2.*@9) */
  w9 = (2.* w9 );
  /* #226: @41 = (@9*@37) */
  w41  = (w9*w37);
  /* #227: @4 = (@39*@41) */
  w4  = (w39*w41);
  /* #228: @29 = (@37/@33) */
  w29  = (w37/w33);
  /* #229: @4 = (@4-@29) */
  w4 -= w29;
  /* #230: @4 = (@3*@4) */
  w4  = (w3*w4);
  /* #231: @7 = (@7/@36) */
  w7 /= w36;
  /* #232: @37 = (@7*@37) */
  w37  = (w7*w37);
  /* #233: @37 = (@12*@37) */
  w37  = (w12*w37);
  /* #234: @37 = (-@37) */
  w37 = (- w37 );
  /* #235: @37 = (2.*@37) */
  w37 = (2.* w37 );
  /* #236: @37 = (@35*@37) */
  w37  = (w35*w37);
  /* #237: @37 = (@12*@37) */
  w37  = (w12*w37);
  /* #238: @36 = (@34*@37) */
  w36  = (w34*w37);
  /* #239: @4 = (@4+@36) */
  w4 += w36;
  /* #240: @30 = (@30-@4) */
  w30 -= w4;
  /* #241: @41 = (@43*@41) */
  w41  = (w43*w41);
  /* #242: @41 = (@3*@41) */
  w41  = (w3*w41);
  /* #243: @37 = (@15*@37) */
  w37  = (w15*w37);
  /* #244: @41 = (@41+@37) */
  w41 += w37;
  /* #245: @37 = (-@41) */
  w37 = (- w41 );
  /* #246: @4 = (@40*@19) */
  w4  = (w40*w19);
  /* #247: @14 = (@44*@14) */
  w14  = (w44*w14);
  /* #248: @14 = (@6*@14) */
  w14  = (w6*w14);
  /* #249: @4 = (@4-@14) */
  w4 -= w14;
  /* #250: @37 = (@37-@4) */
  w37 -= w4;
  /* #251: @41 = (@13*@41) */
  w41  = (w13*w41);
  /* #252: @4 = (@8*@4) */
  w4  = (w8*w4);
  /* #253: @41 = (@41-@4) */
  w41 -= w4;
  /* #254: @48 = 00 */
  /* #255: @49 = vertcat(@21, @22, @23, @24, @25, @26, @30, @37, @41, @45, @48, @19) */
  rr=w49;
  *rr++ = w30;
  *rr++ = w37;
  *rr++ = w41;
  *rr++ = w19;
  /* #256: @50 = @49[:4] */
  for (rr=w50, ss=w49+0; ss!=w49+4; ss+=1) *rr++ = *ss;
  /* #257: (@20[1, 6, 10, 16] = @50) */
  for (cii=casadi_s1, rr=w20, ss=w50; cii!=casadi_s1+4; ++cii, ++ss) rr[*cii] = *ss;
  /* #258: @30 = ones(12x1,1nz) */
  w30 = 1.;
  /* #259: {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, @37, NULL, NULL, NULL} = vertsplit(@30) */
  w37 = w30;
  /* #260: @30 = (@8*@37) */
  w30  = (w8*w37);
  /* #261: @1 = (@30/@1) */
  w1  = (w30/w1);
  /* #262: @10 = (@10*@30) */
  w10 *= w30;
  /* #263: @31 = (@31*@10) */
  w31 *= w10;
  /* #264: @1 = (@1-@31) */
  w1 -= w31;
  /* #265: @1 = (@6*@1) */
  w1  = (w6*w1);
  /* #266: @11 = (@11*@30) */
  w11 *= w30;
  /* #267: @11 = (@5*@11) */
  w11  = (w5*w11);
  /* #268: @11 = (-@11) */
  w11 = (- w11 );
  /* #269: @11 = (2.*@11) */
  w11 = (2.* w11 );
  /* #270: @11 = (@18*@11) */
  w11  = (w18*w11);
  /* #271: @11 = (@5*@11) */
  w11  = (w5*w11);
  /* #272: @30 = (@17*@11) */
  w30  = (w17*w11);
  /* #273: @1 = (@1+@30) */
  w1 += w30;
  /* #274: @37 = (@13*@37) */
  w37  = (w13*w37);
  /* #275: @33 = (@37/@33) */
  w33  = (w37/w33);
  /* #276: @9 = (@9*@37) */
  w9 *= w37;
  /* #277: @39 = (@39*@9) */
  w39 *= w9;
  /* #278: @33 = (@33-@39) */
  w33 -= w39;
  /* #279: @33 = (@3*@33) */
  w33  = (w3*w33);
  /* #280: @7 = (@7*@37) */
  w7 *= w37;
  /* #281: @7 = (@12*@7) */
  w7  = (w12*w7);
  /* #282: @7 = (2.*@7) */
  w7 = (2.* w7 );
  /* #283: @35 = (@35*@7) */
  w35 *= w7;
  /* #284: @12 = (@12*@35) */
  w12 *= w35;
  /* #285: @34 = (@34*@12) */
  w34 *= w12;
  /* #286: @33 = (@33+@34) */
  w33 += w34;
  /* #287: @1 = (@1-@33) */
  w1 -= w33;
  /* #288: @15 = (@15*@12) */
  w15 *= w12;
  /* #289: @43 = (@43*@9) */
  w43 *= w9;
  /* #290: @3 = (@3*@43) */
  w3 *= w43;
  /* #291: @15 = (@15-@3) */
  w15 -= w3;
  /* #292: @3 = (-@15) */
  w3 = (- w15 );
  /* #293: @43 = (@40*@11) */
  w43  = (w40*w11);
  /* #294: @44 = (@44*@10) */
  w44 *= w10;
  /* #295: @6 = (@6*@44) */
  w6 *= w44;
  /* #296: @43 = (@43-@6) */
  w43 -= w6;
  /* #297: @3 = (@3-@43) */
  w3 -= w43;
  /* #298: @13 = (@13*@15) */
  w13 *= w15;
  /* #299: @43 = (@8*@43) */
  w43  = (w8*w43);
  /* #300: @13 = (@13-@43) */
  w13 -= w43;
  /* #301: @48 = 00 */
  /* #302: @50 = vertcat(@21, @22, @23, @24, @25, @26, @1, @3, @13, @45, @48, @11) */
  rr=w50;
  *rr++ = w1;
  *rr++ = w3;
  *rr++ = w13;
  *rr++ = w11;
  /* #303: @49 = @50[:4] */
  for (rr=w49, ss=w50+0; ss!=w50+4; ss+=1) *rr++ = *ss;
  /* #304: (@20[2, 7, 11, 17] = @49) */
  for (cii=casadi_s2, rr=w20, ss=w49; cii!=casadi_s2+4; ++cii, ++ss) rr[*cii] = *ss;
  /* #305: @1 = ones(12x1,1nz) */
  w1 = 1.;
  /* #306: {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, @3, NULL} = vertsplit(@1) */
  w3 = w1;
  /* #307: @3 = (@0*@3) */
  w3  = (w0*w3);
  /* #308: @1 = (2.*@3) */
  w1 = (2.* w3 );
  /* #309: @38 = (@38*@1) */
  w38 *= w1;
  /* #310: @3 = (2.*@3) */
  w3 = (2.* w3 );
  /* #311: @42 = (@42*@3) */
  w42 *= w3;
  /* #312: @38 = (@38+@42) */
  w38 += w42;
  /* #313: @2 = (@2*@38) */
  w2 *= w38;
  /* #314: @2 = (-@2) */
  w2 = (- w2 );
  /* #315: @48 = 00 */
  /* #316: @51 = 00 */
  /* #317: @0 = (@0*@38) */
  w0 *= w38;
  /* #318: @52 = 00 */
  /* #319: @27 = vertcat(@21, @22, @23, @24, @25, @26, @2, @48, @51, @45, @0, @52) */
  rr=w27;
  *rr++ = w2;
  *rr++ = w0;
  /* #320: @53 = @27[:2] */
  for (rr=w53, ss=w27+0; ss!=w27+2; ss+=1) *rr++ = *ss;
  /* #321: (@20[3:25:11] = @53) */
  for (rr=w20+3, ss=w53; rr!=w20+25; rr+=11) *rr = *ss++;
  /* #322: @2 = ones(12x1,1nz) */
  w2 = 1.;
  /* #323: {NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL, @0} = vertsplit(@2) */
  w0 = w2;
  /* #324: @0 = (@5*@0) */
  w0  = (w5*w0);
  /* #325: @0 = (2.*@0) */
  w0 = (2.* w0 );
  /* #326: @18 = (@18*@0) */
  w18 *= w0;
  /* #327: @5 = (@5*@18) */
  w5 *= w18;
  /* #328: @17 = (@17*@5) */
  w17 *= w5;
  /* #329: @40 = (@40*@5) */
  w40 *= w5;
  /* #330: @18 = (-@40) */
  w18 = (- w40 );
  /* #331: @8 = (@8*@40) */
  w8 *= w40;
  /* #332: @8 = (-@8) */
  w8 = (- w8 );
  /* #333: @48 = 00 */
  /* #334: @49 = vertcat(@21, @22, @23, @24, @25, @26, @17, @18, @8, @45, @48, @5) */
  rr=w49;
  *rr++ = w17;
  *rr++ = w18;
  *rr++ = w8;
  *rr++ = w5;
  /* #335: @50 = @49[:4] */
  for (rr=w50, ss=w49+0; ss!=w49+4; ss+=1) *rr++ = *ss;
  /* #336: (@20[4, 8, 12, 18] = @50) */
  for (cii=casadi_s3, rr=w20, ss=w50; cii!=casadi_s3+4; ++cii, ++ss) rr[*cii] = *ss;
  /* #337: @54 = @20' */
  casadi_trans(w20,casadi_s4, w54, casadi_s4, iw);
  /* #338: output[2][0] = @54 */
  casadi_copy(w54, 19, res[2]);
  return 0;
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_release(int mem) {
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_incref(void) {
}

CASADI_SYMBOL_EXPORT void f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_n_out(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_real f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    case 4: return "o4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s5;
    case 1: return casadi_s6;
    case 2: return casadi_s7;
    case 3: return casadi_s8;
    case 4: return casadi_s8;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s7;
    case 1: return casadi_s9;
    case 2: return casadi_s4;
    case 3: return casadi_s10;
    case 4: return casadi_s8;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 17;
  if (sz_res) *sz_res = 17;
  if (sz_iw) *sz_iw = 13;
  if (sz_w) *sz_w = 112;
  return 0;
}

CASADI_SYMBOL_EXPORT int f1tenth_bicycle_model_constr_h_fun_jac_uxt_zt_hess_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 17*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 17*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 13*sizeof(casadi_int);
  if (sz_w) *sz_w = 112*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
