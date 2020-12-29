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
  #define CASADI_PREFIX(ID) g_Constraints_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

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

static const casadi_int casadi_s0[16] = {12, 1, 0, 12, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const casadi_int casadi_s1[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s2[31] = {6, 12, 0, 2, 5, 8, 10, 13, 16, 16, 16, 16, 16, 16, 16, 1, 2, 0, 1, 2, 0, 1, 2, 4, 5, 3, 4, 5, 3, 4, 5};

/* g_Constraint:(q[12])->(Task[6]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=1.9600000000000001e-01;
  a1=-2.1149999999999999e-01;
  a2=arg[0]? arg[0][1] : 0;
  a3=sin(a2);
  a4=(a1*a3);
  a4=(a0+a4);
  a5=-2.3039000000000001e-01;
  a6=cos(a2);
  a7=arg[0]? arg[0][2] : 0;
  a8=sin(a7);
  a6=(a6*a8);
  a7=cos(a7);
  a3=(a3*a7);
  a6=(a6+a3);
  a6=(a5*a6);
  a4=(a4+a6);
  if (res[0]!=0) res[0][0]=a4;
  a4=4.9664000000000000e-02;
  a6=7.7476000000000003e-02;
  a3=arg[0]? arg[0][0] : 0;
  a9=cos(a3);
  a9=(a6*a9);
  a4=(a4+a9);
  a9=sin(a3);
  a10=cos(a2);
  a11=(a9*a10);
  a12=(a1*a11);
  a4=(a4-a12);
  a2=sin(a2);
  a9=(a9*a2);
  a9=(a9*a8);
  a11=(a11*a7);
  a9=(a9-a11);
  a9=(a5*a9);
  a4=(a4+a9);
  if (res[0]!=0) res[0][1]=a4;
  a4=sin(a3);
  a6=(a6*a4);
  a3=cos(a3);
  a10=(a3*a10);
  a4=(a1*a10);
  a6=(a6+a4);
  a10=(a10*a7);
  a3=(a3*a2);
  a3=(a3*a8);
  a10=(a10-a3);
  a10=(a5*a10);
  a6=(a6+a10);
  if (res[0]!=0) res[0][2]=a6;
  a6=arg[0]? arg[0][4] : 0;
  a10=sin(a6);
  a3=(a1*a10);
  a0=(a0+a3);
  a3=cos(a6);
  a8=arg[0]? arg[0][5] : 0;
  a2=sin(a8);
  a3=(a3*a2);
  a8=cos(a8);
  a10=(a10*a8);
  a3=(a3+a10);
  a3=(a5*a3);
  a0=(a0+a3);
  if (res[0]!=0) res[0][3]=a0;
  a0=-4.9664000000000000e-02;
  a3=-7.7476000000000003e-02;
  a10=arg[0]? arg[0][3] : 0;
  a7=cos(a10);
  a7=(a3*a7);
  a0=(a0+a7);
  a7=sin(a10);
  a4=cos(a6);
  a9=(a7*a4);
  a11=(a1*a9);
  a0=(a0-a11);
  a6=sin(a6);
  a7=(a7*a6);
  a7=(a7*a2);
  a9=(a9*a8);
  a7=(a7-a9);
  a7=(a5*a7);
  a0=(a0+a7);
  if (res[0]!=0) res[0][4]=a0;
  a0=sin(a10);
  a3=(a3*a0);
  a10=cos(a10);
  a4=(a10*a4);
  a1=(a1*a4);
  a3=(a3+a1);
  a4=(a4*a8);
  a10=(a10*a6);
  a10=(a10*a2);
  a4=(a4-a10);
  a5=(a5*a4);
  a3=(a3+a5);
  if (res[0]!=0) res[0][5]=a3;
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int g_Constraint_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int g_Constraint_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_release(int mem) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_incref(void) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real g_Constraint_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_name_in(casadi_int i){
  switch (i) {
    case 0: return "q";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_name_out(casadi_int i){
  switch (i) {
    case 0: return "Task";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int g_Constraint_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* g_Constraint_Jacobian:(q[12])->(TaskJacobian[6x12,16nz]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=-2.3039000000000001e-01;
  a1=arg[0]? arg[0][2] : 0;
  a2=sin(a1);
  a3=arg[0]? arg[0][1] : 0;
  a4=sin(a3);
  a5=arg[0]? arg[0][0] : 0;
  a6=cos(a5);
  a7=(a4*a6);
  a7=(a2*a7);
  a8=cos(a1);
  a9=cos(a3);
  a6=(a9*a6);
  a10=(a8*a6);
  a7=(a7-a10);
  a7=(a0*a7);
  a10=7.7476000000000003e-02;
  a11=sin(a5);
  a11=(a10*a11);
  a12=-2.1149999999999999e-01;
  a6=(a12*a6);
  a11=(a11+a6);
  a7=(a7-a11);
  if (res[0]!=0) res[0][0]=a7;
  a7=cos(a5);
  a10=(a10*a7);
  a7=sin(a5);
  a11=(a9*a7);
  a6=(a12*a11);
  a10=(a10-a6);
  a7=(a4*a7);
  a7=(a2*a7);
  a11=(a8*a11);
  a7=(a7-a11);
  a7=(a0*a7);
  a10=(a10+a7);
  if (res[0]!=0) res[0][1]=a10;
  a10=cos(a3);
  a7=(a12*a10);
  a10=(a8*a10);
  a11=sin(a3);
  a11=(a2*a11);
  a10=(a10-a11);
  a10=(a0*a10);
  a7=(a7+a10);
  if (res[0]!=0) res[0][2]=a7;
  a7=sin(a5);
  a10=sin(a3);
  a11=(a7*a10);
  a6=(a12*a11);
  a13=cos(a3);
  a14=(a7*a13);
  a14=(a2*a14);
  a11=(a8*a11);
  a14=(a14+a11);
  a14=(a0*a14);
  a6=(a6+a14);
  if (res[0]!=0) res[0][3]=a6;
  a5=cos(a5);
  a10=(a5*a10);
  a6=(a12*a10);
  a8=(a8*a10);
  a13=(a5*a13);
  a2=(a2*a13);
  a8=(a8+a2);
  a8=(a0*a8);
  a6=(a6+a8);
  a6=(-a6);
  if (res[0]!=0) res[0][4]=a6;
  a6=cos(a3);
  a8=cos(a1);
  a6=(a6*a8);
  a3=sin(a3);
  a1=sin(a1);
  a3=(a3*a1);
  a6=(a6-a3);
  a6=(a0*a6);
  if (res[0]!=0) res[0][5]=a6;
  a6=(a7*a4);
  a6=(a6*a8);
  a7=(a7*a9);
  a7=(a7*a1);
  a6=(a6+a7);
  a6=(a0*a6);
  if (res[0]!=0) res[0][6]=a6;
  a9=(a5*a9);
  a9=(a9*a1);
  a5=(a5*a4);
  a5=(a5*a8);
  a9=(a9+a5);
  a9=(a0*a9);
  a9=(-a9);
  if (res[0]!=0) res[0][7]=a9;
  a9=arg[0]? arg[0][5] : 0;
  a5=sin(a9);
  a8=arg[0]? arg[0][4] : 0;
  a4=sin(a8);
  a1=arg[0]? arg[0][3] : 0;
  a6=cos(a1);
  a7=(a4*a6);
  a7=(a5*a7);
  a3=cos(a9);
  a2=cos(a8);
  a6=(a2*a6);
  a13=(a3*a6);
  a7=(a7-a13);
  a7=(a0*a7);
  a13=-7.7476000000000003e-02;
  a10=sin(a1);
  a10=(a13*a10);
  a6=(a12*a6);
  a10=(a10+a6);
  a7=(a7-a10);
  if (res[0]!=0) res[0][8]=a7;
  a7=cos(a1);
  a13=(a13*a7);
  a7=sin(a1);
  a10=(a2*a7);
  a6=(a12*a10);
  a13=(a13-a6);
  a7=(a4*a7);
  a7=(a5*a7);
  a10=(a3*a10);
  a7=(a7-a10);
  a7=(a0*a7);
  a13=(a13+a7);
  if (res[0]!=0) res[0][9]=a13;
  a13=cos(a8);
  a7=(a12*a13);
  a13=(a3*a13);
  a10=sin(a8);
  a10=(a5*a10);
  a13=(a13-a10);
  a13=(a0*a13);
  a7=(a7+a13);
  if (res[0]!=0) res[0][10]=a7;
  a7=sin(a1);
  a13=sin(a8);
  a10=(a7*a13);
  a6=(a12*a10);
  a14=cos(a8);
  a11=(a7*a14);
  a11=(a5*a11);
  a10=(a3*a10);
  a11=(a11+a10);
  a11=(a0*a11);
  a6=(a6+a11);
  if (res[0]!=0) res[0][11]=a6;
  a1=cos(a1);
  a13=(a1*a13);
  a12=(a12*a13);
  a3=(a3*a13);
  a14=(a1*a14);
  a5=(a5*a14);
  a3=(a3+a5);
  a3=(a0*a3);
  a12=(a12+a3);
  a12=(-a12);
  if (res[0]!=0) res[0][12]=a12;
  a12=cos(a8);
  a3=cos(a9);
  a12=(a12*a3);
  a8=sin(a8);
  a9=sin(a9);
  a8=(a8*a9);
  a12=(a12-a8);
  a12=(a0*a12);
  if (res[0]!=0) res[0][13]=a12;
  a12=(a7*a4);
  a12=(a12*a3);
  a7=(a7*a2);
  a7=(a7*a9);
  a12=(a12+a7);
  a12=(a0*a12);
  if (res[0]!=0) res[0][14]=a12;
  a2=(a1*a2);
  a2=(a2*a9);
  a1=(a1*a4);
  a1=(a1*a3);
  a2=(a2+a1);
  a0=(a0*a2);
  a0=(-a0);
  if (res[0]!=0) res[0][15]=a0;
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f1(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_release(int mem) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_incref(void) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_Jacobian_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_Jacobian_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real g_Constraint_Jacobian_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_Jacobian_name_in(casadi_int i){
  switch (i) {
    case 0: return "q";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_Jacobian_name_out(casadi_int i){
  switch (i) {
    case 0: return "TaskJacobian";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_Jacobian_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_Jacobian_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* g_Constraint_Jacobian_derivative:(q[12],v[12])->(TaskJacobian_derivative[6x12,16nz]) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a3, a4, a5, a6, a7, a8, a9;
  a0=-2.3039000000000001e-01;
  a1=arg[0]? arg[0][2] : 0;
  a2=cos(a1);
  a3=arg[0]? arg[0][1] : 0;
  a4=cos(a3);
  a5=arg[0]? arg[0][0] : 0;
  a6=sin(a5);
  a7=(a4*a6);
  a8=(a2*a7);
  a9=sin(a1);
  a10=sin(a3);
  a6=(a10*a6);
  a6=(a9*a6);
  a8=(a8-a6);
  a8=(a0*a8);
  a6=7.7476000000000003e-02;
  a11=cos(a5);
  a11=(a6*a11);
  a12=-2.1149999999999999e-01;
  a7=(a12*a7);
  a11=(a11-a7);
  a8=(a8-a11);
  a11=arg[1]? arg[1][0] : 0;
  a8=(a8*a11);
  a7=cos(a5);
  a13=cos(a3);
  a14=(a7*a13);
  a14=(a9*a14);
  a15=sin(a3);
  a16=(a7*a15);
  a17=(a2*a16);
  a14=(a14+a17);
  a14=(a0*a14);
  a16=(a12*a16);
  a14=(a14+a16);
  a16=arg[1]? arg[1][1] : 0;
  a14=(a14*a16);
  a8=(a8+a14);
  a14=(a10*a7);
  a17=cos(a1);
  a14=(a14*a17);
  a7=(a4*a7);
  a18=sin(a1);
  a7=(a7*a18);
  a14=(a14+a7);
  a14=(a0*a14);
  a7=arg[1]? arg[1][2] : 0;
  a14=(a14*a7);
  a8=(a8+a14);
  if (res[0]!=0) res[0][0]=a8;
  a8=cos(a5);
  a14=(a10*a8);
  a14=(a9*a14);
  a8=(a4*a8);
  a19=(a2*a8);
  a14=(a14-a19);
  a14=(a0*a14);
  a19=sin(a5);
  a6=(a6*a19);
  a8=(a12*a8);
  a6=(a6+a8);
  a14=(a14-a6);
  a14=(a14*a11);
  a6=sin(a5);
  a8=(a6*a15);
  a19=(a12*a8);
  a20=(a6*a13);
  a20=(a9*a20);
  a8=(a2*a8);
  a20=(a20+a8);
  a20=(a0*a20);
  a19=(a19+a20);
  a19=(a19*a16);
  a14=(a14+a19);
  a19=(a10*a6);
  a19=(a19*a17);
  a6=(a4*a6);
  a6=(a6*a18);
  a19=(a19+a6);
  a19=(a0*a19);
  a19=(a19*a7);
  a14=(a14+a19);
  if (res[0]!=0) res[0][1]=a14;
  a14=sin(a3);
  a19=(a12*a14);
  a14=(a2*a14);
  a6=cos(a3);
  a6=(a9*a6);
  a14=(a14+a6);
  a14=(a0*a14);
  a19=(a19+a14);
  a19=(a19*a16);
  a14=cos(a3);
  a14=(a14*a18);
  a6=sin(a3);
  a6=(a6*a17);
  a14=(a14+a6);
  a14=(a0*a14);
  a14=(a14*a7);
  a19=(a19+a14);
  a19=(-a19);
  if (res[0]!=0) res[0][2]=a19;
  a19=sin(a3);
  a14=cos(a5);
  a6=(a19*a14);
  a20=(a12*a6);
  a8=cos(a3);
  a21=(a8*a14);
  a21=(a9*a21);
  a6=(a2*a6);
  a21=(a21+a6);
  a21=(a0*a21);
  a20=(a20+a21);
  a20=(a20*a11);
  a21=sin(a5);
  a6=cos(a3);
  a22=(a21*a6);
  a23=(a12*a22);
  a22=(a2*a22);
  a24=sin(a3);
  a25=(a21*a24);
  a25=(a9*a25);
  a22=(a22-a25);
  a22=(a0*a22);
  a23=(a23+a22);
  a23=(a23*a16);
  a20=(a20+a23);
  a23=(a21*a8);
  a23=(a23*a17);
  a22=(a21*a19);
  a22=(a22*a18);
  a23=(a23-a22);
  a23=(a0*a23);
  a23=(a23*a7);
  a20=(a20+a23);
  if (res[0]!=0) res[0][3]=a20;
  a20=sin(a5);
  a23=(a19*a20);
  a22=(a12*a23);
  a23=(a2*a23);
  a25=(a8*a20);
  a25=(a9*a25);
  a23=(a23+a25);
  a23=(a0*a23);
  a22=(a22+a23);
  a22=(a22*a11);
  a5=cos(a5);
  a6=(a5*a6);
  a23=(a12*a6);
  a2=(a2*a6);
  a24=(a5*a24);
  a9=(a9*a24);
  a2=(a2-a9);
  a2=(a0*a2);
  a23=(a23+a2);
  a23=(a23*a16);
  a22=(a22-a23);
  a8=(a5*a8);
  a8=(a8*a17);
  a19=(a5*a19);
  a19=(a19*a18);
  a8=(a8-a19);
  a8=(a0*a8);
  a8=(a8*a7);
  a22=(a22-a8);
  if (res[0]!=0) res[0][4]=a22;
  a22=cos(a1);
  a8=sin(a3);
  a8=(a22*a8);
  a19=sin(a1);
  a18=cos(a3);
  a18=(a19*a18);
  a8=(a8+a18);
  a8=(a0*a8);
  a8=(a8*a16);
  a18=cos(a3);
  a17=sin(a1);
  a18=(a18*a17);
  a3=sin(a3);
  a1=cos(a1);
  a3=(a3*a1);
  a18=(a18+a3);
  a18=(a0*a18);
  a18=(a18*a7);
  a8=(a8+a18);
  a8=(-a8);
  if (res[0]!=0) res[0][5]=a8;
  a8=(a10*a14);
  a8=(a22*a8);
  a14=(a4*a14);
  a14=(a19*a14);
  a8=(a8+a14);
  a8=(a0*a8);
  a8=(a8*a11);
  a14=(a21*a13);
  a14=(a22*a14);
  a18=(a21*a15);
  a18=(a19*a18);
  a14=(a14-a18);
  a14=(a0*a14);
  a14=(a14*a16);
  a8=(a8+a14);
  a14=(a21*a4);
  a14=(a14*a1);
  a21=(a21*a10);
  a21=(a21*a17);
  a14=(a14-a21);
  a14=(a0*a14);
  a14=(a14*a7);
  a8=(a8+a14);
  if (res[0]!=0) res[0][6]=a8;
  a8=(a4*a20);
  a8=(a19*a8);
  a20=(a10*a20);
  a20=(a22*a20);
  a8=(a8+a20);
  a8=(a0*a8);
  a8=(a8*a11);
  a13=(a5*a13);
  a22=(a22*a13);
  a15=(a5*a15);
  a19=(a19*a15);
  a22=(a22-a19);
  a22=(a0*a22);
  a22=(a22*a16);
  a8=(a8-a22);
  a4=(a5*a4);
  a4=(a4*a1);
  a5=(a5*a10);
  a5=(a5*a17);
  a4=(a4-a5);
  a4=(a0*a4);
  a4=(a4*a7);
  a8=(a8-a4);
  if (res[0]!=0) res[0][7]=a8;
  a8=arg[0]? arg[0][5] : 0;
  a4=cos(a8);
  a7=arg[0]? arg[0][4] : 0;
  a5=cos(a7);
  a17=arg[0]? arg[0][3] : 0;
  a10=sin(a17);
  a1=(a5*a10);
  a22=(a4*a1);
  a16=sin(a8);
  a19=sin(a7);
  a10=(a19*a10);
  a10=(a16*a10);
  a22=(a22-a10);
  a22=(a0*a22);
  a10=-7.7476000000000003e-02;
  a15=cos(a17);
  a15=(a10*a15);
  a1=(a12*a1);
  a15=(a15-a1);
  a22=(a22-a15);
  a15=arg[1]? arg[1][3] : 0;
  a22=(a22*a15);
  a1=cos(a17);
  a13=cos(a7);
  a11=(a1*a13);
  a11=(a16*a11);
  a20=sin(a7);
  a14=(a1*a20);
  a21=(a4*a14);
  a11=(a11+a21);
  a11=(a0*a11);
  a14=(a12*a14);
  a11=(a11+a14);
  a14=arg[1]? arg[1][4] : 0;
  a11=(a11*a14);
  a22=(a22+a11);
  a11=(a19*a1);
  a21=cos(a8);
  a11=(a11*a21);
  a1=(a5*a1);
  a18=sin(a8);
  a1=(a1*a18);
  a11=(a11+a1);
  a11=(a0*a11);
  a1=arg[1]? arg[1][5] : 0;
  a11=(a11*a1);
  a22=(a22+a11);
  if (res[0]!=0) res[0][8]=a22;
  a22=cos(a17);
  a11=(a19*a22);
  a11=(a16*a11);
  a22=(a5*a22);
  a3=(a4*a22);
  a11=(a11-a3);
  a11=(a0*a11);
  a3=sin(a17);
  a10=(a10*a3);
  a22=(a12*a22);
  a10=(a10+a22);
  a11=(a11-a10);
  a11=(a11*a15);
  a10=sin(a17);
  a22=(a10*a20);
  a3=(a12*a22);
  a23=(a10*a13);
  a23=(a16*a23);
  a22=(a4*a22);
  a23=(a23+a22);
  a23=(a0*a23);
  a3=(a3+a23);
  a3=(a3*a14);
  a11=(a11+a3);
  a3=(a19*a10);
  a3=(a3*a21);
  a10=(a5*a10);
  a10=(a10*a18);
  a3=(a3+a10);
  a3=(a0*a3);
  a3=(a3*a1);
  a11=(a11+a3);
  if (res[0]!=0) res[0][9]=a11;
  a11=sin(a7);
  a3=(a12*a11);
  a11=(a4*a11);
  a10=cos(a7);
  a10=(a16*a10);
  a11=(a11+a10);
  a11=(a0*a11);
  a3=(a3+a11);
  a3=(a3*a14);
  a11=cos(a7);
  a11=(a11*a18);
  a10=sin(a7);
  a10=(a10*a21);
  a11=(a11+a10);
  a11=(a0*a11);
  a11=(a11*a1);
  a3=(a3+a11);
  a3=(-a3);
  if (res[0]!=0) res[0][10]=a3;
  a3=sin(a7);
  a11=cos(a17);
  a10=(a3*a11);
  a23=(a12*a10);
  a22=cos(a7);
  a2=(a22*a11);
  a2=(a16*a2);
  a10=(a4*a10);
  a2=(a2+a10);
  a2=(a0*a2);
  a23=(a23+a2);
  a23=(a23*a15);
  a2=sin(a17);
  a10=cos(a7);
  a9=(a2*a10);
  a24=(a12*a9);
  a9=(a4*a9);
  a6=sin(a7);
  a25=(a2*a6);
  a25=(a16*a25);
  a9=(a9-a25);
  a9=(a0*a9);
  a24=(a24+a9);
  a24=(a24*a14);
  a23=(a23+a24);
  a24=(a2*a22);
  a24=(a24*a21);
  a9=(a2*a3);
  a9=(a9*a18);
  a24=(a24-a9);
  a24=(a0*a24);
  a24=(a24*a1);
  a23=(a23+a24);
  if (res[0]!=0) res[0][11]=a23;
  a23=sin(a17);
  a24=(a3*a23);
  a9=(a12*a24);
  a24=(a4*a24);
  a25=(a22*a23);
  a25=(a16*a25);
  a24=(a24+a25);
  a24=(a0*a24);
  a9=(a9+a24);
  a9=(a9*a15);
  a17=cos(a17);
  a10=(a17*a10);
  a12=(a12*a10);
  a4=(a4*a10);
  a6=(a17*a6);
  a16=(a16*a6);
  a4=(a4-a16);
  a4=(a0*a4);
  a12=(a12+a4);
  a12=(a12*a14);
  a9=(a9-a12);
  a22=(a17*a22);
  a22=(a22*a21);
  a3=(a17*a3);
  a3=(a3*a18);
  a22=(a22-a3);
  a22=(a0*a22);
  a22=(a22*a1);
  a9=(a9-a22);
  if (res[0]!=0) res[0][12]=a9;
  a9=cos(a8);
  a22=sin(a7);
  a22=(a9*a22);
  a3=sin(a8);
  a18=cos(a7);
  a18=(a3*a18);
  a22=(a22+a18);
  a22=(a0*a22);
  a22=(a22*a14);
  a18=cos(a7);
  a21=sin(a8);
  a18=(a18*a21);
  a7=sin(a7);
  a8=cos(a8);
  a7=(a7*a8);
  a18=(a18+a7);
  a18=(a0*a18);
  a18=(a18*a1);
  a22=(a22+a18);
  a22=(-a22);
  if (res[0]!=0) res[0][13]=a22;
  a22=(a19*a11);
  a22=(a9*a22);
  a11=(a5*a11);
  a11=(a3*a11);
  a22=(a22+a11);
  a22=(a0*a22);
  a22=(a22*a15);
  a11=(a2*a13);
  a11=(a9*a11);
  a18=(a2*a20);
  a18=(a3*a18);
  a11=(a11-a18);
  a11=(a0*a11);
  a11=(a11*a14);
  a22=(a22+a11);
  a11=(a2*a5);
  a11=(a11*a8);
  a2=(a2*a19);
  a2=(a2*a21);
  a11=(a11-a2);
  a11=(a0*a11);
  a11=(a11*a1);
  a22=(a22+a11);
  if (res[0]!=0) res[0][14]=a22;
  a22=(a5*a23);
  a22=(a3*a22);
  a23=(a19*a23);
  a23=(a9*a23);
  a22=(a22+a23);
  a22=(a0*a22);
  a22=(a22*a15);
  a13=(a17*a13);
  a9=(a9*a13);
  a20=(a17*a20);
  a3=(a3*a20);
  a9=(a9-a3);
  a9=(a0*a9);
  a9=(a9*a14);
  a22=(a22-a9);
  a5=(a17*a5);
  a5=(a5*a8);
  a17=(a17*a19);
  a17=(a17*a21);
  a5=(a5-a17);
  a0=(a0*a5);
  a0=(a0*a1);
  a22=(a22-a0);
  if (res[0]!=0) res[0][15]=a22;
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_derivative(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f2(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_derivative_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_derivative_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_derivative_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_derivative_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_derivative_release(int mem) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_derivative_incref(void) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_derivative_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_Jacobian_derivative_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_Jacobian_derivative_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real g_Constraint_Jacobian_derivative_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_Jacobian_derivative_name_in(casadi_int i){
  switch (i) {
    case 0: return "q";
    case 1: return "v";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_Jacobian_derivative_name_out(casadi_int i){
  switch (i) {
    case 0: return "TaskJacobian_derivative";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_Jacobian_derivative_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_Jacobian_derivative_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_derivative_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
