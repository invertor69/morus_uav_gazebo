/* Produced by CVXGEN, 2017-06-28 13:19:13 -0400.  */
/* CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2012 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: solver.h. */
/* Description: Header file with relevant definitions. */
#ifndef SOLVER_H
#define SOLVER_H

// enable the C++ compiler to compile C code
#ifdef __cplusplus
extern "C" {
#endif

/* Uncomment the next line to remove all library dependencies. */
/*#define ZERO_LIBRARY_MODE */
#ifdef MATLAB_MEX_FILE
/* Matlab functions. MATLAB_MEX_FILE will be defined by the mex compiler. */
/* If you are not using the mex compiler, this functionality will not intrude, */
/* as it will be completely disabled at compile-time. */
#include "mex.h"
#else
#ifndef ZERO_LIBRARY_MODE
#include <stdio.h>
#endif
#endif
/* Space must be allocated somewhere (testsolver.c, csolve.c or your own */
/* program) for the global variables vars, params, work and settings. */
/* At the bottom of this file, they are externed. */
#ifndef ZERO_LIBRARY_MODE
#include <math.h>
#define pm(A, m, n) printmatrix(#A, A, m, n, 1)
#endif
typedef struct Params_t {
  double u_ss[2];
  double R[4];
  double u_prev[2];
  double R_delta[4];
  double x_ss_1[6];
  double Q[36];
  double x_ss_2[6];
  double x_ss_3[6];
  double x_ss_4[6];
  double x_ss_5[6];
  double x_ss_6[6];
  double x_ss_7[6];
  double x_ss_8[6];
  double x_ss_9[6];
  double x_ss_10[6];
  double x_ss_11[6];
  double x_ss_12[6];
  double x_ss_13[6];
  double x_ss_14[6];
  double x_ss_15[6];
  double x_ss_16[6];
  double x_ss_17[6];
  double x_ss_18[6];
  double x_ss_19[6];
  double P[36];
  double A[36];
  double x_0[6];
  double B[12];
  double Bd[36];
  double d[6];
  double u_min[2];
  double u_max[2];
  double *x_ss[20];
  double *x[1];
} Params;
typedef struct Vars_t {
  double *u_0; /* 2 rows. */
  double *x_1; /* 6 rows. */
  double *u_1; /* 2 rows. */
  double *t_01; /* 2 rows. */
  double *x_2; /* 6 rows. */
  double *u_2; /* 2 rows. */
  double *t_02; /* 2 rows. */
  double *x_3; /* 6 rows. */
  double *u_3; /* 2 rows. */
  double *t_03; /* 2 rows. */
  double *x_4; /* 6 rows. */
  double *u_4; /* 2 rows. */
  double *t_04; /* 2 rows. */
  double *x_5; /* 6 rows. */
  double *u_5; /* 2 rows. */
  double *t_05; /* 2 rows. */
  double *x_6; /* 6 rows. */
  double *u_6; /* 2 rows. */
  double *t_06; /* 2 rows. */
  double *x_7; /* 6 rows. */
  double *u_7; /* 2 rows. */
  double *t_07; /* 2 rows. */
  double *x_8; /* 6 rows. */
  double *u_8; /* 2 rows. */
  double *t_08; /* 2 rows. */
  double *x_9; /* 6 rows. */
  double *u_9; /* 2 rows. */
  double *t_09; /* 2 rows. */
  double *x_10; /* 6 rows. */
  double *u_10; /* 2 rows. */
  double *t_10; /* 2 rows. */
  double *x_11; /* 6 rows. */
  double *u_11; /* 2 rows. */
  double *t_11; /* 2 rows. */
  double *x_12; /* 6 rows. */
  double *u_12; /* 2 rows. */
  double *t_12; /* 2 rows. */
  double *x_13; /* 6 rows. */
  double *u_13; /* 2 rows. */
  double *t_13; /* 2 rows. */
  double *x_14; /* 6 rows. */
  double *u_14; /* 2 rows. */
  double *t_14; /* 2 rows. */
  double *x_15; /* 6 rows. */
  double *u_15; /* 2 rows. */
  double *t_15; /* 2 rows. */
  double *x_16; /* 6 rows. */
  double *u_16; /* 2 rows. */
  double *t_16; /* 2 rows. */
  double *x_17; /* 6 rows. */
  double *u_17; /* 2 rows. */
  double *t_17; /* 2 rows. */
  double *x_18; /* 6 rows. */
  double *u_18; /* 2 rows. */
  double *t_18; /* 2 rows. */
  double *x_19; /* 6 rows. */
  double *u[19];
  double *x[20];
} Vars;
typedef struct Workspace_t {
  double h[72];
  double s_inv[72];
  double s_inv_z[72];
  double b[150];
  double q[188];
  double rhs[482];
  double x[482];
  double *s;
  double *z;
  double *y;
  double lhs_aff[482];
  double lhs_cc[482];
  double buffer[482];
  double buffer2[482];
  double KKT[1896];
  double L[2765];
  double d[482];
  double v[482];
  double d_inv[482];
  double gap;
  double optval;
  double ineq_resid_squared;
  double eq_resid_squared;
  double block_33[1];
  /* Pre-op symbols. */
  double quad_830833205248[1];
  double quad_689417555968[1];
  double quad_600568381440[1];
  double quad_898851794944[1];
  double quad_88433618944[1];
  double quad_240204779520[1];
  double quad_635618762752[1];
  double quad_732753989632[1];
  double quad_427523055616[1];
  double quad_976046530560[1];
  double quad_688550678528[1];
  double quad_304816418816[1];
  double quad_819339411456[1];
  double quad_101800079360[1];
  double quad_976903761920[1];
  double quad_141299838976[1];
  double quad_343404097536[1];
  double quad_815806124032[1];
  double quad_997002137600[1];
  double quad_141630619648[1];
  double quad_677275955200[1];
  int converged;
} Workspace;
typedef struct Settings_t {
  double resid_tol;
  double eps;
  int max_iters;
  int refine_steps;
  int better_start;
  /* Better start obviates the need for s_init and z_init. */
  double s_init;
  double z_init;
  int verbose;
  /* Show extra details of the iterative refinement steps. */
  int verbose_refinement;
  int debug;
  /* For regularization. Minimum value of abs(D_ii) in the kkt D factor. */
  double kkt_reg;
} Settings;
extern Vars vars;
extern Params params;
extern Workspace work;
extern Settings settings;
/* Function definitions in ldl.c: */
void ldl_solve(double *target, double *var);
void ldl_factor(void);
double check_factorization(void);
void matrix_multiply(double *result, double *source);
double check_residual(double *target, double *multiplicand);
void fill_KKT(void);

/* Function definitions in matrix_support.c: */
void multbymA(double *lhs, double *rhs);
void multbymAT(double *lhs, double *rhs);
void multbymG(double *lhs, double *rhs);
void multbymGT(double *lhs, double *rhs);
void multbyP(double *lhs, double *rhs);
void fillq(void);
void fillh(void);
void fillb(void);
void pre_ops(void);

/* Function definitions in solver.c: */
double eval_gap(void);
void set_defaults(void);
void setup_pointers(void);
void setup_indexed_params(void);
void setup_indexed_optvars(void);
void setup_indexing(void);
void set_start(void);
double eval_objv(void);
void fillrhs_aff(void);
void fillrhs_cc(void);
void refine(double *target, double *var);
double calc_ineq_resid_squared(void);
double calc_eq_resid_squared(void);
void better_start(void);
void fillrhs_start(void);
long solve(void);

/* Function definitions in testsolver.c: */
int main(int argc, char **argv);
void load_default_data(void);

/* Function definitions in util.c: */
void tic(void);
float toc(void);
float tocq(void);
void printmatrix(char *name, double *A, int m, int n, int sparse);
double unif(double lower, double upper);
float ran1(long *idum, int reset);
float randn_internal(long *idum, int reset);
double randn(void);
void reset_rand(void);

// enable the C++ compiler to compile C code
#ifdef __cplusplus
}
#endif

#endif
