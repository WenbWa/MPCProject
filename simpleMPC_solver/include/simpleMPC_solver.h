/*
Header file containing definitions for C interface of simpleMPC_solver,
 a fast costumized optimization solver.
*/

#ifndef simpleMPC_solver_H
#define simpleMPC_solver_H

#include <stdio.h>

/* For Visual Studio 2015 Compatibility */
#if (_MSC_VER >= 1900)
FILE * __cdecl __iob_func(void);
#endif
/* DATA TYPE ------------------------------------------------------------*/
typedef double simpleMPC_solver_float;

#ifndef SOLVER_STANDARD_TYPES
#define SOLVER_STANDARD_TYPES

typedef signed char solver_int8_signed;
typedef unsigned char solver_int8_unsigned;
typedef char solver_int8_default;
typedef signed short int solver_int16_signed;
typedef unsigned short int solver_int16_unsigned;
typedef short int solver_int16_default;
typedef signed int solver_int32_signed;
typedef unsigned int solver_int32_unsigned;
typedef int solver_int32_default;
typedef signed long long int solver_int64_signed;
typedef unsigned long long int solver_int64_unsigned;
typedef long long int solver_int64_default;

#endif

/* SOLVER SETTINGS ------------------------------------------------------*/
/* print level */
#ifndef SET_PRINTLEVEL_simpleMPC_solver
#define SET_PRINTLEVEL_simpleMPC_solver    (2)
#endif

/* PARAMETERS -----------------------------------------------------------*/
/* fill this with data before calling the solver! */
typedef struct
{
	/* column vector of length 3 */
	simpleMPC_solver_float xinit[3];

} simpleMPC_solver_params;


/* OUTPUTS --------------------------------------------------------------*/
/* the desired variables are put here by the solver */
typedef struct
{
	/* column vector of length 3 */
	simpleMPC_solver_float u0[3];

} simpleMPC_solver_output;


/* SOLVER INFO ----------------------------------------------------------*/
/* diagnostic data from last interior point step */
typedef struct
{
	/* iteration number */
	solver_int32_default it;

	/* number of iterations needed to optimality (branch-and-bound) */
	solver_int32_default it2opt;

	/* inf-norm of equality constraint residuals */
	simpleMPC_solver_float res_eq;

	/* inf-norm of inequality constraint residuals */
	simpleMPC_solver_float res_ineq;

	/* primal objective */
	simpleMPC_solver_float pobj;

	/* dual objective */
	simpleMPC_solver_float dobj;

	/* duality gap := pobj - dobj */
	simpleMPC_solver_float dgap;

	/* relative duality gap := |dgap / pobj | */
	simpleMPC_solver_float rdgap;

	/* duality measure */
	simpleMPC_solver_float mu;

	/* duality measure (after affine step) */
	simpleMPC_solver_float mu_aff;

	/* centering parameter */
	simpleMPC_solver_float sigma;

	/* number of backtracking line search steps (affine direction) */
	solver_int32_default lsit_aff;

	/* number of backtracking line search steps (combined direction) */
	solver_int32_default lsit_cc;

	/* step size (affine direction) */
	simpleMPC_solver_float step_aff;

	/* step size (combined direction) */
	simpleMPC_solver_float step_cc;

	/* solvertime */
	simpleMPC_solver_float solvetime;

} simpleMPC_solver_info;


/* SOLVER FUNCTION DEFINITION -------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif

/* examine exitflag before using the result! */
extern solver_int32_default simpleMPC_solver_solve(simpleMPC_solver_params *params, simpleMPC_solver_output *output, simpleMPC_solver_info *info, FILE *fs);

#ifdef __cplusplus
}
#endif

#endif /* simpleMPC_solver_H */
