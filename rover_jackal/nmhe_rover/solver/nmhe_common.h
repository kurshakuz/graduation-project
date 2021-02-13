/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#ifndef NMHE_COMMON_H
#define NMHE_COMMON_H

#include <math.h>
#include <string.h>

#ifndef __MATLAB__
#ifdef __cplusplus
extern "C"
{
#endif /* __cplusplus */
#endif /* __MATLAB__ */

/** \defgroup NMHE ACADO CGT generated module. */
/** @{ */

/** qpOASES QP solver indicator. */
#define NMHE_QPOASES  0
#define NMHE_QPOASES3 1
/** FORCES QP solver indicator.*/
#define NMHE_FORCES   2
/** qpDUNES QP solver indicator.*/
#define NMHE_QPDUNES  3
/** HPMPC QP solver indicator. */
#define NMHE_HPMPC    4
#define NMHE_GENERIC    5

/** Indicator for determining the QP solver used by the ACADO solver code. */
#define NMHE_QP_SOLVER NMHE_QPOASES

#include "nmhe_qpoases_interface.hpp"


/*
 * Common definitions
 */
/** User defined block based condensing. */
#define NMHE_BLOCK_CONDENSING 0
/** Compute covariance matrix of the last state estimate. */
#define NMHE_COMPUTE_COVARIANCE_MATRIX 0
/** Flag indicating whether constraint values are hard-coded or not. */
#define NMHE_HARDCODED_CONSTRAINT_VALUES 1
/** Indicator for fixed initial state. */
#define NMHE_INITIAL_STATE_FIXED 0
/** Number of control/estimation intervals. */
#define NMHE_N 40
/** Number of online data values. */
#define NMHE_NOD 0
/** Number of path constraints. */
#define NMHE_NPAC 0
/** Number of control variables. */
#define NMHE_NU 2
/** Number of differential variables. */
#define NMHE_NX 7
/** Number of algebraic variables. */
#define NMHE_NXA 0
/** Number of differential derivative variables. */
#define NMHE_NXD 0
/** Number of references/measurements per node on the first N nodes. */
#define NMHE_NY 7
/** Number of references/measurements on the last (N + 1)st node. */
#define NMHE_NYN 5
/** Total number of QP optimization variables. */
#define NMHE_QP_NV 87
/** Number of integration steps per shooting interval. */
#define NMHE_RK_NIS 2
/** Number of Runge-Kutta stages per integration step. */
#define NMHE_RK_NSTAGES 4
/** Providing interface for arrival cost. */
#define NMHE_USE_ARRIVAL_COST 1
/** Indicator for usage of non-hard-coded linear terms in the objective. */
#define NMHE_USE_LINEAR_TERMS 0
/** Indicator for type of fixed weighting matrices. */
#define NMHE_WEIGHTING_MATRICES_TYPE 1


/*
 * Globally used structure definitions
 */

/** The structure containing the user data.
 * 
 *  Via this structure the user "communicates" with the solver code.
 */
typedef struct NMHEvariables_
{
int dummy;
/** Matrix of size: 41 x 7 (row major format)
 * 
 *  Matrix containing 41 differential variable vectors.
 */
real_t x[ 287 ];

/** Matrix of size: 40 x 2 (row major format)
 * 
 *  Matrix containing 40 control variable vectors.
 */
real_t u[ 80 ];

/** Column vector of size: 280
 * 
 *  Matrix containing 40 reference/measurement vectors of size 7 for first 40 nodes.
 */
real_t y[ 280 ];

/** Column vector of size: 5
 * 
 *  Reference/measurement vector for the 41. node.
 */
real_t yN[ 5 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t W[ 49 ];

/** Matrix of size: 5 x 5 (row major format) */
real_t WN[ 25 ];

/** Matrix of size: 7 x 7 (row major format)
 * 
 *  Arrival cost term: inverse of the covariance matrix.
 */
real_t SAC[ 49 ];

/** Column vector of size: 7
 * 
 *  Arrival cost term: a priori state estimate.
 */
real_t xAC[ 7 ];

/** Matrix of size: 7 x 7 (row major format)
 * 
 *  Arrival cost term: Cholesky decomposition, lower triangular,  of the inverse of the state noise covariance matrix.
 */
real_t WL[ 49 ];


} NMHEvariables;

/** Private workspace used by the auto-generated code.
 * 
 *  Data members of this structure are private to the solver.
 *  In other words, the user code should not modify values of this 
 *  structure. 
 */
typedef struct NMHEworkspace_
{
/** Column vector of size: 93 */
real_t rhs_aux[ 93 ];

real_t rk_ttt;

/** Row vector of size: 72 */
real_t rk_xxx[ 72 ];

/** Matrix of size: 4 x 70 (row major format) */
real_t rk_kkk[ 280 ];

/** Row vector of size: 72 */
real_t state[ 72 ];

/** Column vector of size: 280 */
real_t d[ 280 ];

/** Column vector of size: 280 */
real_t Dy[ 280 ];

/** Column vector of size: 5 */
real_t DyN[ 5 ];

/** Matrix of size: 280 x 7 (row major format) */
real_t evGx[ 1960 ];

/** Matrix of size: 280 x 2 (row major format) */
real_t evGu[ 560 ];

/** Row vector of size: 9 */
real_t objValueIn[ 9 ];

/** Row vector of size: 7 */
real_t objValueOut[ 7 ];

/** Matrix of size: 280 x 7 (row major format) */
real_t Q1[ 1960 ];

/** Matrix of size: 280 x 7 (row major format) */
real_t Q2[ 1960 ];

/** Matrix of size: 80 x 2 (row major format) */
real_t R1[ 160 ];

/** Matrix of size: 80 x 7 (row major format) */
real_t R2[ 560 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t QN1[ 49 ];

/** Matrix of size: 7 x 5 (row major format) */
real_t QN2[ 35 ];

/** Column vector of size: 7 */
real_t DxAC[ 7 ];

/** Matrix of size: 21 x 16 (row major format) */
real_t acA[ 336 ];

/** Column vector of size: 21 */
real_t acb[ 21 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t acP[ 49 ];

/** Row vector of size: 22 */
real_t rk_actemp[ 22 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t acVL[ 49 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t acHx[ 49 ];

/** Matrix of size: 7 x 2 (row major format) */
real_t acHu[ 14 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t acXx[ 49 ];

/** Matrix of size: 7 x 2 (row major format) */
real_t acXu[ 14 ];

/** Column vector of size: 7 */
real_t acXTilde[ 7 ];

/** Column vector of size: 7 */
real_t acHTilde[ 7 ];

/** Matrix of size: 7 x 7 (row major format) */
real_t T[ 49 ];

/** Matrix of size: 5740 x 2 (row major format) */
real_t E[ 11480 ];

/** Matrix of size: 5740 x 2 (row major format) */
real_t QE[ 11480 ];

/** Matrix of size: 280 x 7 (row major format) */
real_t QGx[ 1960 ];

/** Column vector of size: 280 */
real_t Qd[ 280 ];

/** Column vector of size: 287 */
real_t QDy[ 287 ];

/** Matrix of size: 80 x 7 (row major format) */
real_t H10[ 560 ];

/** Matrix of size: 87 x 87 (row major format) */
real_t H[ 7569 ];

/** Matrix of size: 80 x 87 (row major format) */
real_t A[ 6960 ];

/** Column vector of size: 87 */
real_t g[ 87 ];

/** Column vector of size: 87 */
real_t lb[ 87 ];

/** Column vector of size: 87 */
real_t ub[ 87 ];

/** Column vector of size: 80 */
real_t lbA[ 80 ];

/** Column vector of size: 80 */
real_t ubA[ 80 ];

/** Column vector of size: 87 */
real_t x[ 87 ];

/** Column vector of size: 167 */
real_t y[ 167 ];


} NMHEworkspace;

/* 
 * Forward function declarations. 
 */


/** Performs the integration and sensitivity propagation for one shooting interval.
 *
 *  \param rk_eta Working array to pass the input values and return the results.
 *  \param resetIntegrator The internal memory of the integrator can be reset.
 *
 *  \return Status code of the integrator.
 */
int nmhe_integrate( real_t* const rk_eta, int resetIntegrator );

/** Export of an ACADO symbolic function.
 *
 *  \param in Input to the exported function.
 *  \param out Output of the exported function.
 */
void nmhe_rhs_forw(const real_t* in, real_t* out);

/** Preparation step of the RTI scheme.
 *
 *  \return Status of the integration module. =0: OK, otherwise the error code.
 */
int nmhe_preparationStep(  );

/** Feedback/estimation step of the RTI scheme.
 *
 *  \return Status code of the qpOASES QP solver.
 */
int nmhe_feedbackStep(  );

/** Solver initialization. Must be called once before any other function call.
 *
 *  \return =0: OK, otherwise an error code of a QP solver.
 */
int nmhe_initializeSolver(  );

/** Initialize shooting nodes by a forward simulation starting from the first node.
 */
void nmhe_initializeNodesByForwardSimulation(  );

/** Shift differential variables vector by one interval.
 *
 *  \param strategy Shifting strategy: 1. Initialize node 41 with xEnd. 2. Initialize node 41 by forward simulation.
 *  \param xEnd Value for the x vector on the last node. If =0 the old value is used.
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void nmhe_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd );

/** Shift controls vector by one interval.
 *
 *  \param uEnd Value for the u vector on the second to last node. If =0 the old value is used.
 */
void nmhe_shiftControls( real_t* const uEnd );

/** Get the KKT tolerance of the current iterate.
 *
 *  \return The KKT tolerance value.
 */
real_t nmhe_getKKT(  );

/** Calculate the objective value.
 *
 *  \return Value of the objective function.
 */
real_t nmhe_getObjective(  );

/** Use this function to update the arrival cost.
 *
 *  \param reset Reset S_{AC}. Set it to 1 to initialize arrival cost calculation, and later should set it to 0.
 */
int nmhe_updateArrivalCost( int reset );


/* 
 * Extern declarations. 
 */

extern NMHEworkspace nmheWorkspace;
extern NMHEvariables nmheVariables;

/** @} */

#ifndef __MATLAB__
#ifdef __cplusplus
} /* extern "C" */
#endif /* __cplusplus */
#endif /* __MATLAB__ */

#endif /* NMHE_COMMON_H */
