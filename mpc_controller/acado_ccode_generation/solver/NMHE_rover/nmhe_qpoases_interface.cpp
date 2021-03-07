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


extern "C"
{
#include "nmhe_common.h"
}

#include "INCLUDE/QProblem.hpp"

#if NMHE_COMPUTE_COVARIANCE_MATRIX == 1
#include "INCLUDE/EXTRAS/SolutionAnalysis.hpp"
#endif /* NMHE_COMPUTE_COVARIANCE_MATRIX */

static int nmhe_nWSR;



#if NMHE_COMPUTE_COVARIANCE_MATRIX == 1
static SolutionAnalysis nmhe_sa;
#endif /* NMHE_COMPUTE_COVARIANCE_MATRIX */

int nmhe_solve( void )
{
	nmhe_nWSR = QPOASES_NWSRMAX;

	QProblem qp(87, 80);
	
	returnValue retVal = qp.init(nmheWorkspace.H, nmheWorkspace.g, nmheWorkspace.A, nmheWorkspace.lb, nmheWorkspace.ub, nmheWorkspace.lbA, nmheWorkspace.ubA, nmhe_nWSR, nmheWorkspace.y);

    qp.getPrimalSolution( nmheWorkspace.x );
    qp.getDualSolution( nmheWorkspace.y );
	
#if NMHE_COMPUTE_COVARIANCE_MATRIX == 1

	if (retVal != SUCCESSFUL_RETURN)
		return (int)retVal;
		
	retVal = nmhe_sa.getHessianInverse( &qp,var );

#endif /* NMHE_COMPUTE_COVARIANCE_MATRIX */

	return (int)retVal;
}

int nmhe_getNWSR( void )
{
	return nmhe_nWSR;
}

const char* nmhe_getErrorString( int error )
{
	return MessageHandling::getErrorString( error );
}
