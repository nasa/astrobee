/*
 *	This file is part of qp42.
 *
 *	qp42 -- An Implementation of qp solver on 42 intervals.
 *	Copyright (C) 2012 by Janick Frasch, Hans Joachim Ferreau et al. 
 *	All rights reserved.
 *
 *	qp42 is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qp5 is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qp42; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file examples/singleItegrator.c
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2012
 *
 *	Example for single integrator
 */



#include <mpc/setup_mpc.h>

#define INFTY 1.0e12

int main( )
{
	int i;
	int k;
	
	return_t statusFlag;

	const unsigned int nI = 50;			/* number of control intervals */
	const unsigned int nX = 1;			/* number of states */
	const unsigned int nU = 1;			/* number of controls */
	unsigned int* nD = 0;	  			/* number of constraints */
	

	double dt = 0.01;	/* discretization sampling time 10ms, needed for constraints */
// 	double dt = 1.0; // for now....
	
	double x0[2] = 
		{ -1.0 };
	
	
	double Q[1*1] =
		{	1.0e1
		};
	
	double R[1*1] =
		{	1.0e-4
		};
	double *S=0;
	
	double* P = Q;
	
	double A[1*1] =
		{	
			1.0
		};
	
	double B[1*1] =
		{
			1.0*dt
		};
	
	double c[1] =
		{	0.0
		};
			
			
	double xiLow[1] =
		{	-1.9	};
	
	double xiUpp[1] =
		{	1.9	};
	double uiLow[1] =
		{	-3.0	};
	double uiUpp[1] =
		{	 3.0	};
	double xiRef[1] =
		{ -0.0 };
	double uiRef[1] =
		{  0.0 };
	

	/** build up bounds and reference vectors */
	double xLow[nX*(nI+1)];
	double xUpp[nX*(nI+1)];
	double uLow[nU*nI];
	double uUpp[nU*nI];
	double xRef[nX*(nI+1)];
	double uRef[nU*nI];
	for ( k=0; k<nI; ++k ) {
		for ( i=0; i<nX; ++i ) {
			xLow[k*nX+i] = xiLow[i];
			xUpp[k*nX+i] = xiUpp[i];
			xRef[k*nX+i] = xiRef[i];
		}
		for ( i=0; i<nU; ++i ) {
			uLow[k*nU+i] = uiLow[i];
			uUpp[k*nU+i] = uiUpp[i];
			uRef[k*nU+i] = uiRef[i];
		}
	}
	for ( i=0; i<nX; ++i ) {
		xLow[nI*nX+i] = xiLow[i];
		xUpp[nI*nX+i] = xiUpp[i];
		xRef[nI*nX+i] = xiRef[i];
	}


	mpcProblem_t mpcProblem;
	
	qpOptions_t qpOptions = qpDUNES_setupDefaultOptions();
	qpOptions.maxIter    = 100;
	qpOptions.printLevel = 3;
	qpOptions.stationarityTolerance = 1.e-6;
	qpOptions.equalityTolerance     = 2.221e-16;
	qpOptions.QPDUNES_ZERO             = 1.0e-50;
	qpOptions.QPDUNES_INFTY            = INFTY;
	qpOptions.maxNumLineSearchIterations            = 8; //4
//	qpOptions.nwtnHssnFacAlg 		= QPDUNES_NH_FAC_BAND_FORWARD;
	
	mpcDUNES_setup( &mpcProblem, nI, nX, nU, nD, &(qpOptions) );


	
	double t0 = getTime();
// 	for ( i=0; i<100; ++i ) {
	/** initialize the MPC problem; note that matrices are factorized already now */
	statusFlag = mpcDUNES_initLtiSb_xu( &mpcProblem, Q, R, 0, S, A, B, c, xLow, xUpp, uLow, uUpp, xRef, uRef );
	if (statusFlag != QPDUNES_OK) {
		printf( "mpcDUNES setup failed.\n" );
		return (int)statusFlag;
	}
	
 	double t1 = getTime();
	/** get feedback for a given initial QP value */
	mpcDUNES_solve( &mpcProblem, x0 );
// 	}
	double t2 = getTime();
	
	printf( "Preparation time:       %.2lf ms\n", 1e3*(t1-t0) );
	printf( "Solution time:          %.2lf ms\n", 1e3*(t2-t1) );
	printf( "Total computation time: %.2lf ms\n", 1e3*(t2-t0) );
// 	printf( "Average computation time of 100 runs: %lf ms\n", 1e3*(t2-t1)/100 );
	
	
	qpDUNES_printMatrixData( mpcProblem.xOpt, nI+1, nX, "xOpt:" );
	qpDUNES_printMatrixData( mpcProblem.uOpt, nI, nU, "uOpt:" );
	qpDUNES_printMatrixData( mpcProblem.lambdaOpt, nI, nX, "lambdaOpt:" );
	
	mpcDUNES_cleanup( &mpcProblem );
	
	return 0;
}


/*
 *	end of file
 */
