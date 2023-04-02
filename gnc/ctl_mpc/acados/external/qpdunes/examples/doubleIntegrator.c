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
 *	\file examples/doubleItegrator.c
 *	\author Janick Frasch, Xin Wang
 *	\version 1.0beta
 *	\date 2012
 *
 *	Example for badminton robot
 */


#include <mpc/setup_mpc.h>


#define INFTY 1.0e12

#define ENERGY_OPT


int main( )
{
	int i,j;
	int k;
	
	return_t statusFlag;

	const unsigned int nI = 200;	/* number of control intervals */
//	const unsigned int nI = 48;		/* number of control intervals */
//	const unsigned int nI = 100;		/* number of control intervals */
	const unsigned int nX = 2;		/* number of states */
	const unsigned int nU = 1;		/* number of controls */
	unsigned int* nD = 0;  			/* number of constraints */
	

	double dt = 0.01;	/* discretization sampling time 10ms, needed for constraints */
// 	double dt = 1.0; // for now....
	
	double x0[2] = 
		{ -1.0, 0.0 };
//		{ -0.00, 0.0 };
	
	
	double Qreg[2*2] =
		{	1.0e0, 0.0,
			0.0, 1.0e0
//		{	1.5e0, 0.0,
//			0.0, 1.5e0
		};
	double Rreg[1*1] =
		{	1.00e0
//		{	1.5e0
		};
#ifdef ENERGY_OPT
	double Qorig[2*2] =
//		{	1.001e-4, 0.0,
//			0.0, 1.001e-4
		{	1.0e-8, 0.0,
			0.0, 1.0e-8
		};
	double Rorig[1*1] =
		{	1.0e0
		};
#else
	double Qorig[2*2] =
// 		{	1.0e-8, 0.0,
		{	1.0e1, 0.0,
//			0.0, 1.0
//			0.0, 1.0e-8
			0.0, 1.0e-4
		};
	double Rorig[1*1] =
// 		{	1.0
//		{	1.0e-8
		{	1.0e-4
		};
#endif

	double *S=0;
	
	double* Preg = Qreg;
	double* Porig = Qorig;
//	double Porig[2*2] =
//		{	 1.520682425311345e-03,     1.074043399830716e-02,
//			 1.074043399830717e-02,     1.524243869845707e-01		// solution to riccati recursion
//		};
	
	double A[2*2] =
		{	
			1.0, 1.0*dt,
			0.0, 1.0
		};
	
	double B[2*1] =
		{
			0.0,
			1.0*dt
		};
	
	double c[2] = 
		{	0.0,
			0.0
		};
			
			
	double xiLow[2] =
//		{	-1.9, -3.0	};
		{	-1.9, -3.0*1	};
	
	double xiUpp[2] =
//		{	1.9, 3.0	};
		{	1.9, 3.0*1	};
	double uiLow[1] =
		{	-30.0	};
	double uiUpp[1] =
		{	30.0	};
// 	double *xRef = 0, *uRef = 0;
	double xiRef[2] =
		{ -0.0, 0.0 };
	double uiRef[1] =
		{ 0.0 };
	

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


	/* arrival constraints */
//	int idxArrivalStart = 10;	/* 44 is the minimum number for this constraint that still works */
//	int idxArrivalStart = 43;	/* 44 is the minimum number for this constraint that still works */
//	int idxArrivalStart = 44;	/* 44 is the minimum number for this constraint that still works */
	int idxArrivalStart = 45;	/* 44 is the minimum number for this constraint that still works */
//	int idxArrivalStart = 50;
//	int idxArrivalStart = 90;
//	int idxArrivalStart = 300;
//	int idxArrivalEnd = 53;


//	for ( k=idxArrivalStart; k<idxArrivalEnd; ++k ) {
//	for ( k=idxArrivalStart; k<nI+1; ++k ) {
//	real_t bndRelax = 1.e-5;
	real_t bndRelax = 0.e-5;
	if ( idxArrivalStart+1 < nI ) {
		k = idxArrivalStart+0;
			xLow[k*nX+0] = xiRef[0] - bndRelax; //+1e-6;
			xUpp[k*nX+0] = xiRef[0] + bndRelax; //+1e-6;
		k = idxArrivalStart+1;
			xLow[k*nX+0] = xiRef[0] - bndRelax; //+1e-6;
			xUpp[k*nX+0] = xiRef[0] + bndRelax; //+1e-6;
	}


	/** set up a new mpcDUNES problem */
	mpcProblem_t mpcProblem;
	qpOptions_t qpOptions = qpDUNES_setupDefaultOptions();
	
	/** set qpDUNES options */
	qpOptions.maxIter    = 80;
//	qpOptions.maxIter   	 			 = 4;
	qpOptions.allowSuboptimalTermination = QPDUNES_FALSE;
	qpOptions.printLevel = 3;
	qpOptions.logLevel = QPDUNES_LOG_ALL_DATA;
//	qpOptions.logLevel = QPDUNES_LOG_ITERATIONS;
//	qpOptions.logLevel = QPDUNES_LOG_OFF;
//	qpOptions.printIterationTiming = QPDUNES_TRUE;
	qpOptions.stationarityTolerance = 1.e-6;
	qpOptions.equalityTolerance     = 2.221e-16;
//	qpOptions.regParam            = 1.e-4;
	qpOptions.regParam            = 1.e-8;
//	qpOptions.newtonHessDiagRegTolerance  = 1.e-4;
	qpOptions.newtonHessDiagRegTolerance  = 1.e-11;
//	qpOptions.newtonHessDiagRegTolerance  = 1.e-8;
	qpOptions.lsType			= QPDUNES_LS_ACCELERATED_GRADIENT_BISECTION_LS;
//	qpOptions.lsType			= QP42_LS_GRADIENT_BISECTION_LS;
//	qpOptions.lsType			= QP42_LS_GRID_LS;
//	qpOptions.lsType			= QP42_LS_ACCELERATED_GRID_LS;
//	qpOptions.lsType			= QP42_LS_BACKTRACKING_LS_WITH_AS_CHANGE;
//	qpOptions.lsType			= QPDUNES_LS_BACKTRACKING_LS;
//	qpOptions.lineSearchNbrGridPoints   		= 10;
	qpOptions.lineSearchNbrGridPoints   		= 5000;
	qpOptions.lineSearchStationarityTolerance = 1.e-3;
//	qpOptions.lineSearchStationarityTolerance = 1.e-6;
	qpOptions.lineSearchReductionFactor	= 0.1;
//	qpOptions.lineSearchReductionFactor	= 0.99;
//	qpOptions.lineSearchMaxStepSize	= 10.;
	qpOptions.lineSearchMaxStepSize	= 1.;
	qpOptions.maxNumLineSearchIterations            = 25;
	qpOptions.maxNumLineSearchRefinementIterations  = 60;
//	qpOptions.regType            = QP42_REG_NORMALIZED_LEVENBERG_MARQUARDT;
	qpOptions.regType            = QPDUNES_REG_SINGULAR_DIRECTIONS;
	qpOptions.nwtnHssnFacAlg	 = QPDUNES_NH_FAC_BAND_REVERSE;
	qpOptions.checkForInfeasibility	 = QPDUNES_TRUE;

	mpcDUNES_setup( &mpcProblem, nI, nX, nU, nD, &(qpOptions) );

	printf( "Solving double integrator [nI = %d, nX = %d, nU = %d]\n", nI, nX, nU );
	printf( "idxArrivalStart = %d\n", idxArrivalStart );
	printf( "Q = [\n" );
	for( i=0; i<nX; ++i ) {
		printf( "[\t" );
		for( j=0; j<nX; ++j ) {
			printf( "%.3e\t", Qorig[i*nX+j] );
//			printf( "%.3e\t", Qreg[i*nX+j] );
		}
		printf( "]\n" );
	}
	printf( "]\n" );
	printf( "R = [\n" );
	for( i=0; i<nU; ++i ) {
		printf( "[\t" );
		for( j=0; j<nU; ++j ) {
			printf( "%.3e\t", Rorig[i*nU+j] );
//			printf( "%.3e\t", Rreg[i*nU+j] );
		}
		printf( "]\n" );
	}
	printf( "]\n\n" );

// 	for ( i=0; i<100; ++i ) {
	/** initialize the MPC problem; note that matrices are factorized already now */
	statusFlag = mpcDUNES_initLtiSb_xu( &mpcProblem, Qorig, Rorig, S, Porig, A, B, c, xLow, xUpp, uLow, uUpp, xRef, uRef );
//	statusFlag = mpcDUNES_initLtiSb( &mpcProblem, Qreg , Rreg , 0, S, A, B, c, xLow, xUpp, uLow, uUpp, xRef, uRef );
	mpcProblem.isLTI = QPDUNES_FALSE;	/* since zLower and zUpper are not constant over the horizon, this is not LTI anymore */
	if (statusFlag != QPDUNES_OK) {
		printf( "mpcDUNES setup failed.\n" );
		return (int)statusFlag;
	}
	
	/** get feedback for a given initial QP value */
//	mpcDUNES_solve( &mpcProblem, x0 );
// 	}
	
//	qp42_printMatrixData( mpcProblem.xOpt, nI+1, nX, "xOpt:" );
//	qp42_printMatrixData( mpcProblem.uOpt, nI, nU, "uOpt:" );
	
//	printf( "presolve lambdaOpt:\n" );
//	for(k=0; k<nI*nX; ++k) {
//		printf( "l[%d]: %.2e\n", k, mpcProblem.qpData.lambda.data[k] );
//	}

//	printf( "\n\n ****************** POST SOLVE ******************\n\n");

	double t0 = getTime();
//	mpcDUNES_initLtiSb( &mpcProblem, Qorig, Rorig, S, A, B, c, xLow, xUpp, uLow, uUpp, xRef, uRef );
//	mpcDUNES_initLtiSb( &mpcProblem, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );
 	double t1 = getTime();
 	statusFlag = mpcDUNES_solve( &mpcProblem, x0 );
	double t2 = getTime();
	if (statusFlag != QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND) {
		printf( "mpcDUNES solve failed.\n" );
		return (int)statusFlag;
	}

//	printf( "lambdaOpt:\n" );
//	for(k=0; k<nI*nX; ++k) {
//		printf( "l*[%d]: %.2e\n", k, mpcProblem.qpData.lambda.data[k] );
//	}


	printf( "xOpt, uOpt:\n" );
//	printf( "xOpt, uOpt:\n[" );
	for(k=0; k<nI; ++k) {
		printf( "[%3d]:\t x= [% .3e  % .3e]\t u= [% .3e]\n", k, mpcProblem.xOpt[k*nX+0], mpcProblem.xOpt[k*nX+1], mpcProblem.uOpt[k*nU+0] );
//		printf( "[% .3e  % .3e  % .3e]\n", mpcProblem.xOpt[k*nX+0], mpcProblem.xOpt[k*nX+1], mpcProblem.uOpt[k*nU+0] );
	}
	printf( "[%3d]:\t x= [% .3e  % .3e]\n\n", nI, mpcProblem.xOpt[nI*nX+0], mpcProblem.xOpt[nI*nX+1] );
//	printf( "[% .3e  % .3e  % .3e]]\n\n", nI, mpcProblem.xOpt[nI*nX+0], mpcProblem.xOpt[nI*nX+1] );


	mpcDUNES_cleanup( &mpcProblem );

	printf( "Preparation time:       %.2lf ms\n", 1e3*(t1-t0) );
	printf( "Solution time:          %.2lf ms\n", 1e3*(t2-t1) );
	printf( "Total computation time: %.2lf ms\n", 1e3*(t2-t0) );

	return 0;
}


/*
 *	end of file
 */
