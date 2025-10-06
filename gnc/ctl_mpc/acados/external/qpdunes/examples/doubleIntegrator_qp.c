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
	const unsigned int nZ = nX+nU;	/* number of QP stage variables */
	unsigned int* nD = 0;  			/* number of constraints */
//	unsigned int nD[nI+1];  		/* number of constraints */
//	for ( k=0; k<nI+1; ++k ) {
//		nD[k] = 0;
//	}
	

	double dt = 0.01;	/* discretization sampling time 10ms, needed for constraints */
// 	double dt = 1.0; // for now....
	
	double x0[2] = 
		{ -1.0, 0.0 };
//		{ -0.00, 0.0 };
	
	
	double Hi[3*3] =
		{
			1.0e-8, 0.0, 0.0,
			0.0, 1.0e-8, 0.0,
			0.0, 0.0, 1.0e0
		};

	double Ci[2*3] =
		{	
			1.0, 1.0*dt, 0.0,
			0.0, 1.0, 1.0*dt
		};
	
	double ci[2] =
		{	0.0,
			0.0
		};

	/** stack full QP data */
	double H[nI*nZ*nZ+nX*nX];
	double C[nI*nX*nZ];
	double c[nI*nX];
	for ( k=0; k<nI; ++k ) {
		for( i=0; i<nZ*nZ; ++i )	{
			H[k*nZ*nZ+i] = Hi[i];
		}
		for( i=0; i<nX*nZ; ++i )	{
			C[k*nX*nZ+i] = Ci[i];
		}
		for( i=0; i<nX; ++i )	{
			c[k*nX+i] = ci[i];
		}
	}
	for ( i=0; i<nX; ++i ) {
		for( j=0; j<nX; ++j )	{
			H[nI*nZ*nZ+i*nX+j] = Hi[i*nZ+j];
		}
	}
			


	/** build up bounds and reference vectors */
	double ziLow[3] =
		{	-1.9, -3.0, -30.0	};
	
	double ziUpp[3] =
		{	 1.9,  3.0,  30.0	};


	double zLow[nX*(nI+1)+nU*nI];
	double zUpp[nX*(nI+1)+nU*nI];
	for ( k=0; k<nI; ++k ) {
		for ( i=0; i<nZ; ++i ) {
			zLow[k*nZ+i] = ziLow[i];
			zUpp[k*nZ+i] = ziUpp[i];
		}
	}
	for ( i=0; i<nX; ++i ) {
		zLow[nI*nZ+i] = ziLow[i];
		zUpp[nI*nZ+i] = ziUpp[i];
	}


	/* arrival constraints */
	int idxArrivalStart = 45;	/* 44 is the minimum number for this constraint that still works */

	if ( idxArrivalStart+1 < nI ) {
		k = idxArrivalStart+0;
			zLow[k*nZ+0] = 0.0;
			zUpp[k*nZ+0] = 0.0;
		k = idxArrivalStart+1;
			zLow[k*nZ+0] = 0.0;
			zUpp[k*nZ+0] = 0.0;
	}


	/* initial value constraints */
	zLow[0] = x0[0];
	zLow[1] = x0[1];
	zUpp[0] = x0[0];
	zUpp[1] = x0[1];


	/** set up a new mpcDUNES problem */
	qpData_t qpData;
	qpOptions_t qpOptions = qpDUNES_setupDefaultOptions();
	
	/** set qpDUNES options */
//	qpOptions.maxIter    = 80;
	qpOptions.maxIter   	 			 = 4;
	qpOptions.allowSuboptimalTermination = QPDUNES_TRUE;
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

	qpDUNES_setup( &qpData, nI, nX, nU, nD, &(qpOptions) );

	printf( "Solving double integrator [nI = %d, nX = %d, nU = %d]\n", nI, nX, nU );

	/** initialize the QP problem; note that matrices are factorized already now */
	statusFlag = qpDUNES_init( &qpData, H, 0, C, c, zLow,zUpp, 0,0,0 );
	if (statusFlag != QPDUNES_OK) {
		printf( "qpDUNES setup failed.\n" );
		return (int)statusFlag;
	}
	

	double t0 = getTime();
 	double t1 = getTime();
 	statusFlag = qpDUNES_solve( &qpData );
 	statusFlag = qpDUNES_solve( &qpData );
// 	statusFlag = mpcDUNES_solve( &mpcProblem, x0 );
	double t2 = getTime();
	if (statusFlag != QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND) {
		printf( "qpDUNES solve failed.\n" );
		return (int)statusFlag;
	}

//	printf( "lambdaOpt:\n" );
//	for(k=0; k<nI*nX; ++k) {
//		printf( "l*[%d]: %.2e\n", k, mpcProblem.qpData.lambda.data[k] );
//	}


//	printf( "xOpt, uOpt:\n" );
////	printf( "xOpt, uOpt:\n[" );
//	for(k=0; k<nI; ++k) {
//		printf( "[%3d]:\t x= [% .3e  % .3e]\t u= [% .3e]\n", k, qpData.xOpt[k*nX+0], mpcProblem.xOpt[k*nX+1], mpcProblem.uOpt[k*nU+0] );
////		printf( "[% .3e  % .3e  % .3e]\n", mpcProblem.xOpt[k*nX+0], mpcProblem.xOpt[k*nX+1], mpcProblem.uOpt[k*nU+0] );
//	}
//	printf( "[%3d]:\t x= [% .3e  % .3e]\n\n", nI, mpcProblem.xOpt[nI*nX+0], mpcProblem.xOpt[nI*nX+1] );
////	printf( "[% .3e  % .3e  % .3e]]\n\n", nI, mpcProblem.xOpt[nI*nX+0], mpcProblem.xOpt[nI*nX+1] );


	qpDUNES_cleanup( &qpData );

	printf( "Preparation time:       %.2lf ms\n", 1e3*(t1-t0) );
	printf( "Solution time:          %.2lf ms\n", 1e3*(t2-t1) );
	printf( "Total computation time: %.2lf ms\n", 1e3*(t2-t0) );

	return 0;
}


/*
 *	end of file
 */
