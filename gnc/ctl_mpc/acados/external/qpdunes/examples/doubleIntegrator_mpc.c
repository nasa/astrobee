/*
 *	This file is part of qpDUNES.
 *
 *	qpDUNES -- A DUal NEwton Strategy for convex quadratic programming.
 *	Copyright (C) 2012 by Janick Frasch, Hans Joachim Ferreau, et al.
 *	All rights reserved.
 *
 *	qpDUNES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpDUNES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpDUNES; if not, write to the Free Software
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
#include <sys/time.h>
#include <stdio.h>


#define INFTY 1.0e12

#define ENERGY_OPT


int main( )
{
	int iter;
	int i, jj, k;
	
	return_t statusFlag;

	/** number of MPC simulation steps */
	const unsigned int nSteps = 20;

	/** noise settings */
	double maxNoise[] = { 1.e-2, 0.e-4 };
//	double maxNoise[] = { 0., 0. };
	srand( (unsigned)time( 0 ) );			/* init pseudo-random seed */

	/** timings */
	double t, tMeas;
	double tPrepTtl = 0.;
	double tSolTtl = 0.;
	double tPrepMax = 0.;
	double tSolMax = 0.;


	/** problem dimensions */
	const unsigned int nI = 200;		/* number of control intervals */
	const int nX = 2;			/* number of states */
	const int nU = 1;			/* number of controls */
	const unsigned int nZ = nX+nU;		/* number of stage variables */
	unsigned int* nD = 0;	  			/* number of constraints */


	/** problem data */
	double dt = 0.01;	/* discretization sampling time 10ms, needed for constraints */

	
	const double H[] =
		{	1.001e-4, 	0.0, 		0.0,
			0.0, 		1.001e-4, 	0.0,
			0.0, 		0.0,		1.0
		};

	const double P[] =
		{	1.001e-4, 	0.0,
			0.0, 		1.001e-4
		};
	

	const double C[] =
		{	
			1.0, 1.0*dt, 0.0,
			0.0, 1.0,    1.0*dt
		};
	
	const double c[] =
		{	0.0,
			0.0
		};
			
			
	const double ziLow[3] =
		{	-1.9, -3.0, -30.0	};
	
	const double ziUpp[3] =
		{	 1.9,  3.0,  30.0	};


	double ziRef[3] =
		{ -0.0, 0.0, 0.0 };
	

	/** build up bounds and reference vectors */
	double zLow[nZ*nI+nX];
	double zUpp[nZ*nI+nX];
	double zRef[nZ*nI+nX];
	for ( k=0; k<nI; ++k ) {
		for ( i=0; i<nZ; ++i ) {
			zLow[k*nZ+i] = ziLow[i];
			zUpp[k*nZ+i] = ziUpp[i];
			zRef[k*nZ+i] = ziRef[i];
		}
	}
	for ( i=0; i<nX; ++i ) {
		zLow[nI*nZ+i] = ziLow[i];
		zUpp[nI*nZ+i] = ziUpp[i];
		zRef[nI*nZ+i] = ziRef[i];
	}

	/* arrival constraints */
	int idxArrivalStart = 44;	/* 44 is the minimum number for this constraint that still works */
//	int idxArrivalStart = 39;
	k = idxArrivalStart+0;
		zLow[k*nZ+0] = ziRef[0];
		zUpp[k*nZ+0] = ziRef[0];
	k = idxArrivalStart+1;
		zLow[k*nZ+0] = ziRef[0];
		zUpp[k*nZ+0] = ziRef[0];


	/** initial value */
	double x0[] =
		{ -1.0, 0.0 };
//		{ -9.699999739663e-1,   1.500000000099e0 };


	/** simulation variables */
	double xLog[nX*(nSteps+1)];
	double uLog[nX*nSteps];


	/** set up a new mpcDUNES problem */
	printf( "Solving double integrator MPC problem [nI = %d, nX = %d, nU = %d]\n", nI, nX, nU );
	mpcProblem_t mpcProblem;
	qpOptions_t qpOptions = qpDUNES_setupDefaultOptions();
	
	/** set qpDUNES options */
//	qpOptions.maxIter    = 30;
	qpOptions.maxIter   	 			 = 4;
	qpOptions.allowSuboptimalTermination = QPDUNES_TRUE;
	qpOptions.printLevel = 3;
	qpOptions.stationarityTolerance = 1.e-6;
	qpOptions.regParam            = 1.e-6;
	qpOptions.newtonHessDiagRegTolerance  = 1.e-9;
//	qpOptions.lsType			= QPDUNES_LS_BACKTRACKING_LS;
	qpOptions.lsType			= QPDUNES_LS_ACCELERATED_GRADIENT_BISECTION_LS;
	qpOptions.lineSearchStationarityTolerance = 1.e-7;
	qpOptions.lineSearchReductionFactor	= 0.1;
	qpOptions.lineSearchMaxStepSize	= 1.;
	qpOptions.maxNumLineSearchIterations            = 15;
//	qpOptions.maxNumLineSearchIterations            = 25;
	qpOptions.maxNumLineSearchRefinementIterations  = 300;
	qpOptions.regType            = QPDUNES_REG_SINGULAR_DIRECTIONS;
//	qpOptions.nwtnHssnFacAlg     = QPDUNES_NH_FAC_BAND_FORWARD;


	/** setup qpDUNES internal data */
	mpcDUNES_setup( &mpcProblem, nI, nX, nU, nD, &(qpOptions) );
	t = getTime();
	statusFlag = mpcDUNES_initLtiSb( &mpcProblem,  H, P, 0,  C, c,  zLow, zUpp,  zRef );	/** initialize MPC problem; note that matrices are factorized here */
	tMeas = getTime() - t;
	tPrepTtl += tMeas;
	if (tMeas > tPrepMax)	tPrepMax = tMeas;
	

	/** MAIN MPC SIMULATION LOOP */
 	for ( iter=0; iter<nSteps; ++iter ) {
 		/** solve QP for current initial value */
		t = getTime();
		mpcDUNES_solve( &mpcProblem, x0 );
		tMeas = getTime() - t;
		tSolTtl += tMeas;
		if (tMeas > tSolMax)	tSolMax = tMeas;



		/** save x0, u0 */
 		for (i=0; i<nX; ++i) {
			xLog[iter*nX+i] = x0[i];
 		}
 		for (i=0; i<nU; ++i) {
			uLog[iter*nU+i] = mpcProblem.uOpt[0*nU+i];
		}

 		/** prepare QP for next solution */
		t = getTime();
		/* shift intervals... now done automatically in mpcDUNES_solve */
//		qpDUNES_shiftIntervals( &(mpcProblem.qpData) );
//		qpDUNES_shiftLambda( &(mpcProblem.qpData) );
//		for (i=0; i<nI*nX; ++i) 	mpcProblem.qpData.lambda.data[i] = 0.;

		/* put new data in second but last interval: * LTI: from interval 0; * LTV: new */
//		statusFlag = qp42_updateInterval( &(mpcProblem.qpData), mpcProblem.qpData.intervals[nI-1], H_, 0, C_, c_, 0, dLow_, dUpp_, cholH );
		statusFlag = qpDUNES_updateIntervalData( &(mpcProblem.qpData), mpcProblem.qpData.intervals[nI-1], 0, 0, 0, 0, ziLow,ziUpp, 0, 0,0, 0 );	/* H, C, c do not need to be updated b/c LTI */
		/* put new data in last interval */
		statusFlag += qpDUNES_updateIntervalData( &(mpcProblem.qpData), mpcProblem.qpData.intervals[nI], 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 );		/* no new P matrix, bounds, etc. given */
		if (statusFlag != QPDUNES_OK) {
			printf( "data update failed.\n" );
			return (int)statusFlag;
		}
		tMeas = getTime() - t;
		tPrepTtl += tMeas;
		if (tMeas > tPrepMax)	tPrepMax = tMeas;

		/** simulate next initial value */
		for (i=0; i<nX; ++i) {
//			x0[i] = mpcProblem.xOpt[1*nX+i] + maxNoise[i] * 2.*( (double)rand()/RAND_MAX -1 );
//			x0[i] = mpcProblem.xOpt[1*nX+i] + maxNoise[i] * (double)rand()/RAND_MAX;
//			x0[i] = mpcProblem.xOpt[1*nX+i] + maxNoise[i] * .5;
			x0[i] = c[i]  + maxNoise[i] * .5;
			for (jj=0; jj<nX; ++jj) {
				x0[i] += C[i*nZ+jj] * mpcProblem.xOpt[jj];
			}
			for (jj=0; jj<nU; ++jj) {
				x0[i] += C[i*nZ+nX+jj] * mpcProblem.uOpt[jj];
			}
			printf( "x0: %+.5e  --  with noise: %+.5e\n", mpcProblem.xOpt[1*nX+i], x0[i] );
 		}
		printf("--- we are completely done with problem %d ---\n", iter);
	}

 	/* save last state of simulation */
 	for (i=0; i<nX; ++i) {
		xLog[nSteps*nX+i] = x0[i];
	}

	/** print information */
	printf( "xLog, uLog:\n[" );
	for(k=0; k<nSteps; ++k) {
		printf( "[% .12e  % .12e  % .12e]\n", xLog[k*nX+0], xLog[k*nX+1], uLog[k*nU+0] );
	}
	printf( "[% .12e  % .12e  % .12e]]\n\n", xLog[nSteps*nX+0], xLog[nSteps*nX+1], 0. );
//	printf( "xOpt, uOpt:\n[" );
//	for(k=0; k<nI; ++k) {
//		printf( "[% .3e  % .3e  % .3e]\n", mpcProblem.xOpt[k*nX+0], mpcProblem.xOpt[k*nX+1], mpcProblem.uOpt[k*nU+0] );
//	}
//	printf( "[% .3e  % .3e  % .3e]]\n\n", mpcProblem.xOpt[nI*nX+0], mpcProblem.xOpt[nI*nX+1], 0. );


	printf( "Computation times    Maximum     Average     Total  \n" );
	printf( "-----------------    --------    --------    --------\n" );
	printf( "Preparation          %5.2lf ms    %5.2lf ms    %5.2lf ms\n", 1e3*tPrepMax, 1e3*tPrepTtl/(nSteps+1), 1e3*tPrepTtl );
	printf( "Solution             %5.2lf ms    %5.2lf ms    %5.2lf ms\n", 1e3*tSolMax, 1e3*tSolTtl/nSteps, 1e3*tSolTtl );


	/** cleanup of allocated data */
	mpcDUNES_cleanup( &mpcProblem );


	return 0;
}


/*
 *	end of file
 */
