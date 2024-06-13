/*
 *	This file is part of qpDUNES.
 *
 *	qpDUNES -- A QP solver applying a DUal NEwton Strategy.
 *	Copyright (C) 2012 by Janick Frasch, Hans Joachim Ferreau et al. 
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
 *	\file examples/example2_affine.c
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2014
 *
 *	Example for NMPC coupling with general path constraints
 *
 *	In the context of NMPC qpDUNES solves a problem
 *
 *	min  sum_{k=0..nI} z_k'*H_k*z_k + g_k'*z_k
 *	s.t. x_{k+1} = C_k * z_k + c_k				for k=0..nI-1
 *	     dLow <= D * z_k <= dUpp				for k=0..nI
 *	     zLow <= z_k     <= zUpp				for k=0..nI
 *
 *	where x_k is implicitly defined by z_k = [ x_k  u_k ] as the first nX variables of z_k
 *
 *	It holds
 *	z_k  \in R^nZ  for k=0..nI-1
 *	z_nI \in R*nX
 *
 *	nX < nZ
 *	nU = nZ - nX
 *
 *
 *	USEFUL OPTIONS:
 *	qpOptions.logLevel = QPDUNES_LOG_OFF;	// switches off logging completely (can decrease memory consumption significantly)
 *
 *	COMPILER FLAGS:
 *  -D__SUPPRESS_ALL_WARNINGS__				// suppress warnings
 *  -D__SUPPRESS_ALL_OUTPUT__				// do not print anything at all
 *  -U__MEASURE_TIMINGS__					// exclude code for detailed runtime profiling in qpDUNES
 *  -U__USE_ASSERTS__						// switch some safety checks off
 *
 */


#include <qpDUNES.h>


#define INFTY 1.0e12
#define __PRINTTRAJECTORY__
#undef __PRINTTRAJECTORY__
#define __USE_AFFINE_CONSTRAINTS__
//#undef __USE_AFFINE_CONSTRAINTS__


int main( )
{
	return_t statusFlag;

	int iter, ii, kk;

	/** define problem dimensions */
	const unsigned int nI = 3;			/* number of control intervals */
	const int nX = 2;					/* number of states */
	const int nU = 1;					/* number of controls */
	const unsigned int nZ = nX+nU;		/* number of stage variables */
	unsigned int nD[nI+1];  			/* number of constraints */
	#ifndef __USE_AFFINE_CONSTRAINTS__
		int nDkk = 0;
	#else
		int nDkk = 1;
	#endif
	for ( kk=0; kk<nI+1; ++kk ) {
		nD[kk] = nDkk;
	}


	/** define problem data */
	const double H[] =
		{	1.0e3, 	0.0, 	0.0,
			0.0, 	1.0, 	0.0,
			0.0, 	0.0,	1.0,

			1.0e3, 	0.0, 	0.0,
			0.0, 	1.0, 	0.0,
			0.0, 	0.0,	1.0,

			1.0e3, 	0.0, 	0.0,
			0.0, 	1.0, 	0.0,
			0.0, 	0.0,	1.0,

			1.0e3, 	0.0,
			0.0, 	1.0
		};

	const double* g = 0;


	const double C[] =
		{	
			1.0, 0.01,   0.0,
			0.0, 1.0,    0.01,

			1.0, 0.01,   0.0,
			0.0, 1.0,    0.01,

			1.0, 0.01,   0.0,
			0.0, 1.0,    0.01
		};
	
	const double c[] =
		{	0.0,
			0.0,

			0.0,
			0.0,

			0.0,
			0.0
		};


	const double zLow[] =
		{	-1.9, -3.0, -30.0,
			-1.9, -3.0, -30.0,
			-1.9, -3.0, -30.0,
			-1.9, -3.0
		};
	const double zUpp[] =
		{	 1.9,  3.0,  30.0,
			 1.9,  3.0,  30.0,
			 1.9,  3.0,  30.0,
			 1.9,  3.0
		};


	const double D[] =
		{	0.0, 	5.0, 	1.0,		/* some random constraint limiting the acceleration for high velocities */

			0.0, 	5.0, 	1.0,

			0.0, 	5.0, 	1.0,

			0.0, 	0.0
		};
	const double dLow[] =
//		{	 5.e0,
//			 5.e0,
//			 5.e0,
//			-INFTY
//		};
		{	 0.e0,
			 0.e0,
			 0.e0,
			 0.e0
		};
//		{	-INFTY,
//			-INFTY,
//			-INFTY,
//			-INFTY
//		};
	const double dUpp[] =
		{	 0.e0,
			 0.e0,
			 0.e0,
			 0.e0
		};
//		{	 5.e5,
//			 5.e5,
//			 5.e5,
//			INFTY
//		};


	/** define simulation environment */
	const unsigned int nSteps = 1;	/* number of simulation steps */
//	const unsigned int nSteps = 20;	/* number of simulation steps */

	double x0[] = { -1.0, 0.0 };	/* initial value */
	double z0Low[nX+nU];			/* auxiliary variables */
	double z0Upp[nX+nU];

	double zOpt[nI*nZ+nX];			/* primal solution */
	double lambdaOpt[nI*nX];		/* dual solution */
	double muOpt[2*nI*(nZ+nDkk)+2*(nX+nDkk)];


	double zLog[nSteps*nZ];			/* log for trajectory */



	/** (1) set up a new qpDUNES problem */
	qpData_t qpData;

	/** (2) set qpDUNES options */
	qpOptions_t qpOptions = qpDUNES_setupDefaultOptions();
	qpOptions.maxIter    = 10;
	qpOptions.printLevel = 4;
//	qpOptions.printLevel = 3;
	qpOptions.stationarityTolerance = 1.e-6;
	qpOptions.regParam            = 1.e-8;
	qpOptions.newtonHessDiagRegTolerance  = 1.e-8;
	#ifdef __USE_AFFINE_CONSTRAINTS__
		qpOptions.lsType			= QPDUNES_LS_HOMOTOPY_GRID_SEARCH;
	#else
		qpOptions.lsType			= QPDUNES_LS_ACCELERATED_GRADIENT_BISECTION_LS;
		qpOptions.lineSearchReductionFactor	= 0.1;
		qpOptions.maxNumLineSearchRefinementIterations  = 150;
	#endif
//	qpOptions.lsType			= QPDUNES_LS_BACKTRACKING_LS_WITH_AS_CHANGE;
//	qpOptions.lsType			= QPDUNES_LS_BACKTRACKING_LS;
	qpOptions.lineSearchReductionFactor	= 0.7;
	qpOptions.lineSearchMaxStepSize	= 1.;
	qpOptions.maxNumLineSearchIterations            = 250;
	qpOptions.regType            = QPDUNES_REG_SINGULAR_DIRECTIONS;
//	qpOptions.nwtnHssnFacAlg     = QPDUNES_NH_FAC_BAND_FORWARD;


	/** (3) allocate data for qpDUNES and set options */
	qpDUNES_setup( &qpData, nI, nX, nU, nD, &(qpOptions) );
	

	/** (4) set sparsity of primal Hessian and local constraint matrix */
	for ( kk=0; kk<nI+1; ++kk ) {
		qpData.intervals[kk]->H.sparsityType = QPDUNES_DIAGONAL;
		qpData.intervals[kk]->D.sparsityType = QPDUNES_DENSE;
	}
	
	/** (5) initial MPC data setup: components not given here are set to zero (if applicable)
	 *      instead of passing g, D, zLow, zUpp, one can also just pass NULL pointers (0) */
	#ifndef __USE_AFFINE_CONSTRAINTS__
	statusFlag = qpDUNES_init( &qpData, H, g, C, c, zLow,zUpp, 0,0,0 );
	#else
	statusFlag = qpDUNES_init( &qpData, H, g, C, c, zLow,zUpp, D,dLow,dUpp );
//	x0[0] = -9.99910049469108e-01;
//	x0[1] = 6.99476363268033e-03;
//	x0[0] = -1.00000000000000e+00;
//	x0[1] = 2.99873067552648e-03;

//	for ( ii=0; ii<nX; ++ii ) {
//		z0Low[ii] = x0[ii];
//		z0Upp[ii] = x0[ii];
//	}
//	for ( ii=nX; ii<nX+nU; ++ii ) {
//		z0Low[ii] = zLow[ii];
//		z0Upp[ii] = zUpp[ii];
//	}
//	qpDUNES_printf("\nHEEEREEEE\n");
//	qpDUNES_printf("\n---- now 0 fix\n");
//	statusFlag = qpDUNES_updateIntervalData( &qpData, qpData.intervals[2], 0, 0, 0, 0, z0Low,z0Upp, 0,0,0, 0 );	/* 0 fix */
//	z0Low[1] = 0.01;
//	z0Upp[1] = 0.01;
//	qpDUNES_printf("\n---- now 0, 1 fix\n");
//	statusFlag = qpDUNES_updateIntervalData( &qpData, qpData.intervals[2], 0, 0, 0, 0, z0Low,z0Upp, 0,0,0, 0 );	/* 0,1 fix */
//	for ( ii=0; ii<nZ; ++ii ) {
//		z0Low[ii] = zLow[ii];
//		z0Upp[ii] = zUpp[ii];
//	}
//	qpDUNES_printf("\n---- now all free\n");
//	statusFlag = qpDUNES_updateIntervalData( &qpData, qpData.intervals[2], 0, 0, 0, 0, z0Low,z0Upp, 0,0,0, 0 );	/* all free */
////	z0Low[0] = -1.0;
////	z0Upp[0] = -1.0;
////	z0Low[1] = 0.01;
////	z0Upp[1] = 0.01;
////	qpDUNES_printf("\n---- now 0, 1 fix\n");
////	statusFlag = qpDUNES_updateIntervalData( &qpData, qpData.intervals[2], 0, 0, 0, 0, z0Low,z0Upp, 0,0,0, 0 );	/* 0,1 fix */
//	for ( ii=0; ii<nZ; ++ii ) {
//		z0Low[ii] = zLow[ii];
//		z0Upp[ii] = zUpp[ii];
//	}
//	z0Low[1] = 0.01;
//	z0Upp[1] = 0.01;
//	qpDUNES_printf("\n---- now 1 fix\n");
//	statusFlag = qpDUNES_updateIntervalData( &qpData, qpData.intervals[2], 0, 0, 0, 0, z0Low,z0Upp, 0,0,0, 0 );	/* 1 fix */
//	for ( ii=0; ii<nZ; ++ii ) {
//		z0Low[ii] = zLow[ii];
//		z0Upp[ii] = zUpp[ii];
//	}
//	qpDUNES_printf("\n---- now all free\n");
//	statusFlag = qpDUNES_updateIntervalData( &qpData, qpData.intervals[2], 0, 0, 0, 0, z0Low,z0Upp, 0,0,0, 0 );	/* all free */

//	statusFlag = qpDUNES_updateIntervalData( &qpData, qpData.intervals[0], 0, 0, 0, 0, z0Low,z0Upp, 0,0,0, 0 );
//	statusFlag = qpDUNES_solve( &qpData );
//	statusFlag = qpDUNES_solve( &qpData );
//	statusFlag = qpDUNES_solve( &qpData );
//	qpDUNES_printMatrixData( qpData.lambda.data, nI*nX, 1, "lambda at end:" );
//	qpDUNES_printMatrixData( qpData.intervals[1]->q.data, nZ, 1, "q[1] at end:" );
//	qpDUNES_printMatrixData( qpData.intervals[1]->qpSolverQpoases.qFullStep.data, nZ, 1, "qFS[1] at end:" );
//	qpDUNES_printMatrixData( zLow, nI*nZ+nX, 1, "zLow passed:" );
//	qpDUNES_printMatrixData( zUpp, nI*nZ+nX, 1, "zUpp passed:" );
//	qpDUNES_printMatrixData( D, nI, nZ, "D passed:" );
//	qpDUNES_printMatrixData( &(D[nI*nZ]), 1, nX, "D passed:" );
//	qpDUNES_printMatrixData( dLow, nI+1, 1, "dLow passed:" );
//	qpDUNES_printMatrixData( dUpp, nI+1, 1, "dUpp passed:" );
//	statusFlag = qpDUNES_updateData( &qpData, H, g, C, c, zLow,zUpp, D,dLow,dUpp );
//	statusFlag = qpDUNES_updateIntervalData( &qpData, qpData.intervals[0], 0, 0, 0, 0, z0Low,z0Upp, 0,0,0, 0 );
//	statusFlag = qpDUNES_solve( &qpData );
//	statusFlag = 22;
	#endif
	if (statusFlag != QPDUNES_OK) {
		printf( "Data init failed.\n" );
		return (int)statusFlag;
	}
	

	/** MAIN MPC SIMULATION LOOP */
 	for ( iter=0; iter<nSteps; ++iter ) {
 		/** (1) embed current initial value */
		for ( ii=0; ii<nX; ++ii ) {
			z0Low[ii] = x0[ii];
			z0Upp[ii] = x0[ii];
		}
		for ( ii=nX; ii<nX+nU; ++ii ) {
			z0Low[ii] = zLow[ii];
			z0Upp[ii] = zUpp[ii];
		}
		statusFlag = qpDUNES_updateIntervalData( &qpData, qpData.intervals[0], 0, 0, 0, 0, z0Low,z0Upp, 0,0,0, 0 );
		if (statusFlag != QPDUNES_OK) {
			printf( "Initial value embedding failed.\n" );
			return (int)statusFlag;
		}


		/** (2) solve QP */
		statusFlag = qpDUNES_solve( &qpData );
		if (statusFlag != QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND) {
			printf( "QP solution %d failed.\n", iter );
			return (int)statusFlag;
		}
		
		// up to here in fdb step


		/** (3) obtain primal and dual optimal solution */
		qpDUNES_getPrimalSol( &qpData, zOpt );
		qpDUNES_getDualSol( &qpData, lambdaOpt, muOpt );
//		qpDUNES_printMatrixData( zOpt, nI, nZ, "zOpt, first intervals");
//		qpDUNES_printMatrixData( lambdaOpt, nI, nX, "lambdaOpt");
		qpDUNES_printMatrixData( muOpt, 1, 2*(nI+1)*(nZ+nDkk)-2*nU, "muOpt");
		/// ...
		

 		/** (4) prepare QP for next solution */
		/** optional: data shift */
		qpDUNES_shiftLambda( &qpData );			/* shift multipliers */
		qpDUNES_shiftIntervals( &qpData );		/* shift intervals (particularly important when using qpOASES for underlying local QPs) */
		
//		qpDUNES_printMatrixData( qpData.lambda.data, nI*nX, 1, "lambda after shift:" );
		
		/** data update */
		statusFlag = qpDUNES_updateData( &qpData, H, g, C, c, zLow,zUpp, D,dLow,dUpp );		/* data update: components not given here keep their previous value */
		if (statusFlag != QPDUNES_OK) {
			printf( "Data update failed.\n" );
			return (int)statusFlag;
		}
		

		/** (5) simulate next initial value */
		for (ii=0; ii<nX; ++ii) {
			/// x0 = ...
			x0[ii] = zOpt[1*nZ+ii];
 		}


		// optional: logging
		for (ii=0; ii<nZ; ++ii) {
			zLog[iter*nZ+ii] = zOpt[ii];
 		}
	}


 	// optional: printing
	#ifdef __PRINTTRAJECTORY__
 	printf("\nPROCESS TRAJECTORY:\n");
 	for (iter=0; iter<nSteps; ++iter) {
 		printf("\nt%02d:\t", iter);
 		for (ii=0; ii<nZ; ++ii) {
 			printf("%+.3e\t", zLog[iter*nZ+ii]);
 		}
	}
 	printf("\n\n");
	#endif

 	// optional: printing
	#ifndef __PRINTTRAJECTORY__
	printf("\nLast zOpt:\n");
	for (iter=0; iter<nI; ++iter) {
		printf("\nt%02d:\t", iter);
		for (ii=0; ii<nZ; ++ii) {
			printf("%+.3e\t", zOpt[iter*nZ+ii]);
		}
	}
	printf("\n\n");
	#endif



	/** mandatory: cleanup of allocated data */
	qpDUNES_cleanup( &qpData );


	return 0;
}


/*
 *	end of file
 */
