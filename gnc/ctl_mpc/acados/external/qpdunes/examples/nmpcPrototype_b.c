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
 *	\file examples/nmpcPrototype.c
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2012
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
#undef __PRINTTRAJECTORY__


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
	for ( kk=0; kk<nI+1; ++kk ) {
//		nD[kk] = 0;
		nD[kk] = 1;
	}
//	nD[nI] = 0;


	/** define problem data */
	const double H[] =
		{	1.0, 	0.0, 	0.0,
			0.0, 	1.0, 	0.0,
			0.0, 	0.0,	1.0,

			1.0, 	0.0, 	0.0,
			0.0, 	1.0, 	0.0,
			0.0, 	0.0,	1.0,

			1.0, 	0.0, 	0.0,
			0.0, 	1.0, 	0.0,
			0.0, 	0.0,	1.0,

			1.0, 	0.0,
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

			0.0, 	5.0
		};
	const double dLow[] =
		{	-INFTY,
			-INFTY,
			-INFTY,
			-INFTY
		};
	const double dUpp[] =
		{	 5.e0,
			 5.e0,
			 5.e0,
			 5.e0
		};


	/** define simulation environment */
	const unsigned int nSteps = 100;	/* number of simulation steps */
//	const unsigned int nSteps = 2;	/* number of simulation steps */

	double x0[] = { -1.0, 0.0 };	/* initial value */
	double z0Low[nX+nU];			/* auxiliary variables */
	double z0Upp[nX+nU];

	double zOpt[nI*nZ+nX];			/* primal solution */
	double lambdaOpt[nI*nX];		/* dual solution */
	double muOpt[2*nI*nZ+2*nX];


	double zLog[nSteps*nZ];			/* log for trajectory */



	/** (1) set up a new qpDUNES problem */
	qpData_t qpData;

	/** (2) set qpDUNES options */
	qpOptions_t qpOptions = qpDUNES_setupDefaultOptions();
	qpOptions.maxIter    = 100;
	qpOptions.printLevel = 3;
	qpOptions.stationarityTolerance = 1.e-6;
	qpOptions.regParam            = 1.e-8;
	qpOptions.newtonHessDiagRegTolerance  = 1.e-8;
//	qpOptions.lsType			= QPDUNES_LS_BACKTRACKING_LS;
//	qpOptions.lsType			= QPDUNES_LS_ACCELERATED_GRADIENT_BISECTION_LS;
	qpOptions.lsType			= QPDUNES_LS_HOMOTOPY_GRID_SEARCH;
	qpOptions.lineSearchReductionFactor	= 0.1;
	qpOptions.lineSearchMaxStepSize	= 1.;
	qpOptions.maxNumLineSearchIterations            = 25;
	qpOptions.maxNumLineSearchRefinementIterations  = 150;
	qpOptions.regType            = QPDUNES_REG_SINGULAR_DIRECTIONS;


	/** (3) allocate data for qpDUNES and set options */
	qpDUNES_setup( &qpData, nI, nX, nU, nD, &(qpOptions) );
	

	/** (4) set sparsity of primal Hessian and local constraint matrix */
	for ( kk=0; kk<nI+1; ++kk ) {
		qpData.intervals[kk]->H.sparsityType = QPDUNES_DIAGONAL;
//		qpData.intervals[kk]->D.sparsityType = QPDUNES_IDENTITY;
		qpData.intervals[kk]->D.sparsityType = QPDUNES_DENSE;
	}
	
	/** (5) initial MPC data setup: components not given here are set to zero (if applicable)
	 *      instead of passing g, D, zLow, zUpp, one can also just pass NULL pointers (0) */
	statusFlag = qpDUNES_init( &qpData, H, g, C, c, zLow,zUpp, D,dLow,dUpp );
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
		/// ...
		

 		/** (4) prepare QP for next solution */
		/** new interface: combined shift and update functionality for safety */
		/** mandatory: data update */
		/// H = ...
		/// g = ...
		/// C = ...
		/// c = ...
		/// zLow = ...
		/// zUpp = ...
		/** (4a) EITHER: MPC style update for RTIs, including shift	*/
//		statusFlag = qpDUNES_shiftAndUpdate( &qpData, H, g, C, c, zLow,zUpp, D,dLow,dUpp );		/* data update: components not given here keep their previous value */
		/** (4b) OR: SQP style update, excluding shift */
		statusFlag = qpDUNES_updateData( &qpData, H, g, C, c, zLow,zUpp, D,dLow,dUpp );			/* data update: components not given here keep their previous value */
		if (statusFlag != QPDUNES_OK) {
			printf( "Data update failed.\n" );
			return (int)statusFlag;
		}

//		/** OLD INTERFACE */
//		/** optional: data shift */
//		qpDUNES_shiftLambda( &qpData );			/* shift multipliers */
//		qpDUNES_shiftIntervals( &qpData );		/* shift intervals (particulary important when using qpOASES for underlying local QPs) */
//		/* WARNING: currently a shift needs to be followed by a data update.
//		 * A pure LTV shift may lead to data inconsistencies, but is not
//		 * prohibited yet. In a further version shift and data update
//		 * functionality might be combined for increased user friendliness
//		 */
//
//		statusFlag = qpDUNES_updateData( &qpData, H, g, C, c, zLow,zUpp, D,dLow,dUpp );		/* data update: components not given here keep their previous value */
//		if (statusFlag != QPDUNES_OK) {
//			printf( "Data update failed.\n" );
//			return (int)statusFlag;
//		}
		

		/** (5) simulate next initial value */
		for (ii=0; ii<nX; ++ii) {
			/// x0 = ...
			x0[ii] = zOpt[1*nZ+ii];
 		}


		// optional: logging
		for (ii=0; ii<nZ; ++ii) {
			zLog[iter*nZ+ii] = zOpt[ii];
 		}

		printf( "Done with QP %d.\n", iter );
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
	#endif
 	printf("\n\n");



	/** mandatory: cleanup of allocated data */
	qpDUNES_cleanup( &qpData );


	return 0;
}


/*
 *	end of file
 */
