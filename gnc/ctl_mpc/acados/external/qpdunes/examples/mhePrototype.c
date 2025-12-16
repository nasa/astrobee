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
 *	\file examples/mhePrototype.c
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2013
 *
 *	Example for MHE coupling with general path constraints
 *
 *	In the context of MHE qpDUNES solves a problem
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
 *	For MHE we assume that H_k and g_k are define in concordance with
 *		x_k := \xi_k - \eta^\xi_k
 *		u_k := \nu_k - \eta^\nu_k,
 *	where \xi and \nu are the actual system states and controls
 *	and \eta are the measurements. For an SQP scheme in the context of MHE
 *	this should be given naturally, as the primal variables typically are
 *	the \Delta variables to the true nonlinear variables anyways.
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
	for ( kk=0; kk<nI; ++kk ) {
		nD[kk] = 1;
	}
	nD[nI] = 0;


	/** define problem data */
	const double H[] =
		{	10.0, 	0.0, 	0.0,
			0.0, 	1.0, 	0.0,
			0.0, 	0.0,	1.0,

			10.0, 	0.0, 	0.0,
			0.0, 	1.0, 	0.0,
			0.0, 	0.0,	1.0,

			10.0, 	0.0, 	0.0,
			0.0, 	1.0, 	0.0,
			0.0, 	0.0,	1.0,

			10.0, 	0.0,
			0.0, 	1.0
		};
	
	const double g[] =
		{   0,
			0,
			0,

			0,
			0,
			0,

			0,
			0,
			0,

			0,
			0
		};


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

			0.0, 	5.0, 	1.0
		};
	const double dLow[] =
		{	-INFTY,
			-INFTY,
			-INFTY
		};
	const double dUpp[] =
		{	 5.e0,
			 5.e0,
			 5.e0
		};


	/** define simulation environment */
	const unsigned int nSteps = 40;	/* number of simulation steps */

	double g_nI[nX];				/* last gradient piece for embedding measurement */
	/* choose some dummy values for demonstration */
	for( ii=0; ii<nX; ++ii ) {
		g_nI[ii] = 0.05;
	}

	double zOpt[nI*nZ+nX];			/* primal solution */
	double lambdaOpt[nI*nX];		/* dual solution */
	double muOpt[2*nI*nZ+2*nX];


	double xLog[nSteps*nX];			/* log for trajectory */



	/** (1) set up a new qpDUNES problem */
	qpData_t qpData;


	/** (2) set qpDUNES options */
	qpOptions_t qpOptions = qpDUNES_setupDefaultOptions();
	qpOptions.maxIter    = 20;
	qpOptions.printLevel = 2;
	qpOptions.stationarityTolerance = 1.e-6;
	qpOptions.regParam            = 1.e-8;
	qpOptions.newtonHessDiagRegTolerance  = 1.e-8;
	qpOptions.lsType			= QPDUNES_LS_BACKTRACKING_LS;
//	qpOptions.lsType			= QPDUNES_LS_ACCELERATED_GRADIENT_BISECTION_LS;
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
		qpData.intervals[kk]->D.sparsityType = QPDUNES_IDENTITY;
	}
	
	/** (5) initial MHE data setup: components not given here are set to zero (if applicable)
	 *      instead of passing g, D, zLow, zUpp, one can also just pass NULL pointers (0) */
	statusFlag = qpDUNES_init( &qpData, H, g, C, c, zLow,zUpp, D,dLow,dUpp );
	if (statusFlag != QPDUNES_OK) {
		printf( "Data init failed.\n" );
		return (int)statusFlag;
	}
	

	/** MAIN MPC SIMULATION LOOP */
 	for ( iter=0; iter<nSteps; ++iter ) {
 		/** (1) embed linear term on last interval (linear compensation for new measurement) */
 		for ( ii=0; ii<nX; ++ii ) {
 			/* do the linear measurement embedding here; qpDUNES needs the consistent
			 * linear QP term on the last interval
			 */
// 			g_nI[ii] = ... * eta_nI;
 			g_nI[ii] = g_nI[ii];
 		}
		statusFlag = qpDUNES_updateIntervalData( &qpData, qpData.intervals[nI], 0, g_nI, 0, 0, 0,0, 0,0,0, 0 );
		if (statusFlag != QPDUNES_OK) {
			printf( "Linear measurement embedding failed.\n" );
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
		/** optional: data shift */
		qpDUNES_shiftLambda( &qpData );			/* shift multipliers */
		qpDUNES_shiftIntervals( &qpData );		/* shift intervals (particulary important when using qpOASES for underlying local QPs) */
		
		printf("after shift:\n");
		
		/** mandatory: data update */
		/// H = ...
		/// g = ...
		/// C = ...
		/// c = ...
		/// zLow = ...
		/// zUpp = ...
		statusFlag = qpDUNES_updateData( &qpData, H, g, C, c, zLow,zUpp, D,dLow,dUpp );		/* data update: components not given here keep their previous value */
		if (statusFlag != QPDUNES_OK) {
			printf( "Data update failed.\n" );
			return (int)statusFlag;
		}


		/** (5) get new measurement value */
		for (ii=0; ii<nX; ++ii) {
			/// eta_nI = ...
 		}


		// optional: logging
		for (ii=0; ii<nX; ++ii) {
			xLog[iter*nX+ii] = zOpt[nI*nZ+ii];	/* get x on last interval */
 		}

		// tmp: print variables
		printf("\nESTIMATED PROCESS TRAJECTORY:\n");
		for (kk=0; kk<nI; ++kk) {
			printf("\nt%02d:\t", kk);
			for (ii=0; ii<nZ; ++ii) {
				printf("%+.3e\t", zOpt[kk*nZ+ii]);
			}
		}
		printf("\nt%02d:\t", nI);
		for (ii=0; ii<nX; ++ii) {
			printf("%+.3e\t", zOpt[nI*nZ+ii]);
		}
		printf("\n\n");
		// end tmp
	}

 	printf("AT END:\n\n");
	printf("iter = %d\n", iter);



 	// optional: printing
 	printf("\nESTIMATED PROCESS TRAJECTORY (ONLY LAST VALUE, TO BE USED IN MPC):\n");
 	for (iter=0; iter<nSteps; ++iter) {
 		printf("\nt%02d:\t", iter);
 		for (ii=0; ii<nX; ++ii) {
 			printf("%+.3e\t", xLog[iter*nX+ii]);
 		}
	}
 	printf("\n\n");



	/** mandatory: cleanup of allocated data */
	qpDUNES_cleanup( &qpData );


	return 0;
}


/*
 *	end of file
 */
