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
 *	\file examples/nmpcPrototype.c
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2012
 *
 *	Example for NMPC coupling
 *
 *	In the context of NMPC qpDUNES solves a problem
 *
 *	min  sum_{k=0..nI} z_k'*H_k*z_k + g_k'*z_k
 *	s.t. x_{k+1} = C_k * z_k + c_k				for k=0..nI-1
 *	     dLow <= D * z_k <= dUpp				for k=0..nI
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
 *  -U__USE_ASSERTS__						// switch some safty checks off
 *
 */


#include <qpDUNES.h>


#define INFTY 1.0e12

//#define USE_D
#undef USE_D


int main( )
{
	return_t statusFlag;

	int iter, ii, kk;

	/** define problem dimensions */
	const unsigned int nI = 3;			/* number of control intervals */
	const int nX = 2;					/* number of states */
	const int nU = 1;					/* number of controls */
	const unsigned int nZ = nX+nU;		/* number of stage variables */
#ifdef USE_D
	unsigned int nD[nI+1];  			/* number of constraints */
	for ( kk=0; kk<nI; ++kk ) {
		nD[kk] = nX+nU;
	}
	nD[nI] = nX;
#else
	unsigned int* nD = 0;	  			/* number of constraints */
#endif /* USE_D */


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

#ifdef USE_D
	const double* zLow = 0;
	const double* zUpp = 0;
#else
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
#endif /* USE_D */


#ifdef USE_D
	const double D[] =
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
	const double dLow[] =
		{	-1.9, -3.0, -30.0,
			-1.9, -3.0, -30.0,
			-1.9, -3.0, -30.0,
			-1.9, -3.0
		};

	const double dUpp[] =
		{	 1.9,  3.0,  30.0,
			 1.9,  3.0,  30.0,
			 1.9,  3.0,  30.0,
			 1.9,  3.0
		};
#else
	const double* D = 0;
	const double* dLow = 0;
	const double* dUpp = 0;
#endif /* USE_D */



	/** define simulation environment */
	double x0[] = { -1.0, 0.0 };	/* initial value */
#ifdef USE_D
	double* z0Low = 0;
	double* z0Upp = 0;
	double d0Low[nX+nU];
	double d0Upp[nX+nU];
#else
	double z0Low[nX+nU];			/* auxiliary variables */
	double z0Upp[nX+nU];
	double* d0Low = 0;
	double* d0Upp = 0;
#endif /* USE_D */

	double zOpt[nI*nZ+nX];			/* primal solution */
	double lambdaOpt[nI*nX];		/* dual solution */
	double muOpt[2*nI*nZ+2*nX];

	const unsigned int nSteps = 6;	/* number of simulation steps */


	/** (1) set up a new qpDUNES problem */
	qpData_t qpData;


	/** (2) set qpDUNES options */
	qpOptions_t qpOptions = qpDUNES_setupDefaultOptions();
	qpOptions.maxIter    = 100;
	qpOptions.printLevel = 2;
	qpOptions.stationarityTolerance = 1.e-6;


	/** (3) allocate data for qpDUNES and set options */
	qpDUNES_setup( &qpData, nI, nX, nU, nD, &(qpOptions) );
	

	/** (4) set sparsity of primal Hessian and local constraint matrix */
	for ( kk=0; kk<nI+1; ++kk ) {
		qpData.intervals[kk]->H.sparsityType = QPDUNES_DIAGONAL;
		qpData.intervals[kk]->D.sparsityType = QPDUNES_IDENTITY;
	}
	
	/** (5) initial MPC data setup: components not given here are set to zero (if applicable)
	 *      instead of passing g, D, zLow, zUpp, one can also just pass NULL pointers (0) */
	statusFlag = qpDUNES_init( &qpData, H, g, C, c, zLow,zUpp, D,dLow,dUpp );	/* todo: add constraint vectors, make non-trivial example */
	if (statusFlag != QPDUNES_OK) {
		printf( "Data init failed.\n" );
		return (int)statusFlag;
	}
	

	/** MAIN MPC SIMULATION LOOP */
 	for ( iter=0; iter<nSteps; ++iter ) {
 		/** (1) embed current initial value */
		for ( ii=0; ii<nX; ++ii ) {
#ifdef USE_D
			d0Low[ii] = x0[ii];
			d0Upp[ii] = x0[ii];
#else
			z0Low[ii] = x0[ii];
			z0Upp[ii] = x0[ii];
#endif /* USE_D */
		}
		for ( ii=nX; ii<nX+nU; ++ii ) {
#ifdef USE_D
			d0Low[ii] = dLow[ii];
			d0Upp[ii] = dUpp[ii];
#else
			z0Low[ii] = zLow[ii];
			z0Upp[ii] = zUpp[ii];
#endif /* USE_D */
		}
		statusFlag = qpDUNES_updateIntervalData( &qpData, qpData.intervals[0], 0, 0, 0, 0, z0Low,z0Upp, D,d0Low,d0Upp, 0 );
		if (statusFlag != QPDUNES_OK) {
			printf( "Initial value embedding failed.\n" );
			return (int)statusFlag;
		}

#ifdef USE_D
		qpDUNES_printMatrixData( d0Low, 1, nZ, "d0Low@it%d", iter);
		qpDUNES_printMatrixData( d0Upp, 1, nZ, "d0Upp@it%d", iter);
#else
		qpDUNES_printMatrixData( z0Low, 1, nZ, "z0Low@it%d", iter);
		qpDUNES_printMatrixData( z0Upp, 1, nZ, "z0Upp@it%d", iter);
#endif /* USE_D */


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
		qpDUNES_printMatrixData( lambdaOpt, 1, nI*nX, "dual lambda" );
		qpDUNES_printMatrixData( muOpt, 1, 2*nI*nZ + 2*nX, "dual mu" );
		/// ...
		

 		/** (4) prepare QP for next solution */
		qpDUNES_shiftLambda( &qpData );			/* shift multipliers */
		qpDUNES_shiftIntervals( &qpData );		/* shift intervals (particulary important when using qpOASES for underlying local QPs) */
		
		// optional
		
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
		
		// mandatory


		/** (5) simulate next initial value */
		for (ii=0; ii<nX; ++ii) {
			/// x0 = ...
			x0[ii] = zOpt[1*nZ+ii];
 		}
	}


	/** cleanup of allocated data */
	qpDUNES_cleanup( &qpData );


	return 0;
}


/*
 *	end of file
 */
