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
 *	\file examples/singleItegrator_d.c
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2013
 *
 *	Simple single integrator example that triggers line search
 */



#include <qpDUNES.h>

#define INFTY 1.0e12

#define USE_AFFINE
//#undef USE_AFFINE

int main( )
{
	int k;
	
	const unsigned int nI = 8;		/* number of control intervals */
	const unsigned int nX = 1;		/* number of states */
	const unsigned int nU = 1;		/* number of controls */
	#ifndef USE_AFFINE
		unsigned int* nD = 0;  		/* number of affine constraints */
	#else
		unsigned int nD[nI+1];  	/* number of affine constraints */
		for ( k=0; k<nI; ++k ) {
			nD[k] = 1;
		}
		nD[nI] = 0;
	#endif
	
	double dt = 0.01;	/* discretization sampling time 10ms, needed for constraints */

	
	
	double H[] =
		{	1.0e1,	0,
			0,		1.0e-2,

			1.0e1,	0,
			0,		1.0e-2,

			1.0e1,	0,
			0,		1.0e-2,

			1.0e1,	0,
			0,		1.0e-2,

			1.0e1,	0,
			0,		1.0e-2,

			1.0e1,	0,
			0,		1.0e-2,

			1.0e1,	0,
			0,		1.0e-2,

			1.0e1,	0,
			0,		1.0e-2,

			1.0e1
		};
	
	double* g = 0;

	
	double C[] =
		{	
			1.0, 1.0*dt,
			1.0, 1.0*dt,
			1.0, 1.0*dt,
			1.0, 1.0*dt,
			1.0, 1.0*dt,
			1.0, 1.0*dt,
			1.0, 1.0*dt,
			1.0, 1.0*dt
		};
	
	
	double c[] =
		{
			0.0,
			0.0,
			0.0,
			0.0,
			0.0,
			0.0,
			0.0,
			0.0
		};


	const double D[] =
		{	1.0,		/* some random constraint limiting the acceleration for high velocities */

			1.0,

			1.0,

			1.0,

			1.0,

			1.0,

			1.0,

			1.0
		};
	const double dLow[] =
		{	-INFTY,
			-INFTY,
			-INFTY,
			-INFTY,
			-INFTY,
			-INFTY,
			-INFTY,
			-INFTY
		};
	const double dUpp[] =
		{	INFTY,
			INFTY,
			INFTY,
			INFTY,
			INFTY,
			INFTY,
			INFTY,
			INFTY
		};

			
			
	double zLow[] =
		{
			-1.0,	-3.0,
			-1.9,	-3.0,
			-1.9,	-3.0,
			-1.9,	-3.0,
			-1.9,	-3.0,
			-1.9,	-3.0,
			-1.9,	-3.0,
			-1.9,	-3.0,
			-1.9
		};
	
	double zUpp[] =
		{
			-1.0,	3.0,
			-0.95,	3.0,
			-0.95,	3.0,
			-0.95,	3.0,
			-0.95,	3.0,
			-0.95,	3.0,
			-0.95,	3.0,
			-0.95,	3.0,
			-0.95
		};

	

//	mpcProblem_t mpcProblem;
	qpData_t qpData;
	
	qpOptions_t qpOptions = qpDUNES_setupDefaultOptions();
	qpOptions.maxIter    = 5;
	qpOptions.printLevel = 3;
	qpOptions.stationarityTolerance = 1.e-6;
	qpOptions.equalityTolerance     = 2.221e-16;
	qpOptions.maxNumLineSearchIterations            = 8; //4
	qpOptions.maxNumLineSearchRefinementIterations  = 50; //4
//	qpOptions.lsType			= QPDUNES_LS_BACKTRACKING_LS;
	qpOptions.lsType			= QPDUNES_LS_ACCELERATED_GRADIENT_BISECTION_LS;
//	qpOptions.regType            = QPDUNES_REG_SINGULAR_DIRECTIONS;
	
//	mpcDUNES_setup( &mpcProblem, nI, nX, nU, nD, &(qpOptions) );
	qpDUNES_setup( &qpData, nI, nX, nU, nD, &(qpOptions) );

	
	/** initialize the MPC problem; note that matrices are factorized already now */
	#ifndef USE_AFFINE
		qpDUNES_init( &qpData, H, g, C, c, zLow,zUpp, 0, 0, 0 );
	#else
		qpDUNES_init( &qpData, H, g, C, c, zLow,zUpp, D, dLow, dUpp );
	#endif

	
	/** get feedback for a given initial QP value */
	qpDUNES_solve( &qpData );
	
	
//	qp42_printMatrixData( mpcProblem.xOpt, nI+1, nX, "xOpt:" );
//	qp42_printMatrixData( mpcProblem.uOpt, nI, nU, "uOpt:" );
//	qp42_printMatrixData( mpcProblem.lambdaOpt, nI, nX, "lambdaOpt:" );
	
	qpDUNES_cleanup( &qpData );
	
	return 0;
}


/*
 *	end of file
 */
