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
 *	\file examples/example1.c
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 *
 *	Very simple example for testing qp42.
 */



#include <qpDUNES.h>

#define INFTY 1.0e12

int main( )
{
	int i;
	boolean_t isLTI;
	
	unsigned int nI = 2;
	unsigned int nX = 3;
	unsigned int nU = 2;
	unsigned int* nD = 0;
//	unsigned int nD[3] =
//		{ 3+2,
//		  3+2,
//		  3	};
	
	double Q[3*3] =
/*		{	2.0, -1.0, 0.0,
		-1.0, 2.0, -1.0,
 			0.0, -1.0, 2.0	}; */
		{	1.0, 0.0, 0.0, 
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0	};
			
	double x0[3] = 
		{ 2.0, 3.0, 4.0 };
		
		
	double R[2*2] =
		{	1.0, 0.0,
			0.0, 1.0	};
	double *S=0;
	
	double* P = Q;
	
	double A[3*3] =
		{	1.0, 0.0, 0.0, 
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0	};
	double B[3*2] =
		{	1.0, 0.0,
			0.0, 1.0,
			1.0, 1.0	};
	double c[3] = 
		{	5.0,
			5.0,
			5.0	};
	
	double xLow[3] = { -INFTY, -INFTY, -INFTY };
	double xUpp[3] = {  INFTY,  INFTY,  INFTY };
	double uLow[2] = { -INFTY, -INFTY };
	double uUpp[2] = {  INFTY,  INFTY };
	
	qpData_t qpData;
	
	/* options need to be set before setup
	qpOptions_t qpOptions = qp42_setupDefaultOptions;
	qpOptions.maxIter    = 20;
	qpOptions.printLevel = 1;
	qpOptions.stationarityTolerance = 1.e-6;
	qpOptions.equalityTolerance     = 2.221e-16;
	qpOptions.QP42_ZERO             = 1.0e-50;
	qpOptions.QP42_INFTY            = INFTY;
	qpOptions.maxNumLineSearchIterations            = 4;
	*/


	qpDUNES_setup( &qpData, nI, nX, nU, nD, 0 );	/* passing 0 in the last argument sets the default QP options */

	for( i=0; i<nI; ++i )
	{
		qpDUNES_setupSimpleBoundedInterval(  &qpData, qpData.intervals[i],Q,R,S, A,B,c, xLow,xUpp,uLow,uUpp );
// 		qp42_updateSimpleBoundedInterval( &qpData, qpData.intervals[i],Q,R,S, A,B,c, xLow,xUpp,uLow,uUpp );
	}
	qpDUNES_setupSimpleBoundedInterval(  &qpData, qpData.intervals[nI], P,0,0, 0,0,0, xLow,xUpp,0,0 );
// 	qp42_updateSimpleBoundedInterval( &qpData, qpData.intervals[nI], P,0,0, 0,0,0, xLow,xUpp,0,0 );

	qpDUNES_setupAllLocalQPs( &qpData, isLTI=QPDUNES_TRUE );	/* determine local QP solvers and set up auxiliary data */

	
//	qp42_solve( &qpData, x0 );
	qpDUNES_solve( &qpData );
	
	for( i=0; i<nI; ++i )
	{
		qpDUNES_printMatrixData( qpData.intervals[i]->z.data, 1, nX+nU, "z[%d]:", i );
	}
	qpDUNES_printMatrixData( qpData.intervals[nI]->z.data, 1, nX, "z[%d]:", i );
	
	qpDUNES_cleanup( &qpData );
	
	printf( "example1 done.\n" );
	
	return 0;
}


/*
 *	end of file
 */
