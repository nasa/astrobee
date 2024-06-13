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
 *	\file examples/example1_ltv.c
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2013
 *
 *	Very simple example for testing qp42.
 */



#include <mpc/setup_mpc.h>


#define INFTY 1.0e12

int main( )
{
	int k, i;
	
	/* Problem dimensions */
	unsigned int nI = 2;
	unsigned int nX = 3;
	unsigned int nU = 2;
	unsigned int* nD = 0;
//	unsigned int nD[3] =
//		{ 3+2,
//		  3+2,
//		  3	};
	
	/* Problem data */
	double H[2*5*5 + 3*3] =
		{	1.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 1.0,

			1.0, 0.0, 0.0, 0.0, 0.0,
			0.0, 1.0, 0.0, 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0, 0.0,
			0.0, 0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 0.0, 1.1,

			1.0, 0.0, 0.0,
			0.0, 1.0, 0.0,
			0.0, 0.0, 1.0
		};
			
	double g[2*5+3] =
		{	-1.0,
			-1.0,
			-1.0,
			-1.0,
			-1.0,

			-1.0,
			-1.0,
			-1.0,
			-1.0,
			-1.0,

			-1.0,
			-1.0,
			-1.0
		};
	
	
	double C[2*3*5] =
		{	1.0, 0.0, 0.0,	1.0, 0.0,
			0.0, 1.0, 0.0,	0.0, 1.0,
			0.0, 0.0, 1.0,	1.0, 1.0,

			1.0, 0.0, 0.0,	1.0, 0.0,
			0.0, 1.0, 0.0,	0.0, 1.0,
			0.0, 0.0, 1.0,	1.0, 1.0
		};
	double c[2*3] =
		{	5.0,
			5.0,
			5.0,

			5.0,
			5.0,
			5.0
		};


	double zLow[2*5+3] =
		{ -INFTY, -INFTY, -INFTY,	-INFTY, -INFTY,
		  -INFTY, -INFTY, -INFTY,	-INFTY, -INFTY,
		  -INFTY, -INFTY, -INFTY
		};
	double zUpp[2*5+3] =
		{  INFTY,  INFTY,  INFTY,	 INFTY,  INFTY,
		   INFTY,  INFTY,  INFTY,	 INFTY,  INFTY,
		   INFTY,  INFTY,  INFTY
		};

	double *zRef=0;


	double x0[3] =
		{ 2.0, 3.0, 4.0 };


	/* Set up a mpcDUNES problem */
	mpcProblem_t mpcProblem;
	qpOptions_t qpOptions = qpDUNES_setupDefaultOptions();

	qpOptions.maxIter    = 20;
	qpOptions.printLevel = 3;
	qpOptions.stationarityTolerance = 1.e-6;
	qpOptions.printLevel = 3;
	qpOptions.maxNumLineSearchIterations            = 4;

	mpcDUNES_setup( &mpcProblem, nI, nX, nU, nD, &(qpOptions) );

	mpcDUNES_initLtvSb(	&mpcProblem, H, g, C,c, zLow,zUpp, zRef );


	/* Call mpcDUNES to solve the problem */
	mpcDUNES_solve( &mpcProblem, x0 );


	/* print results to screen */
	printf( "xOpt, uOpt:\n[\n[\t" );
	for(k=0; k<nI; ++k) {
		for(i=0; i<nX; ++i) {
			printf( "% .3e\t", mpcProblem.xOpt[k*nX+i] );
		}
		for(i=0; i<nU; ++i) {
			printf( "% .3e\t", mpcProblem.uOpt[k*nU+i] );
		}
		printf( "]\n[\t" );
	}
	for(i=0; i<nX; ++i) {
		printf( "% .3e\t", mpcProblem.xOpt[nI*nX+i] );
	}
	printf( "]\n]\n\n" );


	
	mpcDUNES_cleanup( &mpcProblem );
	
	printf( "example1_ltv done.\n" );
	
	return 0;
}


/*
 *	end of file
 */
