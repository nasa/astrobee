/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2012 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file examples/example5.cpp
 *	\author Andreas Potschka, Christian Kirches
 *	\version 3.0beta
 *	\date 2011
 *
 *	Very simple example for testing qpOASES (using the possibility to
 *  compute the local linear feedback law)
 */



#include <stdlib.h>

#include <qpOASES.hpp>
#include "example4CP.cpp"


/**	Example for qpOASES main function using the possibility to specify
 *	user-defined constraint product function. */
int main( )
{
	USING_NAMESPACE_QPOASES

	int i,j,jj;
	real_t d = 0.0;

	/* Setup data of first QP... */
	real_t H[7*7];
	real_t A[50*7];
	real_t g[7];
	real_t lbA[50];

	/*	    ( 1.0 0.5 |                    )
	 *	    ( 0.5 2.0 |                    )
	 *	    ( --------+------------------- )
	 *	H = (         | 1e-6               )
	 *	    (         |      1e-6          )
	 *	    (         |           ...      )
	 *	    (         |               1e-6 ) */
	for( i=0; i<7*7; ++i )
		H[i] = 0.0;
	for( i=2; i<7; ++i )
		H[i*7+i] = 1.0e-6;
	H[0] = 1.0;
	H[1] = 0.5;
	H[7] = 0.5;
	H[8] = 2.0;

	/*	    ( x.x x.x | 1.0             )
	 *	    ( x.x x.x | ...             )
	 *	    ( x.x x.x | 1.0             )
	 *	    ( x.x x.x |     1.0         )
	 *	A = ( x.x x.x |     ...         )
	 *	    ( x.x x.x |     1.0         )
	 *	    ( x.x x.x |         ...     )
	 *	    ( x.x x.x |             1.0 )
	 *	    ( x.x x.x |             ... )
	 *	    ( x.x x.x |             1.0 ) */
	for( i=0; i<50*7; ++i )
		A[i] = 0.0;
	for( i=0; i<50; ++i )
	{
		for( j=0; j<2; ++j )
			A[i*7+j] = (real_t)rand() / (real_t)RAND_MAX;

		A[i*7 + (i/10)+2] = 1.0;
	}

	/*	    ( -1.0 )
	 *	    ( -0.5 )
	 *	    ( ---- )
	 *	g = (      )
	 *	    (      )
	 *	    (      )
	 *	    (      ) */
	for( i=0; i<7; ++i )
		g[i] = 0.0;
	g[0] = -1.0;
	g[1] = -0.5;

	for( i=0; i<50; ++i )
		lbA[i] = 1.0;

	/* ... and setting up user-defined constraint product function. */
	MyConstraintProduct myCP( 7,50,A );


	/* Setting up QProblem object and set construct product function. */
	QProblem example( 7,50 );
	example.setConstraintProduct( &myCP );


	/* Solve first QP. */
	real_t cputime = 1.0;
	int nWSR = 100;
	example.init( H,g,A,0,0,lbA,0, nWSR,&cputime );

	/* Get and print solution of QP. */
	real_t xOpt[7], yOpt[7+50];
	example.getPrimalSolution( xOpt );
	example.getDualSolution( yOpt );


	/* Compute local linear feedback law */
	const int n_rhs = 7+7+50;
	real_t g_in[7*n_rhs];
	real_t b_in[7*n_rhs];
	real_t bA_in[50*n_rhs];
	real_t x_out[7*n_rhs];
	real_t y_out[(7+50)*n_rhs];

	int ii;
	memset (g_in, 0, sizeof (g_in));
	memset (b_in, 0, sizeof (b_in));
	memset (bA_in, 0, sizeof (bA_in));

	for ( ii = 0; ii < 7; ++ii )
		g_in[ii*7 + ii] = 1.0;
	for ( ii = 0; ii < 7; ++ii )
		b_in[(ii+7)*7 + ii] = 1.0;
	for ( ii = 0; ii < 50; ++ii )
		bA_in[(ii+14)*50 + ii] = 1.0;

	example.solveCurrentEQP ( n_rhs, g_in, b_in, b_in, bA_in, bA_in, x_out, y_out );

	/* Verify validity of local feedback law by perturbation and hot starts */
	real_t perturb = 1.0e-6;
	real_t nrm = 0.0;
	for ( ii = 0; ii < n_rhs; ++ii )
	{
		for ( jj = 0; jj < 7; ++jj )
			g_in[ii*7 + jj] = g[jj] + g_in[ii*7+jj]*perturb;
		for ( jj = 0; jj < 50; ++jj )
			bA_in[ii*50 + jj] = lbA[jj] + bA_in[ii*50+jj]*perturb;

		nWSR = 100;
		example.hotstart( &g_in[ii*7],0,0,&bA_in[ii*50],0, nWSR, 0 );

		real_t xPer[7], yPer[7+50];
		example.getPrimalSolution( xPer );
		example.getDualSolution( yPer );

		for ( jj = 0; jj < 7; ++jj )
		{
			d = fabs (x_out[ii*7+jj]*perturb - (xPer[jj]-xOpt[jj]) );
			if (nrm < d) nrm=d;
		}
		for ( jj = 0; jj < 7+50; ++jj )
		{
			d = fabs (y_out[ii*(7+50)+jj]*perturb - (yPer[jj]-yOpt[jj]) );
			if (nrm < d) nrm=d;
		}
	}
	printf ("Maximum perturbation over all directions: %e\n", nrm);

	/* // print feedback matrix
	for (ii = 0; ii < n_rhs; ++ii)
	{
		printf ("x: ");
		for (jj = 0; jj < 7; ++jj )
			printf ("%8.2e ", x_out[ii*7+jj]);
		printf (" y: ");
		for (jj = 0; jj < 7+50; ++jj )
			printf ("%8.2e ", y_out[ii*(7+50)+jj]);
		printf("\n");
	}
*/

	return 0;
}


/*
 *	end of file
 */
