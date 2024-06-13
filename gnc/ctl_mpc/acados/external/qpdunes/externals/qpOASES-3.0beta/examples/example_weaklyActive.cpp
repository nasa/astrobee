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
 *	\file examples/example1.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.0beta
 *	\date 2007-2012
 *
 *	Very simple example for testing qpOASES (using QProblem class).
 */



#include <qpOASES.hpp>


/** Example for qpOASES main function using the QProblem class. */
int main( )
{
	USING_NAMESPACE_QPOASES

	int nWSR = 10;

	real_t D[3*3] = {	1.0,	 0.0,	 0.0,
						0.0,	 1.0,	 0.0,
						0.0,	 0.0,	 1.0
					};

	real_t bndRelax = 1.e-9;
//	real_t bndRelax = 1.e-4;


	real_t dLow[3] = { 0.-bndRelax,	-1.e4,	-1.e4	};
	real_t dUpp[3] = { 0.+bndRelax,	 1.e4,	 1.e4	};


	real_t H[3*3] = {
						1.00100000000000e-04,	 0.00000000000000e+00,	 0.00000000000000e+00,
						0.00000000000000e+00,	 1.00100000000000e-04,	 0.00000000000000e+00,
						0.00000000000000e+00,	 0.00000000000000e+00,	 1.00000000000000e+00
					};

	real_t q[3] = { 0.0,	 0.0,	 0.0	};



	/* Setting up QProblem object. */
//	QProblem example( 3,0 );
//	returnValue status = example.init( H,q,0,dLow,dUpp,0,0, nWSR );
	QProblem example( 3,3 );
	returnValue status = example.init( H,q,D,0,0,dLow,dUpp, nWSR );

	/* resolve by giving an initial guess for homotopy */
//	Bounds guessedBounds( 3 );
//	guessedBounds.setupAllFree( );
//	Constraints guessedConstraints( 3 );
//	guessedConstraints.setupAllInactive( );
//	returnValue status = example.init( H,q,D,0,0,dLow,dUpp, nWSR, 0, 0, 0, &guessedBounds, &guessedConstraints );

	printf("\nsolver status: %d\n", (int)status );


	/* Get and print solution of second QP. */
	real_t xOpt[3];
//	real_t muOpt[example.getNC()+example.getNV()];
	real_t muOpt[3+3];
	example.getPrimalSolution( xOpt );
	example.getDualSolution( muOpt );
	printf( "\nxOpt = [ %.12e, %.12e, %.12e ];  objVal = %.12e\n\n", xOpt[0],xOpt[1],xOpt[2], example.getObjVal() );
	printf( "muOptBounds = [ %.12e, %.12e, %.12e ]\n", muOpt[0],muOpt[1],muOpt[2] );
	printf( "muOptCnstrs = [ %.12e, %.12e, %.12e ]\n\n", muOpt[3],muOpt[4],muOpt[5] );
	printf( "number free variables (nZ): %d\n",example.getNZ() );
	printf( "number of implicitly detected equality constraints: %d\n",example.getNEC() );
	printf( "number of active constraints: %d\n",example.getNAC() );
	printf( "number of fixed bounds: %d\n",example.getNFX() );
	printf( "number of constraints: %d\n",example.getNC() );



	example.printOptions();
	
	return 0;
}


/*
 *	end of file
 */
