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
 *	\file examples/example6.cpp
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

	/* Setup data of first QP. */
	real_t H[5*5] = {	1.224642131370767e+01, 2.908638763113702e+00, 0.0, 0.0, 0.0,
						2.908638763113702e+00, 2.497106275003180e+00, 0.0, 0.0, 0.0,
						0.0, 0.0, 1.0, 0.0, 0.0,
						0.0, 0.0, 0.0, 5.158460640334052e-02, 4.723556059962540e-02,
						0.0, 0.0, 0.0, 4.723556059962540e-02, 4.325317843302175e-02 };
	real_t A[2*5] = { 	-1.404358970692652e+00, -2.556613491156063e+00, 3.202524559238066e+00, -1.0, 0.0,
						6.587910295430314e-01, -5.349454475937998e-01, 4.391976356955536e-01, 0.0, -1.0 };
	real_t g[5] = { 	2.474135331302147e+01,
						5.857286430296258e+00,
						2.359382646348721e-01,
						1.721047069188781e-01,
						1.575947337774199e-01 };
	real_t lb[5] = { -10.0, -10.0, -10.0, -10.0, -10.0 };
	real_t ub[5] = {  10.0,  10.0,  10.0,  10.0,  10.0 };
	real_t lbA[2] = { 1.643135416077167e+00, 1.056813028189597e+00 };
	real_t ubA[2] = { 1.643135416077167e+00, 1.056813028189597e+00 };

	/* Setting up QProblem object. */
	QProblem example( 5,2 );

	Options options;
// 	options.enableFlippingBounds = BT_FALSE;
	options.initialStatusBounds = ST_INACTIVE;
	example.setOptions( options );
	
	/* Solve first QP. */
	int nWSR = 10;
	example.init( H,g,A,lb,ub,lbA,ubA, nWSR,0 );

	/* Get and print solution of second QP. */
	real_t xOpt[5];
	example.getPrimalSolution( xOpt );
	printf( "\nxOpt = [ %e, %e, ... ];  objVal = %e\n\n", xOpt[0],xOpt[1],example.getObjVal() );

	return 0;
}


/*
 *	end of file
 */
