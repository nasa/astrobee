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
 *	\file examples/example3.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.0beta
 *	\date 2008-2009
 *
 *	Example demonstrating usage of qpOASES for solving a QP sequence of the
 *	Online QP Benchmark Collection. In order to run it, you have to download
 *	"Example 02" from http://homes.esat.kuleuven.be/~optec/software/onlineQP/
 *	and store it into the directory EXAMPLES/chain80w/.
 */



#include <qpOASES.hpp>


/** Example for qpOASES main function using the OQP interface. */
int main( )
{
	USING_NAMESPACE_QPOASES

	/* 1) Define benchmark arguments. */
	BooleanType isSparse = BT_FALSE;
	Options options;
	options.setToMPC();
// 	options.setToReliable();

	int nWSR = 600;
	real_t maxCPUtime = 10.0; /* seconds */
	real_t maxPrimalDeviation, maxDualDeviation, maxObjDeviation;

	/* 2) Run benchmark. */
	if ( runOQPbenchmark(	"./chain80w/",
							isSparse,
							options,
							nWSR,
							maxCPUtime,
							maxPrimalDeviation,
							maxDualDeviation,
							maxObjDeviation
							) != SUCCESSFUL_RETURN )
	{
		myPrintf( "In order to run this example, you need to download example no. 02\nfrom the Online QP Benchmark Collection website first!\n" );
		return -1;
	}

	/* 3) Print results. */
	printf( "\n\n" );
	printf( "OQP Benchmark Results:\n" );
	printf( "======================\n\n" );
	printf( "maximum primal deviation:     %.2e\n",maxPrimalDeviation );
	printf( "maximum dual deviation:       %.2e\n",maxDualDeviation );
	printf( "maximum objective deviation:  %.2e\n",maxObjDeviation );
	printf( "\n" );
	printf( "maximum CPU time:             %.2f milliseconds\n\n",1000.0*maxCPUtime );

	return 0;
}


/*
 *	end of file
 */
