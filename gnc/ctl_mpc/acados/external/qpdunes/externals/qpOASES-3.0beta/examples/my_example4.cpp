

#include <qpOASES.hpp>
#include <stdio.h>

#define __MAKE_POS_DEF__
//#undef __MAKE_POS_DEF__

int main( )
{
	USING_NAMESPACE_QPOASES

	int nWSR = 100;
	/* Setting up QProblem object. */
	SQProblem example( 3,1 );

	Options options;
	options.setToFast();
//	options.setToDefault();
	options.initialStatusBounds = qpOASES::ST_INACTIVE;

	options.terminationTolerance = 1.e-12;
	options.initialStatusBounds = qpOASES::ST_INACTIVE;
	options.enableFarBounds = qpOASES::BT_FALSE;
	options.enableRegularisation = qpOASES::BT_FALSE;

	example.setOptions( options );


	/* Setup data of first QP. */
	real_t H[3*3] = {
	 1.00000000000000e+03,	 0.00000000000000e+00,	 0.00000000000000e+00,
	 0.00000000000000e+00,	 1.00000000000000e+00,	 0.00000000000000e+00,
	 0.00000000000000e+00,	 0.00000000000000e+00,	 1.00000000000000e+00,
	};
	real_t g[3] =	{
	 -3.00000000000000e+03,
	 -5.95000000000000e+01,
	 -2.95000000000000e-01,
	};
	real_t zLow[3] =	{
	-1.00000000000000e+00,
	 0.00000000000000e+00,
	-3.00000000000000e+01,
	};
	real_t zUpp[3] =	{
	 -1.00000000000000e+00,
	  0.00000000000000e+00,
	  3.00000000000000e+01,
	};
	real_t D[3*1] =	{
	0.00000000000000e+00,	 5.00000000000000e+00,	 1.00000000000000e+00,
	};
	real_t dLow[1] =	{
	0.00000000000000e+00,
	};
	real_t dUpp[1] =	{
	0.00000000000000e+00,
	};


	returnValue status = qpOASES::SUCCESSFUL_RETURN;
	status = example.init( H,g,D,zLow,zUpp,dLow,dUpp, nWSR );
	printf("qpOASES_status = %d\n", (int)status );

	/* Get and print solution of second QP. */
	real_t xOpt[3];
	example.getPrimalSolution( xOpt );
	printf("first QP:\n");
	for (int ii =0; ii<3; ++ii )	{
		printf("x[%d] = %.3e\n", ii, xOpt[ii]);
	}


	nWSR = 100;

	/* Setup data of second QP. */
	real_t H2[3*3] = {
	 1.00000000000000e+03,	 0.00000000000000e+00,	 0.00000000000000e+00,
	 0.00000000000000e+00,	 1.00000000000000e+00,	 0.00000000000000e+00,
	 0.00000000000000e+00,	 0.00000000000000e+00,	 1.00000000000000e+00,
	};
	real_t g2[3] =	{
	 0.00000000000000e+00,
	-1.00000000000000e+01,
	 3.33066907387547e-18,
	};
	real_t zLow2[3] =	{
	-1.90000000000000e+00,
	-3.00000000000000e+00,
	-3.00000000000000e+01,
	};
	real_t zUpp2[3] =	{
	 1.90000000000000e+00,
	 3.00000000000000e+00,
	 3.00000000000000e+01,
	};
	real_t D2[3*1] =	{
	 0.00000000000000e+00,	 5.00000000000000e+00,	 1.00000000000000e+00,
	};
	real_t dLow2[1] =	{
	 0.00000000000000e+00,
	};
	real_t dUpp2[1] =	{
	 0.00000000000000e+00,
	};


	status = example.hotstart( H2,g2,D2,zLow2,zUpp2,dLow2,dUpp2, nWSR );
//	status = example.init( H2,g2,D2,zLow2,zUpp2,dLow2,dUpp2, nWSR );
	printf("qpOASES_status = %d\n", (int)status );

	example.getPrimalSolution( xOpt );
	printf("second QP:\n");
	for (int ii =0; ii<3; ++ii )	{
		printf("x[%d] = %.3e\n", ii, xOpt[ii]);
	}



	return 0;
}


/*
 *	end of file
 */
