/*
 *	This file is part of qpDUNES.
 *
 *	qpDUNES -- A DUal NEwton Strategy for convex quadratic programming.
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
 *	\file interfaces/matlab/qpDUNES_matlab_utils.cpp
 *	\author Janick Frasch
 *	\version 0.1alpha
 *	\date 2012-2013
 *
 *	Collects utility functions for Interface to Matlab(R) that
 *	enables to call qpDUNES as a MEX function.
 *
 */

extern "C" {
	#include <setup_mpc.h>
	#include <qpDUNES.h>
}


#include "mex.h"
#include "matrix.h"
#include "string.h"



/*
 *	a l l o c a t e O u t p u t s M P C
 */
void allocateOutputsMPC (	mxArray* plhs[],
					  		int nlhs,
					  		int nI,
					  		int nX,
					  		int nU
							)
{
	/* Create output vectors and assign pointers to them. */
	plhs[0] = mxCreateDoubleMatrix( nU*nI, 1, mxREAL );	/* uOpt */

	if ( nlhs >= 2 )
	{
		plhs[1] = mxCreateDoubleMatrix( nX*(nI+1), 1, mxREAL );	/* xOpt */

		if ( nlhs >= 3 )
		{
			plhs[2] = mxCreateDoubleMatrix( 1, 1, mxREAL );	/* status */

			if ( nlhs >= 4 )
			{
				plhs[3] = mxCreateDoubleMatrix( 1, 1, mxREAL );	/* objVal */

				if ( nlhs >= 5 )
				{
					plhs[4] = mxCreateDoubleMatrix( 1, 1, mxREAL );	/* timing */
				}
			}
		}
	}
}


/*
 *	o b t a i n O u t p u t s M P C
 */
void obtainOutputsMPC ( 	const mpcProblem_t* const mpcProblem,
							mxArray* const plhs[],
							int nlhs,
							real_t dTime
							)
{
	const qpData_t* qpData = &(mpcProblem->qpData);
	int nI = qpData->nI;
	int nX = qpData->nX;
	int nU = qpData->nU;
	

	/* uOpt */
	double* u = mxGetPr( plhs[0] );
	qpDUNES_copyArray( u, mpcProblem->uOpt, nI*nU );
	
	if ( nlhs >= 2 )
	{
		/* xOpt */
		double* x = mxGetPr( plhs[1] );
		qpDUNES_copyArray( x, mpcProblem->xOpt, (nI+1)*nX );
		

		if ( nlhs >= 3 )
		{
			/* status */
			double* status = mxGetPr( plhs[2] );
			*status = mpcProblem->exitFlag;

			if ( nlhs >= 4 )
			{
				/* objVal */
				double* objVal = mxGetPr( plhs[3] );
				*objVal = mpcProblem->optObjVal;

				if ( nlhs >= 4 )
				{
					/* timings */
					double* timingPtr = mxGetPr( plhs[3] );
					*timingPtr = dTime;
				}
			}
		}
	}
}


/*
 *	a l l o c a t e O u t p u t s Q P
 */
void allocateOutputsQP(	mxArray* plhs[],
					  	int nlhs,
					  	int nI,
					  	int nX,
					  	int nZ
						)
{
	/* Create output vectors and assign pointers to them. */
	plhs[0] = mxCreateDoubleMatrix( nZ*nI+nX, 1, mxREAL );	/* zOpt */

	if ( nlhs >= 2 )
	{
		plhs[1] = mxCreateDoubleMatrix( 1, 1, mxREAL );	/* status */

		if ( nlhs >= 3 )
		{
			plhs[2] = mxCreateDoubleMatrix( nX*nI, 1, mxREAL );	/* lambdaOpt */

			if ( nlhs >= 4 )
			{
				plhs[3] = mxCreateDoubleMatrix( 0, 0, mxREAL );	/* muOpt */

				if ( nlhs >= 5 )
				{
					plhs[4] = mxCreateDoubleMatrix( 1, 1, mxREAL );	/* objVal */

					if ( nlhs >= 6 )
					{
						plhs[5] = mxCreateDoubleMatrix( 1, 1, mxREAL );	/* timing */
					}
				}
			}
		}
	}
}


/*
 *	o b t a i n O u t p u t s Q P
 */
void obtainOutputsQP( 	qpData_t* const qpData,
						mxArray* const plhs[],
						int nlhs,
						return_t solverStatus,
						real_t dTime
						)
{
	int kk;
	int nI = qpData->nI;
	int nX = qpData->nX;
	int nZ = qpData->nZ;


	/* zOpt (primal solution) */
	double* zOpt = mxGetPr( plhs[0] );
	for ( kk=0; kk<nI+1; ++kk ) { /* regular intervals */
		qpDUNES_copyArray( &(zOpt[kk*nZ]), qpData->intervals[kk]->z.data, qpData->intervals[kk]->nV );
	}

	if ( nlhs >= 2 )
	{
		/* status */
		double* status = mxGetPr( plhs[1] );
		*status = solverStatus;

		if ( nlhs >= 3 )
		{
			/* lambda (dual equality constraints) */
			double* lambdaOpt = mxGetPr( plhs[2] );
			qpDUNES_copyArray( lambdaOpt, qpData->lambda.data, nI*nX );

			if ( nlhs >= 4 )
			{
				/* mu (dual inequality constraints) */
				/* TODO: compute mu  */
				/* double* muOpt = mxGetPr( plhs[3] ); */

				if ( nlhs >= 5 )
				{
					/* objVal */
					double* objVal = mxGetPr( plhs[4] );
					*objVal = qpDUNES_computeObjectiveValue( qpData );

					if ( nlhs >= 6 )
					{
						/* timings */
						double* timingPtr = mxGetPr( plhs[5] );
						*timingPtr = dTime;
					}
				}
			}
		}
	}
}



/*
 *	h a s O p t i o n s V a l u e
 */
boolean_t getOptionValue( const mxArray* const optionsPtr, const char* const optionString, double** optionValue )
{
	mxArray* optionFieldPtr = mxGetField( optionsPtr,0,optionString );

    if( !mxIsEmpty(optionFieldPtr) )
	{
		if ( ( mxGetM( optionFieldPtr ) != 1 ) || ( mxGetN( optionFieldPtr ) != 1 ) ) {
			mexPrintf( "Error reading options string '%s'.", optionString );
			mexErrMsgTxt( "ERROR (qpDUNES_options): Option value has to be a numerical constant." );
		}

		*optionValue = mxGetPr( optionFieldPtr );
		return QPDUNES_TRUE;
	}

	return QPDUNES_FALSE;
}


/*
 *	s e t u p O p t i o n s
 */
return_t qpDUNES_setupOptionsMatlab( qpOptions_t* options, const mxArray* const optionsPtr )
{
	double* optionValue;


	/* iteration limits */
	if ( getOptionValue( optionsPtr, "maxIter", &optionValue ) == QPDUNES_TRUE )
		options->maxIter = (int_t)*optionValue;

	if ( getOptionValue( optionsPtr, "maxNumLineSearchIterations", &optionValue ) == QPDUNES_TRUE )
		options->maxNumLineSearchIterations = (int_t)*optionValue;

	if ( getOptionValue( optionsPtr, "maxNumLineSearchRefinementIterations", &optionValue ) == QPDUNES_TRUE )
		options->maxNumLineSearchRefinementIterations = (int_t)*optionValue;

	if ( getOptionValue( optionsPtr, "maxNumQpoasesIterations", &optionValue ) == QPDUNES_TRUE )
		options->maxNumQpoasesIterations = (int_t)*optionValue;


	/* logging */
	if ( getOptionValue( optionsPtr, "logLevel", &optionValue ) == QPDUNES_TRUE )
		options->logLevel = (logLevel_t)*optionValue;


	/* printing */
	if ( getOptionValue( optionsPtr, "printLevel", &optionValue ) == QPDUNES_TRUE )
		options->printLevel = (int_t)*optionValue;

	if ( getOptionValue( optionsPtr, "printIntervalHeader", &optionValue ) == QPDUNES_TRUE )
		options->printIntervalHeader = (int_t)*optionValue;

	if ( getOptionValue( optionsPtr, "printIterationTiming", &optionValue ) == QPDUNES_TRUE )
		options->printIterationTiming = (boolean_t)*optionValue;

	if ( getOptionValue( optionsPtr, "printLineSearchTiming", &optionValue ) == QPDUNES_TRUE )
		options->printLineSearchTiming = (boolean_t)*optionValue;


	/* numerical tolerances */
	if ( getOptionValue( optionsPtr, "stationarityTolerance", &optionValue ) == QPDUNES_TRUE )
		options->stationarityTolerance = (real_t)*optionValue;

	if ( getOptionValue( optionsPtr, "equalityTolerance", &optionValue ) == QPDUNES_TRUE )
		options->equalityTolerance = (real_t)*optionValue;

	if ( getOptionValue( optionsPtr, "newtonHessDiagRegTolerance", &optionValue ) == QPDUNES_TRUE )
		options->newtonHessDiagRegTolerance = (real_t)*optionValue;

	if ( getOptionValue( optionsPtr, "QPDUNES_ZERO", &optionValue ) == QPDUNES_TRUE )
		options->QPDUNES_ZERO = (real_t)*optionValue;

	if ( getOptionValue( optionsPtr, "QPDUNES_INFTY", &optionValue ) == QPDUNES_TRUE )
		options->QPDUNES_INFTY = (real_t)*optionValue;


	/* other options */
	if ( getOptionValue( optionsPtr, "checkForInfeasibility", &optionValue ) == QPDUNES_TRUE )
		options->checkForInfeasibility = (boolean_t)*optionValue;
	if ( getOptionValue( optionsPtr, "allowSuboptimalTermination", &optionValue ) == QPDUNES_TRUE )
		options->allowSuboptimalTermination = (boolean_t)*optionValue;


	/* regularization options */
	if ( getOptionValue( optionsPtr, "regType", &optionValue ) == QPDUNES_TRUE )
		options->regType = (nwtnHssnRegType_t)*optionValue;

	if ( getOptionValue( optionsPtr, "regParam", &optionValue ) == QPDUNES_TRUE )
		options->regParam = (real_t)*optionValue;


	/* line search options */
	if ( getOptionValue( optionsPtr, "lsType", &optionValue ) == QPDUNES_TRUE )
		options->lsType	= (lineSearchType_t)*optionValue;

	if ( getOptionValue( optionsPtr, "lineSearchReductionFactor", &optionValue ) == QPDUNES_TRUE )
		options->lineSearchReductionFactor = (real_t)*optionValue;

	if ( getOptionValue( optionsPtr, "lineSearchIncreaseFactor", &optionValue ) == QPDUNES_TRUE )
		options->lineSearchIncreaseFactor = (real_t)*optionValue;

	if ( getOptionValue( optionsPtr, "lineSearchMinAbsProgress", &optionValue ) == QPDUNES_TRUE )
		options->lineSearchMinAbsProgress = (real_t)*optionValue;

	if ( getOptionValue( optionsPtr, "lineSearchMinRelProgress", &optionValue ) == QPDUNES_TRUE )
		options->lineSearchMinRelProgress = (real_t)*optionValue;

	if ( getOptionValue( optionsPtr, "lineSearchStationarityTolerance", &optionValue ) == QPDUNES_TRUE )
		options->lineSearchStationarityTolerance = (real_t)*optionValue;

	if ( getOptionValue( optionsPtr, "lineSearchMaxStepSize", &optionValue ) == QPDUNES_TRUE )
		options->lineSearchMaxStepSize = (real_t)*optionValue;

	if ( getOptionValue( optionsPtr, "lineSearchNbrGridPoints", &optionValue ) == QPDUNES_TRUE )
		options->lineSearchNbrGridPoints = (int_t)*optionValue;


	return QPDUNES_OK;
}



/*
 *	m a k e N e w t o n H e s s i a n D e n s e
 */
return_t makeNewtonHessianDense( const qpData_t* const qpData,
							   	 real_t* const to,
							   	 const xn2x_matrix_t* const hessian
							  	 )
{
	int_t kk, ii, jj;

	int_t nI = qpData->nI;
	int_t nX = qpData->nX;

 	for( kk=0; kk<nI; ++kk ) {		/* block rows */
		for( ii=0; ii<nX; ++ii ) {	/* rows in block */
			for( jj=0; jj<(kk-1)*nX; ++jj ) {  				/* begin of row */
			/*     blocks rows   rows       cols     */
				to[kk*nX*(nI*nX)+ii*(nI*nX)+jj] = 0.;
			}
			for( jj=0; jj<(kk > 0 ? 1 : 0)*nX; ++jj ) {  	/* subdiagonal block */
				to[kk*nX*(nI*nX)+ii*(nI*nX)+(kk-1)*nX+jj] = accHessian( kk, -1, ii, jj );
			}
			for( jj=0; jj<nX; ++jj ) {						/* diagonal block */
				to[kk*nX*(nI*nX)+ii*(nI*nX)+kk*nX+jj] = accHessian( kk, 0, ii, jj );
			}
			for( jj=0; jj<(kk < nI-1 ? 1 : 0)*nX; ++jj ) {  /* superdiagonal block */
				to[kk*nX*(nI*nX)+ii*(nI*nX)+(kk+1)*nX+jj] = accHessian( kk+1, -1, jj, ii );
			}
			for( jj=(kk+2)*nX; jj<nI*nX; ++jj ) {  			/* remaining row */
				to[kk*nX*(nI*nX)+ii*(nI*nX)+jj] = 0.;
			}
		}
	}

 	return QPDUNES_OK;
}



/*
 *	m a k e C h o l N e w t o n H e s s i a n D e n s e
 */
return_t makeCholNewtonHessianDense( const qpData_t* const qpData,
							   	 real_t* const to,
							   	 const xn2x_matrix_t* const cholHessian
							  	 )
{
	int_t kk, ii, jj;

	int_t nI = qpData->nI;
	int_t nX = qpData->nX;

 	for( kk=0; kk<nI; ++kk ) {		/* block rows */
		for( ii=0; ii<nX; ++ii ) {	/* rows in block */
			for( jj=0; jj<(kk-1)*nX; ++jj ) {  				/* begin of row */
			/* write Fortran-style for Matlab:
			 *     block rows    rows       cols
			 *     block rows rows cols     */
				to[kk*nX     +ii  +jj*(nI*nX)] = 0.;
			}
			for( jj=0; jj<(kk > 0 ? 1 : 0)*nX; ++jj ) {  	/* subdiagonal block */
				to[kk*nX     +ii  +(kk-1)*nX*(nI*nX) +jj*(nI*nX)] = accCholHessian( kk, -1, ii, jj );
			}
			for( jj=0; jj<=ii; ++jj ) {						/* diagonal block: before diagonal */
				to[kk*nX     +ii  +kk*nX*(nI*nX) +jj*(nI*nX)] = accCholHessian( kk, 0, ii, jj );
			}
			for( jj=kk*nX+ii+1; jj<nI*nX; ++jj ) { 			/* remaining row */
				to[kk*nX     +ii  +jj*(nI*nX)] = 0.;
			}
		}
	}

 	return QPDUNES_OK;
}



/*
 *	c o n v e r t F o r t r a n T o C
 */
 return_t convertFortranToC( real_t* const M_C,
		 	 	 	 	 	 const real_t* const M_F,
		 	 	 	 	 	 int nR, 		/** number of Rows */
		 	 	 	 	 	 int nC			/** number of Columns */
		 	 	 	 	 	 )
 {
 	int_t rIdx,cIdx;

 	for ( rIdx=0; rIdx<nR; ++rIdx ) {
 		for ( cIdx=0; cIdx<nC; ++cIdx ) {
 			M_C[rIdx*nC+cIdx] = M_F[rIdx+cIdx*nR];	/* copy from Fortran-style column-major format to C-style row-major */
 		}
 	}

 	return QPDUNES_OK;
 }


/*
 *	M E A S U R E    T I M I N G S
 */
//#if (defined __WINDOWS__)
//
//void tic(timer* t)
//{
//	QueryPerformanceFrequency(&t->freq);
//	QueryPerformanceCounter(&t->tic);
//}
//
//real_t toc(timer* t)
//{
//	QueryPerformanceCounter(&t->toc);
//	return ((t->toc.QuadPart - t->tic.QuadPart) / (real_t)t->freq.QuadPart);
//}
//

//#elif (defined __APPLE__)
//
//void tic(timer* t)
//{
//    /* read current clock cycles */
//    t->tic = mach_absolute_time();
//}
//
//real_t toc(timer* t)
//{
//
//    uint64_t duration; /* elapsed time in clock cycles*/
//
//    t->toc = mach_absolute_time();
//    duration = t->toc - t->tic;
//
//    /*conversion from clock cycles to nanoseconds*/
//    mach_timebase_info(&(t->tinfo));
//    duration *= t->tinfo.numer;
//    duration /= t->tinfo.denom;
//
//    return (real_t)duration / 1e9;
//}

//#else
//
//
///* read current time */
//void tic(timer* t)
//{
//	gettimeofday(&t->tic, 0);
//}
//
///* return time passed since last call to tic on this timer */
//real_t toc(timer* t)
//{
//	struct timeval temp;
//
//	gettimeofday(&t->toc, 0);
//
//	if ((t->toc.tv_usec - t->tic.tv_usec) < 0)
//	{
//		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec - 1;
//		temp.tv_usec = 1000000 + t->toc.tv_usec - t->tic.tv_usec;
//	}
//	else
//	{
//		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec;
//		temp.tv_usec = t->toc.tv_usec - t->tic.tv_usec;
//	}
//
//	return (real_t)temp.tv_sec + (real_t)temp.tv_usec / 1e6;
//}
//#endif



 /*
 *	f u l l L o g g i n g
 */
void fullLogging( const qpData_t* const qpData, mxArray** const logPtr )
{
	int_t numIter = qpData->log.numIter;
	int_t nI = qpData->nI;
	int_t nX = qpData->nX;
	int_t nZ = qpData->nZ;

	mwSize dims[2] = { 1, numIter+1 };
	mwSize nbrOfStructFields = 11;
	const char *field_names[] = {"lambda", "deltaLambda",
								  "hessian", "cholHessian", "invHessian", "gradient",
								  "z", "zUnconstrained", "dz",
								  "y", "ieqStatus" };
	*logPtr = mxCreateStructArray(2, dims, nbrOfStructFields, field_names);

	int_t lambdaIdx 		= mxGetFieldNumber(*logPtr,"lambda");
	int_t deltaLambdaIdx 	= mxGetFieldNumber(*logPtr,"deltaLambda");
	int_t gradientIdx 		= mxGetFieldNumber(*logPtr,"gradient");
	int_t hessianIdx 		= mxGetFieldNumber(*logPtr,"hessian");
	int_t cholHessianIdx 	= mxGetFieldNumber(*logPtr,"cholHessian");
	int_t invHessianIdx 	= mxGetFieldNumber(*logPtr,"invHessian");
	int_t zIdx				= mxGetFieldNumber(*logPtr,"z");
	int_t zUnconstrainedIdx = mxGetFieldNumber(*logPtr,"zUnconstrained");
	int_t dzIdx 			= mxGetFieldNumber(*logPtr,"dz");
	int_t yIdx 				= mxGetFieldNumber(*logPtr,"y");
	int_t ieqStatusIdx 		= mxGetFieldNumber(*logPtr,"ieqStatus");

	/* Copy data */
	for( int ii=0; ii<=numIter; ++ii ) {
		mxArray *dataPtr;


		/* lambda */
		dataPtr = mxCreateDoubleMatrix(nI*nX,1,mxREAL);												/* allocate array */
		qpDUNES_copyArray( mxGetPr( dataPtr ), qpData->log.itLog[ii].lambda.data, nI*nX );	/* copy data to array */
		mxSetFieldByNumber( *logPtr, ii,lambdaIdx, dataPtr );										/* pass to struct */

		/* deltaLambda */
		dataPtr = mxCreateDoubleMatrix(nI*nX,1,mxREAL);														/* allocate array */
		qpDUNES_copyArray( mxGetPr( dataPtr ), qpData->log.itLog[ii].deltaLambda.data, nI*nX );	/* copy data to array */
		mxSetFieldByNumber( *logPtr, ii,deltaLambdaIdx, dataPtr );										/* pass to struct */


		/* gradient */
		dataPtr = mxCreateDoubleMatrix(nI*nX,1,mxREAL);												/* allocate array */
		qpDUNES_copyArray( mxGetPr( dataPtr ), qpData->log.itLog[ii].gradient.data, nI*nX );	/* copy data to array */
		mxSetFieldByNumber( *logPtr, ii,gradientIdx, dataPtr );										/* pass to struct */

		/* hessian */
		dataPtr = mxCreateDoubleMatrix(nI*nX,nI*nX,mxREAL);																/* allocate array */
		makeNewtonHessianDense( qpData, (real_t*)mxGetPr( dataPtr ), &(qpData->log.itLog[ii].hessian) );	/* copy data to array */
		mxSetFieldByNumber( *logPtr, ii,hessianIdx, dataPtr );															/* pass to struct */

		/* cholHessian */
		dataPtr = mxCreateDoubleMatrix(nI*nX,nI*nX,mxREAL);																	/* allocate array */
		makeCholNewtonHessianDense( qpData, (real_t*)mxGetPr( dataPtr ), &(qpData->log.itLog[ii].cholHessian) );	/* copy data to array */
		mxSetFieldByNumber( *logPtr, ii,cholHessianIdx, dataPtr );															/* pass to struct */

		#if defined(__ANALYZE_FACTORIZATION__)
		/* invHessian */
		dataPtr = mxCreateDoubleMatrix(nI*nX,nI*nX,mxREAL);																	/* allocate array */
		qpDUNES_copyArray( mxGetPr( dataPtr ), qpData->log.itLog[ii].invHessian.data, nI*nX*nI*nX );				/* copy data to array */
		mxSetFieldByNumber( *logPtr, ii,invHessianIdx, dataPtr );															/* pass to struct */
		#endif

		/* z */
		dataPtr = mxCreateDoubleMatrix(nI*nZ+nX,1,mxREAL);											/* allocate array */
		qpDUNES_copyArray( mxGetPr( dataPtr ), qpData->log.itLog[ii].dz.data, nI*nZ+nX );	/* copy data to array */
		mxSetFieldByNumber( *logPtr, ii,dzIdx, dataPtr );											/* pass to struct */

		/* zUnconstrained */
		dataPtr = mxCreateDoubleMatrix(nI*nZ+nX,1,mxREAL);														/* allocate array */
		qpDUNES_copyArray( mxGetPr( dataPtr ), qpData->log.itLog[ii].zUnconstrained.data, nI*nZ+nX );	/* copy data to array */
		mxSetFieldByNumber( *logPtr, ii,zUnconstrainedIdx, dataPtr );											/* pass to struct */

		/* dz */
		dataPtr = mxCreateDoubleMatrix(nI*nZ+nX,1,mxREAL);											/* allocate array */
		qpDUNES_copyArray( mxGetPr( dataPtr ), qpData->log.itLog[ii].z.data, nI*nZ+nX );		/* copy data to array */
		mxSetFieldByNumber( *logPtr, ii,zIdx, dataPtr );											/* pass to struct */


		/* y */
		dataPtr = mxCreateDoubleMatrix(qpData->nDttl,2,mxREAL);											/* allocate array */
		qpDUNES_copyArray( mxGetPr( dataPtr ), qpData->log.itLog[ii].y.data, 2*qpData->nDttl );	/* copy data to array */
		mxSetFieldByNumber( *logPtr, ii,yIdx, dataPtr );															/* pass to struct */

//		/* ieqStatus */
//		int nDmax = -1;
//		for (int kk=0; kk<nI; ++kk) {
//			if (qpData->intervals[kk]->nD > nDmax)	nDmax = qpData->intervals[kk]->nD;
//		}
//		dataPtr = mxCreateDoubleMatrix(nI,nDmax+nZ,mxREAL);							/* allocate array */
//		double* data = mxGetPr( dataPtr );
//		for( int kk=0; kk<nI; ++kk ) {											/* copy data to array */
//			for( int jj=0; jj<qpData->intervals[kk]->nD; ++jj ) {											/* copy data to array */
//				data[kk*nDmax+jj] = (double)qpData->log.itLog[ii].ieqStatus[kk][jj];							/* cast to double while copying */
//			}
//		}
//		mxSetFieldByNumber( *logPtr, ii,ieqStatusIdx, dataPtr );									/* pass to struct */
	}

	return;
}



/*
 *	end of file
 */
