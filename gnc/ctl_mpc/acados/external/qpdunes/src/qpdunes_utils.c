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
 *	\file src/utils.c
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */


#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "qp/qpdunes_utils.h"


/* ----------------------------------------------
 * safe free routine
 * 
 > >>>>>                  *                         */
void qpDUNES_free(	real_t** data
				)
{
	if ( *data != 0 )
	{
		free( *data );
		*data = 0;
	}
}
/*<<< END OF qpDUNES_free */


/* ----------------------------------------------
 * safe free routine
 *
 > >>>>>                  *                         */
void qpDUNES_intFree(	int_t** data
					)
{
	if ( *data != 0 )
	{
		free( *data );
		*data = 0;
	}
}
/*<<< END OF qpDUNES_intFree */



/* ----------------------------------------------
 * safe array offset routine, avoids NULL
 * pointer offsetting
 *
>>>>>>                                            */
real_t const* offsetArray(	const real_t* const data,
							int_t offset
							)
{
	return (data != 0) ? &(data[offset]) : 0;
}
/*<<< END OF offsetArray */


/* ----------------------------------------------
 * safe array offset routine for int arrays,
 * avoids NULL pointer offsetting
 *
>>>>>>                                            */
const int_t* offsetIntArray(	const int_t* const data,
								int_t offset
								)
{
	return (data != 0) ? &(data[offset]) : 0;
}
/*<<< END OF offsetIntArray */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
void qpDUNES_copyRealArray( int_t n,
						 real_t* const to,
						 const real_t* const from
						 )
{
	int_t i;
	
	if ( from != 0 )
	{
		for( i=0; i<n; ++i )
			to[i] = from[i];
	}
}
/*<<< END OF qpDUNES_copyRealArray */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
sparsityType_t qpDUNES_detectMatrixSparsity(	const real_t* const M,
												int_t nRows,
												int_t nCols
												)
{
	sparsityType_t sparsityM;
	int_t i,j;
	
	if ( ( nRows < 1 ) || ( nCols < 1 ) || ( M == 0 ) )
		return sparsityM;
	
	if ( nRows != nCols )
	{
		sparsityM = QPDUNES_DENSE;
		return sparsityM;
	}
	
	/* check for sparsity */
	sparsityM = QPDUNES_DIAGONAL;
	
	for( i=0; i<nRows; ++i ) {	/* check if dense */
		for( j=0; j<i-1; ++j ) {	/* lower triangle */
			if ( fabs( M[i*nCols+j] ) > 1e-15 ) {	/* TODO: make threshold adjustable! */
				sparsityM = QPDUNES_DENSE;
				break;
			}
		}
		for( j=i+1; j<nCols; ++j ) {	/* upper triangle */
			if ( fabs( M[i*nCols+j] ) > 1e-15 ) {
				sparsityM = QPDUNES_DENSE;
				break;
			}
		}
	}
	
	/* check whether diagonal or identity */
	if ( sparsityM != QPDUNES_DENSE )
	{
		sparsityM = QPDUNES_IDENTITY;
		
		for( i=0; i<nRows; ++i ) {
			if ( fabs( M[i*nCols+i] - 1.0 ) > 1e-15 ) {
				sparsityM = QPDUNES_DIAGONAL;
				break;
			}
		}
	}
	
	return sparsityM;
}
/*<<< END OF qpDUNES_detectMatrixSparsity */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t qpDUNES_updateMatrixData(	matrix_t* const to,
									const real_t* const from,
									int_t nRows,
									int_t nCols
									)
{
	int_t i;
	
	if ( from == 0 )
		return QPDUNES_OK;
	
	if ( to == 0 )
		return QPDUNES_ERR_INVALID_ARGUMENT;

	switch ( to->sparsityType )
	{
		case QPDUNES_DENSE:
			for( i=0; i<nRows*nCols; ++i )
				to->data[i] = from[i];
			break;
			
		case QPDUNES_DIAGONAL:
			for( i=0; i<nRows; ++i )
				to->data[i] = from[i*nCols+i];
			break;
			
		case QPDUNES_IDENTITY:
			break;
			
		default:
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
	
	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_updateMatrixData */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t qpDUNES_setupZeroMatrix(	int_t nRows,
								int_t nCols,
								matrix_t* to
								)
{
	int_t i;
	
	for( i=0; i<nRows*nCols; ++i )
		to->data[i] = 0.0;

	to->sparsityType = QPDUNES_ALLZEROS;
	
	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_setupZeroMatrix */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t qpDUNES_setMatrixNull(	matrix_t* const matrix
								)
{
	qpDUNES_free( &(matrix->data) );
	matrix->sparsityType = QPDUNES_MATRIX_UNDEFINED;
	
	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_setMatrixNull */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
boolean_t qpDUNES_existsMatrix(	matrix_t* matrix
							)
{
	if( matrix->data == 0 ) {
		return QPDUNES_FALSE;
	}
	else {
		return QPDUNES_TRUE;
	}
}
/*<<< END OF qpDUNES_existsMatrix */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
boolean_t qpDUNES_existsVector(	vector_t* vector
							)
{
	if( vector->data == 0 ) {
		return QPDUNES_FALSE;
	}
	else {
		return QPDUNES_TRUE;
	}
}
/*<<< END OF qpDUNES_existsVector */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t qpDUNES_setupIdentityMatrix(	matrix_t* to
									)
{
	to->sparsityType = QPDUNES_IDENTITY;

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_setupIdentityMatrix */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t qpDUNES_setupScaledIdentityMatrix(	int_t nRows,
											real_t scalar,
											matrix_t* to
											)
{
	int_t i;
	
	qpDUNES_setupZeroMatrix( nRows,nRows,to );
	
	for( i=0; i<nRows; ++i )
		to->data[i*nRows+i] = scalar;
	
	to->sparsityType = QPDUNES_DIAGONAL;

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_setupScaledIdentityMatrix */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t qpDUNES_setupVector(	vector_t* const to,
							const real_t* const from,
							int_t n
							)
{
	return qpDUNES_updateVector( to,from,n );
}
/*<<< END OF qpDUNES_setupVector */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t qpDUNES_updateVector(	vector_t* const to,
							const real_t* const from,
							int_t n
							)
{
	int_t i;

	if ( ( n < 1 ) || ( from == 0 ) )
		return QPDUNES_OK;
	
	if ( to == 0 )
		return QPDUNES_ERR_INVALID_ARGUMENT;

	/* copy data */
	for( i=0; i<n; ++i ) {
		to->data[i] = from[i];
	}

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_updateVector */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t qpDUNES_updateSimpleBoundVector(	qpData_t* qpData,
										vector_t* const to,
										const real_t* const dBnd,
										const real_t* const xBnd,
										const real_t* const uBnd
										)
{
	int_t i;
	
	if ( dBnd != 0 ) {
		for( i=0; i<_NX_+_NU_; ++i ) {
			to->data[i] = dBnd[i];
		}
	}
	else {
		if ( xBnd != 0 ) {
			for( i=0; i<_NX_; ++i ) {
				to->data[i] = xBnd[i];
			}
		}
		if ( uBnd != 0 ) {
			for( i=0; i<_NU_; ++i ) {
				to->data[_NX_+i] = uBnd[i];
			}
		}
	}

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_updateSimpleBoundVector */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t qpDUNES_updateConstraintVector( 	vector_t* const to,
										const real_t* const dBnd,
										int_t nD
										)
{
	int_t i;
	
	if ( dBnd != 0 ) {
		for( i=0; i<nD; ++i ) {
			to->data[i] = dBnd[i];
		}
	}
	else {
		return QPDUNES_ERR_INVALID_ARGUMENT;
	}

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_updateConstraintVector */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t qpDUNES_setupZeroVector(	vector_t* const to,
								int_t n
								)
{
	int_t i;
	
	for( i=0; i<n; ++i )
		to->data[i] = 0.0;

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_setupZeroVector */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t qpDUNES_setupUniformVector(	vector_t* const to,
										real_t value,
										int_t n
										)
{
	int_t i;
	
	for( i=0; i<n; ++i ) {
		to->data[i] = value;
	}

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_setupUniformVector */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t qpDUNES_copyVector(	vector_t* const to,
								const vector_t* const from,
								int_t n
								)
{
	int_t ii;
	
	for( ii=0; ii<n; ++ii )
		to->data[ii] = from->data[ii];

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_copyVector */



/* ----------------------------------------------
 * deep matrix copy
 *
 >>>>>                                            */
return_t qpDUNES_copyMatrix(	matrix_t* const to,
								const matrix_t* const from,
								int_t dim0,
								int_t dim1
								)
{
	int_t ii;
	
	/** choose appropriate copy routine */
	switch( from->sparsityType )	
	{
		case QPDUNES_DENSE		:
		case QPDUNES_SPARSE	:
			for( ii=0; ii < dim0*dim1; ++ii ) {
				to->data[ii] = from->data[ii];
			}
			to->sparsityType = QPDUNES_DENSE;
			break;
		case QPDUNES_DIAGONAL	:
			/* matrix diagonal is saved in first line */
			for( ii=0; ii < dim1; ++ii ) {
				to->data[ii] = from->data[ii];
			}
			to->sparsityType = QPDUNES_DIAGONAL;
			break;
		case QPDUNES_IDENTITY	:
			to->sparsityType = QPDUNES_IDENTITY;
			break;
		default				:
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
	
	return QPDUNES_OK;
}
/*<<<< END OF qpDUNES_copyMatrix */


/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t qpDUNES_makeMatrixDense( 	matrix_t* const M_ptr, 
									int_t dim0,
									int_t dim1
									)
{
	int_t ii, jj;
	
	real_t* M = M_ptr->data; /* enable matrix access by preprocessor macro */
	
	switch( M_ptr->sparsityType )	
	{
		case QPDUNES_DENSE		:
			break;
		
		case QPDUNES_SPARSE	:
			//qpDUNES_printWarning( __FILE__, __LINE__, "Sparse to dense Matrix conversion not implemented. Assuming matrix is already dense." );
			M_ptr->sparsityType = QPDUNES_DENSE;
			break;
		
		case QPDUNES_DIAGONAL	:
			/* matrix diagonal is saved in first line */
			for( ii=dim0-1; ii >= 0; --ii ) {	/* go through matrix back to front */
				for( jj=dim1-1; jj > ii; --jj ) {
					accM( ii,jj,dim1 ) = 0.;
				}
				accM( ii,ii,dim1 ) = accM( 0,ii,dim1 );
				for( jj=ii-1; jj >= 0; --jj ) {
					accM( ii,jj,dim1 ) = 0.;
				}
			}
			M_ptr->sparsityType = QPDUNES_DENSE;
			break;
		
		case QPDUNES_IDENTITY	:
			for( ii=dim0-1; ii >= 0; --ii ) {	/* go through matrix back to front */
				for( jj=dim1-1; jj > ii; --jj ) {
					accM( ii,jj,dim1 ) = 0.;
				}
				accM( ii,ii,dim1 ) = 1.;
				for( jj=ii-1; jj >= 0; --jj ) {
					accM( ii,jj,dim1 ) = 0.;
				}
			}
			M_ptr->sparsityType = QPDUNES_DENSE;
			break;
		
		default				:
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
	
	return QPDUNES_OK;
}
/*<<<< END OF qpDUNES_makeMatrixDense */



/* ----------------------------------------------
 * transpose matrix (deep copy)
 *
 >>>>>                                            */
return_t qpDUNES_transposeMatrix(	matrix_t* const to,
								const matrix_t* const from,
								int_t dim0,
								int_t dim1
								)
{
	int_t ii,jj;
	
	/** choose appropriate copy routine */
	switch( from->sparsityType )	
	{
		case QPDUNES_DENSE		:
			to->sparsityType = QPDUNES_DENSE;
			for( ii=0; ii < dim1; ++ii ) {	/* go by columns of from matrix */
				for( jj=0; jj < dim0; ++jj ) {
					to->data[ii*dim0+jj] = from->data[jj*dim1+ii];
				}
			}
			break;
		
		case QPDUNES_SPARSE	:
			//qpDUNES_printWarning( __FILE__, __LINE__, "Sparse tranposeMatrix not implemented. Copying densely instead." );
			to->sparsityType = QPDUNES_DENSE;
			for( ii=0; ii < dim1; ++ii ) {	/* go by columns of from matrix */
				for( jj=0; jj < dim0; ++jj ) {
					to->data[ii*dim0+jj] = from->data[jj*dim1+ii];
				}
			}
			break;
		
		case QPDUNES_DIAGONAL	:
			break;
		
		case QPDUNES_IDENTITY	:
			to->sparsityType = QPDUNES_IDENTITY;
			break;
		
		default				:
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
	
	return QPDUNES_OK;
}
/*<<<< END OF qpDUNES_transposeMatrix */



/* ----------------------------------------------
 * selftranspose a square matrix
 *
 >>>>>                                            */
return_t qpDUNES_selftransposeMatrix(	matrix_t* const Mptr,
									int_t dim			/**< leading and secondary dimension of M */
									)
{
	int_t ii,jj;

	real_t swap;
	real_t* M = Mptr->data;

	/** choose appropriate copy routine */
	switch( Mptr->sparsityType )
	{
		case QPDUNES_DENSE		:
		case QPDUNES_SPARSE	:
			Mptr->sparsityType = QPDUNES_DENSE;
			for( ii=0; ii < dim; ++ii ) {	/* go by rows of untransposed M in lower triangle */
				for( jj=0; jj < ii; ++jj ) {
					swap = accM(ii,jj,dim);
					accM(ii,jj,dim) = accMT(ii,jj,dim);
					accMT(ii,jj,dim) = swap;
				}
			}
			break;

		case QPDUNES_DIAGONAL	:
		case QPDUNES_IDENTITY	:
			break;

		default				:
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}

	return QPDUNES_OK;
}
/*<<<< END OF qpDUNES_selftransposeMatrix */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
return_t qpDUNES_copyArray(	real_t* const to,
							const real_t* const from,
							int_t n
							)
{
	int_t ii;
	
	for( ii=0; ii<n; ++ii )
		to[ii] = from[ii];
	
	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_copyArray */



/* TODO: move code of inline functions to header file, only leave extern here */
/* ----------------------------------------------
 * max routine
 * 
 >>>>>                                            */
//extern inline int_t qpDUNES_max(	int_t a,
int_t qpDUNES_max(	int_t a,
						int_t b )
{
	return (a > b) ? a : b;
}
/*<<< END OF qpDUNES_max */



/* ----------------------------------------------
 * min routine
 >>>>>                                            */
//extern inline int_t qpDUNES_min(	int_t a,
int_t qpDUNES_min(	int_t a,
						int_t b )
{
	return (a < b) ? a : b;
}
/*<<< END OF qpDUNES_min */



/* ----------------------------------------------
 * max routine
 >>>>>                                            */
//extern inline real_t qpDUNES_fmax(	real_t a,
real_t qpDUNES_fmax(	real_t a,
							real_t b )
{
	return (a > b) ? a : b;
}
/*<<< END OF qpDUNES_fmax */



/* ----------------------------------------------
 * min routine
 >>>>>                                            */
//extern inline real_t qpDUNES_fmin(	real_t a,
real_t qpDUNES_fmin(	real_t a,
						real_t b )
{
	return (a < b) ? a : b;
}
/*<<< END OF qpDUNES_fmin */



/* ----------------------------------------------
 * sign routine
 >>>>>                                            */
//extern inline int_t qpDUNES_sign(	const qpData_t* const qpData,
int_t qpDUNES_sign(	const qpData_t* const qpData,
					real_t a
					)
{
	return (a < -qpData->options.equalityTolerance) ? -1 : ( (a > qpData->options.equalityTolerance) ? 1 : 0 );
}
/*<<< END OF qpDUNES_sign */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
//extern inline void qpDUNES_assertOK(	return_t statusFlag,
void qpDUNES_assertOK(	return_t statusFlag,
						char* fileName,
						int_t lineNumber,
						char* errString 
						)
{
	#ifdef USE_ASSERTS
	if ( statusFlag != QPDUNES_OK ) {
		qpDUNES_printError( fileName, lineNumber, errString );
	}
	#endif
}
/*<<< END OF qpDUNES_assertOK */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
real_t getTime(  ){
	#ifdef __MEASURE_TIMINGS__
		real_t current_time = 0.0;
		struct timeval theclock;
		gettimeofday( &theclock,0 );
		current_time = 1.0*theclock.tv_sec + 1.0e-6*theclock.tv_usec;
		return current_time;
	#else
		return 0.0;
	#endif
}
/*<<< END OF getTime */



/* -------------------------------------------------------------
 * P R I N T I N G     R O U T I N E S
 * ------------------------------------------------------------- */


/* ----------------------------------------------
 * low-level printing routine
 >>>>>                                            */
void qpDUNES_printStrArgs(	const char* const string,
							...
							)
{
	#ifndef __SUPPRESS_ALL_OUTPUT__
	/* get printf arguments list */
	va_list printArgs;
	va_start( printArgs, string );
	
	/* print output */
	#ifdef __MATLAB__
		char buffer[MAX_STR_LEN];
		vsprintf(buffer,string, printArgs);
		va_end( printArgs );

		mexPrintf( "%s", buffer );
	#else
		vprintf( string, printArgs );
		va_end( printArgs );
	#endif
	
	#endif /* __SUPPRESS_ALL_OUTPUT__ */
}
/*<<< END OF qpDUNES_printStrArgs */



/* ----------------------------------------------
 * print to file (low level)
 >>>>>                                            */
void qpDUNES_printStrArgsToFile(	FILE* filePtr,
									const char* const string,
									...
									)
{
	#ifndef __SUPPRESS_ALL_OUTPUT__
	va_list printArgs;
	va_start( printArgs, string );		/* get printf arguments list */

	vfprintf( filePtr, string, printArgs );	/* print output */

	va_end( printArgs );	/* close arguments */
	#endif /* __SUPPRESS_ALL_OUTPUT__ */
}
/*<<< END OF qpDUNES_printStrArgsToFile */



/* ----------------------------------------------
 * low-level printing routine
 >>>>>                                            */
void qpDUNES_printStrArgsList(	const char* const string,
								va_list printArgs
								)
{
	#ifndef __SUPPRESS_ALL_OUTPUT__

	/* print output */
	#ifdef __MATLAB__
		char buffer[MAX_STR_LEN];
		vsprintf(buffer,string, printArgs);
		mexPrintf( "%s", buffer );

	#else
		vprintf( string, printArgs );
	#endif

	#endif /* __SUPPRESS_ALL_OUTPUT__ */
}
/*<<< END OF qpDUNES_printStrArgsList */



/* ----------------------------------------------
 * print to file (low level)
 >>>>>                                            */
void qpDUNES_printStrArgsListToFile(	FILE* filePtr,
										const char* const string,
										va_list printArgs
										)
{
	#ifndef __SUPPRESS_ALL_OUTPUT__
	vfprintf( filePtr, string, printArgs );	/* print output */
	#endif /* __SUPPRESS_ALL_OUTPUT__ */
}
/*<<< END OF qpDUNES_printStrArgsListToFile */



/* ----------------------------------------------
 * customizable printing routine
 >>>>>                                            */
void qpDUNES_printf(	const char* const string,
						...
						)
{
	/* get printf arguments list */
	va_list printArgs;
	va_start( printArgs, string );
	
	/* print output, add newline */
	qpDUNES_printStrArgsList( string, printArgs );
	qpDUNES_printStrArgs( "\n" );
	
	va_end( printArgs );
}
/*<<< END OF qpDUNES_printf */



/* ----------------------------------------------
 * customizable printing routine without "\n" at the end
 >>>>>                                            */
void qpDUNES_printf_noNewLine(	const char* const string,
								...
								)
{
	/* get printf arguments list */
	va_list printArgs;
	va_start( printArgs, string );

	/* print output, add newline */
	qpDUNES_printStrArgsList( string, printArgs );

	va_end( printArgs );
}
/*<<< END OF qpDUNES_printf */



/* ----------------------------------------------
 * ...
 >>>>>                                            */
void qpDUNES_printSuccess( 	const qpData_t* const qpData,
							const char* const string,
							...
							)
{
	/* get printf arguments list */
	va_list printArgs;
	va_start( printArgs, string );

	/* print green output */
	if( qpData->options.printLevel >= 1 ) {
		qpDUNES_printStrArgs( "[qpDUNES] %s", COL_SUCC );
		qpDUNES_printStrArgsList( string, printArgs );
		qpDUNES_printStrArgs( "%s\n\n", COL_STD );
	}

	va_end( printArgs );
}
/*<<< END OF qpDUNES_printSuccess */



/* ----------------------------------------------
 * ...
 * 
 > >>>>                                            */
void qpDUNES_printWarning(	const qpData_t* const qpData,
							const char* const fileName,
							const int_t lineNumber,
							const char* const string )
{
	#if !(defined __SUPPRESS_ALL_WARNINGS__)
	if( qpData->options.printLevel >= 2 ) {
		qpDUNES_printStrArgs( "[qpDUNES] %s", COL_WARN );
		qpDUNES_printStrArgs( "WARNING in %s:%d: \n          %s",fileName, lineNumber, COL_STD );
		qpDUNES_printStrArgs( "%s\n", string );
	}
	#endif
}
/*<<< END OF qpDUNES_printWarning */



/* ----------------------------------------------
 * ...
 * 
 >>>>>                                            */
void qpDUNES_printError(	const qpData_t* const qpData,
							const char* const fileName,
							const int_t lineNumber,
							const char* const errString,
							...
							)
{
	/* get printf arguments list */
	va_list printArgs;
	va_start( printArgs, errString );
	
	/* print red output */
	if( qpData->options.printLevel >= 1 ) {
		qpDUNES_printStrArgs( "[qpDUNES] %s", COL_ERR );
		qpDUNES_printStrArgs( "ERROR in %s:%d: %s\n          ", fileName, lineNumber, COL_STD );
		qpDUNES_printStrArgsList( errString, printArgs );
		qpDUNES_printStrArgs( "\n" );
	}
	
	va_end( printArgs );
}
/*<<< END OF qpDUNES_printError */



/* ----------------------------------------------
 * ...
 * 
 >>>>>                                            */
void qpDUNES_printDebugInfo( const char* const string )
{
	qpDUNES_printf( string );
}
/*<<< END OF qpDUNES_printDebugInfo */



/* ----------------------------------------------
 * ...
 * 
 >>>>>                                            */
void qpDUNES_printMatrixData(	const real_t* const M,
								const int_t dim0,
								const int_t dim1,
								const char* const string,
								...
								)
{
	int_t ii, jj;
	
	/* get printf arguments list */
	va_list printArgs;
	va_start( printArgs, string );
	
	/* print */
	qpDUNES_printStrArgsList( string, printArgs );
	qpDUNES_printStrArgs( "\n[" );
	
	for( ii=0; ii<dim0; ++ii ) {
		qpDUNES_printStrArgs( "[" );
		for( jj=0; jj<dim1; ++jj ) {
			qpDUNES_printStrArgs( "% .*e,\t", PRINTING_PRECISION, (real_t)accM( ii, jj, dim1 ) );	/* TODO: use variable from qpOptions Struct; currently defined in types.h */
		}
		qpDUNES_printStrArgs( "]" );
		if(ii<dim0-1) {
			qpDUNES_printStrArgs( "\n" );
		}
	}
	
	qpDUNES_printStrArgs( "]\n" );
	
	va_end( printArgs );
}
/*<<< END OF qpDUNES_printMatrix */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
void qpDUNES_printIntMatrixData(	const int_t* const M,
									const int_t dim0,
									const int_t dim1,
									const char* const string,
									...
									)
{
	int_t ii, jj;

	/* get printf arguments list */
	va_list printArgs;
	va_start( printArgs, string );

	/* print */
	qpDUNES_printStrArgsList( string, printArgs );
	qpDUNES_printStrArgs( "\n[" );

	for( ii=0; ii<dim0; ++ii ) {
		qpDUNES_printStrArgs( "[" );
		for( jj=0; jj<dim1; ++jj ) {
			qpDUNES_printStrArgs( "% *d\t", PRINTING_PRECISION, (int_t)accM( ii, jj, dim1 ) );	/* TODO: use variable from qpOptions Struct; currently defined in types.h */
		}
		qpDUNES_printStrArgs( "]" );
		if(ii<dim0-1) {
			qpDUNES_printStrArgs( "\n" );
		}
	}

	qpDUNES_printStrArgs( "]\n" );

	va_end( printArgs );
}
/*<<< END OF qpDUNES_printMatrix */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
void qpDUNES_printMatrixDataToFile(	const real_t* const M,
									const int_t dim0,
									const int_t dim1,
									char* fileName,
									const char* const varNameString,
									...
									)
{
	int_t ii, jj;

	/* get printf arguments list */
	va_list printArgs;
	FILE* filePtr;

	va_start( printArgs, varNameString );
	filePtr=fopen(fileName, "a");		/* access file */


	/* print */
	qpDUNES_printStrArgsListToFile( filePtr, varNameString, printArgs );
	qpDUNES_printStrArgsToFile( filePtr, "\n" );

	for( ii=0; ii<dim0; ++ii ) {
		qpDUNES_printStrArgsToFile( filePtr, "" );
		for( jj=0; jj<dim1; ++jj ) {
			qpDUNES_printStrArgsToFile( filePtr, "% .*e\t", PRINTING_PRECISION, (real_t)accM( ii, jj, dim1 ) );	/* TODO: use variable from qpOptions Struct; currently defined in types.h */
		}
		qpDUNES_printStrArgsToFile( filePtr, "\n" );
	}

	qpDUNES_printStrArgsToFile( filePtr, "\n" );


	fclose(filePtr);		/* close file */
	va_end( printArgs );
}
/*<<< END OF qpDUNES_printMatrixDataToFile */



/* ----------------------------------------------
 * ...
 * 
 > >>>>                          *                  */
void qpDUNES_printVectorData(	const real_t* const x,
								const int_t dim0,
								const char* const string,
								... 
								)
{
	int_t ii;
	
	/* get printf arguments list */
	va_list printArgs;
	va_start( printArgs, string );
	
	/* print */
	qpDUNES_printStrArgsList( string, printArgs );
	qpDUNES_printStrArgs( "\n[\n" );
	
	for( ii=0; ii<dim0; ++ii ) {
		qpDUNES_printStrArgs( "% .*e\n", PRINTING_PRECISION, x[ii] );
	}
	
	qpDUNES_printStrArgs( "]\n\n" );
	
	va_end( printArgs );
}
/*<<< END OF qpDUNES_printMatrix */



/* ----------------------------------------------
 * ...
 * 
 >>>>>                                            */
void qpDUNES_printVector( const char* const string )
{
//	qpDUNES_printWarning( __FILE__, __LINE__, "qpDUNES_printVector not yet implemented. Doing nothing." );
	qpDUNES_printf( "qpDUNES_printVector not yet implemented. Doing nothing." );
}
/*<<< END OF qpDUNES_printVector */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
void qpDUNES_printNewtonHessian(	const qpData_t* const qpData,
									const xn2x_matrix_t* const hessian
									)
{
	int_t ii, jj, kk;

	qpDUNES_printStrArgs( "NewtonHessian = ...\n" );
	qpDUNES_printStrArgs( "[\n" );

	for( kk=0; kk<_NI_; ++kk ) {
		for( ii=0; ii<_NX_; ++ii ) {
			qpDUNES_printStrArgs( "[" );
			for( jj=0; jj < ((kk-1) * (int)_NX_); ++jj ) {  /* begin of row */  /* need casting from uint_t to int_t to enable safe comparisons */
				qpDUNES_printStrArgs( " 0.0\t\t\t" );
			}
			for( jj=0; jj<(kk > 0 ? 1 : 0)*(int)_NX_; ++jj ) {  /* subdiagonal block */
				qpDUNES_printStrArgs( "% .*e\t", PRINTING_PRECISION, accHessian( kk, -1, ii, jj ) );	/* TODO: use variable from qpOptions Struct; currently defined in types.h */
			}
			for( jj=0; jj<_NX_; ++jj ) {	/* diagonal block */
				qpDUNES_printStrArgs( "% .*e\t", PRINTING_PRECISION, accHessian( kk, 0, ii, jj ) );	/* TODO: use variable from qpOptions Struct; currently defined in types.h */
			}
			for( jj=0; jj<(kk < _NI_-1 ? 1 : 0)*(int)_NX_; ++jj ) {  /* superdiagonal block */
				qpDUNES_printStrArgs( "% .*e\t", PRINTING_PRECISION, accHessian( kk+1, -1, jj, ii ) );	/* TODO: use variable from qpOptions Struct; currently defined in types.h */
			}
			for( jj=(kk+2)*_NX_; jj<_NI_*_NX_; ++jj ) {  /* remaining row */
				qpDUNES_printStrArgs( " 0.0\t\t\t" );
			}
			qpDUNES_printStrArgs( "]\n" );
		}
	}

	qpDUNES_printStrArgs( "]\n\n" );
}
/*<<< END OF qpDUNES_printNewtonHessian */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
void qpDUNES_printNewtonHessianToFile(	const qpData_t* const qpData,
										const xn2x_matrix_t* const hessian,
										const char* const fileName,
										const char* const variableName,
										...
										)
{
	int_t ii, jj, kk;

	va_list printArgs;

	FILE* filePtr;
	filePtr=fopen(fileName, "a");		/* access file */


	va_start( printArgs, variableName );
	qpDUNES_printStrArgsListToFile( filePtr, variableName, printArgs );
	qpDUNES_printStrArgsToFile( filePtr, "\n" );
	va_end( printArgs );

	for( kk=0; kk<_NI_; ++kk ) {
		for( ii=0; ii<_NX_; ++ii ) {
			qpDUNES_printStrArgsToFile( filePtr,  "" );
			for( jj=0; jj < ((kk-1) * (int)_NX_); ++jj ) {  /* begin of row */  /* need casting from uint_t to int_t to enable safe comparisons */
				qpDUNES_printStrArgsToFile( filePtr, " 0.0\t\t\t" );
			}
			for( jj=0; jj<(kk > 0 ? 1 : 0)*_NX_; ++jj ) {  /* subdiagonal block */
				qpDUNES_printStrArgsToFile( filePtr, "% .*e\t", PRINTING_PRECISION, accHessian( kk, -1, ii, jj ) );	/* TODO: use variable from qpOptions Struct; currently defined in types.h */
			}
			for( jj=0; jj<_NX_; ++jj ) {	/* diagonal block */
				qpDUNES_printStrArgsToFile( filePtr, "% .*e\t", PRINTING_PRECISION, accHessian( kk, 0, ii, jj ) );	/* TODO: use variable from qpOptions Struct; currently defined in types.h */
			}
			for( jj=0; jj<(kk < _NI_-1 ? 1 : 0)*_NX_; ++jj ) {  /* superdiagonal block */
				qpDUNES_printStrArgsToFile( filePtr, "% .*e\t", PRINTING_PRECISION, accHessian( kk+1, -1, jj, ii ) );	/* TODO: use variable from qpOptions Struct; currently defined in types.h */
			}
			for( jj=(kk+2)*_NX_; jj<_NI_*_NX_; ++jj ) {  /* remaining row */
				qpDUNES_printStrArgsToFile( filePtr, " 0.0\t\t\t" );
			}
			qpDUNES_printStrArgsToFile( filePtr, "\n" );
		}
	}

	qpDUNES_printStrArgsToFile( filePtr, "\n" );


	fclose(filePtr);		/* close file */
}
/*<<< END OF qpDUNES_printNewtonHessianToFile */



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
void qpDUNES_printCholNewtonHessian(	const qpData_t* const qpData,
										const xn2x_matrix_t* const cholHessian
										)
{
	int_t ii, jj, kk;

	qpDUNES_printStrArgs( "Cholesky factor of Newton Hessian\n" );
	qpDUNES_printStrArgs( "[\n" );

	for( kk=0; kk<_NI_; ++kk ) {
		for( ii=0; ii<_NX_; ++ii ) {
			qpDUNES_printStrArgs( "[" );
			for( jj=0; jj < ((kk-1) * (int)_NX_); ++jj ) {  /* begin of row */  /* need casting from uint_t to int_t to enable safe comparisons */
				qpDUNES_printStrArgs( "0.0\t\t\t" );
			}
			for( jj=0; jj<(kk > 0 ? 1 : 0)*_NX_; ++jj ) {  /* subdiagonal block */
				qpDUNES_printStrArgs( "%.*e\t", PRINTING_PRECISION, accCholHessian( kk, -1, ii, jj ) );	/* TODO: use variable from qpOptions Struct; currently defined in types.h */
			}
			for( jj=0; jj<=ii; ++jj ) {	/* diagonal block */
				qpDUNES_printStrArgs( "%.*e\t", PRINTING_PRECISION, accCholHessian( kk, 0, ii, jj ) );	/* TODO: use variable from qpOptions Struct; currently defined in types.h */
			}
			for( jj=kk*_NX_+ii+1; jj<_NI_*_NX_; ++jj ) {  /* remaining row */
				qpDUNES_printStrArgs( "0.0\t\t\t" );
			}
			qpDUNES_printStrArgs( "]\n" );
		}
	}

	qpDUNES_printStrArgs( "]\n\n" );
}
/*<<< END OF qpDUNES_printCholNewtonHessian */



/* ----------------------------------------------
 *  print qpDUNES header information
 *
 >>>>>                                            */
void qpDUNES_printHeader( qpData_t* qpData )
{
	if ( qpData->options.printLevel > 0 ) {
		qpDUNES_printStrArgs( "\nqpDUNES -- A DUal NEwton Strategy for convex quadratic programming.\n" );
		qpDUNES_printStrArgs( "Copyright (C) 2012-2014 by Janick Frasch and Hans Joachim Ferreau.\n" );
		qpDUNES_printStrArgs( "Developed within the Optimization in Engineering Center (OPTEC) at \n" );
		qpDUNES_printStrArgs( "KU Leuven, Belgium under supervision of Moritz Diehl. All rights\n" );
		qpDUNES_printStrArgs( "reserved.\n\n" );

		qpDUNES_printStrArgs( "qpDUNES is distributed under the terms of the GNU Lesser\n" );
		qpDUNES_printStrArgs( "General Public License 3 in the hope that it will be useful,\n" );
		qpDUNES_printStrArgs( "but WITHOUT ANY WARRANTY; without even the implied warranty of\n" );
		qpDUNES_printStrArgs( "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the\n" );
		qpDUNES_printStrArgs( "GNU Lesser General Public License for more details.\n\n" );
	}
}
/*<<< END OF qpDUNES_printHeader */



/*
 *	end of file
 */
