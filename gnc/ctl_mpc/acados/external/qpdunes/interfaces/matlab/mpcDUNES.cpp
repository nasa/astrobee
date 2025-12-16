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
 *	\file interfaces/matlab/mpcDUNES.cpp
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 *
 *	Interface for Matlab(R) that enables to call qpDUNES as a MEX function.
 *
 */


#include <setup_mpc.h>

#include <qpDUNES_matlab_utils.cpp>

#include "mex.h"
#include "matrix.h"
#include "string.h"


/* global pointer to MPC object */
static mpcProblem_t* mpcProblemGlobal = 0;



/*
 *	m p c D U N E S _ s a f e C l e a n u p M a t l a b
 */
//static void mpcDUNES_safeCleanupMatlab( mpcProblem_t** mpcProblemPtr
static void mpcDUNES_safeCleanupMatlab(
										 )
{
	if (mpcProblemGlobal != 0) {
		mpcDUNES_cleanup( mpcProblemGlobal );
		delete mpcProblemGlobal;
		mpcProblemGlobal = 0;
	}
}
/*<<< END OF mpcDUNES_cleanupMatlab */



/*
 *	m p c D U N E S _ s e t u p M a t l a b
 */
void mpcDUNES_setupMatlab( mpcProblem_t** mpcProblemPtr,
						  uint_t nI,
						  uint_t nX,
						  uint_t nU,
						  uint_t* nD,
						  const mxArray* const optionsPtr
						  )
{
	mpcDUNES_safeCleanupMatlab( );
	*mpcProblemPtr = new mpcProblem_t;
	qpOptions_t qpOptions = qpDUNES_setupDefaultOptions();

	/* set up user options */
    if ( optionsPtr != 0 ) {
    	qpDUNES_setupOptionsMatlab( &(qpOptions), optionsPtr );
    }

	/* allocate qpDUNES data */
	mpcDUNES_setup( *mpcProblemPtr, nI, nX, nU, nD, &(qpOptions) );

	/* register mpcDUNES memory for MATLAB-triggered clearing */
	mexAtExit(mpcDUNES_safeCleanupMatlab);
}
/*<<< END OF mpcDUNES_setupMatlab */


/*
 *	i n i t X U S t y l e
 */
void initXUStyle( int nrhs, const mxArray* const prhs[] )
{
	mexPrintf( "setupXUStyle...\n" );
	/* 0) VARIABLE DECLARATIONS: */
	return_t statusFlag;
	/* WARNING! x-u style inputs currently not possible for LTV systems */
	mexPrintf( "WARNING! x-u style inputs currently not possible for LTV systems.\n" );
	boolean_t isLTI = QPDUNES_TRUE;

	real_t* nIuser = 0;

	real_t* Q = 0;
	real_t* R = 0;

	real_t* P = 0;

	real_t* g = 0;


	real_t* c = 0;

	real_t* A_F = 0;
	mxArray* Bptr = 0;
	real_t* B_F = 0;

	mxArray* xLowPtr = 0;
	real_t* xLow = 0;
	mxArray* xUppPtr = 0;
	real_t* xUpp = 0;
	mxArray* uLowPtr = 0;
	real_t* uLow = 0;
	mxArray* uUppPtr = 0;
	real_t* uUpp = 0;

	mxArray* xRefPtr = 0;
	real_t* xRef = 0;
	mxArray* uRefPtr = 0;
	real_t* uRef = 0;

	const mxArray* optionsPtr = 0;

	/* II) GET MATLAB POINTERS */
	nIuser = (real_t*)  mxGetPr( prhs[0] );
	Q = (real_t*) mxGetPr( prhs[1] );
	R = (real_t*) mxGetPr( prhs[2] );
	P = (real_t*) mxGetPr( prhs[3] );
	A_F = (real_t*) mxGetPr( prhs[4] );
	Bptr = (mxArray*)prhs[5];	/* to get dimensions */
	c = (real_t*) mxGetPr( prhs[6] );
	xLowPtr = (mxArray*)prhs[7];
	xUppPtr = (mxArray*)prhs[8];
	uLowPtr = (mxArray*)prhs[9];
	uUppPtr = (mxArray*)prhs[10];
	xRefPtr = (mxArray*)prhs[11];
	uRefPtr = (mxArray*)prhs[12];
	if ( ( nrhs == 14 ) &&					/* check whether options are specified */
		 ( !mxIsEmpty(prhs[nrhs-1]) ) &&
		 ( mxIsStruct(prhs[nrhs-1]) ) )
	{
		optionsPtr = prhs[nrhs-1];
	}
	/* extract data */
	B_F = (real_t*) mxGetPr( Bptr );
	xLow = (real_t*) mxGetPr( xLowPtr );
	xUpp = (real_t*) mxGetPr( xUppPtr );
	uLow = (real_t*) mxGetPr( uLowPtr );
	uUpp = (real_t*) mxGetPr( uUppPtr );
	xRef = (real_t*) mxGetPr( xRefPtr );
	uRef = (real_t*) mxGetPr( uRefPtr );

	/* III) CONSISTENCY CHECKS */
	if ( nIuser == 0 )	mexErrMsgTxt( "ERROR (qpDUNES): Number of Intervals nI undefined!" );
	if ( Bptr == 0 )	mexErrMsgTxt( "ERROR (qpDUNES): B matrix undefined!" );


	/* 3) get dimensions */
	uint_t nI = (uint_t)(*nIuser);
	uint_t nColsH, nColsC;
	uint_t nX, nU, nZ;
	uint_t* nD = new uint_t[nI+1];

	nX = (uint_t) mxGetM( Bptr );	/* get number of rows */
	nU = (uint_t) mxGetN( Bptr );	/* get number of columns */
	nZ = nX+nU;

	for ( uint_t ii=0; ii<nI; ++ii ) {
		nD[ii] = nZ;
	}
	nD[nI] = nX;


	/* 4) check dimension consistency */
	if ( xLow != 0 ) {	/* xLow exists */
		if ( mxGetM( xLowPtr )*mxGetN( xLowPtr ) != (nI+1)*nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected xLow, but dimensions are inconsistent." );
	}
	if ( xUpp != 0 ) {	/* xUpp exists */
		if ( mxGetM( xUppPtr )*mxGetN( xUppPtr ) != (nI+1)*nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected xUpp, but dimensions are inconsistent." );
	}
	if ( uLow != 0 ) {	/* uLow exists */
		if ( mxGetM( uLowPtr )*mxGetN( uLowPtr ) != nI*nU )	mexErrMsgTxt( "ERROR (qpDUNES): Detected uLow, but dimensions are inconsistent." );
	}
	if ( uUpp != 0 ) {	/* uUpp exists */
		if ( mxGetM( uUppPtr )*mxGetN( uUppPtr ) != nI*nU )	mexErrMsgTxt( "ERROR (qpDUNES): Detected uUpp, but dimensions are inconsistent." );
	}
	if ( xRef != 0 ) {	/* xRef exists */
		if ( mxGetM( xRefPtr )*mxGetN( xRefPtr ) != (nI+1)*nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected xRef, but dimensions are inconsistent." );
	}
	if ( uRef != 0 ) {	/* uRef exists */
		if ( mxGetM( uRefPtr )*mxGetN( uRefPtr ) != nI*nU )	mexErrMsgTxt( "ERROR (qpDUNES): Detected uRef, but dimensions are inconsistent." );
	}


	/* III) ACTUALLY SET UP QP42 PROBLEM: */
	mpcDUNES_setupMatlab( &mpcProblemGlobal, nI, nX, nU, nD, optionsPtr );

	/* transpose matrices (Fortran to C) */
	/* only for A, B;
	 * Q, R, P should be symmetric;
	 * WARNING: if S matrix included, this has to be transposed as well! */
	real_t* A = 0;
	real_t* B = 0;
	A = new real_t[nX*nX];
	B = new real_t[nX*nU];
	convertFortranToC( A, A_F, nX, nX );
	convertFortranToC( B, B_F, nX, nU );


	/* setup Data */
	if (mpcProblemGlobal->qpData.options.printLevel >= 3) {
		if (!isLTI) {
			mexPrintf( "Detected LTV problem of size [nI = %d, nX = %d, nU = %d]\n", nI, nX, nU );
		}
		else {
			mexPrintf( "Detected LTI problem of size [nI = %d, nX = %d, nU = %d]\n", nI, nX, nU );
		}
	}

	/* depending on given problem data, either setup LTI or LTV problem */
	if (!isLTI) {
		mexErrMsgTxt( "Error (qpDUNES): x-u style inputs currently not possible for LTV systems.\n" );
	}
	else {
		statusFlag = mpcDUNES_initLtiSb_xu( mpcProblemGlobal, Q, R, 0, P, A, B, c, xLow, xUpp, uLow, uUpp, xRef, uRef );
	}
	if ( statusFlag != QPDUNES_OK ) {
		mexPrintf( "Problem setup returned error code %d", statusFlag );
		mexErrMsgTxt( "ERROR (qpDUNES): Problem setup failed!" );
	}


	/* clean up temporary variables */
	if( nD ) delete[] nD;
	if( A ) delete[] A;
	if( B ) delete[] B;

//	mexErrMsgTxt( "setupXUStyle done" );

	return;
}
/*<<< END OF initXUStyle */


/*
 *	i n i t Z S t y l e
 */
void initZStyle( int nrhs, const mxArray* const prhs[] )
{
	/* 0) VARIABLE DECLARATIONS: */
	return_t statusFlag;
	boolean_t inputsXuStyle;
	boolean_t isLTI;

	real_t* nIuser = 0;
	mxArray* Hptr = 0;
	real_t* H_in = 0;

	real_t* P = 0;

	mxArray* gPtr = 0;
	real_t* g = 0;

	mxArray* Cptr = 0;
	real_t* C_F = 0;

	real_t* c = 0;

	mxArray* zLowPtr = 0;
	real_t* zLow = 0;
	mxArray* zUppPtr = 0;
	real_t* zUpp = 0;

	mxArray* zRefPtr = 0;
	real_t* zRef = 0;

	const mxArray* optionsPtr = 0;


	/* I) CONSISTENCY CHECKS: */


	/* I) GET OUTPUT POINTERS CHECKS: */
	inputsXuStyle = QPDUNES_FALSE;
	nIuser = (real_t*)  mxGetPr( prhs[0] );
	Hptr = (mxArray*)prhs[1];	/* to get dimensions */
	P = (real_t*) mxGetPr( prhs[2] );
	gPtr = (mxArray*)prhs[3];	/* to check dimensions */
	g = (real_t*) mxGetPr( prhs[3] );
	Cptr = (mxArray*)prhs[4];	/* to get dimensions */
	c = (real_t*) mxGetPr( prhs[5] );
	zLowPtr = (mxArray*)prhs[6];
	zUppPtr = (mxArray*)prhs[7];
	zRefPtr = (mxArray*)prhs[8];
	if ( ( nrhs == 10 ) &&					/* check whether options are specified */
		 ( !mxIsEmpty(prhs[nrhs-1]) ) &&
		 ( mxIsStruct(prhs[nrhs-1]) ) )
	{
		optionsPtr = prhs[nrhs-1];
	}
	/* exatract data */
	H_in = (real_t*) mxGetPr( Hptr );
	C_F = (real_t*) mxGetPr( Cptr );
	zLow = (real_t*) mxGetPr( zLowPtr );
	zUpp = (real_t*) mxGetPr( zUppPtr );
	zRef = (real_t*) mxGetPr( zRefPtr );


	/* 2) check pointer consistency */
	if ( nIuser == 0 )	mexErrMsgTxt( "ERROR (qpDUNES): Number of Intervals nI undefined!" );
	if ( Cptr == 0 )	mexErrMsgTxt( "ERROR (qpDUNES): C matrix undefined!" );


	/* 3) get dimensions */
	uint_t nI = (uint_t)(*nIuser);
	uint_t nColsH, nColsC;
	uint_t nX, nU, nZ;
	uint_t* nD = new uint_t[nI+1];

	nZ = (uint_t) mxGetM( Hptr );			/* get number of rows */
	nColsH = (uint_t) mxGetN( Hptr );		/* get number of columns */
	if ( ( nZ == nColsH ) && ( nI > 1 ) )
	{
		isLTI = QPDUNES_TRUE;
	}
	else {
		isLTI = QPDUNES_FALSE;
	}

	nX = (uint_t) mxGetM( Cptr );		/* get number of rows */
	nColsC = (uint_t) mxGetN( Cptr );	/* get number of columns */

	nU = nZ - nX;

	/* TODO: deal with affine constraints! */
	for ( uint_t ii=0; ii<nI; ++ii ) {
		/* 		nD[ii] = nZ; */
		nD[ii] = 0;
	}
	/* 	nD[nI] = nX;	*/
	nD[nI] = 0;


	/* 4) check dimension consistency */
	/* ToDo: check dimensions of all inputs! */
	if ( !isLTI ) {
		if ( nColsH != nI*nZ )	mexErrMsgTxt( "ERROR (qpDUNES): Detected inputs for LTV system, but dimensions of H are inconsistent." );
		if ( nColsC != nI*nZ )	mexErrMsgTxt( "ERROR (qpDUNES): Detected inputs for LTV system, but dimensions of C are inconsistent." );
		if ( g != 0 ) {		/* g given */
			if ( mxGetM( gPtr )*mxGetN( gPtr ) != nI*nZ + nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected inputs for LTV system, but dimensions of g are inconsistent." );
		}
	}
	else {
		if ( nColsC != nZ )	mexErrMsgTxt( "ERROR (qpDUNES): Detected inputs for LTI system, but dimensions of C are inconsistent." );
	}
	if ( zLow != 0 ) {	/* zLow exists */
		if ( mxGetM( zLowPtr )*mxGetN( zLowPtr ) != nI*nZ+nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected zLow, but dimensions are inconsistent." );
	}
	if ( zUpp != 0 ) {	/* zUpp exists */
		if ( mxGetM( zUppPtr )*mxGetN( zUppPtr ) != nI*nZ+nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected zUpp, but dimensions are inconsistent." );
	}
	if ( zRef != 0 ) {	/* zRef exists */
		if ( mxGetM( zRefPtr )*mxGetN( zRefPtr ) != nI*nZ+nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected zRef, but dimensions are inconsistent." );
	}



	/* III) ACTUALLY SET UP QP42 PROBLEM: */
	mpcDUNES_setupMatlab( &mpcProblemGlobal, nI, nX, nU, nD, optionsPtr );

	/* transpose matrices (Fortran to C) */
	/* only for C (or A, B);
	 * H (or Q, R, P) should be symmetric;
	 * WARNING: if S matrix included, this has to be transposed as well! */
	real_t* C = 0;
	if (!isLTI) {
		C = new real_t[nI*nX*nZ];
		for (uint_t kk=0; kk<nI; ++kk) {
			convertFortranToC( &(C[kk*nX*nZ]), &(C_F[kk*nX*nZ]), nX, nZ );
		}
	}
	else {
		C = new real_t[nX*nZ];
		convertFortranToC( C, C_F, nX, nZ );
	}

	/* combine H and P if not xu-style inputs */
	uint_t cIdx, rIdx;
	real_t* H = 0;
	if(!isLTI) {
		H = new real_t[nI*nZ*nZ+nX*nX];
		/* parse H and P Fortran-style (column-major); note that H and P are by definition self-transposed */
		for ( cIdx=0; cIdx<nI*nZ; ++cIdx ) {
			for ( rIdx=0; rIdx<nZ; ++rIdx ) {
				H[rIdx+cIdx*nZ] = H_in[rIdx+cIdx*nZ];
			}
		}
		for ( cIdx=0; cIdx<nX; ++cIdx ) {
			for ( rIdx=0; rIdx<nX; ++rIdx ) {
				H[nI*nZ*nZ+rIdx+cIdx*nX] = P[rIdx+cIdx*nX];
			}
		}
	}
	else {
		mexErrMsgTxt( "ERROR (qpDUNES): LTI systems so far only supported for x-u style inputs." );
	}


	/* setup Data */
	if (mpcProblemGlobal->qpData.options.printLevel >= 3) {
		if (!isLTI) {
			mexPrintf( "Detected LTV problem of size [nI = %d, nX = %d, nU = %d]\n", nI, nX, nU );
		}
		else {
			mexPrintf( "Detected LTI problem of size [nI = %d, nX = %d, nU = %d]\n", nI, nX, nU );
		}
	}


	/* depending on given problem data, either setup LTI or LTV problem */
	if (!isLTI) {
		statusFlag = mpcDUNES_initLtvSb( mpcProblemGlobal, H, g, C, c, zLow, zUpp, zRef );
	}
	else {
		mexErrMsgTxt( "ERROR (qpDUNES): LTI systems so far only supported for x-u style inputs." );
	}
	if ( statusFlag != QPDUNES_OK ) {
		mexPrintf( "Problem setup returned error code %d", statusFlag );
		mexErrMsgTxt( "ERROR (qpDUNES): Problem setup failed!" );
	}


	if( nD ) delete[] nD;
	if( H ) delete[] H;
	if( C ) delete[] C;


	return;
}
/*<<< END OF initZStyle */



/*
 *	u p d a t e Z S t y l e
 */
void updateZStyle( mpcProblem_t* mpcProblem, int nrhs, const mxArray* const prhs[] )
{
	/* 0) VARIABLE DECLARATIONS: */
	return_t statusFlag;

	mxArray* Hptr = 0;
	real_t* H_in = 0;
	real_t* H = 0;

	mxArray* Pptr = 0;
	real_t* P = 0;

	mxArray* gPtr = 0;
	real_t* g = 0;

	mxArray* Cptr = 0;
	real_t* C_F = 0;
	real_t* C = 0;

	mxArray* cPtr = 0;
	real_t* c = 0;

	mxArray* Dptr = 0;
	real_t* D_F = 0;
	real_t* D = 0;

	mxArray* zLowPtr = 0;
	real_t* zLow = 0;
	mxArray* zUppPtr = 0;
	real_t* zUpp = 0;

	mxArray* zRefPtr = 0;
	real_t* zRef = 0;


	/* I) GET OUTPUT POINTERS CHECKS: */
	Hptr = (mxArray*)prhs[0];	/* to get dimensions */
	Pptr = (mxArray*)prhs[1];
	gPtr = (mxArray*)prhs[2];
	Cptr = (mxArray*)prhs[3];	/* to get dimensions */
	cPtr = (mxArray*)prhs[4];
	Dptr = (mxArray*)prhs[5];
	zLowPtr = (mxArray*)prhs[6];
	zUppPtr = (mxArray*)prhs[7];
	zRefPtr = (mxArray*)prhs[8];
	/* exatract data */
	H_in = (real_t*) mxGetPr( Hptr );
	P = (real_t*) mxGetPr( Pptr );
	g = (real_t*) mxGetPr( gPtr );
	C_F = (real_t*) mxGetPr( Cptr );
	c = (real_t*) mxGetPr( cPtr );
	D_F = (real_t*) mxGetPr( Dptr );
	zLow = (real_t*) mxGetPr( zLowPtr );
	zUpp = (real_t*) mxGetPr( zUppPtr );
	zRef = (real_t*) mxGetPr( zRefPtr );



	/* II) CONSISTENCY CHECKS: */
	uint_t nI = mpcProblem->qpData.nI;
	uint_t nX = mpcProblem->qpData.nX;
	uint_t nZ = mpcProblem->qpData.nZ;

	if ( ( H_in != 0 ) != ( P != 0 ) ) {
		mexErrMsgTxt( "ERROR (qpDUNES): For update of LTV system either both H and P, or none of them have to be provided." );
	}
	if ( H_in != 0 ) {	/* H given => also P needs to be given (checked  before) */
		if ( !mpcProblem->isLTI ) {
			if ( mxGetM( Hptr ) != nZ )	mexErrMsgTxt( "ERROR (qpDUNES): Detected H, but inconsistent row number" );
			if ( mxGetN( Hptr ) != nI*nZ )	mexErrMsgTxt( "ERROR (qpDUNES): Detected H, but inconsistent column number for LTV system (set up originally)." );
		}
		else {
			/* H does not need to be updated b/c LTI */
			mexErrMsgTxt( "ERROR (qpDUNES): Detected H, but LTI system set up originally." );
		}
	}
	if ( H_in != 0 ) {	/* P given */
		if ( mxGetM( Pptr ) != nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected P, but inconsistent row number" );
		if ( mxGetN( Pptr ) != nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected P, but inconsistent column number" );
	}
	if ( g != 0 ) {		/* g given */
		if ( !mpcProblem->isLTI ) {
			if ( mxGetM( gPtr )*mxGetN( gPtr ) != nI*nZ + nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected g, but inconsistent dimensionality for LTV system (set up originally)." );
		}
		else {
			/* g do not need to be updated b/c LTI */
			mexErrMsgTxt( "ERROR (qpDUNES): Detected g, but LTI system set up originally." );
		}
	}
	if ( C_F != 0 ) {	/* C given */
		if ( !mpcProblem->isLTI ) {
			if ( mxGetM( Cptr ) != nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected C, but inconsistent row number." );
			if ( mxGetN( Cptr ) != nI*nZ )	mexErrMsgTxt( "ERROR (qpDUNES): Detected C, but inconsistent column number for LTV system (set up originally)." );
		}
		else {
			/* C does not need to be updated b/c LTI */
			mexErrMsgTxt( "ERROR (qpDUNES): Detected C, but LTI system set up originally." );
		}
	}
	if ( c != 0 ) {	/* c given */
		if ( !mpcProblem->isLTI ) {
			if ( mxGetM( cPtr )*mxGetN( cPtr ) != nI*nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected c, but inconsistent dimensionality for LTV system (set up originally)." );
		}
		else{
			/* c does not need to be updated b/c LTI */
			mexErrMsgTxt( "ERROR (qpDUNES): Detected c, but LTI system set up originally." );
		}
	}
	if ( D_F != 0 ) {	/* C given */
		/* TODO: D not yet supported */
		mexErrMsgTxt( "ERROR (qpDUNES): General constraint matrices not yet supported." );
	}
	if ( zLow != 0 ) {	/* zLow given */
		if ( mxGetM( zLowPtr )*mxGetN( zLowPtr ) != nI*nZ+nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected zLow, but dimensions are inconsistent." );
	}
	if ( zUpp != 0 ) {	/* zUpp exists */
		if ( mxGetM( zUppPtr )*mxGetN( zUppPtr ) != nI*nZ+nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected zUpp, but dimensions are inconsistent." );
	}
	if ( zRef != 0 ) {	/* zRef exists */
		if ( mxGetM( zRefPtr )*mxGetN( zRefPtr ) != nI*nZ+nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected zRef, but dimensions are inconsistent." );
	}



	/* III) ACTUALLY SET UP QP42 PROBLEM: */
	/* transpose matrices (Fortran to C) */
	/* only for C, D */
	if (C_F != 0) {
		C = new real_t[nI*nX*nZ];
		for (uint_t kk=0; kk<nI; ++kk) {
			convertFortranToC( &(C[kk*nX*nZ]), &(C_F[kk*nX*nZ]), nX, nZ );
		}
	}
	if (D_F != 0) {
		/* TODO: transpose D */
	}

	/* combine H and P (only given if LTV system) */
	uint_t cIdx, rIdx;
	if( (H_in != 0) && (P != 0) ) {
		H = new real_t[nI*nZ*nZ+nX*nX];
		/* parse H and P Fortran-style (column-major); note that H and P are by definition self-transposed */
		for ( cIdx=0; cIdx<nI*nZ; ++cIdx ) {
			for ( rIdx=0; rIdx<nZ; ++rIdx ) {
				H[rIdx+cIdx*nZ] = H_in[rIdx+cIdx*nZ];
			}
		}
		for ( cIdx=0; cIdx<nX; ++cIdx ) {
			for ( rIdx=0; rIdx<nX; ++rIdx ) {
				H[nI*nZ*nZ+rIdx+cIdx*nX] = P[rIdx+cIdx*nX];
			}
		}
	}


	/* print which data fields are updated */
	if (mpcProblemGlobal->qpData.options.printLevel >= 3) {
		mexPrintf( "Detected Data updates for " );
		mpcProblem->isLTI ?	mexPrintf( "LTI " ) : mexPrintf( "LTV " );
		mexPrintf( "problem of size [nI = %d, nX = %d, nU = %d]:\n", nI, nX, mpcProblem->qpData.nU );
		( H_in != 0 ) ? mexPrintf( "H\n" ) : mexPrintf( "" );
		( P != 0 ) ? mexPrintf( "P\n" ) : mexPrintf( "" );
		( g != 0 ) ? mexPrintf( "g\n" ) : mexPrintf( "" );
		( C_F != 0 ) ? mexPrintf( "C\n" ) : mexPrintf( "" );
		( c != 0 ) ? mexPrintf( "c\n" ) : mexPrintf( "" );
		( D_F != 0 ) ? mexPrintf( "D\n" ) : mexPrintf( "" );
		( zLow != 0 ) ? mexPrintf( "zLow\n" ) : mexPrintf( "" );
		( zUpp != 0 ) ? mexPrintf( "zUpp\n" ) : mexPrintf( "" );
		( zRef != 0 ) ? mexPrintf( "zRef\n" ) : mexPrintf( "" );
	}


	if( !mpcProblem->isLTI ) {
		/* date all intervals up */
		/* TODO: maybe distinguish also between LTV MPC (matrices only get shifted) and NMPC (all matrices get renewed) */
		statusFlag = qpDUNES_updateData( &(mpcProblem->qpData), H, g, C, c, zLow, zUpp, 0, 0, 0 );	/* TODO: support general constraints	*/
		if (statusFlag != QPDUNES_OK) {
			mexErrMsgTxt( "LTV data update failed." );
			return;
		}
		if (mpcProblemGlobal->qpData.options.printLevel >= 3) {
			mexPrintf( "Did LTV data update for:\n" );
			( H_in != 0 ) ? mexPrintf( "H\n" ) : mexPrintf( "" );
			( P != 0 ) ? mexPrintf( "P\n" ) : mexPrintf( "" );
			( g != 0 ) ? mexPrintf( "g\n" ) : mexPrintf( "" );
			( C_F != 0 ) ? mexPrintf( "C\n" ) : mexPrintf( "" );
			( c != 0 ) ? mexPrintf( "c\n" ) : mexPrintf( "" );
/*			( D_F != 0 ) ? mexPrintf( "D\n" ) : mexPrintf( "" );	*/
			( zLow != 0 ) ? mexPrintf( "zLow\n" ) : mexPrintf( "" );
			( zUpp != 0 ) ? mexPrintf( "zUpp\n" ) : mexPrintf( "" );
/*			( zRef != 0 ) ? mexPrintf( "zRef\n" ) : mexPrintf( "" );*/
		}
	}
	else {
		/* only date objective in last interval up (if given) */
		/* H, C, c do not need to be updated b/c LTI */
		statusFlag = qpDUNES_updateIntervalData( &(mpcProblem->qpData), mpcProblem->qpData.intervals[nI], P, g, 0, 0, 0, 0, 0, 0, 0, 0 );
		if (statusFlag != QPDUNES_OK) {
			mexErrMsgTxt( "LTI data update failed." );
			return;
		}
		/* in all other intervals only date bounds and reference up */
		statusFlag = qpDUNES_updateData( &(mpcProblem->qpData), 0, 0, 0, 0, zLow, zUpp, 0, 0, 0 );
		if (statusFlag != QPDUNES_OK) {
			mexErrMsgTxt( "LTI data update failed." );
			return;
		}
		if (mpcProblemGlobal->qpData.options.printLevel >= 3) {
			mexPrintf( "Did LTV data update for:\n" );
			( P != 0 ) ? mexPrintf( "P\n" ) : mexPrintf( "" );
			( g != 0 ) ? mexPrintf( "g\n" ) : mexPrintf( "" );
			( zLow != 0 ) ? mexPrintf( "zLow\n" ) : mexPrintf( "" );
			( zUpp != 0 ) ? mexPrintf( "zUpp\n" ) : mexPrintf( "" );
		}
	}
/*	statusFlag = qpDUNES_updateIntervalData( &(mpcProblem->qpData), mpcProblem->qpData.intervals[nI-1], 0, 0, 0, 0, 0, ziLow, ziUpp, 0 );	*/



	if( H ) delete[] H;
	if( C ) delete[] C;
	if( D ) delete[] D;


	return;
}
/*<<< END OF updateZStyle */



/*
 *	s o l v e M a t l a b
 */
void solveMatlab( int nlhs, mxArray* plhs[], int nrhs, const mxArray* const prhs[] )
{
	/* GET INPUTS */
	mxArray* x0Ptr = (mxArray*)prhs[0];
	real_t* x0 = (real_t*) mxGetPr( x0Ptr );


	/* CONSISTENCY CHECKS */
	/* 1)   Check for proper number of output arguments. */
	if (mpcProblemGlobal->qpData.options.logLevel == QPDUNES_LOG_ALL_DATA ) {
		if ( nlhs > 6 )	mexErrMsgTxt( "ERROR (qpDUNES): At most six output arguments are allowed: \n       [uOpt, xOpt, status, objVal, time, log]!" );
	}
	else {
		if ( nlhs > 5 )	mexErrMsgTxt( "ERROR (qpDUNES): At most five output arguments are allowed: \n       [uOpt, xOpt, status, objVal, time]!" );
	}
	if ( nlhs < 1 )	mexErrMsgTxt( "ERROR (qpDUNES): At least one output argument is required: [uOpt,...]!" );

	/* 2)   Check inputs */
	if ( x0 != 0 ) {	/* x0 exists */
		if ( mxGetM( x0Ptr )*mxGetN( x0Ptr ) != mpcProblemGlobal->qpData.nX )	mexErrMsgTxt( "ERROR (qpDUNES): Detected x0, but dimensions are inconsistent." );
	}


	/* ALLOCATE OUTPUTS */
	allocateOutputsMPC( plhs,nlhs, mpcProblemGlobal->qpData.nI,mpcProblemGlobal->qpData.nX,mpcProblemGlobal->qpData.nU );


	#ifndef __WINDOWS__
	real_t tic = 0.0;
	real_t toc = 0.0;
	struct timeval theclock;
	gettimeofday( &theclock,0 );
	tic = 1.0*theclock.tv_sec + 1.0e-6*theclock.tv_usec;
	#endif
	/* SOLVE QPDUNES PROBLEM: */
	return_t statusFlag = mpcDUNES_solve( mpcProblemGlobal, x0 );
	#ifndef __WINDOWS__
	gettimeofday( &theclock,0 );
	toc = 1.0*theclock.tv_sec + 1.0e-6*theclock.tv_usec - tic;
	#endif
	if ( ( statusFlag != QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND ) &&
		 ( statusFlag != QPDUNES_SUCC_SUBOPTIMAL_TERMINATION ) )
	{
		mexPrintf( "qpDUNES returned flag %d\n", statusFlag );
		if (mpcProblemGlobal->qpData.options.logLevel == QPDUNES_LOG_ALL_DATA ) {
			if ( nlhs == 5 ) {
				fullLogging( &(mpcProblemGlobal->qpData), &(plhs[4]) );
			}
			mexPrintf( "ERROR (qpDUNES): Problem could not be solved!\n" );
		}
		else {
			mexErrMsgTxt( "ERROR (qpDUNES): Problem could not be solved!" );
		}
	}


	/* V) PASS SOLUTION ON TO MATLAB: */
	obtainOutputsMPC( mpcProblemGlobal, plhs, nlhs, toc );


	/* VI) PASS DETAILED LOG INFORMATION ON TO MATLAB: */
	if (mpcProblemGlobal->qpData.options.logLevel == QPDUNES_LOG_ALL_DATA ) {
		if ( nlhs == 6 ) {
			fullLogging( &(mpcProblemGlobal->qpData), &(plhs[5]) );
		}
	}


	return;
}
/*<<< END OF solveMatlab */




/*
 *	m e x F u n c t i o n
 */
void mexFunction( int nlhs, mxArray* plhs[], int nrhs, const mxArray* prhs[] )
{
	/* I) CONSISTENCY CHECKS: Ensure that first input is a string */
	if ( mxIsChar( prhs[0] ) != 1 )
		mexErrMsgTxt( "ERROR (qpDUNES): First input argument must be a string!" );

	char* typeString = (char*) mxGetPr( prhs[0] );


	/* SELECT QPDUNES ACTION TO BE PERFORMED */
	/* a) initial setup */
	if ( ( strcmp( typeString,"i" ) == 0 ) ||
		 ( strcmp( typeString,"I" ) == 0 ) ||
		 ( strcmp( typeString,"init" ) == 0 ) ||
		 ( strcmp( typeString,"Init" ) == 0 ) ||
		 ( strcmp( typeString,"INIT" ) == 0 ) )
	{
		if (nlhs > 0) {
			mexErrMsgTxt( "ERROR (qpDUNES): Too many output arguments for action 'init'.\n                 Type 'help mpcDUNES' for further information.");
		}

		switch ( nrhs-1 ) {
			case 9:
			case 10:
				initZStyle( nrhs-1, prhs+1 );	/* shift input pointers to begin of data */
				break;

			case 13:
			case 14:
				initXUStyle( nrhs-1, prhs+1 );	/* shift input pointers to begin of data */
				break;

			default:
				mexPrintf( "Received %d input arguments.\n", nrhs );
				mexErrMsgTxt( "ERROR (qpDUNES): Wrong number of input arguments for action 'init'.\n                 Type 'help mpcDUNES' for further information." );
				break;
		}

		return;
	}

	/* b) data update */
	if ( ( strcmp( typeString,"u" ) == 0 ) ||
		 ( strcmp( typeString,"U" ) == 0 ) ||
		 ( strcmp( typeString,"update" ) == 0 ) ||
		 ( strcmp( typeString,"Update" ) == 0 ) ||
		 ( strcmp( typeString,"UPDATE" ) == 0 ) )
	{
		if( mpcProblemGlobal == 0 )
			mexErrMsgTxt( "ERROR (qpDUNES): QP data needs to initialized at least once before action 'update' can be performed.\n                 Type 'help mpcDUNES' for further information." );

		if (nlhs > 0) {
			mexErrMsgTxt( "ERROR (qpDUNES): Too many output arguments for action 'update'.\n                 Type 'help mpcDUNES' for further information.");
		}

		if (nrhs-1 != 9) {
			mexErrMsgTxt( "ERROR (qpDUNES): Wrong number of input arguments for action 'update'.\n                 Type 'help mpcDUNES' for further information.");
		}

		updateZStyle( mpcProblemGlobal, nrhs-1, prhs+1 );

		return;
	}

	/* c) solve */
	if ( ( strcmp( typeString,"s" ) == 0 ) ||
		 ( strcmp( typeString,"S" ) == 0 ) ||
		 ( strcmp( typeString,"solve" ) == 0 ) ||
		 ( strcmp( typeString,"Solve" ) == 0 ) ||
		 ( strcmp( typeString,"SOLVE" ) == 0 ) )
	{
		if( mpcProblemGlobal == 0 )
			mexErrMsgTxt( "ERROR (qpDUNES): QP data needs to initialized before action 'solve' can be performed.\n                 Type 'help mpcDUNES' for further information." );

		if (nrhs > 2) {
			mexErrMsgTxt( "ERROR (qpDUNES): Too many input arguments for action 'solve'.\n                 Type 'help mpcDUNES' for further information.");
		}

		solveMatlab( nlhs, plhs, nrhs-1, prhs+1 );

		return;
	}

	/* d) cleanup */
	if ( ( strcmp( typeString,"c" ) == 0 ) ||
		 ( strcmp( typeString,"C" ) == 0 ) ||
		 ( strcmp( typeString,"cleanup" ) == 0 ) ||
		 ( strcmp( typeString,"Cleanup" ) == 0 ) ||
		 ( strcmp( typeString,"CLEANUP" ) == 0 ) )
	{
		mexWarnMsgTxt( "[qpDUNES]: A 'cleanup' is not needed anymore and the command will be deprecated in the future.\n                    Simply do a new 'init', use MATLAB's 'clear mpcDUNES', or exit MATLAB." );
		/*mpcDUNES_safeCleanupMatlab( &mpcProblemGlobal );*/

		return;
	}

	mexErrMsgTxt( "ERROR (qpDUNES): Undefined first input argument!\nType 'help mpcDUNES' for further information." );

	return;
}

/*
 *	end of file
 */
