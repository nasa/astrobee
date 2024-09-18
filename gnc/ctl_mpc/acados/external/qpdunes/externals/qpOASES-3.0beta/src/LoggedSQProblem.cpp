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
 *	\file src/QProblem.cpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.0beta
 *	\date 2007-2012
 *
 *	Implementation of the QProblem class which is able to use the newly
 *	developed online active set strategy for parametric quadratic programming.
 */


#include <qpOASES/LoggedSQProblem.hpp>
#include <stdio.h>
//#include <mex.h>

//#define __DEBUG_ITER__

BEGIN_NAMESPACE_QPOASES


/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/


/*
 *	L o g g e d S Q P r o b l e m
 */
LoggedSQProblem::LoggedSQProblem( int _nV, int _nC, HessianType _hessianType ) : SQProblem( _nV,_nC,_hessianType )
{
}


/*
 *	h o t s t a r t    w i t h    h o m o t o p y    l o g g i n g
 *	(for line search in qpDUNES)
 */
returnValue LoggedSQProblem::hotstart_withHomotopyLogging(	const real_t* const g_new,
													const real_t* const lb_new, const real_t* const ub_new,
													const real_t* const lbA_new, const real_t* const ubA_new,
													real_t* const parametricObjFctn_alpha,			/* log for homotopy kinks (active set changes) */
													real_t* const parametricObjFctn_f,				/* log for objective value */
													real_t* const parametricObjFctn_fPrime,			/* log for objective derivative in homotopy direction */
													real_t* const parametricObjFctn_fPrimePrime,	/* log for objective second derivative in homotopy direction */
													int& nWSR, real_t* const cputime
													)
{
	int i, nActiveFar;
	int nV = getNV ();
	int nC = getNC ();

	if (nV == 0)
		return THROWERROR( RET_QPOBJECT_NOT_SETUP );

	returnValue returnvalue = SUCCESSFUL_RETURN;

	/* Simple check for consistency of bounds and constraints. */
	if ( areBoundsConsistent(lb_new, ub_new, lbA_new, ubA_new) != SUCCESSFUL_RETURN )
		return setInfeasibilityFlag(returnvalue,BT_TRUE);

	++count;

	int nWSR_max = nWSR;
	int nWSR_performed = 0;

	real_t cputime_remaining = INFTY, *pcputime_rem;
	real_t cputime_needed = 0.0;

	real_t farbound = options.initialFarBounds;

	/* writeQpDataIntoMatFile( "qpData.mat" ); */
	/* writeQpWorkspaceIntoMatFile( "qpWorkspace.mat" ); */

	if ( haveCholesky == BT_FALSE )
	{
		returnvalue = setupInitialCholesky( );
		if (returnvalue != SUCCESSFUL_RETURN)
			return THROWERROR(returnvalue);
	}


	/** >> here comes solveQP */
//	if ( options.enableFarBounds == BT_FALSE )
//	{
//		/* Automatically call standard solveQP if regularisation is not active. */
//		returnvalue = solveRegularisedQP( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,cputime,0 );
//	}
	/** we know that we do not have regularized QP, so we can call regular solve QP directly */
	returnvalue = solveQP_withHomotopyLogging( g_new,lb_new,ub_new,lbA_new,ubA_new,
											parametricObjFctn_alpha,
											parametricObjFctn_f,
											parametricObjFctn_fPrime,
											parametricObjFctn_fPrimePrime,
											nWSR,cputime,0
											);

	/** <<  end of solveQP */

///////////////////////////////////////////////////////////////
//	else
//	{
//		real_t *ub_new_far = new real_t[nV];
//		real_t *lb_new_far = new real_t[nV];
//		real_t *ubA_new_far = new real_t[nC];
//		real_t *lbA_new_far = new real_t[nC];
//
//		/* possibly extend initial far bounds to largest bound/constraint data */
//		if (ub_new)
//			for (i = 0; i < nV; i++)
//				if ((ub_new[i] < INFTY) && (ub_new[i] > farbound)) farbound = ub_new[i];
//		if (lb_new)
//			for (i = 0; i < nV; i++)
//				if ((lb_new[i] > -INFTY) && (lb_new[i] < -farbound)) farbound = -lb_new[i];
//		if (ubA_new)
//			for (i = 0; i < nC; i++)
//				if ((ubA_new[i] < INFTY) && (ubA_new[i] > farbound)) farbound = ubA_new[i];
//		if (lbA_new)
//			for (i = 0; i < nC; i++)
//				if ((lbA_new[i] > -INFTY) && (lbA_new[i] < -farbound)) farbound = -lbA_new[i];
//
//		updateFarBounds(	farbound,nV+nC,
//							lb_new,lb_new_far, ub_new,ub_new_far,
//							lbA_new,lbA_new_far, ubA_new,ubA_new_far
//							);
//
//		for ( ;; )
//		{
//			nWSR = nWSR_max;
//			if ( cputime != 0 )
//			{
//				cputime_remaining = *cputime - cputime_needed;
//				pcputime_rem = &cputime_remaining;
//			}
//			else
//				pcputime_rem = 0;
//
//			/* Automatically call standard solveQP if regularisation is not active. */
//			returnvalue = solveRegularisedQP( g_new,lb_new_far,ub_new_far,lbA_new_far,ubA_new_far, nWSR,pcputime_rem,nWSR_performed );
//
//			nWSR_performed  = nWSR;
//			cputime_needed += cputime_remaining;
//
//			/* Check for active far-bounds and move them away */
//			nActiveFar = 0;
//			farbound *= options.growFarBounds;
//
//			if ( infeasible == BT_TRUE )
//			{
//				if ( farbound >= INFTY )
//				{
//					returnvalue = RET_HOTSTART_STOPPED_INFEASIBILITY;
//					break; // goto farewell;
//				}
//
//				updateFarBounds(	farbound,nV+nC,
//									lb_new,lb_new_far, ub_new,ub_new_far,
//									lbA_new,lbA_new_far, ubA_new,ubA_new_far
//									);
//			}
//			else if ( status == QPS_SOLVED )
//			{
//				real_t tol = farbound/options.growFarBounds * options.boundTolerance;
//
//				for ( i=0; i<nV; ++i )
//				{
//					if ( ( ( lb_new == 0 ) || ( lb_new_far[i] > lb_new[i] ) ) && ( getAbs ( lb_new_far[i] - x[i] ) < tol ) )
//						++nActiveFar;
//					if ( ( ( ub_new == 0 ) || ( ub_new_far[i] < ub_new[i] ) ) && ( getAbs ( ub_new_far[i] - x[i] ) < tol ) )
//						++nActiveFar;
//				}
//				for ( i=0; i<nC; ++i )
//				{
//					if ( ( ( lbA_new == 0 ) || ( lbA_new_far[i] > lbA_new[i] ) ) && ( getAbs ( lbA_new_far[i] - Ax[i] ) < tol ) )
//						++nActiveFar;
//					if ( ( ( ubA_new == 0 ) || ( ubA_new_far[i] < ubA_new[i] ) ) && ( getAbs ( ubA_new_far[i] - Ax[i] ) < tol ) )
//						++nActiveFar;
//				}
//
//				if ( nActiveFar == 0 )
//					break;
//
//				status = QPS_HOMOTOPYQPSOLVED;
//
//				if ( farbound >= INFTY )
//				{
//					unbounded = BT_TRUE;
//					returnvalue = RET_HOTSTART_STOPPED_UNBOUNDEDNESS;
//					goto farewell;
//				}
//
//				updateFarBounds(	farbound,nV+nC,
//									lb_new,lb_new_far, ub_new,ub_new_far,
//									lbA_new,lbA_new_far, ubA_new,ubA_new_far
//									);
//			}
//			else
//			{
//				/* some other error when solving QP */
//				break;
//			}
//
//			/* advance ramp offset to avoid Ramping cycles */
//			rampOffset++;
//		}
//
//		farewell:
//			if ( cputime != 0 )
//				*cputime = cputime_needed;
//			delete[] lbA_new_far; delete[] ubA_new_far;
//			delete[] lb_new_far; delete[] ub_new_far;
//	}

	return ( returnvalue != SUCCESSFUL_RETURN ) ? THROWERROR( returnvalue ) : returnvalue;
}
//
//{
//	if ( getNV( ) == 0 )
//		return THROWERROR( RET_QPOBJECT_NOT_SETUP );
//
////	printf( "\nblabla!!!!\nblabla!!!!\nblabla!!!!\n");
////	printf( "nV = %d\n\n", getNV() );
//
////	for ( int ii = 0; ii< getNV(); ++ii ) {
////	for ( int ii = 0; ii< getNC(); ++ii ) {
////		real_t *roww   = new real_t[getNV()];
////		A->getRow (ii, 0, 1.0, roww);
////		for ( int jj = 0; jj< getNV(); ++jj ) {
////			printf( "\t%.2e\t", roww[jj]);
////		}
////		printf( "\t\t\t%.2e\t", lbA[ii] );
////		printf("\n");
////	}
////	printf( "\nblabla!!!!\nblabla!!!!\nblabla!!!!\n");
//
//
//
//	++count;
//
//	returnValue returnvalue = SUCCESSFUL_RETURN;
////	int nFar, i;
////	int nV = getNV ();
////	int nC = getNC ();
//
////	int nWSR_max = nWSR;
////	int nWSR_performed = 0;
//
////	real_t cputime_remaining = INFTY, *pcputime_rem;
////	real_t cputime_needed = 0.0;
//
////	real_t farbound = options.initialFarBounds;
////	real_t t, rampVal;
//
//	/* exclude some default qpOASES options which we do not need/support here */
//	if ( options.enableFarBounds == BT_TRUE )
//	{
//		printf( "[qpOASES] ERROR: far bound strategy not available for qpDUNES\n" );
//		returnvalue = RET_NOT_YET_IMPLEMENTED;
//		return THROWERROR(returnvalue);
//	}
//	if ( usingRegularisation( ) == BT_TRUE )	{
//		printf( "[qpOASES] ERROR: regularization strategy not available for qpDUNES\n" );
//		returnvalue = RET_NOT_YET_IMPLEMENTED;
//		return THROWERROR(returnvalue);
////			returnvalue = solveRegularisedQP( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR,cputime,0 );
//	}
//
//
//	/* normal qpOASES solution */
//	if (haveCholesky == BT_FALSE)
//	{
//		returnvalue = computeInitialCholesky();
//		if (returnvalue != SUCCESSFUL_RETURN)
//			return THROWERROR(returnvalue);
//	}
//
//
//	/** >> here comes solveQP */
//	returnvalue = solveQP_withHomotopyLogging( g_new,lb_new,ub_new,lbA_new,ubA_new,
//											parametricObjFctn_alpha,
//											parametricObjFctn_f,
//											parametricObjFctn_fPrime,
//											parametricObjFctn_fPrimePrime,
//											nWSR,cputime,0
//											);
//
//	/** <<  end of solveQP */
//
////		real_t *ub_new_far = new real_t[nV];
////		real_t *lb_new_far = new real_t[nV];
////		real_t *ubA_new_far = new real_t[nC];
////		real_t *lbA_new_far = new real_t[nC];
////
////		/* possibly extend initial far bounds to largest bound/constraint data */
////		if (ub_new)
////			for (i = 0; i < nV; i++)
////				if (ub_new[i] < INFTY && ub_new[i] > farbound) farbound = ub_new[i];
////		if (lb_new)
////			for (i = 0; i < nV; i++)
////				if (lb_new[i] > -INFTY && lb_new[i] < -farbound) farbound = -lb_new[i];
////		if (ubA_new)
////			for (i = 0; i < nC; i++)
////				if (ubA_new[i] < INFTY && ubA_new[i] > farbound) farbound = ubA_new[i];
////		if (lbA_new)
////			for (i = 0; i < nC; i++)
////				if (lbA_new[i] > -INFTY && lbA_new[i] < -farbound) farbound = -lbA_new[i];
////
////		if ( options.enableRamping == BT_TRUE )
////		{
////			/* TODO: encapsule this part to avoid code duplication! */
////			for ( i=0; i<nV; ++i )
////			{
////				t = static_cast<real_t>((i + rampOffset) % nV) / static_cast<real_t>(nV+nC-1);
////				rampVal = farbound * (1.0 + t * (ramp1 - ramp0));
////
////				if ( ( lb_new == 0 ) || ( lb_new[i] <= -rampVal ) )
////					lb_new_far[i] = - rampVal;
////				else
////					lb_new_far[i] = lb_new[i];
////				if ( ( ub_new == 0 ) || ( ub_new[i] >= rampVal ) )
////					ub_new_far[i] = rampVal;
////				else
////					ub_new_far[i] = ub_new[i];
////			}
////			for ( i=0; i<nC; ++i )
////			{
////				t = static_cast<real_t>((nV+i + rampOffset) % (nV+nC)) / static_cast<real_t>(nV+nC-1);
////				rampVal = farbound * (1.0 + t * (ramp1 - ramp0));
////
////				if ( ( lbA_new == 0 ) || ( lbA_new[i] <= -rampVal ) )
////					lbA_new_far[i] = - rampVal;
////				else
////					lbA_new_far[i] = lbA_new[i];
////				if ( ( ubA_new == 0 ) || ( ubA_new[i] >= rampVal ) )
////					ubA_new_far[i] = rampVal;
////				else
////					ubA_new_far[i] = ubA_new[i];
////			}
////		}
////		else
////		{
////			for ( i=0; i<nV; ++i )
////			{
////				lb_new_far[i] = lb_new[i];
////				ub_new_far[i] = ub_new[i];
////			}
////
////			for ( i=0; i<nC; ++i )
////			{
////				lbA_new_far[i] = lbA_new[i];
////				ubA_new_far[i] = ubA_new[i];
////			}
////		}
////
////		if (haveCholesky == BT_FALSE)
////		{
////			returnvalue = computeInitialCholesky();
////			if (returnvalue != SUCCESSFUL_RETURN)
////				goto farewell;
////		}
////
////		for ( ;; )
////		{
////			nWSR = nWSR_max;
////			if ( cputime != 0 )
////			{
////				cputime_remaining = *cputime - cputime_needed;
////				pcputime_rem = &cputime_remaining;
////			}
////			else
////				pcputime_rem = 0;
////
////			if ( usingRegularisation( ) == BT_TRUE )
////				returnvalue = solveRegularisedQP( g_new,lb_new_far,ub_new_far,lbA_new_far,ubA_new_far, nWSR,pcputime_rem,nWSR_performed );
////			else
////				returnvalue = solveQP( g_new,lb_new_far,ub_new_far,lbA_new_far,ubA_new_far, nWSR,pcputime_rem,nWSR_performed );
////
////			nWSR_performed  = nWSR;
////			cputime_needed += cputime_remaining;
////
////			/* Check for active far-bounds and move them away */
////			nFar = 0;
////			farbound *= options.growFarBounds;
////
////			real_t maxFarbound = 1e20;
////			if ( infeasible == BT_TRUE )
////			{
////				if ( farbound > maxFarbound )
////				{
////					returnvalue = RET_HOTSTART_STOPPED_INFEASIBILITY;
////					break; // goto farewell;
////				}
////
////				if ( options.enableRamping == BT_TRUE )
////				{
////					/* TODO: encapsule this part to avoid code duplication! */
////					for ( i=0; i<nV; ++i )
////					{
////						t = static_cast<real_t>((i + rampOffset) % (nV + nC)) / static_cast<real_t>(nV+nC-1);
////						rampVal = farbound * (1.0 + t * (ramp1 - ramp0));
////
////						if ( lb_new == 0 )
////							lb_new_far[i] = - rampVal;
////						else
////							lb_new_far[i] = getMax (- rampVal, lb_new[i]);
////
////						if ( ub_new == 0 )
////							ub_new_far[i] = rampVal;
////						else
////							ub_new_far[i] = getMin (rampVal, ub_new[i]);
////					}
////					for ( i=0; i<nC; ++i )
////					{
////						t = static_cast<real_t>((nV+i + rampOffset) % (nV + nC)) / static_cast<real_t>(nV+nC-1);
////						rampVal = farbound * (1.0 + t * (ramp1 - ramp0));
////
////						if ( lbA_new == 0 )
////							lbA_new_far[i] = - rampVal;
////						else
////							lbA_new_far[i] = getMax (- rampVal, lbA_new[i]);
////
////						if ( ubA_new == 0 )
////							ubA_new_far[i] = rampVal;
////						else
////							ubA_new_far[i] = getMin (rampVal, ubA_new[i]);
////					}
////				}
////				else
////				{
////					for ( i=0; i<nV; ++i )
////					{
////						lb_new_far[i] = lb_new[i];
////						ub_new_far[i] = ub_new[i];
////					}
////
////					for ( i=0; i<nC; ++i )
////					{
////						lbA_new_far[i] = lbA_new[i];
////						ubA_new_far[i] = ubA_new[i];
////					}
////				}
////			}
////			else if ( status == QPS_SOLVED )
////			{
////				real_t tol = farbound * options.boundTolerance;
////				status = QPS_HOMOTOPYQPSOLVED;
////				/* TODO: encapsule this part to avoid code duplication! */
////				nFar = 0;
////				for ( i=0; i<nV; ++i )
////				{
////					if ( ( ( lb_new == 0 ) || ( lb_new_far[i] > lb_new[i] ) ) && ( getAbs ( lb_new_far[i] - x[i] ) < tol ) )
////						++nFar;
////					if ( ( ( ub_new == 0 ) || ( ub_new_far[i] < ub_new[i] ) ) && ( getAbs ( ub_new_far[i] - x[i] ) < tol ) )
////						++nFar;
////				}
////				for ( i=0; i<nC; ++i )
////				{
////					if ( ( ( lbA_new == 0 ) || ( lbA_new_far[i] > lbA_new[i] ) ) && ( getAbs ( lbA_new_far[i] - Ax[i] ) < tol ) )
////						++nFar;
////					if ( ( ( ubA_new == 0 ) || ( ubA_new_far[i] < ubA_new[i] ) ) && ( getAbs ( ubA_new_far[i] - Ax[i] ) < tol ) )
////						++nFar;
////				}
////
////				if ( nFar == 0 )
////					break;
////
////				if ( farbound > maxFarbound )
////				{
////					unbounded = BT_TRUE;
////					returnvalue = RET_HOTSTART_STOPPED_UNBOUNDEDNESS;
////					goto farewell;
////				}
////
////				if ( options.enableRamping == BT_TRUE )
////				{
////					/* TODO: encapsule this part to avoid code duplication! */
////					for ( i=0; i<nV; ++i )
////					{
////						t = static_cast<real_t>((i + rampOffset) % (nV + nC)) / static_cast<real_t>(nV+nC-1);
////						rampVal = farbound + (1.0-t) * ramp0 + t * ramp1;
////
////						if ( lb_new == 0 )
////							lb_new_far[i] = - rampVal;
////						else
////							lb_new_far[i] = getMax (- rampVal, lb_new[i]);
////
////						if ( ub_new == 0 )
////							ub_new_far[i] = rampVal;
////						else
////							ub_new_far[i] = getMin (rampVal, ub_new[i]);
////					}
////					for ( i=0; i<nC; ++i )
////					{
////						t = static_cast<real_t>((nV+i + rampOffset) % (nV + nC)) / static_cast<real_t>(nV+nC-1);
////						rampVal = farbound * (1.0 + t * (ramp1 - ramp0));
////
////						if ( lbA_new == 0 )
////							lbA_new_far[i] = - rampVal;
////						else
////							lbA_new_far[i] = getMax (- rampVal, lbA_new[i]);
////
////						if ( ubA_new == 0 )
////							ubA_new_far[i] = rampVal;
////						else
////							ubA_new_far[i] = getMin (rampVal, ubA_new[i]);
////					}
////				}
////				else
////				{
////					for ( i=0; i<nV; ++i )
////					{
////						lb_new_far[i] = lb_new[i];
////						ub_new_far[i] = ub_new[i];
////					}
////
////					for ( i=0; i<nC; ++i )
////					{
////						lbA_new_far[i] = lbA_new[i];
////						ubA_new_far[i] = ubA_new[i];
////					}
////				}
////			}
////			else
////			{
////				/* some other error */
////				break;
////			}
////
////			/* advance ramp offset to avoid Ramping cycles */
////			rampOffset++;
////		}
////
////		farewell:
////			if ( cputime != 0 )
////				*cputime = cputime_needed;
////			delete[] lbA_new_far; delete[] ubA_new_far;
////			delete[] lb_new_far; delete[] ub_new_far;
////	}
//
//	return ( returnvalue != SUCCESSFUL_RETURN ) ? THROWERROR( returnvalue ) : returnvalue;
//}
///* END OF hotstart_withHomotopyLog */


/*
 *	g e t R
 */
void LoggedSQProblem::getR( real_t** const ROut ) const
{
//	int nZ = getNZ();
//	int nV = getNV();

	// TODO: work with shared memory between qpDUNES and qpOASES

//	for ( int ii=0; ii<nV; ++ii ) {
//		for ( int jj=0; jj<nV; ++jj ) {
//			ROut[ii*nV+jj] = RR(ii,jj);	// todo: save transposed for lower triangular format
//		}
//	}

	*ROut = R;

}


/*
 *	g e t Z
 */
void LoggedSQProblem::getQT( real_t** const QTOut, int* nZ ) const
{
	*nZ = getNZ();
//	int nV = getNV();
//	int nFR = getNFR();

//	int i,j, jj;
//
//	// TODO: work with shared memory between qpDUNES and qpOASES
//
//	// access column-wise for higher cache efficiency; QQ is saved column-major anyways
//	for ( int jj=0; jj<*nZ; ++jj ) {
//		for ( int ii=0; ii<nV; ++ii ) {
//			ZTOut[jj*nV+ii] = QQ(ii,jj);	// save transposed, since we need it transposed anyways
//		}
//	}

///////////////
//	for( j=0; j<nFR; ++j )
//	{
//		jj = FR_idx[j];
//		for( i=0; i<nZ; ++i )
//			delta_xFRz[i] -= QQ(jj,i) * tempA[j];
//	}

//	/* get indices of free variables */
//	int* FR_idx;
//	bounds.getFree()->getNumberArray( &FR_idx );
//
//	for( i=0; i<*nZ; ++i )	{
//		/* compute one column of R'^{-1} * Z' */
//		for( j=0; j<nFR; ++j )		/* get a column of Z' (row in QQ's memory layout) */
//		{
//			jj = FR_idx[j];
//			/* todo: use own temporary memory */
//			delta_xFRz[i] = QQ(jj,i);
//		}
//		if ( backsolveR( delta_xFRz,BT_TRUE,delta_xFRz ) != SUCCESSFUL_RETURN )
//			return; // THROWERROR( RET_STEPDIRECTION_FAILED_CHOLESKY );
//
////		if ( backsolveR( delta_xFRz,BT_FALSE,delta_xFRz ) != SUCCESSFUL_RETURN )
////			return THROWERROR( RET_STEPDIRECTION_FAILED_CHOLESKY );
//
//		/* now save this column... */
//	}

////////////////



	// note: QQ is saved column-major, therefore output is QT after row-major interpretation
	*QTOut = Q;		// a bit dirty: just copy Q pointer for fast access

}


/*****************************************************************************
 *  P R O T E C T E D                                                        *
 *****************************************************************************/

/*
 *	s o l v e Q P
 */
returnValue LoggedSQProblem::solveQP_withHomotopyLogging(	const real_t* const g_new,
													const real_t* const lb_new, const real_t* const ub_new,
													const real_t* const lbA_new, const real_t* const ubA_new,
													real_t* const parametricObjFctn_alpha,			/* log for homotopy kinks (active set changes) */
													real_t* const parametricObjFctn_f,				/* log for objective value */
													real_t* const parametricObjFctn_fPrime,			/* log for objective derivative in homotopy direction */
													real_t* const parametricObjFctn_fPrimePrime,	/* log for objective second derivative in homotopy direction */
													int& nWSR, real_t* const cputime, int nWSRperformed
													)
{
	int iter;
	int nV  = getNV( );
	int nC  = getNC( );

	returnValue returnvalue;

	/* consistency check */
	if ( ( getStatus( ) == QPS_NOTINITIALISED )       ||
		 ( getStatus( ) == QPS_PREPARINGAUXILIARYQP ) ||
		 ( getStatus( ) == QPS_PERFORMINGHOMOTOPY )   )
	{
		return THROWERROR( RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED );
	}

	/* start runtime measurement */
	real_t starttime = 0.0;
	if ( cputime != 0 )
		starttime = getCPUtime( );

	/* I) PREPARATIONS */
	/* 1) Allocate delta vectors of gradient and (constraints') bounds,
	 *    index arrays and step direction arrays. */
	real_t* delta_xFR = new real_t[nV];
	real_t* delta_xFX = new real_t[nV];
	real_t* delta_yAC = new real_t[nC];
	real_t* delta_yFX = new real_t[nV];

	real_t* delta_g   = new real_t[nV];
	real_t* delta_lb  = new real_t[nV];
	real_t* delta_ub  = new real_t[nV];
	real_t* delta_lbA = new real_t[nC];
	real_t* delta_ubA = new real_t[nC];

	BooleanType Delta_bC_isZero, Delta_bB_isZero;

	int BC_idx;
	SubjectToStatus BC_status;
	BooleanType BC_isBound;

	real_t homotopyLength;

	#ifndef __XPCTARGET__
	char messageString[80];
	#endif


	/* 2) Update type of bounds and constraints, e.g.
	 *    a former equality constraint might have become a normal one etc. */

  // (ckirches) disabled this, as inactive but tight bounds may become inactive equalities
    //            which would then never become active again!
/*
	if ( setupSubjectToType( lb_new,ub_new,lbA_new,ubA_new ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_HOTSTART_FAILED );
*/

	/* 3) Reset status flags. */
	infeasible = BT_FALSE;
	unbounded  = BT_FALSE;


	/* II) MAIN HOMOTOPY LOOP */
	for( iter=nWSRperformed; iter<nWSR; ++iter )
	{
		tabularOutput.idxAddB = tabularOutput.idxRemB = tabularOutput.idxAddC = tabularOutput.idxRemC = -1;
		tabularOutput.excAddB = tabularOutput.excRemB = tabularOutput.excAddC = tabularOutput.excRemC = 0;

		if ( isCPUtimeLimitExceeded( cputime,starttime,iter-nWSRperformed ) == BT_TRUE )
		{
			/* If CPU time limit is exceeded, stop homotopy loop immediately!
			* Assign number of working set recalculations (runtime measurement is stopped later). */
			nWSR = iter;
			break;
		}

		status = QPS_PERFORMINGHOMOTOPY;

		#ifndef __XPCTARGET__
		snprintf( messageString,80,"%d ...",iter );
		getGlobalMessageHandler( )->throwInfo( RET_ITERATION_STARTED,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
		#endif

		/* 2) Determination of shift direction of the gradient and the (constraints') bounds. */
		returnvalue = determineDataShift(	g_new,lbA_new,ubA_new,lb_new,ub_new,
											delta_g,delta_lbA,delta_ubA,delta_lb,delta_ub,
											Delta_bC_isZero, Delta_bB_isZero
											);
		if ( returnvalue != SUCCESSFUL_RETURN )
		{
			delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;

			/* Assign number of working set recalculations and stop runtime measurement. */
			nWSR = iter;
			if ( cputime != 0 )
				*cputime = getCPUtime( ) - starttime;

			THROWERROR( RET_SHIFT_DETERMINATION_FAILED );
			return returnvalue;
		}



		/* (jfrasch) log homotopy origin */
		if (iter == 0) {
			parametricObjFctn_alpha[0] = 0.0;
			parametricObjFctn_f[0] = getObjVal( x );		/** (jfrasch): this is not yet so efficient, maybe better use precomputed results during homotopy
																			 (and for zero value even old result...) */
			real_t fPrime0 = 0.0;
			for ( int ii=0; ii < nV; ++ii ) {
				fPrime0 += x[ii] * delta_g[ii];
			}
			parametricObjFctn_fPrime[0] = fPrime0;
		}




		/* 3) Determination of step direction of X and Y. */
		returnvalue = determineStepDirection(	delta_g,delta_lbA,delta_ubA,delta_lb,delta_ub,
												Delta_bC_isZero, Delta_bB_isZero,
												delta_xFX,delta_xFR,delta_yAC,delta_yFX
												);
		if ( returnvalue != SUCCESSFUL_RETURN )
		{
			delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;

			/* Assign number of working set recalculations and stop runtime measurement. */
			nWSR = iter;
			if ( cputime != 0 )
				*cputime = getCPUtime( ) - starttime;

			THROWERROR( RET_STEPDIRECTION_DETERMINATION_FAILED );
			return returnvalue;
		}

		/* 4) Determination of step length TAU.
		 *    This step along the homotopy path is also taken (without changing working set). */
		returnvalue = performStep(	delta_g, delta_lbA,delta_ubA,delta_lb,delta_ub,
									delta_xFX,delta_xFR,delta_yAC,delta_yFX,
									BC_idx,BC_status,BC_isBound
									);
		if ( returnvalue != SUCCESSFUL_RETURN )
		{
			delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;

			/* Assign number of working set recalculations and stop runtime measurement. */
			nWSR = iter;
			if ( cputime != 0 )
				*cputime = getCPUtime( ) - starttime;

			THROWERROR( RET_STEPLENGTH_DETERMINATION_FAILED );
			return returnvalue;
		}





		/* added by jfrasch:
		 * log tau and objective value
		 */
		/* get derivative of objective in homotopy direction
		 * note: c * deltaLambda needs to be added afterwards (constant QP objective term)
		 */
		real_t fPrime0 = parametricObjFctn_fPrime[iter];
		real_t fPrime1 = 0.0;
		for ( int ii=0; ii < nV; ++ii ) {
			fPrime1 += x[ii] * delta_g[ii] / ( 1 - parametricObjFctn_alpha[iter] );		/* todo: avoid adding up errors, and division by zero (should not occur theoretically, b/c alpha==1 implies tau==1 */
//			if (options.printLevel > PL_NONE)	{
//				printf( "%d: fPrime += %+12.5f * %+12.5f = %12.5f\n", ii, x[ii], delta_g[ii], fPrime1 );
//			}
		}
//		if (options.printLevel > PL_NONE)	{
//			printf( "fPrime1 raw = %+8.5f\n", fPrime1 );
//			printf( "tau = %+.5e\n", tau );
//		}
		/* compute second derivative in homotopy direction:
		 *   exploit that second derivative is constant between AS changes
		 */
		real_t deltaAlpha = ( 1 - parametricObjFctn_alpha[iter] ) * tau;		/* take into account that tau is only relative on each homotopy section */
		real_t fPrimePrime01 = 0.0;
		if ( deltaAlpha > EPS )	{							/* avoid division by zero */
			fPrimePrime01 = (fPrime1 - fPrime0)/deltaAlpha;
		}
		/* get new objective function value by own prediction.
		 * this cheaper than computing it directly,
		 * But: errors might accumulate!
		 */
		real_t f1Pred = parametricObjFctn_f[iter] + fPrime0 * deltaAlpha + .5 * fPrimePrime01 * deltaAlpha * deltaAlpha;

		/* do some logging: log gradient and function value term at new homotopie point,
		 * as well as hessian on previous homotopy interval  */
		parametricObjFctn_alpha[iter+1] = parametricObjFctn_alpha[iter] + deltaAlpha;
		parametricObjFctn_f[iter+1] = f1Pred;
		parametricObjFctn_fPrime[iter+1] = fPrime1;			// note: this still needs to be corrected for const QP objective term  c \cdot deltaLambda
//		printf( "fPrime1 raw = %+8.5f\n", fPrime1 );
		parametricObjFctn_fPrimePrime[iter] = fPrimePrime01;

//		/** now do prediction */
//		real_t predObjVal = objValOld + fPrime0 * alpha1 + .5 * fPrimePrime01 * alpha1 * alpha1;
//		printf( "\n***\nWARNING:tmp computations, only work if tau==1 immediately, and c == 0!\ntau = % 8.5f\nold objVal = %+8.5f\ng0 = %+8.5f\ng1 = %+8.5f\nh0 = %+8.5f\npredicted objVal= %+8.5f\nactual    objVal= %+8.5f\n",
//				tau, objValOld, fPrime0, fPrime1, fPrimePrime01, predObjVal,f1 );





		/* 5) Termination criterion. */
		nV = getNV( );
		nC = getNC( );

		homotopyLength = getRelativeHomotopyLength( g_new,lb_new,ub_new,lbA_new,ubA_new );
		if ( homotopyLength <= options.terminationTolerance )
		{
			// (jfrasch)
//			printf( "\n***\nWARNING:tmp computations, only work if tau==1 immediately, and c == 0!\n" );
//			for ( int ii = 0; ii<= iter; ++ii ) {
//				printf( "%d: alpha = % 8.5f *** f0 = %+8.5f *** g0 = %+8.5f *** h0 = %+8.5f\n",
//							ii,
//							parametricObjFctn_alpha[ii],
//							parametricObjFctn_f[ii],
//							parametricObjFctn_fPrime[ii],
//							parametricObjFctn_fPrimePrime[ii] );
//			}
			// done (jfrasch)


			status = QPS_SOLVED;

			THROWINFO( RET_OPTIMAL_SOLUTION_FOUND );

			if ( printIteration( iter,BC_idx,BC_status,BC_isBound,homotopyLength ) != SUCCESSFUL_RETURN )
				THROWERROR( RET_PRINT_ITERATION_FAILED ); /* do not pass this as return value! */

			nWSR = iter;
			if ( cputime != 0 )
				*cputime = getCPUtime( ) - starttime;

			delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;

			return SUCCESSFUL_RETURN;
		}

		/* 6) Change active set. */
		returnvalue = changeActiveSet( BC_idx,BC_status,BC_isBound );
		if ( returnvalue != SUCCESSFUL_RETURN )
		{
			delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;

			/* Assign number of working set recalculations and stop runtime measurement. */
			nWSR = iter;
			if ( cputime != 0 )
				*cputime = getCPUtime( ) - starttime;

			/* Checks for infeasibility... */
			if ( isInfeasible( ) == BT_TRUE )
			{
				status = QPS_HOMOTOPYQPSOLVED;
				return setInfeasibilityFlag( RET_HOTSTART_STOPPED_INFEASIBILITY );
			}

			/* ...unboundedness... */
			if ( unbounded == BT_TRUE ) /* not necessary since objective function convex! */
				return THROWERROR( RET_HOTSTART_STOPPED_UNBOUNDEDNESS );

			/* ... and throw unspecific error otherwise */
			THROWERROR( RET_HOMOTOPY_STEP_FAILED );
			return returnvalue;
		}

		/* 6a) Possibly refactorise projected Hessian from scratch. */
		if ( ( options.enableCholeskyRefactorisation > 0 ) && ( (iter % options.enableCholeskyRefactorisation) == 0 ) )
		{
			returnvalue = computeProjectedCholesky( );
			if (returnvalue != SUCCESSFUL_RETURN)
			{
				delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
				delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;
				return returnvalue;
			}
		}

		/* 7) Output information of successful QP iteration. */
		status = QPS_HOMOTOPYQPSOLVED;

		if ( printIteration( iter,BC_idx,BC_status,BC_isBound,homotopyLength ) != SUCCESSFUL_RETURN )
			THROWERROR( RET_PRINT_ITERATION_FAILED ); /* do not pass this as return value! */

		/* 8) Perform Ramping Strategy on zero homotopy step or drift correction (if desired). */
		if (BC_status != ST_UNDEFINED)
		{
			if ( ( tau <= EPS ) && ( options.enableRamping == BT_TRUE ) )
				performRamping( );
			else
			if ( (options.enableDriftCorrection > 0) && ((iter+1) % options.enableDriftCorrection == 0) )
				performDriftCorrection( );  /* always returns SUCCESSFUL_RETURN */
		}
	}

	delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
	delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;

	/* stop runtime measurement */
	if ( cputime != 0 )
		*cputime = getCPUtime( ) - starttime;


	/* if program gets to here, output information that QP could not be solved
	 * within the given maximum numbers of working set changes */
	if ( options.printLevel == PL_HIGH )
	{
		#ifndef __XPCTARGET__
		snprintf( messageString,80,"(nWSR = %d)",iter );
		return getGlobalMessageHandler( )->throwWarning( RET_MAX_NWSR_REACHED,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
		#else
		return RET_MAX_NWSR_REACHED;
		#endif
	}
	else
	{
		return RET_MAX_NWSR_REACHED;
	}
}
//{
//	int iter;
//	int nV  = getNV( );
//	int nC  = getNC( );
//
//	/* consistency check */
//	if ( ( getStatus( ) == QPS_NOTINITIALISED )       ||
//		 ( getStatus( ) == QPS_PREPARINGAUXILIARYQP ) ||
//		 ( getStatus( ) == QPS_PERFORMINGHOMOTOPY )   )
//	{
//		return THROWERROR( RET_HOTSTART_FAILED_AS_QP_NOT_INITIALISED );
//	}
//
//	/* start runtime measurement */
//	real_t starttime = 0.0;
//	if ( cputime != 0 )
//		starttime = getCPUtime( );
//
//	/* I) PREPARATIONS */
//	/* 1) Allocate delta vectors of gradient and (constraints') bounds,
//	 *    index arrays and step direction arrays. */
//	//int nFR, nFX, nAC, nIAC;
//
//	real_t* delta_xFR = new real_t[nV];
//	real_t* delta_xFX = new real_t[nV];
//	real_t* delta_yAC = new real_t[nC];
//	real_t* delta_yFX = new real_t[nV];
//
//	real_t* delta_g   = new real_t[nV];
//	real_t* delta_lb  = new real_t[nV];
//	real_t* delta_ub  = new real_t[nV];
//	real_t* delta_lbA = new real_t[nC];
//	real_t* delta_ubA = new real_t[nC];
//
//	returnValue returnvalue;
//	BooleanType Delta_bC_isZero, Delta_bB_isZero;
//
//	int BC_idx;
//	SubjectToStatus BC_status;
//	BooleanType BC_isBound;
//
//	real_t homotopyLength;
//
//	char messageString[80];
//
//
//	/* 2) Update type of bounds and constraints, e.g.
//	 *    a former equality constraint might have become a normal one etc. */
//
//  // (ckirches) disabled this, as inactive but tight bounds may become inactive equalities
//    //            which would then never become active again!
///*
//	if ( setupSubjectToType( lb_new,ub_new,lbA_new,ubA_new ) != SUCCESSFUL_RETURN )
//		return THROWERROR( RET_HOTSTART_FAILED );
//*/
//	/* 3) Reset status flags. */
//	infeasible = BT_FALSE;
//	unbounded  = BT_FALSE;
//
//
//	/* II) MAIN HOMOTOPY LOOP */
//	for( iter=nWSRperformed; iter<nWSR; ++iter )
//	{
//		idxAddB = idxRemB = idxAddC = idxRemC = -1;
//		excAddB = excRemB = excAddC = excRemC = 0;
//
//		if ( isCPUtimeLimitExceeded( cputime,starttime,iter-nWSRperformed ) == BT_TRUE )
//		{
//			/* If CPU time limit is exceeded, stop homotopy loop immediately!
//			* Assign number of working set recalculations (runtime measurement is stopped later). */
//			nWSR = iter;
//			break;
//		}
//
//		status = QPS_PERFORMINGHOMOTOPY;
//
//		#ifndef __XPCTARGET__
//		snprintf( messageString,80,"%d ...",iter );
//		getGlobalMessageHandler( )->throwInfo( RET_ITERATION_STARTED,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
//		#endif
//
//		/* some more definitions */
//		//nFR  = getNFR( );
//		//nFX  = getNFX( );
//		//nAC  = getNAC( );
//		//nIAC = getNIAC( );
//
//
//		/* 2) Detemination of shift direction of the gradient and the (constraints') bounds. */
//		returnvalue = determineDataShift(	g_new,lbA_new,ubA_new,lb_new,ub_new,
//											delta_g,delta_lbA,delta_ubA,delta_lb,delta_ub,
//											Delta_bC_isZero, Delta_bB_isZero
//											);
//		if ( returnvalue != SUCCESSFUL_RETURN )
//		{
//			delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
//			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;
//
//			/* Assign number of working set recalculations and stop runtime measurement. */
//			nWSR = iter;
//			if ( cputime != 0 )
//				*cputime = getCPUtime( ) - starttime;
//
//			THROWERROR( RET_SHIFT_DETERMINATION_FAILED );
//			return returnvalue;
//		}
//
//
//
//		/* (jfrasch) log homotopy origin */
//		if (iter == 0) {
//			parametricObjFctn_alpha[0] = 0.0;
//			parametricObjFctn_f[0] = getObjVal( x );		/** (jfrasch): this is not yet so efficient, maybe better use precomputed results during homotopy
//			 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 (and for zero value even old result...) */
//			real_t fPrime0 = 0.0;
//			for ( int ii=0; ii < nV; ++ii ) {
//				fPrime0 += x[ii] * delta_g[ii];
//			}
//			parametricObjFctn_fPrime[0] = fPrime0;
//		}
//
//
//
//		/* 3) Determination of step direction of X and Y. */
//		returnvalue = determineStepDirection(	delta_g,delta_lbA,delta_ubA,delta_lb,delta_ub,
//												Delta_bC_isZero, Delta_bB_isZero,
//												delta_xFX,delta_xFR,delta_yAC,delta_yFX
//												);
//		if ( returnvalue != SUCCESSFUL_RETURN )
//		{
//			delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
//			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;
//
//			/* Assign number of working set recalculations and stop runtime measurement. */
//			nWSR = iter;
//			if ( cputime != 0 )
//				*cputime = getCPUtime( ) - starttime;
//
//			THROWERROR( RET_STEPDIRECTION_DETERMINATION_FAILED );
//			return returnvalue;
//		}
//
//
//		/* 4) Determination of step length TAU.
//		 *    This step along the homotopy path is also taken (without changing working set). */
//		returnvalue = performStep(	delta_g, delta_lbA,delta_ubA,delta_lb,delta_ub,
//									delta_xFX,delta_xFR,delta_yAC,delta_yFX,
//									BC_idx,BC_status,BC_isBound
//									);
//		if ( returnvalue != SUCCESSFUL_RETURN )
//		{
//			delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
//			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;
//
//			/* Assign number of working set recalculations and stop runtime measurement. */
//			nWSR = iter;
//			if ( cputime != 0 )
//				*cputime = getCPUtime( ) - starttime;
//
//			THROWERROR( RET_STEPLENGTH_DETERMINATION_FAILED );
//			return returnvalue;
//		}
//
//		/* added by jfrasch:
//		 * log tau and objective value
//		 */
//		/* get derivative of objective in homotopy direction
//		 * note: c * deltaLambda needs to be added afterwards (constant QP objective term)
//		 */
//		real_t fPrime0 = parametricObjFctn_fPrime[iter];
//		real_t fPrime1 = 0.0;
//		for ( int ii=0; ii < nV; ++ii ) {
//			fPrime1 += x[ii] * delta_g[ii] / ( 1 - parametricObjFctn_alpha[iter] );		/* todo: avoid adding up errors, and division by zero (should not occur theoretically, b/c alpha==1 implies tau==1 */
////			if (options.printLevel > PL_NONE)	{
////				printf( "%d: fPrime += %+12.5f * %+12.5f = %12.5f\n", ii, x[ii], delta_g[ii], fPrime1 );
////			}
//		}
////		if (options.printLevel > PL_NONE)	{
////			printf( "fPrime1 raw = %+8.5f\n", fPrime1 );
////			printf( "tau = %+.5e\n", tau );
////		}
//		/* compute second derivative in homotopy direction:
//		 *   exploit that second derivative is constant between AS changes
//		 */
//		real_t deltaAlpha = ( 1 - parametricObjFctn_alpha[iter] ) * tau;		/* take into account that tau is only relative on each homotopy section */
//		real_t fPrimePrime01 = 0.0;
//		if ( deltaAlpha > EPS )	{							/* avoid division by zero */
//			fPrimePrime01 = (fPrime1 - fPrime0)/deltaAlpha;
//		}
//		/* get new objective function value by own prediction.
//		 * this cheaper than computing it directly,
//		 * But: errors might accumulate!
//		 */
//		real_t f1Pred = parametricObjFctn_f[iter] + fPrime0 * deltaAlpha + .5 * fPrimePrime01 * deltaAlpha * deltaAlpha;
//
//		/* do some logging: log gradient and function value term at new homotopie point,
//		 * as well as hessian on previous homotopy interval  */
//		parametricObjFctn_alpha[iter+1] = parametricObjFctn_alpha[iter] + deltaAlpha;
//		parametricObjFctn_f[iter+1] = f1Pred;
//		parametricObjFctn_fPrime[iter+1] = fPrime1;			// note: this still needs to be corrected for const QP objective term  c \cdot deltaLambda
////		printf( "fPrime1 raw = %+8.5f\n", fPrime1 );
//		parametricObjFctn_fPrimePrime[iter] = fPrimePrime01;
//
////		/** now do prediction */
////		real_t predObjVal = objValOld + fPrime0 * alpha1 + .5 * fPrimePrime01 * alpha1 * alpha1;
////		printf( "\n***\nWARNING:tmp computations, only work if tau==1 immediately, and c == 0!\ntau = % 8.5f\nold objVal = %+8.5f\ng0 = %+8.5f\ng1 = %+8.5f\nh0 = %+8.5f\npredicted objVal= %+8.5f\nactual    objVal= %+8.5f\n",
////				tau, objValOld, fPrime0, fPrime1, fPrimePrime01, predObjVal,f1 );
//
//
//
//		/* 5) Termination criterion. */
//		homotopyLength = relativeHomotopyLength(g_new, lb_new, ub_new, lbA_new, ubA_new);
//		if ( homotopyLength <= options.terminationTolerance )
//		{
//			// (jfrasch)
////			printf( "\n***\nWARNING:tmp computations, only work if tau==1 immediately, and c == 0!\n" );
////			for ( int ii = 0; ii<= iter; ++ii ) {
////				printf( "%d: alpha = % 8.5f *** f0 = %+8.5f *** g0 = %+8.5f *** h0 = %+8.5f\n",
////							ii,
////							parametricObjFctn_alpha[ii],
////							parametricObjFctn_f[ii],
////							parametricObjFctn_fPrime[ii],
////							parametricObjFctn_fPrimePrime[ii] );
////			}
//			// done (jfrasch)
//
//
//			status = QPS_SOLVED;
//
//			THROWINFO( RET_OPTIMAL_SOLUTION_FOUND );
//
//			if ( options.printLevel > PL_NONE )
//				if ( printIteration( iter,BC_idx,BC_status,BC_isBound ) != SUCCESSFUL_RETURN )
//					THROWERROR( RET_PRINT_ITERATION_FAILED ); /* do not pass this as return value! */
//
//			nWSR = iter;
//			if ( cputime != 0 )
//				*cputime = getCPUtime( ) - starttime;
//
//			delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
//			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;
//
//			return SUCCESSFUL_RETURN;
//		}
//
//
//		/* 6) Change active set. */
//		returnvalue = changeActiveSet( BC_idx,BC_status,BC_isBound );
//		if ( returnvalue != SUCCESSFUL_RETURN )
//		{
//			delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
//			delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;
//
//			/* Assign number of working set recalculations and stop runtime measurement. */
//			nWSR = iter;
//			if ( cputime != 0 )
//				*cputime = getCPUtime( ) - starttime;
//
//			/* Checks for infeasibility... */
//			if ( isInfeasible( ) == BT_TRUE )
//			{
//				status = QPS_HOMOTOPYQPSOLVED;
//				return setInfeasibilityFlag( RET_HOTSTART_STOPPED_INFEASIBILITY );
//			}
//
//			/* ...unboundedness... */
//			if ( unbounded == BT_TRUE ) /* not necessary since objective function convex! */
//				return THROWERROR( RET_HOTSTART_STOPPED_UNBOUNDEDNESS );
//
//			/* ... and throw unspecific error otherwise */
//			THROWERROR( RET_HOMOTOPY_STEP_FAILED );
//			return returnvalue;
//		}
//
//		/* 6.5) Possibly refactorise projected Hessian from scratch. */
//		if (options.enableCholeskyRefactorisation > 0 && iter % options.enableCholeskyRefactorisation == 0)
//		{
//			returnvalue = setupCholeskyDecompositionProjected();
//			if (returnvalue != SUCCESSFUL_RETURN)
//			{
//				delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
//				delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;
//				return returnvalue;
//			}
//		}
//
//
//#ifdef __DEBUG_ITER__
//		nFR = getNFR();
//		nAC = getNAC();
//		int i;
//		real_t stat, bfeas, cfeas, bcmpl, ccmpl, errUnitary, errTQ, Tmaxomin;
//		real_t *grad = new real_t[nV];
//		real_t *AX = new real_t[nC];
//		real_t Tmin, Tmax;
//
//		stat = bfeas = cfeas = bcmpl = ccmpl = errUnitary = errTQ = Tmaxomin = 0.0;
//
//		/* stationarity */
//		for (i = 0; i < nV; i++) grad[i] = g[i] - y[i];
//		H->times(1, 1.0, x, nV, 1.0, grad, nV);
//		A->transTimes(1, -1.0, y+nV, nC, 1.0, grad, nV);
//		for (i = 0; i < nV; i++) if (getAbs(grad[i]) > stat) stat = getAbs(grad[i]);
//
//		/* feasibility */
//		for (i = 0; i < nV; i++) if (lb[i] - x[i] > bfeas) bfeas = lb[i] - x[i];
//		for (i = 0; i < nV; i++) if (x[i] - ub[i] > bfeas) bfeas = x[i] - ub[i];
//		A->times(1, 1.0, x, nV, 0.0, AX, nC);
//		for (i = 0; i < nC; i++) if (lbA[i] - AX[i] > cfeas) cfeas = lbA[i] - AX[i];
//		for (i = 0; i < nC; i++) if (AX[i] - ubA[i] > cfeas) cfeas = AX[i] - ubA[i];
//
//		/* complementarity */
//		for (i = 0; i < nV; i++) if (y[i] > +EPS && getAbs((lb[i] - x[i])*y[i]) > bcmpl) bcmpl = getAbs((lb[i] - x[i])*y[i]);
//		for (i = 0; i < nV; i++) if (y[i] < -EPS && getAbs((ub[i] - x[i])*y[i]) > bcmpl) bcmpl = getAbs((ub[i] - x[i])*y[i]);
//		for (i = 0; i < nC; i++) if (y[nV+i] > +EPS && getAbs((lbA[i]-AX[i])*y[nV+i]) > ccmpl) ccmpl = getAbs((lbA[i]-AX[i])*y[nV+i]);
//		for (i = 0; i < nC; i++) if (y[nV+i] < -EPS && getAbs((ubA[i]-AX[i])*y[nV+i]) > ccmpl) ccmpl = getAbs((ubA[i]-AX[i])*y[nV+i]);
//
//		Tmin = 1.0e16; Tmax = 0.0;
//		for (i = 0; i < nAC; i++)
//			if (getAbs(TT(i,sizeT-i-1)) < Tmin)
//				Tmin = getAbs(TT(i,sizeT-i-1));
//			else if (getAbs(TT(i,sizeT-i-1)) > Tmax)
//				Tmax = getAbs(TT(i,sizeT-i-1));
//		Tmaxomin = Tmax/Tmin;
//
//		if (iter % 10 == 0)
//			fprintf(stderr, "\n%5s %4s %4s %4s %4s %9s %9s %9s %9s %9s %9s %9s %9s\n",
//					"iter", "addB", "remB", "addC", "remC", "hom len", "tau", "stat",
//					"bfeas", "cfeas", "bcmpl", "ccmpl", "Tmin");
//		fprintf(stderr, "%5d ", iter);
//		if (idxAddB >= 0) fprintf(stderr, "%4d ", idxAddB);
//		else fprintf(stderr, "%4s ", " ");
//		if (idxRemB >= 0) fprintf(stderr, "%4d ", idxRemB);
//		else fprintf(stderr, "%4s ", " ");
//		if (idxAddC >= 0) fprintf(stderr, "%4d ", idxAddC);
//		else fprintf(stderr, "%4s ", " ");
//		if (idxRemC >= 0) fprintf(stderr, "%4d ", idxRemC);
//		else fprintf(stderr, "%4s ", " ");
//		fprintf(stderr, "%9.2e %9.2e %9.2e %9.2e %9.2e %9.2e %9.2e %9.2e\n",
//				homotopyLength, tau, stat, bfeas, cfeas, bcmpl, ccmpl, Tmin);
//
//		delete[] AX;
//		delete[] grad;
//#else
//		if (options.printLevel == -1)
//		{
//			const char excStr[] = " ef";
//			if (iter % 10 == 0)
//				fprintf(stdout, "\n%5s %6s %6s %6s %6s %9s %9s\n",
//						"iter", "addB", "remB", "addC", "remC", "hom len", "tau");
//			fprintf(stdout, "%5d ", iter);
//			if (idxAddB >= 0) fprintf(stdout, "%5d%c ", idxAddB, excStr[excAddB]);
//			else fprintf(stdout, "%6s ", " ");
//			if (idxRemB >= 0) fprintf(stdout, "%5d%c ", idxRemB, excStr[excRemB]);
//			else fprintf(stdout, "%6s ", " ");
//			if (idxAddC >= 0) fprintf(stdout, "%5d%c ", idxAddC, excStr[excAddC]);
//			else fprintf(stdout, "%6s ", " ");
//			if (idxRemC >= 0) fprintf(stdout, "%5d%c ", idxRemC, excStr[excRemC]);
//			else fprintf(stdout, "%6s ", " ");
//			fprintf(stdout, "%9.2e %9.2e\n", homotopyLength, tau);
//		}
//#endif /* __DEBUG_ITER__ */
//
//
//		/* 7) Output information of successful QP iteration. */
//		status = QPS_HOMOTOPYQPSOLVED;
//
//		if ( options.printLevel > PL_NONE )
//			if ( printIteration( iter,BC_idx,BC_status,BC_isBound ) != SUCCESSFUL_RETURN )
//				THROWERROR( RET_PRINT_ITERATION_FAILED ); /* do not pass this as return value! */
//
//		/* 8) Perform Ramping Strategy on zero homotopy step or drift correction (if desired). */
//		if (BC_status != ST_UNDEFINED)
//		{
//			if ( ( tau < EPS ) && ( options.enableRamping == BT_TRUE ) )
//				performRamping( );
//			else
//			if ( (options.enableDriftCorrection > 0)
//			  && ((iter+1) % options.enableDriftCorrection == 0) )
//				performDriftCorrection( );  /* always returns SUCCESSFUL_RETURN */
//		}
//	}
//
//	delete[] delta_yAC; delete[] delta_yFX; delete[] delta_xFX; delete[] delta_xFR;
//	delete[] delta_ub; delete[] delta_lb; delete[] delta_ubA; delete[] delta_lbA; delete[] delta_g;
//
//
//	/* stop runtime measurement */
//	if ( cputime != 0 )
//		*cputime = getCPUtime( ) - starttime;
//
//
//	/* if programm gets to here, output information that QP could not be solved
//	 * within the given maximum numbers of working set changes */
//	if ( options.printLevel == PL_HIGH )
//	{
//		#ifndef __XPCTARGET__
//		snprintf( messageString,80,"(nWSR = %d)",iter );
//		return getGlobalMessageHandler( )->throwWarning( RET_MAX_NWSR_REACHED,messageString,__FUNCTION__,__FILE__,__LINE__,VS_VISIBLE );
//		#else
//		return RET_MAX_NWSR_REACHED;
//		#endif
//	}
//	else
//	{
//		return RET_MAX_NWSR_REACHED;
//	}
//}
/* end of solveQP_withHomotopyLogging */



/*
 *	r e m o v e B o u n d
 */
returnValue LoggedSQProblem::removeBound(	int number,
									BooleanType updateCholesky,
									BooleanType allowFlipping,
									BooleanType ensureNZC
									)
{
	for (int kkk = 0; kkk<100; ++kkk)	{
		printf("i was here!!!\n");
	}
	int i, j, ii, jj;
	returnValue returnvalue = SUCCESSFUL_RETURN;
	int addIdx;
	BooleanType addBoundNotConstraint;
	SubjectToStatus addStatus;
	BooleanType exchangeHappened = BT_FALSE;


	/* consistency checks */
	if ( bounds.getStatus( number ) == ST_INACTIVE )
		return THROWERROR( RET_BOUND_NOT_ACTIVE );

	if ( ( getStatus( ) == QPS_NOTINITIALISED )    ||
		 ( getStatus( ) == QPS_AUXILIARYQPSOLVED ) ||
		 ( getStatus( ) == QPS_HOMOTOPYQPSOLVED )  ||
 		 ( getStatus( ) == QPS_SOLVED )            )
	{
		return THROWERROR( RET_UNKNOWN_BUG );
	}

	/* some definitions */
	int nV  = getNV( );
	int nFR = getNFR( );
	int nAC = getNAC( );
	int nZ  = getNZ( );

	int tcol = sizeT - nAC;

	/* 0) PERFORM ZERO CURVATURE TEST. */
	if (ensureNZC == BT_TRUE)
	{
		returnvalue = ensureNonzeroCurvature(BT_TRUE, number, exchangeHappened, addBoundNotConstraint, addIdx, addStatus);

		if (returnvalue != SUCCESSFUL_RETURN)
			return returnvalue;
	}

	/* save index sets and decompositions for flipping bounds strategy */
	if ( ( options.enableFlippingBounds == BT_TRUE ) && ( allowFlipping == BT_TRUE ) && ( exchangeHappened == BT_FALSE ) )
		flipper.set( &bounds,R,&constraints,Q,T );

	/* I) UPDATE INDICES */
//	idxRemB = number;
	if ( bounds.moveFixedToFree( number ) != SUCCESSFUL_RETURN )
		return THROWERROR( RET_REMOVEBOUND_FAILED );

	int* FR_idx;
	bounds.getFree( )->getNumberArray( &FR_idx );

	////////////////////////////////////////////////////////////////////////////////////////////////////
	// THIS stuff needs to be exchanged in QProblem...
	////////////////////////////////////////////////////////////////////////////////////////////////////

	/* I) APPEND <nFR+1>th UNITY VECTOR TO Q. */
	int nnFRp1 = FR_idx[nFR];
//	printf("FR_idx = [ %d %d %d ], nFR = %d\n", FR_idx[0], FR_idx[1], FR_idx[2], getNFR());
	int* FX_idx;
	bounds.getFixed( )->getNumberArray( &FX_idx );
//	printf("FX_idx = [ %d %d %d ], nFX = %d\n", FX_idx[0], FX_idx[1], FX_idx[2], getNFX());
//	printf("nnFRp1 = %d\n", nnFRp1);
	for( i=0; i<nFR; ++i )
	{
		/* ii = FR_idx[i];	 // <- this is commented out, since redundant due to hack below */
//		printf("ii=%d, nFR=%d, nnFRp1=%d, i=%d\n", ii, nFR, nnFRp1, i);	// nnFRp1 should always be identical to number...
//		printf("QQ(ii,nFR) was: %.3e\n", QQ(ii,nFR));
		/* QQ(ii,nFR) = 0.0; // <- this is commented out, since redundant due to hack below */
//		printf("QQ(ii,nFR) is: %.3e\n", QQ(ii,nFR));
//		printf("QQ(nnFRp1,i) was: %.3e\n", QQ(nnFRp1,i));
		QQ(nnFRp1,i) = 0.0;
//		printf("QQ(nnFRp1,i) is: %.3e\n", QQ(nnFRp1,i));
	}
	/* (jfrasch): HACK: completely erase whole column of Q (row by memory access pattern) to make sure Q contains really clean vectors */
	for (i=0; i<nV; ++i)
	{
		QQ(i,nFR) = 0.0;
	}
	/* (end) */
//	printf("QQ(nnFRp1,nFR) was: %.3e\n", QQ(nnFRp1,nFR));
	QQ(nnFRp1,nFR) = 1.0;
//	printf("QQ(nnFRp1,nFR) is: %.3e\n", QQ(nnFRp1,nFR));

	////////////////////////////////////////////////////////////////////////////////////////////////////

	if ( nAC > 0 )
	{
		/* store new column a in a temporary vector instead of shifting T one column to the left and appending a */
		int* AC_idx;
		constraints.getActive( )->getNumberArray( &AC_idx );

		real_t* tmp = new real_t[nAC];
		A->getCol(number, constraints.getActive(), 1.0, tmp);


		/* II) RESTORE TRIANGULAR FORM OF T,
		 *     use column-wise Givens rotations to restore reverse triangular form
		 *     of T = [T A(:,number)], simultanenous change of Q (i.e. Y and Z). */
		real_t c, s, nu;

		for( j=(nAC-1); j>=0; --j )
		{
			computeGivens( tmp[nAC-1-j],TT(nAC-1-j,tcol+j),TT(nAC-1-j,tcol+j),tmp[nAC-1-j],c,s );
			nu = s/(1.0+c);

			for( i=(nAC-j); i<nAC; ++i )
				applyGivens( c,s,nu,tmp[i],TT(i,tcol+j),TT(i,tcol+j),tmp[i] );

			for( i=0; i<=nFR; ++i )
			{
				ii = FR_idx[i];
				/* nZ+1+nAC = nFR+1  /  nZ+(1) = nZ+1 */
				applyGivens( c,s,nu,QQ(ii,nZ+1+j),QQ(ii,nZ+j),QQ(ii,nZ+1+j),QQ(ii,nZ+j) );
			}
		}

		delete[] tmp;
	}


	if ( ( updateCholesky == BT_TRUE ) &&
		 ( hessianType != HST_ZERO )   && ( hessianType != HST_IDENTITY ) )
	{
		/* III) UPDATE CHOLESKY DECOMPOSITION,
		 *      calculate new additional column (i.e. [r sqrt(rho2)]')
		 *      of the Cholesky factor R: */
		real_t z2 = QQ(nnFRp1,nZ);
		real_t rho2 = H->diag(nnFRp1)*z2*z2; /* rho2 = h2*z2*z2 */

		if ( nFR > 0 )
		{
			/* Attention: Index list of free variables has already grown by one! */
			real_t* Hz = new real_t[nFR+1];
			real_t* z = new real_t[nFR+1];
			/* 1) Calculate R'*r = Zfr'*Hfr*z1 + z2*Zfr'*h1 =: Zfr'*Hz + z2*Zfr'*h1 =: rhs and
			 *    rho2 = z1'*Hfr*z1 + 2*z2*h1'*z1 + h2*z2^2 - r'*r =: z1'*Hz + 2*z2*h1'*z1 + h2*z2^2 - r'r */
			for( j=0; j<nFR; ++j )
				z[j] = QQ(FR_idx[j],nZ);
			z[nFR] = 0.0;
			H->times(bounds.getFree(), bounds.getFree(), 1, 1.0, z, nFR+1, 0.0, Hz, nFR+1);

			H->getCol(nnFRp1, bounds.getFree(), 1.0, z);

			if ( nZ > 0 )
			{
				real_t* r = new real_t[nZ];
				real_t* rhs = new real_t[nZ];
				for( i=0; i<nZ; ++i )
					rhs[i] = 0.0;

				/* 2) Calculate rhs. */
				for( j=0; j<nFR; ++j )
				{
					jj = FR_idx[j];
					for( i=0; i<nZ; ++i )
										/* Zfr' * ( Hz + z2*h1 ) */
						rhs[i] += QQ(jj,i) * ( Hz[j] + z2 * z[j] );
				}

				/* 3) Calculate r = R^-T * rhs. */
				if ( backsolveR( rhs,BT_TRUE,BT_TRUE,r ) != SUCCESSFUL_RETURN )
				{
					delete[] z;
					delete[] Hz; delete[] r; delete[] rhs;
					return THROWERROR( RET_REMOVEBOUND_FAILED );
				}


				/* 4) Calculate rho2 = rho^2 = z'*Hz - r'*r
				 *    and store r into R. */
				for( i=0; i<nZ; ++i )
				{
					rho2 -= r[i]*r[i];
					RR(i,nZ) = r[i];
				}

				delete[] rhs; delete[] r;
			}

			for( j=0; j<nFR; ++j )
			{
				jj = FR_idx[j];
							/* z1' * ( Hz + 2*z2*h1 ) */
				rho2 += QQ(jj,nZ) * ( Hz[j] + 2.0*z2*z[j] );
			}

			delete[] z;
			delete[] Hz;
		}

		/* 5) Store rho into R. */
		if ( ( options.enableFlippingBounds == BT_TRUE ) && ( allowFlipping == BT_TRUE ) && ( exchangeHappened == BT_FALSE ) )
		{
			if ( rho2 > options.epsFlipping )
				RR(nZ,nZ) = sqrt( rho2 );
			else
			{
				hessianType = HST_SEMIDEF;

				flipper.get( &bounds,R,&constraints,Q,T );
				bounds.flipFixed(number);
//				idxAddB = number;
//				excAddB = 2;

				switch (bounds.getStatus(number))
				{
					case ST_LOWER:
						lb[number] = ub[number];
						break;
					case ST_UPPER:
						ub[number] = lb[number];
						break;
					default: return THROWERROR( RET_MOVING_BOUND_FAILED );
				}

			}
		}
		else if ( exchangeHappened == BT_FALSE )
		{
			if ( rho2 > ZERO )
				RR(nZ,nZ) = sqrt( rho2 );
			else
			{
				if ( allowFlipping == BT_FALSE )
					RR(nZ,nZ) = 100.0*EPS;
				else
				{
					hessianType = HST_SEMIDEF;
					return THROWERROR( RET_HESSIAN_NOT_SPD );
				}
			}
		}
		else
		{
			/* add bound or constraint */

			/* hessianType = HST_SEMIDEF; */
			RR(nZ,nZ) = 0.0;

			if ( addBoundNotConstraint )
			{
				addBound(addIdx, addStatus, BT_TRUE, BT_FALSE);
//				excAddB = 1;
			}
			else
			{
				addConstraint(addIdx, addStatus, BT_TRUE, BT_FALSE);
//				excAddC = 1;
			}
		}
	}

	return SUCCESSFUL_RETURN;
}






END_NAMESPACE_QPOASES


/*
 *	end of file
 */
