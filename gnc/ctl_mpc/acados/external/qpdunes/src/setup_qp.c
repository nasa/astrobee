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
 *	\file src/dual_qp.c
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */


#include "qp/dual_qp.h"


/* ----------------------------------------------
 * memory allocation
 * 
#>>>>>>                                           */
return_t qpDUNES_setup(	qpData_t* const qpData,
						uint_t nI,
						uint_t nX,
						uint_t nU,
						uint_t* nD,
						qpOptions_t* options
						)
{
	uint_t ii, kk;
	
	int_t nZ = nX+nU;

	int_t nDttl = 0;	/* total number of constraints */

	/* set up options */
	if (options != 0) {
		qpData->options = *options;
	}
	else {
		qpData->options = qpDUNES_setupDefaultOptions();
	}

	/* set up dimensions */
	qpData->nI = nI;
	qpData->nX = nX;
	qpData->nU = nU;
	qpData->nZ = nZ;

	if (nD != 0) {
		for( kk=0; kk<nI+1; ++kk ) {
			nDttl += nD[kk];
		}
	}
	qpData->nDttl = nDttl;

	qpData->intervals = (interval_t**)calloc( nI+1,sizeof(interval_t*) );


	/* normal intervals */
	for( kk=0; kk<nI; ++kk )
	{
		qpData->intervals[kk] = qpDUNES_allocInterval( qpData, nX, nU, nZ, ( (nD != 0) ? nD[kk] : 0 ) );
		
//		qpDUNES_printf("after alloc %d", kk);
//		qpOASES_getNbrActConstr( &(qpData->intervals[kk]->qpSolverQpoases) );

		qpData->intervals[kk]->id = kk;		/* give interval its initial stage index */

		qpData->intervals[kk]->xVecTmp.data  = (real_t*)calloc( nX,sizeof(real_t) );
		qpData->intervals[kk]->uVecTmp.data  = (real_t*)calloc( nU,sizeof(real_t) );
		qpData->intervals[kk]->zVecTmp.data  = (real_t*)calloc( nZ,sizeof(real_t) );
	}
	

	/* last interval */
	qpData->intervals[nI] = qpDUNES_allocInterval( qpData, nX, nU, nX, ( (nD != 0) ? nD[nI] : 0 ) );
	
	qpData->intervals[nI]->id = nI;		/* give interval its initial stage index */

	qpDUNES_setMatrixNull( &( qpData->intervals[nI]->C ) );
	qpDUNES_free( &(qpData->intervals[nI]->c.data) );

	qpData->intervals[nI]->xVecTmp.data  = (real_t*)calloc( nX,sizeof(real_t) );
	qpData->intervals[nI]->uVecTmp.data  = (real_t*)calloc( nU,sizeof(real_t) );
	qpData->intervals[nI]->zVecTmp.data  = (real_t*)calloc( nZ,sizeof(real_t) );
	
	
	/* undefined not-defined lambda parts */
	/* do not free, memory might be needed for interval rotation in MPC context */
	qpData->intervals[0]->lambdaK.isDefined = QPDUNES_FALSE;
	qpData->intervals[nI]->lambdaK1.isDefined = QPDUNES_FALSE;


	/* remainder of qpData struct */
	qpData->lambda.data      = (real_t*)calloc( nX*nI,sizeof(real_t) );
	qpData->deltaLambda.data = (real_t*)calloc( nX*nI,sizeof(real_t) );
	
	qpData->hessian.data  = (real_t*)calloc( (nX*2)*(nX*nI),sizeof(real_t) );
	qpData->cholHessian.data  = (real_t*)calloc( (nX*2)*(nX*nI),sizeof(real_t) );
	qpData->gradient.data = (real_t*)calloc( nX*nI,sizeof(real_t) );
	
	/* allocate unconstrained hessian if needed*/
	if( qpData->options.regType == QPDUNES_REG_UNCONSTRAINED_HESSIAN ||
				qpData->options.regType == QPDUNES_REG_ADD_UNCONSTRAINED_HESSIAN ||
				qpData->options.regType == QPDUNES_REG_ADD_UNCONSTRAINED_HESSIAN_DIAG ||
				(qpData->options.nbrInitialGradientSteps > 0))
	{
		qpData->unconstrainedHessian.data = (real_t*)calloc( (nX*2)*(nX*nI), sizeof(real_t));
		qpData->cholUnconstrainedHessian.data = (real_t*)calloc( (nX*2)*(nX*nI), sizeof(real_t) );
	}
	
	qpData->xVecTmp.data  = (real_t*)calloc( nX,sizeof(real_t) );
	qpData->uVecTmp.data  = (real_t*)calloc( nU,sizeof(real_t) );
	qpData->zVecTmp.data  = (real_t*)calloc( nZ,sizeof(real_t) );
	qpData->xnVecTmp.data  = (real_t*)calloc( nX*nI,sizeof(real_t) );
	qpData->xnVecTmp2.data  = (real_t*)calloc( nX*nI,sizeof(real_t) );
	qpData->xxMatTmp.data = (real_t*)calloc( nX*nX,sizeof(real_t) );
	qpData->xxMatTmp2.data = (real_t*)calloc( nX*nX,sizeof(real_t) );
	qpData->xzMatTmp.data = (real_t*)calloc( nX*nZ,sizeof(real_t) );
	qpData->uxMatTmp.data = (real_t*)calloc( nU*nX,sizeof(real_t) );
	qpData->zxMatTmp.data = (real_t*)calloc( nZ*nX,sizeof(real_t) );
	qpData->zzMatTmp.data = (real_t*)calloc( nZ*nZ,sizeof(real_t) );
	qpData->zzMatTmp2.data = (real_t*)calloc( nZ*nZ,sizeof(real_t) );
	
	
	/* set incumbent objective function value to minus infinity */
	qpData->optObjVal = -qpData->options.QPDUNES_INFTY;
	
	
	/* Set up log struct */
	if ( qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS )
	{
		qpDUNES_setupLog( qpData );
		qpData->log.itLog = (itLog_t*)calloc( qpData->options.maxIter+1, sizeof(itLog_t) );

		for( ii=0; ii<qpData->options.maxIter+1; ++ii ) {
			qpData->log.itLog[ii].numQpoasesIter = (int_t*)calloc( nI+1,sizeof(int_t) );

			if ( qpData->options.logLevel == QPDUNES_LOG_ALL_DATA )
			{
				qpData->log.itLog[ii].regDirections.data = (real_t*)calloc( nX*nI,sizeof(real_t) );

				qpData->log.itLog[ii].lambda.data      = (real_t*)calloc( nX*nI,sizeof(real_t) );
				qpData->log.itLog[ii].deltaLambda.data = (real_t*)calloc( nX*nI,sizeof(real_t) );

				qpData->log.itLog[ii].gradient.data = (real_t*)calloc( nX*nI,sizeof(real_t) );
				qpData->log.itLog[ii].hessian.data  = (real_t*)calloc( (nX*2)*(nX*nI),sizeof(real_t) );
				qpData->log.itLog[ii].cholHessian.data  = (real_t*)calloc( (nX*2)*(nX*nI),sizeof(real_t) );
				#if defined(__ANALYZE_FACTORIZATION__)
				qpData->log.itLog[ii].invHessian.data =  (real_t*)calloc( (nX*nI)*(nX*nI),sizeof(real_t) );
				#endif

				qpData->log.itLog[ii].dz.data = (real_t*)calloc( nI*nZ+nX,sizeof(real_t) );
				qpData->log.itLog[ii].zUnconstrained.data = (real_t*)calloc( nI*nZ+nX,sizeof(real_t) );
				qpData->log.itLog[ii].z.data  = (real_t*)calloc( nI*nZ+nX,sizeof(real_t) );
				qpData->log.itLog[ii].y.data  = (real_t*)calloc( 2*nZ + 2*nDttl,sizeof(real_t) );
				/* TODO: make multiplier definition clean! */
			}
		}
	}
	else {
		qpData->log.itLog = (itLog_t*)calloc( 1, sizeof(itLog_t) );
		/* allocate memory to save number of qpoases iterations */
		qpData->log.itLog[0].numQpoasesIter = (int_t*)calloc( nI+1,sizeof(int_t) );
	}

//	/* reset current active set to force initial Hessian factorization */
//	qpDUNES_indicateDataChange( qpData );
	/* this is done when data is passed */


	qpDUNES_printHeader( qpData );


	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_allocate */


/* ----------------------------------------------
 *
#>>>>>>                                           */
interval_t* qpDUNES_allocInterval(	qpData_t* const qpData,
									uint_t nX,		/* FIXME: just use these temporary, work with nZ later on */
									uint_t nU,		/* FIXME: just use these temporary, work with nZ later on */
									uint_t nV,
									uint_t nD
									)
{
	interval_t* interval = (interval_t*)calloc( 1,sizeof(interval_t) );

	interval->nD = nD;
	interval->nV = nV;

	interval->H.data = (real_t*)calloc( nV*nV,sizeof(real_t) );
	interval->H.sparsityType = QPDUNES_MATRIX_UNDEFINED;
	interval->cholH.data = (real_t*)calloc( nV*nV,sizeof(real_t) );
	interval->cholH.sparsityType = QPDUNES_MATRIX_UNDEFINED;

	interval->g.data  = (real_t*)calloc( nV,sizeof(real_t) );

	interval->q.data  = (real_t*)calloc( nV,sizeof(real_t) );

	interval->C.data = (real_t*)calloc( nX*nV,sizeof(real_t) );
	interval->C.sparsityType = QPDUNES_MATRIX_UNDEFINED;
	interval->c.data = (real_t*)calloc( nX,sizeof(real_t) );

	interval->zLow.data = (real_t*)calloc( nV,sizeof(real_t) );
	interval->zUpp.data = (real_t*)calloc( nV,sizeof(real_t) );
//	qpDUNES_printf("zUpp pointer = %d", (int)(interval->zUpp.data));

	interval->D.data = (real_t*)calloc(  nD*nV,sizeof(real_t) );
	interval->D.sparsityType = QPDUNES_MATRIX_UNDEFINED;
	interval->dLow.data = (real_t*)calloc( nD,sizeof(real_t) );
	interval->dUpp.data = (real_t*)calloc( nD,sizeof(real_t) );

	interval->z.data = (real_t*)calloc( nV,sizeof(real_t) );

	interval->y.data = (real_t*)calloc( 2*nV + 2*nD,sizeof(real_t) );	/* TODO: clean multiplier definition */
	interval->yPrev.data = (real_t*)calloc( 2*nV + 2*nD,sizeof(real_t) );

	interval->lambdaK.data = (real_t*)calloc( nX,sizeof(real_t) );
	interval->lambdaK.isDefined = QPDUNES_TRUE;							/* define both lambda parts by default */
	interval->lambdaK1.data = (real_t*)calloc( nX,sizeof(real_t) );
	interval->lambdaK1.isDefined = QPDUNES_TRUE;

	/* get memory for clipping QP solver */
	interval->qpSolverClipping.qStep.data  = (real_t*)calloc( nV,sizeof(real_t) );
	interval->qpSolverClipping.zUnconstrained.data = (real_t*)calloc( nV,sizeof(real_t) );
	interval->qpSolverClipping.dz.data = (real_t*)calloc( nV,sizeof(real_t) );

	/* get memory for qpOASES QP solver */
	/* TODO: do this only if needed later on in code generated / static memory version */
	/* TODO: utilize special bound version of qpOASES later on for full Hessians, but box constraints */
	interval->qpSolverQpoases.qpoasesObject = qpOASES_constructor( qpData, nV, nD );
	interval->qpSolverQpoases.qFullStep.data  = (real_t*)calloc( nV,sizeof(real_t) );

	/* get memory for objecte function parameterization */
	interval->parametricObjFctn_alpha.data = (real_t*)calloc( qpData->options.maxNumQpoasesIterations + 2 , sizeof(real_t) );
	interval->parametricObjFctn_f.data = (real_t*)calloc( qpData->options.maxNumQpoasesIterations + 2 , sizeof(real_t) );
	interval->parametricObjFctn_fPrime.data = (real_t*)calloc( qpData->options.maxNumQpoasesIterations + 2 , sizeof(real_t) );
	interval->parametricObjFctn_fPrimePrime.data = (real_t*)calloc( qpData->options.maxNumQpoasesIterations + 2 , sizeof(real_t) );
	interval->parametricObjFctn_fSum.data = (real_t*)calloc( qpData->options.maxNumQpoasesIterations + 2 , sizeof(real_t) );


	interval->qpSolverSpecification = QPDUNES_STAGE_QP_SOLVER_UNDEFINED;

	return interval;
}



/* ----------------------------------------------
 * memory deallocation
 * 
 >>>>>>                                           */
return_t qpDUNES_cleanup(	qpData_t* const qpData
						)
{
	uint_t ii, kk;

	/* free all normal intervals */
	for( kk=0; kk<_NI_; ++kk )
	{
		qpDUNES_freeInterval( qpData, qpData->intervals[kk] );
		
		qpDUNES_free( &(qpData->intervals[kk]->xVecTmp.data) );
		qpDUNES_free( &(qpData->intervals[kk]->uVecTmp.data) );
		qpDUNES_free( &(qpData->intervals[kk]->zVecTmp.data) );

		free( qpData->intervals[kk] );
	}
	
	/* free last interval */
	qpDUNES_freeInterval( qpData, qpData->intervals[_NI_] );
	
	qpDUNES_free( &(qpData->intervals[_NI_]->xVecTmp.data) );
	qpDUNES_free( &(qpData->intervals[_NI_]->uVecTmp.data) );
	qpDUNES_free( &(qpData->intervals[_NI_]->zVecTmp.data) );
	
	free( qpData->intervals[_NI_] );


	if ( qpData->intervals != 0 )
		free( qpData->intervals );
	qpData->intervals = 0;
	
	
	/* free remainder of qpData struct */
	qpDUNES_free( &(qpData->lambda.data) );
	qpDUNES_free( &(qpData->deltaLambda.data) );
	
	qpDUNES_free( &(qpData->hessian.data) );
	qpDUNES_free( &(qpData->cholHessian.data) );
	qpDUNES_free( &(qpData->gradient.data) );
	
	
	qpDUNES_free( &(qpData->xVecTmp.data) );
	qpDUNES_free( &(qpData->uVecTmp.data) );
	qpDUNES_free( &(qpData->zVecTmp.data) );
	qpDUNES_free( &(qpData->xnVecTmp.data) );
	qpDUNES_free( &(qpData->xnVecTmp2.data) );
	qpDUNES_free( &(qpData->xxMatTmp.data) );
	qpDUNES_free( &(qpData->xxMatTmp2.data) );
	qpDUNES_free( &(qpData->xzMatTmp.data) );
	qpDUNES_free( &(qpData->uxMatTmp.data) );
	qpDUNES_free( &(qpData->zxMatTmp.data) );
	qpDUNES_free( &(qpData->zzMatTmp.data) );
	qpDUNES_free( &(qpData->zzMatTmp2.data) );
	
	
	/* free log */
	if ( qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS )
	{
		for( ii=0; ii<qpData->options.maxIter+1; ++ii ) {
			free( qpData->log.itLog[ii].numQpoasesIter );

			/* free remainder of data */
			if ( qpData->options.logLevel == QPDUNES_LOG_ALL_DATA )
			{
				qpDUNES_free( &(qpData->log.itLog[ii].regDirections.data) );

				qpDUNES_free( &(qpData->log.itLog[ii].lambda.data) );
				qpDUNES_free( &(qpData->log.itLog[ii].deltaLambda.data) );

				qpDUNES_free( &(qpData->log.itLog[ii].gradient.data) );
				qpDUNES_free( &(qpData->log.itLog[ii].hessian.data) );
				qpDUNES_free( &(qpData->log.itLog[ii].cholHessian.data) );
				#if defined(__ANALYZE_FACTORIZATION__)
				qpDUNES_free( &(qpData->log.itLog[ii].invHessian.data) );
				#endif

				qpDUNES_free( &(qpData->log.itLog[ii].dz.data) );
				qpDUNES_free( &(qpData->log.itLog[ii].zUnconstrained.data) );
				qpDUNES_free( &(qpData->log.itLog[ii].z.data) );
				qpDUNES_free( &(qpData->log.itLog[ii].y.data) );
			}
		}
	}
	else {
		/* free qpoases iterations number log */
		free( qpData->log.itLog[0].numQpoasesIter );
	}


	if ( qpData->log.itLog != 0 )
		free( qpData->log.itLog );
	qpData->log.itLog = 0;

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_deallocate */



/* ----------------------------------------------
 *
#>>>>>>                                           */
void qpDUNES_freeInterval(	qpData_t* const qpData,
						interval_t* const interval
						)
{
	qpDUNES_free( &(interval->H.data) );

	qpDUNES_free( &(interval->g.data) );

	qpDUNES_free( &(interval->q.data) );

	qpDUNES_free( &(interval->cholH.data) );

	qpDUNES_free( &(interval->C.data) );
	qpDUNES_free( &(interval->c.data) );

	qpDUNES_free( &(interval->zLow.data) );
	qpDUNES_free( &(interval->zUpp.data) );

	qpDUNES_free( &(interval->D.data) );
	qpDUNES_free( &(interval->dLow.data) );
	qpDUNES_free( &(interval->dUpp.data) );

	qpDUNES_free( &(interval->z.data) );

	qpDUNES_free( &(interval->y.data) );
	qpDUNES_free( &(interval->yPrev.data) );

	qpDUNES_free( &(interval->lambdaK.data) );
	qpDUNES_free( &(interval->lambdaK1.data) );


	qpDUNES_free( &(interval->qpSolverClipping.qStep.data) );
	qpDUNES_free( &(interval->qpSolverClipping.zUnconstrained.data) );
	qpDUNES_free( &(interval->qpSolverClipping.dz.data) );


	qpOASES_destructor( &(interval->qpSolverQpoases.qpoasesObject) );
	qpDUNES_free( &(interval->qpSolverQpoases.qFullStep.data) );


	qpDUNES_free( &(interval->parametricObjFctn_alpha.data) );
	qpDUNES_free( &(interval->parametricObjFctn_f.data) );
	qpDUNES_free( &(interval->parametricObjFctn_fPrime.data) );
	qpDUNES_free( &(interval->parametricObjFctn_fPrimePrime.data) );
	qpDUNES_free( &(interval->parametricObjFctn_fSum.data) );
}
/*<<< END OF qpDUNES_freeInterval */



///* ----------------------------------------------
// *
//#>>>>>>                                           */
//void qpDUNES_indicateDataChange(	qpData_t* const qpData,
//									interval_t* const interval
//									)
//{
//	int_t ii;
//
//	/* initialize prevIeqStatus to safe values when data was changed to force Hessian refactorization */
//	interval->rebuildHessianBlock = QPDUNES_TRUE;
//}
///*<<< END OF qpDUNES_indicateDataChange */



/* ----------------------------------------------
 *
 >>>>>>                                           */
return_t qpDUNES_init(	qpData_t* const qpData,
						const real_t* const H_,
						const real_t* const g_,
						const real_t* const C_,
						const real_t* const c_,
						const real_t* const zLow_,
						const real_t* const zUpp_,
						const real_t* const D_,
						const real_t* const dLow_,
						const real_t* const dUpp_
						)
{
	int_t kk;

	int_t nDoffset = 0;

	boolean_t isLTI = QPDUNES_FALSE;	/* todo: auto-detect, or specify through interface! */

//	qpDUNES_printMatrixData( H_, _NZ_*_NI_, _NZ_, "H:" );
//	qpDUNES_printMatrixData( g_, _NZ_*_NI_, 1, "g:" );
//	qpDUNES_printMatrixData( C_, _NX_*_NI_, _NZ_, "C:" );
//	qpDUNES_printMatrixData( c_, _NX_*_NI_, 1, "c:" );
//	qpDUNES_printMatrixData( zLow_, _NZ_*_NI_, 1, "zLow:" );
//	qpDUNES_printMatrixData( zUpp_, _NZ_*_NI_, 1, "zUpp:" );
//	qpDUNES_printf( "H_: %d", (int_t)H_ );
//	qpDUNES_printf( "D: %d", (int_t)D_ );

	/** set up regular intervals */
	for( kk=0; kk<_NI_; ++kk )
	{
		qpDUNES_setupRegularInterval( qpData, qpData->intervals[kk],
								   offsetArray(H_, kk*_NZ_*_NZ_), 0, 0, 0, offsetArray(g_, kk*_NZ_),
								   offsetArray(C_, kk*_NX_*_NZ_), 0, 0, offsetArray(c_, kk*_NX_),
								   offsetArray(zLow_, kk*_NZ_), offsetArray(zUpp_, kk*_NZ_), 0, 0, 0, 0,
								   offsetArray(D_, nDoffset*_NZ_), offsetArray(dLow_, nDoffset), offsetArray(dUpp_, nDoffset) );
		nDoffset += qpData->intervals[kk]->nD;
	}
	/** set up final interval */
	qpDUNES_setupFinalInterval( qpData, qpData->intervals[_NI_],
							 offsetArray(H_, _NI_*_NZ_*_NZ_), offsetArray(g_, _NI_*_NZ_),
							 offsetArray(zLow_, _NI_*_NZ_), offsetArray(zUpp_, _NI_*_NZ_),
							 offsetArray(D_, nDoffset*_NZ_), offsetArray(dLow_, nDoffset), offsetArray(dUpp_, nDoffset) );


//	qpDUNES_printMatrixData( D_, nDoffset, _NZ_, "D (first):" );
//	qpDUNES_printMatrixData( D_, qpData->intervals[_NI_]->nD, _NX_, "D (last):" );
//	nDoffset += qpData->intervals[_NI_]->nD;
//	qpDUNES_printMatrixData( dLow_, nDoffset, 1, "dLow:" );
//	qpDUNES_printMatrixData( dUpp_, nDoffset, 1, "dUpp:" );


	/** determine local QP solvers and set up auxiliary data */
	qpDUNES_setupAllLocalQPs( qpData, isLTI );

	/** setup unconstrained Hessian if required */
	if( qpData->options.regType == QPDUNES_REG_UNCONSTRAINED_HESSIAN ||
				qpData->options.regType == QPDUNES_REG_ADD_UNCONSTRAINED_HESSIAN ||
				qpData->options.regType == QPDUNES_REG_ADD_UNCONSTRAINED_HESSIAN_DIAG ||
				(qpData->options.nbrInitialGradientSteps > 0))
	{
		/** compute Cholesky factorization of default newton hessian */
		qpDUNES_setupUnconstrainedNewtonSystem(qpData);
	}

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_init */


/* ----------------------------------------------
 * Combined shift and update function
 >>>>>>                                           */
return_t qpDUNES_shiftAndUpdate(	qpData_t* const qpData,
									const real_t* const H_,
									const real_t* const g_,
									const real_t* const C_,
									const real_t* const c_,
									const real_t* const zLow_,
									const real_t* const zUpp_,
									const real_t* const D_,
									const real_t* const dLow_,
									const real_t* const dUpp_
									)
{
	qpDUNES_shiftLambda( qpData );			/* shift multipliers */
	qpDUNES_shiftIntervals( qpData );		/* shift intervals (particulary important when using qpOASES for underlying local QPs) */

	return qpDUNES_updateData( qpData, H_, g_, C_, c_, zLow_,zUpp_, D_,dLow_,dUpp_ );	/* stage QPs are resolved here */
}
/*<<< END OF qpDUNES_shiftAndUpdate */


/* ----------------------------------------------
 *
 >>>>>>                                           */
return_t qpDUNES_updateData(	qpData_t* const qpData,
								const real_t* const H_,
								const real_t* const g_,
								const real_t* const C_,
								const real_t* const c_,
								const real_t* const zLow_,
								const real_t* const zUpp_,
								const real_t* const D_,
								const real_t* const dLow_,
								const real_t* const dUpp_
								)
{
	int_t kk;

	int_t nDoffset = 0;
	return_t statusFlag = QPDUNES_OK;
	return_t nextFlag;

	/** setup regular intervals */
	for( kk=0; kk<_NI_; ++kk )
	{
		if (qpData->intervals[kk]->nD > 0) {
			nextFlag = qpDUNES_updateIntervalData( qpData, qpData->intervals[kk],
									 offsetArray(H_, kk*_NZ_*_NZ_), offsetArray(g_, kk*_NZ_),
									 offsetArray(C_, kk*_NX_*_NZ_), offsetArray(c_, kk*_NX_),
									 offsetArray(zLow_, kk*_NZ_), offsetArray(zUpp_, kk*_NZ_),
									 offsetArray(D_, nDoffset*_NZ_), offsetArray(dLow_, nDoffset), offsetArray(dUpp_, nDoffset),
									 0 );
			nDoffset += qpData->intervals[kk]->nD;
		}
		else {
			nextFlag = qpDUNES_updateIntervalData( qpData, qpData->intervals[kk],
									 offsetArray(H_, kk*_NZ_*_NZ_), offsetArray(g_, kk*_NZ_),
									 offsetArray(C_, kk*_NX_*_NZ_), offsetArray(c_, kk*_NX_),
									 offsetArray(zLow_, kk*_NZ_), offsetArray(zUpp_, kk*_NZ_),
									 0, 0, 0,
									 0 );
		}
		if( nextFlag != QPDUNES_OK ) statusFlag = nextFlag;
	}
	/** set up final interval */
	if (qpData->intervals[_NI_]->nD > 0) {
		nextFlag = qpDUNES_updateIntervalData( qpData, qpData->intervals[_NI_],
							 offsetArray(H_, _NI_*_NZ_*_NZ_), offsetArray(g_, _NI_*_NZ_),
							 0, 0,
							 offsetArray(zLow_, _NI_*_NZ_), offsetArray(zUpp_, _NI_*_NZ_),
							 offsetArray(D_, nDoffset*_NZ_), offsetArray(dLow_, nDoffset), offsetArray(dUpp_, nDoffset),
							 0 );
	}
	else {
		nextFlag = qpDUNES_updateIntervalData( qpData, qpData->intervals[_NI_],
							 offsetArray(H_, _NI_*_NZ_*_NZ_), offsetArray(g_, _NI_*_NZ_),
							 0, 0,
							 offsetArray(zLow_, _NI_*_NZ_), offsetArray(zUpp_, _NI_*_NZ_),
							 0, 0, 0,
							 0 );
	}
	if( nextFlag != QPDUNES_OK ) statusFlag = nextFlag;


	return ( statusFlag == QPDUNES_OK ) ? QPDUNES_OK : QPDUNES_ERR_INVALID_ARGUMENT;
}
/*<<< END OF qpDUNES_updateData */



return_t qpDUNES_setupSimpleBoundedInterval(	qpData_t* const qpData,
											interval_t* interval,
											const real_t* const Q,
											const real_t* const R,
											const real_t* const S,
											const real_t* const A,
											const real_t* const B,
											const real_t* const c,
											const real_t* const xLow,
											const real_t* const xUpp,
											const real_t* const uLow,
											const real_t* const uUpp
											)
{
	if ( R != 0 ) {
		return qpDUNES_setupRegularInterval( qpData, interval, 0, Q, R, S, 0, 0, A, B, c, 0, 0, xLow, xUpp, uLow, uUpp, 0, 0, 0 );
// 		return qpDUNES_setupRegularInterval( qpData, interval, 0, Q, R, S, 0, A, B, c, 0, 0, 0, xLow, xUpp, uLow, uUpp, 0, xRef, uRef );
	}
	else {	/* final interval */
		return qpDUNES_setupFinalInterval( qpData, interval, Q, 0, xLow, xUpp, 0, 0, 0 );
// 		return qpDUNES_setupFinalInterval( qpData, interval, 0, Q, 0, 0, 0, xLow, xUpp, 0, xRef );
	}
}


/* ----------------------------------------------
 * data setup function
 * 
 >>>>>>                                           */
return_t qpDUNES_setupRegularInterval(	qpData_t* const qpData,
									interval_t* interval,
									const real_t* const H_,
									const real_t* const Q_,
									const real_t* const R_,
									const real_t* const S_,
									const real_t* const g_,
									const real_t* const C_,
									const real_t* const A_,
									const real_t* const B_,
									const real_t* const c_, 
									const real_t* const zLow_,
									const real_t* const zUpp_,
									const real_t* const xLow_,
									const real_t* const xUpp_,
									const real_t* const uLow_,
									const real_t* const uUpp_,
									const real_t* const D_,
									const real_t* const dLow_,
									const real_t* const dUpp_
									)
{
	int_t ii, jj;
	
	int_t nD = interval->nD;	// TODO: enable ND static for full static memory!
	int_t nV = interval->nV;

	vv_matrix_t* H = &(interval->H);
	xz_matrix_t* C = &(interval->C);
	
	sparsityType_t sparsityQ;
	sparsityType_t sparsityR;

	/** (1) quadratic term of cost function */
	if ( H_ != 0 ) {	/* Hessian given directly */
		if (H->sparsityType == QPDUNES_MATRIX_UNDEFINED) {
			H->sparsityType = qpDUNES_detectMatrixSparsity( H_, _NZ_, _NZ_ );
		}
		qpDUNES_updateMatrixData( (matrix_t*)H, H_, _NZ_, _NZ_ );
	}
	else {	/* assemble Hessian */
		/* TODO: move Q, R out to MPC module */
		/* detect sparsity of Q, R */
		sparsityQ =  (Q_ != 0) ? qpDUNES_detectMatrixSparsity( Q_, _NX_, _NX_ ) : QPDUNES_IDENTITY;
		sparsityR =  (R_ != 0) ? qpDUNES_detectMatrixSparsity( R_, _NU_, _NU_ ) : QPDUNES_IDENTITY;

		if ( S_ != 0 ) {	/* assemble full (dense) Hessian */
			H->sparsityType = QPDUNES_DENSE;
			/* Hessian written completely; TODO: check if one triangular half Hessian would be sufficient */
			for ( ii=0; ii<_NX_; ++ii ) {
				if ( Q_ != 0 ) {			/* Q part */
					for( jj=0; jj<_NX_; ++jj ) {
						accH( ii,jj ) = Q_[ii*_NX_+jj];
					}
				}
				else {
					accH( ii,ii ) = qpData->options.regParam;
				}
				for( jj=0; jj<_NU_; ++jj ) {	/* S part */
					accH( ii,_NX_+jj ) = S_[ii*_NU_+jj];
				}
			}
			for ( ii=0; ii<_NU_; ++ii ) {
				for( jj=0; jj<_NX_; ++jj ) {	/* S^T part */
					accH( _NX_+ii,jj ) = S_[jj*_NU_+ii];
				}
				if ( R_ != 0 ) {			/* R part */
					for( jj=0; jj<_NU_; ++jj ) {
						accH( _NX_+ii,_NX_+jj ) = R_[ii*_NU_+jj];
					}
				}
				else {
					accH( _NX_+ii,_NX_+ii ) = qpData->options.regParam;
				}
			}
		}
		else {	/* write Hessian blocks */
			if ( (sparsityQ == QPDUNES_DENSE) || (sparsityR == QPDUNES_DENSE) ) {
				H->sparsityType = QPDUNES_DENSE;
				for ( ii=0; ii<_NX_; ++ii ) {
					/* Q part */
					if ( Q_ != 0 ) {
						for( jj=0; jj<_NX_; ++jj ) {
							accH( ii,jj ) = Q_[ii*_NX_+jj];
						}
					}
					else {
						for( jj=0; jj<ii; ++jj ) {
							accH( ii,jj ) = 0.;
						}
						accH( ii,ii ) = qpData->options.regParam;
						for( jj=ii+1; jj<_NX_; ++jj ) {
							accH( ii,jj ) = 0.;
						}
					}
					/* S part */
					for( jj=_NX_; jj<_NZ_; ++jj ) {
						accH( ii,jj ) = 0.;
					}
				}
				for ( ii=0; ii<_NU_; ++ii ) {
					/* S^T part */
					for( jj=0; jj<_NX_; ++jj ) {
						accH( _NX_+ii,jj ) = 0.;
					}
					/* R part */
					if ( R_ != 0 ) {
						for( jj=0; jj<_NU_; ++jj ) {
							accH( _NX_+ii,_NX_+jj ) = R_[ii*_NU_+jj];
						}
					}
					else {
						for( jj=0; jj<ii; ++jj ) {
							accH( _NX_+ii,_NX_+jj ) = 0.;
						}
						accH( _NX_+ii,_NX_+ii ) = qpData->options.regParam;
						for( jj=ii+1; jj<_NX_; ++jj ) {
							accH( _NX_+ii,_NX_+jj ) = 0.;
						}
					}
				}
			}
			else {	/* Q and R block are diagonal or identity */
				if ( (sparsityQ == QPDUNES_IDENTITY) && (sparsityR == QPDUNES_IDENTITY) ) {
					H->sparsityType = QPDUNES_IDENTITY;
					/* no data needs to be written */
				}
				else {
					H->sparsityType = QPDUNES_DIAGONAL;
					/* write diagonal in first line for cache efficiency */
					/* Q part */
					if (sparsityQ == QPDUNES_IDENTITY) {
						for( ii=0; ii<_NX_; ++ii) {
							accH( 0,ii ) = 1.;
						}
					}
					else {
						for( ii=0; ii<_NX_; ++ii) {
							accH( 0,ii ) = Q_[ii*_NX_+ii];
						}
					}
					/* R part */
					if (sparsityR == QPDUNES_IDENTITY) {
						for( ii=0; ii<_NU_; ++ii) {
							accH( 0,_NX_+ii ) = 1.;
						}
					}
					else {
						for( ii=0; ii<_NU_; ++ii) {
							accH( 0,_NX_+ii ) = R_[ii*_NU_+ii];
						}
					}
				}
			}
		} /* end of write Hessian blocks */
	} /* end of Hessian */
	
	

	/** (2) linear term of cost function */
	if ( g_ != 0 ) {
		qpDUNES_setupVector( (vector_t*)&(interval->g), g_, nV );
	}
	else {
		qpDUNES_setupZeroVector( (vector_t*)&(interval->g), nV );
	}


	/** (3) dynamic system */
	if (C->sparsityType == QPDUNES_MATRIX_UNDEFINED) {
		C->sparsityType = QPDUNES_DENSE;
	}
	if ( C_ != 0 ) {
		/* set up C directly */
		qpDUNES_updateMatrixData( (matrix_t*)C, C_, _NX_, _NZ_ );
	}
	else {
		/* TODO: move assembly out to MPC interface */
		if ( (A_ != 0) && (B_ != 0) ) {
			/* build up C */
			for ( ii=0; ii<_NX_; ++ii ) {
				for( jj=0; jj<_NX_; ++jj ) {
					accC( ii, jj ) = A_[ii*_NX_+jj];
				}
				for( jj=0; jj<_NU_; ++jj ) {
					accC( ii, _NX_+jj ) = B_[ii*_NU_+jj];
				}
			}
		}
		else {
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Matrices either for C or for A and B need to be supplied." );
			return QPDUNES_ERR_INVALID_ARGUMENT;
		}
	}
	
	if ( c_ != 0 ) {
		qpDUNES_setupVector( (vector_t*)&(interval->c), c_, _NX_ );
	}
	else {
		qpDUNES_setupZeroVector( (vector_t*)&(interval->c), _NX_ );
	}
	
	
	/** (4) bounds */
	qpDUNES_setupUniformVector( (vector_t*)&(interval->zLow), -qpData->options.QPDUNES_INFTY, nV );
	qpDUNES_updateSimpleBoundVector( qpData, (vector_t*)&(interval->zLow), zLow_, xLow_, uLow_ );
//	qpDUNES_printf("zUpp: %d", (int)(interval->zUpp.data));
//	qpDUNES_printf("interval id: %d", interval->id);
//	qpDUNES_printf("interval pointer %d", (int)interval);
//	qpDUNES_printf("nV: %d", nV);
	qpDUNES_setupUniformVector( (vector_t*)&(interval->zUpp), qpData->options.QPDUNES_INFTY, nV );
	qpDUNES_updateSimpleBoundVector( qpData, (vector_t*)&(interval->zUpp), zUpp_, xUpp_, uUpp_ );


	/** (5) constraints */
	/*  - Matrix */
	if ( D_ != 0 ) {	/* generically bounded QP */
		if (interval->D.sparsityType == QPDUNES_MATRIX_UNDEFINED) {
			interval->D.sparsityType = QPDUNES_DENSE;	/* currently only dense matrices are supported in affine constraints */
		}
		qpDUNES_updateMatrixData( (matrix_t*)&(interval->D), D_, nD, _NZ_ );

	}
	else {	/* simply bounded QP */
		qpDUNES_setMatrixNull( (matrix_t*)&(interval->D) );
	}
	
	/*  - Vectors */
	qpDUNES_updateVector( (vector_t*)&(interval->dLow), dLow_, nD );
	qpDUNES_updateVector( (vector_t*)&(interval->dUpp), dUpp_, nD );

	
	/* reset current active set to force Hessian refactorization
	 * (needed if matrix data entering the Newton Hessian has changed) */
	if ( (H_ !=0) || (Q_ !=0) || (C_ != 0) || (A_ !=0) || (D_ != 0) ) {
		interval->rebuildHessianBlock = QPDUNES_TRUE;
	}


	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_setupRegularInterval */



/* ----------------------------------------------
 * data setup function
 * 
 >>>>>>                                           */
return_t qpDUNES_setupFinalInterval(	qpData_t* const qpData,
										interval_t* interval,
										const real_t* const H_,
										const real_t* const g_,
										const real_t* const zLow_,
										const real_t* const zUpp_,
										const real_t* const D_,
										const real_t* const dLow_,
										const real_t* const dUpp_
										)
{
	int_t nV = interval->nV;
	int_t nD = interval->nD;
	
	vv_matrix_t* H = &(interval->H);
	

	/** (1) quadratic term of cost function */
 	if ( H_ != 0 ) {	/* H given */
		if (H->sparsityType == QPDUNES_MATRIX_UNDEFINED) {
	 		H->sparsityType = qpDUNES_detectMatrixSparsity( H_, nV, nV );
		}
 		qpDUNES_updateMatrixData( (matrix_t*)H, H_, nV, nV );
 	}
	else {
		qpDUNES_setupScaledIdentityMatrix( _NX_, qpData->options.regParam, (matrix_t*)H );
	}


 	/** (2) linear term of cost function */
	if ( g_ != 0 ) {
		qpDUNES_setupVector( (vector_t*)&(interval->g), g_, nV );
	}
	else {
		qpDUNES_setupZeroVector( (vector_t*)&(interval->g), nV );
	}


	/** (3) local bounds */
	qpDUNES_setupUniformVector( (vector_t*)&(interval->zLow), -qpData->options.QPDUNES_INFTY, nV );
	qpDUNES_updateVector( (vector_t*)&(interval->zLow), zLow_, nV );
	qpDUNES_setupUniformVector( (vector_t*)&(interval->zUpp), qpData->options.QPDUNES_INFTY, nV );
	qpDUNES_updateVector( (vector_t*)&(interval->zUpp), zUpp_, nV );


	/** (4) local constraints */
	if ( D_ != 0 ) {	/* generically bounded QP */
		if (interval->D.sparsityType == QPDUNES_MATRIX_UNDEFINED) {
			interval->D.sparsityType = QPDUNES_DENSE;	/* currently only dense matrices are supported in affine constraints */
		}
		qpDUNES_updateMatrixData( (matrix_t*)&(interval->D), D_, nD, nV );
	}
	else {	/* simply bounded QP */
		qpDUNES_setMatrixNull( (matrix_t*)&(interval->D) );
	}
	
	qpDUNES_updateVector( (vector_t*)&(interval->dLow), dLow_, nD );
	qpDUNES_updateVector( (vector_t*)&(interval->dUpp), dUpp_, nD );


	/* reset current active set to force Hessian refactorization
	 * (needed if matrix data entering the Newton Hessian has changed) */
	if ( (H_ !=0) || (D_ != 0) ) {
		interval->rebuildHessianBlock = QPDUNES_TRUE;
	}

		
	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_setupFinalIntervalData */




/* ----------------------------------------------
 * data update function
 *
 >>>>>>                                           */
return_t qpDUNES_updateIntervalData(	qpData_t* const qpData,
										interval_t* interval,
										const real_t* const H_,
										const real_t* const g_,
										const real_t* const C_,
										const real_t* const c_,
										const real_t* const zLow_,
										const real_t* const zUpp_,
										const real_t* const D_,
										const real_t* const dLow_,
										const real_t* const dUpp_,
										vv_matrix_t* const cholH
										)
{
	int_t ii;

	int_t nD = interval->nD;
	int_t nV = interval->nV;

	int_t nQpoasesIter;

	return_t statusFlag;

//	qpDUNES_printMatrixData( H_, _NZ_, _NZ_, "H:" );
//	qpDUNES_printMatrixData( g_, _NZ_*_NI_, 1, "g:" );
//	qpDUNES_printMatrixData( C_, _NX_*_NI_, _NZ_, "C:" );
//	qpDUNES_printMatrixData( c_, _NX_*_NI_, 1, "c:" );
//	qpDUNES_printMatrixData( zLow_, _NZ_, 1, "zLow:" );
//	qpDUNES_printMatrixData( zUpp_, _NZ_, 1, "zUpp:" );
//	qpDUNES_printf( "new H_: %d", (int_t)H_ );
//	qpDUNES_printf( "D: %d", (int_t)D_ );
//	if (dLow_)  qpDUNES_printMatrixData( dLow_, nD, nV, "dLow:" );
//	if (dUpp_)  qpDUNES_printMatrixData( dUpp_, nD, nV, "dUpp:" );


	boolean_t refactorHessian;
	boolean_t H_changed;
	boolean_t g_changed;
	boolean_t zLow_changed;
	boolean_t zUpp_changed;
	boolean_t D_changed;
	boolean_t dLow_changed;
	boolean_t dUpp_changed;

	/** consistency checks */
	if ( ( (D_ != 0) || (dLow_ != 0) || (dUpp_ != 0) ) &&
		 ( interval->qpSolverSpecification != QPDUNES_STAGE_QP_SOLVER_QPOASES ) )
	{
		qpDUNES_printError( qpData, __FILE__, __LINE__, "Affine constraint data update on stage %d detected,\n          but incompatible stage stage solver %d initially selected.", interval->id, (int)interval->qpSolverSpecification );
		return QPDUNES_ERR_INVALID_ARGUMENT;
	}


	/** copy data */
	qpDUNES_updateMatrixData( (matrix_t*)&(interval->H), H_, nV, nV );
	qpDUNES_updateVector( (vector_t*)&(interval->g), g_, nV );

	qpDUNES_updateMatrixData( (matrix_t*)&(interval->C), C_, _NX_, _NZ_ );
	qpDUNES_updateVector( (vector_t*)&(interval->c), c_, _NX_ );

	qpDUNES_updateVector( (vector_t*)&(interval->zLow), zLow_, nV );
	qpDUNES_updateVector( (vector_t*)&(interval->zUpp), zUpp_, nV );
//	qpDUNES_printMatrixData( interval->zLow.data, 1, interval->nV, "i[%3d]: zLowReceived:", interval->id);
//	qpDUNES_printMatrixData( interval->zUpp.data, 1, interval->nV, "i[%3d]: zUppReceived:", interval->id);

	/* affine constraints */
	qpDUNES_updateMatrixData( (matrix_t*)&(interval->D), D_, nD, nV );
	qpDUNES_updateVector( (vector_t*)&(interval->dLow), dLow_, nD );
	qpDUNES_updateVector( (vector_t*)&(interval->dUpp), dUpp_, nD );


	/* reset current active set to force Hessian refactorization
	 * (needed if matrix data entering the Newton Hessian has changed) */
	if ( (H_ !=0) || (C_ != 0) || (D_ != 0) ) {
		interval->rebuildHessianBlock = QPDUNES_TRUE;
	}


	/* update stage QPs */
	switch ( interval->qpSolverSpecification ) {
		case QPDUNES_STAGE_QP_SOLVER_CLIPPING:
			refactorHessian = QPDUNES_FALSE;
			/* check if Hessian needs to be refactored */
			if ( H_ != 0 ) {
				if (cholH != 0) {	/* factorization provided */
					qpDUNES_copyMatrix( (matrix_t*)&(interval->cholH), (matrix_t*)cholH, nV, nV );
				}
				else {				/* no factorization provided */
					refactorHessian = QPDUNES_TRUE;
				}
			}

			/* do re-setup of clipping QP solver only if needed */
			if ( (H_ != 0) || (g_ != 0) || (zLow_ != 0) || (zUpp_ != 0) )
			{
				statusFlag = qpDUNES_setupClippingSolver( qpData, interval, refactorHessian );

				/* check if Hessian contribution changed */
				if (interval->rebuildHessianBlock != QPDUNES_TRUE)	{
					for (ii = 0; ii < nV; ++ii ) {
						if ( (boolean_t) ( (interval->y.data[2 * ii] * interval->y.data[2 * ii + 1]) < 0 )  !=
							 (boolean_t) ( (interval->yPrev.data[2 * ii] * interval->yPrev.data[2 * ii + 1]) < 0 ) )	/* AS change if sign is different */
						{
							interval->rebuildHessianBlock = QPDUNES_TRUE;
							break;
						}
					}
				}
			}
			break;

		case QPDUNES_STAGE_QP_SOLVER_QPOASES:
			H_changed 	 = ( H_    != 0 ) ? QPDUNES_TRUE : QPDUNES_FALSE;
			g_changed 	 = ( g_    != 0 ) ? QPDUNES_TRUE : QPDUNES_FALSE;
			zLow_changed = ( zLow_ != 0 ) ? QPDUNES_TRUE : QPDUNES_FALSE;
			zUpp_changed = ( zUpp_ != 0 ) ? QPDUNES_TRUE : QPDUNES_FALSE;
			D_changed 	 = ( D_    != 0 ) ? QPDUNES_TRUE : QPDUNES_FALSE;
			dLow_changed = ( dLow_ != 0 ) ? QPDUNES_TRUE : QPDUNES_FALSE;
			dUpp_changed = ( dUpp_ != 0 ) ? QPDUNES_TRUE : QPDUNES_FALSE;

//			if (interval->id == 0) 	qpDUNES_printMatrixData( interval->z.data, 1, interval->nV, "i[%3d]: z@resolve:", interval->id);

			statusFlag = qpDUNES_updateQpoases( qpData, interval, H_changed, g_changed, zLow_changed, zUpp_changed, D_changed, dLow_changed, dUpp_changed, &nQpoasesIter );
//			if (interval->id == 0) 	qpDUNES_printMatrixData( interval->z.data, 1, interval->nV, "i[%3d]: z@AfterResolve:", interval->id);
			if (nQpoasesIter >= 1)	interval->rebuildHessianBlock = QPDUNES_TRUE;
//			qpDUNES_printf("nQpoasesIter[%d] = %d", interval->id, nQpoasesIter);
//			interval->rebuildHessianBlock = QPDUNES_TRUE;
			break;

		default:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Unknown stage QP solver selected." );
			return QPDUNES_ERR_INVALID_ARGUMENT;
	}

	switch (statusFlag) {
		case QPDUNES_OK:
			return QPDUNES_OK;

		case QPDUNES_ERR_STAGE_QP_INFEASIBLE:
			qpDUNES_printError(qpData, __FILE__, __LINE__, "Update with infeasible data on stage %d.", interval->id );
			return QPDUNES_ERR_STAGE_QP_INFEASIBLE;

		default:
			qpDUNES_printError(qpData, __FILE__, __LINE__, "Update of stage %d failed for unexpected reason.", interval->id );
			return statusFlag;
	}
}
/*<<< END OF qpDUNES_updateIntervalData */




/* ----------------------------------------------
 * set up of local QP
 * 
 >>>>>>                                           */
return_t qpDUNES_setupAllLocalQPs(	qpData_t* const qpData,
									boolean_t isLTI
									)
{
	int_t kk;
	interval_t* interval;
	return_t statusFlag;
	
	boolean_t refactorStageHessian = QPDUNES_FALSE;


	/* (1) set up initial lambda guess */
	qpDUNES_updateVector( &(qpData->intervals[0]->lambdaK1), &(qpData->lambda.data[0]), _NX_ );
	for( kk=1; kk<_NI_; ++kk ) {
		qpDUNES_updateVector( &(qpData->intervals[kk]->lambdaK), &(qpData->lambda.data[(kk-1)*_NX_]), _NX_ );
		qpDUNES_updateVector( &(qpData->intervals[kk]->lambdaK1), &(qpData->lambda.data[kk*_NX_]), _NX_ );
	}
	qpDUNES_updateVector( &(qpData->intervals[_NI_]->lambdaK), &(qpData->lambda.data[(_NI_-1)*_NX_]), _NX_ );


	/* (2) decide which QP solver to use and set up */
	for( kk=0; kk<_NI_+1; ++kk ) {
		interval = qpData->intervals[kk];

		/* decide which stage QP solver to use */
		if ( interval->qpSolverSpecification == QPDUNES_STAGE_QP_SOLVER_UNDEFINED )
		{
			if ( ( interval->H.sparsityType >= QPDUNES_DIAGONAL ) &&
				 ( interval->nD == 0 ) )
			{
				/* (a) Clipping Solver */
				interval->qpSolverSpecification = QPDUNES_STAGE_QP_SOLVER_CLIPPING;
				if( qpData->options.printLevel >= 3 ) {
					qpDUNES_printf("[qpDUNES] Stage %d: Using clipping QP solver. %lf", kk,12.);
				}

				if ( (isLTI) && (kk != 0) && (kk != _NI_) )	{
					/* only first Hessian needs to be factorized in LTI case, others can be copied;
					 * last one might still be different, due to terminal cost, even in LTI case */
					qpDUNES_copyMatrix( &(interval->cholH), &(qpData->intervals[0]->cholH), interval->nV, interval->nV );

					refactorStageHessian = QPDUNES_FALSE;
				}
				else {
					refactorStageHessian = QPDUNES_TRUE;
				}

				statusFlag = qpDUNES_setupClippingSolver( qpData, interval, refactorStageHessian );
			}
			else
			{
				/* (b) qpOASES */
				interval->qpSolverSpecification = QPDUNES_STAGE_QP_SOLVER_QPOASES;
				if( qpData->options.printLevel >= 3 ) {
					qpDUNES_printf("[qpDUNES] Stage %d: Using qpOASES.", kk);
				}

				statusFlag = qpDUNES_setupQpoases( qpData, interval );
			}
		}
	}

	/* compute H.Q norm */
	/* TODO: check if really needed */
	/* TODO: this is deprecated; either fix, or remove */
	/* fixme: check which norm to actually use */
	/* TODO: only compute if a regularization option that needs the norm is chosen! */
//	for( kk=0; kk<_NI_+1; ++kk ) {
//		qpData->intervals[kk]->HQNorm = qpData->options.QPDUNES_INFTY;
//		/* TODO: make also work for non-diagonal weighting matrices! */
//		if ( qpData->intervals[kk]->H.HQ.sparsityType == QPDUNES_DIAGONAL ) {
//			for( ii=0; ii<_NX_; ++ii ) {
//				/* for diagonal matrices this corresponds to the spectral norm */
////				qpData->intervals[kk]->HQNorm = qpDUNES_fmax( qpData->intervals[kk]->H.HQ.data[ii], qpData->intervals[kk]->HQNorm );
//				/* this gives us the mininum eigen value */
////				qpData->intervals[kk]->HQNorm = qpDUNES_fmin( qpData->intervals[kk]->H.HQ.data[ii], qpData->intervals[kk]->HQNorm );
//			}
////			qpDUNES_printf( "this is norm[%d]: %.1e", kk, qpData->intervals[kk]->HQNorm );
//		}
//		else {
//			if ( qpData->intervals[kk]->H.HQ.sparsityType == QPDUNES_IDENTITY ) {
////				qpData->intervals[kk]->HQNorm = 1.;
//			}
//			else {
//				qpDUNES_printError( qpData, __FILE__, __LINE__, "norm computation for general weighting matrices not yet implemented" );
//				return QPDUNES_ERR_UNKNOWN_ERROR;
//			}
//		}
//	}

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_setupAllLocalQPs */


/* ----------------------------------------------
 *
 >>>>>>                                           */
return_t qpDUNES_setupClippingSolver(	qpData_t* const qpData,
										interval_t* const interval,
										boolean_t refactorHessian
										)
{
	return_t statusFlag;

	/* (b) prepare clipping QP solver */
	if ( refactorHessian == QPDUNES_TRUE ) {	/* only first Hessian needs to be factorized in LTI case, others can be copied; last one might still be different, due to terminal cost, even in LTI case */
		factorizeH( qpData, &(interval->cholH), &(interval->H), interval->nV );
	}

	/* (c) solve unconstrained local QP for g and initial lambda guess: */
	/*	   - get (possibly updated) lambda guess */
	if (interval->id > 0) {		/* lambdaK exists */
		qpDUNES_updateVector( &(interval->lambdaK), &(qpData->lambda.data[((interval->id)-1)*_NX_]), _NX_ );
	}
	if (interval->id < _NI_) {		/* lambdaK1 exists */
		qpDUNES_updateVector( &(interval->lambdaK1), &(qpData->lambda.data[(interval->id)*_NX_]), _NX_ );
	}

	/*     - update first order term */
	/*       reset q; qStep is added in directQpSolver_doStep, when bounds are known */
	qpDUNES_setupZeroVector( &(interval->q), interval->nV );
	clippingQpSolver_updateDualGuess( qpData, interval, &(interval->lambdaK), &(interval->lambdaK1) );
	addToVector( &(interval->qpSolverClipping.qStep), &(interval->g), interval->nV );	/* Note: qStep is rewritten in line before */
	/*     - solve */
	statusFlag = directQpSolver_solveUnconstrained( qpData, interval, &(interval->qpSolverClipping.qStep) );
	if ( statusFlag != QPDUNES_OK ) {
//		qpDUNES_printError( qpData, __FILE__, __LINE__, "QP on interval %d infeasible!", interval->id );
		qpDUNES_printError( qpData, __FILE__, __LINE__, "Stage QP backsolve failed. Check if all stage QPs are positive definite.", interval->id );
		if (qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS )	qpDUNES_logIteration( qpData, &(qpData->log.itLog[0]), qpData->options.QPDUNES_INFTY, _NI_ );
		return statusFlag;
	}

	qpDUNES_setupZeroVector( &(interval->qpSolverClipping.zUnconstrained), interval->nV );	/* reset zUnconstrained */



	/* clip directly on setup */
	statusFlag = directQpSolver_doStep(	qpData,
														interval,
														&(interval->qpSolverClipping.dz), 1,
														&(interval->qpSolverClipping.zUnconstrained),
														&(interval->z),
														&(interval->y),
														&(interval->q),
														&(interval->p)
														);

//	qpDUNES_printMatrixData( interval->z.data, 1, interval->nV, "starting with z[%d] = ", interval->id);

	return statusFlag;
}
/*<<< END OF qpDUNES_setupClippingSolver */



/* ----------------------------------------------
 *
 >>>>>>                                           */
return_t qpDUNES_setupQpoases(	qpData_t* const qpData,
								interval_t* const interval
								)
{
	return_t statusFlag;

	/* (a) prepare first order term: initial lambda guess and g */
	/*	   - get primal first order term */
	qpDUNES_copyVector( &(interval->q), &(interval->g), interval->nV );
	/*	   - get (possibly updated) lambda guess */
	if (interval->id > 0) {		/* lambdaK exists */
		qpDUNES_updateVector( &(interval->lambdaK), &(qpData->lambda.data[((interval->id)-1)*_NX_]), _NX_ );
	}
	if (interval->id < _NI_) {		/* lambdaK1 exists */
		qpDUNES_updateVector( &(interval->lambdaK1), &(qpData->lambda.data[(interval->id)*_NX_]), _NX_ );
	}
	qpOASES_updateDualGuess( qpData, interval, &(interval->lambdaK), &(interval->lambdaK1) );

	/* (b) initialize qpOASES and run initial homotopy (i.e., solve first QP) */
	statusFlag = qpOASES_setup( qpData, interval->qpSolverQpoases.qpoasesObject, interval );

//	/* try.... solve directly ... needed at all?? */
//	statusFlag = qpOASES_doStep(qpData, interval->qpSolverQpoases.qpoasesObject, interval, 1, &(interval->z), &(interval->y), &(interval->q), &(interval->p));

//	zz_matrix_t ZT;
//	int_t nFree;
//	qpOASES_getZT(qpData, interval->qpSolverQpoases.qpoasesObject, &nFree,	&ZT);
//	if (interval->id == 2) qpDUNES_printMatrixData( ZT.data, _NZ_, _NZ_, "Z' [%d] here (nfree = %d)", interval->id, nFree );

	return statusFlag;
}
/*<<< END OF qpDUNES_setupQpoases */


/* ----------------------------------------------
 *
 >>>>>>                                           */
return_t qpDUNES_updateQpoases(	qpData_t* const qpData,
								interval_t* const interval,
								boolean_t H_changed,
								boolean_t g_changed,
								boolean_t zLow_changed,
								boolean_t zUpp_changed,
								boolean_t D_changed,
								boolean_t dLow_changed,
								boolean_t dUpp_changed,
								int_t* nQpoasesIter
								)
{
	return_t statusFlag;

//	qpDUNES_printMatrixData( interval->qpSolverQpoases.qFullStep.data, interval->nV, 1, "updateQpoases[%d]: (1: qFS raw)", interval->id );

	/* (a) prepare first order term: initial lambda guess and g */
	/*	   - get primal first order term */
//	if (g_changed == QPDUNES_TRUE) {
		// we always need to get original g, since we add lambda completely!
//	}
	qpDUNES_copyVector( &(interval->q), &(interval->g), interval->nV );
//	qpDUNES_printMatrixData( interval->q.data, interval->nV, 1, "updateQpoases[%d]: (1: g only)", interval->id );
	/*	   - get (possibly updated) lambda guess */
	if (interval->id > 0) {		/* lambdaK exists */
//		qpDUNES_printMatrixData( interval->lambdaK.data, 1, _NX_, "lambdaK[%d] was", interval->id );
		qpDUNES_updateVector( &(interval->lambdaK), &(qpData->lambda.data[((interval->id)-1)*_NX_]), _NX_ );
//		qpDUNES_printMatrixData( interval->lambdaK.data, 1, _NX_, "lambdaK[%d] is", interval->id );
	}
	if (interval->id < _NI_) {		/* lambdaK1 exists */
//		qpDUNES_printMatrixData( interval->lambdaK1.data, 1, _NX_, "lambdaK1[%d] was", interval->id );
		qpDUNES_updateVector( &(interval->lambdaK1), &(qpData->lambda.data[(interval->id)*_NX_]), _NX_ );
//		qpDUNES_printMatrixData( interval->lambdaK1.data, 1, _NX_, "lambdaK1[%d] is", interval->id );
	}
	qpOASES_updateDualGuess( qpData, interval, &(interval->lambdaK), &(interval->lambdaK1) );
//	qpDUNES_printMatrixData( interval->q.data, interval->nV, 1, "updateQpoases[%d]: (1: g with lambda)", interval->id );
//	qpDUNES_printMatrixData( interval->qpSolverQpoases.qFullStep.data, interval->nV, 1, "updateQpoases[%d]: (1: qFS with lambda)", interval->id );


	/* (b) update qpOASES data and rerun initial homotopy (i.e., solve first QP) */
	#ifdef __DEBUG__
	if (qpData->options.printLevel >= 4) {
		qpDUNES_printf("\n\nI am interval %d/%d, and I will solve qpOASES for data update now:", interval->id, _NI_);
	}
	#endif
//	zz_matrix_t ZT;
//	zz_matrix_t cholProjHess;
//	int_t nFree;
//	qpOASES_getZT(qpData, interval->qpSolverQpoases.qpoasesObject, &nFree,	&ZT);
//	if (interval->id == 2) qpDUNES_printMatrixData( ZT.data, _NZ_, _NZ_, "Z' [%d] was (nfree = %d)", interval->id, nFree );
//	qpOASES_getCholZTHZ(qpData, interval->qpSolverQpoases.qpoasesObject, &cholProjHess);
//	if (interval->id == 2) qpDUNES_printMatrixData( cholProjHess.data, _NZ_, _NZ_, "EE: cholProjHess [%d] was", interval->id );
	statusFlag = qpOASES_dataUpdate(	qpData,
										interval->qpSolverQpoases.qpoasesObject,
										interval,
										H_changed,
										zLow_changed,
										zUpp_changed,
										D_changed,
										dLow_changed,
										dUpp_changed,
										nQpoasesIter
										);

	/* try.... solve directly ... needed at all?? */
	// y gets recomputed here
//	qpDUNES_printMatrixData( interval->y.data, 1, 2*interval->nD + 2*interval->nV, "y[%d] was", interval->id );
//	qpDUNES_printMatrixData( interval->z.data, 1, interval->nV, "z[%d] was", interval->id );
	/* statusFlag = qpOASES_doStep(qpData, interval->qpSolverQpoases.qpoasesObject, interval, 1, &(interval->z), &(interval->y), &(interval->q), &(interval->p)); */
//	qpOASES_getZT(qpData, interval->qpSolverQpoases.qpoasesObject, &nFree,	&ZT);
//	if (interval->id == 2) qpDUNES_printMatrixData( ZT.data, _NZ_, _NZ_, "EE: Z' [%d] is (nfree = %d)", interval->id, nFree );
//	qpOASES_getCholZTHZ(qpData, interval->qpSolverQpoases.qpoasesObject, &cholProjHess);
//	if (interval->id == 2) qpDUNES_printMatrixData( cholProjHess.data, _NZ_, _NZ_, "EE: cholProjHess [%d] is", interval->id );
//	qpDUNES_printMatrixData( interval->y.data, 1, 2*interval->nD + 2*interval->nV, "y[%d] is", interval->id );
//	qpDUNES_printMatrixData( interval->z.data, 1, interval->nV, "z[%d] is", interval->id );

	return statusFlag;
}
/*<<< END OF qpDUNES_updateQpoases */



/* ----------------------------------------------
 *
 >>>>>>                                           */
return_t qpDUNES_shiftIntervals(	qpData_t* const qpData
								)
{
	int_t kk;

	/** (1) Shift Interval pointers */
	/*  save pointer to first interval */
	interval_t* freeInterval = qpData->intervals[0];

	/*  shift all but the last interval (different size) left */
	for (kk=0; kk<_NI_-1; ++kk) {
		qpData->intervals[kk] = qpData->intervals[kk+1];
		qpData->intervals[kk]->id = kk;			/* correct stage index */
		qpData->intervals[kk]->rebuildHessianBlock = QPDUNES_TRUE;
	}
	/*  hang the free interval on the second but last position */
	qpData->intervals[_NI_-1] = freeInterval;
	qpData->intervals[_NI_-1]->id = _NI_-1;		/* correct stage index */
	qpData->intervals[_NI_-1]->rebuildHessianBlock = QPDUNES_TRUE;

	/* update definedness of lambda parts */
	qpData->intervals[0]->lambdaK.isDefined = QPDUNES_FALSE;
	qpData->intervals[_NI_-1]->lambdaK.isDefined = QPDUNES_TRUE;


	// not needed: checks for AS changes are redundant, since blocks get rebuilt anyways

//	/** (2) keep old AS statuses (through multipliers) for Hessian rebuilding checks	 */
//	/*  save status and multipliers of last interval */
//	rebuildHessianBlock_NI1 = qpData->intervals[_NI_-1]->rebuildHessianBlock;
//	yPrev_NI1 = qpData->intervals[_NI_-1]->yPrev.data;
//	/*  shift all statuses and multiplies (except the one from first interval) back again */
//	for (kk=_NI_-1; kk>0; --kk) {
//		qpData->intervals[kk]->rebuildHessianBlock = qpData->intervals[kk-1]->rebuildHessianBlock;
//		qpData->intervals[kk]->yPrev.data = qpData->intervals[kk-1]->yPrev.data;
//	}
//	/*  correct dual Hessian update for first interval */
//	qpData->intervals[0]->rebuildHessianBlock = rebuildHessianBlock_NI1;
//	qpData->intervals[0]->yPrev.data = yPrev_NI1;

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_shiftIntervals */


/* ----------------------------------------------
 *
 >>>>>>                                           */
return_t qpDUNES_shiftIntervalsLTI(	qpData_t* const qpData
									)
{
	int_t kk, ii;

	boolean_t rebuildHessianBlock_NI1;
	real_t* yPrev_NI1;


	/** (1) Shift Interval pointers */
	/*  save pointer to first interval */
	interval_t* freeInterval = qpData->intervals[0];


	/*  shift all but the last interval (different size & may contain terminal cost) left */
	for (kk=0; kk<_NI_-1; ++kk) {
		qpData->intervals[kk] = qpData->intervals[kk+1];
		qpData->intervals[kk]->id = kk;			/* correct stage index */
	}
	qpData->intervals[0]->lambdaK.isDefined = QPDUNES_FALSE;	/* update definedness of lambda parts */

	/*  hang the free interval on the second but last position */
	qpData->intervals[_NI_-1] = freeInterval;
	qpData->intervals[_NI_-1]->id = _NI_-1;							/* correct stage index */
	qpData->intervals[_NI_-1]->lambdaK.isDefined = QPDUNES_TRUE;	/* update definedness of lambda parts */


	/** (2) correct dual Hessian matrix for AS changes in last iteration of previous QP
	 *      and keep old AS statuses (through multipliers) for Hessian rebuilding checks	 */
	/*  save status and multipliers of last interval */
	rebuildHessianBlock_NI1 = qpData->intervals[_NI_-1]->rebuildHessianBlock;
	yPrev_NI1 = qpData->intervals[_NI_-1]->yPrev.data;
	/*  shift all statuses and multiplies (except the one from first interval) back again */
	for (kk=_NI_-1; kk>0; --kk) {
		qpData->intervals[kk]->rebuildHessianBlock = qpData->intervals[kk-1]->rebuildHessianBlock;
		qpData->intervals[kk]->yPrev.data = qpData->intervals[kk-1]->yPrev.data;
	}
	/*  correct dual Hessian update for first interval */
	qpData->intervals[0]->rebuildHessianBlock = rebuildHessianBlock_NI1;
	qpData->intervals[0]->yPrev.data = yPrev_NI1;


	/** (3) check for active set changes from shift ... ((in)active constraints that were moved to different intervals) */
//	/*  check first interval */
//	if ( qpData->intervals[kk]->rebuildHessianBlock != QPDUNES_TRUE )	{
//		qpDUNES_setupClippingSolver( qpData, qpData->intervals[0], QPDUNES_FALSE );
//		for (ii = 0; ii < _NV(_NI_-1); ++ii ) {
//			if ( (boolean_t) ( (qpData->intervals[0]->y.data[2 * ii] * qpData->intervals[0]->y.data[2 * ii + 1]) < 0 )  !=
//				 (boolean_t) ( (qpData->intervals[_NI_-1]->yPrev.data[2 * ii] * qpData->intervals[_NI_-1]->yPrev.data[2 * ii + 1]) < 0 ) )	/* AS change if sign is different */
//			{
//				qpData->intervals[0]->rebuildHessianBlock = QPDUNES_TRUE;
//				break;
//			}
//		}
//	}


	/*  check remaining intervals */
//	for (kk=1; kk<_NI_; ++kk) {
	for (kk=0; kk<_NI_; ++kk) {
		/* if block is scheduled for update anyways, we do not need to check for changes again */
		if ( qpData->intervals[kk]->rebuildHessianBlock == QPDUNES_TRUE )  continue;

		/* resolve QP to see if Hessian changed for shifted bounds, etc. */
		qpDUNES_setupClippingSolver( qpData, qpData->intervals[kk], QPDUNES_FALSE );

		/* check if Hessian contribution changed */
		for (ii = 0; ii < _NV(kk); ++ii ) {
			if ( (boolean_t) ( (qpData->intervals[kk]->y.data[2 * ii] * qpData->intervals[kk]->y.data[2 * ii + 1]) < 0 )  !=
//				 (boolean_t) ( (qpData->intervals[(kk-1)]->yPrev.data[2 * ii] * qpData->intervals[(kk-1)]->yPrev.data[2 * ii + 1]) < 0 ) )	/* AS change if sign is different */
				 (boolean_t) ( (qpData->intervals[kk]->yPrev.data[2 * ii] * qpData->intervals[kk]->yPrev.data[2 * ii + 1]) < 0 ) )	/* AS change if sign is different */
			{
				qpData->intervals[kk]->rebuildHessianBlock = QPDUNES_TRUE;
				break;
			}
		}
	}


	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_shiftIntervalsLTI */


/* ----------------------------------------------
 *
 >>>>>>                                           */
return_t qpDUNES_shiftLambda(	qpData_t* const qpData
							)
{
	int_t kk, ii;

	for (kk=0; kk<_NI_-1; ++kk) {
		for (ii=0; ii<_NX_; ++ii) {
			qpData->lambda.data[kk*_NX_+ii] = qpData->lambda.data[(kk+1)*_NX_+ii];
		}
	}

	/* TODO: what shall we do with the multipliers between the last three intervals? shift? if yes, how? how to initialize new multipliers?
	 * Is the (possible) terminal constraint a problem?
	 */


	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_shiftLambda */




qpOptions_t qpDUNES_setupDefaultOptions( )
{
	qpOptions_t options;

	/* iteration limits */
	options.maxIter               		= 100;
	options.maxNumLineSearchIterations 	= 19;				/* 0.3^19 = 1e-10 */
	options.maxNumLineSearchRefinementIterations 	= 40;	/* 0.62^49 = 1e-10 */
	options.maxNumQpoasesIterations 	= 1000;				/* should be chosen depending on problem size */

	/* printing */
	options.printLevel            		= 2;
	options.printIntervalHeader        	= 20;
	options.printIterationTiming		= QPDUNES_FALSE;
	options.printLineSearchTiming		= QPDUNES_FALSE;

	/* logging */
	options.logLevel            		= QPDUNES_LOG_OFF;

	/* numerical tolerances */
	options.stationarityTolerance 		= 1.e-6;
	options.equalityTolerance     		= 2.221e-16;
	options.newtonHessDiagRegTolerance  = 1.e-10;
	options.activenessTolerance			= 1e4*options.equalityTolerance;
	options.QPDUNES_ZERO             		= 1.e-50;
	options.QPDUNES_INFTY            		= 1.e12;
	options.ascentCurvatureTolerance	= 1.e-6;
	
	/* additional options */
	options.nbrInitialGradientSteps		= 0;
	options.checkForInfeasibility		= QPDUNES_FALSE;
	options.allowSuboptimalTermination	= QPDUNES_FALSE;

	/* regularization option */
	options.regType 					= QPDUNES_REG_LEVENBERG_MARQUARDT;
	options.regParam			   		= 1.e-6;	/**< the regularization parameter added on singular Hessian elements
	 	 	 	 	 	 	 	 	 	 	 	 	 	 - should be quite a bit bigger than regularization tolerance
	 	 	 	 	 	 	 	 	 	 	 	 	 	 - assumption: if regularization needed, than Hessian has a singular direction
	 	 	 	 	 	 	 	 	 	 	 	 	 	 - in this singular direction i want to do mostly a gradient step,
	 	 	 	 	 	 	 	 	 	 	 	 	 	   few Hessian information usable
	 	 	 	 	 	 	 	 	 	 	 	 	  */

	options.nwtnHssnFacAlg				= QPDUNES_NH_FAC_BAND_REVERSE;


	/* line search options */
	options.lsType							= QPDUNES_LS_ACCELERATED_GRADIENT_BISECTION_LS;
	options.lineSearchReductionFactor		= 0.1;	/**< needs to be between 0 and 1 */
	options.lineSearchIncreaseFactor		= 1.5;	/**< needs to be greater than 1 */
	options.lineSearchMinAbsProgress    	= options.equalityTolerance;
	options.lineSearchMinRelProgress    	= 1.e-14;
	options.lineSearchStationarityTolerance = 1.e-3;
	options.lineSearchMaxStepSize   		= 1.;
	options.lineSearchNbrGridPoints   		= 5;

	/* qpOASES options */
	options.qpOASES_terminationTolerance	= 1.e-12;	/*< stationarity tolerance for qpOASES, see qpOASES::Options -> terminationTolerance */

	return options;
}
/*<<< END OF qpDUNES_setupOptions */



return_t qpDUNES_setupLog(	qpData_t* const qpData
							)
{
	log_t* itLog = &(qpData->log);

	itLog->nI = _NI_;
	itLog->nX = _NX_;
	itLog->nU = _NU_;
	itLog->nZ = _NZ_;
	itLog->nDttl = _NDTTL_;

	itLog->qpOptions = qpData->options;	/* WARNING: this relies on qpOptions_t to only have primitive non-pointer data */

	return QPDUNES_OK;
}


/*
 *	end of file
 */

