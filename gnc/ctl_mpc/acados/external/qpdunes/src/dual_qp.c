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
 * main solve function
 * 
 >>>>>>                                           */
return_t qpDUNES_solve(qpData_t* const qpData) {
	uint_t kk, ii;

	int_t* itCntr = &(qpData->log.numIter);
	*itCntr = 0;
	return_t statusFlag; /* generic status flag */
	int_t hessRefactorIdx;

	real_t objValIncumbent = qpDUNES_computeObjectiveValue(qpData);

	real_t tItStart, tItEnd, tQpStart, tQpEnd, tNwtnSetupStart, tNwtnSetupEnd,
			tNwtnFactorStart, tNwtnFactorEnd, tNwtnSolveStart, tNwtnSolveEnd,
			tLineSearchStart, tLineSearchEnd;

	itLog_t* itLogPtr = &(qpData->log.itLog[0]);
	itLogPtr->itNbr = 0;

	real_t* y_swap = 0;


	/** (3a) log and display */
	if (qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS)
		qpDUNES_logIteration(qpData, itLogPtr, objValIncumbent, hessRefactorIdx);

	/** (3b) measure timings */
	#ifdef __MEASURE_TIMINGS__
	if (qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS) {
		itLogPtr->tQP = 0;
		itLogPtr->tIt = 0;
	}
	#endif


	/** LOOP OF NONSMOOTH NEWTON ITERATIONS */
	/*  ----------------------------------- */
	for ((*itCntr) = 1; (*itCntr) <= qpData->options.maxIter; ++(*itCntr)) {

		//qpDUNES_printMatrixData( qpData->lambda.data, _NI_*_NX_, 1, "lambda is currently:" );

		#ifdef __MEASURE_TIMINGS__
		tItStart = getTime();
		#endif


		/** (0) prepare logging */
		if (qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS) {
			itLogPtr = &(qpData->log.itLog[(*itCntr)]);
		}
		itLogPtr->itNbr = *itCntr;


		/** (1) get a step direction:
		 *      switch between preconditioned gradient and Newton steps */
		itLogPtr->isHessianRegularized = QPDUNES_FALSE;
		if ((*itCntr > 1) && (*itCntr - 1 <= qpData->options.nbrInitialGradientSteps)) { /* always do one Newton step first */

			/** preconditioned gradient step with unconstrained hessian */

			tNwtnSetupStart = getTime();
			qpDUNES_computeNewtonGradient(qpData, &(qpData->gradient), &(qpData->xVecTmp));
			tNwtnSetupEnd = getTime();

			tNwtnSolveStart = getTime();
			switch (qpData->options.nwtnHssnFacAlg) {
			case QPDUNES_NH_FAC_BAND_FORWARD:
				statusFlag = qpDUNES_solveNewtonEquation(qpData, &(qpData->deltaLambda), &(qpData->cholUnconstrainedHessian), &(qpData->gradient));
				break;

			case QPDUNES_NH_FAC_BAND_REVERSE:
				statusFlag = qpDUNES_solveNewtonEquationBottomUp(qpData, &(qpData->deltaLambda), &(qpData->cholUnconstrainedHessian), &(qpData->gradient));
				break;

			default:
				qpDUNES_printError(qpData, __FILE__, __LINE__, "Unknown Newton Hessian factorization algorithm. Cannot do backsolve.");
				return QPDUNES_ERR_INVALID_ARGUMENT;
			}
			tNwtnSolveEnd = getTime();
			if (statusFlag != QPDUNES_OK) {
				qpDUNES_printError(qpData, __FILE__, __LINE__,	"Could not compute Newton step direction.");
				if (qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS)	 qpDUNES_logIteration(qpData, itLogPtr, objValIncumbent, hessRefactorIdx);
				return statusFlag;
			}

			//if (*itCntr - 1 == qpData->options.nbrInitialGradientSteps){
				// make sure that the next Newton Hessian is recomputed entirely
				for (kk = 0; kk<_NI_; ++kk)
					qpData->intervals[kk]->rebuildHessianBlock = QPDUNES_TRUE;
			//}
		}
		else{

			/** (1Ba) set up Newton system */
			tNwtnSetupStart = getTime();
			statusFlag = qpDUNES_setupNewtonSystem( qpData, &hessRefactorIdx );
			switch (statusFlag) {
				case QPDUNES_OK:
					break;
				case QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND: /* zero gradient norm detected */
					qpDUNES_printSuccess(qpData, "Optimal solution found: gradient norm %.1e",	vectorNorm(&(qpData->gradient), _NI_ * _NX_));
					if (qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS)  qpDUNES_logIteration(qpData, itLogPtr, objValIncumbent, hessRefactorIdx);
					/* save the final active set (the one where the solution lies).
					 *   Even if the hessian is not yet updated according to the last
					 *   active set changesX, the corresponding blocks are already flagged
					 *   and will be updated in the first iteration of the next call.
					 *   Then ieqStatus needs to be consistent */
					/* swap multipliers, such current multipliers are available as old multipliers in next QP solution */
					for (kk = 0; kk < _NI_ + 1; ++kk) {
						y_swap = qpData->intervals[kk]->yPrev.data;
						qpData->intervals[kk]->yPrev.data = qpData->intervals[kk]->y.data;
						qpData->intervals[kk]->y.data = y_swap;
						#ifdef __DEBUG__
//						if ( (kk >= _NI_-3) || (kk == 0) )	qpDUNES_printMatrixData( qpData->intervals[kk]->z.data, 1, qpData->intervals[kk]->nV, "z@end[%3d]:", kk);
						if (qpData->options.printLevel >= 4)	{
							for (ii = 0; ii < 2* qpData->intervals[kk]->nV; ++ii)	{
								if (qpData->intervals[kk]->yPrev.data[ii] > 0)	{
									qpDUNES_printf("[%d, %d] active", kk, ii);
								}
							}
						}
						#endif
					}
					/* ...and leave */
					return QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND;
				default:
					qpDUNES_printError(qpData, __FILE__, __LINE__,	"Setup of Newton Equation failed.");
					if (qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS)  qpDUNES_logIteration(qpData, itLogPtr, objValIncumbent, hessRefactorIdx);
					return statusFlag;
			}
			tNwtnSetupEnd = getTime();

			/** (1Bb) factorize Newton system */
			tNwtnFactorStart = getTime();
			statusFlag = qpDUNES_factorNewtonSystem(qpData, &(qpData->cholHessian), &(qpData->hessian), &(itLogPtr->isHessianRegularized), hessRefactorIdx);		// TODO! can we get a problem with on-the-fly regularization in partial refactorization? might only be partially reg.
			switch (statusFlag) {
				case QPDUNES_OK:
					break;
				default:
					qpDUNES_printError(qpData, __FILE__, __LINE__,	"Factorization of Newton Equation failed.");
					if (qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS)	 qpDUNES_logIteration(qpData, itLogPtr, objValIncumbent, hessRefactorIdx);
					return statusFlag;
			}
			tNwtnFactorEnd = getTime();

			/** (1Bc) compute step direction */
			tNwtnSolveStart = getTime();
			switch (qpData->options.nwtnHssnFacAlg) {
			case QPDUNES_NH_FAC_BAND_FORWARD:
				statusFlag = qpDUNES_solveNewtonEquation(qpData, &(qpData->deltaLambda), &(qpData->cholHessian), &(qpData->gradient));
				break;

			case QPDUNES_NH_FAC_BAND_REVERSE:
				statusFlag = qpDUNES_solveNewtonEquationBottomUp(qpData, &(qpData->deltaLambda), &(qpData->cholHessian), &(qpData->gradient));
				break;

			default:
				qpDUNES_printError(qpData, __FILE__, __LINE__, "Unknown Newton Hessian factorization algorithm. Cannot do backsolve.");
				return QPDUNES_ERR_INVALID_ARGUMENT;
			}
			tNwtnSolveEnd = getTime();
			if (statusFlag != QPDUNES_OK) {
				qpDUNES_printError(qpData, __FILE__, __LINE__,	"Could not compute Newton step direction.");
				if (qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS)	 qpDUNES_logIteration(qpData, itLogPtr, objValIncumbent, hessRefactorIdx);
				return statusFlag;
			}
		}


		/** (2) do QP solution for full step */
		#ifdef __MEASURE_TIMINGS__
		tQpStart = getTime();
		#endif
		statusFlag = qpDUNES_solveAllLocalQPs( qpData, &(qpData->deltaLambda), itLogPtr->numQpoasesIter );
		#ifdef __MEASURE_TIMINGS__
		tQpEnd = getTime();
		#endif
		switch (statusFlag) {
			case QPDUNES_OK:
				break;
			default:
				qpDUNES_printError(qpData, __FILE__, __LINE__,	"Problem infeasible (infeasible subproblem detected).");
				if (qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS)	 qpDUNES_logIteration(qpData, itLogPtr, objValIncumbent, hessRefactorIdx);
				return statusFlag;
		}
		/* clipping solver: now unsaturated dz is available locally */


		/** (3) do infeasibility check */
		if ( qpData->options.checkForInfeasibility == QPDUNES_TRUE )
		{
			if ( ( itLogPtr->isHessianRegularized == QPDUNES_TRUE ) ||
				 ( hessRefactorIdx < 0 ) )
			{
				statusFlag = qpDUNES_infeasibilityCheck( qpData );
				if ( statusFlag == QPDUNES_ERR_STAGE_COUPLING_INFEASIBLE ) {
					qpDUNES_printError( qpData, __FILE__, __LINE__, "QP determined infeasible in iteration %d.", *itCntr );
					if (qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS)  qpDUNES_logIteration(qpData, itLogPtr, objValIncumbent, hessRefactorIdx);
					return statusFlag;
				}
			}
		}



		/** (4) determine step length: do line search along the way of the full step
		 * 		and do the step */
		#ifdef __MEASURE_TIMINGS__
		tLineSearchStart = getTime();
		#endif
		statusFlag = qpDUNES_determineStepLength(	qpData,
													&(qpData->lambda),
													&(qpData->deltaLambda),
													&(itLogPtr->numLineSearchIter),
													&(qpData->alpha),
													&objValIncumbent,
													itLogPtr->isHessianRegularized
													);
		#ifdef __MEASURE_TIMINGS__
		tLineSearchEnd = getTime();
		#endif
		switch (statusFlag) {
			case QPDUNES_OK:
			case QPDUNES_ERR_NUMBER_OF_MAX_LINESEARCH_ITERATIONS_REACHED:
			case QPDUNES_ERR_EXCEEDED_MAX_LINESEARCH_STEPSIZE:
				break;
			case QPDUNES_ERR_DECEEDED_MIN_LINESEARCH_STEPSIZE: /* deltaLambda is no ascent direction */
				qpDUNES_printError(qpData, __FILE__, __LINE__, "Search direction is not an ascent direction. QP could not be solved.");
				if (qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS)  qpDUNES_logIteration(qpData, itLogPtr, objValIncumbent, hessRefactorIdx);
				return QPDUNES_ERR_NEWTON_SYSTEM_NO_ASCENT_DIRECTION;
			default:
				qpDUNES_printError(qpData, __FILE__, __LINE__, "Could not determine step length.");
				if (qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS)  qpDUNES_logIteration(qpData, itLogPtr, objValIncumbent, hessRefactorIdx);
				return statusFlag;
		}

		/** ----- TEMPORARY: LINE SEARCH IN REGULARIZED DIRECTIONS */
//		real_t alphaReg;
//		uint_t numRegLSIter = 0;
//		if (qpData->log.itLog[(*itCntr)].isHessianRegularized == QPDUNES_TRUE) {
//		if (QPDUNES_FALSE == QPDUNES_TRUE) {
//			qpDUNES_printf("**** LS in regularized directions ****");
//			qpData->options.lineSearchStationarityTolerance = 1.e-6;
//			/** () do QP solution for full step */
//			qpDUNES_solveAllLocalQPs(qpData, &(qpData->log.itLog[0].regDirections));
//			/* now unsaturated dz is available locally */
//
//			/** () determine step length: do line search along the way of the full step */
//			statusFlag = qpDUNES_determineStepLength(qpData, &(qpData->lambda),
//					&(qpData->log.itLog[0].regDirections), &(numRegLSIter),
//					&(alphaReg), &objValIncumbent,
//					qpData->log.itLog[(*itCntr)].isHessianRegularized);
//			qpDUNES_printf(
//					"** LS in regularized directions: %d iterations, alphaReg = %.2e",
//					numRegLSIter, alphaReg);
//			qpData->options.lineSearchStationarityTolerance = 1.e-3;
//		}
		/** ----- END OF TEMPORARY */

		/** (5) regular log and display iteration */
		/* get active set of local constraints */
		/* - save old active set */
		qpDUNES_getActSetChanges( 	qpData,
									&(itLogPtr->nActConstr),
									&(itLogPtr->nChgdConstr)
									);
		qpDUNES_logIteration(qpData, itLogPtr, objValIncumbent, hessRefactorIdx);
		/* display */
		if ((*itCntr) % qpData->options.printIntervalHeader == 1) {
			qpDUNES_printIterationHeader(qpData);
		}
		qpDUNES_printIteration(qpData, itLogPtr);



		/** (7) display timings */
		#ifdef __MEASURE_TIMINGS__
		tItEnd = getTime();
		if (qpData->options.logLevel >= QPDUNES_LOG_ITERATIONS) {
			qpData->log.itLog[(*itCntr)].tIt = tItEnd - tItStart;
			qpData->log.itLog[(*itCntr)].tNwtnSetup = tNwtnSetupEnd
					- tNwtnSetupStart;
			qpData->log.itLog[(*itCntr)].tNwtnSolve = tNwtnSolveEnd
					- tNwtnSolveStart;
			qpData->log.itLog[(*itCntr)].tQP = tQpEnd - tQpStart;
			qpData->log.itLog[(*itCntr)].tLineSearch = tLineSearchEnd
					- tLineSearchStart;
		}
		if ((qpData->options.printIterationTiming == QPDUNES_TRUE)
				&& (qpData->options.printLevel >= 2)) {
			qpDUNES_printf("\nTimings Iteration %d:", (*itCntr));
			qpDUNES_printf("Setup of Newton system:         %7.3f ms (%5.2f%%)",
					1e3 * (tNwtnSetupEnd - tNwtnSetupStart) / 1,
					(tNwtnSetupEnd - tNwtnSetupStart) / (tItEnd - tItStart)
							* 100);
			qpDUNES_printf("Factorization of Newton system: %7.3f ms (%5.2f%%)",
					1e3 * (tNwtnFactorEnd - tNwtnFactorStart) / 1,
					(tNwtnFactorEnd - tNwtnFactorStart) / (tItEnd - tItStart)
							* 100);
			qpDUNES_printf("Backsolve of newton system:     %7.3f ms (%5.2f%%)",
					1e3 * (tNwtnSolveEnd - tNwtnSolveStart) / 1,
					(tNwtnSolveEnd - tNwtnSolveStart) / (tItEnd - tItStart)
							* 100);
			qpDUNES_printf("QP solution:                    %7.3f ms (%5.2f%%)",
					1e3 * (tQpEnd - tQpStart) / 1,
					(tQpEnd - tQpStart) / (tItEnd - tItStart) * 100);
			qpDUNES_printf("Line search:                    %7.3f ms (%5.2f%%)",
					1e3 * (tLineSearchEnd - tLineSearchStart) / 1,
					(tLineSearchEnd - tLineSearchStart) / (tItEnd - tItStart)
							* 100);
			qpDUNES_printf("                               -----------");
			qpDUNES_printf("Full iteration:                 %7.3f ms\n",
					1e3 * (tItEnd - tItStart) / 1);
			qpDUNES_printf("Begin:  %.3f ms\n",
					1e3 * (tNwtnSetupStart - tItStart) / 1);
			qpDUNES_printf("End:  %.3f ms\n", 1e3 * (tItEnd - tLineSearchEnd) / 1);
		}
		#endif
	}	/* end of main qpDUNES loop */


	/* get number of performed iterations right (itCntr is going one up before realizing it's too big) */
	qpData->log.numIter = qpData->options.maxIter;

	if ( qpData->options.allowSuboptimalTermination == QPDUNES_TRUE )	{
		qpDUNES_computeNewtonGradient(qpData, &(qpData->gradient), &(qpData->xVecTmp) );
		qpDUNES_printSuccess(qpData, "Early termination due to iteration limit.\n          Dual suboptimal with remaining ascent slope %.1e",	vectorNorm(&(qpData->gradient), _NI_ * _NX_));
		/* save the final active set (the one where the solution lies).
		 *   Even if the hessian is not yet updated according to the last
		 *   active set changes, the corresponding blocks are already flagged
		 *   and will be updated in the first iteration of the next call.
		 *   Then ieqStatus needs to be consistent */
		/* swap multipliers, such current multipliers are available as old multipliers in next QP solution */
		for (kk = 0; kk < _NI_ + 1; ++kk) {
			y_swap = qpData->intervals[kk]->yPrev.data;
			qpData->intervals[kk]->yPrev.data = qpData->intervals[kk]->y.data;
			qpData->intervals[kk]->y.data = y_swap;
			#ifdef __DEBUG__
//						if ( (kk >= _NI_-3) || (kk == 0) )	qpDUNES_printMatrixData( qpData->intervals[kk]->z.data, 1, qpData->intervals[kk]->nV, "z@end[%3d]:", kk);
			if (qpData->options.printLevel >= 4)	{
				for (ii = 0; ii < 2* qpData->intervals[kk]->nV; ++ii)	{
					if (qpData->intervals[kk]->yPrev.data[ii] > 0)	{
						qpDUNES_printf("[%d, %d] active", kk, ii);
					}
				}
			}
			#endif
		}
		/* ...and leave */
		return QPDUNES_SUCC_SUBOPTIMAL_TERMINATION;
	}
	else	{
		qpDUNES_printError(qpData, __FILE__, __LINE__, "Exceeded iteration limit. QP could not be solved." );
		return QPDUNES_ERR_ITERATION_LIMIT_REACHED;
	}
}
/*<<< END OF qpDUNES_solve */


/* ----------------------------------------------
 * log all data of this iteration
 *
 >>>>>>                                           */
void qpDUNES_logIteration(	qpData_t* qpData,
							itLog_t* itLogPtr,
							real_t objValIncumbent,
							int_t hessRefactorIdx
							)
{
	int_t kk, ii;

	itLogPtr->gradNorm = vectorNorm(&(qpData->gradient), _NI_ * _NX_);
	itLogPtr->stepNorm = vectorNorm(&(qpData->deltaLambda), _NI_ * _NX_);
	itLogPtr->stepSize = qpData->alpha;
	itLogPtr->lambdaNorm = vectorNorm(&(qpData->lambda), _NI_ * _NX_);
	itLogPtr->objVal = objValIncumbent;
	itLogPtr->hessRefactorIdx = hessRefactorIdx;

	/* full logging */
	if (qpData->options.logLevel == QPDUNES_LOG_ALL_DATA) {
		/* - dual variables */
		qpDUNES_copyVector(&(itLogPtr->lambda), &(qpData->lambda), _NI_ * _NX_);
		qpDUNES_copyVector(&(itLogPtr->deltaLambda), &(qpData->deltaLambda),
				_NI_ * _NX_);
		/* - Newton system */
		for (ii = 0; ii < _NI_ * _NX_ * 2 * _NX_; ++ii) {
			itLogPtr->hessian.data[ii] = qpData->hessian.data[ii];
			itLogPtr->cholHessian.data[ii] = qpData->cholHessian.data[ii];
		}
		for (ii = 0; ii < _NI_ * _NX_; ++ii) {
			itLogPtr->gradient.data[ii] = qpData->gradient.data[ii];
		}
		/* - local primal and variables */
		for (kk = 0; kk < _NI_ + 1; ++kk) {
			interval_t* interval = qpData->intervals[kk];
			/* dz */
			for (ii = 0; ii < interval->nV; ++ii) {
				itLogPtr->dz.data[kk * _NZ_ + ii] =
						interval->qpSolverClipping.dz.data[ii];
			}
			/* zUnconstrained */
			for (ii = 0; ii < interval->nV; ++ii) {
				itLogPtr->zUnconstrained.data[kk * _NZ_ + ii] =
						interval->qpSolverClipping.zUnconstrained.data[ii];
			}
			/* z */
			for (ii = 0; ii < interval->nV; ++ii) {
				itLogPtr->z.data[kk * _NZ_ + ii] = interval->z.data[ii];
			}
			/*TODO: fix logging of multipliers! */
			/* y */
//			assert( interval->nD == interval->nV );		/* otherwise the copying below doesn't work */
//			for( ii=0; ii<interval->nD; ++ii ) {
//				itLogPtr->y.data[kk*2*_NZ_+2*ii] = interval->y.data[2*ii];
//				itLogPtr->y.data[kk*2*_NZ_+2*ii+1] = interval->y.data[2*ii+1];
//			}
		}
#if defined(__ANALYZE_FACTORIZATION__)
		if (itLogPtr->itNbr > 0) { /* do not log in first iteration, as Newton Hessian does not exist */
			/* do a backsolve with unit vectors to obtain inverse Newton Hessian for analysis */
			xn_vector_t* unitVec = &(qpData->xnVecTmp);
			xn_vector_t* resVec = &(qpData->xnVecTmp2);
			for( ii=0; ii<_NI_*_NX_; ++ii ) {
				unitVec->data[ii] = 0.;
			}
			for( ii=0; ii<_NI_*_NX_; ++ii ) {
				unitVec->data[ii] = 1.;
				qpDUNES_solveNewtonEquation( qpData, resVec, &(qpData->cholHessian), unitVec );
				for ( kk=0; kk<_NI_*_NX_; ++kk) {
					itLogPtr->invHessian.data[kk*_NI_*_NX_+ii] = resVec->data[kk];
				}
				unitVec->data[ii] = 0.;
			}
		}
#endif
	}

	return;
}
/*<<< END OF qpDUNES_logIteration */


/* ----------------------------------------------
 * update all qSteps and pSteps (linear and constant objective function contribution) of the local QPs
 *
 >>>>>>                                           */
return_t qpDUNES_updateAllLocalQPs(	qpData_t* const qpData,
									const xn_vector_t* const lambda
									)
{
	int_t kk;
	interval_t* interval;

	/* first interval: */
	interval = qpData->intervals[0];
	qpDUNES_updateVector( &(interval->lambdaK1), &(lambda->data[0]), _NX_ );
	/* intermediate intervals: */
	for (kk = 1; kk < _NI_; ++kk) {
		interval = qpData->intervals[kk];
		qpDUNES_updateVector( &(interval->lambdaK), &(lambda->data[(kk - 1) * _NX_]), _NX_ );
		qpDUNES_updateVector( &(interval->lambdaK1), &(lambda->data[kk * _NX_]), _NX_ );
	}
	/* last interval: */
	interval = qpData->intervals[_NI_];
	qpDUNES_updateVector( &(interval->lambdaK), &(lambda->data[(_NI_ - 1) * _NX_]), _NX_ );

	for (kk = 0; kk < _NI_ + 1; ++kk) {
		interval = qpData->intervals[kk];
		switch (interval->qpSolverSpecification) {
			case QPDUNES_STAGE_QP_SOLVER_CLIPPING:
				clippingQpSolver_updateDualGuess( qpData, interval, &(interval->lambdaK), &(interval->lambdaK1) );
				break;
			case QPDUNES_STAGE_QP_SOLVER_QPOASES:
				qpOASES_updateDualGuess( qpData, interval, &(interval->lambdaK), &(interval->lambdaK1) );
				break;
			default:
				qpDUNES_printError( qpData, __FILE__, __LINE__, "Stage QP solver undefined! Bailing out..." );
				return QPDUNES_ERR_INVALID_ARGUMENT;
		}
	}

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_updateAllLocalQPs */


/* ----------------------------------------------
 * solve local QPs for a multiplier guess lambda
 * 
 >>>>>>                                           */
return_t qpDUNES_solveAllLocalQPs(	qpData_t* const qpData,
									const xn_vector_t* const lambda,
									int_t* const numQpoasesIter
									)
{
	int_t kk;
	int_t errCntr = 0;
	return_t statusFlag;

	/* 1) update local QP data */
	qpDUNES_updateAllLocalQPs(qpData, lambda);

	/* 2) solve local QPs */
	// Note: const variables are predetermined shared (at least on apple)
	#pragma omp parallel for private(kk) shared(statusFlag) schedule(static) //shared(qpData)    // todo: manage threads outside!
		for (kk = 0; kk < _NI_ + 1; ++kk) {
			statusFlag = qpDUNES_solveLocalQP( qpData, qpData->intervals[kk], (int_t*)offsetIntArray( numQpoasesIter, kk) );
			if (statusFlag != QPDUNES_OK) { /* note that QPDUNES_OK == 0 */
				qpDUNES_printError(qpData, __FILE__, __LINE__,	"QP on interval %d infeasible!", kk);
				errCntr++;
			}
			#if defined (__QPDUNES_PARALLEL__)
			//			qpDUNES_printf("Computed QP %d by thread %d/%d.", kk, omp_get_thread_num(), omp_get_num_threads() );
			#endif
		}
//	} /* END of omp parallel */
	if (errCntr > 0) {
		return QPDUNES_ERR_STAGE_QP_INFEASIBLE;
	}

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_solveAllLocalQPs */


/* ----------------------------------------------
 * solve local QP
 *
 >>>>>>                                           */
return_t qpDUNES_solveLocalQP(	qpData_t* const qpData,
								interval_t* const interval,
								int_t* const numQpoasesIter
								)
{
	return_t statusFlag;

	/* set up local QP in standard form for solver and solve it */
	switch (interval->qpSolverSpecification) {
		case QPDUNES_STAGE_QP_SOLVER_CLIPPING:
			/* solve QPs in first-order term updates only, to mimic homotopy */
			statusFlag = directQpSolver_solveUnconstrained(qpData, interval, &(interval->qpSolverClipping.qStep));
			if (statusFlag != QPDUNES_OK) {
				qpDUNES_printError(qpData, __FILE__, __LINE__, "Direct QP solver infeasible.");
				return statusFlag;
			}
			break;

		case QPDUNES_STAGE_QP_SOLVER_QPOASES:
			*numQpoasesIter = 0;
			/* qpOASES has homotopy internally, so we work with full first-order terms */
			statusFlag = qpOASES_hotstart(qpData, interval->qpSolverQpoases.qpoasesObject, interval, &(interval->qpSolverQpoases.qFullStep), numQpoasesIter, QPDUNES_TRUE );
			if (statusFlag != QPDUNES_OK) {
				qpDUNES_printError(qpData, __FILE__, __LINE__,
						"Direct QP solver infeasible.");
				return statusFlag;
			}
			break;

		default:
			qpDUNES_printError(qpData, __FILE__, __LINE__, "Stage QP solver undefined! Bailing out...");
			return QPDUNES_ERR_INVALID_ARGUMENT;
	}

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_solveLocalQP */


/* ----------------------------------------------
 * ...
 * 
 >>>>>>                                           */
return_t qpDUNES_setupNewtonSystem(	qpData_t* const qpData,
									int_t* const lastHessianDataChangeIdx
									)
{
	int_t ii, jj, kk;

	x_vector_t* xVecTmp = &(qpData->xVecTmp);
	xx_matrix_t* xxMatTmp = &(qpData->xxMatTmp);
	xx_matrix_t* xxMatTmp2 = &(qpData->xxMatTmp2);
	ux_matrix_t* uxMatTmp = &(qpData->uxMatTmp);
	zx_matrix_t* zxMatTmp = &(qpData->zxMatTmp);
	zx_matrix_t* zxMatTmp2 = &(qpData->xzMatTmp);

	// get Matrices for reduced Hessian and for projection matrix
	zz_matrix_t ZT;
	ZT.data = 0;
	ZT.sparsityType = QPDUNES_DENSE;
	zz_matrix_t cholProjHess;
	cholProjHess.data = 0;
	cholProjHess.sparsityType = QPDUNES_DENSE;
	int_t nFree; /* number of active constraints of stage QP */

	interval_t** intervals = qpData->intervals;

	xn2x_matrix_t* hessian = &(qpData->hessian);

	/** calculate gradient and check gradient norm for convergence */
	qpDUNES_computeNewtonGradient(qpData, &(qpData->gradient), xVecTmp);
	if ( (vectorNorm(&(qpData->gradient), _NX_ * _NI_) < qpData->options.stationarityTolerance) ) {
		return QPDUNES_SUCC_OPTIMAL_SOLUTION_FOUND;
	}

//	qpDUNES_printMatrixData( qpData->gradient.data, _NI_*_NX_, 1, "grad" );


	/** calculate hessian */
	*lastHessianDataChangeIdx = -1;

	/* 1) diagonal blocks */
	/*    E_{k+1} P_{k+1}^-1 E_{k+1}' + C_{k} P_{k} C_{k}'  for projected Hessian  P = Z (Z'HZ)^-1 Z'  */
	for (kk = 0; kk < _NI_; ++kk) {
		/* check whether block needs to be recomputed */
		if ( (intervals[kk]->rebuildHessianBlock == QPDUNES_TRUE) || (intervals[kk+1]->rebuildHessianBlock == QPDUNES_TRUE)
//		if ( 1
			 ) {
			if ( kk > *lastHessianDataChangeIdx )	*lastHessianDataChangeIdx = kk;
			#ifdef __DEBUG__
			if (qpData->options.printLevel >= 4) {
				qpDUNES_printf("rebuilt diagonal block %d of %d", kk, _NI_-1);
			}
			#endif
			/* get EPE part */
			if (intervals[kk + 1]->qpSolverSpecification == QPDUNES_STAGE_QP_SOLVER_QPOASES)
			{
				for (ii=0; ii<_NX_*_NZ_; ++ii)	{
					zxMatTmp2->data[ii] = 0;	/* wipe out matrices for safety */
				}
				for (ii=0; ii<_NX_*_NX_; ++ii)	{
					xxMatTmp->data[ii] = 0;	/* wipe out matrices for safety */
				}
				qpOASES_getZT(qpData, intervals[kk + 1]->qpSolverQpoases.qpoasesObject, &nFree,	&ZT);
				qpOASES_getCholZTHZ(qpData, intervals[kk + 1]->qpSolverQpoases.qpoasesObject, &cholProjHess);
				backsolveRT_ZTET(qpData, zxMatTmp2, &cholProjHess, &ZT, xVecTmp, intervals[kk + 1]->nV, nFree);
				boolean_t addToRes = QPDUNES_FALSE;
				multiplyMatrixTMatrixDenseDense(xxMatTmp->data, zxMatTmp2->data, zxMatTmp2->data, nFree, _NX_, _NX_, addToRes);
//				if (kk == 1) qpDUNES_printMatrixData( ZT.data, _NZ_, _NZ_, "EE: Z' [%d] (nfree = %d)", kk, nFree );
//				if (kk == 1) qpDUNES_printMatrixData( cholProjHess.data, _NZ_, _NZ_, "EE: cholProjHess [%d] (nfree = %d)", kk, nFree );
//				if (kk == 1) qpDUNES_printMatrixData( zxMatTmp2->data, _NZ_, _NX_, "EE: EPE sqrt[%d]", kk );
//				if (kk == 1) qpDUNES_printMatrixData( xxMatTmp->data, _NX_, _NX_, "EE: EPE[%d] (with nFree = %d)", kk, nFree );
			}
			else { /* clipping QP solver */

				getInvQ(qpData, xxMatTmp, &(intervals[kk + 1]->cholH), intervals[kk + 1]->nV); /* getInvQ not supported with matrices other than diagonal... is this even possible? */

				/* Annihilate columns in invQ; note: this can only be applied for diagonal matrices */
				qpDUNES_makeMatrixDense(xxMatTmp, _NX_, _NX_);
				for (ii = 0; ii < _NX_; ++ii) {
					if ((intervals[kk + 1]->y.data[2 * ii] >= qpData->options.equalityTolerance) ||		// check if local constraint lb_x is active
						(intervals[kk + 1]->y.data[2 * ii + 1] >= qpData->options.equalityTolerance))	// check if local constraint ub_x is active		// WARNING: weakly active constraints are excluded here!
					/* check if a bound is active. WARNING: WEAKLY ACTIVE CONSTRAINTS ARE EXCLUDED HERE! */
//					if ( intervals[kk + 1]->y.data[2 * ii] * intervals[kk + 1]->y.data[2 * ii + 1] <= -qpData->options.activenessTolerance )
					{
						xxMatTmp->data[ii * _NX_ + ii] = 0.;
					}
				}
//				if (kk == 1) qpDUNES_printMatrixData( xxMatTmp->data, _NX_, _NX_, "clipping: EPE part[%d]", kk );
			}

			/* add CPC part */
			if (intervals[kk]->qpSolverSpecification == QPDUNES_STAGE_QP_SOLVER_QPOASES)
			{
				for (ii=0; ii<_NX_*_NZ_; ++ii)	{
					zxMatTmp2->data[ii] = 0;	/* wipe out matrices for safety */
					zxMatTmp->data[ii] = 0;	/* wipe out matrices for safety */
				}
				/* get data from qpOASES */
				qpOASES_getZT(qpData, intervals[kk]->qpSolverQpoases.qpoasesObject,	&nFree, &ZT);
				qpOASES_getCholZTHZ(qpData,	intervals[kk]->qpSolverQpoases.qpoasesObject, &cholProjHess);
				/* computer Z.T * C.T */
				zx_matrix_t* ZTCT = zxMatTmp;
				multiplyMatrixMatrixTDenseDense(ZTCT->data, ZT.data, intervals[kk]->C.data, nFree, _NZ_, _NX_);
				/* compute "squareroot" of C_{k} P_{k} C_{k}' */
				backsolveRT_ZTCT(qpData, zxMatTmp2, &cholProjHess, ZTCT, xVecTmp, intervals[kk]->nV, nFree);
				/* compute C_{k} P_{k} C_{k}' contribution */
				boolean_t addToRes = QPDUNES_TRUE;
				multiplyMatrixTMatrixDenseDense(xxMatTmp->data, zxMatTmp2->data, zxMatTmp2->data, nFree, _NX_, _NX_, addToRes);
//				if (kk == 1) qpDUNES_printMatrixData( ZT.data, _NZ_, _NZ_, "Z' [%d] (nfree = %d)", kk, nFree );
//				if (kk == 1) qpDUNES_printMatrixData( ZTCT->data, _NZ_, _NX_, "ZTCT [%d] (nfree = %d)", kk, nFree );
//				if (kk == 1) qpDUNES_printMatrixData( zxMatTmp2->data, _NZ_, _NX_, "CPC sqrt[%d]", kk );
//				if (kk == 1) qpDUNES_printMatrixData( xxMatTmp->data, _NX_, _NX_, "EPE+CPC[%d]", kk );
			}
			else { /* clipping QP solver */
				addCInvHCT(qpData, xxMatTmp, &(intervals[kk]->cholH), &(intervals[kk]->C), &(intervals[kk]->y), xxMatTmp2, uxMatTmp, zxMatTmp);
//				if (kk == 1) qpDUNES_printMatrixData( xxMatTmp->data, _NX_, _NX_, "clipping: CPC+ EPE part[%d]", kk );
			}

			/* write Hessian part */
			for (ii = 0; ii < _NX_; ++ii) {
				for (jj = 0; jj < _NX_; ++jj) {
					accHessian( kk, 0, ii, jj ) = xxMatTmp->data[ii * _NX_ + jj];
					/* clean xxMatTmp */
					xxMatTmp->data[ii * _NX_ + jj] = 0.; /* TODO: this cleaning part is probably not needed, but we need to be very careful if we decide to leave it out! */
				}
			}
		}
	}	/* END OF diagonal block for loop */

	/* 2) sub-diagonal blocks */
	for (kk = 1; kk < _NI_; ++kk) {
		if ( (intervals[kk]->rebuildHessianBlock == QPDUNES_TRUE) )
//		if ( 1 )
		{
			if ( kk > *lastHessianDataChangeIdx )	*lastHessianDataChangeIdx = kk;
			#ifdef __DEBUG__
			if (qpData->options.printLevel >= 4) {
				qpDUNES_printf("rebuilt off-diag block %d of %d", kk, _NI_-1);
			}
			#endif
			if (intervals[kk]->qpSolverSpecification == QPDUNES_STAGE_QP_SOLVER_QPOASES) {
				/* get data from qpOASES */
				qpOASES_getZT(qpData, intervals[kk]->qpSolverQpoases.qpoasesObject,	&nFree, &ZT);
				qpOASES_getCholZTHZ(qpData,	intervals[kk]->qpSolverQpoases.qpoasesObject, &cholProjHess);

				/* compute "squareroot" of C_{k} P_{k} C_{k}' */
				/* computer Z.T * C.T */
				multiplyMatrixMatrixTDenseDense(zxMatTmp->data, ZT.data, intervals[kk]->C.data, nFree, _NZ_, _NX_);
				backsolveRT_ZTCT(qpData, zxMatTmp2, &cholProjHess, zxMatTmp, xVecTmp, intervals[kk]->nV, nFree);

				/* compute "squareroot" of E_{k} P_{k} E_{k}' */
				backsolveRT_ZTET(qpData, zxMatTmp, &cholProjHess, &ZT, xVecTmp, intervals[kk]->nV, nFree);

				/* compute C_{k} P_{k} E_{k}' contribution */
				boolean_t addToRes = QPDUNES_FALSE;
				multiplyMatrixTMatrixDenseDense(xxMatTmp->data, zxMatTmp2->data, zxMatTmp->data, nFree, _NX_, _NX_, addToRes);

				/* write Hessian part */
				for (ii = 0; ii < _NX_; ++ii) {
					for (jj = 0; jj < _NX_; ++jj) {
						accHessian( kk, -1, ii, jj ) = - xxMatTmp->data[ii * _NX_ + jj];
					}
				}
			}
			else { /* clipping QP solver */
				multiplyAInvQ( qpData, &(qpData->xxMatTmp), &(intervals[kk]->C), &(intervals[kk]->cholH) );

				/* write Hessian part */
				for (ii=0; ii<_NX_; ++ii) {
					for (jj=0; jj<_NX_; ++jj) {
						/* cheap way of annihilating columns; TODO: make already in multiplication routine! */
						if ( ( intervals[kk]->y.data[2*jj] <= qpData->options.equalityTolerance ) &&		// check if local constraint lb_x is inactive
							 ( intervals[kk]->y.data[2*jj+1] <= qpData->options.equalityTolerance ) )		// check if local constraint ub_x is inactive
						/* check if a bound is active. WARNING: WEAKLY ACTIVE CONSTRAINTS ARE EXCLUDED HERE! */
//						if ( intervals[kk]->y.data[2 * jj] * intervals[kk]->y.data[2 * jj + 1] > -qpData->options.activenessTolerance )
						{
							accHessian( kk, -1, ii, jj ) = - xxMatTmp->data[ii * _NX_ + jj];
						}
						else {
							/* eliminate column if variable bound is active */
							accHessian( kk, -1, ii, jj ) = 0.;
						}
					}
				}
			}
		}
	}	/* END OF sub-diagonal block for loop */

//	qpDUNES_printMatrixData( qpData->hessian.data, _NI_*_NX_, 2*_NX_, "H = ");

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_setupNewtonSystem */

/* ----------------------------------------------
 * ...
 *
 >>>>>>                                           */
return_t qpDUNES_setupUnconstrainedNewtonSystem(	qpData_t* const qpData	)
{
	int_t ii, jj, kk;
	boolean_t isHessianRegularized;

	xx_matrix_t* xxMatTmp = &(qpData->xxMatTmp);
	xx_matrix_t* xxMatTmp2 = &(qpData->xxMatTmp2);
	ux_matrix_t* uxMatTmp = &(qpData->uxMatTmp);
	zx_matrix_t* zxMatTmp = &(qpData->zxMatTmp);

	interval_t** intervals = qpData->intervals;

	xn2x_matrix_t* hessian = &(qpData->unconstrainedHessian);

	/* 1) diagonal blocks */
	/*    E_{k+1} H_{k+1}^-1 E_{k+1}' + C_{k} P_{k} C_{k}'  for projected Hessian  P = Z (Z'HZ)^-1 Z'  */
	for (kk = 0; kk < _NI_; ++kk) {

		getInvQ(qpData, xxMatTmp, &(intervals[kk + 1]->cholH), intervals[kk + 1]->nV); /* getInvQ not supported with matrices other than diagonal... is this even possible? */

//		if (kk == 1) qpDUNES_printMatrixData( xxMatTmp->data, _NX_, _NX_, "clipping: EPE part[%d]", kk );

		/* add CPC part */
		/* call addCInvHCT with zero pointer instead of interval->y */


		addCInvHCT(qpData, xxMatTmp, &(intervals[kk]->cholH), &(intervals[kk]->C),
				0, xxMatTmp2, uxMatTmp, zxMatTmp);
//		if (kk == 1) qpDUNES_printMatrixData( xxMatTmp->data, _NX_, _NX_, "clipping: CPC+ EPE part[%d]", kk );


		/* write Hessian part */
		for (ii = 0; ii < _NX_; ++ii) {
			for (jj = 0; jj < _NX_; ++jj) {
				accHessian( kk, 0, ii, jj ) = xxMatTmp->data[ii * _NX_ + jj];
				/* clean xxMatTmp */
				xxMatTmp->data[ii * _NX_ + jj] = 0.; /* TODO: this cleaning part is probably not needed, but we need to be very careful if we decide to leave it out! */
			}
		}
	}	/* END OF diagonal block for loop */



	/* 2) sub-diagonal blocks */
	for (kk = 1; kk < _NI_; ++kk) {
		multiplyAInvQ( qpData, &(qpData->xxMatTmp), &(intervals[kk]->C), &(intervals[kk]->cholH) );

		/* write Hessian part */
		for (ii=0; ii<_NX_; ++ii) {
			for (jj=0; jj<_NX_; ++jj) {
				accHessian( kk, -1, ii, jj ) = - xxMatTmp->data[ii * _NX_ + jj];
			}
		}
	}	/* END OF sub-diagonal block for loop */

//	qpDUNES_printMatrixData( qpData->cholDefaultHessian.data, _NI_*_NX_, 2*_NX_, "H = ");


	return qpDUNES_factorNewtonSystem(qpData, &(qpData->cholUnconstrainedHessian), &(qpData->unconstrainedHessian), &isHessianRegularized, -1);
}
/*<<< END OF qpDUNES_setupNewtonSystem */



/* ----------------------------------------------
 * ...
 *
 * >>>>>>                                           */
return_t qpDUNES_computeNewtonGradient(	qpData_t* const qpData,
										xn_vector_t* gradient,
										x_vector_t* gradPiece
										)
{
	int_t kk, ii;

	interval_t** intervals = qpData->intervals;

	/* d/(d lambda_ii) for kk=0.._NI_-1 */
	for (kk = 0; kk < _NI_; ++kk) {
//		if ( (kk >= _NI_-4) || (kk == 0) )	qpDUNES_printMatrixData( intervals[kk+1]->z.data, 1, intervals[kk+1]->nV, "z[%3d]:", kk+1);
		/* ( C_kk*z_kk^opt + c_kk ) - x_(kk+1)^opt */
		multiplyCz(qpData, gradPiece, &(intervals[kk]->C), &(intervals[kk]->z));
		addToVector(gradPiece, &(intervals[kk]->c), _NX_);

		/* subtractFromVector( xVecTmp, &(intervals[kk+1]->x), _NX_ ); */
		for (ii = 0; ii < _NX_; ++ii) {
			gradPiece->data[ii] -= intervals[kk + 1]->z.data[ii];
		}

		/* write gradient part */
		for (ii = 0; ii < _NX_; ++ii) {
			gradient->data[kk * _NX_ + ii] = gradPiece->data[ii];
		}
	}
	return QPDUNES_OK;
}
/*<<< END OF computeNewtonGradient */


return_t qpDUNES_factorNewtonSystem( qpData_t* const qpData,
									 xn2x_matrix_t* cholHessian,
									 xn2x_matrix_t* hessian,
								  	 boolean_t* const isHessianRegularized,
								  	 int_t lastActSetChangeIdx
								  	 )
{
	int_t ii, jj, kk;

	return_t statusFlag;

	real_t minDiagElem = qpData->options.QPDUNES_INFTY;

	/* Try to factorize Newton Hessian, to check if positive definite */
	switch (qpData->options.nwtnHssnFacAlg) {
		case QPDUNES_NH_FAC_BAND_FORWARD:
			statusFlag = qpDUNES_factorizeNewtonHessian( qpData, cholHessian, hessian, isHessianRegularized );
			break;

		case QPDUNES_NH_FAC_BAND_REVERSE:
			statusFlag = qpDUNES_factorizeNewtonHessianBottomUp( qpData, cholHessian, hessian, lastActSetChangeIdx, isHessianRegularized );
			break;

		default:
			qpDUNES_printError(qpData, __FILE__, __LINE__, "Unknown Newton Hessian factorization algorithm.");
			return QPDUNES_ERR_INVALID_ARGUMENT;
	}

	/* check maximum diagonal element */
	if (statusFlag == QPDUNES_OK) {
		for (kk = 0; kk < _NI_; ++kk) {
			for (ii = 0; ii < _NX_; ++ii) {
				if (minDiagElem > accCholHessian(kk, 0, ii, ii) ) {
					minDiagElem = accCholHessian(kk, 0, ii, ii);
				}
			}
		}
	}
	#ifdef __DEBUG__
		if (qpData->options.printLevel >= 4) {
			qpDUNES_printf( "Minimum NH diagonal element: % .5e", minDiagElem );
		}
	#endif


	if ( ( statusFlag == QPDUNES_ERR_DIVISION_BY_ZERO ) || 					/* regularize if Cholesky failed */
		 ( minDiagElem < qpData->options.newtonHessDiagRegTolerance ) ) 	/* or if diagonal elements are too small */
	{
		switch (qpData->options.regType) {
		case QPDUNES_REG_LEVENBERG_MARQUARDT:
			for (kk = 0; kk < _NI_; ++kk) {
				for (jj = 0; jj < _NX_; ++jj) {
					accHessian( kk, 0, jj, jj )+= qpData->options.regParam;
				}
			}
			break;

			case QPDUNES_REG_NORMALIZED_LEVENBERG_MARQUARDT :
			qpDUNES_printError(qpData, __FILE__, __LINE__, "QPDUNES_REG_NORMALIZED_LEVENBERG_MARQUARDT is deprecated.");
			break;
//				/* FIXME: this is only experimental, play around with different norms, think about theory, think what would happen with non-diag Q */
//				for( kk=0; kk<_NI_; ++kk ) {
//					for( jj=0; jj<_NX_; ++jj ) {
//						switch ( qpData->intervals[kk]->H.HQ.sparsityType )
//						{
//							case QPDUNES_DENSE:
//								accHessian( kk, 0, jj, jj ) += qpData->options.regParam * 1./intervals[kk]->cholH.HQ.data[jj*_NX_+jj];
//								break;
//
//							case QPDUNES_DIAGONAL:
//								/* FIXME: rather use kk+1 instead? ...coupling between kk and kk+1 */
////								accHessian( kk, 0, jj, jj ) += qpData->options.regParam * qpData->intervals[kk]->cholH.HQ.data[jj];
//								accHessian( kk, 0, jj, jj ) += qpData->options.regParam * 1./intervals[kk]->cholH.HQ.data[jj];
//								break;
//
//							case QPDUNES_IDENTITY:
//								accHessian( kk, 0, jj, jj ) += qpData->options.regParam; /* normal Levenberg-Marquardt for identity Q matrices */
//								break;
//
//							default:
//								qpDUNES_printError( qpData, __FILE__, __LINE__, "Unknow primal Hessian sparsity type. Can not regularize." );
//								return QPDUNES_ERR_UNKNOWN_ERROR;
//						}
//					}
//				}
//				break;

			case QPDUNES_REG_SINGULAR_DIRECTIONS :
			/* already done inside factorization, we do not get here anyways */
			return QPDUNES_OK;

			case QPDUNES_REG_UNCONSTRAINED_HESSIAN :
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Regularization with unconstrained Hessian not yet implemented." );
			return QPDUNES_ERR_UNKNOWN_ERROR;

			case QPDUNES_REG_ADD_UNCONSTRAINED_HESSIAN :
			for (kk = 0; kk < _NI_; ++kk)
				for (ii = 0; ii < _NX_; ++ii)
					for (jj = 0; jj < _NX_; ++jj)
						accHessian( kk, 0, ii, jj) +=
								qpData->options.regParam * accUnconstrainedHessian(kk, 0, ii, jj);
			for (kk = 1; kk < _NI_-1; ++kk)
				for (ii = 0; ii < _NX_; ++ii)
					for (jj = 0; jj < _NX_; ++jj)
						accHessian( kk, -1, ii, jj) +=
								qpData->options.regParam * accUnconstrainedHessian(kk, 0, ii, jj);
			break;

			case QPDUNES_REG_ADD_UNCONSTRAINED_HESSIAN_DIAG :
			for (kk = 0; kk < _NI_; ++kk) {
				for (jj = 0; jj < _NX_; ++jj) {
					accHessian( kk, 0, jj, jj ) +=
							qpData->options.regParam * accUnconstrainedHessian( kk, 0, jj, jj);
				}
			}
			break;

			case QPDUNES_REG_GRADIENT_STEP :
			*isHessianRegularized = QPDUNES_TRUE;
			return QPDUNES_ERR_DIVISION_BY_ZERO;

			default:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Unknown regularization type.");
			break;
		}
		*isHessianRegularized = QPDUNES_TRUE;

		/* refactor Newton Hessian */
		switch (qpData->options.nwtnHssnFacAlg) {
			case QPDUNES_NH_FAC_BAND_FORWARD:
			statusFlag = qpDUNES_factorizeNewtonHessian( qpData, cholHessian, hessian, isHessianRegularized );
			break;

			case QPDUNES_NH_FAC_BAND_REVERSE:
			statusFlag = qpDUNES_factorizeNewtonHessianBottomUp( qpData, cholHessian, hessian, _NI_+1, isHessianRegularized );	/* refactor full hessian */
			break;

			default:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Unknown Newton Hessian factorization algorithm." );
			return QPDUNES_ERR_INVALID_ARGUMENT;
		}
		if ( statusFlag != QPDUNES_OK ) {
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Regularization of Newton Hessian failed." );
			return statusFlag;
		}
	}
	else {
		if ( statusFlag != QPDUNES_OK ) {
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Factorization of Newton Hessian failed for unknown reason." );
			return statusFlag;
		}
	}

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_factorNewtonSystem */


/* ----------------------------------------------
 * Special block tridiagonal Cholesky for special storage format of Newton matrix
 * 
 >>>>>>                                           */
return_t qpDUNES_factorizeNewtonHessian( qpData_t* const qpData,
									  xn2x_matrix_t* const cholHessian,
									  xn2x_matrix_t* const hessian,
									  boolean_t* isHessianRegularized
									  )
{
	int_t jj, ii, kk, ll;
	real_t sum;


	/* go by block columns */
	for (kk = 0; kk < _NI_; ++kk) {
		/* go by in-block columns */
		for (jj = 0; jj < _NX_; ++jj) {
			/* 1) compute diagonal element: ii == jj */
			/* take diagonal element of original */
			sum = accHessian(kk,0,jj,jj);

			/* subtract squared forepart of corresponding row: */
			/*  - this diagonal block */
			for( ll = 0; ll < jj; ++ll ) {
				sum -= accCholHessian(kk,0,jj,ll) * accCholHessian(kk,0,jj,ll);
			}
			/*  - this row's subdiagonal block */
			if( kk > 0 ) { /* for all block columns but the first one */
				for( ll = 0; ll < _NX_; ++ll ) {
					sum -= accCholHessian(kk,-1,jj,ll) * accCholHessian(kk,-1,jj,ll);
				}
			}
			

			/* 2) check for too small diagonal elements */
			if((qpData->options.regType == QPDUNES_REG_SINGULAR_DIRECTIONS) &&	/* Add regularization on too small values already in factorization */
			   (sum < qpData->options.newtonHessDiagRegTolerance) ) 		/* TODO: take branching in options.regType out of the loop if too slow */
			{
				/* FIXME: maybe log which elements were regularized */
				#ifdef __DEBUG__
				if ( qpData->options.printLevel >= 3 ) {
					qpDUNES_printf( "Regularized NH[k=%d,j=%d] = %.2e + %.2e", kk, jj, sum, qpData->options.regParam );
				}
				#endif
//				sum += qpData->options.regParam;
				sum += qpData->options.QPDUNES_INFTY * qpData->options.QPDUNES_INFTY + 1.;
				*isHessianRegularized = QPDUNES_TRUE;
				/** -- EXPERIMENTAL -- regularize all diagonal elements after the first one where singularity is detected... */
//				for( kk=0; kk<_NI_; ++kk ) {
//					for( jj=0; jj<_NX_; ++jj ) {
//						accHessian( kk, 0, jj, jj ) += qpData->options.regParam;
//					}
//				}
				/** -- END OF EXPERIMENTAL -- */
				/* TODO: only log if logging flag switched on */
//				qpData->log.itLog[0].regDirections.data[kk*_NX_+jj] = -1; //qpData->options.regParam;	/* todo: really log, not only in first iteration... */
				// TRY
//				qpData->log.itLog[0].regDirections.data[kk*_NX_+jj] = -qpData->gradient.data[kk*_NX_+jj];
//				if (kk==51) qpData->log.itLog[0].regDirections.data[kk*_NX_+jj] *= 1./100;
			}
			else {
				if ( sum < qpData->options.newtonHessDiagRegTolerance ) {	/* matrix not positive definite */
					return QPDUNES_ERR_DIVISION_BY_ZERO;
				}
			}
			accCholHessian(kk,0,jj,jj) = sqrt( sum );
			

			/* 3) write remainder of jj-th column: */		// TODO: think about how we can avoid this column-wise access and postpone until the respective element is needed */
			/*  - this diagonal block */
			for( ii=(jj+1); ii<_NX_; ++ii )
			{
				sum = accHessian(kk,0,ii,jj);
				
				/* subtract forepart of this row times forepart of jj-th row */
				/*  - diagonal block */
				for( ll = 0; ll < jj; ++ll ) {	
					sum -= accCholHessian(kk,0,ii,ll) * accCholHessian(kk,0,jj,ll);
				}
				/*  - subdiagonal block */
				if( kk > 0 ) {	/* for all block columns but the first one */
					for( ll = 0; ll < _NX_; ++ll ) {
						sum -= accCholHessian(kk,-1,ii,ll) * accCholHessian(kk,-1,jj,ll);
					}
				}

				accCholHessian(kk,0,ii,jj) = sum / accCholHessian(kk,0,jj,jj);
			}
			/*  - following row's subdiagonal block */
			if( kk < _NI_-1 ) {	/* for all block columns but the last one */
				for( ii=0; ii<_NX_; ++ii )
				{
					sum = accHessian(kk+1,-1,ii,jj);
					
					/* subtract forepart of this row times forepart of jj-th row (only this block is non-zero) */
					for( ll = 0; ll < jj; ++ll ) {	
						sum -= accCholHessian(kk+1,-1,ii,ll) * accCholHessian(kk,0,jj,ll);
					}
					
					accCholHessian(kk+1,-1,ii,jj) = sum / accCholHessian(kk,0,jj,jj);
				}
			}
		} /* next column */
		} /* next block column */

//	qpDUNES_factorizeNewtonHessianBottomUp( qpData, cholHessian, hessian, isHessianRegularized );

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_factorizeNewtonHessian */

/* ----------------------------------------------
 * Bottom-up block-tridiagonal Cholesky for special storage format of Newton matrix
 *
 >>>>>>                                           */
return_t qpDUNES_factorizeNewtonHessianBottomUp(	qpData_t* const qpData,
													xn2x_matrix_t* const cholHessian,
													xn2x_matrix_t* const hessian,
													int_t lastActSetChangeIdx, 			/**< index from where the reverse factorization is restarted */
													boolean_t* isHessianRegularized
													)
{
	int_t jj, ii, kk, ll;
	real_t sum;

//	qpDUNES_printMatrixData( cholHessian->data, _NI_*_NX_,2*_NX_, "before" );
//	qpDUNES_printMatrixData( hessian->data, _NI_*_NX_,2*_NX_, "before" );

// TODO: switch to upper triangular matrix for higher cache efficiency!!

	int_t blockIdxStart = (lastActSetChangeIdx>=0)  ?  qpDUNES_min(lastActSetChangeIdx, _NI_-1)  :  -1;
//	int_t blockIdxStart = _NI_-1;

	#ifdef __DEBUG__
	if (qpData->options.printLevel >= 3) {
		qpDUNES_printf( "[qpDUNES] Restarting reverse Cholesky factorization at block %d of %d", blockIdxStart, _NI_-1 );
	}
	#endif
//	qpDUNES_printf( "Restarting reverse Cholesky factorization at block %d of %d", blockIdxStart, _NI_-1 );

	/* go by block columns */
	for (kk = blockIdxStart; kk >= 0; --kk) {
		/* go by in-block columns */
		for (jj = _NX_ - 1; jj >= 0; --jj) {
			/* 1) compute diagonal element: ii == jj */
			/* take diagonal element of original */
			sum = accHessian(kk,0,jj,jj);

			/* subtract squared rearpart of corresponding row (transposed access, therefore rest of column): */
			/*  - this diagonal block */
			for( ll = jj+1; ll < _NX_; ++ll ) {
				/* TODO: get rid of transposed access...maybe start to save Hessian also in upper triangular format */
				sum -= accCholHessian(kk,0,ll,jj) * accCholHessian(kk,0,ll,jj); /* transposed access */
			}
			/*  - this row's subdiagonal block */
			if( kk < _NI_-1 ) { /* for all block columns but the last one */
				for( ll = 0; ll < _NX_; ++ll ) {
					sum -= accCholHessian(kk+1,-1,ll,jj) * accCholHessian(kk+1,-1,ll,jj);	/* transposed access */
				}
			}


			/* 2) check for too small diagonal elements */
			if ( (qpData->options.regType == QPDUNES_REG_SINGULAR_DIRECTIONS) &&	/* Add regularization on too small values already in factorization */
			     (sum < qpData->options.newtonHessDiagRegTolerance) ) 		/* TODO: take branching in options.regType out of the loop if too slow */
			{
				sum += qpData->options.regParam;
				*isHessianRegularized = QPDUNES_TRUE;
				#ifdef __DEBUG__
				if (sum < qpData->options.newtonHessDiagRegTolerance) {
					qpDUNES_printError( qpData, __FILE__, __LINE__, "On-the-fly regularization failed. Your problem might be too ill-conditioned.");
					return QPDUNES_ERR_DIVISION_BY_ZERO;
				}
				#endif
			}
			else {
				if ( sum < 1.e2*qpData->options.equalityTolerance ) {	/* matrix not positive definite */
					return QPDUNES_ERR_DIVISION_BY_ZERO;
				}
			}

			accCholHessian(kk,0,jj,jj) = sqrt( sum );


			/* 3) write remainder of jj-th column (upwards! via transposed access: jj-th row, leftwards): */
			/*  - this diagonal block */
			for( ii=jj-1; ii>=0; --ii )
			{
				sum = accHessian(kk,0,jj,ii);	/* transposed access */

				/* subtract rear part of this row times rear part of jj-th row */
				/*  - diagonal block */
				for( ll = jj+1; ll < _NX_; ++ll ) {
					sum -= accCholHessian(kk,0,ll,ii) * accCholHessian(kk,0,ll,jj);		/* transposed access */
				}
				/*  - subdiagonal block */
				if( kk < _NI_-1 ) {	/* for all block rows but the last one */
					for( ll = 0; ll < _NX_; ++ll ) {
						sum -= accCholHessian(kk+1,-1,ll,ii) * accCholHessian(kk+1,-1,ll,jj);	/* transposed access */
					}
				}

				/* write transposed! (otherwise it's upper triangular matrix) */
//				accCholHessian(kk,0,ii,jj) = sum / accCholHessian(kk,0,jj,jj);
				accCholHessian(kk,0,jj,ii) = sum / accCholHessian(kk,0,jj,jj);
			}
			/*  - following row's subdiagonal block */
			if( kk > 0 ) {	/* for all block rows but the first one */
				for( ii=_NX_-1; ii>=0; --ii )
				{
					sum = accHessian(kk,-1,jj,ii);	/* transposed access */

					/* subtract rear part of this row times rear part of jj-th row (only this block is non-zero) */
					for( ll = jj+1; ll < _NX_; ++ll ) {
						sum -= accCholHessian(kk,-1,ll,ii) * accCholHessian(kk,0,ll,jj);	/* transposed access */
					}

					/* write transposed! (otherwise it's upper triangular matrix) */
//					accCholHessian(kk-1,+1,ii,jj) = sum / accCholHessian(kk,0,jj,jj);
					accCholHessian(kk,-1,jj,ii) = sum / accCholHessian(kk,0,jj,jj);
				}
			}
		} /* next column */
	} /* next block column */

//	qpDUNES_printMatrixData( cholHessian->data, _NI_*_NX_,2*_NX_, "after" );

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_factorizeNewtonHessianBottomUp */



/* ----------------------------------------------
 * special backsolve for block tridiagonal Newton matrix
 * 
 >>>>>>                                           */
return_t qpDUNES_solveNewtonEquation(	qpData_t* const qpData,
									xn_vector_t* const res,
									const xn2x_matrix_t* const cholHessian, /**< lower triangular Newton Hessian factor */
									const xn_vector_t* const gradient	)
{
	int_t ii, jj, kk;

	real_t sum;

	/* solve L*x = g */
	for (kk = 0; kk < _NI_; ++kk) /* go by block rows top down */
	{
		for (ii = 0; ii < _NX_; ++ii) /* go by in-block rows top down */
		{
			sum = gradient->data[kk * _NX_ + ii];
			/* subtract all previously resolved unknowns ... */
			if (kk > 0) { /* ... of corresponding subdiagonal block (if not first block row) */
				for (jj = 0; jj < _NX_; ++jj) {
					sum -= accCholHessian(kk,-1,ii,jj)* res->data[(kk-1)*_NX_+jj];
				}
			}
			for (jj = 0; jj < ii; ++jj) { /* ... of corresponding diagonal block */
				sum -= accCholHessian(kk,0,ii,jj)* res->data[kk*_NX_+jj];
			}

			/* divide by diagonal element */
			#if defined(__USE_ASSERTS__)
			if ( fabs( accCholHessian(kk,0,ii,ii) ) < qpData->options.QPDUNES_ZERO * fabs( sum ) ) {
				qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveDenseL.\nsum = %.3e, diag = %.3e\nRank-deficient Matrix?", sum, accCholHessian(kk,0,ii,ii) );
				return QPDUNES_ERR_DIVISION_BY_ZERO;
			}
			#endif
			// temporarily to try Wright1999
			if (accCholHessian(kk,0,ii,ii)> qpData->options.QPDUNES_INFTY) {
				res->data[kk*_NX_+ii] = 0.;
			}
			else {
				res->data[kk*_NX_+ii] = sum / accCholHessian(kk,0,ii,ii);
			}
		}
	}

	/* solve L^T*res = x */
	for (kk = (_NI_ - 1); kk >= 0; --kk) /* go by block rows bottom up */
	{
		for (ii = (_NX_ - 1); ii >= 0; --ii) /* go by in-block rows top down */
		{
			sum = res->data[kk * _NX_ + ii]; /* intermediate result of first backsolve is stored in res */
			/* subtract all previously resolved unknowns ... */
			for (jj = ii + 1; jj < _NX_; ++jj) { /* ... of corresponding diagonal block */
				sum -= accCholHessian(kk,0,jj,ii)* res->data[kk*_NX_+jj];
			}
			if (kk < _NI_ - 1) { /* ... of corresponding superdiagonal block, access via following row's subdiagonal block (if not first block row from bottom) */
				for (jj = 0; jj < _NX_; ++jj) {
					sum -= accCholHessian(kk+1,-1,jj,ii)* res->data[(kk+1)*_NX_+jj];
				}
			}

			/* divide by diagonal element */
			#if defined(__USE_ASSERTS__)
				if ( fabs( accCholHessian(kk,0,ii,ii) ) < qpData->options.QPDUNES_ZERO * fabs( sum ) ) {
					qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveDenseL.\nsum = %.3e, diag = %.3e\nRank-deficient Matrix?", sum, accCholHessian(kk,0,ii,ii) );
					return QPDUNES_ERR_DIVISION_BY_ZERO;
				}
			#endif
			res->data[kk * _NX_ + ii] = sum / accCholHessian(kk,0,ii,ii);
		}
	}

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_solveNewtonEquation */


/* ----------------------------------------------
 * special backsolve for backwards factorized block tridiagonal Newton matrix
 *
 >>>>>>                                           */
return_t qpDUNES_solveNewtonEquationBottomUp(	qpData_t* const qpData,
											xn_vector_t* const res,
											const xn2x_matrix_t* const cholHessian, /**< lower triangular Newton Hessian factor */
											const xn_vector_t* const gradient	)
{
	// TODO: switch to upper triangular matrix for higher cache efficiency!!
	int_t ii, jj, kk;

	real_t sum;

//	qpDUNES_printMatrixDataToFile( res->data, _NI_*_NX_, 1, "dLambda_normal", "" );

	/* solve L^T*x = g */
	for (kk = (_NI_ - 1); kk >= 0; --kk) /* go by block rows bottom up */
	{
		for (ii = (_NX_ - 1); ii >= 0; --ii) /* go by in-block rows top down */
		{
			sum = gradient->data[kk * _NX_ + ii];
			/* subtract all previously resolved unknowns ... */
			for (jj = ii + 1; jj < _NX_; ++jj) { /* ... of corresponding diagonal block */
				sum -= accCholHessian(kk,0,jj,ii)* res->data[kk*_NX_+jj]; /* transposed access */
			}
			if (kk < _NI_ - 1) { /* ... of corresponding superdiagonal block, access via following row's subdiagonal block (if not first block row from bottom) */
				for (jj = 0; jj < _NX_; ++jj) {
					sum -= accCholHessian(kk+1,-1,jj,ii)* res->data[(kk+1)*_NX_+jj];// TODO: maybe change access pattern, start with superdiag block, so cholH access is more continuous
				}
			}

			/* divide by diagonal element */
			#if defined(__USE_ASSERTS__)
				if ( fabs( accCholHessian(kk,0,ii,ii) ) < qpData->options.QPDUNES_ZERO * fabs( sum ) ) {
					qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveDenseL.\nsum = %.3e, diag = %.3e\nRank-deficient Matrix?", sum, accCholHessian(kk,0,ii,ii) );
					return QPDUNES_ERR_DIVISION_BY_ZERO;
				}
			#endif
			res->data[kk * _NX_ + ii] = sum / accCholHessian(kk,0,ii,ii);
		}
	}

	/* solve L*res = x */
	for (kk = 0; kk < _NI_; ++kk) /* go by block rows top down */
	{
		for (ii = 0; ii < _NX_; ++ii) /* go by in-block rows top down */
		{
			sum = res->data[kk * _NX_ + ii]; /* intermediate result of first backsolve is stored in res */
			/* subtract all previously resolved unknowns ... */
			if (kk > 0) { /* ... of corresponding subdiagonal block (if not first block row) */
				for (jj = 0; jj < _NX_; ++jj) {
					sum -= accCholHessian(kk,-1,ii,jj)* res->data[(kk-1)*_NX_+jj];
				}
			}
			for (jj = 0; jj < ii; ++jj) { /* ... of corresponding diagonal block */
				sum -= accCholHessian(kk,0,ii,jj)* res->data[kk*_NX_+jj];
			}

			/* divide by diagonal element */
			#if defined(__USE_ASSERTS__)
				if ( fabs( accCholHessian(kk,0,ii,ii) ) < qpData->options.QPDUNES_ZERO * fabs( sum ) ) {
					qpDUNES_printError( qpData, __FILE__, __LINE__, "Division by 0 in backsolveDenseL.\nsum = %.3e, diag = %.3e\nRank-deficient Matrix?", sum, accCholHessian(kk,0,ii,ii) );
					return QPDUNES_ERR_DIVISION_BY_ZERO;
				}
			#endif
			res->data[kk * _NX_ + ii] = sum / accCholHessian(kk,0,ii,ii);
		}
	}

//	qpDUNES_printMatrixDataToFile( res->data, _NI_*_NX_, 1, "dLambda_reverse", "" );

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_solveNewtonEquationBottomUp */


/* ----------------------------------------------
 * special multiplication routine for Newton Hessian with a vector
 *
 >>>>>>                                           */
return_t qpDUNES_multiplyNewtonHessianVector(	qpData_t* const qpData,
												xn_vector_t* const res,
												const xn2x_matrix_t* const hessian, /**< Newton Hessian */
												const xn_vector_t* const vec	)
{
	// TODO: switch to upper triangular matrix for higher cache efficiency!!
	int_t ii, jj, kk;


	for (kk = 0; kk < _NI_; ++kk) 			/* go by block rows top down */
	{
		/* empty full result vector chunk beforehand, needed for cache-efficient transposed multiplication */
		for (ii = 0; ii < _NX_; ++ii)
		{
			res->data[kk * _NX_ + ii] = 0.0;
		}

		/* write result */
		for (ii = 0; ii < _NX_; ++ii) 		/* go by in-block rows top down */
		{
			if (kk > 0)
			{
				/* go through subdiagonal block by columns */
				for (jj = 0; jj < _NX_; ++jj)
				{
					res->data[kk*_NX_ + ii] += accHessian(kk,-1,ii,jj) * vec->data[(kk-1)*_NX_ + jj];
				}
			}

			/* go through diagonal block by columns */
			for (jj = 0; jj < _NX_; ++jj)
			{
				res->data[kk*_NX_ + ii] += accHessian(kk,0,ii,jj) * vec->data[kk*_NX_ + jj];
			}

			if (kk < _NI_-1)
			{
				/* go through superdiagonal block by columns; transposed access though */
				for (jj = 0; jj < _NX_; ++jj)
				{
//					res->data[kk*_NX_ + ii] = accHessian(kk-1,-1,jj,ii) * vec[(kk-1)*_NI_ + jj];	/* transposed access of Newton Hessian */
					/* by-column multiplication for higher cache efficiency in Newton Hessian (transposed access of Newton Hessian) */
					res->data[kk*_NX_ + jj] += accHessian(kk+1,-1,ii,jj) * vec->data[(kk+1)*_NX_ + ii];
//					res->data[kk*_NX_ + ii] += accHessian(kk+1,-1,ii,jj) * vec->data[(kk+1)*_NX_ + jj];
				}
			}
		} 	/* end of in-block rows */
	}		/* end of block rows */

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_multiplyNewtonHessianVector */



/* ----------------------------------------------
 * ...
 * 
 >>>>>>                                           */
return_t qpDUNES_determineStepLength(	qpData_t* const qpData,
										xn_vector_t* const lambda,
										xn_vector_t* const deltaLambdaFS,
										uint_t* const itCntr,
										real_t* const alpha,
										real_t* const objValIncumbent,
										boolean_t newtonHessianRegularized
										)
{
	return_t statusFlag;

	int_t nV = _NX_ * _NI_;

	real_t alphaMin = 0.;
	real_t alphaMax = 1.;

	xn_vector_t* lambdaTry = &(qpData->xnVecTmp);

	*itCntr = 0;

	int_t kk;
	interval_t* interval;


	/* compute minimum step size for active set change */
	if ( newtonHessianRegularized != QPDUNES_TRUE )	{
		alphaMin = qpData->options.QPDUNES_INFTY;
		for ( kk = 0; kk < _NI_ + 1; ++kk )
		{
			switch (qpData->intervals[kk]->qpSolverSpecification)
			{
				case QPDUNES_STAGE_QP_SOLVER_CLIPPING:
					/* do check based on old multipliers */
					clippingQpSolver_getMinStepsize( qpData, qpData->intervals[kk], &alphaMin );	/* only rewrites alphaMin if smaller than incumbent */
					break;

				case QPDUNES_STAGE_QP_SOLVER_QPOASES:
					/* do check based on shortest homotopy step */
					qpOASES_getMinStepsize( qpData, qpData->intervals[kk], &alphaMin );
					break;

				default:
					qpDUNES_printError(qpData, __FILE__, __LINE__, "Stage QP solver undefined! Bailing out...");
					return QPDUNES_ERR_INVALID_ARGUMENT;
			}
	//		qpDUNES_printf("[qpDUNES] kk = %d, Minimum stepsize leading to first AS Change =  %.3e", kk, alphaMin );
		}
	}
	else	{
		/* Existence of AS change cannot be concluded if Hessian is regularized */
		alphaMin = 0.0;
	}
	if (qpData->options.printLevel >= 3) {
		qpDUNES_printf("[qpDUNES] Minimum stepsize leading to first AS Change =  %.3e (0.0 chosen if Hessian regularized)", alphaMin );
	}


	/* save old multipliers: swap y and yPrev */
	for ( kk = 0; kk < _NI_ + 1; ++kk )
	{
		interval = qpData->intervals[kk];
		real_t* y_swap = interval->yPrev.data;
		interval->yPrev.data = interval->y.data;
		interval->y.data = y_swap;
	}


	/* either: take full step and leave */
	if ( (alphaMin > 1. - qpData->options.equalityTolerance) && (newtonHessianRegularized == QPDUNES_FALSE) )
	{
		*alpha = 1.;

		addVectorScaledVector(lambda, lambda, *alpha, deltaLambdaFS, nV); /* temporary; TODO: move out to mother function */
		for (kk = 0; kk < _NI_ + 1; ++kk) {
			interval = qpData->intervals[kk];
			/* update primal, dual, and internal QP solver variables */
			switch (interval->qpSolverSpecification) {
				case QPDUNES_STAGE_QP_SOLVER_CLIPPING:
					directQpSolver_doStep(qpData, interval,
							&(interval->qpSolverClipping.dz), *alpha,
							&(interval->qpSolverClipping.zUnconstrained),
							&(interval->z), &(interval->y), &(interval->q),
							&(interval->p));
					break;

				case QPDUNES_STAGE_QP_SOLVER_QPOASES:
					qpOASES_doStep(qpData, interval->qpSolverQpoases.qpoasesObject,
							interval, *alpha, &(interval->z), &(interval->y),
							&(interval->q), &(interval->p));
					break;

				default:
					qpDUNES_printError(qpData, __FILE__, __LINE__, "Stage QP solver undefined! Bailing out...");
					return QPDUNES_ERR_UNKNOWN_ERROR;
			}
		}
		*objValIncumbent = qpDUNES_computeObjectiveValue(qpData);
		return QPDUNES_OK;
	}



	/* or: do a line search */
	switch (qpData->options.lsType) {
	case QPDUNES_LS_BACKTRACKING_LS:
		statusFlag = qpDUNES_backTrackingLineSearch( qpData, alpha, itCntr, deltaLambdaFS, lambdaTry, nV, 0., alphaMax, *objValIncumbent );
		if (statusFlag == QPDUNES_ERR_DECEEDED_MIN_LINESEARCH_STEPSIZE) {
			return statusFlag;
		}
		break;

	case QPDUNES_LS_BACKTRACKING_LS_WITH_AS_CHANGE:
		statusFlag = qpDUNES_backTrackingLineSearch( qpData, alpha, itCntr, deltaLambdaFS, lambdaTry, nV, 0., alphaMax, *objValIncumbent );
		if (statusFlag == QPDUNES_ERR_DECEEDED_MIN_LINESEARCH_STEPSIZE) {
			return statusFlag;
		}
		/* check for active set change: we need at least one AS change to get new Hessian information in next step */
		if ((alphaMin < 1. - qpData->options.equalityTolerance)
				&& (*alpha < alphaMin)) {
			*alpha = alphaMin;
		}
		break;

	case QPDUNES_LS_GOLDEN_SECTION_LS:
		/* TODO: add zi and dz at some point after finishing line search!!! */
		statusFlag = qpDUNES_goldenSectionIntervalSearch(qpData, alpha, itCntr, lambda, deltaLambdaFS, lambdaTry, nV, alphaMin, alphaMax);
//			qpDUNES_copyVector( lambda, lambdaTry, nV );
		break;

	case QPDUNES_LS_GRADIENT_BISECTION_LS:
		statusFlag = qpDUNES_bisectionIntervalSearch(qpData, alpha, itCntr,
				deltaLambdaFS, lambdaTry, nV, alphaMin, alphaMax);
		break;

	case QPDUNES_LS_ACCELERATED_GRADIENT_BISECTION_LS: /* first backtracking line search, then gradient bisection search */
		statusFlag = qpDUNES_backTrackingLineSearch(qpData, alpha, itCntr, deltaLambdaFS, lambdaTry, nV, 0., alphaMax, *objValIncumbent);
		if (statusFlag == QPDUNES_ERR_DECEEDED_MIN_LINESEARCH_STEPSIZE) { /* handle backtracking line search errors */
			return statusFlag;
		}
		alphaMax = qpDUNES_fmin(alphaMax, (*alpha) / qpData->options.lineSearchReductionFactor); /* take last alpha that did not yet lead to ascent */
		statusFlag = qpDUNES_bisectionIntervalSearch( qpData, alpha, itCntr, deltaLambdaFS, lambdaTry, nV, alphaMin, alphaMax );
//			TEMPORARY TRYING QUAD_APPROX_LS
//			did not work as well as expected..., probably quadratic approximation of shape over AS changes is not good; that's why we need to do the line search after all
//			qpDUNES_printf( "bisection search returned optimal alpha %.5e", *alpha);
//			*itCntr = 0;
//			statusFlag = qpDUNES_qpadApproxIntervalSearch( qpData, alpha, itCntr, deltaLambdaFS, lambdaTry, nV, alphaMin, alphaMax );
		break;

	case QPDUNES_LS_GRID_LS:
		statusFlag = qpDUNES_gridSearch( qpData, alpha, itCntr, objValIncumbent, alphaMin, alphaMax );
		break;

	case QPDUNES_LS_ACCELERATED_GRID_LS: /* backtracking line search as starter to reduce search interval, then grid search */
		statusFlag = qpDUNES_backTrackingLineSearch( qpData, alpha, itCntr,	deltaLambdaFS, lambdaTry, nV, 0., alphaMax, *objValIncumbent );
		if (statusFlag == QPDUNES_ERR_DECEEDED_MIN_LINESEARCH_STEPSIZE) { /* handle backtracking line search errors */
			return statusFlag;
		}
		alphaMax = qpDUNES_fmin(alphaMax, (*alpha) / qpData->options.lineSearchReductionFactor); /* take last alpha that did not yet lead to ascent */

		statusFlag = qpDUNES_gridSearch( qpData, alpha, itCntr, objValIncumbent, alphaMin, alphaMax );
		break;

	case QPDUNES_LS_HOMOTOPY_GRID_SEARCH:
		statusFlag = qpDUNES_homotopyGridSearch(	qpData, alpha, itCntr );
		break;

	default:
		statusFlag = QPDUNES_ERR_UNKNOWN_LS_TYPE;
		break;
	}

	/* UPDATE VARIABLES */
	/* lambda */
//	addVectorScaledVector( lambda, lambda, *alpha, deltaLambdaFS, nV );
	addScaledVector(lambda, *alpha, deltaLambdaFS, nV);
	/* stage QP variables */
	for (kk = 0; kk < _NI_ + 1; ++kk) {
		interval = qpData->intervals[kk];
		/* TODO: this might have already been done in line search; do not redo */
		/* update primal, dual, and internal QP solver variables */
		switch (interval->qpSolverSpecification) {
			case QPDUNES_STAGE_QP_SOLVER_CLIPPING:
				directQpSolver_doStep(qpData, interval,
						&(interval->qpSolverClipping.dz), *alpha,
						&(interval->qpSolverClipping.zUnconstrained),
						&(interval->z), &(interval->y), &(interval->q),
						&(interval->p));
				break;

			case QPDUNES_STAGE_QP_SOLVER_QPOASES:
				qpOASES_doStep(qpData, interval->qpSolverQpoases.qpoasesObject,
						interval, *alpha, &(interval->z), &(interval->y),
						&(interval->q), &(interval->p));
				break;

			default:
				qpDUNES_printError(qpData, __FILE__, __LINE__, "Stage QP solver undefined! Bailing out...");
				return QPDUNES_ERR_INVALID_ARGUMENT;
		}
	}
	*objValIncumbent = qpDUNES_computeObjectiveValue(qpData);

	/* return */
	return statusFlag;
}
/*<<< END OF qpDUNES_determineStepLength */


/* ----------------------------------------------
 * ...
 *
 >>>>>>                                           */
return_t qpDUNES_backTrackingLineSearch(	qpData_t* const qpData,
										real_t* const alpha,
										uint_t* const itCntr,
										const xn_vector_t* const deltaLambdaFS,
										xn_vector_t* const lambdaTry,
										int_t nV,
										real_t alphaMin,
										real_t alphaMax,
										real_t const objValIncumbent
										)
{
	real_t objVal;

	real_t minimumProgress = qpData->options.lineSearchMinRelProgress * fabs(objValIncumbent) + qpData->options.lineSearchMinAbsProgress;
	real_t normDeltaLambda = vectorNorm( deltaLambdaFS, nV );

	*alpha = alphaMax;

	/** perform line search */
	for ( /*continuous itCntr*/; (*itCntr) < qpData->options.maxNumLineSearchIterations; ++(*itCntr) )
	{
		/* get objective value */
		objVal = qpDUNES_computeParametricObjectiveValue(qpData, *alpha);

		/* check for progress */
		if (objVal > objValIncumbent + minimumProgress) {
			return QPDUNES_OK;
		}
		else { /* try smaller step size */
			*alpha = (*alpha) * qpData->options.lineSearchReductionFactor;
		}

		/* ensure minimum step size */
		if (normDeltaLambda * (*alpha - alphaMin) < qpData->options.equalityTolerance) {
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Backtracking line search: Deceeded minimum step size.\n          Itertation %d\n          alpha = %.3e\n          alphaMin = %.3e\n          normDeltaLambda = %.3e\n          minimumProgress = %.3e\n          last objVal = %.12e\n          objValIncumbet = %.12e", *itCntr, *alpha, alphaMin, normDeltaLambda, minimumProgress, objVal, objValIncumbent );
			*alpha = alphaMin;
			return QPDUNES_ERR_DECEEDED_MIN_LINESEARCH_STEPSIZE;
		}
	}

	if ( qpData->options.printLevel >= 3 ) {
		qpDUNES_printf("Leaving backtracking line search due to iteration limit, with alpha = %.3e. ObjVal = %.3e, incumbent objVal = %.3e", *alpha, objVal, objValIncumbent );
	}
	qpDUNES_printWarning( qpData, __FILE__, __LINE__, "Backtracking line search: Maximum number of iterations reached" );
	return QPDUNES_ERR_NUMBER_OF_MAX_LINESEARCH_ITERATIONS_REACHED;
}
/*<<< END OF qpDUNES_backTrackingLineSearch */


/* ----------------------------------------------
 * ...
 *
 >>>>>>                                           */
return_t qpDUNES_reductionLineSearchWithASChange(	qpData_t* const qpData,
													real_t* const alpha,
													uint_t* const itCntr,
													xn_vector_t* const lambda,
													const xn_vector_t* const deltaLambdaFS,
													xn_vector_t* const lambdaTry,
													int_t nV,
													real_t alphaMin,
													real_t alphaMax,
													real_t const objValIncumbent
													)
{
	assert(1 == 0);
	printf("qpDUNES_reductionLineSearchWithASChange not fixed yet!");
	real_t minimumProgress;
	real_t objVal;

	int_t nChgdConstr;

	minimumProgress = qpData->options.lineSearchMinRelProgress
			* fabs(objValIncumbent) + qpData->options.lineSearchMinAbsProgress;

	/** perform bisection */
	/* We know that objVal(alphaMax) < objValIncumbent, and objVal(alphaMin) > objValIncumbent
	 * We also know that alphaMax induces AS changes, while alphaMin does not induce AS changes
	 */
	for ( /*continuous itCntr*/;
			(*itCntr) < qpData->options.maxNumLineSearchRefinementIterations;
			++(*itCntr)) {
		*alpha = 0.5 * (alphaMin + alphaMax);
		addVectorScaledVector(lambdaTry, lambda, *alpha, deltaLambdaFS, nV);
		qpDUNES_solveAllLocalQPs(qpData, lambdaTry, 0);
		objVal = qpDUNES_computeObjectiveValue(qpData);

		if (objVal < objValIncumbent + minimumProgress) { /* no progress */
			alphaMax = (*alpha);
		} else { /* we make progress at current candidate */
			qpDUNES_printf(	"qpDUNES_reductionLineSearchWithASChange() currently not usable" );
			assert(1 == 0);
//			qpDUNES_getActSet( qpData, qpData->ieqStatus );
//			nChgdConstr = qpDUNES_compareActSets( qpData, qpData->ieqStatus, qpData->prevIeqStatus );
			if (nChgdConstr <= 0) { /* no active set changes */
				alphaMin = (*alpha);
			} else { /* both progress and AS changes */
				return QPDUNES_OK;
			}
		}
	}

	qpDUNES_printError(qpData, __FILE__, __LINE__,
			"Backtracking line search: Maximum number of iterations reached");
	return QPDUNES_ERR_NUMBER_OF_MAX_LINESEARCH_ITERATIONS_REACHED;
}
/*<<< END OF qpDUNES_backTrackingLineSearch */


/* ----------------------------------------------
 * golden section interval reduction search on interval ]alphaMin,alphaMax[
 * WARNING: currently no direct check for full steps implemented
 *
 >>>>>>                                           */
return_t qpDUNES_goldenSectionIntervalSearch(	qpData_t* const qpData,
											real_t* const alpha,
											uint_t* const itCntr,
											xn_vector_t* const lambda,
											const xn_vector_t* const deltaLambdaFS,
											xn_vector_t* const lambdaTry,
											int_t nV,
											real_t alphaMin,
											real_t alphaMax	)
{
	assert(1 == 0);
	printf("qpDUNES_goldenSectionIntervalSearch not fixed yet!");

	real_t objValLL, objValL, objValR, objValRR;
	real_t aLL, aL, aR, aRR;
	real_t alphaCheckedLast;

	real_t goldSec = 0.6180339887; /**< golden section ratio for interval line search (sqrt(5)-1)/2 */

	aLL = alphaMin;
	addVectorScaledVector(lambdaTry, lambda, aLL, deltaLambdaFS, nV);
	qpDUNES_solveAllLocalQPs(qpData, lambdaTry, 0);
	objValLL = qpDUNES_computeObjectiveValue(qpData);

	aRR = alphaMax;
	addVectorScaledVector(lambdaTry, lambda, aRR, deltaLambdaFS, nV);
	qpDUNES_solveAllLocalQPs(qpData, lambdaTry, 0);
	objValRR = qpDUNES_computeObjectiveValue(qpData);

	/** (1) ensure that L, R have bigger objective Values than LL and RR, respectively */
	for ( /*continuous itCntr*/; (*itCntr) < qpData->options.maxNumLineSearchRefinementIterations; ++(*itCntr)) {
		aL = aRR - goldSec * (aRR);
		addVectorScaledVector(lambdaTry, lambda, aL, deltaLambdaFS, nV);
		qpDUNES_solveAllLocalQPs(qpData, lambdaTry, 0);
		objValL = qpDUNES_computeObjectiveValue(qpData);

		if (objValLL > objValL) { /* minimum has to lie on left-most interval */
			aRR = aL;
			objValRR = objValL;
			continue;
		} else {
			break;
		}
	}
	for ( /*continuous itCntr*/; (*itCntr) < qpData->options.maxNumLineSearchRefinementIterations; ++(*itCntr)) {
		aR = aLL + goldSec * (aRR - aLL);
		addVectorScaledVector(lambdaTry, lambda, aR, deltaLambdaFS, nV);
		qpDUNES_solveAllLocalQPs(qpData, lambdaTry, 0);
		objValR = qpDUNES_computeObjectiveValue(qpData);

		if (objValRR > objValR) { /* minimum has to lie on right-most interval */
			aLL = aR;
			objValLL = objValR;
			continue;
		} else {
			break;
		}
	}
	alphaCheckedLast = aR;

	/** (2) regular golden section interval search */
	for ( /*continuous itCntr*/; (*itCntr) < qpData->options.maxNumLineSearchRefinementIterations; ++(*itCntr)) {
		/* check for stationarity */
		if ( (2 * objValL - objValR - objValLL <= 2 * qpData->options.lineSearchStationarityTolerance)
			 || (2 * objValR - objValL - objValRR <= 2 * qpData->options.lineSearchStationarityTolerance))
		{
			/* use last computed a in case of stationarity for consistent lambda and alpha.
			 * This can lead to a slightly worse IS performance than always taking the minimum,
			 * but is easier to handle in terms of memory management, etc.
			 */
			*alpha = alphaCheckedLast;
			return QPDUNES_OK;
		}

		/* remove one interval */
		if (objValL >= objValR) { /* throw out right interval */
			aRR = aR;
			aR = aL;
			objValRR = objValR;
			objValR = objValL;

			aL = aRR - goldSec * (aRR - aLL);
			addVectorScaledVector(lambdaTry, lambda, aL, deltaLambdaFS, nV);
			qpDUNES_solveAllLocalQPs(qpData, lambdaTry, 0);
			alphaCheckedLast = aL;
			objValL = qpDUNES_computeObjectiveValue(qpData);
		} else { /* throw out left interval */
			aLL = aL;
			aL = aR;
			objValLL = objValL;
			objValL = objValR;

			aR = aLL + goldSec * (aRR - aLL);
			addVectorScaledVector(lambdaTry, lambda, aR, deltaLambdaFS, nV);
			qpDUNES_solveAllLocalQPs(qpData, lambdaTry, 0);
			alphaCheckedLast = aR;
			objValR = qpDUNES_computeObjectiveValue(qpData);
		}
	}

	*alpha = alphaCheckedLast;
	qpDUNES_printError(qpData, __FILE__, __LINE__,
			"Golden section interval search: Maximum number of iterations reached");
	return QPDUNES_ERR_NUMBER_OF_MAX_LINESEARCH_ITERATIONS_REACHED;
}
/*<<< END OF qpDUNES_goldenSectionIntervalSearch */


/* ----------------------------------------------
 * ...
 *
 >>>>>>                                           */
return_t qpDUNES_bisectionIntervalSearch(	qpData_t* const qpData,
										real_t* const alpha,
										uint_t* const itCntr,
										const xn_vector_t* const deltaLambdaFS,
										xn_vector_t* const lambdaTry,
										int_t nV,
										real_t alphaMin,
										real_t alphaMax
										)
{
	real_t alphaC;

	real_t alphaSlope;
	real_t slopeNormalization = fmin( 1., vectorNorm((vector_t*)deltaLambdaFS,nV) ); 	/* demand more stationarity for smaller steps */
	/* FIXME: is this meaningful? */

	/* todo: get memory passed on from determine step length */
	xn_vector_t* gradientTry = &(qpData->xnVecTmp2);
	/* todo: no need to recompute gradient in next Newton iteration! */

	int_t kk, ii;
	interval_t* interval;

	/* TODO: take line search iterations and maxNumLineSearchRefinementIterations together! */
	/** (1) check if full step is stationary or even still ascent direction */
	for ( /*continuous itCntr*/; (*itCntr) < qpData->options.maxNumLineSearchRefinementIterations; ++(*itCntr)) {
		/* update z locally according to alpha guess */
		for (kk = 0; kk < _NI_ + 1; ++kk) {
			interval = qpData->intervals[kk];
			z_vector_t* zTry = &(interval->zVecTmp);
			/* get primal variables for trial step length */
			addVectorScaledVector(zTry,	&(interval->qpSolverClipping.zUnconstrained), alphaMax,	&(interval->qpSolverClipping.dz), interval->nV);
			directQpSolver_saturateVector(qpData, zTry, &(interval->y), &(interval->zLow), &(interval->zUpp), &(interval->H), interval->nV);
		}

		/* manual gradient computation; TODO: use function, but watch out with z, dz, zTry, etc. */
		for (kk = 0; kk < _NI_; ++kk) {
			/* ( A_kk*x_kk^opt + B_kk*u_kk^opt + c_kk ) - x_(kk+1)^opt */
			multiplyCz( qpData, &(qpData->xVecTmp), &(qpData->intervals[kk]->C), &(qpData->intervals[kk]->zVecTmp) );
			addToVector(&(qpData->xVecTmp), &(qpData->intervals[kk]->c), _NX_); /* TODO: avoid using global memory!!! */

			/* subtractFromVector( xVecTmp, &(intervals[kk+1]->x), _NX_ ); */
			for (ii = 0; ii < _NX_; ++ii) {
				qpData->xVecTmp.data[ii] -=	qpData->intervals[kk + 1]->zVecTmp.data[ii];
			}

			/* write gradient part */
			for (ii = 0; ii < _NX_; ++ii) {
				gradientTry->data[kk * _NX_ + ii] = qpData->xVecTmp.data[ii];
			}
		}
//		computeNewtonGradient( qpData, gradientTry, &(qpData->xVecTmp) );	/* TODO: avoid to use own memory here! */
		alphaSlope = scalarProd(gradientTry, deltaLambdaFS, nV);

		/* take full step if stationary */
		if (fabs(alphaSlope / slopeNormalization) <= qpData->options.lineSearchStationarityTolerance)
		{
//			qpDUNES_printf("Full step is stationary, alphaSlope = %.3e, normalization = %.3e", alphaSlope, slopeNormalization );
			*alpha = alphaMax;
			return QPDUNES_OK;
		}

		/* go into normal interval search if full step leads to descent */
		if (alphaSlope / slopeNormalization < 0.) {
			break;
		}

		/* increase step size otherwise (full step still leads to ascent) */
		alphaMin = alphaMax;
		alphaMax *= qpData->options.lineSearchIncreaseFactor;

		/* break if maximum step size reached */
		if (alphaMax > qpData->options.lineSearchMaxStepSize) {
			*alpha = alphaMin;
//			qpDUNES_copyVector( lambda, lambdaTry, nV );	/* TODO: reuse lambdaTry! */
			if ( qpData->options.printLevel >= 3 ) {
				qpDUNES_printf("[qpDUNES] Bisection interval search reached maximum step size");
			}
			return QPDUNES_ERR_EXCEEDED_MAX_LINESEARCH_STEPSIZE;
		}
	}


	/** (2) regular bisection interval search */
	for ( /*continuous itCntr*/; (*itCntr) < qpData->options.maxNumLineSearchRefinementIterations; ++(*itCntr) ) {
		alphaC = 0.5 * (alphaMin + alphaMax);

		/* update z locally according to alpha guess */
		for (kk = 0; kk < _NI_ + 1; ++kk) {
			interval = qpData->intervals[kk];
			z_vector_t* zTry = &(interval->zVecTmp);
			/* get primal variables for trial step length */
			addVectorScaledVector( zTry, &(interval->qpSolverClipping.zUnconstrained), alphaC, &(interval->qpSolverClipping.dz), interval->nV );
			directQpSolver_saturateVector( qpData, zTry, &(interval->y), &(interval->zLow), &(interval->zUpp), &(interval->H), interval->nV );
		}

		/* manual gradient computation; TODO: use function, but watch out with z, dz, zTry, etc. */
		for (kk = 0; kk < _NI_; ++kk) {
			/* ( A_kk*x_kk^opt + B_kk*u_kk^opt + c_kk ) - x_(kk+1)^opt */
			multiplyCz( qpData, &(qpData->xVecTmp), &(qpData->intervals[kk]->C), &(qpData->intervals[kk]->zVecTmp) );
			addToVector( &(qpData->xVecTmp), &(qpData->intervals[kk]->c), _NX_ );

			/* subtractFromVector( xVecTmp, &(intervals[kk+1]->x), _NX_ ); */
			for (ii = 0; ii < _NX_; ++ii) {
				qpData->xVecTmp.data[ii] -= qpData->intervals[kk + 1]->zVecTmp.data[ii];
			}

			/* write gradient part */
			for (ii = 0; ii < _NX_; ++ii) {
				gradientTry->data[kk * _NX_ + ii] = qpData->xVecTmp.data[ii];
			}
		}
//		computeNewtonGradient( qpData, gradientTry, &(qpData->xVecTmp) );	/* TODO: avoid to use own memory here! */
		alphaSlope = scalarProd(gradientTry, deltaLambdaFS, nV);

		/* check for stationarity in search direction */
//		qpDUNES_printf("AlphaC = %.15e is tried, alphaSlope = %.15e, normalization = %.15e", alphaC, alphaSlope, slopeNormalization );
		if ( fabs(alphaSlope / slopeNormalization) <= qpData->options.lineSearchStationarityTolerance ) {
			if ( qpData->options.printLevel >= 4 ) {
				qpDUNES_printf("AlphaC = %.5e is stationary, alphaSlope = %.3e, normalization = %.3e", alphaC, alphaSlope, slopeNormalization );
			}
			*alpha = alphaC;
			return QPDUNES_OK;
		}
		else {
			/* half interval */
			if (alphaSlope > 0) { /* ascent right of gradient */
				alphaMin = alphaC; /* throw out left interval */
			}
			else { /* ascent left of gradient */
				alphaMax = alphaC; /* throw out right interval */
			}
		}
	}

	#ifdef __DEBUG__
	if( qpData->options.printLevel >= 3 ) {
		qpDUNES_printf( "Bisection interval search: itCntr = %d, last alpha: %.3e", *itCntr, alphaC );
	}
	#endif
	qpDUNES_printWarning( qpData, __FILE__, __LINE__,	"Bisection interval search: Maximum number of iterations reached!" );
	*alpha = alphaC;

	return QPDUNES_ERR_NUMBER_OF_MAX_LINESEARCH_ITERATIONS_REACHED;
}
/*<<< END OF qpDUNES_bisectionIntervalSearch */


/* ----------------------------------------------
 * ...
 *
 >>>>>>                                           */
return_t qpDUNES_qpadApproxIntervalSearch(	qpData_t* const qpData,
										real_t* const alpha,
										uint_t* const itCntr,
										const xn_vector_t* const deltaLambdaFS,
										xn_vector_t* const lambdaTry,
										int_t nV,
										real_t alphaL,
										real_t alphaR
										)
{
	real_t alphaC, alphaD;
	real_t objValL, objValC, objValR, objValD;
	real_t sCL, sRC, sLR;

	real_t alphaSlope;
//	real_t slopeNormalization = fmin( 1., vectorNorm( (vector_t*)deltaLambdaFS, nV ));	/* demand more stationarity for smaller steps */
	/* FIXME: is this meaningful? */

	/* todo: get memory passed on from determine step length */
	xn_vector_t* gradientTry = &(qpData->xnVecTmp2);
	/* todo: no need to recompute gradient in next Newton iteration! */

	int_t kk, ii;
	interval_t* interval;

	/* get objective value */
	/** get first alphaC and function values */
//	alphaC = 0.5*(alphaL+alphaR);
	objValL = qpDUNES_computeParametricObjectiveValue(qpData, alphaL);
	objValR = qpDUNES_computeParametricObjectiveValue(qpData, alphaR);
//	objValC = qpDUNES_computeParametricObjectiveValue( qpData, alphaC );

	for ( /*continuous itCntr*/; (*itCntr) < qpData->options.maxNumLineSearchRefinementIterations; ++(*itCntr) )
	{
		alphaC = 0.5 * (alphaL + alphaR);
		objValC = qpDUNES_computeParametricObjectiveValue(qpData, alphaC);
		qpDUNES_printf("aL = %.5e, aC = %.5e, aR = %.5e", alphaL, alphaC, alphaR);
		qpDUNES_printf("oL = %.5e, oC = %.5e, oR = %.5e", objValL, objValC, objValR);
		sCL = (objValC - objValL) / (alphaC - alphaL);
		sRC = (objValR - objValC) / (alphaR - alphaC);
		sLR = (sRC - sCL) / (alphaR - alphaL);
		alphaD = 0.5 * ((alphaL + alphaC) - sCL / sLR);
		objValD = qpDUNES_computeParametricObjectiveValue(qpData, alphaD);
		qpDUNES_printf("found aD = %.5e, oD = %.5e", alphaD, objValD);
//        qpDUNES_printf("sCL = %.5e, sRC = %.5e", sCL, sRC );

		/* check for stationarity in search direction */
//		first in the cheap way, though gradient is not so much more expensive...
//        alphaSlope = qpDUNES_fmax( fabs(sCL) , fabs(sRC) );
//      cleaner, using gradient:
		/* update z locally according to alpha guess */
		for (kk = 0; kk < _NI_ + 1; ++kk) {
			interval = qpData->intervals[kk];
			z_vector_t* zTry = &(interval->zVecTmp);
			/* get primal variables for trial step length */
			addVectorScaledVector(zTry, &(interval->qpSolverClipping.zUnconstrained), alphaD, &(interval->qpSolverClipping.dz), interval->nV);
			directQpSolver_saturateVector(qpData, zTry, &(interval->y),	&(interval->zLow), &(interval->zUpp), &(interval->H), interval->nV);
		}

		/* manual gradient computation; TODO: use function, but watch out with z, dz, zTry, etc. */
		for (kk = 0; kk < _NI_; ++kk) {
			/* ( A_kk*x_kk^opt + B_kk*u_kk^opt + c_kk ) - x_(kk+1)^opt */
			multiplyCz(qpData, &(qpData->xVecTmp), &(qpData->intervals[kk]->C), &(qpData->intervals[kk]->zVecTmp));
			addToVector(&(qpData->xVecTmp), &(qpData->intervals[kk]->c), _NX_);

			/* subtractFromVector( xVecTmp, &(intervals[kk+1]->x), _NX_ ); */
			for (ii = 0; ii < _NX_; ++ii) {
				qpData->xVecTmp.data[ii] -=	qpData->intervals[kk + 1]->zVecTmp.data[ii];
			}

			/* write gradient part */
			for (ii = 0; ii < _NX_; ++ii) {
				gradientTry->data[kk * _NX_ + ii] = qpData->xVecTmp.data[ii];
			}
		}
		alphaSlope = fabs(scalarProd(gradientTry, deltaLambdaFS, nV));
//		end of cleaner...

		qpDUNES_printf("alphaSlope is %.5e", alphaSlope);
		qpDUNES_printf("--");
		if (alphaSlope <= qpData->options.lineSearchStationarityTolerance) {
			qpDUNES_printf(
					"AlphaD = %.5e is stationary, alphaSlope = %.3e, normalization = %.3e",
					alphaD, alphaSlope, 0.);
			*alpha = alphaD;
			return QPDUNES_OK;
		}

//        if (objValD > objValC) {	/* quadratic model did not lead to improvement, so keep alphaC as center */
//        	if ( alphaD < alphaC ) {	/* alphaD was left of min, so move left interval boundary alphaL */
//        		alphaL = alphaD;	objValL = objValD;
//        	}
//        	else { 	/* alphaD > alphaC, so alphaD was right of min, so move right interval boundary alphaR */
//        		alphaR = alphaD; 	objValR = objValD;
//        	}
//        }
//        else {		/* quadratic model did lead to improvement, so update center alphaC */
//        	if ( alphaC < alphaD ) {	/* alphaC was left of min, so move left interval boundary alphaL */
//        		alphaL = alphaC;	objValL = objValC;
//        	}
//        	else { 	/* alphaC > alphaD, so alphaC was right of min, so move right interval boundary alphaR */
//        		alphaR = alphaC; 	objValR = objValC;
//        	}
//			alphaC = alphaD;	objValC = objValD;
//        }
		if (objValD > objValC) {
			qpDUNES_printf("quadratic model did not lead to improvement! ");
		} else {
			qpDUNES_printSuccess(qpData,
					"quadratic model did lead to improvement");
		}
		// try to choose best two
		if (alphaD < alphaC) {
			if (objValC > objValL) { /* take L, D */
				alphaR = alphaD;
				objValR = objValD;
			} else {
				if (objValD > objValR) { /* take C, R */
					alphaL = alphaC;
					objValL = objValC;
				} else { /* take D, C */
					alphaL = alphaD;
					objValL = objValD;
					alphaR = alphaC;
					objValR = objValC;
				}
			}
		} else { /* alphaC < alphaD */
			if (objValD > objValL) { /* take L, C */
				alphaR = alphaC;
				objValR = objValC;
			} else {
				if (objValC > objValR) { /* take D, R */
					alphaL = alphaD;
					objValL = objValD;
				} else { /* take C, D */
					alphaL = alphaC;
					objValL = objValC;
					alphaR = alphaD;
					objValR = objValD;
				}
			}
		}
	}

	qpDUNES_printError(qpData, __FILE__, __LINE__,
			"Quadratic approximation interval search: Maximum number of iterations reached (itCntr = %d, last alpha: %.3e)",
			*itCntr, alphaC);
	*alpha = alphaC;
	return QPDUNES_ERR_NUMBER_OF_MAX_LINESEARCH_ITERATIONS_REACHED;
}
/*<<< END OF qpDUNES_qpadApproxIntervalSearch */


/* ----------------------------------------------
 * ...
 *
 >>>>>>                                           */
return_t qpDUNES_gridSearch(	qpData_t* const qpData,
								real_t* const alpha,
								uint_t* const itCntr,
								real_t* const objValIncumbent,
								real_t alphaMin,
								real_t alphaMax
								)
{
	int_t kk;

	real_t alphaTry;
	real_t objValTry;

//	qpDUNES_printf("[");
	/* todo: maybe do more efficiently for a parallelized version by passing grid directly to QP nodes */
	for (kk = 0; kk < qpData->options.lineSearchNbrGridPoints; ++kk) {
		alphaTry = alphaMin
				+ kk * (alphaMax - alphaMin)
						/ (qpData->options.lineSearchNbrGridPoints - 1);
		objValTry = qpDUNES_computeParametricObjectiveValue(qpData, alphaTry);
//		qpDUNES_printf("[ %.12e\t %.12e ];", alphaTry, objValTry );
		if (objValTry > *objValIncumbent) {
			*objValIncumbent = objValTry;
			*alpha = alphaTry;
		}
	}
	*itCntr += qpData->options.lineSearchNbrGridPoints;

//	qpDUNES_printf("]");

	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_gridSearch */


/* ----------------------------------------------
 * smarter Grid search that considers all points
 * where a stage homotopy has an active set change
 *
 >>>>>>                                           */
return_t qpDUNES_homotopyGridSearch(	qpData_t* const qpData,
										real_t* const alpha,
										uint_t* const itCntr
										)
{
	int_t kkThis, kkOther, ii;

	interval_t* intervalThis;

	real_t alphaIncumbent;
	real_t objVal_alphaIncumbent = -qpData->options.QPDUNES_INFTY;
	real_t dfdAlpha;
	int_t dfdAlphaSign;
	real_t ddfdAlpha;


	/* (1) sum up objective function values at all active set changes */
	for (kkThis = 0; kkThis < _NI_+1; ++kkThis) {
		intervalThis = qpData->intervals[kkThis];

		/* initialize sums at alpha points of this node */
		for ( ii=1; ii<intervalThis->parametricObjFctn_nBasePoints-1; ++ii )	{
			intervalThis->parametricObjFctn_fSum.data[ii] = intervalThis->parametricObjFctn_f.data[ii];
		}

		/* sum function values of all other nodes at alpha points of this node
		 * note: this loop must not be parallelized, otherwise race conditions may appear; maximally cyclic reduction */
		for ( kkOther = 0; kkOther < _NI_+1; ++kkOther )	{
			if (kkOther == kkThis)	continue;
//			///
//			qpDUNES_printf_noNewLine( "interval %d sums before %d :\t", intervalThis->id, kkOther );
//			for ( ii=1; ii<intervalThis->parametricObjFctn_nBasePoints-1; ++ii )	{
//				qpDUNES_printf_noNewLine( "%.6e\t", intervalThis->parametricObjFctn_fSum.data[ii] );
//			}
//			qpDUNES_printf("");
//			///

			qpOASES_evalAddParametricObjFctn( 	qpData,
												qpData->intervals[kkOther],
												&(intervalThis->parametricObjFctn_fSum),
												&(intervalThis->parametricObjFctn_alpha),
												intervalThis->parametricObjFctn_nBasePoints
												);
		}
//		///
//		qpDUNES_printf_noNewLine( "interval %d sums after %d :\t", intervalThis->id, _NI_+1 );
//		for ( ii=1; ii<intervalThis->parametricObjFctn_nBasePoints-1; ++ii )	{
//			qpDUNES_printf_noNewLine( "%.6e\t", intervalThis->parametricObjFctn_fSum.data[ii] );
//		}
//		qpDUNES_printf("");
//		qpDUNES_printf_noNewLine( "Validation interval %d    :\t", intervalThis->id );
//		for ( ii=1; ii<intervalThis->parametricObjFctn_nBasePoints-1; ++ii )	{
//			real_t validationVal = qpDUNES_computeParametricObjectiveValue( qpData, intervalThis->parametricObjFctn_alpha.data[ii] );
//			qpDUNES_printf_noNewLine( "%.6e\t", validationVal );
//		}
//		qpDUNES_printf("");
//		///

		/* (2) take the alpha value with the largest objective function value
		 */
		for ( ii=1; ii<intervalThis->parametricObjFctn_nBasePoints-1; ++ii )	{
//			qpDUNES_printf("this = %d, had fsum[%d] = %.5e (alpha = %.3e)", kkThis, ii, intervalThis->parametricObjFctn_fSum.data[ii], intervalThis->parametricObjFctn_alpha.data[ii]);
			if ( intervalThis->parametricObjFctn_fSum.data[ii] > objVal_alphaIncumbent )	{
				objVal_alphaIncumbent = intervalThis->parametricObjFctn_fSum.data[ii];
				alphaIncumbent = intervalThis->parametricObjFctn_alpha.data[ii];
			}
		}
	}

//	qpDUNES_printf("incumbent pair is fsum = %.5e (alpha = %.3e)", objVal_alphaIncumbent, alphaIncumbent);

	/* (3) compute minimum between these three points */


	/* evaluate gradient at alphaIncumbent */
	dfdAlpha = 0.0;
	for (kkOther = 0; kkOther < _NI_+1; ++kkOther)	{
//		if (kkOther == alphaMaxIdx)	continue;
		dfdAlpha += qpOASES_getParametricObjFctnGrad( qpData, qpData->intervals[kkOther], alphaIncumbent );
		/* todo: write this function */
		/* sum all other hessians...problem, if AS change at exactly the same problem... */
	}

	/* depending on sign of gradient, choose hessian of this */
	if ( fabs( dfdAlpha ) < qpData->options.lineSearchStationarityTolerance )	{
		/* stationarity */
		*alpha = alphaIncumbent;
		return QPDUNES_OK;
	}
	else	{
		if ( dfdAlpha < -qpData->options.equalityTolerance )	{
			/* lower branch */
			dfdAlphaSign = -1;
		}
		else	{
			/* upper branch */
			dfdAlphaSign = +1;
		}
	}

	/* get hessian depending on gradient direction */
	ddfdAlpha = 0.0;
	for (kkOther = 0; kkOther < _NI_+1; ++kkOther)	{
		ddfdAlpha += qpOASES_getParametricObjFctnHess( qpData, qpData->intervals[kkOther], alphaIncumbent, dfdAlphaSign );
	}


	/* get optimal alpha by one Newton iteration */
//	qpDUNES_printf( "alphaIncumbent = %.3e", alphaIncumbent );
//	qpDUNES_printf( "dfdAlpha = %.3e", dfdAlpha );
//	qpDUNES_printf( "dfdAlphaSign = %d", dfdAlphaSign );
//	qpDUNES_printf( "ddfdAlpha = %.3e", ddfdAlpha );
	*alpha = qpDUNES_fmin( 1, alphaIncumbent - dfdAlpha/ddfdAlpha );		/* clip alpha at 1, since we do not have homotopy information to extrapolate beyond */
//	qpDUNES_printf( "*alpha = %.3e", *alpha );


	/* SOME TESTS */
#ifdef __DO_UNIT_TESTS__
	/* Unit test 1: check whether predicted gradient is zero at alpha */
	dfdAlpha = 0.0;
	for (kkOther = 0; kkOther < _NI_+1; ++kkOther)	{
		dfdAlpha += qpOASES_getParametricObjFctnGrad( qpData, qpData->intervals[kkOther], *alpha );
	}
	qpDUNES_printf( "[qpDUNES] homotopyGridSearch UNIT TEST 1:\n          line search function model stationarity at found alpha=%.3e is %.3e", *alpha, dfdAlpha );

	/* Unit test 2: evaluate objective function left and right of alpha and compare total objective value */
	real_t delta = 1.e-3;
	real_t objVal_alpha;
	real_t objVal_lower;
	real_t objVal_upper;
	objVal_alpha = qpDUNES_computeParametricObjectiveValue( qpData, *alpha );
	objVal_lower = qpDUNES_computeParametricObjectiveValue( qpData, *alpha - delta );
	objVal_upper = qpDUNES_computeParametricObjectiveValue( qpData, *alpha + delta );
	qpDUNES_printf( "[qpDUNES] homotopyGridSearch UNIT TEST 2:\n          comparing incumbent objective value with slightly the ones of slightly smaller and larger alphas:" );
	qpDUNES_printf( "          smaller alpha : %+.12e", objVal_lower );
	qpDUNES_printf( "          incumbent     : %+.12e", objVal_alpha );
	qpDUNES_printf( "          bigger alpha  : %+.12e", objVal_upper );
	qpDUNES_printf( "          incumbentPred.: %+.12e", objVal_alphaIncumbent );
#endif

	*itCntr += _NI_+1;



	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_homotopyGridSearch */


/* ----------------------------------------------
 * temporary infeasibility check...
 *
 >>>>>>                                           */
return_t qpDUNES_infeasibilityCheck(	qpData_t* qpData
										)
{
	qpDUNES_printWarning( qpData, __FILE__, __LINE__, "Infeasibility check is currently only performed for clipping QP solver!" );

	uint_t kk;
	interval_t* interval;
	xn_vector_t* hessianStepProduct = &(qpData->xnVecTmp);

	real_t minStepSizeASChange = qpData->options.QPDUNES_INFTY;
	real_t minStepSizeASChange_k;
	real_t ascentCurvature;

	//TODO: also abort if multipliers grow too big

	// 1) check in z step direction whether active set change possible
	for( kk=0; kk< _NI_+1; ++kk ) {
		interval = qpData->intervals[kk];
		clippingQpSolver_ratioTest( qpData, &minStepSizeASChange_k,
									&(interval->qpSolverClipping.dz), &(interval->y),
									&(interval->zLow), &(interval->zUpp), interval->nV );
		if ( minStepSizeASChange_k < minStepSizeASChange )
		{
			#ifdef __DEBUG__
			if ( qpData->options.printLevel >= 4 ) {
				qpDUNES_printf( "[qpDUNES] Infeasibility check: New minimum step size k=%d: %.3e", kk, minStepSizeASChange_k );
			}
			#endif
			minStepSizeASChange = minStepSizeASChange_k;
		}
	}

	#ifdef __DEBUG__
	if ( qpData->options.printLevel >= 3 ) {
		qpDUNES_printf( "[qpDUNES] Infeasibility check: Minimum step size for next active set change: %.3e", minStepSizeASChange );
	}
	#endif
	if ( minStepSizeASChange < qpData->options.QPDUNES_INFTY - qpData->options.equalityTolerance )
	{	// step direction is not unbounded
		return QPDUNES_OK;
	}


	// 2) check whether deltaLambda is in Hessian null space (i.e., infinite ascent direction)
	qpDUNES_multiplyNewtonHessianVector( qpData, hessianStepProduct, &(qpData->hessian), &(qpData->deltaLambda) );
//	qpDUNES_printNewtonHessian( qpData, &(qpData->hessian) );
//	qpDUNES_printMatrixData( qpData->hessian.data, _NI_*_NX_, 2*_NX_, "hess");
//	qpDUNES_printMatrixData( qpData->deltaLambda.data, 1, _NI_*_NX_, "step");
//	qpDUNES_printMatrixData( hessianStepProduct->data, 1, _NI_*_NX_, "hess step product");
	ascentCurvature = vectorNorm( hessianStepProduct, _NI_ * _NX_ );
//	qpDUNES_printf( "Curvature in Newton Step direction: %.3e", ascentCurvature );
//	assert(1==0);
	#ifdef __DEBUG__
	if ( qpData->options.printLevel >= 3 ) {
		qpDUNES_printf( "Curvature in Newton Step direction: %.3e", ascentCurvature );
	}
	#endif


	// 3) decide whether problem can be considered infeasible
	if ( ascentCurvature <= qpData->options.ascentCurvatureTolerance )	{
		qpDUNES_printError( qpData, __FILE__, __LINE__, "Stage coupling constraints determined infeasible." );
		return QPDUNES_ERR_STAGE_COUPLING_INFEASIBLE;
	}
	else {
		return QPDUNES_OK;
	}
}
/*<<< END OF qpDUNES_infeasibilityCheck */


/* ----------------------------------------------
 * ...
 * 
 >>>>>>                                           */
return_t qpDUNES_getPrimalSol(const qpData_t* const qpData, real_t* const z) {
	int_t kk;

	for (kk = 0; kk < _NI_ + 1; ++kk) {
		qpDUNES_copyArray(&(z[kk * _NZ_]), qpData->intervals[kk]->z.data,
				qpData->intervals[kk]->nV);
	}

	return QPDUNES_ERR_INVALID_ARGUMENT;
}
/*<<< END OF qpDUNES_getPrimalSol */


/* ----------------------------------------------
 * ...
 *
 >>>>>>                                           */
return_t qpDUNES_getDualSol(const qpData_t* const qpData, real_t* const lambda, real_t* const y) {
	int_t kk, ii;

	int_t nStageMult;
	int_t nDOffset = 0;

	/* get lambda */
	qpDUNES_copyArray( lambda, qpData->lambda.data, _NI_*_NX_ );

	/* get y */
	for( kk=0; kk<_NI_+1; ++kk ) {
		nStageMult = 2* (qpData->intervals[kk]->nV + qpData->intervals[kk]->nD);
		switch (qpData->intervals[kk]->qpSolverSpecification)	{
			case QPDUNES_STAGE_QP_SOLVER_CLIPPING:
				/* we still have to clean the multipliers */
				for ( ii=0; ii<nStageMult; ++ii )	{
					y[nDOffset+ii] = (qpData->intervals[kk]->y.data[ii] > 0)  ?  qpData->intervals[kk]->y.data[ii]  :  0.0;
				}
				break;

			case QPDUNES_STAGE_QP_SOLVER_QPOASES:
				qpDUNES_copyArray( &(y[nDOffset]), qpData->intervals[kk]->y.data, nStageMult );
				break;

			default:
				qpDUNES_printError( qpData, __FILE__, __LINE__,	"Stage QP %d solver undefined! Bailing out...", kk );
				return QPDUNES_ERR_INVALID_ARGUMENT;
		}
		nDOffset += nStageMult;
	}
	return QPDUNES_OK;
}
/*<<< END OF qpDUNES_getDualSol */


/* ----------------------------------------------
 * ...
 *
 >>>>>>                                           */
real_t qpDUNES_computeObjectiveValue(qpData_t* const qpData) {
	int_t kk;
	interval_t* interval;

	real_t objVal = 0.;

	for (kk = 0; kk < _NI_ + 1; ++kk) {
		interval = qpData->intervals[kk];

		/* quadratic objective part */
		interval->optObjVal = 0.5
				* multiplyzHz(qpData, &(interval->H), &(interval->z),
						interval->nV);
		/* linear objective part */
		interval->optObjVal += scalarProd(&(interval->q), &(interval->z),
				interval->nV);
		/* constant objective part */
		interval->optObjVal += interval->p;

//		qpDUNES_printMatrixData( interval->dLow.data, 1, interval->nV, "dLow[%d]",kk );
//		qpDUNES_printMatrixData( interval->dUpp.data, 1, interval->nV, "dUpp[%d]",kk );
//		qpDUNES_printMatrixData( interval->z.data, 1, interval->nV, "z[%d]",kk );
//		qpDUNES_printMatrixData( interval->q.data, 1, _NZ_, "q[%d]",kk );
//		qpDUNES_printf( "p[%d] = %.10e", kk, interval->p );

		/* sum up */
		objVal += interval->optObjVal;
	}

	return objVal;
}
/*<<< END OF qpDUNES_computeObjectiveValue */


/* ----------------------------------------------
 * ...
 *
 >>>>>>                                           */
real_t qpDUNES_computeParametricObjectiveValue(	qpData_t* const qpData,
												const real_t alpha
												)
{
	int_t kk;

	real_t objVal = 0.;

	interval_t* interval;

	/* TODO: move to own function in direct QP solver, a la getObjVal( qpData, interval, alpha ) */
	for (kk = 0; kk < _NI_ + 1; ++kk) {
		interval = qpData->intervals[kk];
		z_vector_t* qTry = &(interval->zVecTmp);
//		qpDUNES_copyVector( qTry, &(interval->q), interval->nV );
		real_t pTry;

		/* get primal variables for trial step length */
		switch (interval->qpSolverSpecification) {
			case QPDUNES_STAGE_QP_SOLVER_CLIPPING:
				directQpSolver_doStep( qpData, interval, &(interval->qpSolverClipping.dz), alpha, &(interval->z ), &(interval->z), &(interval->y), qTry, &pTry );
				break;

			case QPDUNES_STAGE_QP_SOLVER_QPOASES:
				qpOASES_doStep( qpData, interval->qpSolverQpoases.qpoasesObject, interval, alpha, &(interval->z), &(interval->y), qTry, &pTry );
				break;

			default:
				qpDUNES_printError(qpData, __FILE__, __LINE__,	"Stage QP solver undefined! Bailing out...");
				return QPDUNES_ERR_INVALID_ARGUMENT;
		}
//		addVectorScaledVector( &(interval->z), &(interval->qpSolverClipping.zUnconstrained), alpha, &(interval->qpSolverClipping.dz), interval->nV );
//		directQpSolver_saturateVector( qpData, &(interval->z), &(interval->y), &(interval->dLow), &(interval->dUpp), interval->nV );

		/* quadratic objective part */
		interval->optObjVal = 0.5 * multiplyzHz(qpData, &(interval->H), &(interval->z),	interval->nV);
		/* linear objective part */
//		addVectorScaledVector( qTry, &(interval->q), alpha, &(interval->qpSolverClipping.qStep), interval->nV );
		interval->optObjVal += scalarProd(qTry, &(interval->z), interval->nV);
		/* constant objective part */
//		interval->optObjVal += interval->p + (alpha) * interval->qpSolverClipping.pStep;
		interval->optObjVal += pTry;

		objVal += interval->optObjVal;
	}

	return objVal;
}
/*<<< END OF qpDUNES_computeObjectiveValue */




/* ----------------------------------------------
 * Get number of active local constraints
 *
 *   Note: this overwrites actSetStatus
 *
 >>>>>>                                           */
return_t qpDUNES_getActSetChanges(	const qpData_t* const qpData,
									uint_t * const nActConstr,
									uint_t * const nChgdConstr
									)
{
	uint_t kk, ii;
	int_t nChgdConstr_kk;

	boolean_t isBoundActive, wasBoundActive;

	interval_t* interval;

	*nActConstr = 0;
	*nChgdConstr = 0;

	for (kk = 0; kk < _NI_ + 1; ++kk) {
		interval = qpData->intervals[kk];
		switch (interval->qpSolverSpecification)
		{
			case QPDUNES_STAGE_QP_SOLVER_CLIPPING:
				interval->rebuildHessianBlock = QPDUNES_FALSE;
				for (ii = 0; ii < _NV(kk); ++ii ) {
					/* NOTE: if bounds are flipping, (for example y.lb act => y.ub act)
					 *       this is not counted as active set change here, since the
					 *       effect on the Hessian blocks is identical, no matter which
					 *       bound is active. Still, we do reach a new region (and the
					 *       dual gradient hanges), but just with an identical Hessian. */
					isBoundActive = (boolean_t) ( (interval->y.data[2 * ii] * interval->y.data[2 * ii + 1]) < 0 );	/* y.lb < 0  &&  y.ub < 0  => inactive */
					wasBoundActive = (boolean_t) ( (interval->yPrev.data[2 * ii] * interval->yPrev.data[2 * ii + 1]) < 0 );
					if ( isBoundActive != wasBoundActive )	/* AS change if sign is different */
					{
						++(*nChgdConstr);
						interval->rebuildHessianBlock = QPDUNES_TRUE;
						#ifdef __DEBUG__
						if (qpData->options.printLevel >= 4) {
							qpDUNES_printf( "AS change in [%d,%d]: %+d => %+d", kk, ii,wasBoundActive, isBoundActive );
						}
						#endif
					}
					/* log number of active constraints */
					*nActConstr += isBoundActive;
				}
				break;

			case QPDUNES_STAGE_QP_SOLVER_QPOASES:
//				qpDUNES_printf("intervalID: %d", interval->id);
//				qpDUNES_printf("nActConst was %d, is %d", *nActConstr, qpOASES_getNbrActConstr( &(interval->qpSolverQpoases) ) );
				*nActConstr += qpOASES_getNbrActConstr( interval->qpSolverQpoases.qpoasesObject );
				nChgdConstr_kk = interval->parametricObjFctn_nBasePoints - 2;
//				qpDUNES_printf("nbr changed on %d is %d", kk, nChgdConstr_kk );
				if ( nChgdConstr_kk > 0 )	{
					*nChgdConstr += nChgdConstr_kk;
					interval->rebuildHessianBlock = QPDUNES_TRUE;
				}
				else {
					interval->rebuildHessianBlock = QPDUNES_FALSE;
				}
				break;

			default:
				qpDUNES_printError( qpData, __FILE__, __LINE__, "Stage QP solver undefined! Bailing out..." );
				return QPDUNES_ERR_INVALID_ARGUMENT;
		}
	}

	return QPDUNES_OK;
}



/* ----------------------------------------------
 * ...
 *
 >>>>>                                            */
void qpDUNES_printIterationHeader(	qpData_t* qpData
									)
{
	if (qpData->options.printLevel >= 2) {
		qpDUNES_printf(
				"\n iter |     gradNorm |     stepNorm |    stepSize | LS iter | reg. NHess |    obj. Val |   #conAct |   #chgAS |  lambdaNorm");
		qpDUNES_printf(
				" ---- | ------------ | ------------ | ----------- | ------- | ---------- | ----------- | --------- | -------- | -----------");
	}
}
/*<<< END OF qpDUNES_printIterationHeader */


/* ----------------------------------------------
 * ...
 *
 >>>>>>                                           */
void qpDUNES_printIteration(	qpData_t* qpData,
								itLog_t* itLogPtr
								)
{
	/* TODO: correct printing! Right now gradNorm gets overwritten in line search, is not the one used for Newton's method */

	if (qpData->options.printLevel >= 2) {
		/*            iter | gradNorm| stepNorm| stepLen | LS iter |   regNH? | objVal |   nAct |  nChg | multiplierNorm  */
		qpDUNES_printf(
				" %4d |    %.3e |    %.3e |    %.2e |     %3d |      %5s |  % .3e |    %6d |   %6d |   %.3e ",
				itLogPtr->itNbr, itLogPtr->gradNorm, itLogPtr->stepNorm,
				itLogPtr->stepSize, itLogPtr->numLineSearchIter,
				(itLogPtr->hessRefactorIdx >= 0) ? ( itLogPtr->isHessianRegularized ? "true" : "false" ) : "n/a",
				itLogPtr->objVal, itLogPtr->nActConstr, itLogPtr->nChgdConstr,
				itLogPtr->lambdaNorm);
	}

}
/*<<< END OF qpDUNES_printIteration */

/*
 *	end of file
 */

