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
 *	\file src/stage_qp_solver_clipping.c
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2012
 */


#include "qp/stage_qp_solver_clipping.h"

/* ----------------------------------------------
 * solve QP
 * 
#>>>>>>                                           */
//return_t qp42_directQpSolver_solve( qpData_t* const qpData,
//									 interval_t* const interval )
//{
//	int_t ii;
//	return_t statusFlag;
//
//	int_t nV = interval->nV;	/* number of QP variables */
//
//	assert( interval->nD == nV );	/* assert that vector dimensions match */
//
//	for( ii = 0; ii < nV; ++ii ) {
//		if( interval->dLow.data[ii] - interval->dUpp.data[ii] > qpData->options.equalityTolerance ) {
//			qp42_printError( qpData, __FILE__, __LINE__, "Local QP infeasible!" );
//			return QPDUNES_ERR_SUB_QP_INFEASIBLE;
//		}
//	}
//
//	/* solve unconstraint QP */
//	qp42_directQpSolver_solveUnconstrained( qpData, interval );
//
//	/* saturate variables at simple bounds */
//	statusFlag = qp42_directQpSolver_saturateVector( qpData, &(interval->dz), &(interval->y), &(interval->dLow), &(interval->dUpp), nV );
//	assert( statusFlag == QPDUNES_OK );
//
////	/* get objective function value:  0.5*zOpt.T*H*zOpt + g.T*zOpt */
////	interval->optObjVal = scalarProd( &(interval->gMod), &(interval->dz), nV );
////	multiplyHz( qpData, &(interval->zVecTmp), &(interval->H), &(interval->dz) );
////	interval->optObjVal += 0.5 * scalarProd( &(interval->zVecTmp), &(interval->dz), nV );
//
//	return QPDUNES_OK;
//}
///*<<< END OF qp42_solveLocalQP */


/* ----------------------------------------------
 * update QP data
 *
 * qStep = C.T*lambdaK1 - [lambdaK.T 0]
 * pStep = c*lambdaK1
#>>>>>>                                           */
return_t clippingQpSolver_updateDualGuess(	qpData_t* const qpData,
											interval_t* const interval,
											const z_vector_t* const lambdaK,
											const z_vector_t* const lambdaK1
											)
{
	int_t ii;

	/* WARNING: working with qStep might lead to high cancellation errors (due to subtraction, result might be almost zero)
	 * but we can hope for this to appear only when  lambda steps are identical over iterations (-> infeasible problem, see journal paper).
	 * Always recomputing q is less efficient.
	 */

	if (lambdaK1->isDefined == QPDUNES_TRUE) {
		/* qStep = C.T*lambdaK1 */
		multiplyCTy( qpData, &(interval->qpSolverClipping.qStep), &(interval->C), lambdaK1 );
		/* pStep = c*lambdaK1 */
		interval->qpSolverClipping.pStep = scalarProd( lambdaK1, &(interval->c), _NX_ );	/* constant objective term */
	}
	else {
		/* qStep = 0 */
		qpDUNES_setupZeroVector( &(interval->qpSolverClipping.qStep), interval->nV );
		/* pStep = 0 */
		interval->qpSolverClipping.pStep = 0.;	/* constant objective term */
	}

	if (lambdaK->isDefined == QPDUNES_TRUE) {
		/* qStep -= [lambdaK.T 0]	*/
		for ( ii=0; ii<_NX_; ++ii ) {
			interval->qpSolverClipping.qStep.data[ii] -= lambdaK->data[ii];
		}
	}


	return QPDUNES_OK;
}
/*<<< END OF qp42_clippingQpSolver_updateStageData */



/* ----------------------------------------------
 * solve unconstrained QP
 *
#>>>>>>                                           */
// TODO: rename to clippingQpSolver
return_t directQpSolver_solveUnconstrained( qpData_t* const qpData,
												 interval_t* const interval,
												 const z_vector_t* const qStep )
{
	return_t statusFlag;

	/* solve unconstraint QP */
	statusFlag = multiplyInvHz( qpData, &(interval->qpSolverClipping.dz), &(interval->cholH), qStep, interval->nV );
	negateVector( &(interval->qpSolverClipping.dz), interval->nV );

//	qpDUNES_printMatrixData(interval->qpSolverClipping.dz.data, 1, interval->nV, "dz" );
//	qp42_printMatrixData(interval->cholH.data, interval->nV, interval->nV, "cholH" );
//	qpDUNES_printMatrixData(interval->H.data, interval->nV, interval->nV, "H" );
//	qp42_printf( "cholH: %d", interval->cholH.sparsityType );
//	qpDUNES_printMatrixData(qStep->data, 1, interval->nV, "qStep" );

	return statusFlag;
}
/*<<< END OF qp42_directQpSolver_solveUnconstrained */


/* ----------------------------------------------
 * gets the step size to the first active set change
 * if it is shorter than an incumbent step size
 * initially in alphaMin
 *
 * compare gaps from z (i.e., multipliers from previous iteration) with dz
 *
#>>>>>>                                           */
return_t clippingQpSolver_getMinStepsize( 	const qpData_t* const qpData,
											const interval_t* const interval,
											real_t* alphaMin )
{
	int ii;
	real_t lbRatio, ubRatio;
	real_t alphaASChange;

	for( ii=0; ii<interval->nV; ++ii ) {
		lbRatio = interval->y.data[2*ii] / interval->qpSolverClipping.dz.data[ii];
		ubRatio = - interval->y.data[2*ii+1] / interval->qpSolverClipping.dz.data[ii];
		alphaASChange = qpDUNES_fmin( (lbRatio >= 0.) ? lbRatio : qpData->options.QPDUNES_INFTY ,
									  (ubRatio >= 0.) ? ubRatio : qpData->options.QPDUNES_INFTY );
//		/* WARNING: compiler support for 1./0. == inf, and (2. < inf) == TRUE are assumed */
//		alphaASChange = 1./
//				qpDUNES_fmax( interval->qpSolverClipping.dz.data[ii] / interval->y.data[2*ii] ,
//						      interval->qpSolverClipping.dz.data[ii] / - interval->y.data[2*ii+1] );
		if ( (alphaASChange >= 0. ) && (alphaASChange < *alphaMin) ) {
			*alphaMin = alphaASChange;
		}
	}

	return QPDUNES_OK;
}
/*<<< END OF clippingQpSolver_getMinStepsize */


/* ----------------------------------------------
 * do a step of length alpha
 *
#>>>>>>                                           */
return_t directQpSolver_doStep( qpData_t* const qpData,
								interval_t* const interval,
								const z_vector_t* const stepDir,
								real_t alpha,
								z_vector_t* const zUnconstrained,
								z_vector_t* const z,
								d2_vector_t* const mu,
								z_vector_t* const q,
								real_t* const p				)
{
	int ii;

	/* update primal solution and get dual solution */
	addVectorScaledVector( zUnconstrained, &(interval->qpSolverClipping.zUnconstrained), alpha, stepDir, interval->nV );
	if ( z != zUnconstrained ) {	/* skip copying if zUnconstrained and z are pointing to the same object (e.g., during line search, when just trying steps) */
//		directQpSolver_saturateVector( qpData, z, mu, &(interval->zLow), &(interval->zUpp), interval->nV );
//	}
//	else {
		qpDUNES_copyVector( z, zUnconstrained, interval->nV );
	}
	directQpSolver_saturateVector( qpData, z, mu, &(interval->zLow), &(interval->zUpp), &(interval->H), interval->nV );

	/* update q */
	for ( ii=0; ii<interval->nV; ++ii ) {
		q->data[ii] = interval->q.data[ii] + alpha * interval->qpSolverClipping.qStep.data[ii];
	}

	/* update p */
	*p = interval->p + alpha * interval->qpSolverClipping.pStep;

	return QPDUNES_OK;
}
/*<<< END OF directQpSolver_doStep */


/* ----------------------------------------------
 * ...
 * 
#>>>>>>                                           */
/* TODO: pass less arguments */
return_t directQpSolver_saturateVector(	qpData_t* const qpData,
										d_vector_t* const vec,
										d2_vector_t* const mu,		/* multipliers, resembling the gaps to the bounds; + active, - inactive */
										const d_vector_t* const lb,
										const d_vector_t* const ub,
										const zz_matrix_t* const H,
										int_t nV
										)
{
	int_t ii;
	
	switch (H->sparsityType)	{
		case QPDUNES_DIAGONAL:		/* H is saved in first row of memory */
			for( ii=0; ii<nV; ++ii ) {
				/* for box constraints and diagonal hessians it holds: lambda_i/H_ii = (zUnconstr_i - zBound_i)	*/
				mu->data[2*ii] = (lb->data[ii] - vec->data[ii])*H->data[ii];	/* feasibility gap to lower bound; negative value means inactive */
				mu->data[2*ii+1] = (vec->data[ii] - ub->data[ii])*H->data[ii];	/* feasibility gap to upper bound; negative value means inactive */
				if ( mu->data[2*ii] >= -qpData->options.activenessTolerance ) {	/* TODO: check if this implementation of activeness Tolerance makes sense */
					vec->data[ii] = lb->data[ii];
				}
				else {
					if ( mu->data[2*ii+1] >= -qpData->options.activenessTolerance ) {
						vec->data[ii] = ub->data[ii];
					}
				}
			}
			break;

		case QPDUNES_IDENTITY:		/* H values not needed */
			for( ii=0; ii<nV; ++ii ) {
				/* for box constraints and diagonal hessians it holds: lambda_i/H_ii = (zUnconstr_i - zBound_i)	*/
				mu->data[2*ii] = (lb->data[ii] - vec->data[ii]);		/* feasibility gap to lower bound; negative value means inactive */
				mu->data[2*ii+1] = (vec->data[ii] - ub->data[ii]);		/* feasibility gap to upper bound; negative value means inactive */
				if ( mu->data[2*ii] >= -qpData->options.activenessTolerance ) {	/* TODO: check if this implementation of activeness Tolerance makes sense */
					vec->data[ii] = lb->data[ii];
				}
				else {
					if ( mu->data[2*ii+1] >= -qpData->options.activenessTolerance ) {
						vec->data[ii] = ub->data[ii];
					}
				}
			}
			break;

		default:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Unknown sparsity type of QP hessian" );
			return QPDUNES_ERR_UNKNOWN_MATRIX_SPARSITY_TYPE;
	}
//		if ( interval->y.data[2*ii] > qpData->options.equalityTolerance ) { /* lower bound active */
//			actSetStatus[kk][ii] = -1;
//			++(*nActConstr);
//		}
//		else {
//			if ( interval->y.data[2*ii+1] > qpData->options.equalityTolerance ) { /* upper bound active */
//				actSetStatus[kk][ii] = 1;
//				++(*nActConstr);
//			}
//			else {		/* no constraint bound active */
//				actSetStatus[kk][ii] = 0;
//			}
//		}
	


	return QPDUNES_OK;
}
/*<<< END OF directQpSolver_saturateVector */


/* ----------------------------------------------
 * ...
 *
#>>>>>>                                           */
return_t clippingQpSolver_ratioTest(	qpData_t* const qpData,
										real_t* minStepSizeASChange,	/* minimum step size that leads to active set change */
										d_vector_t* const zStepDir,
										d2_vector_t* const mu,			/* pseudo multipliers, resembling the gaps to the bounds; + active, - inactive */
										const d_vector_t* const lb,
										const d_vector_t* const ub,
										int_t nV
										)
{
	int_t ii;

	real_t stepRatio;

	*minStepSizeASChange = qpData->options.QPDUNES_INFTY;	/* minimum step size that leads to active set change */

	for( ii=0; ii<nV; ++ii ) {
		/* check ratio distance to lower bound and step direction */
		stepRatio = mu->data[2*ii] / zStepDir->data[ii];
		if ( ( stepRatio >= 0.0 ) && ( stepRatio < *minStepSizeASChange ) ) {
			if ( ( mu->data[2*ii] < qpData->options.equalityTolerance ) &&			/* treat lb inactive and -infty separately */
				 ( lb->data[ii] < qpData->options.QPDUNES_INFTY * (-1 + qpData->options.equalityTolerance) ) )
			{
				*minStepSizeASChange = stepRatio;
//				qpDUNES_printf("dist to lb: %.3e", mu->data[2*ii]);
//				qpDUNES_printf("step dir: %.3e", zStepDir->data[ii]);
			}
		}

		/* check ratio distance to upper bound and step direction */
		stepRatio = - mu->data[2*ii+1] / zStepDir->data[ii];
		if ( ( stepRatio >= 0.0 ) && ( stepRatio < *minStepSizeASChange ) ) {
			if ( ( mu->data[2*ii+1] < qpData->options.equalityTolerance ) &&		/* treat ub inactive and +infty separately */
				 ( ub->data[ii] < qpData->options.QPDUNES_INFTY * (1 - qpData->options.equalityTolerance) ) )
			{
				*minStepSizeASChange = stepRatio;
//				qpDUNES_printf("dist to ub: %.3e", mu->data[2*ii+1]);
//				qpDUNES_printf("step dir: %.3e", zStepDir->data[ii]);
			}
		}
	}


//	qpDUNES_printf("min step size ratio: %.3e", *minStepSizeASChange);


	return QPDUNES_OK;
}
/*<<< END OF directQpSolver_ratioTest */



/* ----------------------------------------------
 * ...
 *
#>>>>>>                                           */
real_t directQpSolver_getObjectiveValue(	qpData_t* const qpData,
												interval_t* const interval
												)
{
	real_t objVal;

	/* quadratic part */
	objVal = 0.5 * multiplyzHz( qpData, &(interval->H), &(interval->z), interval->nV );
	/* linear part */
	objVal += scalarProd( &(interval->q), &(interval->z), interval->nV );
	/* constant part */
	objVal += interval->p;

	return objVal;
}
/*<<< END OF qp42_directQpSolver_saturate */


/*
 *	end of file
 */
