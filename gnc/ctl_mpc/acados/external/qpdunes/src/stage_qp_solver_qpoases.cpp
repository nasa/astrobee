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
 *	License along with qp42; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file src/stage_qp_solver_qpoases.c
 *	\author Janick Frasch
 *	\version 1.0beta
 *	\date 2012
 */


#include "qp/stage_qp_solver_qpoases.hpp"
#include <qpOASES.hpp>
#include <stdlib.h>


extern "C" {

/* ----------------------------------------------
 * own qpOASES memory allocation routine
 *
#>>>>>>                                           */
qpoasesObject_t* qpOASES_constructor( qpData_t* qpData,
									 int_t nV,
							 	 	 int_t nD )
{
	/* set up new qpOASES object */
	qpoasesObject_t* qpoasesObject = (qpoasesObject_t*)calloc( 1,sizeof(qpoasesObject_t) );
	qpoasesObject->qpoases = static_cast<qpoases_t*>( new qpOASES::LoggedSQProblem( nV, nD ) );

	/* set some options */
	qpoasesObject->options = static_cast<qpoasesOptions_t*>( new qpOASES::Options() );
	static_cast<qpOASES::Options*>( qpoasesObject->options )->setToFast();
	if ( qpData->options.printLevel < 2 ) {
		static_cast<qpOASES::Options*>( qpoasesObject->options )->printLevel = qpOASES::PL_NONE;
	}
	else {
		if ( qpData->options.printLevel <= 3 ) {
			static_cast<qpOASES::Options*>( qpoasesObject->options )->printLevel = qpOASES::PL_LOW;
		}
		else {	/* PL > 3 */
			static_cast<qpOASES::Options*>( qpoasesObject->options )->printLevel = qpOASES::PL_MEDIUM;
		}
	}

//	static_cast<qpOASES::Options*>( qpoasesObject->options )->printLevel = qpOASES::PL_HIGH;	/* qpOASES uses global variables for message handlung */

	static_cast<qpOASES::Options*>( qpoasesObject->options )->terminationTolerance = qpData->options.qpOASES_terminationTolerance;
	static_cast<qpOASES::Options*>( qpoasesObject->options )->initialStatusBounds = qpOASES::ST_INACTIVE;
	static_cast<qpOASES::Options*>( qpoasesObject->options )->enableFarBounds = qpOASES::BT_FALSE;
	static_cast<qpOASES::Options*>( qpoasesObject->options )->enableRegularisation = qpOASES::BT_FALSE;
	static_cast<qpOASES::LoggedSQProblem*>( qpoasesObject->qpoases )->setOptions( *( static_cast<qpOASES::Options*>( qpoasesObject->options ) ) );

	#ifdef __DEBUG__
	if (qpData->options.printLevel > 3)	{
		static_cast<qpOASES::Options*>( qpoasesObject->options )->print();
	}
	#endif

	/* return pointer to qpOAES object */
	return qpoasesObject;
}
/*<<< END OF qpOASES_contstuctor */


/* ----------------------------------------------
 * solve QP
 *
#>>>>>>                                           */
void qpOASES_destructor( qpoasesObject_t** qpoasesObject )
{
	if (*qpoasesObject) {
		delete static_cast<qpOASES::LoggedSQProblem*>( (*qpoasesObject)->qpoases );
		delete static_cast<qpOASES::Options*>( (*qpoasesObject)->options );
		free( *qpoasesObject );
	}
	*qpoasesObject = 0;
	return;
}
/*<<< END OF qpOASES_destructor */


/* ----------------------------------------------
 * first QP solution
 *
#>>>>>>                                           */
return_t qpOASES_setup( qpData_t* qpData,
						qpoasesObject_t* qpoasesObject,
						interval_t* interval
						)
{
	uint_t ii;

	int_t nWSR = qpData->options.maxNumQpoasesIterations;		/* number of working set recalculations;
															 	 * input: maximum permitted
															 	 * output: iterations actually needed */
	qpOASES::returnValue qpOASES_statusFlag;

	/* make matrix data dense */
	qpDUNES_makeMatrixDense( &(interval->H), interval->nV, interval->nV );
	qpDUNES_makeMatrixDense( &(interval->D), interval->nD, interval->nV );

//	qpDUNES_printMatrixData( interval->D.data, interval->nD, interval->nV, "I am qpoases setup, D[%d] = ", interval->id );
//	qpDUNES_printMatrixData( interval->dLow.data, interval->nD, 1, "I am qpoases setup, dLow[%d] = ", interval->id );
//	qpDUNES_printMatrixData( interval->dUpp.data, interval->nD, 1, "I am qpoases setup, dUpp[%d] = ", interval->id );
//	if (interval->id == 0) {
//	qpDUNES_printf("pointer check (H): %ld", (long int)(interval->H.data) );
//	qpDUNES_printMatrixData( interval->H.data, interval->nV, interval->nV, "H: (setup---)" );
//	qpDUNES_printMatrixData( interval->qpSolverQpoases.qFullStep.data, interval->nV, 1, "g (id = %d):", interval->id );
//	qpDUNES_printMatrixData( interval->zLow.data, interval->nV, 1, "zLow:" );
//	qpDUNES_printMatrixData( interval->zUpp.data, interval->nV, 1, "zUpp:" );
//	if (interval->D.data)  qpDUNES_printMatrixData( interval->D.data, interval->nD, interval->nV, "D:" );
//	if (interval->dLow.data)  qpDUNES_printMatrixData( interval->dLow.data, interval->nD, 1, "dLow:" );
//	if (interval->dUpp.data)  qpDUNES_printMatrixData( interval->dUpp.data, interval->nD, 1, "dUpp:" );
//	}

	qpOASES_statusFlag = static_cast<qpOASES::LoggedSQProblem*>(qpoasesObject->qpoases)->init( 	interval->H.data,
																					interval->qpSolverQpoases.qFullStep.data,
																					interval->D.data,
																					interval->zLow.data, interval->zUpp.data,
																					interval->dLow.data, interval->dUpp.data,
																					nWSR
																					);
	switch ( qpOASES_statusFlag )
	{
		case qpOASES::SUCCESSFUL_RETURN:
			break;

		case qpOASES::RET_INIT_FAILED_INFEASIBILITY:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "qpOASES subproblem on stage %d infeasible.", interval->id );
			return QPDUNES_ERR_STAGE_QP_INFEASIBLE;

		case qpOASES::RET_MAX_NWSR_REACHED:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Maximum number of permitted qpOASES iterations reached on stage %d.", interval->id );
			return QPDUNES_ERR_ITERATION_LIMIT_REACHED;

		default:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "qpOASES init failed on stage %d for unexpected reason (qpOASES return code %d).", interval->id, (int_t)qpOASES_statusFlag );
			return QPDUNES_ERR_UNKNOWN_ERROR;
	}

	/* get primal and dual solution */
	static_cast<qpOASES::LoggedSQProblem*>(qpoasesObject->qpoases)->getPrimalSolution( static_cast<qpOASES::real_t*>(interval->z.data) );
	qpOASES_getDualSol( qpData, interval, qpoasesObject, &(interval->y) );

	//	qpDUNES_printMatrixData( mu->data, 2*interval->nV+2*interval->nD, 1, "qpoases multipliers[%d]", interval->id );

	/* update stage data (q, p) */
	for ( ii=0; ii<interval->nV; ++ii ) {
		interval->q.data[ii] = interval->qpSolverQpoases.qFullStep.data[ii];
	}
	interval->p = interval->qpSolverQpoases.pFullStep;

	return QPDUNES_OK;
}
/*<<< END OF qpOASES_setup */



/* ----------------------------------------------
 * major QP data upadate (note that qpOASES
 * immediate resolves the QP here)
 *
#>>>>>>                                           */
return_t qpOASES_dataUpdate( qpData_t* const qpData,
							qpoasesObject_t* const qpoasesObject,
							interval_t* const interval,
							boolean_t H_changed,
							boolean_t zLow_changed,
							boolean_t zUpp_changed,
							boolean_t D_changed,
							boolean_t dLow_changed,
							boolean_t dUpp_changed,
							int_t* nQpoasesIter
							)
{
	uint_t ii;

	*nQpoasesIter = qpData->options.maxNumQpoasesIterations;		/* number of working set recalculations;
															 	 	 * input: maximum permitted
															 	 	 * output: iterations actually needed */
	qpOASES::returnValue qpOASES_statusFlag;

//	if ( (interval->id == 0) || (interval->id == 99) )
//	if ( (interval->id == 0) || (interval->id == _NI_-1) )
//	{
//	qpDUNES_printf("pointer check (H): %ld", (long int)(interval->H.data) );
//	qpDUNES_printMatrixData( interval->H.data, interval->nV, interval->nV, "H:" );
//	qpDUNES_printMatrixData( interval->qpSolverQpoases.qFullStep.data, interval->nV, 1, "g (id = %d):", interval->id );
//	qpDUNES_printMatrixData( interval->zLow.data, interval->nV, 1, "zLow:" );
//	qpDUNES_printMatrixData( interval->zUpp.data, interval->nV, 1, "zUpp:" );
//	if (interval->D.data)  qpDUNES_printMatrixData( interval->D.data, interval->nD, interval->nV, "D:" );
//	if (interval->dLow.data)  qpDUNES_printMatrixData( interval->dLow.data, interval->nD, 1, "dLow:" );
//	if (interval->dUpp.data)  qpDUNES_printMatrixData( interval->dUpp.data, interval->nD, 1, "dUpp:" );
//	}

//	if ( 0 ) {
	if ( (H_changed == QPDUNES_TRUE) || (D_changed == QPDUNES_TRUE) ) {		/** qpOASES distinguishes between matrix-free and matrix updates */
//		if (interval->id == 0)
//			qpDUNES_printMatrixData( interval->z.data, 1, interval->nV, "i[%3d]: z, before Oases:", interval->id);
//		qpDUNES_printMatrixData( interval->H.data, interval->nV, interval->nV, "H:" );
//		if (interval->id == 0) 	qpDUNES_printMatrixData( interval->zLow.data, 1, interval->nV, "i[%3d]: zLowBeforeOases:", interval->id);
//		if (interval->id == 0) 	qpDUNES_printMatrixData( interval->zUpp.data, 1, interval->nV, "i[%3d]: zUppBeforeOases:", interval->id);
		qpOASES_statusFlag
//			= static_cast<qpOASES::SQProblem*>(qpoasesObject)->hotstart(	interval->H.data,
//																			interval->qpSolverQpoases.qFullStep.data,
//																			interval->D.data,
//																			(zLow_changed == QPDUNES_TRUE)? interval->zLow.data : 0,
//																			(zUpp_changed == QPDUNES_TRUE)? interval->zUpp.data : 0,
//																			(dLow_changed == QPDUNES_TRUE)? interval->dLow.data : 0,
//																			(dUpp_changed == QPDUNES_TRUE)? interval->dUpp.data : 0,
//																			nWSR
//																			);
			= static_cast<qpOASES::LoggedSQProblem*>(qpoasesObject->qpoases)->hotstart(	interval->H.data,
																			interval->qpSolverQpoases.qFullStep.data,
																			interval->D.data,
																			interval->zLow.data,
																			interval->zUpp.data,
																			interval->dLow.data,
																			interval->dUpp.data,
																			*nQpoasesIter
																			);
//		if (interval->id == 0) {
//			qpDUNES_printMatrixData( interval->H.data, interval->nV, interval->nV, "D = ");
//			qpDUNES_printMatrixData( interval->qpSolverQpoases.qFullStep.data, 1, interval->nV, "g = ");
//			qpDUNES_printMatrixData( interval->zLow.data, 1, interval->nV, "zLow = ");
//			qpDUNES_printMatrixData( interval->zUpp.data, 1, interval->nV, "zUpp = ");
//			qpDUNES_printMatrixData( interval->D.data, interval->nD, interval->nV, "D = ");
//			qpDUNES_printMatrixData( interval->dLow.data, 1, interval->nD, "dLow = ");
//			qpDUNES_printMatrixData( interval->dUpp.data, 1, interval->nD, "dUpp = ");
//		}
//		if (interval->id == 0) 	qpDUNES_printMatrixData( interval->zLow.data, 1, interval->nV, "i[%3d]: zLowAfterOases:", interval->id);
//		if (interval->id == 0) 	qpDUNES_printMatrixData( interval->zUpp.data, 1, interval->nV, "i[%3d]: zUppAfterOases:", interval->id);
		static_cast<qpOASES::LoggedSQProblem*>(qpoasesObject->qpoases)->getPrimalSolution( static_cast<qpOASES::real_t*>(interval->z.data) );
//		if (interval->id == 0) 	qpDUNES_printMatrixData( interval->z.data, 1, interval->nV, "i[%3d]: z@AfterOases:", interval->id);
	}
	else {
//		qpOASES_statusFlag
//			= static_cast<qpOASES::SQProblem*>(qpoasesObject)->hotstart( 	interval->qpSolverQpoases.qFullStep.data,
//																			(zLow_changed == QPDUNES_TRUE)? interval->zLow.data : 0,
//																			(zUpp_changed == QPDUNES_TRUE)? interval->zUpp.data : 0,
//																			(dLow_changed == QPDUNES_TRUE)? interval->dLow.data : 0,
//																			(dUpp_changed == QPDUNES_TRUE)? interval->dUpp.data : 0,
//																			nWSR );
		qpOASES_statusFlag
			= static_cast<qpOASES::LoggedSQProblem*>(qpoasesObject->qpoases)->hotstart( 	interval->qpSolverQpoases.qFullStep.data,
																			interval->zLow.data,
																			interval->zUpp.data,
																			interval->dLow.data,
																			interval->dUpp.data,
																			*nQpoasesIter
																			);
	}
	switch ( qpOASES_statusFlag )
	{
		case qpOASES::SUCCESSFUL_RETURN:
			break;

		case qpOASES::RET_HOTSTART_STOPPED_INFEASIBILITY:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "qpOASES subproblem on stage %d infeasible.", interval->id );
			return QPDUNES_ERR_STAGE_QP_INFEASIBLE;

		case qpOASES::RET_MAX_NWSR_REACHED:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Maximum number of permitted qpOASES iterations reached on stage %d.", interval->id );
			return QPDUNES_ERR_ITERATION_LIMIT_REACHED;

		default:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "qpOASES update failed on stage %d for unexpected reason (qpOASES return code %d).", interval->id, (int_t)qpOASES_statusFlag );
			return QPDUNES_ERR_UNKNOWN_ERROR;
	}


	/* get primal and dual solution */
	static_cast<qpOASES::LoggedSQProblem*>(qpoasesObject->qpoases)->getPrimalSolution( static_cast<qpOASES::real_t*>(interval->z.data) );
	qpOASES_getDualSol( qpData, interval, qpoasesObject, &(interval->y) );

	//	qpDUNES_printMatrixData( mu->data, 2*interval->nV+2*interval->nD, 1, "qpoases multipliers[%d]", interval->id );
//	if (interval->id == 0)
//		qpDUNES_printMatrixData( interval->z.data, 1, interval->nV, "i[%3d]: z, after Oases:", interval->id);

	/* update stage data (q, p) */
	for ( ii=0; ii<interval->nV; ++ii ) {
		interval->q.data[ii] = interval->qpSolverQpoases.qFullStep.data[ii];
	}
	interval->p = interval->qpSolverQpoases.pFullStep;

	return QPDUNES_OK;
}
/*<<< END OF qpOASES_dataUpdate */



/* ----------------------------------------------
 * update QP data
 *
 * qStep = C.T*lambdaK1 - [lambdaK.T 0]
 * pStep = c*lambdaK1
#>>>>>>                                           */
return_t qpOASES_updateDualGuess(	qpData_t* const qpData,
									interval_t* const interval,
									const z_vector_t* const lambdaK,
									const z_vector_t* const lambdaK1
									)
{
//#ifndef __MATLAB__
	uint_t ii;

	if (lambdaK1->isDefined == QPDUNES_TRUE) {
		/* qFullStep = q + C.T*lambdaK1 */
		multiplyCTy( qpData, &(interval->qpSolverQpoases.qFullStep), &(interval->C), lambdaK1 );
		addToVector( &(interval->qpSolverQpoases.qFullStep), &(interval->q), interval->nV );
		/* pStep = c*lambdaK1 */
		interval->qpSolverQpoases.pFullStep = scalarProd( lambdaK1, &(interval->c), _NX_ );	/* constant objective term */
	}
	else {
		/* qFullStep = q */
		qpDUNES_copyVector( &(interval->qpSolverQpoases.qFullStep), &(interval->q), interval->nV );
		/* pStep = 0 */
		interval->qpSolverQpoases.pFullStep = 0.;	/* constant objective term */
	}
	if (lambdaK->isDefined == QPDUNES_TRUE) {
		/* qFullStep -= [lambdaK.T 0]	*/
		for ( ii=0; ii<_NX_; ++ii ) {
			interval->qpSolverQpoases.qFullStep.data[ii] -= lambdaK->data[ii];
		}
	}

	return QPDUNES_OK;
//#endif
}
/*<<< END OF qpOASES_updateStageData */


/* ----------------------------------------------
 * Hotstarted QP solution for updated gradient
 *
#>>>>>>                                           */
return_t qpOASES_hotstart( 	qpData_t* qpData,
							qpoasesObject_t* qpoasesObject,
							interval_t* interval,
							z_vector_t* q,
							int_t* const numQpoasesIter,
							boolean_t logHomotopy
							)
{
//#ifndef __MATLAB__
	int_t ii;
	int_t nWSR = qpData->options.maxNumQpoasesIterations;		/* number of working set recalculations;
																 * input: maximum permitted
																 * output: iterations actually needed */
	qpOASES::returnValue qpOASES_statusFlag;

//	real_t tmpOldObjVal = static_cast<qpOASES::SQProblem*>(qpoasesObject->qpoases)->getObjVal();

//	if (interval->id == 0)
//	if ( (interval->id == 0) || (interval->id == _NI_-1) )
//	{
//		qpDUNES_printf("pointer check (H): %ld", (long int)(interval->H.data) );
//		qpDUNES_printMatrixData( interval->qpSolverQpoases.qFullStep.data, interval->nV, 1, "g (id = %d):", interval->id );
//	}

	if ( logHomotopy ) 	{
		qpOASES_statusFlag
			= static_cast<qpOASES::LoggedSQProblem*>(qpoasesObject->qpoases)->hotstart_withHomotopyLogging( 	q->data,
																		interval->zLow.data,
																		interval->zUpp.data,
																		interval->dLow.data,
																		interval->dUpp.data,
																		interval->parametricObjFctn_alpha.data,
																		interval->parametricObjFctn_f.data,
																		interval->parametricObjFctn_fPrime.data,
																		interval->parametricObjFctn_fPrimePrime.data,
																		nWSR );	/* todo: avoid updating zLow, etc. from interval data! */
	}
	else	{
//		if (interval->id == 1) {
//			qpDUNES_printMatrixData( interval->H.data, interval->nV, interval->nV, "H = ");
//			qpDUNES_printMatrixData( q->data, 1, interval->nV, "g = ");
//			qpDUNES_printMatrixData( interval->zLow.data, 1, interval->nV, "zLow = ");
//			qpDUNES_printMatrixData( interval->zUpp.data, 1, interval->nV, "zUpp = ");
//			qpDUNES_printMatrixData( interval->D.data, interval->nD, interval->nV, "D = ");
//			qpDUNES_printMatrixData( interval->dLow.data, 1, interval->nD, "dLow = ");
//			qpDUNES_printMatrixData( interval->dUpp.data, 1, interval->nD, "dUpp = ");
//		}
		qpOASES_statusFlag
			= static_cast<qpOASES::LoggedSQProblem*>(qpoasesObject->qpoases)->hotstart( 	q->data,
																		interval->zLow.data,
																		interval->zUpp.data,
																		interval->dLow.data,
																		interval->dUpp.data,
																		nWSR );
	}
	// janick: qpOASES needs to check if data changed each time. keeping constraints does not seem to be supported...
//		= static_cast<qpOASES::SQProblem*>(qpoasesObject)->hotstart( 	q->data,
//																		0,
//																		0,
//																		0,
//																		0,
//																		nWSR );	/* todo: avoid updating zLow, etc. from interval data! */
	/* updating zLow cannot be avoided in first call to hotstart -> zLow,zUpp needed to include initial value constraint, added after initial homotopy. but can be done more efficiently by using shared memory */
	switch ( qpOASES_statusFlag )
	{
		case qpOASES::SUCCESSFUL_RETURN:
			break;

		case qpOASES::RET_HOTSTART_STOPPED_INFEASIBILITY:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "qpOASES subproblem on stage %d infeasible.", interval->id );
			return QPDUNES_ERR_STAGE_QP_INFEASIBLE;

		case qpOASES::RET_MAX_NWSR_REACHED:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Maximum number of permitted qpOASES iterations reached on stage %d.", interval->id );
			return QPDUNES_ERR_ITERATION_LIMIT_REACHED;

		default:
			qpDUNES_printError( qpData, __FILE__, __LINE__, "qpOASES hotstart failed on stage %d for unexpected reason (qpOASES return code %d).", interval->id, (int_t)qpOASES_statusFlag );
			return QPDUNES_ERR_UNKNOWN_ERROR;
	}


	/* log number of active set changes */
//	qpDUNES_printf("nWSR = %d", nWSR);
	if ( numQpoasesIter != 0 ) 	{
		*numQpoasesIter += nWSR;
	}

	if ( logHomotopy ) 	{
		/** save homotopy parameterization */
		interval->parametricObjFctn_nBasePoints = nWSR + 2;
		/* correct QP objective function value parameterization by constant term */
		for ( ii=0; ii <= nWSR; ++ii )	{
			interval->parametricObjFctn_f.data[ii] += interval->parametricObjFctn_alpha.data[ii] * interval->qpSolverQpoases.pFullStep;
			interval->parametricObjFctn_fPrime.data[ii] += interval->qpSolverQpoases.pFullStep;
		}

		/* SOME TESTS */
		#ifdef __DO_UNIT_TESTS__
		/* Unit test 1: check whether homotopy logging is consistent */
		qpDUNES_printf( "[qpDUNES] qpOASES_hotstart (homotopy logging) UNIT TEST:" );
		if ( numQpoasesIter != 0 ) 	{
			qpDUNES_printf( "          old obj val: %+.8e", tmpOldObjVal );
			for ( ii = 0; ii<= nWSR+1; ++ii ) {
				qpDUNES_printf( "          int%d-sec%d: alpha = % 8.12f *** f0 = %+.8e *** g0 = %+.8e *** h0 = %+.8e",
							interval->id,
							ii,
							interval->parametricObjFctn_alpha.data[ii],
							interval->parametricObjFctn_f.data[ii],
							interval->parametricObjFctn_fPrime.data[ii],
							interval->parametricObjFctn_fPrimePrime.data[ii] );
				if (ii < nWSR+1)	{
					/* check prediction */
					qpDUNES_printf( "          int%d-sec%d: PREDICTION CONSISTENCY CHECK: predicted: %+.8e ** logged reality: %+.8e",
							interval->id,
							ii,
							interval->parametricObjFctn_f.data[ii] + (interval->parametricObjFctn_alpha.data[ii+1]-interval->parametricObjFctn_alpha.data[ii]) * interval->parametricObjFctn_fPrime.data[ii] + .5*(interval->parametricObjFctn_alpha.data[ii+1]-interval->parametricObjFctn_alpha.data[ii]) * (interval->parametricObjFctn_alpha.data[ii+1]-interval->parametricObjFctn_alpha.data[ii]) * interval->parametricObjFctn_fPrimePrime.data[ii],
							interval->parametricObjFctn_f.data[ii+1]
							);
				}
				else {
					/* check with final prediction */
					qpDUNES_printf( "          int%d-sec%d: PREDICTION VS REALITY CHECK: predicted: %+.8e ** final reality: %+.8e",
							interval->id,
							ii,
							interval->parametricObjFctn_f.data[ii-1] + (interval->parametricObjFctn_alpha.data[ii]-interval->parametricObjFctn_alpha.data[ii-1]) * interval->parametricObjFctn_fPrime.data[ii-1] + .5*(interval->parametricObjFctn_alpha.data[ii]-interval->parametricObjFctn_alpha.data[ii-1]) * (interval->parametricObjFctn_alpha.data[ii]-interval->parametricObjFctn_alpha.data[ii-1]) * interval->parametricObjFctn_fPrimePrime.data[ii-1],
							static_cast<qpOASES::SQProblem*>(qpoasesObject)->getObjVal()
							);
				}
			}
		}
		#endif	/* __DO_UNIT_TESTS__ */
	}

	return QPDUNES_OK;
//#endif
}
/*<<< END OF qp42_solveLocalQP */


/* ----------------------------------------------
 * Get dual solution and translate it to qpDUNES
 * style
 *
#>>>>>>                                           */
return_t qpOASES_getDualSol( 	qpData_t* qpData,
								interval_t* interval,
								qpoasesObject_t* qpoasesObject,
								d2_vector_t* mu
								)
{
	int_t ii;

	/* (1) get qpOASES multipliers
	 * 	   qpOASES internal: first nV entries for bounds, then nC for constraints (positve: lower, negative: upper) */
	static_cast<qpOASES::LoggedSQProblem*>(qpoasesObject->qpoases)->getDualSolution( static_cast<qpOASES::real_t*>(mu->data) );

	//qpDUNES_printMatrixData( mu->data, 2*interval->nV+2*interval->nD, 1, "qpoases multipliers[%d]", interval->id );

	/* (2) translate to qpDUNES style: (muLow, muUpp)-pairs >= 0, first for bounds then for constraints
	 *     start at end, go to beginning in paris of two */
	for (ii = _NV_+_ND_-1; ii >= 0; --ii)	{
		if ( mu->data[ii] >= 0.0 )	{	/* lower bound active */
			mu->data[2*ii  ] = mu->data[ii];
			mu->data[2*ii+1] = 0.0;
		}
		else {							/* upper bound active */
			mu->data[2*ii+1] = -mu->data[ii];
			mu->data[2*ii  ] = 0.0;
		}
	}

	//qpDUNES_printMatrixData( mu->data, 2*interval->nV+2*interval->nD, 1, "translated multipliers[%d]", interval->id );

	return QPDUNES_OK;
}
/*<<< END OF qpOASES_getDualSol */


/* ----------------------------------------------
 * Get projected Hessian
 *
#>>>>>>                                           */
return_t qpOASES_getCholZTHZ( 	qpData_t* qpData,
								qpoasesObject_t* qpoasesObject,
								zz_matrix_t* cholZTHZ 				)
{
	qpOASES::real_t* qpOASES_Rptr = 0;
	static_cast<qpOASES::LoggedSQProblem*>(qpoasesObject->qpoases)->getR( &qpOASES_Rptr );

	cholZTHZ->data = qpOASES_Rptr;	// work with qpOASES memory

	return QPDUNES_OK;
}
/*<<< END OF qpOASES_getCholZTHZ */


/* ----------------------------------------------
 * Get null-space basis matrix Z
 *
#>>>>>>                                           */
return_t qpOASES_getZT( qpData_t* qpData,
						qpoasesObject_t* qpoasesObject,
						int_t* nFree,
						zz_matrix_t* ZT 				)
{
	qpOASES::real_t* qpOASES_Qptr = 0;
	static_cast<qpOASES::LoggedSQProblem*>(qpoasesObject->qpoases)->getQT( &qpOASES_Qptr, nFree );

	ZT->data = qpOASES_Qptr;	// work with qpOASES memory

	return QPDUNES_OK;
}
/*<<< END OF qp42_solveLocalQP */


/* ----------------------------------------------
 * Get number of active constraints and bounds
 *
#>>>>>>                                           */
int_t qpOASES_getNbrActConstr( 	qpoasesObject_t* qpoasesObject
								)
{
	/* for some weird reason getNAC produces illegal memory accesses in valgrind. TODO: debug! */
//	qpDUNES_printf("qpoases 2: %d", (int_t)(static_cast<qpOASES::SQProblem*>(qpoasesObject->qpoases)->getNFX()) );
//	qpDUNES_printf("qpoases 1: %d", (int_t)(static_cast<qpOASES::SQProblem*>(qpoasesObject->qpoases)->getNAC()) );
	return -1000;
//	return (int_t)(static_cast<qpOASES::SQProblem*>(qpoasesObject->qpoases)->getNAC()) + (int_t)(static_cast<qpOASES::SQProblem*>(qpoasesObject->qpoases)->getNFX());
}
/*<<< END OF qp42_solveLocalQP */



/* ----------------------------------------------
 * gets the step size to the first active set change
 * if it is shorter than an incumbent step size
 * initially in alphaMin
 *
#>>>>>>                                           */
return_t qpOASES_getMinStepsize(	const qpData_t* const qpData,
									const interval_t* const interval,
									real_t* alphaMin )
{
	real_t alphaASChange;

	if (interval->parametricObjFctn_nBasePoints > 2)	{			/* == full step leads to AS change on this stage */
		alphaASChange = interval->parametricObjFctn_alpha.data[1];	/* alphas are ordered, so this is the first AS change */
		if ( alphaASChange < *alphaMin ) {
			*alphaMin = alphaASChange;
		}
	}

	return QPDUNES_OK;
}
/*<<< END OF qpOASES_getMinStepsize */


/* ----------------------------------------------
 * do a step of length alpha
 *
#>>>>>>                                           */
return_t qpOASES_doStep( qpData_t* const qpData,
						 qpoasesObject_t* qpoasesObject,
						 interval_t* const interval,
						 real_t alpha,
						 z_vector_t* const z,
						 d2_vector_t* const mu,
						 z_vector_t* const qCandidate,
						 real_t* const p
						 )
{
	uint_t ii;
	return_t statusFlag;

	/** (1) do not call qpOASES again for full steps */
	if (alpha > 1. - qpData->options.equalityTolerance)	{
		/* get primal and dual solution */
		static_cast<qpOASES::LoggedSQProblem*>(qpoasesObject->qpoases)->getPrimalSolution( static_cast<qpOASES::real_t*>(z->data) );
		qpOASES_getDualSol( qpData, interval, qpoasesObject, mu );

		/* update  q := qFullStep  */
		for ( ii=0; ii<interval->nV; ++ii ) {
			qCandidate->data[ii] = interval->qpSolverQpoases.qFullStep.data[ii];
		}
		/* update p := pFullStep  */
		*p = interval->qpSolverQpoases.pFullStep;

		return QPDUNES_OK;
	}


	/** (2) drive back homotopy if demanded by line search */

	/* update  q := (1-alpha)*qFullStep + alpha*qOld  */
	for ( ii=0; ii<interval->nV; ++ii ) {
		qCandidate->data[ii] = (1-alpha) * interval->q.data[ii] + alpha * interval->qpSolverQpoases.qFullStep.data[ii];
	}

	/* resolve qpOASES for desired step size */
	statusFlag = qpOASES_hotstart( qpData, interval->qpSolverQpoases.qpoasesObject, interval, qCandidate, 0, QPDUNES_FALSE );

	/* get primal and dual solution */
	static_cast<qpOASES::LoggedSQProblem*>(qpoasesObject->qpoases)->getPrimalSolution( static_cast<qpOASES::real_t*>(z->data) );
	qpOASES_getDualSol( qpData, interval, qpoasesObject, mu );

	/* update p := (1-alpha)*pFullStep + alpha*pOld  */
	*p = (1-alpha) * interval->p + alpha * interval->qpSolverQpoases.pFullStep;

	return statusFlag;
}
/*<<< END OF qpOASES_doStep */



/* ----------------------------------------------
 * evaluate stage objective at given alpha values
 * along homotopy, based on precomputed piecewise
 * quadratic parameterization
 *
#>>>>>>                                           */
return_t qpOASES_evalAddParametricObjFctn(	qpData_t* const qpData,
											interval_t* const interval,
											large_vector_t* resVec,			/* results vector, to be added to! */
											large_vector_t* alphaVec,
											int_t nBasePoints
											)
{
	int_t ii;
	int_t secIdx = 0;		/* section index for evaluation of objective function parameterization */

	real_t alpha;
	real_t dAlpha;


	/* run through homotopy and evaluate objective at given alpha values */
	for ( ii=1; ii<nBasePoints-1; ++ii)	{
		alpha = alphaVec->data[ii];

		/* find right section for alpha
		 * note: alphaVec is ordered */
		for ( ;  ; ++secIdx ) {
			if ( secIdx+1 >= interval->parametricObjFctn_nBasePoints )	{
				qpDUNES_printError( qpData, __FILE__, __LINE__, "Parametric objective could not be evaluated at stage %d for alpha=%.3e.", interval->id, alpha );
				return QPDUNES_ERR_INVALID_ARGUMENT;
			}
			if ( interval->parametricObjFctn_alpha.data[secIdx+1] >= alpha )	break;
		}

//		qpDUNES_printf("I am interval %d and I found alpha = %.3e in section %d", interval->id, alpha, secIdx );

		/* evaluate parameterized objective function at alpha
		 * and _ADD_ it to results */
		dAlpha = alpha - interval->parametricObjFctn_alpha.data[secIdx];
		resVec->data[ii] += interval->parametricObjFctn_f.data[secIdx]
		                   + interval->parametricObjFctn_fPrime.data[secIdx] * dAlpha
		                   + .5 * interval->parametricObjFctn_fPrimePrime.data[secIdx] * dAlpha * dAlpha;
	}

	return QPDUNES_OK;
}
/*<<< END OF qpOASES_evalParametricObjFctn */


/* ----------------------------------------------
 * return stage objective derivative at given
 * alpha value along homotopy, based on
 * precomputed piecewise quadratic parameterization
 *
#>>>>>>                                           */
real_t qpOASES_getParametricObjFctnGrad(	qpData_t* const qpData,
											interval_t* const interval,
											real_t alpha
											)
{
	int_t secIdx;		/* section index for evaluation of objective function parameterization */

	real_t dAlpha;


	/* find right homotopy section for alpha */
	for ( secIdx = 0;  ; ++secIdx ) {
		if ( secIdx+1 >= interval->parametricObjFctn_nBasePoints )	{
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Derivative of parametric objective could not be evaluated at stage %d for alpha=%.3e.", interval->id, alpha );
			return qpData->options.QPDUNES_INFTY;
		}
		if ( interval->parametricObjFctn_alpha.data[secIdx+1] >= alpha )	break;
	}

	/* evaluate derivative of parameterized objective function at alpha */
	dAlpha = alpha - interval->parametricObjFctn_alpha.data[secIdx];

	return interval->parametricObjFctn_fPrime.data[secIdx] + interval->parametricObjFctn_fPrimePrime.data[secIdx] * dAlpha;
}
/*<<< END OF qpOASES_getParametricObjFctnGrad */


/* ----------------------------------------------
 * return stage objective second derivative at
 * given alpha value (in given direction) along
 * homotopy, based on precomputed piecewise
 * quadratic parameterization
 *
#>>>>>>                                           */
real_t qpOASES_getParametricObjFctnHess(	qpData_t* const qpData,
											interval_t* const interval,
											real_t alpha,
											int_t direction
											)
{
	int_t secIdx;		/* section index for evaluation of objective function parameterization */


	/* find right homotopy section for alpha */
	for ( secIdx = 0;  ; ++secIdx ) {
		if ( secIdx+1 >= interval->parametricObjFctn_nBasePoints )	{
			qpDUNES_printError( qpData, __FILE__, __LINE__, "Second derivative of parametric objective could not be evaluated at stage %d for alpha=%.3e.", interval->id, alpha );
			return qpData->options.QPDUNES_INFTY;
		}
		if ( interval->parametricObjFctn_alpha.data[secIdx+1] >= alpha - qpData->options.equalityTolerance )	break;
	}

//	qpDUNES_printf("this is interval %d, want hess for alpha = %.3e, dir=%d closest partner to the right is alpha = %.3e", interval->id, alpha, direction, interval->parametricObjFctn_alpha.data[secIdx+1] );
	/* check in which direction we need the second derivative in case we are very close to a homotopy kink (hessian jumping point) */
	if ( fabs( alpha - interval->parametricObjFctn_alpha.data[secIdx+1] ) <= qpData->options.equalityTolerance )	{
		if ( direction > 0 )	{
			if ( secIdx >= interval->parametricObjFctn_nBasePoints-1 )	{
				/* WARNING: this could indeed happen, if the optimal stepsize is > 1! */
//				qpDUNES_printError( qpData, __FILE__, __LINE__, "Second derivative of parametric objective could not be evaluated at stage %d for alpha=%.3e and derivativeDirection=%d.", interval->id, alpha, direction );
//				return qpData->options.QPDUNES_INFTY;
				/* we do not have any homotopy information beyond 1,
				 * so the best we can do is to take the last available hessian */
				return interval->parametricObjFctn_fPrimePrime.data[secIdx];
			}
			else {
				return interval->parametricObjFctn_fPrimePrime.data[secIdx+1];
			}
		}
		else {
			return interval->parametricObjFctn_fPrimePrime.data[secIdx];
		}
	}

	return interval->parametricObjFctn_fPrimePrime.data[secIdx];
}
/*<<< END OF qpOASES_getParametricObjFctnGrad */




} /* extern C */

/*
 *	end of file
 */
