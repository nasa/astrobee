/*
 *	This file is part of qp42.
 *
 *	qp42 -- An Implementation of qp solver on 42 intervals.
 *	Copyright (C) 2012 by Janick Frasch, Hans Joachim Ferreau et al. 
 *	All rights reserved.
 *
 *	qp42 is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qp5 is distributed in the hope that it will be useful,
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
 *	\file include/qp/types.h
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 *
 *	Declaration of all non-built-in types (except for classes).
 */


#ifndef SETUP_MPC_H
#define SETUP_MPC_H


#include <qpDUNES.h>


/** 
 *	\brief ...
 *
 *	...
 *
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */
typedef struct
{
	/* problem data */
	qpData_t qpData;
	
	real_t* zRef;			/**<  */
	real_t* qpObjConst;			/**<  */
//	real_t* xRef;			/**<  */
//	real_t* uRef;			/**<  */
	
	/* solution */
	real_t* xOpt;
	real_t* uOpt;
	real_t* lambdaOpt;
	
	real_t optObjVal;
	
	/* flags */
	return_t exitFlag;
	boolean_t isLTI;
	
	/* workspace */
	real_t* xnTmp;
	real_t* zn1Tmp;
	real_t* zn1Tmp2;
	real_t* xn1Tmp;
	real_t* xn1Tmp2;
	real_t* xn1Tmp3;
	real_t* unTmp;
	real_t* unTmp2;
	
	real_t* z0LowOrig;
	real_t* z0UppOrig;

	/* TODO: also have a central y, here */

} mpcProblem_t;



/** 
 *	\brief Set up an linear time invariant (LTI) MPC problem with simple bounds
 *
 *	...
 *
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */
return_t mpcDUNES_setup(	mpcProblem_t* const mpcProblem,
							uint_t nI,
							uint_t nX,
							uint_t nU,
							uint_t* nD,
							qpOptions_t* qpOptions
							);



/** 
 *	\brief ...
 *
 *	...
 *
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */
return_t mpcDUNES_cleanup(	mpcProblem_t* const mpcProblem
							);



/** 
 *	\brief ...
 *
 *	...
 *
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */
return_t mpcDUNES_initLtiSb_xu(	mpcProblem_t* const mpcProblem,
								const real_t* const Q,
								const real_t* const R,
								const real_t* const S,
								const real_t* const P,
								const real_t* const A,
								const real_t* const B,
								const real_t* const c,
								const real_t* const xLow,
								const real_t* const xUpp,
								const real_t* const uLow,
								const real_t* const uUpp,
								const real_t* const xRef,
								const real_t* const uRef
								);



/**
 *	\brief ...
 *
 *	...
 *
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */
return_t mpcDUNES_initLtiSb(	mpcProblem_t* const mpcProblem,
								const real_t* const H_,
								const real_t* const P_,
								const real_t* const g_,
								const real_t* const C_,
								const real_t* const c_,
								const real_t* const zLow_,
								const real_t* const zUpp_,
								const real_t* const zRef_
								);



/** 
 *	\brief ...
 *
 *	...
 *
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */
return_t mpcDUNES_initLtvSb(	mpcProblem_t* const mpcProblem,
								const real_t* const H_,
								const real_t* const g_,
								const real_t* const C_,
								const real_t* const c_,
								const real_t* const zLow_,
								const real_t* const zUpp_,
								const real_t* const zRef_
								);



/**
 *	\brief ...
 *
 *	...
 *
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */
return_t mpcDUNES_initLtv(	mpcProblem_t* const mpcProblem,
							const real_t* const H_,
							const real_t* const g_,
							const real_t* const C_,
							const real_t* const c_,
							const real_t* const zLow_,
							const real_t* const zUpp_,
							const real_t* const D_,
							const real_t* const dLow_,
							const real_t* const dUpp_,
							const real_t* const zRef_
							);



/**
 *	\brief ...
 *
 *	...
 *
 *	\author Janick Frasch, Hans Joachim Ferreau
 *	\version 1.0beta
 *	\date 2012
 */
return_t mpcDUNES_solve(	mpcProblem_t* const mpcProblem,
							const real_t* const x0
							);



#endif	/* SETUP_MPC_H */


/*
 *	end of file
 */
