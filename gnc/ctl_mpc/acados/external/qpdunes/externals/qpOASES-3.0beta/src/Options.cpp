/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2014 by Hans Joachim Ferreau, Andreas Potschka,
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
 *	\file src/Options.cpp
 *	\author Hans Joachim Ferreau, Andreas Potschka, Christian Kirches
 *	\version 3.0
 *	\date 2007-2014
 *
 *	Implementation of the Options class designed to manage working sets of
 *	constraints and bounds within a QProblem.
 */


#include <qpOASES/Options.hpp>


BEGIN_NAMESPACE_QPOASES


/*****************************************************************************
 *  P U B L I C                                                              *
 *****************************************************************************/


/*
 *	O p t i o n s
 */
Options::Options( )
{
	setToDefault( );
}


/*
 *	O p t i o n s
 */
Options::Options( const Options& rhs )
{
	copy( rhs );
}


/*
 *	~ O p t i o n s
 */
Options::~Options( )
{
}


/*
 *	o p e r a t o r =
 */
Options& Options::operator=( const Options& rhs )
{
	if ( this != &rhs )
	{
		copy( rhs );
	}

	return *this;
}



/*
 *	s e t T o D e f a u l t
 */
returnValue Options::setToDefault( )
{
	printLevel = PL_MEDIUM;
	#ifdef __DEBUG__
	printLevel = PL_HIGH;
	#endif
	#ifdef __XPCTARGET__
	printLevel = PL_NONE;
	#endif
	#ifdef __SUPPRESSANYOUTPUT__
	printLevel = PL_NONE;
	#endif

	enableRamping                 =  BT_TRUE;
	enableFarBounds               =  BT_TRUE;
	enableFlippingBounds          =  BT_TRUE;
	enableRegularisation          =  BT_FALSE;
	enableFullLITests             =  BT_FALSE;
	enableNZCTests                =  BT_TRUE;
	enableDriftCorrection         =  1;
	enableCholeskyRefactorisation =  0;
	enableEqualities              =  BT_FALSE;

	#ifdef __USE_SINGLE_PRECISION__
	terminationTolerance          =  1.0e2 * EPS;
	boundTolerance                =  1.0e2 * EPS;
	#else
	terminationTolerance          =  5.0e6 * EPS;
	boundTolerance                =  1.0e6 * EPS;
	#endif
	boundRelaxation               =  1.0e4;
	#ifdef __USE_SINGLE_PRECISION__
	epsNum                        = -1.0e2 * EPS;
	epsDen                        =  1.0e2 * EPS;
	#else
	epsNum                        = -1.0e3 * EPS;
	epsDen                        =  1.0e3 * EPS;
	#endif
	maxPrimalJump                 =  1.0e8;
	maxDualJump                   =  1.0e8;

	initialRamping                =  0.5;
	finalRamping                  =  1.0;
	initialFarBounds              =  1.0e6;
	growFarBounds                 =  1.0e3;
 	initialStatusBounds           =  ST_LOWER;
	#ifdef __USE_SINGLE_PRECISION__
	epsFlipping                   =  5.0e1 * EPS;
	#else
	epsFlipping                   =  1.0e3 * EPS;
	#endif
	numRegularisationSteps        =  0;
	#ifdef __USE_SINGLE_PRECISION__
	epsRegularisation             =  2.0e1 * EPS;
	numRefinementSteps            =  2;
	#else
	epsRegularisation             =  1.0e3 * EPS;
	numRefinementSteps            =  1;
	#endif
	epsIterRef                    =  1.0e2 * EPS;
	#ifdef __USE_SINGLE_PRECISION__
	epsLITests                    =  5.0e1 * EPS;
	epsNZCTests                   =  1.0e2 * EPS;
	#else
	epsLITests                    =  1.0e5 * EPS;
	epsNZCTests                   =  3.0e3 * EPS;
	#endif

	enableDropInfeasibles         =  BT_FALSE;
    dropBoundPriority             =  1;
    dropEqConPriority             =  1;
    dropIneqConPriority           =  1;
    
	return SUCCESSFUL_RETURN;
}


/*
 *	s e t T o R e l i a b l e
 */
returnValue Options::setToReliable( )
{
	setToDefault( );

	enableFullLITests             =  BT_TRUE;
	enableCholeskyRefactorisation =  1;

	#ifdef __USE_SINGLE_PRECISION__
	numRefinementSteps            =  3;
	#else
	numRefinementSteps            =  2;
	#endif

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t T o M P C
 */
returnValue Options::setToMPC( )
{
	setToDefault( );

	enableRamping                 =  BT_FALSE;
	enableFarBounds               =  BT_TRUE;
	enableFlippingBounds          =  BT_FALSE;
	enableRegularisation          =  BT_TRUE;
	enableNZCTests                =  BT_FALSE;
	enableDriftCorrection         =  0;
	enableEqualities              =  BT_TRUE;

	#ifdef __USE_SINGLE_PRECISION__
	terminationTolerance          =  1.0e3 * EPS;
	#else
	terminationTolerance          =  1.0e9 * EPS;
	#endif

	initialStatusBounds           =  ST_INACTIVE;
	numRegularisationSteps        =  1;
	#ifdef __USE_SINGLE_PRECISION__
	numRefinementSteps            =  2;
	#else
	numRefinementSteps            =  0;
	#endif

	return SUCCESSFUL_RETURN;
}


/*
 *	s e t T o F a s t
 */
returnValue Options::setToFast( )
{
	return setToMPC( );
}



/*
 *	e n s u r e C o n s i s t e n c y
 */
returnValue Options::ensureConsistency( )
{
	BooleanType needToAdjust = BT_FALSE;

	/* flipping bounds require far bounds */
    /* (ckirches) Removed this as per filter's trust region
	if( enableFlippingBounds == BT_TRUE )
		enableFarBounds = BT_TRUE;
    */
	
	if( enableDriftCorrection < 0 )
	{
		enableDriftCorrection = 0;
		needToAdjust = BT_TRUE;
	}
	
	if( enableCholeskyRefactorisation < 0 )
	{
		enableCholeskyRefactorisation = 0;
		needToAdjust = BT_TRUE;
	}

	if ( terminationTolerance <= 0.0 )
	{
		terminationTolerance = EPS;
		needToAdjust = BT_TRUE;
	}

	if ( epsIterRef <= 0.0 )
	{
		epsIterRef = EPS;
		needToAdjust = BT_TRUE;
	}

	if ( epsRegularisation <= 0.0 )
	{
		epsRegularisation = EPS;
		needToAdjust = BT_TRUE;
	}

	if ( boundTolerance <= 0.0 )
	{
		boundTolerance = EPS;
		needToAdjust = BT_TRUE;
	}

	if ( boundRelaxation <= 0.0 )
	{
		boundRelaxation = EPS;
		needToAdjust = BT_TRUE;
	}
	
	if ( maxPrimalJump <= 0.0 )
	{
		maxPrimalJump = EPS;
		needToAdjust = BT_TRUE;
	}

	if ( maxDualJump <= 0.0 )
	{
		maxDualJump = EPS;
		needToAdjust = BT_TRUE;
	}


	if ( initialRamping < 0.0 )
	{
		initialRamping = 0.0;
		needToAdjust = BT_TRUE;
	}

	if ( finalRamping < 0.0 )
	{
		finalRamping = 0.0;
		needToAdjust = BT_TRUE;
	}

	if ( initialFarBounds <= boundRelaxation )
	{
		initialFarBounds = boundRelaxation+EPS;
		needToAdjust = BT_TRUE;
	}
	
	if ( growFarBounds < 1.1 )
	{
		growFarBounds = 1.1;
		needToAdjust = BT_TRUE;
	}

	if ( epsFlipping <= 0.0 )
	{
		epsFlipping = EPS;
		needToAdjust = BT_TRUE;
	}

	if ( numRegularisationSteps < 0 )
	{
		numRegularisationSteps = 0;
		needToAdjust = BT_TRUE;
	}

	if ( epsRegularisation < 0.0 )
	{
		epsRegularisation = EPS;
		needToAdjust = BT_TRUE;
	}

	if ( numRefinementSteps < 0 )
	{
		numRefinementSteps = 0;
		needToAdjust = BT_TRUE;
	}

	if ( epsIterRef < 0.0 )
	{
		epsIterRef = EPS;
		needToAdjust = BT_TRUE;
	}

	if ( epsLITests < 0.0 )
	{
		epsLITests = EPS;
		needToAdjust = BT_TRUE;
	}

	if ( epsNZCTests < 0.0 )
	{
		epsNZCTests = EPS;
		needToAdjust = BT_TRUE;
	}

	if ( needToAdjust == BT_TRUE)
		return THROWWARNING( RET_OPTIONS_ADJUSTED );

	return SUCCESSFUL_RETURN;
}


/*
 *	p r i n t
 */
returnValue Options::print( ) const
{
	#ifndef __XPCTARGET__
	#ifndef __DSPACE__
	char myPrintfString[160];
	char info[20];

	myPrintf( "\n###################   qpOASES  --  QP OPTIONS   ##################\n" );
	myPrintf( "\n" );

	convertPrintLevelToString( printLevel,info );
	snprintf( myPrintfString,80,"printLevel                     =  %s\n",info );
	myPrintf( myPrintfString );

	myPrintf( "\n" );

	convertBooleanTypeToString( enableRamping,info );
	snprintf( myPrintfString,80,"enableRamping                  =  %s\n",info );
	myPrintf( myPrintfString );

	convertBooleanTypeToString( enableFarBounds,info );
	snprintf( myPrintfString,80,"enableFarBounds                =  %s\n",info );
	myPrintf( myPrintfString );

	convertBooleanTypeToString( enableFlippingBounds,info );
	snprintf( myPrintfString,80,"enableFlippingBounds           =  %s\n",info );
	myPrintf( myPrintfString );

	convertBooleanTypeToString( enableRegularisation,info );
	snprintf( myPrintfString,80,"enableRegularisation           =  %s\n",info );
	myPrintf( myPrintfString );

	convertBooleanTypeToString( enableFullLITests,info );
	snprintf( myPrintfString,80,"enableFullLITests              =  %s\n",info );
	myPrintf( myPrintfString );

	convertBooleanTypeToString( enableNZCTests,info );
	snprintf( myPrintfString,80,"enableNZCTests                 =  %s\n",info );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"enableDriftCorrection          =  %d\n",enableDriftCorrection );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"enableCholeskyRefactorisation  =  %d\n",enableCholeskyRefactorisation );
	myPrintf( myPrintfString );

	convertBooleanTypeToString( enableEqualities,info );
	snprintf( myPrintfString,80,"enableEqualities               =  %s\n",info );
	myPrintf( myPrintfString );

	myPrintf( "\n" );

	snprintf( myPrintfString,80,"terminationTolerance           =  %e\n",terminationTolerance );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"boundTolerance                 =  %e\n",boundTolerance );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"boundRelaxation                =  %e\n",boundRelaxation );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"epsNum                         =  %e\n",epsNum );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"epsDen                         =  %e\n",epsDen );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"maxPrimalJump                  =  %e\n",maxPrimalJump );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"maxDualJump                    =  %e\n",maxDualJump );
	myPrintf( myPrintfString );

	myPrintf( "\n" );

	snprintf( myPrintfString,80,"initialRamping                 =  %e\n",initialRamping );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"finalRamping                   =  %e\n",finalRamping );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"initialFarBounds               =  %e\n",initialFarBounds );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"growFarBounds                  =  %e\n",growFarBounds );
	myPrintf( myPrintfString );

	convertSubjectToStatusToString( initialStatusBounds,info );
	snprintf( myPrintfString,80,"initialStatusBounds            =  %s\n",info );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"epsFlipping                    =  %e\n",epsFlipping );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"numRegularisationSteps         =  %d\n",numRegularisationSteps );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"epsRegularisation              =  %e\n",epsRegularisation );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"numRefinementSteps             =  %d\n",numRefinementSteps );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"epsIterRef                     =  %e\n",epsIterRef );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"epsLITests                     =  %e\n",epsLITests );
	myPrintf( myPrintfString );

	snprintf( myPrintfString,80,"epsNZCTests                    =  %e\n",epsNZCTests );
	myPrintf( myPrintfString );

	myPrintf( "\n\n" );
	#endif
	#endif

	return SUCCESSFUL_RETURN;
}



/*****************************************************************************
 *  P R O T E C T E D                                                        *
 *****************************************************************************/

/*
 *	c o p y
 */
returnValue Options::copy(	const Options& rhs
							)
{
	printLevel             = rhs.printLevel;

	enableRamping                 =  rhs.enableRamping;
	enableFarBounds               =  rhs.enableFarBounds;
	enableFlippingBounds          =  rhs.enableFlippingBounds;
	enableRegularisation          =  rhs.enableRegularisation;
	enableFullLITests             =  rhs.enableFullLITests;
	enableNZCTests                =  rhs.enableNZCTests;
	enableDriftCorrection         =  rhs.enableDriftCorrection;
	enableCholeskyRefactorisation =  rhs.enableCholeskyRefactorisation;
	enableEqualities              =  rhs.enableEqualities;

	terminationTolerance          =  rhs.terminationTolerance;
	boundTolerance                =  rhs.boundTolerance;
	boundRelaxation               =  rhs.boundRelaxation;
	epsNum                        =  rhs.epsNum;
	epsDen                        =  rhs.epsDen;
	maxPrimalJump                 =  rhs.maxPrimalJump;
	maxDualJump                   =  rhs.maxDualJump;

	initialRamping                =  rhs.initialRamping;
	finalRamping                  =  rhs.finalRamping;
	initialFarBounds              =  rhs.initialFarBounds;
	growFarBounds                 =  rhs.growFarBounds;
 	initialStatusBounds           =  rhs.initialStatusBounds;
	epsFlipping                   =  rhs.epsFlipping;
	numRegularisationSteps        =  rhs.numRegularisationSteps;
	epsRegularisation             =  rhs.epsRegularisation;
	numRefinementSteps            =  rhs.numRefinementSteps;
	epsIterRef                    =  rhs.epsIterRef;
	epsLITests                    =  rhs.epsLITests;
	epsNZCTests                   =  rhs.epsNZCTests;

	enableDropInfeasibles         =  rhs.enableDropInfeasibles;
    dropBoundPriority             =  rhs.dropBoundPriority;
    dropEqConPriority             =  rhs.dropEqConPriority;
    dropIneqConPriority           =  rhs.dropIneqConPriority;

	return SUCCESSFUL_RETURN;
}


END_NAMESPACE_QPOASES


/*
 *	end of file
 */
