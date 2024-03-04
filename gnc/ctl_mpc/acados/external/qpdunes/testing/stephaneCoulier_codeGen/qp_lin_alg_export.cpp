/*
 *    This file is part of ACADO Toolkit.
 *
 *    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
 *    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
 *    Developed within the Optimization in Engineering Center (OPTEC) under
 *    supervision of Moritz Diehl. All rights reserved.
 *
 *    ACADO Toolkit is free software; you can redistribute it and/or
 *    modify it under the terms of the GNU Lesser General Public
 *    License as published by the Free Software Foundation; either
 *    version 3 of the License, or (at your option) any later version.
 *
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *    Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public
 *    License along with ACADO Toolkit; if not, write to the Free Software
 *    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */



/**
 *    \file src/code_generation/condensing_export.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Stephane Coulier, Janick Frasch
 *    \date 2010-2011
 */

#include <acado/code_generation/qp_lin_alg_export.hpp>



BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

QPLinAlgExport::QPLinAlgExport(	UserInteraction* _userInteraction,
									const String& _commonHeaderName
									) : ExportAlgorithm( _userInteraction,_commonHeaderName )
{
	xBoundsIdx = 0;
	nxBounds = 0;
}


QPLinAlgExport::QPLinAlgExport(	const QPLinAlgExport& arg
									) : ExportAlgorithm( arg )
{
	copy( arg );
}


QPLinAlgExport::~QPLinAlgExport( )
{
	clear( );
}


QPLinAlgExport& QPLinAlgExport::operator=(	const QPLinAlgExport& arg
												)
{
	if( this != &arg )
	{
		clear( );
		ExportAlgorithm::operator=( arg );
		copy( arg );
	}

	return *this;
}



returnValue QPLinAlgExport::setup( )
{

	String fileName("qp_lin_alg.c");
	
	int printLevel;
	get( PRINTLEVEL,printLevel );

	if ( (PrintLevel)printLevel >= HIGH )
		acadoPrintf("--> Preparing to export %s... ",fileName.getName() );


	Q.setDataStruct( ACADO_VARIABLES );
	x.setDataStruct( ACADO_VARIABLES );
	Qx.setDataStruct( ACADO_VARIABLES );

	setupMultiplicationRoutines( );

	if ( (PrintLevel)printLevel >= HIGH )
		acadoPrintf("done.\n" );
	

	return SUCCESSFUL_RETURN;
}




returnValue QPLinAlgExport::setWeightingMatrices(	const ExportVariable& _Q,
													const ExportVariable& _R,
													const ExportVariable& _QF,
													const ExportVariable& _QS,
													const ExportVariable& _QS2
													)
{
	QS  = _QS;
	QS2 = _QS2;
    Q   = _Q;
    QF  = _QF;

    R = _R;

    return SUCCESSFUL_RETURN;
}


returnValue QPLinAlgExport::setStateBounds(	const VariablesGrid& _xBounds
												)
{
	BooleanType isFinite = BT_FALSE;
	Vector lbTmp = _xBounds.getLowerBounds(0);
	Vector ubTmp = _xBounds.getUpperBounds(0);
	
	if ( xBoundsIdx != 0 )
		delete[] xBoundsIdx;
	xBoundsIdx = new int[_xBounds.getDim()+1];

	for( uint j=0; j<lbTmp.getDim(); ++j )
	{
		if ( acadoIsGreater( ubTmp(j),lbTmp(j) ) == BT_FALSE )
			return ACADOERROR( RET_INVALID_ARGUMENTS );
			
		if ( ( acadoIsFinite( ubTmp(j) ) == BT_TRUE ) || ( acadoIsFinite( lbTmp(j) ) == BT_TRUE ) )
			isFinite = BT_TRUE;
	}

	for( uint i=1; i<_xBounds.getNumPoints(); ++i )
	{
		lbTmp = _xBounds.getLowerBounds(i);
		ubTmp = _xBounds.getUpperBounds(i);

		for( uint j=0; j<lbTmp.getDim(); ++j )
		{
			if ( acadoIsGreater( ubTmp(j),lbTmp(j) ) == BT_FALSE )
				return ACADOERROR( RET_INVALID_ARGUMENTS );
			
			if ( ( acadoIsFinite( ubTmp(j) ) == BT_TRUE ) || ( acadoIsFinite( lbTmp(j) ) == BT_TRUE ) )
			{
				xBoundsIdx[nxBounds] = i*lbTmp.getDim()+j;
				++nxBounds;
				isFinite = BT_TRUE;
			}
		}
	}

	xBoundsIdx[nxBounds] = -1;

	if ( isFinite == BT_TRUE )
		xBounds = _xBounds;
	else
		xBounds.init();

	return SUCCESSFUL_RETURN;
}



uint QPLinAlgExport::getNumStateBounds( ) const
{
	return nxBounds;
}


int QPLinAlgExport::getStateBoundComponent(	uint idx
												) const
{
	if( idx >= getNumStateBounds() )
		return -ACADOERROR( RET_INDEX_OUT_OF_BOUNDS );

	return xBoundsIdx[idx];
}


Vector QPLinAlgExport::getLowerBoundsX0( ) const
{
	if ( xBounds.isEmpty() == BT_FALSE )
		return xBounds.getLowerBounds(0);
	else
	{
		Vector tmp( getNX() );
		tmp.setAll( -INFTY );
		return tmp;
	}
}


Vector QPLinAlgExport::getUpperBoundsX0( ) const
{
	if ( xBounds.isEmpty() == BT_FALSE )
		return xBounds.getUpperBounds(0);
	else
	{
		Vector tmp( getNX() );
		tmp.setAll( INFTY );
		return tmp;
	}
}



BooleanType QPLinAlgExport::isInitialStateFixed( ) const
{
	int fixInitialState;
	get( FIX_INITIAL_STATE,fixInitialState );

	return (BooleanType)fixInitialState;
}



returnValue QPLinAlgExport::getDataDeclarations(	ExportStatementBlock& declarations,
													ExportStruct dataStruct
													) const
{
	// ACADO_VARIABLES
	declarations.addDeclaration( x,dataStruct );
	declarations.addDeclaration( u,dataStruct );
	declarations.addDeclaration( p,dataStruct );
	declarations.addDeclaration( xRef,dataStruct );
	declarations.addDeclaration( uRef,dataStruct );
	declarations.addDeclaration( x0Ref,dataStruct );
	declarations.addDeclaration( x0Ref2,dataStruct );

	if ( Q.isGiven() == BT_FALSE )
		declarations.addDeclaration( Q,dataStruct );
	
	if ( R.isGiven() == BT_FALSE )
		declarations.addDeclaration( R,dataStruct );
	
	if ( QF.isGiven() == BT_FALSE )
		declarations.addDeclaration( QF,dataStruct );
	
	if ( QS.isGiven() == BT_FALSE )
		declarations.addDeclaration( QS,dataStruct );

	if ( QS2.isGiven() == BT_FALSE )
		declarations.addDeclaration( QS2,dataStruct );


	// ACADO_WORKSPACE
	if ( QQF.isGiven() == BT_FALSE )
		declarations.addDeclaration( QQF,dataStruct );

	declarations.addDeclaration( state,dataStruct );
	declarations.addDeclaration( residuum,dataStruct );
	declarations.addDeclaration( g0,dataStruct );
	declarations.addDeclaration( g1,dataStruct );
	declarations.addDeclaration( H00,dataStruct );
	declarations.addDeclaration( H01,dataStruct );
	declarations.addDeclaration( H11,dataStruct );
	declarations.addDeclaration( lbA,dataStruct );
	declarations.addDeclaration( ubA,dataStruct );
	declarations.addDeclaration( d,dataStruct );
	declarations.addDeclaration( deltaX0,dataStruct );
	declarations.addDeclaration( C,dataStruct );
	declarations.addDeclaration( QC,dataStruct );
	declarations.addDeclaration( Gx,dataStruct );
	declarations.addDeclaration( E,dataStruct );

	return SUCCESSFUL_RETURN;
}


returnValue QPLinAlgExport::getFunctionDeclarations(	ExportStatementBlock& declarations
														) const
{
	declarations.addDeclaration( expand );
	declarations.addDeclaration( setupQP );
	
	declarations.addDeclaration( getObjectiveValue );
	declarations.addDeclaration( multiplyQx_exported );

	return SUCCESSFUL_RETURN;
}


returnValue QPLinAlgExport::getCode(	ExportStatementBlock& code
										)
{
	code.addFunction( expand );
	code.addFunction( setupQP );
	code.addFunction( multiplyQx_exported );

	code.addFunction( getObjectiveValue );

	return SUCCESSFUL_RETURN;
}


//
// PROTECTED MEMBER FUNCTIONS:
//


returnValue QPLinAlgExport::copy(	const QPLinAlgExport& arg
									)
{
	xBounds = arg.xBounds;
	
	if ( arg.xBoundsIdx != 0 )
	{
		xBoundsIdx = new int[arg.nxBounds];
		for( uint i=0; i<arg.nxBounds; ++i )
			xBoundsIdx[i] = arg.xBoundsIdx[i];
	}
	else
		xBoundsIdx = 0;

	nxBounds  = arg.nxBounds;

	// ExportVariables
	Q   = arg.Q;
	R   = arg.R;
	QF  = arg.QF;
	QQF = arg.QQF;
	QS  = arg.QS;
	QS2 = arg.QS2;

	x = arg.x;
	u = arg.u;
	p = arg.p;
	xRef   = arg.xRef;
	uRef   = arg.uRef;
	x0Ref  = arg.x0Ref;
	x0Ref2 = arg.x0Ref2;

	state    = arg.state;
	residuum = arg.residuum;
	g0       = arg.g0;
	g1       = arg.g1;
	H00      = arg.H00;
	H01      = arg.H01;
	H11      = arg.H11;
	lbA      = arg.lbA;
	ubA      = arg.ubA;
	d        = arg.d;
	deltaX0  = arg.deltaX0;
	C        = arg.C;
	QC       = arg.QC;
	Gx       = arg.Gx;
	Gu       = arg.Gu;
	E        = arg.E;
	QE       = arg.QE;
	Dx0      = arg.Dx0;
	Dx0b     = arg.Dx0b;
	Dx       = arg.Dx;
	QDx      = arg.QDx;
	Du       = arg.Du;
	RDu      = arg.RDu;

	deltaU = arg.deltaU;

	// ExportFunctions
	expand    = arg.expand;
	setupQP   = arg.setupQP;

	getObjectiveValue  = arg.getObjectiveValue;
	multiplyQx_exported = arg.multiplyQx_exported;

	return SUCCESSFUL_RETURN;
}


returnValue QPLinAlgExport::clear( )
{
	if ( xBoundsIdx != 0 )
		delete[] xBoundsIdx;

	return SUCCESSFUL_RETURN;
}



returnValue QPLinAlgExport::setupMultiplicationRoutines( )
{
	ExportVariable QQ = Q;
	QQ.setDataStruct( ACADO_LOCAL );
	int nrow = QQ.getNumRows();
	//ExportVariable xx = x;
	//xx.setDataStruct( ACADO_LOCAL );
	//int ncol = QQ.getNumCols();
	//xx.setup(nrow,ncol,REAL,ACADO_VARIABLES );
	
	ExportVariable xVec( "xVec", nrow, 1 );
	ExportVariable Qx("Qx", nrow , 1 );

	multiplyQx_exported.setup("multiplyQx_exported", QQ, xVec, Qx );
	multiplyQx_exported.addStatement( Qx == QQ*xVec );

	return SUCCESSFUL_RETURN;
}




CLOSE_NAMESPACE_ACADO

// end of file.
