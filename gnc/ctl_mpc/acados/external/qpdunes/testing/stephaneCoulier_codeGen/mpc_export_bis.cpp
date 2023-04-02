
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
 *    \file src/code_generation/mpc_export_bis.cpp
 *    \author Hans Joachim Ferreau, Boris Houska, Milan Vukov, Stephane Coulier
 *    \date 2010-2012
 */

#include <acado/code_generation/mpc_export_bis.hpp>

#include <acado/code_generation/export_algorithm_factory.hpp>

BEGIN_NAMESPACE_ACADO


//
// PUBLIC MEMBER FUNCTIONS:
//

MPCexportBis::MPCexportBis( ) : ExportModule( )
{
	qpLinAlg    = 0;

	setStatus( BS_NOT_INITIALIZED );
}


MPCexportBis::MPCexportBis(	const OCP& _ocp
						) : ExportModule( _ocp )
{
	qpLinAlg    = 0;

	setStatus( BS_NOT_INITIALIZED );
}


MPCexportBis::MPCexportBis(	const MPCexportBis& arg
						) :  ExportModule( arg )
{
	copy( arg );
}


MPCexportBis::~MPCexportBis( )
{
	clear( );
}


MPCexportBis& MPCexportBis::operator=(	const MPCexportBis& arg
									)
{
	if( this != &arg )
	{
		clear( );
		ExportModule::operator=( arg );
		copy( arg );
	}

	return *this;
}



returnValue MPCexportBis::exportCode(	const String& dirName,
									const String& _realString,
									const String& _intString,
									int _precision
									)
{
	if ( setup( ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	int printLevel;
	get( PRINTLEVEL,printLevel );

	// export mandatory source code files
	if ( exportAcadoHeader( dirName,commonHeaderName,_realString,_intString,_precision ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
	
	if( qpLinAlg != 0 )
	{
		
		String fileName( dirName ) ;
		fileName << "/qp_lin_alg.c";

		ExportFile qpLinAlgFile( fileName,commonHeaderName,_realString,_intString,_precision );
		qpLinAlg->getCode( qpLinAlgFile );	
		if ( qpLinAlgFile.exportCode( ) != SUCCESSFUL_RETURN )
			return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );
	}

	if ( (PrintLevel)printLevel >= HIGH ) 
		acadoPrintf( "done.\n" );

	if ( (PrintLevel)printLevel > NONE )
		ACADOINFO( RET_CODE_EXPORT_SUCCESSFUL );


    return SUCCESSFUL_RETURN; 
}




//
// PROTECTED MEMBER FUNCTIONS:
//

returnValue MPCexportBis::copy(	const MPCexportBis& arg
								)
{
	// TODO: why like this? Why the copy is made this way?
	if ( arg.qpLinAlg != 0 )
		qpLinAlg = new QPLinAlgExport( *(arg.qpLinAlg) );
	else
		qpLinAlg = 0;
	
	return SUCCESSFUL_RETURN;
}


returnValue MPCexportBis::clear( )
{
	if ( qpLinAlg != 0 )
		delete qpLinAlg;

	return SUCCESSFUL_RETURN;
}



returnValue MPCexportBis::setup( )
{
	if ( qpLinAlg != 0 )
		delete qpLinAlg;

	qpLinAlg = new QPLinAlgExport( this,commonHeaderName );
	
	ExportVariable Q, R, QF, QS, QS2;

	ocp.getQRmatrices( Q, R, QF, QS, QS2 );

	
	// check for positive semi-definiteness of Q
	if ( Q.isGiven() == BT_TRUE )
		if ( Q.getGivenMatrix().isPositiveSemiDefinite() == BT_FALSE )
			return ACADOERROR( RET_NONPOSITIVE_WEIGHT );

	if( Q.getDim() == 0 )
		Q = zeros( NX,NX );

	acadoPrintf("setupmpc");

	qpLinAlg->setWeightingMatrices( Q, R, QF, QS, QS2 );
	qpLinAlg->setup( );


	setStatus( BS_READY );

	return SUCCESSFUL_RETURN;
}



returnValue MPCexportBis::collectDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct
												) const
{
	if ( qpLinAlg->getDataDeclarations( declarations,dataStruct ) != SUCCESSFUL_RETURN )
		return RET_UNABLE_TO_EXPORT_CODE;

	return SUCCESSFUL_RETURN;
}


returnValue MPCexportBis::collectFunctionDeclarations(	ExportStatementBlock& declarations
													) const
{
	if ( qpLinAlg->getFunctionDeclarations( declarations ) != SUCCESSFUL_RETURN )
		return RET_UNABLE_TO_EXPORT_CODE;

	return SUCCESSFUL_RETURN;
}



returnValue MPCexportBis::exportAcadoHeader(	const String& _dirName,
											const String& _fileName,
											const String& _realString,
											const String& _intString,
											int _precision
											) const
{
	int qpSolver;
	get( QP_SOLVER,qpSolver );

	int operatingSystem;
	get( OPERATING_SYSTEM,operatingSystem );

	int useSinglePrecision;
	get( USE_SINGLE_PRECISION,useSinglePrecision );
	
	int fixInitialState;
	get( FIX_INITIAL_STATE,fixInitialState );


	String fileName( _dirName );
	fileName << "/" << _fileName;
	ExportFile acadoHeader( fileName,"", _realString,_intString,_precision );

	acadoHeader.addStatement( "#include <stdio.h>\n" );
	acadoHeader.addStatement( "#include <math.h>\n" );

	if ( (OperatingSystem)operatingSystem == OS_WINDOWS )
	{
		acadoHeader.addStatement( "#include <windows.h>\n" );
	}
	else
	{
		// OS_UNIX
		acadoHeader.addStatement( "#include <time.h>\n" );
		acadoHeader.addStatement( "#include <sys/stat.h>\n" );
		acadoHeader.addStatement( "#include <sys/time.h>\n" );
	}
	acadoHeader.addLinebreak( );

	switch ( (QPSolverName)qpSolver )
	{
		case QP_CVXGEN:
			acadoHeader.addStatement( "#define USE_CVXGEN\n" );
			acadoHeader.addStatement( "#include \"cvxgen/solver.h\"\n" );
			acadoHeader.addLinebreak( 2 );
			
			if ( (BooleanType)useSinglePrecision == BT_TRUE )
				acadoHeader.addStatement( "typedef float real_t;\n" );
			else
				acadoHeader.addStatement( "typedef double real_t;\n" );
			break;
			acadoHeader.addLinebreak( 2 );
		
		case QP_QPOASES:
			acadoHeader.addStatement( "#ifndef __MATLAB__\n" );
			acadoHeader.addStatement( "#ifdef __cplusplus\n" );
			acadoHeader.addStatement( "extern \"C\"\n" );
			acadoHeader.addStatement( "{\n" );
			acadoHeader.addStatement( "#endif\n" );
			acadoHeader.addStatement( "#endif\n" );
			acadoHeader.addStatement( "#include \"qpoases/solver.hpp\"\n" );
			acadoHeader.addLinebreak( 2 );
			break;
			
		case QP_QPOASES3:
			acadoHeader.addStatement( "#include \"qpoases3/solver.h\"\n" );
			acadoHeader.addLinebreak( 2 );
			break;

		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}
	
	acadoHeader.addComment( "GLOBAL VARIABLES:               " );
	acadoHeader.addComment( "--------------------------------" );
	acadoHeader.addStatement( "typedef struct ACADOvariables_ {\n" );

	if ( collectDataDeclarations( acadoHeader,ACADO_VARIABLES ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	acadoHeader.addLinebreak( );
	acadoHeader.addStatement( "} ACADOvariables;\n" );
	acadoHeader.addLinebreak( 2 );

	acadoHeader.addComment( "GLOBAL WORKSPACE:               " );
	acadoHeader.addComment( "--------------------------------" );
	acadoHeader.addStatement( "typedef struct ACADOworkspace_ {\n" );

	if ( collectDataDeclarations( acadoHeader,ACADO_WORKSPACE ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	acadoHeader.addLinebreak( );
	acadoHeader.addStatement( "} ACADOworkspace;\n" );
	acadoHeader.addLinebreak( 2 );

	acadoHeader.addComment( "GLOBAL FORWARD DECLARATIONS:         " );
	acadoHeader.addComment( "-------------------------------------" );

	if ( collectFunctionDeclarations( acadoHeader ) != SUCCESSFUL_RETURN )
		return ACADOERROR( RET_UNABLE_TO_EXPORT_CODE );

	acadoHeader.addComment( "-------------------------------------" );
	acadoHeader.addLinebreak( 2 );

	acadoHeader.addComment( "EXTERN DECLARATIONS:                 " );
	acadoHeader.addComment( "-------------------------------------" );
	acadoHeader.addStatement( "extern ACADOworkspace acadoWorkspace;\n" );
	acadoHeader.addStatement( "extern ACADOvariables acadoVariables;\n" );
	acadoHeader.addComment( "-------------------------------------" );

	switch ( (QPSolverName) qpSolver )
	{
		case QP_CVXGEN:
			break;
		
		case QP_QPOASES:
			acadoHeader.addStatement( "#ifndef __MATLAB__\n");
			acadoHeader.addStatement( "#ifdef __cplusplus\n" );
			acadoHeader.addLinebreak( );
			acadoHeader.addStatement( "} // extern \"C\"\n" );
			acadoHeader.addStatement( "#endif\n" );
			acadoHeader.addStatement( "#endif\n" );
			break;

		case QP_QPOASES3:
			break;

		default:
			return ACADOERROR( RET_INVALID_OPTION );
	}

	acadoHeader.addLinebreak( );
    acadoHeader.addComment( "END OF FILE." );
	acadoHeader.addLinebreak( );

	return acadoHeader.exportCode( );
}



CLOSE_NAMESPACE_ACADO

// end of file.
