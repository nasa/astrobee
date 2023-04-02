
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
 *    \file include/acado/code_generation/mpc_export.hpp
 *    \author Hans Joachim Ferreau, Boris Houska, Stephane Coulier
 *    \date 2010-2012
 */


#ifndef ACADO_TOOLKIT_MPC_EXPORT_BIS_HPP
#define ACADO_TOOLKIT_MPC_EXPORT_BIS_HPP


#include <acado/code_generation/export_module.hpp>
#include <acado/code_generation/qp_lin_alg_export.hpp>
#include <acado/code_generation/export_file.hpp>

BEGIN_NAMESPACE_ACADO

/** 
 *	\brief User-interface to automatically generate algorithms for fast model predictive control
 *
 *	\ingroup UserInterfaces
 *
 *  The class MPCexport is a user-interface to automatically generate tailored
 *  algorithms for fast model predictive control. It takes an optimal control 
 *  problem (OCP) formulation and generates code based on given user options, 
 *  e.g specifying the number of integrator steps or the online QP solver.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class MPCexportBis : public ExportModule
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

		/** Default constructor. 
		 */
		MPCexportBis( );

		/** Constructor which takes OCP formulation.
		 *
		 *	@param[in] _ocp		OCP formulation for code export.
		 */
		MPCexportBis(	const OCP& _ocp
					);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		MPCexportBis(	const MPCexportBis& arg
					);

		/** Destructor. 
		 */
		virtual ~MPCexportBis( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
        MPCexportBis& operator=(	const MPCexportBis& arg
								);


		/** Exports all files of the auto-generated code into the given directory.
		 *
		 *	@param[in] dirName			Name of directory to be used to export files.
		 *	@param[in] _realString		String to be used to declare real variables.
		 *	@param[in] _intString		String to be used to declare integer variables.
		 *	@param[in] _precision		Number of digits to be used for exporting real values.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
        virtual returnValue exportCode(	const String& dirName,
										const String& _realString = "real_t",
										const String& _intString = "int",
										int _precision = 16
										);




    protected:

		/** Copies all class members from given object.
		 *
		 *	@param[in] arg		Right-hand side object.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue copy(	const MPCexportBis& arg
							);

		/** Frees internal dynamic memory to yield an empty function.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue clear( );


		/** Sets-up code export and initializes underlying export modules.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        RET_INVALID_OPTION, \n
		 *	        RET_INVALID_OBJECTIVE_FOR_CODE_EXPORT, \n
		 *	        RET_ONLY_ODE_FOR_CODE_EXPORT, \n
		 *	        RET_NO_DISCRETE_ODE_FOR_CODE_EXPORT, \n
		 *	        RET_ONLY_STATES_AND_CONTROLS_FOR_CODE_EXPORT, \n
		 *	        RET_ONLY_EQUIDISTANT_GRID_FOR_CODE_EXPORT, \n
		 *	        RET_ONLY_BOUNDS_FOR_CODE_EXPORT, \n
		 *	        RET_UNABLE_TO_EXPORT_CODE
		 */
		returnValue setup( );

		/** Collects all data declarations of the auto-generated sub-modules to given 
		 *	list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        RET_UNABLE_TO_EXPORT_CODE
		 */
		returnValue collectDataDeclarations(	ExportStatementBlock& declarations,
												ExportStruct dataStruct = ACADO_ANY
												) const;

		/** Collects all function (forward) declarations of the auto-generated sub-modules 
		 *	to given list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        RET_UNABLE_TO_EXPORT_CODE
		 */
		returnValue collectFunctionDeclarations(	ExportStatementBlock& declarations
													) const;


		/** Exports main header file for using the exported MPC algorithm.
		 *
		 *	@param[in] _dirName			Name of directory to be used to export file.
		 *	@param[in] _fileName		Name of file to be exported.
		 *	@param[in] _realString		String to be used to declare real variables.
		 *	@param[in] _intString		String to be used to declare integer variables.
		 *	@param[in] _precision		Number of digits to be used for exporting real values.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue exportAcadoHeader(	const String& _dirName,
										const String& _fileName,
										const String& _realString = "real_t",
										const String& _intString = "int",
										int _precision = 16
										) const;



    protected:

		QPLinAlgExport*  qpLinAlg;			/**< Module for exporting a tailored condensing algorithm. */
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_MPC_EXPORTBIS__HPP

// end of file.
