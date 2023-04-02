
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
 *    \file include/acado/code_generation/qp_lin_alg_export.hpp
 *    \author Coulier Stephane
 *    \date 2012
 */


#ifndef ACADO_TOOLKIT_QP_LIN_ALG_EXPORT_HPP
#define ACADO_TOOLKIT_QP_LIN_ALG_EXPORT_HPP

#include <acado/utils/acado_utils.hpp>
#include <acado/matrix_vector/matrix_vector.hpp>
#include <acado/function/function.hpp>
#include <acado/code_generation/export_algorithm.hpp>
#include <acado/code_generation/integrator_export.hpp>


BEGIN_NAMESPACE_ACADO

/** 
 *	\brief Allows to export a tailored condensing algorithm for fast model predictive control.
 *
 *	\ingroup NumericalAlgorithms
 *
 *	The class QPLinAlgExport allows to export a tailored condensing algorithm 
 *	for fast model predictive control. This algorithm is typically used after a
 *	shooting discretization to eliminate the states from the underlying QP, 
 *	resulting in a smaller-scale QP.
 *
 *	\author Hans Joachim Ferreau, Boris Houska
 */
class QPLinAlgExport : public ExportAlgorithm
{
    //
    // PUBLIC MEMBER FUNCTIONS:
    //
    public:

		/** Default constructor. 
		 *
		 *	@param[in] _userInteraction		Pointer to corresponding user interface.
		 *	@param[in] _commonHeaderName	Name of common header file to be included.
		 */
        QPLinAlgExport(	UserInteraction* _userInteraction = 0,
							const String& _commonHeaderName = ""
							);

		/** Copy constructor (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		QPLinAlgExport(	const QPLinAlgExport& arg
							);

		/** Destructor. 
		*/
		virtual ~QPLinAlgExport( );

		/** Assignment operator (deep copy).
		 *
		 *	@param[in] arg		Right-hand side object.
		 */
		QPLinAlgExport& operator=(	const QPLinAlgExport& arg
										);


		/** Initializes export of a tailored condensing algorithm.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue setup( );


		/** Assigns weighting matrices of OCP objective function.
		 *
		 *	@param[in] _Q		Weighting matrix for differential states (along the prediction horizon).
		 *	@param[in] _R		Weighting matrix for control inputs (along the control horizon).
		 *	@param[in] _QS		First weighting matrix for free initial value.
		 *	@param[in] _QS2		Second weighting matrix for free initial value.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue setWeightingMatrices( 	const ExportVariable& _Q,
											const ExportVariable& _R,
											const ExportVariable& _QF,
											const ExportVariable& _QS,
											const ExportVariable& _QS2
											);

		/** Assigns lower/upper bounds on differential states.
		 *
		 *	@param[in] _xBounds		Lower/upper bounds on differential states.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        RET_INVALID_ARGUMENTS
		 */
		returnValue setStateBounds(	const VariablesGrid& _xBounds
									);


		/** Adds all data declarations of the auto-generated condensing algorithm 
		 *	to given list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue getDataDeclarations(	ExportStatementBlock& declarations,
													ExportStruct dataStruct = ACADO_ANY
													) const;

		/** Adds all function (forward) declarations of the auto-generated condensing algorithm 
		 *	to given list of declarations.
		 *
		 *	@param[in] declarations		List of declarations.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue getFunctionDeclarations(	ExportStatementBlock& declarations
														) const;


		/** Exports source code of the auto-generated condensing algorithm 
		 *  into the given directory.
		 *
		 *	@param[in] code				Code block containing the auto-generated condensing algorithm.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		virtual returnValue getCode(	ExportStatementBlock& code
										);




		/** Returns number of bounds on differential states.
		 *
		 *  \return Number of bounds on differential states
		 */
		uint getNumStateBounds( ) const;


		/** Returns component within VariablesGrid of lower/upper bounds of differential states
		 *  that corresponds to given index of finite bounds.
		 *
		 *	@param[in] idx		Index of finite bounds on differential states.
		 *
		 *	\return SUCCESSFUL_RETURN, \n
		 *	        -RET_INDEX_OUT_OF_BOUNDS
		 */
		int getStateBoundComponent(	uint idx
									) const;


		/** Returns lower bounds on initial state x0.
		 *
		 *	\return Vector of lower bounds on initial state x0.
		 */
		Vector getLowerBoundsX0( ) const;

		/** Returns upper bounds on initial state x0.
		 *
		 *	\return Vector of upper bounds on initial state x0.
		 */
		Vector getUpperBoundsX0( ) const;


		/** Returns whether initial value of the differential states is fixed.
		 *
		 *	\return BT_TRUE  iff initial value is fixed, \n
		 *	        BT_FALSE otherwise
		 */
		BooleanType isInitialStateFixed( ) const;


	protected:

		/** Copies all class members from given object.
		 *
		 *	@param[in] arg		Right-hand side object.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue copy(	const QPLinAlgExport& arg
							);

		/** Frees internal dynamic memory to yield an empty function.
		 *
		 *	\return SUCCESSFUL_RETURN
		 */
		returnValue clear( );

		/** Setup multiplication routines
 		 * 
 		 * 	\return SUCCESSFUL_RETURN
 		 */
		returnValue setupMultiplicationRoutines( );


		// TODO: All member variables should be preinitialized!
//		returnValue setupVariables( );




    protected:
		

		VariablesGrid xBounds;			/**< Stores lower/upper bounds on differential states. */
		int* xBoundsIdx;				/**< Indices of finite lower/upper bounds on differential states. */
		uint nxBounds;					/**< Number of finite lower/upper bounds on differential states. */     
        
		ExportVariable x;				/**< Variable containing the exported differential states. */
		ExportVariable u;				/**< Variable containing the exported control inputs. */
		ExportVariable p;				/**< Variable containing the exported parameters. */
		ExportVariable xRef;			/**< Variable containing the exported reference trajectory for differential states. */
		ExportVariable uRef;			/**< Variable containing the exported reference trajectory for control inputs. */
		ExportVariable x0Ref;			/**< Variable containing the exported first reference trajectory for initial value. */
		ExportVariable x0Ref2;			/**< Variable containing the exported second reference trajectory for initial value. */

		ExportVariable Q;				/**< Weighting matrix for differential states (along the prediction horizon). */
		ExportVariable R;				/**< Weighting matrix for control inputs (along the control horizon). */
		ExportVariable QF;				/**< Weighting matrix for differential states at end of prediction horizon. */
		ExportVariable QS;				/**< First weighting matrix for free initial value. */
		ExportVariable QS2;				/**< Second weighting matrix for free initial value. */
		ExportVariable QQF;				/**< Variable containing the sum of Q and QF. */
		ExportVariable Qx;

		ExportVariable state;			/**< Variable containing the augmented state vector to call the integrator. */
		ExportVariable residuum;		/**< Variable containing multi-shooting residuum. */
		ExportVariable g0;				/**< Variable containing gradient of Lagrange function w.r.t. initial state. */
		ExportVariable g1;				/**< Variable containing gradient of Lagrange function w.r.t. control inputs. */
		ExportVariable H00;				/**< Variable containing Hessian approximation of Lagrange function w.r.t. to initial state. */
		ExportVariable H01;				/**< Variable containing Hessian approximation of Lagrange function w.r.t. to initial state/control inputs. */
		ExportVariable H11;				/**< Variable containing Hessian approximation of Lagrange function w.r.t. to control inputs. */
		ExportVariable lbA;				/**< Variable containing user-defined lower limits on differential states. */
		ExportVariable ubA;				/**< Variable containing user-defined upper limits on differential states. */
		ExportVariable d;				/**< Variable containing condensed multi-shooting residuum. */
		ExportVariable deltaX0;			/**< Variable containing difference between current measurement and current initial state. */
		ExportVariable C;				/**< Variable containing sensitivities w.r.t. to initial state. */
		ExportVariable QC;				/**< Variable containing sensitivities w.r.t. to initial state multiplied with objective weight Q. */
		ExportVariable Gx;				/**< Variable containing sensitivities w.r.t. to initial state on current interval. */
		ExportVariable E;				/**< Variable containing sensitivities w.r.t. to control inputs. */
		ExportVariable QE;				/**< Variable containing sensitivities w.r.t. to control inputs multiplied with objective weight Q. */
		ExportVariable Gu;				/**< Variable containing sensitivities w.r.t. to control inputs on current interval. */
		ExportVariable Dx0;				/**< Variable containing difference between current initial state and its first reference value. */
		ExportVariable Dx0b;			/**< Variable containing difference between current initial state and its second reference value. */
		ExportVariable Dx;				/**< Variable containing difference between current differential states and their reference values. */
		ExportVariable QDx;				/**< Variable containing difference between current differential states and their reference values multiplied with objective weight Q. */
		ExportVariable Du;				/**< Variable containing difference between current control inputs and their reference values. */
		ExportVariable RDu;				/**< Variable containing difference between current control inputs and their reference values multiplied with objective weight R. */

		ExportVariable deltaU;			/**< Variable containing the primal QP variables. */

		ExportFunction condense1;		/**< Function implementing the first part of the condensing algorithm that calls the integrator. */
		ExportFunction condense2;		/**< Function implementing the second part of the condensing algorithm that actually delivers the data of the condensed QP. */
		ExportFunction expand;			/**< Function implementing the expand step to recover all state variables from condensed QP solution. */
		ExportFunction setupQP;			/**< Function that coordinates the calls to condese1, condense2. */

		ExportFunction multiplyQx_exported; /**< Function implementing the calculation of Q*x. */
		
		ExportFunction getObjectiveValue; /**< Function implementing calculation of objective value. */
};


CLOSE_NAMESPACE_ACADO


#endif  // ACADO_TOOLKIT_QP_LIN_ALG_EXPORT_HPP

// end of file.
