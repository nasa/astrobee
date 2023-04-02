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
 *    \file include/code_generation/code_generation.hpp
 *    \author Hans Joachim Ferreau, Boris Houska, Stephane Coulier
 *    \date 2010-2011
 */

#ifndef ACADO_TOOLKIT_CODE_GENERATION_HPP
#define ACADO_TOOLKIT_CODE_GENERATION_HPP


// LIST OF HEADER FILES:
// -----------------------------------------------------

   #include <acado/code_generation/integrator_export.hpp>
   #include <acado/code_generation/rk_export.hpp>
   #include <acado/code_generation/erk_export.hpp>
   #include <acado/code_generation/explicit_euler_export.hpp>
   #include <acado/code_generation/erk2_export.hpp>
   #include <acado/code_generation/erk3_export.hpp>
   #include <acado/code_generation/erk4_export.hpp>
   #include <acado/code_generation/irk_export.hpp>
   #include <acado/code_generation/gauss_legendre2_export.hpp>
   #include <acado/code_generation/gauss_legendre4_export.hpp>
   #include <acado/code_generation/gauss_legendre6_export.hpp>
   #include <acado/code_generation/gauss_legendre8_export.hpp>
   #include <acado/code_generation/radau_IIA1_export.hpp>
   #include <acado/code_generation/radau_IIA3_export.hpp>
   #include <acado/code_generation/radau_IIA5_export.hpp>
   #include <acado/code_generation/linear_solver_export.hpp>
   #include <acado/code_generation/gaussian_elimination_export.hpp>
   #include <acado/code_generation/householder_qr_export.hpp>
   #include <acado/code_generation/condensing_export.hpp>
   #include <acado/code_generation/gauss_newton_export.hpp>
   #include <acado/code_generation/auxiliary_functions_export.hpp>
   #include <acado/code_generation/export_algorithm.hpp>
   #include <acado/code_generation/mpc_export.hpp>
   #include <acado/code_generation/sim_export.hpp>
   #include <acado/code_generation/export_module.hpp>
   #include <acado/code_generation/export_data.hpp>
   #include <acado/code_generation/export_index.hpp>
   #include <acado/code_generation/export_variable.hpp>
   #include <acado/code_generation/export_function.hpp>
   #include <acado/code_generation/export_ode_function.hpp>
   #include <acado/code_generation/export_statement.hpp>
   #include <acado/code_generation/export_arithmetic_statement.hpp>
   #include <acado/code_generation/export_statement_block.hpp>
   #include <acado/code_generation/export_file.hpp>
   #include <acado/code_generation/export_for_loop.hpp>
   #include <acado/code_generation/export_argument_list.hpp>
   #include <acado/code_generation/export_function_call.hpp>
   #include <acado/code_generation/export_statement_string.hpp>
   #include <acado/code_generation/export_data_declaration.hpp>
   #include <acado/code_generation/export_function_declaration.hpp>
   #include <acado/code_generation/export_printf.hpp>
   #include <acado/code_generation/qp_lin_alg_export.hpp>
   #include <acado/code_generation/mpc_export_bis.hpp>

// -----------------------------------------------------


#endif  // ACADO_TOOLKIT_CODE_GENERATION_HPP
