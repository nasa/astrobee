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
 *    \file   examples/code_generation/crane_mpc.cpp
 *    \author Boris Houska, Hans Joachim Ferreau, Stephane Coulier
 *    \date   2010
 */


#include <acado_code_generation.hpp>


int main( ){

    USING_NAMESPACE_ACADO

    // DEFINE THE VARIABLES:
    // ----------------------------------------------------------
       DifferentialState   x    ;  // the trolley position
       DifferentialState   v    ;  // the trolley velocity 
       DifferentialState   phi  ;  // the excitation angle
       DifferentialState   omega;  // the angular velocity
       Control             ax   ;  // the acc. of the trolley
       Parameter           pp;

       const double     g = 9.81;  // the gravitational constant 
       const double     b = 0.20;  // the friction coefficient
    // ----------------------------------------------------------


    // DEFINE THE MODEL EQUATIONS:
    // ----------------------------------------------------------
       DifferentialEquation  f                                  ; 

       f << dot(  x    )  ==  v+pp                              ;
       f << dot(  v    )  ==  ax                                ;
       f << dot( phi   )  ==  omega                             ;
       f << dot( omega )  == -g*sin(phi) - ax*cos(phi) - b*omega;
    // ----------------------------------------------------------


    // DEFINE THE WEIGHTING MATRICES:
    // ----------------------------------------------------------
        ExportVariable Q = eye(4);
//       ExportVariable Q( "Q",4,4 );
       Matrix S  = eye(4);
//        Matrix R  = eye(1);
       ExportVariable R( "R",1,1 );
    // ----------------------------------------------------------

    // SET UP THE MPC - OPTIMAL CONTROL PROBLEM:
    // ----------------------------------------------------------
	   Grid grid( 0.0, 3.0, 11 );
       OCP ocp( grid );

       ocp.minimizeLSQ       ( Q, R );
       ocp.minimizeLSQEndTerm( S    );


	   ExportVariable QS1( "QS1",4,4 );
	   
	   ExportVariable QS2( "QS2",4,4 );
	   Matrix fixedMatrix = zeros( 4,4 );
	   QS2 = fixedMatrix;

	   ocp.minimizeLSQStartTerm( QS1,fixedMatrix );
// 	   ocp.minimizeLSQStartTerm( QS1,QS2 );


       ocp.subjectTo( f );
       ocp.subjectTo( -1.0 <= ax <= 1.0 );

// 	   ocp.subjectTo( AT_END, x     == 0.0 );
// 	   ocp.subjectTo( AT_END, v     == 0.0 );
// 	   ocp.subjectTo( AT_END, phi   == 0.0 );
// 	   ocp.subjectTo( AT_END, omega == 0.0 );

// 		   ocp.subjectTo( -0.5 <= v <= 1.5 );
		
    // ----------------------------------------------------------


	 // DEFINE AN MPC EXPORT MODULE AND GENERATE THE CODE:
	 // ----------------------------------------------------------
		MPCexportBis mpc(ocp);

//		mpc.set( INTEGRATOR_TYPE, INT_RK4 );
//		mpc.set( NUM_INTEGRATOR_STEPS, 30 );
//		mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
//		mpc.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
		mpc.set( MAX_NUM_QP_ITERATIONS, 40 );
		mpc.set( HOTSTART_QP, NO );
		mpc.set( QP_SOLVER, QP_QPOASES3 );
		mpc.set( SPARSE_QP_SOLUTION, CONDENSING );
		mpc.set( FIX_INITIAL_STATE, NO );
		mpc.set( LEVENBERG_MARQUARDT, 0.001 );
		mpc.set( GENERATE_TEST_FILE, NO );
// 		mpc.set( GENERATE_SIMULINK_INTERFACE, YES );
// 		mpc.set( OPERATING_SYSTEM, OS_WINDOWS );
// 		mpc.set( USE_SINGLE_PRECISION, YES );
		mpc.set( PRINTLEVEL, HIGH );

// 		mpc.printDimensionsQP( );
		mpc.exportCode( "./crane_bis_export" );
	// ----------------------------------------------------------


	return 0;
}



