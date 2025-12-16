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
 *    \file   simple_compressor.cpp
 *    \author Hans Joachim Ferreau, Milan Vukov, Janick Frasch
 *    \date   2013
 */

#include <acado_toolkit.hpp>
// #include <ocp_export.hpp>


#define smoothAbs( X )  ( sqrt( pow((X),2) + 1e-4 ) )
#define smoothMax( X )  ( ((X)+smoothAbs(X))/2.0 )


#define p1_scaling 1.0e-5
#define p1_scaled (p1/p1_scaling)

#define p2_scaling 1.0e-5
#define p2_scaled (p2/p2_scaling)

#define omega_comp_scaling 1.0e-3
#define omega_comp_scaled (omega_comp/omega_comp_scaling)

USING_NAMESPACE_ACADO

int main(int argc, char * const argv[ ])
{
#ifdef RUN_BENCHMARKS
#warning "Compiling for benchmarks!"
	if (argc != 2)
	{
		std::cout << "Number of horizon intervals is needed!" << std::endl;
		
		return 1;
	}
	
	const int N = atoi( static_cast< char* >(argv[ 1 ]) );
	
	std::cout << "Used a horizon length of " << N << std::endl;
#else
	// Set the horizon length here
	const int N = 20;
	std::cout << "Horizon length is set to: " << N << std::endl;
#endif /* RUN_BENCHMARKS */

    // Define a Right-Hand-Side:
    // -------------------------

    DifferentialState p1, p2, m_comp, p_ratio, omega_comp, m_rec;

	Control Recycle_opening, delta_torque_drive;
	Parameter torque_drive, Inflow_opening, Outflow_opening;
	
    IntermediateState m_in_ss, m_rec_ss, m_out_tmp, m_out_ss, dp, torque_comp;

	// Constants
	const double SpeedSound = 340.0;
	const double In_pres    = 100.0e3;
	const double Out_pres   = 100.0e3;

	const double Valve_in_gain  = 1.0 / 250.0;
	const double Valve_rec_gain = 1.0 / 350.0;
	const double Valve_out_gain = 1.0 / 250.0;

	const double VolumeT1 = 0.0412;
    const double VolumeT2 = 0.5953;
    const double AdivL =    0.0017;

	// Compressor
	const double J          = 0.607;
	const double tauComp    = 1.0/0.001;
	const double tauRecycle = 1.0/1.0;


	// Algebraic Equations
	m_in_ss   = Valve_in_gain*Inflow_opening * (In_pres - p1_scaled) / sqrt( smoothAbs(In_pres - p1_scaled)   );
	m_rec_ss  = Valve_rec_gain*Recycle_opening * (p2_scaled - p1_scaled)/ sqrt( smoothAbs(p2_scaled - p1_scaled) );
	m_out_tmp = Valve_out_gain*Outflow_opening * (p2_scaled - Out_pres) / sqrt( smoothAbs(p2_scaled - Out_pres)  );
	m_out_ss  = smoothMax( m_out_tmp );


	// compressor pressure ratio
	dp = 0.729222822119477 + 0.826310582074149*m_comp + 0.000787832484878*omega_comp_scaled - 3.474778296522684*m_comp*m_comp - 0.000132556329124*m_comp*omega_comp_scaled + 0.000000660458389*omega_comp_scaled*omega_comp_scaled;

	// compressor torque
	torque_comp = (3.78792353463046*omega_comp_scaled - 2632.50721525659)*m_comp*m_comp*m_comp + (1047.38222897593 - 1.12328712732367*omega_comp_scaled)*m_comp*m_comp + (0.0780410504794268*omega_comp_scaled - 57.1317533416153)*m_comp + 0.0677138383285905*omega_comp_scaled - 17.4964260445939;


	// RHS
	DifferentialEquation f;

	f << dot(p1)         == p1_scaling * ( SpeedSound * SpeedSound / VolumeT1 * (m_in_ss + m_rec - m_comp) );
	f << dot(p2)         == p2_scaling * ( SpeedSound * SpeedSound / VolumeT2 * (m_comp - m_rec - m_out_ss) );
	f << dot(m_comp)     == AdivL * (p_ratio * p1_scaled - p2_scaled);
	f << dot(p_ratio)    == tauComp * (dp - p_ratio);
	f << dot(omega_comp) == omega_comp_scaling * ( 1.0/J * (torque_drive + delta_torque_drive - torque_comp) );
	f << dot(m_rec)      == tauRecycle * (m_rec_ss - m_rec);


	// DEFINE THE WEIGHTING MATRICES:
	// ----------------------------------------------------------
    // ----------------------------------------------------------
    
//     Matrix S = eye(6 + 2);
// 	Matrix SN = eye( 6 );
	DMatrix S = eye<double>(6 + 2);
	DMatrix SN = eye<double>(6);
	
	S(0, 0) = 1.0e-1;
	S(1, 1) = 1.0e-1;
	S(2, 2) = 1.0e+2;
	S(3, 3) = 1.0e+2;
	S(4, 4) = 1.0e-1;
	S(5, 5) = 1.0e-1;
	
	S(6, 6) = 1.0e-1;
	S(7, 7) = 1.0e-4;
	
	SN(0, 0) = S(0, 0);
	SN(1, 1) = S(1, 1);
	SN(2, 2) = S(2, 2);
	SN(3, 3) = S(3, 3);
	SN(4, 4) = S(4, 4);
	SN(5, 5) = S(5, 5);

    Function h, hN;
    
    h << p1 << p2 << m_comp << p_ratio << omega_comp << m_rec;
    h << Recycle_opening << delta_torque_drive;
    
	hN << p1 << p2 << m_comp << p_ratio << omega_comp << m_rec;

	// SET UP THE MPC - OPTIMAL CONTROL PROBLEM:
	// ----------------------------------------------------------
	OCP ocp(0.0, 0.25, N);
// 	OCP ocp(0.0, 0.5, 10);
		
	ocp.minimizeLSQ(S, h);
	ocp.minimizeLSQEndTerm (SN, hN);

	ocp.subjectTo( f );
		
	ocp.subjectTo(  -10.0e0 <= delta_torque_drive <= 10.0e0 );
	//ocp.subjectTo(    0.0   <= Recycle_opening    <= 0.0   );
	//ocp.subjectTo(    0.0e0 <= delta_torque_drive <= 0.0e0 );

	// ----------------------------------------------------------


	// DEFINE AN MPC EXPORT MODULE AND GENERATE THE CODE:
	// ----------------------------------------------------------
	OCPexport mpc( ocp );

// 	mpc.set( INTEGRATOR_TYPE      , INT_IRK_GL4  );
// 	mpc.set( NUM_INTEGRATOR_STEPS , 250 );

	mpc.set( DISCRETIZATION_TYPE,         MULTIPLE_SHOOTING );

// 	mpc.set( INTEGRATOR_TYPE      , INT_RK4  );
// 	mpc.set( NUM_INTEGRATOR_STEPS , 1000 );
	mpc.set( INTEGRATOR_TYPE      , INT_IRK_GL4  );
	mpc.set( NUM_INTEGRATOR_STEPS , 1*N );
	
	mpc.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );

// 	mpc.set( SPARSE_QP_SOLUTION,		  FULL_CONDENSING );
// 	mpc.set( SPARSE_QP_SOLUTION,		  FULL_CONDENSING_N2 );
//	mpc.set( SPARSE_QP_SOLUTION,		  CONDENSING );
// 	mpc.set( QP_SOLVER,                   QP_QPOASES      );

	mpc.set( SPARSE_QP_SOLUTION,		  SPARSE_SOLVER );
//	mpc.set( QP_SOLVER,                   QP_FORCES      );
	mpc.set( QP_SOLVER,                   QP_QPDUNES     );


	mpc.set( GENERATE_TEST_FILE,    NO );
	mpc.set( GENERATE_MAKE_FILE,    NO );
	mpc.set( PRINTLEVEL, HIGH );
// 	mpc.set( USE_SINGLE_PRECISION, YES );
// 	mpc.set( GENERATE_SIMULINK_INTERFACE, NO );
// 	mpc.set( OPERATING_SYSTEM,      OS_WINDOWS );

	mpc.exportCode( "simple_compressor_export" );
	// ----------------------------------------------------------

    return 0;
}
