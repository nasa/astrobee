##
##	This file is part of qp42.
##
##	qp42 -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2012 by Janick Frasch, Hans Joachim Ferreau et al. 
##	All rights reserved.
##
##	qp42 is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qp42 is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##	See the GNU Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qp42; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##	Filename:  examples/Makefile
##	Author:    Janick Frasch, Hans Joachim Ferreau
##	Version:   1.0beta
##	Date:      2012
##

SRCDIR = ../src
INTERFACEDIR = ../interfaces
QPOASESDIR = ../externals/qpOASES-3.0beta

# select your operating system here!
include ../make_linux.mk
#include ../make_windows.mk


##
##	flags
##

IFLAGS      =  -I. \
               -I../include \
               -I../interfaces

QPDUNES_EXES = \
	example1${EXE} \
	example1_ltv${EXE} \
	example2_affine${EXE} \
	example3_affine_mpc${EXE} \
	wang2010${EXE} \
	singleIntegrator${EXE} \
	singleIntegrator_b${EXE} \
	singleIntegrator_sc${EXE} \
	singleIntegrator_d${EXE} \
	doubleIntegrator${EXE} \
	doubleIntegrator_qp${EXE} \
	doubleIntegrator_mpc${EXE} \
	mhePrototype${EXE}	\
	nmpcPrototype${EXE}	\
	nmpcPrototype_b${EXE}	\
	chainMass_M3_N50_1stStep${EXE}



##
##	targets
##

all: ${QPDUNES_EXES}


example1${EXE}: example1.${OBJEXT} ../src/libqpdunes.a ../externals/qpOASES-3.0beta/bin/libqpOASES.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}

example1_ltv${EXE}: example1_ltv.${OBJEXT} ../interfaces/mpc/libmpcdunes.a ../src/libqpdunes.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${MPCDUNES_LIB} ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}
	
example2_affine${EXE}: example2_affine.${OBJEXT} ../interfaces/mpc/libmpcdunes.a ../src/libqpdunes.a ../externals/qpOASES-3.0beta/bin/libqpOASES.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${MPCDUNES_LIB} ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}
	
example3_affine_mpc${EXE}: example3_affine_mpc.${OBJEXT} ../interfaces/mpc/libmpcdunes.a ../src/libqpdunes.a ../externals/qpOASES-3.0beta/bin/libqpOASES.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${MPCDUNES_LIB} ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}

wang2010${EXE}: wang2010.${OBJEXT} ../src/libqpdunes.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}
	
singleIntegrator${EXE}: singleIntegrator.${OBJEXT} ../interfaces/mpc/libmpcdunes.a ../src/libqpdunes.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${MPCDUNES_LIB} ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}
	
singleIntegrator_b${EXE}: singleIntegrator_b.${OBJEXT} ../interfaces/mpc/libmpcdunes.a ../src/libqpdunes.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${MPCDUNES_LIB} ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}
	
singleIntegrator_sc${EXE}: singleIntegrator_sc.${OBJEXT} ../interfaces/mpc/libmpcdunes.a ../src/libqpdunes.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${MPCDUNES_LIB} ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}
	
singleIntegrator_d${EXE}: singleIntegrator_d.${OBJEXT} ../interfaces/mpc/libmpcdunes.a ../src/libqpdunes.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${MPCDUNES_LIB} ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}

doubleIntegrator${EXE}: doubleIntegrator.${OBJEXT} ../interfaces/mpc/libmpcdunes.a ../src/libqpdunes.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${MPCDUNES_LIB} ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}
	
doubleIntegrator_qp${EXE}: doubleIntegrator_qp.${OBJEXT} ../interfaces/mpc/libmpcdunes.a ../src/libqpdunes.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${MPCDUNES_LIB} ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}

doubleIntegrator_mpc${EXE}: doubleIntegrator_mpc.${OBJEXT} ../interfaces/mpc/libmpcdunes.a ../src/libqpdunes.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${MPCDUNES_LIB} ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}
	
mhePrototype${EXE}: mhePrototype.${OBJEXT} ../src/libqpdunes.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}
	
nmpcPrototype${EXE}: nmpcPrototype.${OBJEXT} ../src/libqpdunes.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}
	
nmpcPrototype_b${EXE}: nmpcPrototype_b.${OBJEXT} ../src/libqpdunes.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}

chainMass_M3_N50_1stStep${EXE}: chainMass_M3_N50_1stStep.${OBJEXT} ../interfaces/mpc/libmpcdunes.a ../src/libqpdunes.a
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${MPCDUNES_LIB} ${QPDUNES_LIB} ${QPOASES_LIB} ${LIBS}



clean:
	${RM} -f *.${OBJEXT} ${QPDUNES_EXES}

clobber: clean


%.${OBJEXT}: %.c
	@echo "Creating" $@
	${CC} ${DEF_TARGET} ${IFLAGS} ${CCFLAGS} -c $<


%.${OBJEXT}: %.cpp
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${IFLAGS} ${CPPFLAGS} -c $<


##
##	end of file
##
