##
##	This file is part of qpDUNES.
##
##	qpDUNES -- A DUal NEwton Strategy for convex quadratic programming.
##	Copyright (C) 2012 by Janick Frasch, Hans Joachim Ferreau et al. 
##	All rights reserved.
##
##	qpDUNES is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qpDUNES is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##	See the GNU Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qpDUNES; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##	Filename:  src/Makefile
##	Author:    Janick Frasch, Hans Joachim Ferreau
##	Version:   1.0beta
##	Date:      2012
##

SRCDIR = .

# select your operating system here!
include ../make_linux.mk
#include ../make_windows.mk


##
##	flags
##

IDIR        =  ../include

IDIR_QPOASES = ../externals/qpOASES-3.0beta/include

IFLAGS      =  -I. \
               -I${IDIR} \
               -I${IDIR_QPOASES}


QPDUNES_OBJECTS = \
	dual_qp.${OBJEXT} \
	stage_qp_solver_clipping.${OBJEXT} \
	stage_qp_solver_qpoases.${OBJEXT} \
	matrix_vector.${OBJEXT} \
	setup_qp.${OBJEXT} \
	qpdunes_utils.${OBJEXT}



##
##	targets
##

all: libqpdunes.${LIBEXT}


libqpdunes.${LIBEXT}: ${QPDUNES_OBJECTS}
	${AR} r $@ $?

dual_qp.${OBJEXT}: \
	dual_qp.c \
	${IDIR}/qp/dual_qp.h \
	${IDIR}/qp/stage_qp_solver_clipping.h \
	${IDIR}/qp/stage_qp_solver_qpoases.hpp \
	${IDIR}/qp/matrix_vector.h \
	${IDIR}/qp/setup_qp.h \
	${IDIR}/qp/qpdunes_utils.h \
	${IDIR}/qp/types.h 
	@echo "Creating" $@
	${CC} ${DEF_TARGET} -c ${IFLAGS} ${CCFLAGS} dual_qp.c
	
stage_qp_solver_clipping.${OBJEXT}: \
	stage_qp_solver_clipping.c \
	${IDIR}/qp/stage_qp_solver_clipping.h \
	${IDIR}/qp/qpdunes_utils.h \
	${IDIR}/qp/types.h
	@echo "Creating" $@
	${CC} ${DEF_TARGET} -c ${IFLAGS} ${CCFLAGS} stage_qp_solver_clipping.c
	
stage_qp_solver_qpoases.${OBJEXT}: \
	stage_qp_solver_qpoases.cpp \
	${IDIR}/qp/stage_qp_solver_qpoases.hpp \
	${IDIR}/qp/qpdunes_utils.h \
	${IDIR}/qp/types.h
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} stage_qp_solver_qpoases.cpp

matrix_vector.${OBJEXT}: \
	matrix_vector.c \
	${IDIR}/qp/matrix_vector.h \
	${IDIR}/qp/qpdunes_utils.h \
	${IDIR}/qp/types.h
	@echo "Creating" $@
	${CC} ${DEF_TARGET} -c ${IFLAGS} ${CCFLAGS} matrix_vector.c

setup_qp.${OBJEXT}: \
	setup_qp.c \
	${IDIR}/qp/qpdunes_utils.h \
	${IDIR}/qp/types.h
	@echo "Creating" $@
	${CC} ${DEF_TARGET} -c ${IFLAGS} ${CCFLAGS} setup_qp.c


clean:
	${RM} -f *.${OBJEXT} *.${LIBEXT}

clobber: clean


%.${OBJEXT}: %.c
	@echo "Creating" $@
	${CC} ${DEF_TARGET} ${IFLAGS} ${CCFLAGS} -c $<


##
##	end of file
##
