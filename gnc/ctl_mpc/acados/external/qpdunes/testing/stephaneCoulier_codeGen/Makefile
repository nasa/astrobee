##
##    This file is part of ACADO Toolkit.
##
##    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
##    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
##    Developed within the Optimization in Engineering Center (OPTEC) under
##    supervision of Moritz Diehl. All rights reserved.
##
##    ACADO Toolkit is free software; you can redistribute it and/or
##    modify it under the terms of the GNU Lesser General Public
##    License as published by the Free Software Foundation; either
##    version 3 of the License, or (at your option) any later version.
##
##    ACADO Toolkit is distributed in the hope that it will be useful,
##    but WITHOUT ANY WARRANTY; without even the implied warranty of
##    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
##    Lesser General Public License for more details.
##
##    You should have received a copy of the GNU Lesser General Public
##    License along with ACADO Toolkit; if not, write to the Free Software
##    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##
##



##
##    Filename:  examples/code_generation/Makefile
##    Author:    Boris Houska and Hans Joachim Ferreau
##    Date:      2010
##


##
##    include global settings
##

LOCAL_PATH_PREFIX = ../..
include ${LOCAL_PATH_PREFIX}/include/acado/include.mk

##
##    flags
##


IFLAGS    =  ${HEADER_PATHS}
CPPFLAGS  =  ${CPP_GLOBAL_FLAGS} ${NO_PARENTHESES_WARNING}

LIBS      =  ${CODE_GENERATION_LIBS}
#CLOSED_LOOP_LIBS =  ${TOOLKIT_LIBS} -L./getting_started_export ${lib}acado_exported_rti
CLOSED_LOOP_LIBS =  ${TOOLKIT_LIBS} -L./kite_export ${lib}acado_exported_rti

SRCS = \
	getting_started.cpp \
	crane.cpp \
	crane_bis.cpp \
	cstr.cpp \
	cstr22.cpp \
	kite_carousel.cpp

DEV_SRCS = \
	lieboud_crane.cpp \
	lieboud_crane2.cpp

CLOSED_LOOP_SRCS = \
	dev_kite_carousel.cpp

#getting_started_closed_loop.cpp 


OBJS = $(SRCS:.cpp=.${OBJEXT})
EXECS = $(SRCS:.cpp=${EXE})

DEV_OBJS = $(DEV_SRCS:.cpp=.${OBJEXT})
DEV_EXECS = $(DEV_SRCS:.cpp=${EXE})

CLOSED_LOOP_OBJS = $(CLOSED_LOOP_SRCS:.cpp=.${OBJEXT})
CLOSED_LOOP_EXECS = $(CLOSED_LOOP_SRCS:.cpp=${EXE})


##
##    targets
##

ifeq ($(VERBOSE),NO)
.SILENT:
endif

all: ${EXECS}

developer: ${EXECS} ${DEV_EXECS}

closed_loop: ${CLOSED_LOOP_EXECS}


${EXECS} : %${EXE} : %.${OBJEXT}
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${LIBS}

${DEV_EXECS} : %${EXE} : %.${OBJEXT}
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${LIBS}

${CLOSED_LOOP_EXECS} : %${EXE} : %.${OBJEXT}
	@echo "Creating" $@
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${CLOSED_LOOP_LIBS}

${OBJS} : %.${OBJEXT}: %.cpp
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<

${DEV_OBJS} : %.${OBJEXT}: %.cpp
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<

${CLOSED_LOOP_OBJS} : %.${OBJEXT}: %.cpp
	${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<

clean:
	${RM} -f ${OBJS} ${EXECS} ${DEV_OBJS} ${DEV_EXECS} ${CLOSED_LOOP_OBJS} ${CLOSED_LOOP_EXECS}

clobber: clean


.PHONY: all developer closed_loop clean clobber



##
##    end of file
##
