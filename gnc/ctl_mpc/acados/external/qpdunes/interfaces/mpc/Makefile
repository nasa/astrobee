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
##	Filename:  src/Makefile
##	Author:    Janick Frasch, Hans Joachim Ferreau
##	Version:   1.0beta
##	Date:      2012
##

SRCDIR = ../../src

# select your operating system here!
include ../../make_linux.mk
#include ../../make_windows.mk


##
##	flags
##

IFLAGS      =  -I. \
               -I../../include


MPCDUNES_OBJECTS = \
	setup_mpc.${OBJEXT}



##
##	targets
##

all: libmpcdunes.${LIBEXT}


libmpcdunes.${LIBEXT}: ${MPCDUNES_OBJECTS}
	${AR} r $@ $?


clean:
	${RM} -f *.${OBJEXT} *.${LIBEXT}

clobber: clean


%.${OBJEXT}: %.c
	@echo "Creating" $@
	${CC} ${DEF_TARGET} ${IFLAGS} ${CCFLAGS} -c $<


##
##	end of file
##
