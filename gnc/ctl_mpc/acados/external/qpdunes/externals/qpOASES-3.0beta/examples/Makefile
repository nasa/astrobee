##
##	This file is part of qpOASES.
##
##	qpOASES -- An Implementation of the Online Active Set Strategy.
##	Copyright (C) 2007-2012 by Hans Joachim Ferreau, Andreas Potschka,
##	Christian Kirches et al. All rights reserved.
##
##	qpOASES is free software; you can redistribute it and/or
##	modify it under the terms of the GNU Lesser General Public
##	License as published by the Free Software Foundation; either
##	version 2.1 of the License, or (at your option) any later version.
##
##	qpOASES is distributed in the hope that it will be useful,
##	but WITHOUT ANY WARRANTY; without even the implied warranty of
##	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
##	See the GNU Lesser General Public License for more details.
##
##	You should have received a copy of the GNU Lesser General Public
##	License along with qpOASES; if not, write to the Free Software
##	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
##



##
##	Filename:  examples/Makefile
##	Author:    Hans Joachim Ferreau
##	Version:   3.0beta
##	Date:      2007-2012
##

include ../make.mk

##
##	flags
##

IDIR = ../include

IFLAGS      =  -I. \
               -I${IDIR}

QPOASES_EXES = \
	${BINDIR}/my_example${EXE} \
	${BINDIR}/my_example3${EXE} \
	${BINDIR}/my_example4${EXE} \
	${BINDIR}/example_weaklyActive${EXE} \
	${BINDIR}/example_infeas${EXE} \
	${BINDIR}/example1${EXE} \
	${BINDIR}/example1a${EXE} \
	${BINDIR}/example1b${EXE} \
	${BINDIR}/example2${EXE} \
	${BINDIR}/example3${EXE} \
	${BINDIR}/example3b${EXE} \
	${BINDIR}/example4${EXE} \
	${BINDIR}/example5${EXE} \
	${BINDIR}/example6${EXE} \
	${BINDIR}/exampleLP${EXE} \
	${BINDIR}/qrecipe${EXE} \
	${BINDIR}/runAllOqpExamples${EXE}

##
##	targets
##

all: ${QPOASES_EXES}

${BINDIR}/example1${EXE}: example1.${OBJEXT}
	@${ECHO} "Creating" $@
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}
	
${BINDIR}/my_example${EXE}: my_example.${OBJEXT}
	@${ECHO} "Creating" $@
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}
	
${BINDIR}/my_example3${EXE}: my_example3.${OBJEXT}
	@${ECHO} "Creating" $@
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}
	
${BINDIR}/my_example4${EXE}: my_example4.${OBJEXT}
	@${ECHO} "Creating" $@
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}
	
${BINDIR}/example_weaklyActive${EXE}: example_weaklyActive.${OBJEXT}
	@${ECHO} "Creating" $@
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}
	
${BINDIR}/example_infeas${EXE}: example_infeas.${OBJEXT}
	@${ECHO} "Creating" $@
	${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/example1a${EXE}: example1a.${OBJEXT}
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/example1b${EXE}: example1b.${OBJEXT}
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/example2${EXE}: example2.${OBJEXT} 
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/example3${EXE}: example3.${OBJEXT} 
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/example3b${EXE}: example3b.${OBJEXT}
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/example4${EXE}: example4.${OBJEXT} 
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/example5${EXE}: example5.${OBJEXT} 
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/example6${EXE}: example6.${OBJEXT} 
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}
	
${BINDIR}/example7${EXE}: example7.${OBJEXT} 
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/exampleLP${EXE}: exampleLP.${OBJEXT} 
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}

${BINDIR}/qrecipe${EXE}: qrecipe.${OBJEXT} 
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}
	
${BINDIR}/runAllOqpExamples${EXE}: runAllOqpExamples.${OBJEXT} 
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} ${CPPFLAGS} $< ${QPOASES_LINK} ${LINK_LIBRARIES}


clean:
	${RM} -f *.${OBJEXT} ${QPOASES_EXES}

clobber: clean


%.${OBJEXT}: %.cpp
	@${ECHO} "Creating" $@
	@${CPP} ${DEF_TARGET} -c ${IFLAGS} ${CPPFLAGS} $<


##
##	end of file
##
