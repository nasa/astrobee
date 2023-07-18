
################################################################################
#
# Description:
#	OCG2 package configuration file
#
#
# Authors:
#	Milan Vukov, milan.vukov@esat.kuleuven.be
#
# Year:
#	2013.
#
# NOTE:
#	- This script is for Linux/Unix use only.
#
#	- PREREQUISITE: sourced ocg2_env.sh in your ~/.bashrc file. This script
#		will try to find OCG2 folders, libraries etc., but looking for them
#		in environmental variables.
#
# Usage:
#	- Linux/Unix: TODO
#
################################################################################

################################################################################
#
# Search for package components
#
################################################################################

MESSAGE( STATUS "********************************************************************************" )
MESSAGE( STATUS "Looking for OCG2 package: \n" )

#
# Include folders
#
MESSAGE( STATUS "Looking for OCG2 include directories" )
SET( OCG2_INCLUDE_DIRS $ENV{OCG2_ENV_INCLUDE_DIRS} )
IF( OCG2_INCLUDE_DIRS )
	MESSAGE( STATUS "Found OCG2 include directories: ${OCG2_INCLUDE_DIRS} \n" )
	SET( OCG2_INCLUDE_DIRS_FOUND TRUE )
ELSE( OCG2_INCLUDE_DIRS )
	MESSAGE( STATUS "Could not find OCG2 include directories \n" )
ENDIF( OCG2_INCLUDE_DIRS )

#
# Library folders
#
MESSAGE( STATUS "Looking for OCG2 library directories" )
SET( OCG2_LIBRARY_DIRS $ENV{OCG2_ENV_LIBRARY_DIRS} )
IF( OCG2_LIBRARY_DIRS )
	MESSAGE( STATUS "Found OCG2 library directories: ${OCG2_LIBRARY_DIRS} \n" )
	SET( OCG2_LIBRARY_DIRS_FOUND TRUE )
ELSE( OCG2_LIBRARY_DIRS )
	MESSAGE( STATUS "Could not find OCG2 library directories \n" )
ENDIF( OCG2_LIBRARY_DIRS )

#
# Shared libraries
#
FIND_LIBRARY( OCG2_SHARED_LIBRARIES
	NAMES ocg2
	PATHS ${OCG2_LIBRARY_DIRS}
	NO_DEFAULT_PATH
)
IF( OCG2_SHARED_LIBRARIES )
	MESSAGE( STATUS "Found OCG2 shared library: ocg2\n" )
ELSE( OCG2_SHARED_LIBRARIES )
	MESSAGE( STATUS "Could not find OCG2 shared library: ocg2\n" )
	SET( OCG2_SHARED_LIBS_FOUND FALSE )
ENDIF( OCG2_SHARED_LIBRARIES )

#
# And finally set found flag...
#
IF( OCG2_INCLUDE_DIRS_FOUND AND OCG2_LIBRARY_DIRS_FOUND 
		AND OCG2_SHARED_LIBS_FOUND )
	SET( OCG2_FOUND TRUE )
ENDIF()

MESSAGE( STATUS "********************************************************************************" )