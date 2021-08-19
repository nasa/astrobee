#### Taken from http://www.openflipper.org/svnrepo/CoMISo/trunk/CoMISo/cmake/FindGUROBI.cmake
# - Try to find GUROBI
# Once done this will define
# GUROBI_FOUND - System has Gurobi
# GUROBI_INCLUDE_DIRS - The Gurobi include directories
# GUROBI_LIBRARIES - The libraries needed to use Gurobi

find_path(GUROBI_INCLUDE_DIR
  NAMES gurobi_c++.h
  PATHS "$ENV{GUROBI_HOME}/include"
  )

find_library(GUROBI_LIBRARY
  NAMES gurobi
  gurobi70
  gurobi65
  gurobi60
  PATHS "$ENV{GUROBI_HOME}/lib"
  )

find_library(GUROBI_CXX_LIBRARY
  NAMES gurobi_c++
  PATHS "$ENV{GUROBI_HOME}/lib"
  )

set(GUROBI_INCLUDE_DIRS "${GUROBI_INCLUDE_DIR}")
set(GUROBI_LIBRARIES "${GUROBI_CXX_LIBRARY};${GUROBI_LIBRARY}" )

# use c++ headers as default
# set(GUROBI_COMPILER_FLAGS "-DIL_STD" CACHE STRING "Gurobi Compiler Flags")

# handle the QUIETLY and REQUIRED arguments and set GUROBI_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(GUROBI DEFAULT_MSG
  GUROBI_LIBRARY GUROBI_CXX_LIBRARY GUROBI_INCLUDE_DIR)
mark_as_advanced(GUROBI_INCLUDE_DIR GUROBI_LIBRARY GUROBI_CXX_LIBRARY)
