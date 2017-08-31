# Copyright (c) 2017, United States Government, as represented by the
# Administrator of the National Aeronautics and Space Administration.
# 
# All rights reserved.
# 
# The Astrobee platform is licensed under the Apache License, Version 2.0
# (the "License"); you may not use this file except in compliance with the
# License. You may obtain a copy of the License at
# 
#     http://www.apache.org/licenses/LICENSE-2.0
# 
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations
# under the License.

find_path(INTEL_MKL_INCLUDES
  NAMES mkl.h
  PATHS "$ENV{MKLROOT}/include"
  )

find_library(INTEL_LP
  NAMES mkl_intel_lp64
  PATHS "$ENV{MKLROOT}/lib/intel64"
  )
find_library(INTEL_TBB
  NAMES mkl_sequential 
  PATHS "$ENV{MKLROOT}/lib/intel64"
  )
find_library(INTEL_CORE
  NAMES mkl_core
  PATHS "$ENV{MKLROOT}/lib/intel64"
  )

set(INTEL_MKL_LIBRARIES "${INTEL_LP};${INTEL_TBB};${INTEL_CORE}")

# use c++ headers as default
# set(GUROBI_COMPILER_FLAGS "-DIL_STD" CACHE STRING "Gurobi Compiler Flags")

# handle the QUIETLY and REQUIRED arguments and set GUROBI_FOUND to TRUE
# if all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(INTELMKL_DEFAULT_MSG
  INTEL_ILP INTEL_TBB INTEL_CORE INTEL_MKL_INCLUDES)
mark_as_advanced(INTEL_LP INTEL_TBB INTEL_CORE INTEL_MKL_INCLUDES)
