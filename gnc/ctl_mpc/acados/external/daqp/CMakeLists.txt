cmake_minimum_required(VERSION 3.10)
project(daqp VERSION 0.4.0)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/codegen)
add_subdirectory(src)
add_subdirectory(include)
add_subdirectory(codegen)

option (PROFILING "Enable profiling" ON)
if(PROFILING)
  add_compile_definitions(PROFILING)
endif()

option (SOFT_WEIGHTS "Use different weights/bounds for soft constraints slacks" OFF)
if(SOFT_WEIGHTS)
  add_compile_definitions(SOFT_WEIGHTS)
endif()

add_library(daqpstat STATIC ${daqp_src} ${daqp_headers} ${daqp_codegen_src} ${daqp_codegen_headers})
add_library(daqp SHARED ${daqp_src} ${daqp_headers} ${daqp_codegen_src} ${daqp_codegen_headers})

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_C_STANDARD_LIBRARIES "${CMAKE_C_STANDARD_LIBRARIES}") # math for C api
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3 -Wall -Wextra")

target_link_libraries(daqpstat daqp)

# Install
include(GNUInstallDirs)
install(TARGETS daqpstat EXPORT  ${PROJECT_NAME}
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(TARGETS daqp EXPORT  ${PROJECT_NAME}
  ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
  RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}")
install(FILES ${daqp_headers} ${daqp_codegen_headers} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/daqp")

install(EXPORT ${PROJECT_NAME} DESTINATION cmake)

if(MATLAB)
  find_package(Matlab)
  set(MATLAB_INTERFACE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/interfaces/daqp-matlab")
  matlab_add_mex(NAME daqpmex SRC ${MATLAB_INTERFACE_DIR}/daqpmex.c LINK_TO daqpstat)
  set_target_properties(daqpmex PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${MATLAB_INTERFACE_DIR})
endif()

if(MATLABTEST)
  find_package(Matlab REQUIRED MAIN_PROGRAM)
  enable_testing()
  matlab_add_unit_test(NAME core_test UNITTEST_FILE ${MATLAB_INTERFACE_DIR}/test/core_test.m ADDITIONAL_PATH ${MATLAB_INTERFACE_DIR} ${MATLAB_INTERFACE_DIR}/utils)
endif()
