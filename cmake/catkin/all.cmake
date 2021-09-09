# prevent multiple inclusion
if(DEFINED _CATKIN_ALL_INCLUDED_)
  message(FATAL_ERROR "catkin/cmake/all.cmake included multiple times")
endif()
set(_CATKIN_ALL_INCLUDED_ TRUE)

if(NOT DEFINED catkin_EXTRAS_DIR)
  message(FATAL_ERROR "catkin_EXTRAS_DIR is not set")
endif()

# define devel space
if(CATKIN_DEVEL_PREFIX)
  set(CATKIN_DEVEL_PREFIX ${CATKIN_DEVEL_PREFIX} CACHE PATH "catkin devel space")
else()
  set(CATKIN_DEVEL_PREFIX "${CMAKE_BINARY_DIR}/devel")
endif()
#message(STATUS "Using CATKIN_DEVEL_PREFIX: ${CATKIN_DEVEL_PREFIX}")

# update develspace marker file with a reference to this sourcespace
set(_catkin_marker_file "${CATKIN_DEVEL_PREFIX}/.catkin")

# check if the develspace marker file exists yet
if(EXISTS ${_catkin_marker_file})
  file(READ ${_catkin_marker_file} _existing_sourcespaces)
  if(_existing_sourcespaces STREQUAL "")
    # write this sourcespace to the marker file
    file(WRITE ${_catkin_marker_file} "${CMAKE_SOURCE_DIR}")
  else()
    # append to existing list of sourcespaces if it's not in the list
    list(FIND _existing_sourcespaces "${CMAKE_SOURCE_DIR}" _existing_sourcespace_index)
    if(_existing_sourcespace_index EQUAL -1)
      file(APPEND ${_catkin_marker_file} ";${CMAKE_SOURCE_DIR}")
    endif()
  endif()
else()
  # create a new develspace marker file
  # NOTE: extra care must be taken when running multiple catkin jobs in parallel 
  #       so that this does not overwrite the result of a similar call in another package
  file(WRITE ${_catkin_marker_file} "${CMAKE_SOURCE_DIR}")
endif()

# use either CMAKE_PREFIX_PATH explicitly passed to CMake as a command line argument
# or CMAKE_PREFIX_PATH from the environment
if(NOT DEFINED CMAKE_PREFIX_PATH)
  if(NOT "$ENV{CMAKE_PREFIX_PATH}" STREQUAL "")
    string(REPLACE ":" ";" CMAKE_PREFIX_PATH $ENV{CMAKE_PREFIX_PATH})
  endif()
endif()
#message(STATUS "Using CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")
# store original CMAKE_PREFIX_PATH
set(CMAKE_PREFIX_PATH_AS_IS ${CMAKE_PREFIX_PATH})

# list of unique catkin workspaces based on CMAKE_PREFIX_PATH
set(CATKIN_WORKSPACES "")
foreach(path ${CMAKE_PREFIX_PATH})
  if(EXISTS "${path}/.catkin")
    list(FIND CATKIN_WORKSPACES ${path} _index)
    if(_index EQUAL -1)
      list(APPEND CATKIN_WORKSPACES ${path})
    endif()
  endif()
endforeach()
if(CATKIN_WORKSPACES)
  #message(STATUS "This workspace overlays: ${CATKIN_WORKSPACES}")
endif()

# prepend devel space to CMAKE_PREFIX_PATH
list(FIND CMAKE_PREFIX_PATH ${CATKIN_DEVEL_PREFIX} _index)
if(_index EQUAL -1)
  list(INSERT CMAKE_PREFIX_PATH 0 ${CATKIN_DEVEL_PREFIX})
endif()


# enable all new policies (if available)
macro(_set_cmake_policy_to_new_if_available policy)
  if(POLICY ${policy})
    cmake_policy(SET ${policy} NEW)
  endif()
endmacro()
_set_cmake_policy_to_new_if_available(CMP0000)
_set_cmake_policy_to_new_if_available(CMP0001)
_set_cmake_policy_to_new_if_available(CMP0002)
_set_cmake_policy_to_new_if_available(CMP0003)
_set_cmake_policy_to_new_if_available(CMP0004)
_set_cmake_policy_to_new_if_available(CMP0005)
_set_cmake_policy_to_new_if_available(CMP0006)
_set_cmake_policy_to_new_if_available(CMP0007)
_set_cmake_policy_to_new_if_available(CMP0008)
_set_cmake_policy_to_new_if_available(CMP0009)
_set_cmake_policy_to_new_if_available(CMP0010)
_set_cmake_policy_to_new_if_available(CMP0011)
_set_cmake_policy_to_new_if_available(CMP0012)
_set_cmake_policy_to_new_if_available(CMP0013)
_set_cmake_policy_to_new_if_available(CMP0014)
_set_cmake_policy_to_new_if_available(CMP0015)
_set_cmake_policy_to_new_if_available(CMP0016)
_set_cmake_policy_to_new_if_available(CMP0017)

# the following operations must be performed inside a project context
if(NOT PROJECT_NAME)
  project(catkin_internal)
endif()

# include CMake functions
include(CMakeParseArguments)

# functions/macros: list_append_unique, safe_execute_process
# python-integration: catkin_python_setup.cmake, interrogate_setup_dot_py.py, templates/__init__.py.in, templates/script.py.in, templates/python_distutils_install.bat.in, templates/python_distutils_install.sh.in, templates/safe_execute_install.cmake.in
foreach(filename
    assert
    atomic_configure_file
    catkin_add_env_hooks
    catkin_destinations
    catkin_generate_environment
    catkin_install_python
    catkin_libraries
    catkin_metapackage
    catkin_package
    catkin_package_xml
    catkin_workspace
    debug_message
    em_expand
    python # defines PYTHON_EXECUTABLE, required by empy
    empy
    find_program_required
    legacy
    list_append_deduplicate
    list_append_unique
    list_insert_in_workspace_order
    safe_execute_process
    stamp
    string_starts_with
    platform/lsb
    platform/ubuntu
    platform/windows
    test/tests # defines CATKIN_ENABLE_TESTING, required by other test functions
    test/catkin_download_test_data
    test/gtest
    test/nosetests
    tools/doxygen
    tools/libraries
    tools/rt

#    tools/threads
  )
  include(${catkin_EXTRAS_DIR}/${filename}.cmake)
endforeach()

# output catkin version for debugging
# _catkin_package_xml(${CMAKE_BINARY_DIR}/catkin/catkin_generated/version DIRECTORY ${catkin_EXTRAS_DIR}/..)
#message(STATUS "catkin ${catkin_VERSION}")
# ensure that no current package name is set
unset(_CATKIN_CURRENT_PACKAGE)

# set global install destinations
set(CATKIN_GLOBAL_BIN_DESTINATION bin)
set(CATKIN_GLOBAL_ETC_DESTINATION etc)
set(CATKIN_GLOBAL_INCLUDE_DESTINATION include)
set(CATKIN_GLOBAL_LIB_DESTINATION lib)
set(CATKIN_GLOBAL_LIBEXEC_DESTINATION lib)
set(CATKIN_GLOBAL_PYTHON_DESTINATION ${PYTHON_INSTALL_DIR})
set(CATKIN_GLOBAL_SHARE_DESTINATION share)

# undefine CATKIN_ENV since it might be set in the cache from a previous build
set(CATKIN_ENV "" CACHE INTERNAL "catkin environment" FORCE)

# generate environment files like env.* and setup.*
# uses em_expand without CATKIN_ENV being set yet
catkin_generate_environment()

# file extension of env script
if(CMAKE_HOST_UNIX) # true for linux, apple, mingw-cross and cygwin
  set(script_ext sh)
else()
  set(script_ext bat)
endif()
# take snapshot of the modifications the setup script causes
# to reproduce the same changes with a static script in a fraction of the time
set(SETUP_DIR ${CMAKE_BINARY_DIR}/catkin_generated)
set(SETUP_FILENAME "setup_cached")
configure_file(${catkin_EXTRAS_DIR}/templates/generate_cached_setup.py.in
  ${CMAKE_BINARY_DIR}/catkin_generated/generate_cached_setup.py)
set(GENERATE_ENVIRONMENT_CACHE_COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_BINARY_DIR}/catkin_generated/generate_cached_setup.py)
# the script is generated once here and refreshed by every call to catkin_add_env_hooks()
safe_execute_process(COMMAND ${GENERATE_ENVIRONMENT_CACHE_COMMAND})
# generate env_cached which just relays to the setup_cached
configure_file(${catkin_EXTRAS_DIR}/templates/env.${script_ext}.in
  ${SETUP_DIR}/env_cached.${script_ext}
  @ONLY)
# environment to call external processes
set(CATKIN_ENV ${SETUP_DIR}/env_cached.${script_ext} CACHE INTERNAL "catkin environment")

# add additional environment hooks
if(CATKIN_BUILD_BINARY_PACKAGE)
  set(catkin_skip_install_env_hooks "SKIP_INSTALL")
endif()
if(CMAKE_HOST_UNIX)
  catkin_add_env_hooks(05.catkin_make SHELLS bash DIRECTORY ${catkin_EXTRAS_DIR}/env-hooks ${catkin_skip_install_env_hooks})
  catkin_add_env_hooks(05.catkin_make_isolated SHELLS bash DIRECTORY ${catkin_EXTRAS_DIR}/env-hooks ${catkin_skip_install_env_hooks})
  catkin_add_env_hooks(05.catkin-test-results SHELLS sh DIRECTORY ${catkin_EXTRAS_DIR}/env-hooks ${catkin_skip_install_env_hooks})
else()
  catkin_add_env_hooks(05.catkin-test-results SHELLS bat DIRECTORY ${catkin_EXTRAS_DIR}/env-hooks ${catkin_skip_install_env_hooks})
endif()

# requires stamp and environment files
include(${catkin_EXTRAS_DIR}/catkin_python_setup.cmake)
