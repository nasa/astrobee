# This macro will interrogate the Python setup.py file in
# ``${${PROJECT_NAME}_SOURCE_DIR}``, and then creates forwarding
# Python :term:`pkgutil` infrastructure in devel space
# accordingly for the scripts and packages declared in setup.py.
#
# Doing so enables mixing :term:`generated code` in
# devel space with :term:`static code` from sourcespace within a
# single Python package.
#
# In addition, it adds the install command of
# distutils/setuputils to the install target.
#
# .. note:: If the project also uses genmsg message generation via
#   ``generate_messages()`` this function must be called before.
#
# @public
#
function(catkin_python_setup)
  if(ARGN)
    message(FATAL_ERROR "catkin_python_setup() called with unused arguments: ${ARGN}")
  endif()

  if(${PROJECT_NAME}_GENERATE_MESSAGES)
    message(FATAL_ERROR "generate_messages() must be called after catkin_python_setup() in project '${PROJECT_NAME}'")
  endif()
  if(${PROJECT_NAME}_GENERATE_DYNAMIC_RECONFIGURE)
    message(FATAL_ERROR "generate_dynamic_reconfigure_options() must be called after catkin_python_setup() in project '${PROJECT_NAME}'")
  endif()

  if(NOT EXISTS ${${PROJECT_NAME}_SOURCE_DIR}/setup.py)
    message(FATAL_ERROR "catkin_python_setup() called without 'setup.py' in project folder ' ${${PROJECT_NAME}_SOURCE_DIR}'")
  endif()

  assert(PYTHON_INSTALL_DIR)
  set(INSTALL_CMD_WORKING_DIRECTORY ${${PROJECT_NAME}_SOURCE_DIR})
  if(NOT WIN32)
    set(INSTALL_SCRIPT
      ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/python_distutils_install.sh)
    configure_file(${catkin_EXTRAS_DIR}/templates/python_distutils_install.sh.in
      ${INSTALL_SCRIPT}
      @ONLY)
  else()
    # need to convert install prefix to native path for python setuptools --prefix (its fussy about \'s)
    file(TO_NATIVE_PATH ${CMAKE_INSTALL_PREFIX} PYTHON_INSTALL_PREFIX)
    set(INSTALL_SCRIPT
      ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/python_distutils_install.bat)
    configure_file(${catkin_EXTRAS_DIR}/templates/python_distutils_install.bat.in
      ${INSTALL_SCRIPT}
      @ONLY)
  endif()

  # generate python script which gets executed at install time
  configure_file(${catkin_EXTRAS_DIR}/templates/safe_execute_install.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/safe_execute_install.cmake)
  install(SCRIPT ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/safe_execute_install.cmake)

  # interrogate setup.py
  stamp(${${PROJECT_NAME}_SOURCE_DIR}/setup.py)
  assert(CATKIN_ENV)
  assert(PYTHON_EXECUTABLE)
  set(cmd
    ${CATKIN_ENV} ${PYTHON_EXECUTABLE}
    ${catkin_EXTRAS_DIR}/interrogate_setup_dot_py.py
    ${PROJECT_NAME}
    ${${PROJECT_NAME}_SOURCE_DIR}/setup.py
    ${${PROJECT_NAME}_BINARY_DIR}/catkin_generated/setup_py_interrogation.cmake
    )
  debug_message(10 "catkin_python_setup() in project '{PROJECT_NAME}' executes:  ${cmd}")
  safe_execute_process(COMMAND ${cmd})
  include(${${PROJECT_NAME}_BINARY_DIR}/catkin_generated/setup_py_interrogation.cmake)

  # call catkin_package_xml() if it has not been called before
  if(NOT _CATKIN_CURRENT_PACKAGE)
    catkin_package_xml()
  endif()
  assert(${PROJECT_NAME}_VERSION)
  # verify that version from setup.py is equal to version from package.xml
  if(NOT "${${PROJECT_NAME}_SETUP_PY_VERSION}" STREQUAL "${${PROJECT_NAME}_VERSION}")
    message(FATAL_ERROR "catkin_python_setup() version in setup.py (${${PROJECT_NAME}_SETUP_PY_VERSION}) differs from version in package.xml (${${PROJECT_NAME}_VERSION})")
  endif()

  # generate relaying __init__.py for each python package
  if(${PROJECT_NAME}_SETUP_PY_PACKAGES)
    list(LENGTH ${PROJECT_NAME}_SETUP_PY_PACKAGES pkgs_count)
    math(EXPR pkgs_range "${pkgs_count} - 1")
    foreach(index RANGE ${pkgs_range})
      list(GET ${PROJECT_NAME}_SETUP_PY_PACKAGES ${index} pkg)
      if("${pkg}" STREQUAL "${PROJECT_NAME}")
        # mark that catkin_python_setup() was called and the setup.py file contains a package with the same name as the current project
        # in order to disable installation of generated __init__.py files in generate_messages() and generate_dynamic_reconfigure_options()
        set(${PROJECT_NAME}_CATKIN_PYTHON_SETUP_HAS_PACKAGE_INIT TRUE PARENT_SCOPE)
      endif()
      list(GET ${PROJECT_NAME}_SETUP_PY_PACKAGE_DIRS ${index} pkg_dir)
      get_filename_component(name ${pkg_dir} NAME)
      if(NOT ("${pkg}" STREQUAL "${name}"))
        message(FATAL_ERROR "The package name '${pkg}' differs from the basename of the path '${pkg_dir}' in project '${PROJECT_NAME}'")
      endif()
      get_filename_component(path ${pkg_dir} PATH)
      set(PACKAGE_PYTHONPATH ${CMAKE_CURRENT_SOURCE_DIR}/${path})
      configure_file(${catkin_EXTRAS_DIR}/templates/__init__.py.in
        ${CATKIN_DEVEL_PREFIX}/${PYTHON_INSTALL_DIR}/${pkg}/__init__.py
        @ONLY)
    endforeach()
  endif()

  # generate relay-script for each python module (and __init__.py files) if available
  if(${PROJECT_NAME}_SETUP_PY_MODULES)
    list(LENGTH ${PROJECT_NAME}_SETUP_PY_MODULES modules_count)
    math(EXPR modules_range "${modules_count} - 1")
    foreach(index RANGE ${modules_range})
      list(GET ${PROJECT_NAME}_SETUP_PY_MODULES ${index} module)
      list(GET ${PROJECT_NAME}_SETUP_PY_MODULE_DIRS ${index} module_dir)
      set(PYTHON_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/${module_dir}/${module})
      if(EXISTS ${PYTHON_SCRIPT})
        get_filename_component(path ${module} PATH)
        file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${path}")
        configure_file(${catkin_EXTRAS_DIR}/templates/relay.py.in
          ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${module}
          @ONLY)
        # relay parent __init__.py files if they exist
        while(NOT "${path}" STREQUAL "")
          set(PYTHON_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/${module_dir}/${path}/__init__.py)
          if(EXISTS ${PYTHON_SCRIPT})
            file(MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${path}")
            configure_file(${catkin_EXTRAS_DIR}/templates/relay.py.in
              ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_PYTHON_DESTINATION}/${path}/__init__.py
              @ONLY)
          else()
            message(WARNING "The module '${module_dir}/${module}' lacks an '__init__.py' file in the parent folder '${module_dir}/${path}' in project '${PROJECT_NAME}'")
          endif()
          get_filename_component(path ${path} PATH)
        endwhile()
      endif()
    endforeach()
  endif()

   # generate relay-script for each python script
  foreach(script ${${PROJECT_NAME}_SETUP_PY_SCRIPTS})
    get_filename_component(name ${script} NAME)
    if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${script})
      message(FATAL_ERROR "The script '${name}' as listed in 'setup.py' of '${PROJECT_NAME}' doesn't exist")
    endif()
    set(PYTHON_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/${script})
    atomic_configure_file(${catkin_EXTRAS_DIR}/templates/script.py.in
      ${CATKIN_DEVEL_PREFIX}/${CATKIN_GLOBAL_BIN_DESTINATION}/${name}
      @ONLY)
  endforeach()
endfunction()

stamp(${catkin_EXTRAS_DIR}/interrogate_setup_dot_py.py)
