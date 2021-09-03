#
# Insert elements to a list in the same order as the chained catkin workspaces.
#
set(CATKIN_ORDERED_SPACES "")
foreach(_space ${CATKIN_DEVEL_PREFIX} ${CATKIN_WORKSPACES})
  list(APPEND CATKIN_ORDERED_SPACES ${_space})
  if(NOT EXISTS "${_space}/.catkin")
    message(FATAL_ERROR "The path '${_space}' is in CATKIN_WORKSPACES but does not have a .catkin file")
  endif()
  # prepend to existing list of sourcespaces
  file(READ "${_space}/.catkin" _sourcespaces)
  list(APPEND CATKIN_ORDERED_SPACES ${_sourcespaces})
endforeach()

debug_message(10 "CATKIN_ORDERED_SPACES ${CATKIN_ORDERED_SPACES}")

macro(list_insert_in_workspace_order listname)
  if(NOT "${ARGN}" STREQUAL "")
    assert(CATKIN_ENV)
    assert(PYTHON_EXECUTABLE)
    set(cmd
      ${CATKIN_ENV} ${PYTHON_EXECUTABLE}
      ${catkin_EXTRAS_DIR}/order_paths.py
      ${${PROJECT_NAME}_BINARY_DIR}/catkin_generated/ordered_paths.cmake
      --paths-to-order ${ARGN}
      --prefixes ${CATKIN_ORDERED_SPACES}
    )
    debug_message(10 "list_insert_in_workspace_order() in project '{PROJECT_NAME}' executes:  ${cmd}")
    safe_execute_process(COMMAND ${cmd})
    include(${${PROJECT_NAME}_BINARY_DIR}/catkin_generated/ordered_paths.cmake)
    set(${listname} ${ORDERED_PATHS})
  else()
    set(${listname} "")
  endif()
endmacro()
