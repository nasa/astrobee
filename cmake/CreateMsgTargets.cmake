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

function(create_msg_targets)
  cmake_parse_arguments(
    msg # Prefix of output variables
    ""  # List of boolean variables
    "DIR;SDIR;ADIR"  # List of mono valued arguments
    "DEPS"  # List of multi value arguments
    ${ARGN})

  if((NOT msg_DIR) AND (NOT msg_SDIR))
    message(WARNING "no msgs or srvs given to create_msg_targets")
	return()
  endif((NOT msg_DIR) AND (NOT msg_SDIR))

  # Determine our module name
  get_filename_component(MODULE_NAME ${CMAKE_CURRENT_SOURCE_DIR} NAME)

  # Search for all the messages to install
  if(msg_DIR)
    file(GLOB MSG_FILES ${msg_DIR}/*.msg)

    # The msgs to just be their name
    foreach(SRC ${MSG_FILES})
      get_filename_component(NAME ${SRC} NAME)
      list(APPEND MSG_FILE_NAMES ${NAME})
    endforeach()

    add_message_files(FILES ${MSG_FILE_NAMES})
  endif(msg_DIR)

  # Search for all the services to install
  if(msg_SDIR)
	  file(GLOB SRV_FILES ${msg_SDIR}/*.srv)

	  foreach(SRC ${SRV_FILES})
      get_filename_component(NAME ${SRC} NAME)
	    list(APPEND SRV_FILE_NAMES ${NAME})
    endforeach()

    add_service_files(FILES ${SRV_FILE_NAMES})
  endif(msg_SDIR)

  # Search for all the actions to install
  if(msg_ADIR)
	  file(GLOB ACTION_FILES ${msg_ADIR}/*.action)

	  foreach(SRC ${ACTION_FILES})
      get_filename_component(NAME ${SRC} NAME)
	    list(APPEND ACTION_FILE_NAMES ${NAME})
    endforeach()

    add_action_files(FILES ${ACTION_FILE_NAMES})
  endif(msg_ADIR)

  # Generate them
  generate_messages(DEPENDENCIES ${msg_DEPS})

  # Create a target that people can DEP to and find the headers
  add_custom_target(${MODULE_NAME}
    DEPENDS ${MODULE_NAME}_gencpp)
  get_target_property(MSG_INC ${MODULE_NAME}_gencpp INCLUDE_DIRECTORIES)
  set_target_properties(${MODULE_NAME}
    PROPERTIES INCLUDE_DIRECTORIES ${MSG_INC})

  # Make sure we install the package.xml so that it can be found later in the install directory
  install(FILES package.xml DESTINATION share/${MODULE_NAME})
endfunction(create_msg_targets)
