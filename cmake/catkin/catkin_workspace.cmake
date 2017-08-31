#
# Search all subfolders in the workspace for ``package.xml`` files.
# Based on the dependencies specified in the ``build_depends``,
# ``buildtool_depends`` and (as of package format version 2)
# ``test_depends`` tags it performs a topological sort and calls
# ``add_subdirectory()`` for each directory.
#
# The functions is only called in catkin's ``toplevel.cmake``, which
# is usually symlinked to the workspace root directory (which
# contains multiple packages).
#
function(catkin_workspace)
  debug_message(10 "catkin_workspace() called in file '${CMAKE_CURRENT_LIST_FILE}'")

  # set global output directories for artifacts and create them if necessary
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/lib)
  if(NOT IS_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
    file(MAKE_DIRECTORY ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
  endif()

  # tools/libraries.cmake
  configure_shared_library_build_settings()

  set(CATKIN_WHITELIST_PACKAGES "" CACHE STRING "List of ';' separated packages to build")
  set(CATKIN_BLACKLIST_PACKAGES "" CACHE STRING "List of ';' separated packages to exclude")
  if(NOT "${CATKIN_WHITELIST_PACKAGES}" STREQUAL "")
    message(STATUS "Using CATKIN_WHITELIST_PACKAGES: ${CATKIN_WHITELIST_PACKAGES}")
  endif()
  if(NOT "${CATKIN_BLACKLIST_PACKAGES}" STREQUAL "")
    message(STATUS "Using CATKIN_BLACKLIST_PACKAGES: ${CATKIN_BLACKLIST_PACKAGES}")
  endif()

  assert(catkin_EXTRAS_DIR)
  em_expand(
    ${catkin_EXTRAS_DIR}/templates/order_packages.context.py.in
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/order_packages.py
    ${catkin_EXTRAS_DIR}/em/order_packages.cmake.em
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/order_packages.cmake
    )
  debug_message(10 "catkin_workspace() including order_packages.cmake")
  include(${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/order_packages.cmake)

  if(CATKIN_ORDERED_PACKAGES)
    set(CATKIN_NONCONFORMANT_METAPACKAGE FALSE)
    set(CATKIN_NONHOMOGENEOUS_WORKSPACE FALSE)
    list(LENGTH CATKIN_ORDERED_PACKAGES count)
    message(STATUS "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
    message(STATUS "~~  traversing ${count} packages in topological order:")
    math(EXPR range "${count} - 1")
    foreach(index RANGE ${range})
      list(GET CATKIN_ORDERED_PACKAGES ${index} name)
      list(GET CATKIN_ORDERED_PACKAGE_PATHS ${index} path)
      list(GET CATKIN_ORDERED_PACKAGES_IS_META ${index} is_meta)
      list(GET CATKIN_ORDERED_PACKAGES_BUILD_TYPE ${index} build_type)
      if(${is_meta})
        message(STATUS "~~  - ${name} (metapackage)")
        # verify that CMakeLists.txt of metapackage conforms to standard
        set(metapackage_arguments "")
        assert(CATKIN_METAPACKAGE_CMAKE_TEMPLATE)
        configure_file(${CATKIN_METAPACKAGE_CMAKE_TEMPLATE}
          ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/metapackages/${name}/CMakeLists.txt
          @ONLY)
        if(EXISTS ${CMAKE_SOURCE_DIR}/${path}/CMakeLists.txt)
          # compare CMakeLists.txt with standard content
          file(STRINGS ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/metapackages/${name}/CMakeLists.txt generated_cmakelists)
          file(STRINGS ${path}/CMakeLists.txt existing_cmakelists)
          if(NOT "${generated_cmakelists}" STREQUAL "${existing_cmakelists}")
            set(CATKIN_NONHOMOGENEOUS_WORKSPACE TRUE)
            message("WARNING: The CMakeLists.txt of the metapackage '${name}' contains non standard content. Use the content of the following file instead: ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/metapackages/${name}/CMakeLists.txt")
          endif()
        else()
          message("WARNING: The metapackage '${name}' has no CMakeLists.txt. Please add one to the package source. You can use the following file: ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/metapackages/${name}/CMakeLists.txt")
        endif()
      else()
        if(${build_type} MATCHES catkin)
          message(STATUS "~~  - ${name}")
        else()
          set(CATKIN_NONHOMOGENEOUS_WORKSPACE TRUE)
          if(${build_type} MATCHES cmake)
            message(STATUS "~~  - ${name} (plain cmake)")
          else()
            message(STATUS "~~  - ${name} (unknown)")
            message(WARNING "Unknown build type '${build_type}' for package '${name}'")
          endif()
        endif()
      endif()
    endforeach()
    message(STATUS "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~")

    if(${CATKIN_NONCONFORMANT_METAPACKAGE})
      message(FATAL_ERROR "This workspace contains metapackages with a non-standard CMakeLists.txt.")
    endif()
    if(${CATKIN_NONHOMOGENEOUS_WORKSPACE})
      message(FATAL_ERROR "This workspace contains non-catkin packages in it, and catkin cannot build a non-homogeneous workspace without isolation. Try the 'catkin_make_isolated' command instead.")
    endif()

    foreach(index RANGE ${range})
      list(GET CATKIN_ORDERED_PACKAGES ${index} name)
      list(GET CATKIN_ORDERED_PACKAGE_PATHS ${index} path)
      list(GET CATKIN_ORDERED_PACKAGES_IS_META ${index} is_meta)
      list(GET CATKIN_ORDERED_PACKAGES_BUILD_TYPE ${index} build_type)
      if(${is_meta})
        message(STATUS "+++ processing catkin metapackage: '${name}'")
        if(EXISTS ${CMAKE_SOURCE_DIR}/${path}/CMakeLists.txt)
          message(STATUS "==> add_subdirectory(${path})")
          add_subdirectory(${path})
        else()
          message(STATUS "==> add_subdirectory(${path}) (using generated file from <buildspace>/catkin_generated/metapackages/${name})")
          message("WARNING: Add a CMakeLists.txt file to the metapackage '${name}'")
          add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/metapackages/${name} ${CMAKE_BINARY_DIR}/${path})
        endif()
      elseif(${build_type} MATCHES catkin)
        message(STATUS "+++ processing catkin package: '${name}'")
        message(STATUS "==> add_subdirectory(${path})")
        add_subdirectory(${path})
      else()
        message(FATAL_ERROR "Non-catkin package found, non-homogeneous workspaces are not supported.")
      endif()
    endforeach()
  endif()
endfunction()
