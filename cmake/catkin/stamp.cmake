#
#   :param path:  file name
#
#   Uses ``configure_file`` to generate a file ``filepath.stamp`` hidden
#   somewhere in the build tree.  This will cause cmake to rebuild its
#   cache when ``filepath`` is modified.
#
function(stamp path)
  get_filename_component(filename "${path}" NAME)
  configure_file(${path}
    ${CMAKE_CURRENT_BINARY_DIR}/catkin_generated/stamps/${PROJECT_NAME}/${filename}.stamp
    COPYONLY)
endfunction()
