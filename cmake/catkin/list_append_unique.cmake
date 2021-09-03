#
# Append elements to a list if they are not already in the list.
#
# .. note:: Using CMake's ``list(APPEND ..)`` and
#   ``list(REMOVE_DUPLICATES ..)`` is not sufficient since its
#   implementation uses a set internally which makes the operation
#   unstable.
#
macro(list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()
