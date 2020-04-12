# Bugfix for bamboo, opencv libraries are in the wrong place

# A little macro to replace in list
macro (LIST_REPLACE LIST INDEX NEWVALUE)
  list (REMOVE_AT ${LIST} ${INDEX})
  list (LENGTH ${LIST} __length)
  # Cannot insert at the end
  if (${__length} EQUAL ${INDEX})
      list (APPEND ${LIST} ${NEWVALUE})
  else (${__length} EQUAL ${INDEX})
      list (INSERT ${LIST} ${INDEX} ${NEWVALUE})
  endif (${__length} EQUAL ${INDEX})
endmacro (LIST_REPLACE)

# Replace with the correct path 
get_cmake_property(variableNames VARIABLES)
foreach (variableName ${variableNames})
  foreach(library ${${variableName}})
    string(FIND "${library}" "lib/libopencv" find_idx)
    if (${find_idx} GREATER -1)
      if(NOT EXISTS "${library}")
        STRING(REGEX REPLACE "lib/libopencv" "lib/x86_64-linux-gnu/libopencv" out_library "${library}")
        list(FIND ${variableName} "${library}" find_idx)
        if(find_idx GREATER -1)
          LIST_REPLACE(${variableName} ${find_idx} "${out_library}")
        endif()
      endif()
    endif()
  endforeach()
endforeach()

