#
# Check if a string starts with a prefix.
#
# :param str: the string
# :type str: string
# :param prefix: the prefix
# :type prefix: string
# :param var: the output variable name
# :type var: bool
#
function(string_starts_with str prefix var)
  string(LENGTH "${str}" str_length)
  string(LENGTH "${prefix}" prefix_length)
  set(value FALSE)
  if(NOT ${str_length} LESS ${prefix_length})
    string(SUBSTRING "${str}" 0 ${prefix_length} str_prefix)
    if("${str_prefix}" STREQUAL "${prefix}")
      set(value TRUE)
    endif()
  endif()
  set(${var} ${value} PARENT_SCOPE)
endfunction()
