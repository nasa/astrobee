set(
    daqp_src
    "${CMAKE_CURRENT_SOURCE_DIR}/api.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/auxiliary.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/bnb.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/daqp.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/daqp_prox.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/factorization.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/utils.c"
)
set(daqp_src "${daqp_src}" PARENT_SCOPE)
