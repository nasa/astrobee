# Basic system
add_subdirectory(mapper)
add_subdirectory(sentinel)
add_subdirectory(choreographer)

# Planners
add_subdirectory(planner_trapezoidal)
if (ENABLE_QP)
  add_subdirectory(planner_qp)
endif()

# High-level tools
add_subdirectory(mobility)