# Basic system
add_subdirectory(choreographer)
add_subdirectory(mapper)

# Planners
add_subdirectory(planner_trapezoidal)
if (ENABLE_QP)
  add_subdirectory(planner_qp)
endif()

# High-level tools
add_subdirectory(mobility)