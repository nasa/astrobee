cmake_minimum_required(VERSION 3.0.2)
project(ctl_mpc)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -DACADOS_WITH_DAQP -DACADOS_WITH_OSQP)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nodelet
  std_msgs
  ff_msgs
  ff_util
  config_reader
)

find_package(Eigen3 REQUIRED)

if (EXISTS "${PROJECT_SOURCE_DIR}/acados/include" AND EXISTS "${PROJECT_SOURCE_DIR}/acados/lib" AND EXISTS "${PROJECT_SOURCE_DIR}/acados/include/daqp/")
  message("Found acados!")
else()
  message( FATAL_ERROR "acados not found. Please compile the acados submodule in this repository first.")
endif()

set(acados_include ${PROJECT_SOURCE_DIR}/acados/include)
set(acados_lib ${PROJECT_SOURCE_DIR}/acados/lib)
set(solver_code ${PROJECT_SOURCE_DIR}/solver/c_generated_code)

# This exports a file that can be used in VSCode so that it also finds the #includes (e.g. for intellisense) 
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

catkin_package(
  INCLUDE_DIRS
    include   
    ${acados_include}/blasfeo/include/
    ${acados_include}/hpipm/include/
    ${acados_include}/
    ${acados_include}/daqp/
    ${acados_include}/qpOASES_e/
    ${acados_include}/osqp/
    ${acados_include}/../
    # "solver/"
    ${solver_code}
    ${catkin_INCLUDE_DIRS}
  LIBRARIES
    ctl_mpc
  CATKIN_DEPENDS
    roscpp
    nodelet
    ff_msgs
    ff_util
    config_reader
)

include_directories(
# include
  ${acados_include}/blasfeo/include/
  ${acados_include}/hpipm/include/
  ${acados_include}/
  ${acados_include}/daqp/
  ${acados_include}/qpOASES_e/
  ${acados_include}/osqp/
  ${acados_include}/../
  ${solver_code}
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(
  ${acados_lib}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  ${solver_code}/astrobee_constraints/astrobee_constr_h_e_fun_jac_uxt_zt_hess.c
  ${solver_code}/astrobee_constraints/astrobee_constr_h_e_fun_jac_uxt_zt.c
  ${solver_code}/astrobee_constraints/astrobee_constr_h_e_fun.c
  ${solver_code}/astrobee_constraints/astrobee_constr_h_fun_jac_uxt_zt_hess.c 
  ${solver_code}/astrobee_constraints/astrobee_constr_h_fun_jac_uxt_zt.c 
  ${solver_code}/astrobee_constraints/astrobee_constr_h_fun.c 
  ${solver_code}/astrobee_cost/astrobee_cost_ext_cost_0_fun_jac_hess.c
  ${solver_code}/astrobee_cost/astrobee_cost_ext_cost_0_fun_jac.c 
  ${solver_code}/astrobee_cost/astrobee_cost_ext_cost_0_fun.c 
  ${solver_code}/astrobee_cost/astrobee_cost_ext_cost_e_fun_jac_hess.c
  ${solver_code}/astrobee_cost/astrobee_cost_ext_cost_e_fun_jac.c 
  ${solver_code}/astrobee_cost/astrobee_cost_ext_cost_e_fun.c 
  ${solver_code}/astrobee_cost/astrobee_cost_ext_cost_fun_jac_hess.c
  ${solver_code}/astrobee_cost/astrobee_cost_ext_cost_fun_jac.c 
  ${solver_code}/astrobee_cost/astrobee_cost_ext_cost_fun.c 
  ${solver_code}/astrobee_model/astrobee_expl_ode_fun.c
  ${solver_code}/astrobee_model/astrobee_expl_ode_hess.c 
  ${solver_code}/astrobee_model/astrobee_expl_vde_adj.c 
  ${solver_code}/astrobee_model/astrobee_expl_vde_forw.c

  ${solver_code}/acados_sim_solver_astrobee.c
  ${solver_code}/acados_solver_astrobee.c

  src/ctl_mpc.cpp
  src/ctl_mpc_nodelet.cpp
)

add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} acados hpipm blasfeo m daqp ${catkin_LIBRARIES})

# Mark libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# Mark nodelet_plugin for installation
install(FILES nodelet_plugins.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

# Mark launch files for installation
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE)