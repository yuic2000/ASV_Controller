cmake_minimum_required(VERSION 3.10)
project(mpc)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native -fopenmp")

message("Project source dir: ${PROJECT_SOURCE_DIR}")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)


find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(osqp REQUIRED)


message("Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}")
message("Python3 include dir: ${Python3_INCLUDE_DIRS}")
message("Python3 library dir: ${Python3_LIBRARIES}")

include_directories( SYSTEM
    ${rclcpp_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${rcl_interfaces_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS} 
    ${Python3_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/include
)

add_library(mpc_lib
    ${PROJECT_SOURCE_DIR}/src/param_class.cpp
    ${PROJECT_SOURCE_DIR}/src/plot.cpp
    ${PROJECT_SOURCE_DIR}/src/blue_boat.cpp
    ${PROJECT_SOURCE_DIR}/src/controller/sim_mpc.cpp
    ${PROJECT_SOURCE_DIR}/src/controller/eMPC.cpp
    ${PROJECT_SOURCE_DIR}/src/utils/eMPCConstraints.cpp
    ${PROJECT_SOURCE_DIR}/src/utils/eMPCCost.cpp
    ${PROJECT_SOURCE_DIR}/src/utils/eigen2osqp_csc.cpp
    ${PROJECT_SOURCE_DIR}/src/utils/LieGroup.cpp
    ${PROJECT_SOURCE_DIR}/src/utils/gnc.cpp
)

add_library(mpc_communication
    ${PROJECT_SOURCE_DIR}/src/communication/mpc_cmd_pub.cpp
    ${PROJECT_SOURCE_DIR}/src/communication/gtodom_sub.cpp
    ${PROJECT_SOURCE_DIR}/src/communication/path_sub.cpp
)

add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/main.cpp)

target_link_libraries(${PROJECT_NAME}
    mpc_lib 
    mpc_communication
    osqp::osqp
)

ament_target_dependencies(${PROJECT_NAME} 
    rclcpp 
    std_msgs 
    nav_msgs
    geometry_msgs
    rcl_interfaces
    Python3
)

install(TARGETS
  mpc
  DESTINATION lib/${PROJECT_NAME}
)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
