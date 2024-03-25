# CMakeList.txt: Slam 的 CMake 项目，在此处包括源代码并定义
# 项目特定的逻辑。
#
cmake_minimum_required (VERSION 3.0.2)
project (slam)

SET( CMAKE_CXX_FLAGS "-std=c++17 -O3" ) # 使用c++17标准，-03开启优化
add_compile_options(-std=c++17)
add_definitions(-DPROJ_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

#添加宏设置lio_zh中omp并行循环
add_definitions(-DMP_EN)
add_definitions(-DMP_PROC_NUM=4)

find_package(OpenMP QUIET)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
	roscpp
	rospy
	sensor_msgs
)
find_package(PCL REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ieskf_slam
#  CATKIN_DEPENDS roscpp rospy sensor_msgs
#  DEPENDS system_lib
)

# 这行代码定义了一个变量 SLAM_FILE，
# 用于存储前端（frontend）的源文件路径。
# 在这里，SLAM_FILE 的值被设置为 src/slam/frontend/frontend.cpp，表示前端源文件的路径。

include_directories(
   include
   lib
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDEDIR}
)
SET(SLAM_FILE
	src/slam/frontend/frontend.cpp
	src/slam/ieskf/ieskf.cpp
	src/slam/map/rect_map_manager.cpp
	src/slam/frontback_propagate/frontback_propagate.cpp
)
SET(FRONTEND_WRAPPER_FILE
	src/wrapper/frontend_wrapper.cpp
)

#定义了可执行文件，指定其源文件及依赖
add_executable(frontend_node src/app/running.cpp
${FRONTEND_WRAPPER_FILE}
${SLAM_FILE})

#指定了可执行文件的链接库
target_link_libraries(frontend_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)