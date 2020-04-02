cmake_minimum_required(VERSION 2.8.8)
project(fa_harris)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3 -march=native -g" )

# Add -O0 to remove optimizations when using gcc
# IF(CMAKE_COMPILER_IS_GNUCC)
#     set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0 -g")
#     set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 -g")
# ENDIF(CMAKE_COMPILER_IS_GNUCC)

find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  dvs_msgs
  cv_bridge
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

# add_library(fa_harris_lib
#   include/fa_harris/include/fa_harris_detector.h
#   include/fa_harris/src/fa_harris_detector.cc
# )

# add_executable(fa_harris src/fa_harris.cc)
# target_link_libraries(fa_harris fa_harris_lib)

# message("Opencv lib : ${OpenCV_INCLUDE_DIRS}")
# set(LIBS ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})

# list(APPEND HEADER_FILES
#  include/fa_harris/include/fa_harris_detector.h
# )

# list(APPEND SOURCE_FILES
#  include/fa_harris/src/fa_harris_detector.cc
# )

# add_executable(fa_harris ${SOURCE_FILES} src/fa_harris.cc)
# target_link_libraries(fa_harris ${LIBS})

add_library(${PROJECT_NAME}
  src/fa_harris_detector.cc
)

add_executable(fa_harris_node src/fa_harris.cc)
target_link_libraries(fa_harris_node ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
