cmake_minimum_required(VERSION 2.8.3)
project(line_extractor)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  visualization_msgs
)

catkin_package( )

include_directories( include ${catkin_INCLUDE_DIRS} )

add_executable(${PROJECT_NAME}_node src/line_extractor_node.cpp src/line_extractor.cpp)
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} )
