cmake_minimum_required(VERSION 3.20.0)
project(dwm3000_ros)

find_package(fsc_serial CONFIG REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp message_generation std_msgs)

add_message_files(DIRECTORY msg FILES UWBRange.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

add_library(${CMAKE_PROJECT_NAME} src/${CMAKE_PROJECT_NAME}.cpp)
add_dependencies(${CMAKE_PROJECT_NAME}
                 ${${CMAKE_PROJECT_NAME}_EXPORTED_TARGETS})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC include
                                                        ${catkin_INCLUDE_DIRS})
target_link_libraries(${CMAKE_PROJECT_NAME} PUBLIC fsc_serial::fsc_serial
                                                   ${catkin_LIBRARIES})

add_executable(${CMAKE_PROJECT_NAME}_node src/${CMAKE_PROJECT_NAME}_node.cpp)
target_link_libraries(${CMAKE_PROJECT_NAME}_node PUBLIC ${CMAKE_PROJECT_NAME})
