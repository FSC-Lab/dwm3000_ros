cmake_minimum_required(VERSION 3.20.0)
project(dwm3000_ros)

find_package(fsc_serial CONFIG REQUIRED)
find_package(nanopb REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp dwm3000_msgs)

find_program(nanopb_GENERATOR NAMES nanopb_generator nanopb_generator.py)

catkin_package()

if(NOT nanopb_GENERATOR)
  message(FATAL_ERROR "nanopb_generator not found")
endif()

set(PROTO_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated-src)

file(MAKE_DIRECTORY ${PROTO_GEN_DIR})

set(PROTO_GEN_SOURCES ${PROTO_GEN_DIR}/UWBRange.pb.c)
set(PROTO_GEN_HEADERS ${PROTO_GEN_DIR}/UWBRange.pb.h)

add_custom_command(
  OUTPUT ${PROTO_GEN_SOURCES} ${PROTO_GEN_HEADERS}
  COMMAND ${nanopb_GENERATOR} ARGS -I ${CMAKE_SOURCE_DIR}/../proto -D
          ${PROTO_GEN_DIR} ${CMAKE_SOURCE_DIR}/../proto/UWBRange.proto
  COMMENT "Generating protobuf messages")

add_custom_target(
  ${CMAKE_PROJECT_NAME}_generate_proto
  DEPENDS ${PROTO_GEN_SOURCES} ${PROTO_GEN_HEADERS}
  COMMENT "Generating protobuf messages")

add_library(${CMAKE_PROJECT_NAME}_proto ${PROTO_GEN_SOURCES})
target_link_libraries(${CMAKE_PROJECT_NAME}_proto
                      PUBLIC nanopb::protobuf-nanopb-static)

add_library(${CMAKE_PROJECT_NAME} src/${CMAKE_PROJECT_NAME}.cpp)
add_dependencies(${CMAKE_PROJECT_NAME} ${CMAKE_PROJECT_NAME}_generate_proto
                 ${${CMAKE_PROJECT_NAME}_EXPORTED_TARGETS})
target_include_directories(
  ${CMAKE_PROJECT_NAME}
  PUBLIC include ${catkin_INCLUDE_DIRS}
  PRIVATE ${PROTO_GEN_DIR})
target_link_libraries(
  ${CMAKE_PROJECT_NAME}
  PUBLIC fsc_serial::fsc_serial ${catkin_LIBRARIES}
         nanopb::protobuf-nanopb-static ${CMAKE_PROJECT_NAME}_proto)

add_executable(${CMAKE_PROJECT_NAME}_node src/${CMAKE_PROJECT_NAME}_node.cpp)
target_link_libraries(${CMAKE_PROJECT_NAME}_node PUBLIC ${CMAKE_PROJECT_NAME})
