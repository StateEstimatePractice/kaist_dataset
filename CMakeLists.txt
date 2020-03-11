cmake_minimum_required(VERSION 2.8)
project(kaist_dataset)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


find_package(OpenCV 3 REQUIRED)

message("opencv include:" ${OpenCV_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/include
    "/usr/include/eigen3"
    ${OpenCV_INCLUDE_DIRS})

ADD_EXECUTABLE(test_kaist test_kaist.cpp src/dataset_kaist)
target_link_libraries(test_kaist ${OpenCV_LIBS} )




