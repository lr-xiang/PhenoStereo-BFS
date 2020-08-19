cmake_minimum_required(VERSION 2.8.12)

project(sensor_client)

#list(APPEND CMAKE_PREFIX_PATH "$ENV{HOME}/odos_imaging/swift_sdk")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11")
set(CUDA_USE_STATIC_CUDA_RUNTIME OFF)
find_package(OpenCV REQUIRED core highgui imgproc)
find_package(Boost REQUIRED filesystem system)

SET(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
#list(APPEND CMAKE_PREFIX_PATH "/opt/odos_imaging/swift_sdk")
#find_package(OdosSwift REQUIRED)

#find_package(GTest REQUIRED)
#include_directories(${GTEST_INCLUDE_DIRS})

set(SRC src/singleCamera.cpp src/flirBFS.cpp src/jetsonGPIO.cpp)
set(SRC_TEST src/mainTest.cpp)
set(SRC_MAIN src/sensorClient.cpp)

include_directories(${Boost_INCLUDE_DIRS} "${PROJECT_SOURCE_DIR}/include" "${PROJECT_SOURCE_DIR}/src" "/usr/include/spinnaker" "/usr/include/jsoncpp")
link_directories(${Boost_LIBRARY_DIRS})

add_executable(sensor_client ${SRC} ${SRC_MAIN} )
target_link_libraries(sensor_client -lpthread -lSpinnaker -ljsoncpp ${OpenCV_LIBS} ${Boost_LIBRARIES})

#add_executable(runTests ${SRC} ${SRC_TEST} )
#target_link_libraries(runTests -lpthread ${Boost_LIBRARIES} ${GTEST_LIBRARIES} -lSpinnaker -ljsoncpp  ${OpenCV_LIBS} -lb64)
