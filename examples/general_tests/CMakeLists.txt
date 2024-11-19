cmake_minimum_required(VERSION 3.5)

project(general_tests LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(${CMAKE_CURRENT_SOURCE_DIR}/../../dependencies.cmake)


add_executable(${CMAKE_PROJECT_NAME} main.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME}
                      dqrobotics
                      dqrobotics-interface-coppeliasim-zmq)
