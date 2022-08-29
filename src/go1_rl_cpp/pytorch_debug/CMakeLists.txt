cmake_minimum_required(VERSION 3.0)
project(libtorch_debug)

list(APPEND CMAKE_PREFIX_PATH "${CMAKE_CURRENT_LIST_DIR}/libtorch")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

find_package(Torch REQUIRED)

message(STATUS "PROJECT NAME: ${PROJECT_NAME}")

add_executable(${PROJECT_NAME}_app main.cpp)
target_link_libraries(${PROJECT_NAME}_app "${TORCH_LIBRARIES}")
set_property(TARGET ${PROJECT_NAME}_app PROPERTY CXX_STANDARD 14)