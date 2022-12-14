cmake_minimum_required(VERSION 3.25)
project(w04)

set(CMAKE_CXX_STANDARD 20)

SET(CMAKE_CXX_FLAGS -pthread)
set(CMAKE_CXX_STANDARD 20)

include_directories(/home/bohm/.mujoco/mujoco-2.3.1/include/)
link_directories(/home/bohm/.mujoco/mujoco-2.3.1/lib/)

find_package(glfw3 REQUIRED)

add_executable(w04 main.cpp)
target_link_libraries(w04 libmujoco.so libglfw.so)

