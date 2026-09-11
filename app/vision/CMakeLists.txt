cmake_minimum_required(VERSION 3.16)
project(wall_robot_vision LANGUAGES CXX)

find_package(OpenCV 4 REQUIRED)

add_executable(camera_app camera_app.cpp)
target_link_libraries(camera_app PRIVATE ${OpenCV_LIBS})
target_include_directories(camera_app PRIVATE ${OpenCV_INCLUDE_DIRS})
target_compile_features(camera_app PRIVATE cxx_std_17)
target_compile_options(camera_app PRIVATE -Wall -Wextra -Wpedantic)
