cmake_minimum_required(VERSION 3.10)
project(camera_app)

find_package(OpenCV REQUIRED)

# 輸出執行檔名稱為 camera_app
add_executable(camera_app camera_app.cpp)

# 連結 OpenCV 函式庫
target_link_libraries(camera_app ${OpenCV_LIBS})