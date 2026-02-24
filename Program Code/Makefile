# 在 Makefile 中新增

# Main Controller
main_ctrl: main_controller/main.c
	$(CROSS)gcc -o main_ctrl main_controller/main.c -lpthread

# Camera App (需連結 OpenCV)
camera_app: camera/camera_app.cpp
	$(CROSS)g++ -o camera_app camera/camera_app.cpp `pkg-config --cflags --libs opencv4`

# Mock Server 的編譯
web_mock: main_controller/web_backend_mock.c
	gcc -o web_mock main_controller/web_backend_mock.c

# 更新 all
all: modules FP_motor_writer_1 main_ctrl camera_app web_mock