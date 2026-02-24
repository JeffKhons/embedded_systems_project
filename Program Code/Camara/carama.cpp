/*
 * camera/camera_app.cpp - 整合版相機程式
 * * 功能：
 * 1. 影像擷取 (OpenCV)
 * 2. AprilTag 偵測 (用於定位) -> 寫入 Named Pipe 通知 Main Controller [Slide 16]
 * 3. 影像串流 (UDP) -> 傳送 JPEG 給 PC 端 Server 進行 YOLO 偵測 [Slide 15]
 * * Compile: See CMakeLists.txt
 * 
 * * Dependencies:
 *   - OpenCV 4.5+ with C++
 *
 * Compile method:
 *   Use CMakeLists.txt
 *
 * Platform:
 *   - Tested on Raspberry Pi with V4L2 camera (OpenCV VideoCapture)
 *
 * (c) 2025  Jeff Chen – GPL-2.0
 */


#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp> // 需要 OpenCV Contrib 模組或內建 Aruco
#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>

// --- 設定參數 ---
#define SERVER_IP "192.168.222.100" // PC 端 IP (YOLO Server)
#define SERVER_PORT 8888
#define PIPE_PATH "/tmp/apriltag_pipe" // 與 Main Controller 通訊的 Pipe

int main() {
    // 1. 初始化攝像頭
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "[Camera] Can't open camera" << std::endl;
        return -1;
    }
    // 降低解析度以提升 FPS 與傳輸速度
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

    // 2. 初始化 UDP Socket
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("[Camera] Socket creation failed");
        return -1;
    }

    struct sockaddr_in serverAddr;
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(SERVER_PORT);
    if (inet_pton(AF_INET, SERVER_IP, &serverAddr.sin_addr) <= 0) {
        std::cerr << "[Camera] Invalid target IP address" << std::endl;
        close(sock);
        return -1;
    }

    // 3. 初始化 Named Pipe (IPC Writer)
    // 確保 Pipe 存在 (通常由 Reader 建立，但為了保險這邊也檢查)
    if (access(PIPE_PATH, F_OK) == -1) {
        mkfifo(PIPE_PATH, 0666);
    }
    
    std::cout << "[Camera] Opening Pipe to Main Controller..." << std::endl;
    // 使用 O_NONBLOCK 避免如果 Main 沒開，這裡會卡住
    int pipe_fd = open(PIPE_PATH, O_WRONLY | O_NONBLOCK);
    if (pipe_fd < 0) {
        std::cerr << "[Camera] Warning: Could not open Pipe. Main Controller may not be running." << std::endl;
    }

    // 4. 初始化 AprilTag 偵測器 (使用 OpenCV Aruco)
    // 使用的是 36h11 家族 (常用的 AprilTag 標準)
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
    
    std::cout << "[Camera] Loop Started: Streaming UDP + Detecting AprilTags" << std::endl;

    cv::Mat frame;
    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners;
    std::vector<uchar> buf;
    std::vector<int> param = {cv::IMWRITE_JPEG_QUALITY, 50}; // 畫質 50 以利傳輸

    while (true) {
        cap >> frame;
        if (frame.empty()) continue;

        // --- A. AprilTag 偵測 (本地導航用) ---
        cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, parameters);

        // 如果偵測到 Tag，且 Pipe 是開啟的
        if (markerIds.size() > 0) {
            // 在畫面上繪製 (Debug 用，也會傳回 PC)
            cv::aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

            if (pipe_fd >= 0) {
                // 將偵測到的第一個 ID 轉為字串傳給 Main
                std::string msg = std::to_string(markerIds[0]);
                write(pipe_fd, msg.c_str(), msg.length());
                // std::cout << "[Camera] Sent Tag ID: " << markerIds[0] << std::endl;
            } else {
                // 嘗試重新開啟 Pipe (若 Main 剛啟動)
                pipe_fd = open(PIPE_PATH, O_WRONLY | O_NONBLOCK);
            }
        }

        // --- B. UDP 影像串流 (監控用) ---
        cv::imencode(".jpg", frame, buf, param);
        if (!buf.empty() && buf.size() <= 65000) {
            sendto(sock, buf.data(), buf.size(), 0,
                   (struct sockaddr*)&serverAddr, sizeof(serverAddr));
        }

        // 控制 FPS，避免吃滿 CPU
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    if (pipe_fd >= 0) close(pipe_fd);
    close(sock);
    cap.release();
    return 0;
}