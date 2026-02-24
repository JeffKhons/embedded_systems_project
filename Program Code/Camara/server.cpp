/*
 * yolo_udp_server.cpp ¡X YOLOv2 ONNX-based real-time object detection server
 *
 * This program listens for incoming JPEG-encoded image frames over UDP,
 * decodes them, performs object detection using a YOLOv2 ONNX model, 
 * draws bounding boxes with class labels, and saves the processed images
 * to a local directory.
 *
 * Dependencies:
 *   - OpenCV 4.5+ with DNN module
 *   - yolov2-coco-9.onnx (YOLOv2 model trained on COCO dataset)
 *
 * Compile:
 *   g++ yolo_udp_server.cpp -o yolo_udp_server `pkg-config --cflags --libs opencv4`
 *
 * Usage:
 *   ./yolo_udp_server
 *
 * Output:
 *   - Detected results saved as output_0.jpg, output_1.jpg, etc. in ./detect_result
 *
 * Author:
 *   Jeff Chen, 2025/06/01. GPL-2.0
 */

#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <filesystem>

// Sigmoid function
inline float sigmoid(float x) {
    return 1.0f / (1.0f + std::exp(-x));
}

// COCO class names
std::vector<std::string> getCocoClassNames() {
    return {
        "person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train",
        "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter",
        "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear",
        "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase",
        "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
        "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
        "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
        "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut",
        "cake", "chair", "sofa", "pottedplant", "bed", "diningtable", "toilet",
        "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
        "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
        "teddy bear", "hair drier", "toothbrush"
    };
}

int main() {
	// === Load YOLOv2 model ===
    cv::dnn::Net net = cv::dnn::readNetFromONNX("yolov2-coco-9.onnx");
    if (net.empty()) {
        std::cerr << "Failed to load model" << std::endl;
        return -1;
    }

    std::vector<std::string> classNames = getCocoClassNames();
	
	// === Setup UDP socket ===
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        perror("socket creation failed");
        return -1;
    }

    sockaddr_in serverAddr{}, clientAddr{};
    socklen_t addrLen = sizeof(clientAddr);
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(8888);
    if (bind(sock, (sockaddr*)&serverAddr, sizeof(serverAddr)) < 0) {
        perror("bind failed");
        close(sock);
        return -1;
    }

    /*** setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, (char*)&(timeval){5, 0}, sizeof(timeval)); ***/
    std::filesystem::create_directory("detect_result");

    std::cout << "Server listening on UDP port 8888..." << std::endl;
    int count = 0;

    while (true) {
		// === Receive image ===
        std::vector<uchar> buffer(65535);
        ssize_t recvLen = recvfrom(sock, buffer.data(), buffer.size(), 0,
                                   (sockaddr*)&clientAddr, &addrLen);
        if (recvLen <= 0) continue;
		
		// === Preprocess input ===
        buffer.resize(recvLen);
        cv::Mat img = cv::imdecode(buffer, cv::IMREAD_COLOR);
        if (img.empty()) continue;
		
		// === Yolo decoder ===
        cv::Mat blob = cv::dnn::blobFromImage(img, 1 / 255.0, cv::Size(416, 416), cv::Scalar(), true, false);
        net.setInput(blob);
        cv::Mat output = net.forward();

        const int numAnchors = 5;
        const int numClasses = 80;
        const int gridSize = 13;
        float anchors[10] = {1.08,1.19, 3.42,4.41, 6.63,11.38, 9.42,5.11, 16.62,10.52};
        const float confThreshold = 0.4;
        const float nmsThreshold = 0.45;

        std::vector<cv::Rect> boxes;
        std::vector<int> classIds;
        std::vector<float> confidences;

        const float* data = (float*)output.data;
        for (int n = 0; n < numAnchors; ++n) {
            for (int i = 0; i < gridSize; ++i) {
                for (int j = 0; j < gridSize; ++j) {
                    int offset = n * (numClasses + 5) * gridSize * gridSize +
                                 i * gridSize + j;
                    int stride = gridSize * gridSize;

                    float bx = sigmoid(data[offset + 0 * stride]);
                    float by = sigmoid(data[offset + 1 * stride]);
                    float bw = std::exp(data[offset + 2 * stride]) * anchors[2 * n];
                    float bh = std::exp(data[offset + 3 * stride]) * anchors[2 * n + 1];
                    float objectness = sigmoid(data[offset + 4 * stride]);

                    float maxProb = 0;
                    int classId = -1;
                    for (int c = 0; c < numClasses; ++c) {
                        float classScore = sigmoid(data[offset + (5 + c) * stride]);
                        if (classScore > maxProb) {
                            maxProb = classScore;
                            classId = c;
                        }
                    }

                    float confidence = objectness * maxProb;
                    if (confidence > confThreshold) {
                        int centerX = static_cast<int>((j + bx) / gridSize * img.cols);
                        int centerY = static_cast<int>((i + by) / gridSize * img.rows);
                        int width   = static_cast<int>(bw / gridSize * img.cols);
                        int height  = static_cast<int>(bh / gridSize * img.rows);
                        int left    = centerX - width / 2;
                        int top     = centerY - height / 2;

                        boxes.emplace_back(left, top, width, height);
                        classIds.push_back(classId);
                        confidences.push_back(confidence);
                    }
                }
            }
        }
		
		// === Apply NMS and draw boxes ===
        std::vector<int> indices;
        cv::dnn::NMSBoxes(boxes, confidences, confThreshold, nmsThreshold, indices);
        for (int idx : indices) {
            cv::Rect box = boxes[idx];
            int classId = classIds[idx];
            std::string label = classNames[classId] + ": " + cv::format("%.2f", confidences[idx]);
            cv::rectangle(img, box, cv::Scalar(0, 255, 0), 2);
            cv::putText(img, label, box.tl() - cv::Point(0, 5), cv::FONT_HERSHEY_SIMPLEX,
                        0.5, cv::Scalar(0, 0, 0), 1);
        }
		
		// === Save result ===
        std::string name = "detect_result/output_" + std::to_string(count++) + ".jpg";
        cv::imwrite(name, img);
        std::cout << "Saved: " << name << std::endl;
    }

    close(sock);
    return 0;
}
