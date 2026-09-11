#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {
constexpr const char *kDefaultServerIp = "127.0.0.1";
constexpr int kDefaultServerPort = 8889;
constexpr const char *kDefaultPipePath = "/tmp/wall_robot_apriltag.fifo";

struct Options {
    std::string server_ip = kDefaultServerIp;
    int server_port = kDefaultServerPort;
    std::string pipe_path = kDefaultPipePath;
    int camera_index = 0;
};

void print_usage(const char *program)
{
    std::cout << "Usage: " << program
              << " [--server IP] [--port PORT] [--pipe PATH] [--camera INDEX]\n";
}

bool parse_options(int argc, char **argv, Options &options)
{
    for (int index = 1; index < argc; ++index) {
        std::string argument = argv[index];
        if (argument == "--help") {
            print_usage(argv[0]);
            std::exit(EXIT_SUCCESS);
        }
        if (index + 1 >= argc)
            return false;
        if (argument == "--server")
            options.server_ip = argv[++index];
        else if (argument == "--port")
            options.server_port = std::stoi(argv[++index]);
        else if (argument == "--pipe")
            options.pipe_path = argv[++index];
        else if (argument == "--camera")
            options.camera_index = std::stoi(argv[++index]);
        else
            return false;
    }
    return options.server_port > 0 && options.server_port <= 65535;
}

int open_pipe_writer(const std::string &path)
{
    return open(path.c_str(), O_WRONLY | O_NONBLOCK | O_CLOEXEC);
}
} // namespace

int main(int argc, char **argv)
{
    Options options;
    try {
        if (!parse_options(argc, argv, options)) {
            print_usage(argv[0]);
            return EXIT_FAILURE;
        }
    } catch (const std::exception &) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    cv::VideoCapture camera(options.camera_index);
    if (!camera.isOpened()) {
        std::cerr << "[vision] unable to open camera " << options.camera_index << '\n';
        return EXIT_FAILURE;
    }
    camera.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    camera.set(cv::CAP_PROP_FRAME_HEIGHT, 240);

    int udp_socket = socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
    if (udp_socket < 0) {
        perror("[vision] socket");
        return EXIT_FAILURE;
    }
    sockaddr_in destination{};
    destination.sin_family = AF_INET;
    destination.sin_port = htons(static_cast<uint16_t>(options.server_port));
    if (inet_pton(AF_INET, options.server_ip.c_str(), &destination.sin_addr) != 1) {
        std::cerr << "[vision] invalid UDP destination\n";
        close(udp_socket);
        return EXIT_FAILURE;
    }

    int pipe_fd = open_pipe_writer(options.pipe_path);
    auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    auto parameters = cv::aruco::DetectorParameters::create();
    std::vector<int> jpeg_parameters = {cv::IMWRITE_JPEG_QUALITY, 50};
    int last_tag = -1;
    auto last_report = std::chrono::steady_clock::time_point::min();

    std::cout << "[vision] streaming UDP to " << options.server_ip << ':'
              << options.server_port << " and reporting tags to " << options.pipe_path << '\n';

    for (;;) {
        cv::Mat frame;
        camera >> frame;
        if (frame.empty())
            continue;

        std::vector<int> marker_ids;
        std::vector<std::vector<cv::Point2f>> marker_corners;
        cv::aruco::detectMarkers(frame, dictionary, marker_corners, marker_ids, parameters);
        if (!marker_ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, marker_corners, marker_ids);
            auto now = std::chrono::steady_clock::now();
            if (marker_ids.front() != last_tag ||
                now - last_report > std::chrono::milliseconds(500)) {
                if (pipe_fd < 0)
                    pipe_fd = open_pipe_writer(options.pipe_path);
                if (pipe_fd >= 0) {
                    std::string message = std::to_string(marker_ids.front()) + "\n";
                    if (write(pipe_fd, message.data(), message.size()) < 0) {
                        close(pipe_fd);
                        pipe_fd = -1;
                    }
                }
                last_tag = marker_ids.front();
                last_report = now;
            }
        }

        std::vector<unsigned char> jpeg;
        cv::imencode(".jpg", frame, jpeg, jpeg_parameters);
        if (!jpeg.empty() && jpeg.size() <= 65000U) {
            sendto(udp_socket, jpeg.data(), jpeg.size(), 0,
                   reinterpret_cast<sockaddr *>(&destination), sizeof(destination));
        }
    }
}
