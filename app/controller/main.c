#include <errno.h>
#include <getopt.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "camera_ipc.h"
#include "controller.h"
#include "motor_service.h"
#include "request_queue.h"
#include "tcp_server.h"

#define DEFAULT_PORT 8888
#define DEFAULT_PIPE_PATH "/tmp/wall_robot_apriltag.fifo"
#define DEFAULT_ROTATIONS_PER_FLOOR 10U

struct options {
    bool mock_motor;
    bool camera_ipc;
    int port;
    unsigned int rotations_per_floor;
    const char *pipe_path;
};

struct shutdown_context {
    sigset_t signals;
    struct request_queue *queue;
    struct motor_service *motor;
};

static void print_usage(const char *program)
{
    printf("Usage: %s [options]\n"
           "  --mock                      use the hardware-free motor backend\n"
           "  --port PORT                 TCP command port (default: 8888)\n"
           "  --pipe PATH                 AprilTag FIFO path\n"
           "  --rotations-per-floor N     motion calibration (default: 10)\n"
           "  --no-camera-ipc             disable the AprilTag FIFO listener\n",
           program);
}

static int parse_positive(const char *text, unsigned int maximum, unsigned int *value)
{
    char *end;
    unsigned long parsed = strtoul(text, &end, 10);
    if (*text == '\0' || *end != '\0' || parsed == 0UL || parsed > maximum)
        return EINVAL;
    *value = (unsigned int)parsed;
    return 0;
}

static int parse_options(int argc, char **argv, struct options *options)
{
    static const struct option long_options[] = {
        {"mock", no_argument, NULL, 'm'},
        {"port", required_argument, NULL, 'p'},
        {"pipe", required_argument, NULL, 'f'},
        {"rotations-per-floor", required_argument, NULL, 'r'},
        {"no-camera-ipc", no_argument, NULL, 'n'},
        {"help", no_argument, NULL, 'h'},
        {NULL, 0, NULL, 0},
    };
    int option;

    *options = (struct options) {
        .mock_motor = false,
        .camera_ipc = true,
        .port = DEFAULT_PORT,
        .rotations_per_floor = DEFAULT_ROTATIONS_PER_FLOOR,
        .pipe_path = DEFAULT_PIPE_PATH,
    };

    while ((option = getopt_long(argc, argv, "mp:f:r:nh", long_options, NULL)) != -1) {
        unsigned int parsed;
        switch (option) {
        case 'm': options->mock_motor = true; break;
        case 'p':
            if (parse_positive(optarg, 65535U, &parsed) != 0)
                return EINVAL;
            options->port = (int)parsed;
            break;
        case 'f': options->pipe_path = optarg; break;
        case 'r':
            if (parse_positive(optarg, 10000U, &options->rotations_per_floor) != 0)
                return EINVAL;
            break;
        case 'n': options->camera_ipc = false; break;
        case 'h': print_usage(argv[0]); exit(EXIT_SUCCESS);
        default: return EINVAL;
        }
    }
    return 0;
}

static void *wait_for_shutdown(void *argument)
{
    struct shutdown_context *context = argument;
    int received_signal;

    if (sigwait(&context->signals, &received_signal) == 0)
        printf("[system] received signal %d; shutting down\n", received_signal);
    request_queue_close(context->queue);
    motor_service_request_stop(context->motor);
    return NULL;
}

int main(int argc, char **argv)
{
    struct options options;
    struct request_queue queue;
    struct motor_service motor;
    struct controller controller;
    struct tcp_server tcp_server;
    struct camera_ipc camera_ipc;
    bool camera_started = false;
    pthread_t signal_thread;
    int error;

    if (parse_options(argc, argv, &options) != 0) {
        print_usage(argv[0]);
        return EXIT_FAILURE;
    }

    sigset_t shutdown_signals;
    sigemptyset(&shutdown_signals);
    sigaddset(&shutdown_signals, SIGINT);
    sigaddset(&shutdown_signals, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &shutdown_signals, NULL);

    struct sigaction ignore_sigpipe = {.sa_handler = SIG_IGN};
    sigemptyset(&ignore_sigpipe.sa_mask);
    sigaction(SIGPIPE, &ignore_sigpipe, NULL);

    error = request_queue_init(&queue);
    if (error != 0) {
        fprintf(stderr, "request queue initialization failed: %s\n", strerror(error));
        return EXIT_FAILURE;
    }

    error = motor_service_init(&motor, options.mock_motor);
    if (error != 0) {
        fprintf(stderr, "motor initialization failed: %s\n", strerror(error));
        request_queue_destroy(&queue);
        return EXIT_FAILURE;
    }

    error = controller_init(&controller, &queue, &motor, options.rotations_per_floor,
                            NULL, NULL);
    if (error != 0) {
        fprintf(stderr, "controller initialization failed: %s\n", strerror(error));
        motor_service_shutdown(&motor);
        request_queue_destroy(&queue);
        return EXIT_FAILURE;
    }

    error = tcp_server_start(&tcp_server, options.port, &queue,
                             controller_format_status, &controller);
    if (error != 0) {
        fprintf(stderr, "TCP server initialization failed: %s\n", strerror(error));
        controller_destroy(&controller);
        motor_service_shutdown(&motor);
        request_queue_destroy(&queue);
        return EXIT_FAILURE;
    }
    controller.status_sink = tcp_server_publish;
    controller.status_context = &tcp_server;

    if (options.camera_ipc) {
        error = camera_ipc_start(&camera_ipc, options.pipe_path, &controller);
        if (error == 0)
            camera_started = true;
        else
            fprintf(stderr, "AprilTag IPC unavailable (%s); continuing without it\n",
                    strerror(error));
    }

    struct shutdown_context shutdown = {
        .signals = shutdown_signals,
        .queue = &queue,
        .motor = &motor,
    };
    error = pthread_create(&signal_thread, NULL, wait_for_shutdown, &shutdown);
    if (error != 0) {
        fprintf(stderr, "signal thread creation failed: %s\n", strerror(error));
        request_queue_close(&queue);
    }

    printf("[system] controller ready in %s mode\n",
           options.mock_motor ? "mock" : "hardware");
    int run_error = controller_run(&controller);

    if (error == 0)
        pthread_join(signal_thread, NULL);
    if (camera_started)
        camera_ipc_stop(&camera_ipc);
    tcp_server_stop(&tcp_server);
    controller_destroy(&controller);
    motor_service_shutdown(&motor);
    request_queue_destroy(&queue);

    return run_error == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
