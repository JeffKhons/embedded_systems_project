#ifndef TCP_SERVER_H
#define TCP_SERVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdatomic.h>
#include <pthread.h>

#include "controller.h"
#include "request_queue.h"

#define TCP_SERVER_MAX_CLIENTS 8

typedef void (*tcp_status_provider)(void *context, char *buffer, size_t size);

struct tcp_client {
    int fd;
    char input[1024];
    size_t used;
};

struct tcp_server {
    int port;
    int listen_fd;
    atomic_bool running;
    pthread_t thread;
    pthread_mutex_t mutex;
    struct tcp_client clients[TCP_SERVER_MAX_CLIENTS];
    struct request_queue *queue;
    tcp_status_provider status_provider;
    void *status_context;
};

int tcp_server_start(struct tcp_server *server,
                     int port,
                     struct request_queue *queue,
                     tcp_status_provider status_provider,
                     void *status_context);
void tcp_server_stop(struct tcp_server *server);
void tcp_server_publish(void *context,
                        enum controller_state state,
                        int floor,
                        const char *message);

#endif
