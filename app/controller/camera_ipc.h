#ifndef CAMERA_IPC_H
#define CAMERA_IPC_H

#include <stdatomic.h>
#include <pthread.h>

#include "controller.h"

struct camera_ipc {
    const char *path;
    int fd;
    atomic_bool running;
    pthread_t thread;
    struct controller *controller;
};

int camera_ipc_start(struct camera_ipc *ipc,
                     const char *path,
                     struct controller *controller);
void camera_ipc_stop(struct camera_ipc *ipc);

#endif
