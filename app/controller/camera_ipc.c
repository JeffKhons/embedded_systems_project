#include "camera_ipc.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

static void *camera_thread(void *argument)
{
    struct camera_ipc *ipc = argument;
    char buffer[256];
    size_t used = 0U;

    while (atomic_load(&ipc->running)) {
        struct pollfd descriptor = {.fd = ipc->fd, .events = POLLIN};
        int ready = poll(&descriptor, 1, 250);
        if (ready <= 0 || (descriptor.revents & POLLIN) == 0)
            continue;

        ssize_t count = read(ipc->fd, buffer + used, sizeof(buffer) - used - 1U);
        if (count <= 0)
            continue;
        used += (size_t)count;
        buffer[used] = '\0';

        char *start = buffer;
        char *newline;
        while ((newline = strchr(start, '\n')) != NULL) {
            *newline = '\0';
            char *end;
            long tag_id = strtol(start, &end, 10);
            if (end != start && *end == '\0' && tag_id >= 0 && tag_id <= 10000)
                controller_handle_tag(ipc->controller, (int)tag_id);
            start = newline + 1;
        }
        size_t remaining = used - (size_t)(start - buffer);
        memmove(buffer, start, remaining);
        used = remaining;
        if (used == sizeof(buffer) - 1U)
            used = 0U;
    }
    return NULL;
}

int camera_ipc_start(struct camera_ipc *ipc,
                     const char *path,
                     struct controller *controller)
{
    struct stat status;

    if (mkfifo(path, 0660) < 0 && errno != EEXIST)
        return errno;
    if (stat(path, &status) < 0)
        return errno;
    if (!S_ISFIFO(status.st_mode))
        return EINVAL;

    ipc->path = path;
    ipc->controller = controller;
    atomic_init(&ipc->running, false);
    ipc->fd = open(path, O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (ipc->fd < 0)
        return errno;

    atomic_store(&ipc->running, true);
    int error = pthread_create(&ipc->thread, NULL, camera_thread, ipc);
    if (error != 0) {
        atomic_store(&ipc->running, false);
        close(ipc->fd);
        return error;
    }
    printf("[vision] listening for AprilTag IDs on %s\n", path);
    return 0;
}

void camera_ipc_stop(struct camera_ipc *ipc)
{
    atomic_store(&ipc->running, false);
    pthread_join(ipc->thread, NULL);
    close(ipc->fd);
}
