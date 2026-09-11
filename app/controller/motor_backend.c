#include "motor_backend.h"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <unistd.h>

int motor_backend_open(struct motor_backend *backend, bool use_mock)
{
    backend->fd = -1;
    backend->mock = use_mock;

    if (use_mock) {
        puts("[motor/mock] backend ready");
        return 0;
    }

    backend->fd = open(DUAL_STEPPER_DEVICE, O_RDWR | O_CLOEXEC);
    if (backend->fd < 0)
        return errno;
    return 0;
}

void motor_backend_close(struct motor_backend *backend)
{
    if (backend->fd >= 0)
        close(backend->fd);
    backend->fd = -1;
}

int motor_backend_start(struct motor_backend *backend,
                        const struct dual_stepper_command *command)
{
    if (backend->mock) {
        printf("[motor/mock] start direction=%s rotations=%u frequency=%u Hz\n",
               command->direction == DUAL_STEPPER_FORWARD ? "forward" : "backward",
               command->rotations, command->frequency_hz);
        return 0;
    }

    if (ioctl(backend->fd, DUAL_STEPPER_START, command) < 0)
        return errno;
    return 0;
}

int motor_backend_stop(struct motor_backend *backend)
{
    if (backend->mock) {
        puts("[motor/mock] stop");
        return 0;
    }

    if (backend->fd < 0)
        return ENODEV;
    if (ioctl(backend->fd, DUAL_STEPPER_STOP) < 0)
        return errno;
    return 0;
}

bool motor_backend_is_mock(const struct motor_backend *backend)
{
    return backend->mock;
}
