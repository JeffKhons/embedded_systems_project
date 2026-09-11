#ifndef MOTOR_BACKEND_H
#define MOTOR_BACKEND_H

#include <stdbool.h>

#include "dual_stepper_uapi.h"

struct motor_backend {
    int fd;
    bool mock;
};

int motor_backend_open(struct motor_backend *backend, bool use_mock);
void motor_backend_close(struct motor_backend *backend);
int motor_backend_start(struct motor_backend *backend,
                        const struct dual_stepper_command *command);
int motor_backend_stop(struct motor_backend *backend);
bool motor_backend_is_mock(const struct motor_backend *backend);

#endif
