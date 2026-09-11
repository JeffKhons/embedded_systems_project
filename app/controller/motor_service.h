#ifndef MOTOR_SERVICE_H
#define MOTOR_SERVICE_H

#include <stdbool.h>
#include <pthread.h>

#include "motor_backend.h"

struct motor_service {
    struct motor_backend backend;
    pthread_t thread;
    pthread_mutex_t mutex;
    pthread_cond_t event;
    bool running;
    bool command_pending;
    bool command_complete;
    bool busy;
    bool stop_requested;
    int result;
    struct dual_stepper_command command;
};

int motor_service_init(struct motor_service *service, bool use_mock);
void motor_service_shutdown(struct motor_service *service);
int motor_service_move(struct motor_service *service, int direction, unsigned int rotations);
void motor_service_request_stop(struct motor_service *service);
bool motor_service_is_busy(struct motor_service *service);

#endif
