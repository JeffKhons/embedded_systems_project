#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <stdbool.h>
#include <stddef.h>
#include <pthread.h>

#include "motor_service.h"
#include "request_queue.h"

enum controller_state {
    CONTROLLER_IDLE,
    CONTROLLER_PROCESSING,
    CONTROLLER_MOVING,
    CONTROLLER_ARRIVED,
    CONTROLLER_STOPPED,
    CONTROLLER_FAULT,
};

typedef void (*controller_status_sink)(void *context,
                                       enum controller_state state,
                                       int floor,
                                       const char *message);

struct controller {
    struct request_queue *queue;
    struct motor_service *motor;
    pthread_mutex_t mutex;
    enum controller_state state;
    int current_floor;
    int target_floor;
    unsigned int rotations_per_floor;
    bool target_confirmed;
    controller_status_sink status_sink;
    void *status_context;
};

int controller_init(struct controller *controller,
                    struct request_queue *queue,
                    struct motor_service *motor,
                    unsigned int rotations_per_floor,
                    controller_status_sink status_sink,
                    void *status_context);
void controller_destroy(struct controller *controller);
int controller_run(struct controller *controller);
void controller_handle_tag(struct controller *controller, int tag_id);
void controller_format_status(void *context, char *buffer, size_t size);
const char *controller_state_name(enum controller_state state);

#endif
