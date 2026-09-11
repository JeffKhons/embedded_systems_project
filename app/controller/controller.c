#include "controller.h"

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const char *controller_state_name(enum controller_state state)
{
    switch (state) {
    case CONTROLLER_IDLE: return "IDLE";
    case CONTROLLER_PROCESSING: return "PROCESSING";
    case CONTROLLER_MOVING: return "MOVING";
    case CONTROLLER_ARRIVED: return "ARRIVED";
    case CONTROLLER_STOPPED: return "STOPPED";
    case CONTROLLER_FAULT: return "FAULT";
    }
    return "UNKNOWN";
}

static void update_state(struct controller *controller,
                         enum controller_state state,
                         const char *message)
{
    int floor;

    pthread_mutex_lock(&controller->mutex);
    controller->state = state;
    floor = controller->current_floor;
    pthread_mutex_unlock(&controller->mutex);

    printf("[controller] state=%s floor=%d: %s\n",
           controller_state_name(state), floor, message);
    if (controller->status_sink != NULL)
        controller->status_sink(controller->status_context, state, floor, message);
}

int controller_init(struct controller *controller,
                    struct request_queue *queue,
                    struct motor_service *motor,
                    unsigned int rotations_per_floor,
                    controller_status_sink status_sink,
                    void *status_context)
{
    int error = pthread_mutex_init(&controller->mutex, NULL);
    if (error != 0)
        return error;

    controller->queue = queue;
    controller->motor = motor;
    controller->state = CONTROLLER_IDLE;
    controller->current_floor = 1;
    controller->target_floor = -1;
    controller->rotations_per_floor = rotations_per_floor;
    controller->target_confirmed = false;
    controller->status_sink = status_sink;
    controller->status_context = status_context;
    return 0;
}

void controller_destroy(struct controller *controller)
{
    pthread_mutex_destroy(&controller->mutex);
}

static void process_request(struct controller *controller, const struct request *request)
{
    int current_floor;

    update_state(controller, CONTROLLER_PROCESSING, request->order_id);

    pthread_mutex_lock(&controller->mutex);
    current_floor = controller->current_floor;
    pthread_mutex_unlock(&controller->mutex);

    if (request->target_floor == current_floor) {
        update_state(controller, CONTROLLER_ARRIVED, "already at target floor");
        return;
    }

    int difference = request->target_floor - current_floor;
    int direction = difference > 0 ? DUAL_STEPPER_FORWARD : DUAL_STEPPER_BACKWARD;
    unsigned int rotations = (unsigned int)abs(difference) * controller->rotations_per_floor;

    pthread_mutex_lock(&controller->mutex);
    controller->target_floor = request->target_floor;
    controller->target_confirmed = false;
    pthread_mutex_unlock(&controller->mutex);
    update_state(controller, CONTROLLER_MOVING, "motor command submitted");

    int result = motor_service_move(controller->motor, direction, rotations);

    pthread_mutex_lock(&controller->mutex);
    bool target_confirmed = controller->target_confirmed;
    if (result == 0 || target_confirmed) {
        controller->current_floor = request->target_floor;
        controller->target_floor = -1;
    }
    pthread_mutex_unlock(&controller->mutex);

    if (result == 0)
        update_state(controller, CONTROLLER_ARRIVED, "motion profile completed");
    else if (target_confirmed)
        update_state(controller, CONTROLLER_ARRIVED, "target confirmed by AprilTag");
    else if (result == ECANCELED)
        update_state(controller, CONTROLLER_STOPPED, "motion interrupted");
    else {
        char message[96];
        snprintf(message, sizeof(message), "motor error: %s", strerror(result));
        update_state(controller, CONTROLLER_FAULT, message);
    }
}

int controller_run(struct controller *controller)
{
    struct request request;
    int error;

    update_state(controller, CONTROLLER_IDLE, "ready");
    while ((error = request_queue_pop(controller->queue, &request)) == 0)
        process_request(controller, &request);

    return error == ECANCELED ? 0 : error;
}

void controller_handle_tag(struct controller *controller, int tag_id)
{
    bool reached_target = false;

    pthread_mutex_lock(&controller->mutex);
    if (controller->state == CONTROLLER_MOVING && controller->target_floor == tag_id) {
        controller->target_confirmed = true;
        reached_target = true;
    }
    pthread_mutex_unlock(&controller->mutex);

    if (reached_target) {
        printf("[vision] target AprilTag %d detected; stopping motor\n", tag_id);
        motor_service_request_stop(controller->motor);
    }
}

void controller_format_status(void *context, char *buffer, size_t size)
{
    struct controller *controller = context;
    enum controller_state state;
    int floor;

    pthread_mutex_lock(&controller->mutex);
    state = controller->state;
    floor = controller->current_floor;
    pthread_mutex_unlock(&controller->mutex);

    snprintf(buffer, size,
             "{\"type\":\"STATUS\",\"state\":\"%s\",\"floor\":%d,"
             "\"queue_depth\":%zu,\"motor_busy\":%s}\n",
             controller_state_name(state), floor,
             request_queue_count(controller->queue),
             motor_service_is_busy(controller->motor) ? "true" : "false");
}
