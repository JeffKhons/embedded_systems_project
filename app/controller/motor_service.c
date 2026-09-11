#ifndef _GNU_SOURCE
#define _GNU_SOURCE
#endif

#include "motor_service.h"

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include <sched.h>

#define MOTOR_STEPS_PER_ROTATION 3200ULL
#define MOTOR_RAMP_FREQUENCY_HZ  200U
#define MOTOR_CRUISE_FREQUENCY_HZ 800U

static struct timespec deadline_after_ms(uint64_t milliseconds)
{
    struct timespec deadline;

    clock_gettime(CLOCK_REALTIME, &deadline);
    deadline.tv_sec += (time_t)(milliseconds / 1000ULL);
    deadline.tv_nsec += (long)((milliseconds % 1000ULL) * 1000000ULL);
    if (deadline.tv_nsec >= 1000000000L) {
        deadline.tv_sec++;
        deadline.tv_nsec -= 1000000000L;
    }
    return deadline;
}

static bool wait_for_segment(struct motor_service *service,
                             unsigned int rotations,
                             unsigned int frequency_hz)
{
    uint64_t duration_ms;
    struct timespec deadline;

    if (motor_backend_is_mock(&service->backend))
        duration_ms = rotations * 40ULL + 100ULL;
    else
        duration_ms = rotations * MOTOR_STEPS_PER_ROTATION * 1000ULL / frequency_hz;

    deadline = deadline_after_ms(duration_ms);
    pthread_mutex_lock(&service->mutex);
    while (service->running && !service->stop_requested) {
        int error = pthread_cond_timedwait(&service->event, &service->mutex, &deadline);
        if (error == ETIMEDOUT)
            break;
    }
    bool completed = service->running && !service->stop_requested;
    pthread_mutex_unlock(&service->mutex);
    return completed;
}

static int run_segment(struct motor_service *service,
                       int direction,
                       unsigned int rotations,
                       unsigned int frequency_hz)
{
    struct dual_stepper_command command = {
        .direction = direction,
        .rotations = rotations,
        .frequency_hz = frequency_hz,
    };
    int error = motor_backend_start(&service->backend, &command);

    if (error != 0)
        return error;
    if (!wait_for_segment(service, rotations, frequency_hz)) {
        motor_backend_stop(&service->backend);
        return ECANCELED;
    }
    return 0;
}

static int execute_profile(struct motor_service *service,
                           const struct dual_stepper_command *command)
{
    int error;

    if (command->rotations < 3U)
        return run_segment(service, command->direction, command->rotations,
                           MOTOR_RAMP_FREQUENCY_HZ);

    error = run_segment(service, command->direction, 1U, MOTOR_RAMP_FREQUENCY_HZ);
    if (error != 0)
        return error;

    error = run_segment(service, command->direction, command->rotations - 2U,
                        MOTOR_CRUISE_FREQUENCY_HZ);
    if (error != 0)
        return error;

    return run_segment(service, command->direction, 1U, MOTOR_RAMP_FREQUENCY_HZ);
}

static void *motor_worker(void *argument)
{
    struct motor_service *service = argument;
#ifdef __linux__
    cpu_set_t cpu_set;

    CPU_ZERO(&cpu_set);
    CPU_SET(2, &cpu_set);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set), &cpu_set) != 0)
        fputs("[motor] CPU affinity unavailable; continuing without pinning\n", stderr);
#endif

    pthread_mutex_lock(&service->mutex);
    while (service->running) {
        while (service->running && !service->command_pending)
            pthread_cond_wait(&service->event, &service->mutex);
        if (!service->running)
            break;

        struct dual_stepper_command command = service->command;
        service->command_pending = false;
        service->busy = true;
        service->stop_requested = false;
        pthread_mutex_unlock(&service->mutex);

        int result = execute_profile(service, &command);

        pthread_mutex_lock(&service->mutex);
        service->result = result;
        service->busy = false;
        service->command_complete = true;
        pthread_cond_broadcast(&service->event);
    }
    pthread_mutex_unlock(&service->mutex);
    return NULL;
}

static int create_worker(struct motor_service *service)
{
    pthread_attr_t attributes;
    struct sched_param scheduling = {.sched_priority = 50};
    int error;

    pthread_attr_init(&attributes);
    pthread_attr_setinheritsched(&attributes, PTHREAD_EXPLICIT_SCHED);
    pthread_attr_setschedpolicy(&attributes, SCHED_FIFO);
    pthread_attr_setschedparam(&attributes, &scheduling);
    error = pthread_create(&service->thread, &attributes, motor_worker, service);
    pthread_attr_destroy(&attributes);

    if (error == 0) {
        puts("[motor] worker started with SCHED_FIFO priority 50");
        return 0;
    }

    fprintf(stderr, "[motor] real-time scheduling unavailable (%s); using SCHED_OTHER\n",
            strerror(error));
    return pthread_create(&service->thread, NULL, motor_worker, service);
}

int motor_service_init(struct motor_service *service, bool use_mock)
{
    int error;

    memset(service, 0, sizeof(*service));
    error = pthread_mutex_init(&service->mutex, NULL);
    if (error != 0)
        return error;
    error = pthread_cond_init(&service->event, NULL);
    if (error != 0) {
        pthread_mutex_destroy(&service->mutex);
        return error;
    }
    error = motor_backend_open(&service->backend, use_mock);
    if (error != 0) {
        pthread_cond_destroy(&service->event);
        pthread_mutex_destroy(&service->mutex);
        return error;
    }

    service->running = true;
    error = create_worker(service);
    if (error != 0) {
        service->running = false;
        motor_backend_close(&service->backend);
        pthread_cond_destroy(&service->event);
        pthread_mutex_destroy(&service->mutex);
    }
    return error;
}

void motor_service_shutdown(struct motor_service *service)
{
    pthread_mutex_lock(&service->mutex);
    service->running = false;
    service->stop_requested = true;
    pthread_cond_broadcast(&service->event);
    pthread_mutex_unlock(&service->mutex);

    motor_backend_stop(&service->backend);
    pthread_join(service->thread, NULL);
    motor_backend_close(&service->backend);
    pthread_cond_destroy(&service->event);
    pthread_mutex_destroy(&service->mutex);
}

int motor_service_move(struct motor_service *service, int direction, unsigned int rotations)
{
    pthread_mutex_lock(&service->mutex);
    if (!service->running) {
        pthread_mutex_unlock(&service->mutex);
        return ECANCELED;
    }
    if (service->busy || service->command_pending) {
        pthread_mutex_unlock(&service->mutex);
        return EBUSY;
    }

    service->command.direction = direction;
    service->command.rotations = rotations;
    service->command.frequency_hz = MOTOR_CRUISE_FREQUENCY_HZ;
    service->command_pending = true;
    service->command_complete = false;
    pthread_cond_signal(&service->event);

    while (service->running && !service->command_complete)
        pthread_cond_wait(&service->event, &service->mutex);

    int result = service->running ? service->result : ECANCELED;
    pthread_mutex_unlock(&service->mutex);
    return result;
}

void motor_service_request_stop(struct motor_service *service)
{
    bool should_stop;

    pthread_mutex_lock(&service->mutex);
    should_stop = service->busy;
    service->stop_requested = true;
    pthread_cond_broadcast(&service->event);
    pthread_mutex_unlock(&service->mutex);

    if (should_stop)
        motor_backend_stop(&service->backend);
}

bool motor_service_is_busy(struct motor_service *service)
{
    bool busy;

    pthread_mutex_lock(&service->mutex);
    busy = service->busy || service->command_pending;
    pthread_mutex_unlock(&service->mutex);
    return busy;
}
