#include "request_queue.h"

#include <errno.h>
#include <string.h>

int request_queue_init(struct request_queue *queue)
{
    int error;

    memset(queue, 0, sizeof(*queue));
    error = pthread_mutex_init(&queue->mutex, NULL);
    if (error != 0)
        return error;

    error = pthread_cond_init(&queue->not_empty, NULL);
    if (error != 0) {
        pthread_mutex_destroy(&queue->mutex);
        return error;
    }
    return 0;
}

void request_queue_destroy(struct request_queue *queue)
{
    pthread_cond_destroy(&queue->not_empty);
    pthread_mutex_destroy(&queue->mutex);
}

int request_queue_push(struct request_queue *queue, const struct request *request)
{
    int result = 0;

    pthread_mutex_lock(&queue->mutex);
    if (queue->closed)
        result = ECANCELED;
    else if (queue->count == REQUEST_QUEUE_CAPACITY)
        result = ENOSPC;
    else {
        queue->items[queue->tail] = *request;
        queue->tail = (queue->tail + 1U) % REQUEST_QUEUE_CAPACITY;
        queue->count++;
        pthread_cond_signal(&queue->not_empty);
    }
    pthread_mutex_unlock(&queue->mutex);
    return result;
}

int request_queue_pop(struct request_queue *queue, struct request *request)
{
    pthread_mutex_lock(&queue->mutex);
    while (queue->count == 0U && !queue->closed)
        pthread_cond_wait(&queue->not_empty, &queue->mutex);

    if (queue->count == 0U && queue->closed) {
        pthread_mutex_unlock(&queue->mutex);
        return ECANCELED;
    }

    *request = queue->items[queue->head];
    queue->head = (queue->head + 1U) % REQUEST_QUEUE_CAPACITY;
    queue->count--;
    pthread_mutex_unlock(&queue->mutex);
    return 0;
}

size_t request_queue_count(struct request_queue *queue)
{
    size_t count;

    pthread_mutex_lock(&queue->mutex);
    count = queue->count;
    pthread_mutex_unlock(&queue->mutex);
    return count;
}

void request_queue_close(struct request_queue *queue)
{
    pthread_mutex_lock(&queue->mutex);
    queue->closed = true;
    pthread_cond_broadcast(&queue->not_empty);
    pthread_mutex_unlock(&queue->mutex);
}
