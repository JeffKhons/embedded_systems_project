#ifndef REQUEST_QUEUE_H
#define REQUEST_QUEUE_H

#include <stdbool.h>
#include <stddef.h>
#include <pthread.h>

#include "request.h"

#define REQUEST_QUEUE_CAPACITY 8

struct request_queue {
    struct request items[REQUEST_QUEUE_CAPACITY];
    size_t head;
    size_t tail;
    size_t count;
    bool closed;
    pthread_mutex_t mutex;
    pthread_cond_t not_empty;
};

int request_queue_init(struct request_queue *queue);
void request_queue_destroy(struct request_queue *queue);
int request_queue_push(struct request_queue *queue, const struct request *request);
int request_queue_pop(struct request_queue *queue, struct request *request);
size_t request_queue_count(struct request_queue *queue);
void request_queue_close(struct request_queue *queue);

#endif
