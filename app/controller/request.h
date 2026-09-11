#ifndef REQUEST_H
#define REQUEST_H

#define REQUEST_ID_SIZE 32

enum request_type {
    REQUEST_DELIVER = 1,
    REQUEST_PICKUP = 2,
};

struct request {
    enum request_type type;
    int target_floor;
    char order_id[REQUEST_ID_SIZE];
    char locker_id;
};

#endif
