#define _GNU_SOURCE

#include "tcp_server.h"

#include <arpa/inet.h>
#include <errno.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

static void send_text(int fd, const char *text)
{
    size_t remaining = strlen(text);
    const char *cursor = text;

    while (remaining > 0U) {
        ssize_t written = send(fd, cursor, remaining, MSG_NOSIGNAL | MSG_DONTWAIT);
        if (written < 0) {
            if (errno == EINTR)
                continue;
            return;
        }
        if (written == 0)
            return;
        cursor += written;
        remaining -= (size_t)written;
    }
}

static bool parse_integer(const char *json, const char *key, int *value)
{
    const char *field = strstr(json, key);
    if (field == NULL)
        return false;
    field = strchr(field, ':');
    if (field == NULL)
        return false;

    char *end;
    long parsed = strtol(field + 1, &end, 10);
    if (end == field + 1 || parsed < 1 || parsed > 100)
        return false;
    *value = (int)parsed;
    return true;
}

static char parse_locker(const char *json)
{
    const char *field = strstr(json, "\"LOCKER\"");
    if (field == NULL)
        return '-';
    field = strchr(field, ':');
    if (field == NULL)
        return '-';
    field = strchr(field, '"');
    return field != NULL && field[1] != '\0' ? field[1] : '-';
}

static void parse_order_id(const char *json, char *output, size_t size)
{
    const char *field = strstr(json, "\"ID\"");
    if (field != NULL)
        field = strchr(field, ':');
    if (field != NULL)
        field = strchr(field, '"');
    if (field != NULL) {
        const char *end = strchr(field + 1, '"');
        if (end != NULL) {
            size_t length = (size_t)(end - field - 1);
            if (length >= size)
                length = size - 1U;
            memcpy(output, field + 1, length);
            output[length] = '\0';
            return;
        }
    }
    snprintf(output, size, "CMD-%ld", (long)time(NULL));
}

static void handle_command(struct tcp_server *server, int fd, const char *command)
{
    if (strstr(command, "\"CMD\":\"STATUS\"") != NULL ||
        strcmp(command, "STATUS") == 0) {
        char status[256];
        server->status_provider(server->status_context, status, sizeof(status));
        send_text(fd, status);
        return;
    }

    bool deliver = strstr(command, "DELIVER") != NULL;
    bool pickup = strstr(command, "PICKUP") != NULL;
    struct request request;
    if ((!deliver && !pickup) ||
        !parse_integer(command, "\"FLOOR\"", &request.target_floor)) {
        send_text(fd, "{\"status\":\"ERROR\",\"reason\":\"invalid command\"}\n");
        return;
    }

    request.type = deliver ? REQUEST_DELIVER : REQUEST_PICKUP;
    request.locker_id = parse_locker(command);
    parse_order_id(command, request.order_id, sizeof(request.order_id));

    int error = request_queue_push(server->queue, &request);
    if (error == 0)
        send_text(fd, "{\"status\":\"QUEUED\"}\n");
    else if (error == ENOSPC)
        send_text(fd, "{\"status\":\"BUSY\",\"reason\":\"queue full\"}\n");
    else
        send_text(fd, "{\"status\":\"ERROR\",\"reason\":\"shutting down\"}\n");
}

static void close_client(struct tcp_server *server, size_t index)
{
    pthread_mutex_lock(&server->mutex);
    if (server->clients[index].fd >= 0)
        close(server->clients[index].fd);
    server->clients[index].fd = -1;
    server->clients[index].used = 0U;
    pthread_mutex_unlock(&server->mutex);
}

static void consume_client_data(struct tcp_server *server, size_t index)
{
    struct tcp_client *client = &server->clients[index];
    ssize_t count = recv(client->fd, client->input + client->used,
                         sizeof(client->input) - client->used - 1U, 0);
    if (count <= 0) {
        close_client(server, index);
        return;
    }

    client->used += (size_t)count;
    client->input[client->used] = '\0';

    char *start = client->input;
    char *newline;
    while ((newline = strchr(start, '\n')) != NULL) {
        *newline = '\0';
        if (*start != '\0')
            handle_command(server, client->fd, start);
        start = newline + 1;
    }

    size_t remaining = client->used - (size_t)(start - client->input);
    memmove(client->input, start, remaining);
    client->used = remaining;
    if (client->used == sizeof(client->input) - 1U) {
        send_text(client->fd, "{\"status\":\"ERROR\",\"reason\":\"command too long\"}\n");
        close_client(server, index);
    }
}

static void accept_client(struct tcp_server *server)
{
    int client_fd = accept4(server->listen_fd, NULL, NULL, SOCK_CLOEXEC);
    if (client_fd < 0)
        return;

    pthread_mutex_lock(&server->mutex);
    size_t index;
    for (index = 0; index < TCP_SERVER_MAX_CLIENTS; ++index) {
        if (server->clients[index].fd < 0) {
            server->clients[index].fd = client_fd;
            server->clients[index].used = 0U;
            break;
        }
    }
    pthread_mutex_unlock(&server->mutex);

    if (index == TCP_SERVER_MAX_CLIENTS) {
        send_text(client_fd, "{\"status\":\"BUSY\",\"reason\":\"too many clients\"}\n");
        close(client_fd);
    } else {
        printf("[tcp] client connected in slot %zu\n", index);
    }
}

static void *server_thread(void *argument)
{
    struct tcp_server *server = argument;

    while (atomic_load(&server->running)) {
        struct pollfd descriptors[1 + TCP_SERVER_MAX_CLIENTS];
        descriptors[0].fd = server->listen_fd;
        descriptors[0].events = POLLIN;

        pthread_mutex_lock(&server->mutex);
        for (size_t i = 0; i < TCP_SERVER_MAX_CLIENTS; ++i) {
            descriptors[i + 1U].fd = server->clients[i].fd;
            descriptors[i + 1U].events = POLLIN;
        }
        pthread_mutex_unlock(&server->mutex);

        int ready = poll(descriptors, 1 + TCP_SERVER_MAX_CLIENTS, 250);
        if (ready < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        if (ready == 0)
            continue;
        if ((descriptors[0].revents & POLLIN) != 0)
            accept_client(server);

        for (size_t i = 0; i < TCP_SERVER_MAX_CLIENTS; ++i) {
            if ((descriptors[i + 1U].revents & POLLIN) != 0)
                consume_client_data(server, i);
            else if ((descriptors[i + 1U].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)
                close_client(server, i);
        }
    }
    return NULL;
}

int tcp_server_start(struct tcp_server *server,
                     int port,
                     struct request_queue *queue,
                     tcp_status_provider status_provider,
                     void *status_context)
{
    memset(server, 0, sizeof(*server));
    atomic_init(&server->running, false);
    server->port = port;
    server->listen_fd = -1;
    server->queue = queue;
    server->status_provider = status_provider;
    server->status_context = status_context;
    for (size_t i = 0; i < TCP_SERVER_MAX_CLIENTS; ++i)
        server->clients[i].fd = -1;

    int error = pthread_mutex_init(&server->mutex, NULL);
    if (error != 0)
        return error;

    server->listen_fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (server->listen_fd < 0) {
        error = errno;
        pthread_mutex_destroy(&server->mutex);
        return error;
    }

    int enabled = 1;
    setsockopt(server->listen_fd, SOL_SOCKET, SO_REUSEADDR, &enabled, sizeof(enabled));
    struct sockaddr_in address = {
        .sin_family = AF_INET,
        .sin_addr.s_addr = htonl(INADDR_ANY),
        .sin_port = htons((unsigned short)port),
    };
    if (bind(server->listen_fd, (struct sockaddr *)&address, sizeof(address)) < 0 ||
        listen(server->listen_fd, TCP_SERVER_MAX_CLIENTS) < 0) {
        error = errno;
        close(server->listen_fd);
        pthread_mutex_destroy(&server->mutex);
        return error;
    }

    atomic_store(&server->running, true);
    error = pthread_create(&server->thread, NULL, server_thread, server);
    if (error != 0) {
        atomic_store(&server->running, false);
        close(server->listen_fd);
        pthread_mutex_destroy(&server->mutex);
        return error;
    }
    printf("[tcp] listening on port %d\n", port);
    return 0;
}

void tcp_server_stop(struct tcp_server *server)
{
    atomic_store(&server->running, false);
    pthread_join(server->thread, NULL);

    pthread_mutex_lock(&server->mutex);
    for (size_t i = 0; i < TCP_SERVER_MAX_CLIENTS; ++i) {
        if (server->clients[i].fd >= 0)
            close(server->clients[i].fd);
    }
    pthread_mutex_unlock(&server->mutex);
    close(server->listen_fd);
    pthread_mutex_destroy(&server->mutex);
}

void tcp_server_publish(void *context,
                        enum controller_state state,
                        int floor,
                        const char *message)
{
    struct tcp_server *server = context;
    char output[256];

    snprintf(output, sizeof(output),
             "{\"type\":\"EVENT\",\"state\":\"%s\",\"floor\":%d,"
             "\"message\":\"%.96s\"}\n",
             controller_state_name(state), floor, message);

    pthread_mutex_lock(&server->mutex);
    for (size_t i = 0; i < TCP_SERVER_MAX_CLIENTS; ++i) {
        if (server->clients[i].fd >= 0)
            send_text(server->clients[i].fd, output);
    }
    pthread_mutex_unlock(&server->mutex);
}
