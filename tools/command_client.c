#include <arpa/inet.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define DEFAULT_PORT 8888

int main(int argc, char **argv)
{
    if (argc < 3 || argc > 4) {
        fprintf(stderr, "Usage: %s HOST FLOOR [DELIVER|PICKUP]\n", argv[0]);
        return EXIT_FAILURE;
    }

    char *end;
    long floor = strtol(argv[2], &end, 10);
    if (*argv[2] == '\0' || *end != '\0' || floor < 1 || floor > 100) {
        fputs("FLOOR must be between 1 and 100\n", stderr);
        return EXIT_FAILURE;
    }
    const char *type = argc == 4 ? argv[3] : "DELIVER";
    if (strcmp(type, "DELIVER") != 0 && strcmp(type, "PICKUP") != 0) {
        fputs("command must be DELIVER or PICKUP\n", stderr);
        return EXIT_FAILURE;
    }

    int socket_fd = socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
    if (socket_fd < 0) {
        perror("socket");
        return EXIT_FAILURE;
    }
    struct sockaddr_in address = {.sin_family = AF_INET, .sin_port = htons(DEFAULT_PORT)};
    if (inet_pton(AF_INET, argv[1], &address.sin_addr) != 1 ||
        connect(socket_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("connect");
        close(socket_fd);
        return EXIT_FAILURE;
    }

    char command[160];
    snprintf(command, sizeof(command),
             "{\"CMD\":\"%s\",\"FLOOR\":%ld,\"ID\":\"DEMO-001\"}\n",
             type, floor);
    if (send(socket_fd, command, strlen(command), 0) < 0) {
        perror("send");
        close(socket_fd);
        return EXIT_FAILURE;
    }

    char response[512];
    for (;;) {
        ssize_t count = read(socket_fd, response, sizeof(response) - 1U);
        if (count <= 0)
            break;
        response[count] = '\0';
        fputs(response, stdout);
        if (strstr(response, "\"state\":\"ARRIVED\"") != NULL)
            break;
    }
    close(socket_fd);
    return EXIT_SUCCESS;
}
