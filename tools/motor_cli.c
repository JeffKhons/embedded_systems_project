#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "dual_stepper_uapi.h"

static void print_help(void)
{
    puts("commands: start FWD|BWD <rotations> <frequency_hz>, pause, resume, stop, quit");
}

int main(void)
{
    int fd = open(DUAL_STEPPER_DEVICE, O_RDWR | O_CLOEXEC);
    if (fd < 0) {
        perror("open " DUAL_STEPPER_DEVICE);
        return EXIT_FAILURE;
    }

    char line[128];
    print_help();
    while (fputs("motor> ", stdout), fflush(stdout), fgets(line, sizeof(line), stdin) != NULL) {
        line[strcspn(line, "\r\n")] = '\0';
        if (strncmp(line, "start ", 6) == 0) {
            char direction[8];
            struct dual_stepper_command command;
            if (sscanf(line, "start %7s %u %u", direction,
                       &command.rotations, &command.frequency_hz) != 3) {
                print_help();
                continue;
            }
            if (strcmp(direction, "FWD") == 0)
                command.direction = DUAL_STEPPER_FORWARD;
            else if (strcmp(direction, "BWD") == 0)
                command.direction = DUAL_STEPPER_BACKWARD;
            else {
                print_help();
                continue;
            }
            if (ioctl(fd, DUAL_STEPPER_START, &command) < 0)
                perror("ioctl START");
        } else if (strcmp(line, "pause") == 0) {
            if (ioctl(fd, DUAL_STEPPER_PAUSE) < 0) perror("ioctl PAUSE");
        } else if (strcmp(line, "resume") == 0) {
            if (ioctl(fd, DUAL_STEPPER_RESUME) < 0) perror("ioctl RESUME");
        } else if (strcmp(line, "stop") == 0) {
            if (ioctl(fd, DUAL_STEPPER_STOP) < 0) perror("ioctl STOP");
        } else if (strcmp(line, "quit") == 0 || strcmp(line, "exit") == 0) {
            break;
        } else {
            print_help();
        }
    }

    close(fd);
    return EXIT_SUCCESS;
}
