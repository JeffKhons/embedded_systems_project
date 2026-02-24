/*
 * motor_writer.c – CLI utility to control /dev/dualstepper
 *
 * Build : gcc motor_writer.c -o motor_writer
 * Usage : sudo ./motor_writer
 *
 * Example session:
 *   > start FWD 2 800
 *   > pause
 *   > resume
 *   > stop
 *   > quit
 */

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/ioctl.h>

/* ==== must match kernel driver ==== */
#define STEP_IOCTL_BASE    's'
#define STEP_CMD_START _IOW(STEP_IOCTL_BASE, 0, struct step_cmd)
#define STEP_CMD_PAUSE _IO (STEP_IOCTL_BASE, 1)
#define STEP_CMD_RESUME _IO(STEP_IOCTL_BASE, 2)
#define STEP_CMD_STOP _IO  (STEP_IOCTL_BASE, 3)

struct step_cmd {
    int dir;       /* 0 = forward, 1 = backward */
    int cycles;    /* full revolutions */
    int frequency; /* STEP freq. (Hz)  */
};

int main(void) {
    int fd = open("/dev/dualstepper", O_RDWR);
    if (fd < 0) {
        perror("open /dev/dualstepper");
        return 1;
    }

    char line[128];
    while (1) {
        printf("cmd> ");
        if (!fgets(line, sizeof(line), stdin))
            break;                 /* EOF */

        /* Remove trailing line break */
        line[strcspn(line, "\r\n")] = 0;

        if (strncmp(line, "start", 5) == 0) {
            struct step_cmd cmd;
            char dir_str[8];
            if (sscanf(line, "start %7s %d %d",
                    dir_str, &cmd.cycles, &cmd.frequency) == 3) {
                if (strcmp(dir_str, "FWD") == 0) cmd.dir = 0;
                else if (strcmp(dir_str, "BWD") == 0) cmd.dir = 1;
                else { printf("方向請輸入 FWD 或 BWD\n"); continue; }

                if (ioctl(fd, STEP_CMD_START, &cmd) < 0)
                    perror("ioctl START");
            } else {
                printf("格式: start FWD|BWD <cycles> <freq>\n");
            }

        } else if (strcmp(line, "pause" ) == 0 || strcmp(line, "p") == 0) {
            if (ioctl(fd, STEP_CMD_PAUSE) < 0) perror("ioctl PAUSE");

        } else if (strcmp(line, "resume") == 0 || strcmp(line, "r") == 0) {
            if (ioctl(fd, STEP_CMD_RESUME) < 0) perror("ioctl RESUME");

        } else if (strcmp(line, "stop") == 0 || strcmp(line, "s") == 0) {
            if (ioctl(fd, STEP_CMD_STOP) < 0) perror("ioctl STOP");
            break;  /* 跳出迴圈，結束程式 */

        } else if (strcmp(line, "exit") == 0) {
            break;

        } else {
            printf("指令: start/pause/resume/stop\n");
        }
    }

    close(fd);
    return 0;
}