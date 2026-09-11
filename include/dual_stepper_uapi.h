#ifndef DUAL_STEPPER_UAPI_H
#define DUAL_STEPPER_UAPI_H

#include <linux/ioctl.h>
#include <linux/types.h>

#define DUAL_STEPPER_DEVICE "/dev/dualstepper"

enum dual_stepper_direction {
    DUAL_STEPPER_FORWARD = 0,
    DUAL_STEPPER_BACKWARD = 1,
};

struct dual_stepper_command {
    __s32 direction;
    __u32 rotations;
    __u32 frequency_hz;
};

#define DUAL_STEPPER_IOCTL_BASE   's'
#define DUAL_STEPPER_START  _IOW(DUAL_STEPPER_IOCTL_BASE, 0, struct dual_stepper_command)
#define DUAL_STEPPER_PAUSE  _IO(DUAL_STEPPER_IOCTL_BASE, 1)
#define DUAL_STEPPER_RESUME _IO(DUAL_STEPPER_IOCTL_BASE, 2)
#define DUAL_STEPPER_STOP   _IO(DUAL_STEPPER_IOCTL_BASE, 3)

#endif
