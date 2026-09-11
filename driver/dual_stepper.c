// SPDX-License-Identifier: GPL-2.0-only
/*
 * Raspberry Pi synchronous dual-stepper character driver.
 *
 * The driver exposes /dev/dualstepper. A high-resolution timer generates a
 * shared STEP waveform while opposite DIR levels make the paired motors move
 * the climbing mechanism in the same physical direction.
 */

#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/hrtimer.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#include <dual_stepper_uapi.h>

#define MOTOR_1_DIRECTION_GPIO 13
#define MOTOR_1_STEP_GPIO      19
#define MOTOR_2_DIRECTION_GPIO 24
#define MOTOR_2_STEP_GPIO      18
#define MOTOR_ENABLE_GPIO      12

#define STEPS_PER_ROTATION 3200U
#define MIN_FREQUENCY_HZ   1U
#define MAX_FREQUENCY_HZ   100000U
#define MAX_ROTATIONS      1000000U

struct stepper_state {
    enum dual_stepper_direction direction;
    u64 steps_remaining;
    u64 half_period_ns;
    bool running;
    bool paused;
    bool step_high;
};

static const unsigned int requested_gpios[] = {
    MOTOR_1_DIRECTION_GPIO,
    MOTOR_1_STEP_GPIO,
    MOTOR_2_DIRECTION_GPIO,
    MOTOR_2_STEP_GPIO,
    MOTOR_ENABLE_GPIO,
};

static struct stepper_state state;
static struct hrtimer step_timer;
static spinlock_t state_lock;
static DEFINE_MUTEX(command_lock);

static void set_enabled(bool enabled)
{
    gpio_set_value(MOTOR_ENABLE_GPIO, enabled ? 0 : 1);
}

static void set_direction(enum dual_stepper_direction direction)
{
    gpio_set_value(MOTOR_1_DIRECTION_GPIO,
                   direction == DUAL_STEPPER_FORWARD ? 0 : 1);
    gpio_set_value(MOTOR_2_DIRECTION_GPIO,
                   direction == DUAL_STEPPER_FORWARD ? 1 : 0);
}

static void set_step(bool high)
{
    gpio_set_value(MOTOR_1_STEP_GPIO, high);
    gpio_set_value(MOTOR_2_STEP_GPIO, high);
}

static enum hrtimer_restart step_timer_callback(struct hrtimer *timer)
{
    unsigned long flags;
    bool completed = false;

    spin_lock_irqsave(&state_lock, flags);
    if (!state.running || state.paused) {
        spin_unlock_irqrestore(&state_lock, flags);
        return HRTIMER_NORESTART;
    }

    state.step_high = !state.step_high;
    set_step(state.step_high);
    if (!state.step_high && --state.steps_remaining == 0) {
        state.running = false;
        completed = true;
    }

    if (!completed)
        hrtimer_forward_now(timer, ns_to_ktime(state.half_period_ns));
    spin_unlock_irqrestore(&state_lock, flags);

    if (completed) {
        set_enabled(false);
        pr_info("dualstepper: motion complete\n");
        return HRTIMER_NORESTART;
    }
    return HRTIMER_RESTART;
}

static int start_motion(const struct dual_stepper_command *command)
{
    unsigned long flags;

    if (command->direction != DUAL_STEPPER_FORWARD &&
        command->direction != DUAL_STEPPER_BACKWARD)
        return -EINVAL;
    if (command->rotations == 0 || command->rotations > MAX_ROTATIONS)
        return -ERANGE;
    if (command->frequency_hz < MIN_FREQUENCY_HZ ||
        command->frequency_hz > MAX_FREQUENCY_HZ)
        return -ERANGE;

    hrtimer_cancel(&step_timer);
    set_enabled(false);
    set_step(false);
    set_direction(command->direction);

    spin_lock_irqsave(&state_lock, flags);
    state.direction = command->direction;
    state.steps_remaining = command->rotations * STEPS_PER_ROTATION;
    state.half_period_ns = div_u64(1000000000ULL, command->frequency_hz * 2ULL);
    state.step_high = false;
    state.paused = false;
    state.running = true;
    spin_unlock_irqrestore(&state_lock, flags);

    set_enabled(true);
    hrtimer_start(&step_timer, ns_to_ktime(state.half_period_ns), HRTIMER_MODE_REL);
    pr_info("dualstepper: start direction=%d rotations=%u frequency=%uHz\n",
            command->direction, command->rotations, command->frequency_hz);
    return 0;
}

static void stop_motion(void)
{
    unsigned long flags;

    spin_lock_irqsave(&state_lock, flags);
    state.running = false;
    state.paused = false;
    state.steps_remaining = 0;
    spin_unlock_irqrestore(&state_lock, flags);

    hrtimer_cancel(&step_timer);
    set_step(false);
    set_enabled(false);
}

static long stepper_ioctl(struct file *file, unsigned int request,
                          unsigned long argument)
{
    struct dual_stepper_command command;
    unsigned long flags;
    u64 half_period_ns = 0;
    bool resume = false;
    int result = 0;

    (void)file;

    if (_IOC_TYPE(request) != DUAL_STEPPER_IOCTL_BASE)
        return -ENOTTY;

    mutex_lock(&command_lock);
    switch (request) {
    case DUAL_STEPPER_START:
        if (copy_from_user(&command, (void __user *)argument, sizeof(command)))
            result = -EFAULT;
        else
            result = start_motion(&command);
        break;

    case DUAL_STEPPER_PAUSE:
        spin_lock_irqsave(&state_lock, flags);
        if (state.running && !state.paused)
            state.paused = true;
        spin_unlock_irqrestore(&state_lock, flags);
        hrtimer_cancel(&step_timer);
        set_step(false);
        break;

    case DUAL_STEPPER_RESUME:
        spin_lock_irqsave(&state_lock, flags);
        if (state.running && state.paused) {
            state.paused = false;
            state.step_high = false;
            half_period_ns = state.half_period_ns;
            resume = true;
        }
        spin_unlock_irqrestore(&state_lock, flags);
        if (resume)
            hrtimer_start(&step_timer, ns_to_ktime(half_period_ns), HRTIMER_MODE_REL);
        break;

    case DUAL_STEPPER_STOP:
        stop_motion();
        break;

    default:
        result = -ENOTTY;
    }
    mutex_unlock(&command_lock);
    return result;
}

static const struct file_operations stepper_operations = {
    .owner = THIS_MODULE,
    .unlocked_ioctl = stepper_ioctl,
    .compat_ioctl = stepper_ioctl,
    .llseek = no_llseek,
};

static struct miscdevice stepper_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = "dualstepper",
    .fops = &stepper_operations,
    .mode = 0660,
};

static int request_output_gpio(unsigned int gpio)
{
    int result = gpio_request(gpio, "dualstepper");
    if (result != 0)
        return result;

    result = gpio_direction_output(gpio, 0);
    if (result != 0)
        gpio_free(gpio);
    return result;
}

static int __init dual_stepper_init(void)
{
    size_t index;
    int result;

    spin_lock_init(&state_lock);
    hrtimer_init(&step_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    step_timer.function = step_timer_callback;

    for (index = 0; index < ARRAY_SIZE(requested_gpios); ++index) {
        result = request_output_gpio(requested_gpios[index]);
        if (result != 0)
            goto free_gpios;
    }
    set_enabled(false);

    result = misc_register(&stepper_device);
    if (result != 0)
        goto free_gpios;
    pr_info("dualstepper: /dev/%s ready\n", stepper_device.name);
    return 0;

free_gpios:
    while (index > 0)
        gpio_free(requested_gpios[--index]);
    return result;
}

static void __exit dual_stepper_exit(void)
{
    size_t index;

    misc_deregister(&stepper_device);
    stop_motion();
    for (index = 0; index < ARRAY_SIZE(requested_gpios); ++index)
        gpio_free(requested_gpios[index]);
    pr_info("dualstepper: unloaded\n");
}

module_init(dual_stepper_init);
module_exit(dual_stepper_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Embedded Systems Project Group 8");
MODULE_DESCRIPTION("Synchronous dual-stepper motor driver for Raspberry Pi 3B");
