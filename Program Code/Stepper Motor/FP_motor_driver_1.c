/*
 * FP_motor_driver.c – synchronous dual-stepper kernel driver
 * for Raspberry Pi 3B (BCM2837).
 *
 * This file is the C-language kernel-space rewrite of the original
 * Python Driver.py / motor_Driver.py.  A high-resolution timer (hrtimer)
 * generates phase-locked STEP pulses for two motors.
 *
 * Build : make -C /lib/modules/$(uname -r)/build M=$PWD modules
 * Load  : sudo insmod fp_motor_driver.ko
 * IOCTL : /dev/dualstepper  (see sample user-space writer)
 *
 * (c) 2025  Hugo Wang – GPL-2.0
 */

/* ======  header file ====== */
#include <linux/module.h>      /* MODULE_* macros, THIS_MODULE           */
#include <linux/kernel.h>      /* printk / pr_* helpers                  */
#include <linux/gpio.h>        /* Linux GPIO API                         */
#include <linux/hrtimer.h>     /* high-resolution timer                  */
#include <linux/miscdevice.h>  /* misc_register => /dev/  node           */
#include <linux/fs.h>          /* file_operations / ioctl                */
#include <linux/uaccess.h>     /* copy_{to,from}_user                    */
#include <linux/spinlock.h>    /* spin locks                             */
#include <linux/ktime.h>       /* ktime_* helpers                        */

#define M1_DIR      13
#define M1_STEP     19
#define M2_DIR      24
#define M2_STEP     18
#define EN_PIN      12

#define STEPS_PER_CYCLE   3200   /* 1 rev = 200 full-step × 1/16 micro-step */
#define LENGTH_PER_CYCLE  205000  /* 對應 1 rev 時機構移動的長度 (µm) */

enum step_dir { DIR_FWD = 0, DIR_BWD = 1 };

#define STEP_IOCTL_BASE      's'
#define STEP_CMD_START   _IOW(STEP_IOCTL_BASE, 0, struct step_cmd)
#define STEP_CMD_PAUSE   _IO (STEP_IOCTL_BASE, 1)
#define STEP_CMD_RESUME  _IO (STEP_IOCTL_BASE, 2)
#define STEP_CMD_STOP    _IO (STEP_IOCTL_BASE, 3)

struct step_cmd {
    int dir;        /* 0 = forward, 1 = backward */
    int cycles;     /* number of revolutions */
    int frequency;  /* STEP pulse frequency (Hz, full wave) */
};

/* Motor 定義 */
struct motor {
    int dir_pin;
    int step_pin;
};

static struct motor motors[2] = {
    { M1_DIR, M1_STEP },
    { M2_DIR, M2_STEP },
};

/* 全域狀態 (受 ctrl_lock 保護) */
static struct {
    enum step_dir dir;
    int steps_remaining;
    u64 half_period_ns;
    bool running;
    bool paused;
    u32  length_per_step_um;  /* µm per step, integer */
} ctrl;

/* 機構資訊 (不需鎖，僅由 timer callback 序列更新) */
static struct {
    u32 location;      /* 已走過的距離 (µm) */
    u32 length;        /* 剩餘距離 (µm) */
} mach_info;

/* 每步對應的長度 (µm) */
static u32 length_per_step;

/* 高解析度定時器 */
static struct hrtimer step_timer;
static spinlock_t ctrl_lock;

/* GPIO helper */
static void gpio_setup_one(int gpio)
{
    if (gpio_request(gpio, "dual_stepper") == 0)
        gpio_direction_output(gpio, 0);
}

static void setup_gpio_pins(void)
{
    int i;
    for (i = 0; i < 2; ++i) {
        gpio_setup_one(motors[i].dir_pin);
        gpio_setup_one(motors[i].step_pin);
    }
    gpio_setup_one(EN_PIN);
}

static void free_gpio_pins(void)
{
    int i;
    for (i = 0; i < 2; ++i) {
        gpio_free(motors[i].dir_pin);
        gpio_free(motors[i].step_pin);
    }
    gpio_free(EN_PIN);
}

static void set_enable(bool on)
{
    /* Active-Low：on==true 拉低；on==false 拉高 */
    gpio_set_value(EN_PIN, on ? 0 : 1);
}

static void set_direction(enum step_dir dir)
{
    gpio_set_value(motors[0].dir_pin, dir == DIR_FWD ? 0 : 1);
    gpio_set_value(motors[1].dir_pin, dir == DIR_FWD ? 1 : 0);
}

/* hrtimer callback：產生 STEP 脈波並更新狀態 */
static enum hrtimer_restart stepper_hrtimer_callback(struct hrtimer *timer)
{
    unsigned long flags;
    bool step_state;

    spin_lock_irqsave(&ctrl_lock, flags);
    if (!ctrl.running || ctrl.paused) {
        spin_unlock_irqrestore(&ctrl_lock, flags);
        pr_info("dualstepper: not running\n");
        return HRTIMER_NORESTART;
    }

    /* 切換 STEP 腳電平 */
    /* 利用 low→high→low 兩次 callback 完成一次 full wave */
    step_state = !gpio_get_value(motors[0].step_pin);
    gpio_set_value(motors[0].step_pin, step_state);
    gpio_set_value(motors[1].step_pin, step_state);

    /* 每當脈波回到 low 時，視為完成一次 full-step */
    if (!step_state) {
        ctrl.steps_remaining--;
        /* 更新距離與位置 */
        if (ctrl.dir == DIR_FWD)
            mach_info.location += length_per_step;
        else
            mach_info.location -= length_per_step;

        if (mach_info.length > length_per_step)
        	mach_info.length -= length_per_step;
        else
        	mach_info.length = 0;

        /* 進度訊息 (大約每 1000 steps 顯示一次) */
        if (ctrl.steps_remaining <= STEPS_PER_CYCLE * 50 /* arbitrary threshold */
            && (ctrl.steps_remaining % 1000) == 0) {
            pr_info("dualstepper: %d mm remaining\n", mach_info.length / 1000);
        }
    }

    /* 若還有步數，排程下一次 callback；否則結束 */
    if (ctrl.steps_remaining > 0) {
        hrtimer_forward_now(timer, ns_to_ktime(ctrl.half_period_ns));
        spin_unlock_irqrestore(&ctrl_lock, flags);
        return HRTIMER_RESTART;
    } else {
        /* 完成 */
        ctrl.running = false;
        ctrl.paused  = true;
        spin_unlock_irqrestore(&ctrl_lock, flags);

        set_enable(false);
        pr_info("dualstepper: Rotation completed.\n");
        return HRTIMER_NORESTART;
    }
}

static long stepper_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct step_cmd ucmd;
    unsigned long flags;

    switch (cmd) {
    case STEP_CMD_START:
        if (copy_from_user(&ucmd, (void __user *)arg, sizeof(ucmd)))
            return -EFAULT;
        if (ucmd.cycles <= 0 || ucmd.frequency <= 0)
            return -EINVAL;

        /* 計算全域參數 */
        length_per_step = LENGTH_PER_CYCLE / STEPS_PER_CYCLE;
        mach_info.location = 0;
        mach_info.length   = ucmd.cycles * LENGTH_PER_CYCLE;

        spin_lock_irqsave(&ctrl_lock, flags);
        ctrl.dir             = ucmd.dir ? DIR_BWD : DIR_FWD;
        ctrl.steps_remaining = ucmd.cycles * STEPS_PER_CYCLE;
        ctrl.half_period_ns  = (1000000000ULL / ucmd.frequency) / 2;
        ctrl.running         = true;
        ctrl.paused          = false;
        spin_unlock_irqrestore(&ctrl_lock, flags);

        set_direction(ctrl.dir);
        set_enable(true);
        pr_info("dualstepper: START dir=%s cycles=%d freq=%dHz\n",
                ctrl.dir == DIR_FWD ? "FWD" : "BWD",
                ucmd.cycles, ucmd.frequency);

        /* 啟動 hrtimer */
        hrtimer_start(&step_timer,
                      ns_to_ktime(ctrl.half_period_ns),
                      HRTIMER_MODE_REL);
        break;

    case STEP_CMD_PAUSE:
        spin_lock_irqsave(&ctrl_lock, flags);
        ctrl.paused = true;
        spin_unlock_irqrestore(&ctrl_lock, flags);
        hrtimer_cancel(&step_timer);
        pr_info("dualstepper: PAUSE\n");
        break;

    case STEP_CMD_RESUME:
        spin_lock_irqsave(&ctrl_lock, flags);
        if (ctrl.running && ctrl.paused) {
            ctrl.paused = false;
            hrtimer_start(&step_timer,
                          ns_to_ktime(ctrl.half_period_ns),
                          HRTIMER_MODE_REL);
            pr_info("dualstepper: RESUME\n");
        }
        spin_unlock_irqrestore(&ctrl_lock, flags);
        break;

    case STEP_CMD_STOP:
        spin_lock_irqsave(&ctrl_lock, flags);
        ctrl.running = false;
        ctrl.paused  = false;
        ctrl.steps_remaining = 0;
        mach_info.length     = 0;
        spin_unlock_irqrestore(&ctrl_lock, flags);
        hrtimer_cancel(&step_timer);
        set_enable(false);
        pr_info("dualstepper: STOP\n");
        break;

    default:
        return -ENOTTY;
    }
    return 0;
}

static const struct file_operations stepper_fops = {
    .owner          = THIS_MODULE,
    .unlocked_ioctl = stepper_ioctl,
    .compat_ioctl   = stepper_ioctl,
};

static struct miscdevice stepper_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = "dualstepper",
    .fops  = &stepper_fops,
};

static int __init dual_stepper_init(void)
{
    int ret;
    spin_lock_init(&ctrl_lock);
    memset(&ctrl, 0, sizeof(ctrl));
    setup_gpio_pins();
    set_enable(false);

    hrtimer_init(&step_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    step_timer.function = stepper_hrtimer_callback;

    ret = misc_register(&stepper_miscdev);
    if (ret) {
        free_gpio_pins();
        return ret;
    }
    pr_info("dualstepper: module loaded - /dev/%s ready\n",
            stepper_miscdev.name);
    return 0;
}

static void __exit dual_stepper_exit(void)
{
    misc_deregister(&stepper_miscdev);
    hrtimer_cancel(&step_timer);
    set_enable(false);
    free_gpio_pins();
    pr_info("dualstepper: module unloaded\n");
}

module_init(dual_stepper_init);
module_exit(dual_stepper_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Hugo Wang");
MODULE_DESCRIPTION("Synchronous dual-stepper motor driver for Raspberry Pi 3B");
