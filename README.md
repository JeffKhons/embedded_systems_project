# 🧗 Wall-Climbing Delivery Robot Control System

![Language](https://img.shields.io/badge/Language-C%2FC%2B%2B-blue.svg)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%203B-red.svg)
![Kernel](https://img.shields.io/badge/Kernel-Linux%20Driver-yellow.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)

---

## 📖 Project Overview

This project implements the embedded control system for a Wall-Climbing Delivery Robot running on a Raspberry Pi 3B.

It combines:

- A Linux Kernel Driver for precision motor control  
- A Multi-threaded User Application for real-time scheduling  
- AprilTag-based indoor localization  
- TCP-based command system  

The system demonstrates RTOS concepts, kernel-level timing, and concurrent network-driven motor control.

---

## 🏗️ System Architecture

The system uses a Hybrid Architecture (Process + Thread):

- The Camera runs as a separate process (fault isolation)
- The Main Controller uses multiple threads
- The Motor thread runs with SCHED_FIFO real-time policy
- A custom kernel module generates precise GPIO pulses

---

### Architecture Diagram

```mermaid
graph TD

    subgraph RPi_System [Raspberry Pi 3B Controller]

        subgraph Proc_Cam [Process: Camera App (C++)]
            CamSensor --> AprilTag
            AprilTag --> Pipe_In
        end

        subgraph Proc_Main [Process: Main Controller (C)]

            subgraph Thread_Net [TCP Server Thread]
                Socket --> Queue
            end

            subgraph Thread_Main [Main Scheduler Thread]
                Queue --> Logic
                Logic --> MotorCond
                Pipe_Out --> Logic
            end

            subgraph Thread_Motor [Motor Control Thread]
                MotorCond --> IOCTL
            end

        end

        subgraph Kernel [Linux Kernel Space]
            CharDev
            GPIO
            HRTimer
        end

        IOCTL --> CharDev
        CharDev --> HRTimer
        HRTimer --> GPIO

    end

🚀 Technical Highlights
Concurrency

Producer–Consumer circular queue

pthread_mutex for race protection

pthread_cond to eliminate polling

Real-Time Optimization

SCHED_FIFO scheduling

CPU affinity binding

Reduced context-switch jitter

Custom Linux Driver

Character device /dev/dualstepper

High-resolution timer (hrtimer)

Microsecond-level step pulse control

.
├── Makefile
├── CMakeLists.txt
├── main.c
├── web_backend_mock.c
├── camera.cpp
├── FP_motor_driver_1.c
└── FP_motor_writer_1.c
