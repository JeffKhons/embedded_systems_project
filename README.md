# 🧗 Wall-Climbing Delivery Robot Control System

![Language](https://img.shields.io/badge/Language-C%2FC%2B%2B-blue.svg)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%203B-red.svg)
![Kernel](https://img.shields.io/badge/Kernel-Linux%20Driver-yellow.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)

## 📖 Project Overview

This project implements the **embedded control system** for a Wall-Climbing Delivery Robot. It runs on a **Raspberry Pi 3B** and features a hybrid architecture combining a **Linux Kernel Driver** for precision motor control and a **Multi-threaded User Application** for real-time task scheduling.

The system is designed to handle:
* **Real-time Motor Control:** Using custom kernel drivers and CPU affinity.
* **Network Communication:** TCP for reliable commands and UDP for video streaming.
* **Computer Vision:** AprilTag detection for indoor localization.
* **Concurrency:** Robust producer-consumer model for handling multiple user requests.

---

## 🏗️ System Architecture

The software adopts a **Hybrid Architecture (Process + Thread)** to balance system stability and real-time performance.

* **Process Isolation:** The Camera module runs as a separate process to prevent vision algorithms (high load) from crashing the main controller.
* **Thread Synchronization:** The Main Controller uses Mutexes and Condition Variables to coordinate Network and Motor threads efficiently.

```mermaid
graph TD
    subgraph RPi_System ["🍓 Raspberry Pi 3B Controller"]
        direction TB
        
        %% Process A: Vision
        subgraph Proc_Cam ["Process: Camera App (C++)"]
            direction TB
            CamSensor[Camera Capture] --> AprilTag[AprilTag Detection]
            AprilTag -->|Write Tag ID| Pipe_In(Named Pipe: /tmp/apriltag_pipe)
        end

        %% Process B: Main Controller
        subgraph Proc_Main ["Process: Main Controller (C)"]
            direction TB
            
            subgraph Thread_Net ["Thread: TCP Server"]
                Socket[Socket Listen] -->|Push Request| Queue[Circular Queue]
            end

            subgraph Thread_Main ["Thread: Main Scheduler"]
                Queue -->|Pop Request| Logic[Task Logic]
                Logic -->|Signal| MotorCond(Condition Variable)
                Pipe_Out(Named Pipe) -->|Read Tag ID| Logic
            end

            subgraph Thread_Motor ["Thread: Motor Control (RT)"]
                MotorCond -->|Wake Up| IOCTL[ioctl Call]
            end
        end

        %% Kernel Space
        subgraph Kernel ["Linux Kernel Space"]
            CharDev[Character Device: /dev/dualstepper]
            GPIO[GPIO Pins]
            HRTimer[High-Resolution Timer]
        end
        
        %% Inter-Process Communication
        Pipe_In -.-> Pipe_Out
        
        %% Kernel Interaction
        IOCTL ==> CharDev
        CharDev -.->|Pulse Generation| HRTimer
        HRTimer ==> GPIO
    end

🚀 Key Technical Highlights
1. Robust Concurrency & Thread Safety
Producer-Consumer Model: Implemented to bridge the asynchronous Network Thread and the synchronous Motor Thread.

Mutex Locks (pthread_mutex): Used to protect the shared Circular Queue, preventing race conditions when multiple users submit orders simultaneously.

Condition Variables: Utilized to put the Motor Thread to sleep when idle, significantly reducing CPU usage compared to polling.

2. Real-time Optimization (RTOS Concepts)
SCHED_FIFO Policy: The Motor Thread is configured with the SCHED_FIFO real-time scheduling policy to preempt non-critical background tasks.

CPU Affinity: Explicitly pinned critical threads to CPU Core 2, optimizing cache locality and minimizing context-switching overhead (Jitter reduction).

3. Custom Linux Kernel Driver
Character Device Driver: Developed /dev/dualstepper to bypass the slow userspace GPIO (sysfs).

High-Resolution Timers (hrtimer): Utilized inside the kernel to generate precise microsecond-level stepper pulses, ensuring smooth acceleration and deceleration profiles.

📂 Project Structure
Please ensure your files are organized as follows to match the Makefile:

Bash
.
├── Makefile                   # Build automation
├── CMakeLists.txt             # Build config for Camera App
├── main.c                     # Main Scheduler, TCP Server, Motor Logic
├── web_backend_mock.c         # Testing tool for TCP commands
├── carama.cpp                 # OpenCV AprilTag Detection
├── FP_motor_driver_1.c        # Linux Kernel Module Source
└── FP_motor_writer_1.c        # CLI tool for driver testing
🛠️ Build & Installation
Prerequisites
Raspberry Pi 3B/4B (Raspberry Pi OS)

Linux Kernel Headers

OpenCV 4.x (C++ libs)

GCC / G++

1. Compile All Components
Use the provided Makefile to compile the Kernel Module, Main Controller, and Camera App:

Bash
make all
Output files: main_ctrl, camera_app, web_mock, FP_motor_writer_1, FP_motor_driver_1.ko

2. Load Kernel Module
Insert the custom driver into the Linux Kernel:

Bash
sudo insmod FP_motor_driver_1.ko
sudo chmod 666 /dev/dualstepper
3. Run the System
Step 1: Start the Main Controller (Requires root for SCHED_FIFO)

Bash
sudo ./main_ctrl
Step 2: Start the Camera Process (In a separate terminal)

Bash
./camera_app
🧪 Testing
Simulated Ground Station
You can use the included mock server to send test commands to the robot without a frontend UI.

Bash
# Send a delivery command to Floor 3
./web_mock 127.0.0.1
Expected Output: JSON command {"CMD":"DELIVER", "FLOOR":3} is sent, and the robot receives it via TCP.

Motor Driver CLI
Test the motor hardware independently using the CLI writer:

Bash
./FP_motor_writer_1
# Type the command below to start motor:
# cmd> start FWD 200 800
👤 Author
Jeff Chen

Graduate Student, Institute of Communications Engineering, NYCU

Embedded System Design, 2025
