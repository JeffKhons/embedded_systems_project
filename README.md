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

.
├── Makefile                   # Build automation
├── CMakeLists.txt             # Build config for Camera App
├── main_controller/
│   ├── main.c                 # Main Scheduler, TCP Server, Motor Logic
│   └── web_backend_mock.c     # Testing tool for TCP commands
├── camera/
│   └── carama.cpp             # OpenCV AprilTag Detection
├── kernel_driver/
│   ├── FP_motor_driver_1.c    # Linux Kernel Module Source
│   └── FP_motor_writer_1.c    # CLI tool for driver testing
