# 🧗 Wall-Climbing Delivery Robot Control System

![Language](https://img.shields.io/badge/Language-C%2FC%2B%2B-blue.svg)
![Platform](https://img.shields.io/badge/Platform-Raspberry%20Pi%203B-red.svg)
![Kernel](https://img.shields.io/badge/Kernel-Linux%20Driver-yellow.svg)
![License](https://img.shields.io/badge/License-MIT-green.svg)

---

## 📖 Project Overview

This project implements the **embedded control system** for a Wall-Climbing Delivery Robot running on a **Raspberry Pi 3B**.

It features a hybrid architecture combining:

- A **Linux Kernel Driver** for precision motor control  
- A **Multi-threaded User Application** for real-time task scheduling  

### System Capabilities

- Real-time Motor Control (Kernel + CPU affinity)
- TCP command handling
- UDP video streaming
- AprilTag indoor localization
- Producer-Consumer concurrency model

---

## 🏗️ System Architecture

The software adopts a **Hybrid Architecture (Process + Thread)** design to balance stability and real-time performance.

- **Process Isolation**  
  The Camera module runs as a separate process to prevent crashes from affecting the main controller.

- **Thread Synchronization**  
  The Main Controller coordinates Network and Motor threads using mutexes and condition variables.

---

### Architecture Diagram (Mermaid)

> ⚠ GitHub must support Mermaid (it does by default now)

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
