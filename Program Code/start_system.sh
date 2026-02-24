#!/bin/bash
# start_system.sh - FoodPanda Robot Launch Script
# 用途: 自動載入 Driver 並啟動 Main Controller 與 Camera
# 使用方法: sudo ./start_system.sh

echo "=== System Initializing ==="

# 1. 載入 Kernel Driver
# 檢查是否已載入，若無則 insmod
if lsmod | grep -q "FP_motor_driver_1"; then
    echo "[Driver] Already loaded."
else
    echo "[Driver] Loading module..."
    insmod motor_driver/FP_motor_driver_1.ko
    if [ $? -ne 0 ]; then
        echo "[Error] Failed to load driver!"
        exit 1
    fi
    # 賦予裝置權限，讓一般使用者也能寫入 (雖然此腳本用 sudo 跑)
    chmod 666 /dev/dualstepper
fi

# 2. 建立 Named Pipe (如果不存在)
PIPE_PATH="/tmp/apriltag_pipe"
if [ ! -p "$PIPE_PATH" ]; then
    mkfifo "$PIPE_PATH"
    chmod 666 "$PIPE_PATH"
    echo "[IPC] Named pipe created."
fi

# 3. 啟動 Main Controller (背景執行)
echo "[Main] Starting Controller..."
./main_ctrl &
MAIN_PID=$!

# 等待一下確保 Main 已經建立好 IPC Listener
sleep 2

# 4. 啟動 Camera App (背景執行)
echo "[Camera] Starting Camera App..."
# 假設你的 Camera 編譯出來叫 camera_app
./camera/build/camera_app &
CAM_PID=$!

echo "=== System Running ==="
echo "Main PID: $MAIN_PID"
echo "Camera PID: $CAM_PID"
echo "Press Ctrl+C to stop all services."

# 5. 等待中斷訊號，並進行清理
trap "echo 'Stopping...'; kill $MAIN_PID $CAM_PID; rmmod FP_motor_driver_1; exit" SIGINT SIGTERM

# 保持腳本執行