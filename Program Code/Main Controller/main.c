/*
 * main.c
 *
 * Topic : FoodPanda Wall-Climbing Robot Main Controller
 * 
 * * * * 系統架構對應 (System Mapping):
 * - [Slide 17] Socket 程式: 負責 Ground Station 與 RPi 的通訊 (tcp_server_thread)
 * - [Slide 17] 主系統整合: Main Thread 負責調度相機、馬達與網路指令
 * - [Slide 9]  Main Thread (C): 系統核心，負責排程與通訊
 * - [Slide 10] Circular Queue: 實作 FIFO 訂單佇列 (解決多用戶排隊問題)
 * - [Slide 11] Thread Synchronization: 主執行緒與馬達執行緒的同步
 * - [Slide 14] Race Condition: 使用 Mutex 保護佇列，確保多用戶同時下單時資料不損毀
 * - [Slide 16] IPC (Named Pipe): 接收相機的 AprilTag 辨識結果
 * 
 * - Signal Handler: 模擬 ISR 處理緊急中斷 (Emergency Stop)
 * - Multi-user Queue Protection: 完整實作 Mutex 鎖定機制
 * 
 * * * Compile method:
 * Use Makefile
 *
 * (c) 2025  Group 8 (Reconstructed)
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <arpa/inet.h>
#include <signal.h> // 用於處理軟體中斷訊號

#include <linux/watchdog.h>

#include <sched.h> // [優化] 用於排程策略設定與 CPU 親和性

// ==========================================
// 1. 硬體與驅動設定 (與 Driver 對接)
// ==========================================
#define DEVICE_PATH "/dev/dualstepper"  ///< Kernel Driver 節點
#define PIPE_PATH "/tmp/apriltag_pipe"  ///< IPC: Named Pipe 路徑
#define PORT 8888                       ///< TCP Server 監聽埠號

// IOCTL Magic Number (必須與 FP_motor_driver_1.c 一致)
#define STEP_IOCTL_BASE    's'
#define STEP_CMD_START _IOW(STEP_IOCTL_BASE, 0, struct step_cmd)
#define STEP_CMD_STOP  _IO (STEP_IOCTL_BASE, 3) // [新增] 緊急停止指令

// [優化] 必須定義此宏才能使用 CPU Affinity 函數，請放在檔案最第一行
#define _GNU_SOURCE

/**
 * @brief 馬達控制指令結構
 * 對應 Driver 中的 struct step_cmd
 */
struct step_cmd {
    int dir;       ///< 0 = FWD (向上), 1 = BWD (向下)
    int cycles;    ///< 馬達轉動圈數 (決定移動距離)
    int frequency; ///< 步進頻率 (決定速度 Hz)
};

// ==========================================
// 2. 資料結構: 環狀佇列 (Circular Queue)
// 對應 [Slide 10]
// 用途：解決多用戶 (Multi-user) 同時請求時的順序問題
// ==========================================
#define QUEUE_SIZE 8

/**
 * @brief 訂單請求結構
 */
typedef struct {
    int type;           ///< 1: DELIVER (送餐), 2: PICKUP (取餐)
    int target_floor;   ///< 目標樓層
    char order_id[20];  ///< 訂單編號
    char locker_id;     ///< [新增] 置物櫃代號 (A, B, C, D) 對應 [Slide 7]
} Request;

/**
 * @brief 環狀佇列結構
 * [Slide 14] Race Condition 解決方案：
 * 使用 Mutex 保護 buffer，確保多個 Client 同時寫入時不會發生衝突。
 */
typedef struct {
    Request buffer[QUEUE_SIZE];
    int head;
    int tail;
    int count;
    pthread_mutex_t lock; ///< [Slide 14] 資源保護鎖 (Critical Section Protection)
} CircularQueue;

// 全域變數
CircularQueue req_queue;
int current_floor = 1;      ///< 系統當前樓層
int motor_fd;               ///< 馬達 Driver 檔案描述符
int g_client_socket = -1;   ///< 目前連線的 Web Server Socket (用於回傳狀態)
volatile int system_running = 1; ///< 系統運行旗標 (volatile 確保 ISR 能正確修改)

// 視覺伺服目標變數
// 當相機看到此 ID 時，會觸發緊急煞車 (Visual Servoing)
// -1 代表沒有鎖定特定目標
volatile int g_target_tag_id = -1;

// ==========================================
// 3. 多執行緒同步物件 (Synchronization)
// 對應 [Slide 11, 12]
// ==========================================
/**
 * @brief 馬達指令鎖 (Mutex)
 * 保護 motor_task 變數，避免 Race Condition
 */
pthread_mutex_t g_motor_cmd_mutex = PTHREAD_MUTEX_INITIALIZER;

/**
 * @brief 馬達任務通知點 (Condition Variable)
 * 主程式 -> 通知馬達執行緒 "有工作了"
 */
pthread_cond_t g_motor_cmd_cond = PTHREAD_COND_INITIALIZER;

/**
 * @brief 馬達完成通知點 (Condition Variable)
 * 馬達執行緒 -> 通知主程式 "工作做完了"
 */
pthread_cond_t g_motor_done_cond = PTHREAD_COND_INITIALIZER;

// 共用任務變數
struct {
    int has_task; ///< 旗標：1 = 有待處理任務
    int dir;      ///< 方向
    int cycles;   ///< 圈數
} motor_task;

// ==========================================
// 4. 輔助函式與 ISR (Interrupt Service Routine)
// ==========================================

/**
 * @brief [新增] 軟體中斷處理 (Signal Handler)
 * 模擬 ISR 行為。當發生緊急狀況 (如使用者按下 Ctrl+C) 時觸發。
 * 功能：安全停止馬達，避免機構損壞。
 */
void sigint_handler(int sig) {
    printf("\n[ISR] Emergency Stop Signal Received (SIGINT)!\n");
    system_running = 0;
    
    // 緊急停止馬達 IOCTL
    if (motor_fd > 0) {
        ioctl(motor_fd, STEP_CMD_STOP);
        printf("[ISR] Motor Emergency STOP command sent.\n");
    }

    // 喚醒所有等待中的執行緒，讓它們能安全退出
    pthread_cond_broadcast(&g_motor_cmd_cond);
    pthread_cond_broadcast(&g_motor_done_cond);
}

/**
 * @brief [新增] 產生系統狀態 JSON 字串 (Thread-Safe Reading)
 * 對應 [Slide 6] 即時系統狀態顯示 (電梯樓層、佇列數量、馬達狀態)
 * 對應 [Slide 7] 查詢狀態 (STATUS) 功能
 */
void get_system_status_json(char* buffer, size_t size) {
    // 鎖定並讀取佇列狀態
    pthread_mutex_lock(&req_queue.lock);
    int q_count = req_queue.count;
    pthread_mutex_unlock(&req_queue.lock);

    // 鎖定並讀取馬達狀態
    pthread_mutex_lock(&g_motor_cmd_mutex);
    int is_busy = motor_task.has_task;
    pthread_mutex_unlock(&g_motor_cmd_mutex);

    // 格式化為 JSON
    snprintf(buffer, size, 
        "{\"type\":\"STATUS_REP\", \"floor\":%d, \"queue\":%d, \"motor\":%s}\n", 
        current_floor, 
        q_count, 
        is_busy ? "\"RUNNING\"" : "\"IDLE\""
    );
}

/**
 * @brief 回傳狀態給 Web Server
 * 格式範例: {"type":"STATUS", "floor":3, "msg":"ARRIVED"}
 */
void send_status_update(const char* status_msg, int floor) {
    if (g_client_socket < 0) return;

    char json_buffer[128];
    snprintf(json_buffer, sizeof(json_buffer), 
             "{\"type\":\"STATUS\", \"floor\":%d, \"msg\":\"%s\"}\n", 
             floor, status_msg);

    // 透過 TCP Socket 傳送
    // 注意：若 socket 已斷開可能會導致 SIGPIPE，實際專案需處理 signal(SIGPIPE, SIG_IGN)
    send(g_client_socket, json_buffer, strlen(json_buffer), 0);
    printf("[TCP] Sent update: %s", json_buffer);
}

// --- Queue 操作 (Thread-Safe) ---
void init_queue(CircularQueue *q) {
    q->head = 0; q->tail = 0; q->count = 0;
    pthread_mutex_init(&q->lock, NULL);
}

/**
 * @brief 將請求加入佇列 (Thread-Safe Push)
 * [Slide 14] 實作 Mutex 保護，支援多用戶同時操作
 */
void push_queue(Request req) {
    // [Slide 14] 進入 Critical Section 前先鎖定 Mutex
    // 這防止了 User A 寫入一半時，User B 插隊導致資料錯亂
    pthread_mutex_lock(&req_queue.lock);
    
    if (req_queue.count < QUEUE_SIZE) {
        req_queue.buffer[req_queue.tail] = req;
        req_queue.tail = (req_queue.tail + 1) % QUEUE_SIZE;
        req_queue.count++;
        printf("[Queue] Added request: %s to Floor %d (Locker: %c)\n", 
               req.order_id, req.target_floor, req.locker_id); // [Update] 顯示 Locker
    } else {
        printf("[Queue] Warning: Queue Full! Dropping request.\n");
    }
    
    // 離開 Critical Section 解鎖
    pthread_mutex_unlock(&req_queue.lock);
}

int pop_queue(Request *req) {
    int ret = 0;
    pthread_mutex_lock(&req_queue.lock); // 鎖定讀取
    
    if (req_queue.count > 0) {
        *req = req_queue.buffer[req_queue.head];
        req_queue.head = (req_queue.head + 1) % QUEUE_SIZE;
        req_queue.count--;
        ret = 1;
    }
    
    pthread_mutex_unlock(&req_queue.lock); // 解鎖
    return ret;
}

// ==========================================
// 5. 馬達控制執行緒 (Motor Thread)
// 對應 [Slide 12] 流程圖
// ==========================================
void *motor_thread_func(void *arg) {
    // ============================================================
    // [優化] CPU Affinity: 將此執行緒綁定到 CPU Core 2
    // 目的：減少 Context Switch 與 Cache Miss，提升馬達控制穩定度
    // ============================================================
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(2, &cpuset); // 指定 CPU 2

    pthread_t current_thread = pthread_self();
    if (pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset) != 0) {
        perror("[Motor] Warning: Failed to set CPU affinity");
    } else {
        printf("[Motor] Pinned to CPU Core 2 for Real-time Performance\n");
    }
    // ============================================================

    printf("[Motor Thread] Started. Waiting for tasks...\n");

    while (system_running) {
        // --- Step 1: 等待任務 (Wait) ---
        // 使用 Mutex + Condition Variable 避免 Busy Waiting
        pthread_mutex_lock(&g_motor_cmd_mutex);
        
        while (!motor_task.has_task && system_running) {
            pthread_cond_wait(&g_motor_cmd_cond, &g_motor_cmd_mutex);
        }
        
        if (!system_running) {
            pthread_mutex_unlock(&g_motor_cmd_mutex);
            break;
        }

        struct step_cmd cmd;
        cmd.dir = motor_task.dir;
        cmd.cycles = motor_task.cycles;
        // [新增機能 2] 定義速度參數
        int freq_low = 200;  // 起步/煞車頻率 (Hz)
        int freq_high = 800; // 巡航頻率 (Hz)
        int ramp_cycles = 10; // 加減速區間的圈數 (例如 10 圈)
        
        pthread_mutex_unlock(&g_motor_cmd_mutex);

        // --- Step 2: 執行物理移動 (IOCTL) ---
        // 這是與 Kernel Driver 溝通的介面
        
        // [新增機能 2: 馬達加減速控制 (Soft Start/Stop)]
        // 如果總圈數夠長，則執行：加速 -> 等速 -> 減速
        if (cmd.cycles > (ramp_cycles * 2)) {
            struct step_cmd ramp_cmd;
            ramp_cmd.dir = cmd.dir;
            
            // 1. 加速段 (Accel)
            printf("[Motor] Ramping UP...\n");
            ramp_cmd.cycles = ramp_cycles;
            ramp_cmd.frequency = freq_low;
            ioctl(motor_fd, STEP_CMD_START, &ramp_cmd);
            usleep(ramp_cycles * (1000000 / freq_low) * 1.2); // 等待加速完成 (乘 1.2 是保險係數)

            // 2. 巡航段 (Cruise)
            // 注意：Driver 接受新指令會更新當前任務，實現無縫切換
            printf("[Motor] Cruising...\n");
            ramp_cmd.cycles = cmd.cycles - (ramp_cycles * 2);
            ramp_cmd.frequency = freq_high;
            ioctl(motor_fd, STEP_CMD_START, &ramp_cmd);
            
            // 模擬等待巡航完成，同時允許中斷
            int cruise_time_us = ramp_cmd.cycles * (1000000 / freq_high);
            for (int i = 0; i < 100; i++) {
                if (!system_running) break;
                // [新增機能 3] 若視覺伺服觸發了停止 (g_target_tag_id 被清空)，提早跳出
                if (g_target_tag_id == -1 && motor_task.has_task) break; 
                usleep(cruise_time_us / 100);
            }

            // 3. 減速段 (Decel)
            if (system_running && g_target_tag_id != -1) { // 只有沒被中斷時才執行減速
                printf("[Motor] Ramping DOWN...\n");
                ramp_cmd.cycles = ramp_cycles;
                ramp_cmd.frequency = freq_low;
                ioctl(motor_fd, STEP_CMD_START, &ramp_cmd);
                usleep(ramp_cycles * (1000000 / freq_low) * 1.2);
            }
        } else {
            // 短距離移動，直接執行 (無加減速)
            cmd.frequency = freq_low; // 短距離用低速比較安全
            printf("[Motor] Short Move (No Ramp): Dir=%d, Cycles=%d\n", cmd.dir, cmd.cycles);
            if (ioctl(motor_fd, STEP_CMD_START, &cmd) < 0) {
                perror("[Motor] IOCTL Failed");
            }
            usleep(cmd.cycles * (1000000 / freq_low));
        }

        // --- Step 3: 通知完成 (Signal) ---
        pthread_mutex_lock(&g_motor_cmd_mutex);
        motor_task.has_task = 0; 
        pthread_cond_broadcast(&g_motor_done_cond); // 通知主執行緒
        pthread_mutex_unlock(&g_motor_cmd_mutex);
        
        printf("[Motor] Task Finished. Signal sent.\n");
    }
    return NULL;
}

// ==========================================
// 6. TCP Server 執行緒 (Web Interface / Multi-user)
// 對應 [Slide 9] B: 伺服器 -> C: 主程式
// 功能: 支援多用戶連線，將指令放入 Queue
// ==========================================
void *tcp_server_thread(void *arg) {
    int server_fd, new_socket;
    struct sockaddr_in address;
    int addrlen = sizeof(address);
    char buffer[1024] = {0};

    // 建立 Socket
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        perror("[TCP] Socket failed"); exit(EXIT_FAILURE);
    }
    
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        perror("[TCP] Bind failed"); exit(EXIT_FAILURE);
    }
    
    if (listen(server_fd, 5) < 0) { // Backlog = 5，允許5個等待連線
        perror("[TCP] Listen failed"); exit(EXIT_FAILURE);
    }
    
    printf("[TCP] Server listening on port %d (Multi-user Ready)...\n", PORT);

    while (system_running) {
        // [Multi-user Note] 
        // 這裡使用 Blocking Accept。若要同時服務多個長連線，需使用 select/epoll 或 fork/thread。
        // 但為了符合 Slide 9 架構，我們採用「接收 -> 處理 -> 繼續監聽」的模式。
        // 因為 push_queue 很快，所以能快速處理下一個用戶。
        
        if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
            if (system_running) perror("[TCP] Accept failed");
            continue;
        }
        
        printf("[TCP] New Connection Accepted.\n");
        g_client_socket = new_socket; // 暫存最新的 socket 用於回傳 (簡化版)

        // 讀取一次指令 (假設 Web 端發送完 JSON 就結束一次請求)
        int valread = read(new_socket, buffer, 1024);
        if (valread > 0) {
            buffer[valread] = '\0';
            printf("[TCP] Received Command: %s\n", buffer);
            
            // [新增] 處理 STATUS 指令 (查詢狀態)
            // 對應 [Slide 7] 查詢狀態 (STATUS)
            if (strstr(buffer, "STATUS")) {
                char status_json[256];
                get_system_status_json(status_json, sizeof(status_json));
                send(new_socket, status_json, strlen(status_json), 0);
                printf("[TCP] Status Reported: %s", status_json);
            }
            // 解析指令
            else if (strstr(buffer, "DELIVER") || strstr(buffer, "PICKUP")) {
                Request req;
                req.type = (strstr(buffer, "DELIVER")) ? 1 : 2;
                req.target_floor = 0;
                req.locker_id = 'N'; // N = None/Unknown
                
                char *floor_ptr = strstr(buffer, "FLOOR");
                if (floor_ptr) {
                    req.target_floor = atoi(floor_ptr + 7);
                }

                // [新增] 解析 LOCKER (置物櫃代號)
                // 對應 [Slide 7] 放置物品 (PLACE) 中的 A, B, C, D 櫃位
                char *locker_ptr = strstr(buffer, "LOCKER");
                if (locker_ptr) {
                    // 假設 JSON 格式為 "LOCKER":"A"
                    // locker_ptr 會指向 "L", 跳過 9 個字元 ("LOCKER":"") 取得 A
                    req.locker_id = *(locker_ptr + 9); 
                }
                
                if (req.target_floor > 0) {
                    snprintf(req.order_id, 20, "CMD-%d", rand()%100);
                    
                    // [Key Point] 這裡的 push_queue 是 Thread-Safe 的
                    // 即使多個用戶極短時間內連線，Mutex 也會確保 Queue 正確
                    push_queue(req); 
                    
                    char *ack = "{\"status\":\"QUEUED\"}\n";
                    send(new_socket, ack, strlen(ack), 0);
                }
            }
        }
        // 這裡不關閉 socket，允許 web_backend_mock 保持連線接收 "ARRIVED"
        // 實際 Web Server 實作通常是短連線 (HTTP) 或 WebSocket
    }
    return NULL;
}

// ==========================================
// 7. 相機監控執行緒 (IPC)
// 對應 [Slide 16] Named Pipe -> Select
// ==========================================
void *camera_monitor_thread(void *arg) {
    if (access(PIPE_PATH, F_OK) == -1) {
        mkfifo(PIPE_PATH, 0666);
    }
    
    // 使用非阻塞模式開啟 Pipe，避免主程式被卡住
    int fd = open(PIPE_PATH, O_RDONLY | O_NONBLOCK);
    char buf[128];
    
    printf("[Camera IPC] Listening on Pipe: %s\n", PIPE_PATH);

    while (system_running) {
        int n = read(fd, buf, sizeof(buf)-1);
        if (n > 0) {
            buf[n] = '\0';
            printf("[Camera IPC] Detected Tag ID: %s\n", buf);
            
            // [新增機能 3: 視覺閉迴路控制 (Visual Servoing)]
            // 如果偵測到的 Tag ID 等於我們目標樓層的 ID，立即停止馬達
            int detected_id = atoi(buf);
            if (g_target_tag_id != -1 && detected_id == g_target_tag_id) {
                printf("[Visual Servoing] Target Tag %d detected! STOPPING MOTOR.\n", detected_id);
                
                // 發送緊急停止指令給 Driver
                if (motor_fd > 0) {
                    ioctl(motor_fd, STEP_CMD_STOP);
                }
                
                // 清除目標，避免重複觸發
                g_target_tag_id = -1;
            }
        }
        
        // 若 Pipe 被寫入端 (camera_app) 關閉，需要重新開啟
        if (n == 0) {
            close(fd);
            usleep(100000);
            fd = open(PIPE_PATH, O_RDONLY | O_NONBLOCK);
        }
        
        usleep(100000); // 避免 Busy Loop
    }
    if (fd > 0) close(fd);
    return NULL;
}

// ==========================================
// 8. 主程式 (Scheduler)
// 對應 [Slide 11] 流程圖
// ==========================================
int main() {
    printf("=== FP Wall-Climbing Robot Main Controller Started ===\n");

    // 註冊 Signal Handler (模擬 ISR)
    signal(SIGINT, sigint_handler);
    signal(SIGPIPE, SIG_IGN); 

    // 看門狗 Watchdog
    // 啟用系統 Watchdog，若程式卡死超過 10 秒，系統將自動重啟
    int wd_fd = open("/dev/watchdog", O_RDWR);
    if (wd_fd >= 0) {
        int timeout = 10;
        ioctl(wd_fd, WDIOC_SETTIMEOUT, &timeout);
        printf("[System] Watchdog Enabled (Timeout: 10s)\n");
    } else {
        printf("[System] Warning: Watchdog device not found.\n");
    }

    // 1. 初始化
    init_queue(&req_queue);
    
    motor_fd = open(DEVICE_PATH, O_RDWR);
    if (motor_fd < 0) {
        perror("[Main] Failed to open /dev/dualstepper");
    }

    // 2. 啟動背景執行緒
    pthread_t t_motor, t_tcp, t_cam;

    // ============================================================
    // [優化 1] Real-time Scheduler (SCHED_FIFO) 設定
    // 目的：確保馬達控制擁有最高優先權，避免被相機運算卡住
    // ============================================================
    pthread_attr_t attr;
    struct sched_param param;

    // A. 初始化執行緒屬性
    pthread_attr_init(&attr);

    // B. 設定不繼承父執行緒屬性 (Explicit)
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    // C. 設定排程策略為 SCHED_FIFO (先進先出，即時等級)
    // 注意：這需要 sudo 權限執行才能生效
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

    // D. 設定優先級 (範圍 1-99)
    // 設為 50，高於一般 Linux 程式 (通常是 0)，但保留空間給 Kernel 重要的中斷
    param.sched_priority = 50;
    pthread_attr_setschedparam(&attr, &param);

    // E. 使用設定好的屬性建立馬達執行緒
    if (pthread_create(&t_motor, &attr, motor_thread_func, NULL) != 0) {
        perror("[Main] Failed to create RT Motor Thread (Try running with sudo?)");
        // 若失敗 (權限不足)，回退使用普通模式建立，確保程式至少能跑
        printf("[Main] Fallback to normal thread priority...\n");
        pthread_create(&t_motor, NULL, motor_thread_func, NULL);
    } else {
        printf("[Main] Real-time Motor Thread Created (SCHED_FIFO Priority=50)\n");
    }
    
    // 清理屬性物件
    pthread_attr_destroy(&attr);
    // ============================================================

    // 建立其他普通執行緒 (優先級較低，使用預設屬性)
    pthread_create(&t_tcp, NULL, tcp_server_thread, NULL);
    pthread_create(&t_cam, NULL, camera_monitor_thread, NULL);

    Request current_req;
    
    // 3. 主迴圈 (Event Loop)
    while (system_running) {
        // 餵狗 (Kick the dog)，告訴系統我還活著
        if (wd_fd >= 0) {
            ioctl(wd_fd, WDIOC_KEEPALIVE, 0);
        }

        // [Slide 11] 檢查佇列是否有新請求 (Atomic Pop)
        if (pop_queue(&current_req)) {
            printf("\n[Main] Processing Order: %s -> Target Floor: %d (Locker: %c)\n", 
                   current_req.order_id, current_req.target_floor, current_req.locker_id);
            
            send_status_update("PROCESSING", current_floor);

            if (current_req.target_floor != current_floor) {
                // 計算移動參數
                int diff = current_req.target_floor - current_floor;
                int dir = (diff > 0) ? 0 : 1; 
                int steps = abs(diff) * 10; 
                
                printf("[Main] Moving Elevator...\n");

                // 設定視覺伺服目標
                // 告訴相機執行緒：如果看到這個 ID 的 Tag，就緊急煞車
                g_target_tag_id = current_req.target_floor;

                // --- A. 設定任務並喚醒馬達 (Signal) ---
                pthread_mutex_lock(&g_motor_cmd_mutex);
                motor_task.dir = dir;
                motor_task.cycles = steps;
                motor_task.has_task = 1; 
                
                pthread_cond_signal(&g_motor_cmd_cond); 
                pthread_mutex_unlock(&g_motor_cmd_mutex);

                // --- B. 等待馬達完成 (Wait) ---
                pthread_mutex_lock(&g_motor_cmd_mutex);
                while (motor_task.has_task && system_running) {
                    pthread_cond_wait(&g_motor_done_cond, &g_motor_cmd_mutex);
                }
                pthread_mutex_unlock(&g_motor_cmd_mutex);
                
                // 清除目標 ID (防止誤動作)
                g_target_tag_id = -1;

                if (system_running) {
                    current_floor = current_req.target_floor;
                    printf("[Main] Arrived at Floor %d\n", current_floor);
                    send_status_update("ARRIVED", current_floor);
                }
            } else {
                printf("[Main] Already at target floor.\n");
                send_status_update("ARRIVED", current_floor);
            }
        }
        
        usleep(100000); // Idle 0.1s
    }
    
    printf("[Main] System Shutting Down...\n");
    if (motor_fd > 0) close(motor_fd);
    if (wd_fd >= 0) close(wd_fd); // 關閉 Watchdog
    pthread_cancel(t_tcp); // 強制結束 Server 執行緒
    return 0;
}