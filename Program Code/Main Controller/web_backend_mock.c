/**
 * @file web_backend_mock.c
 * @brief 模擬 PC 端的 Web Server
 * 用途: 測試 RPi 的 TCP Server 功能與狀態回傳機制
 * 編譯: gcc web_backend_mock.c -o web_mock
 * 執行: ./web_mock <RPi_IP>
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>

#define PORT 8888

int main(int argc, char const *argv[]) {
    int sock = 0;
    struct sockaddr_in serv_addr;
    char *server_ip = "127.0.0.1"; // 預設 localhost，若在 PC 跑請改為 RPi IP

    if (argc > 1) {
        server_ip = (char*)argv[1];
    }

    // 1. 建立 Socket
    if ((sock = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
        printf("\n Socket creation error \n");
        return -1;
    }

    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);

    if (inet_pton(AF_INET, server_ip, &serv_addr.sin_addr) <= 0) {
        printf("\nInvalid address/ Address not supported \n");
        return -1;
    }

    // 2. 連線到 RPi
    printf("Connecting to RPi at %s:%d ...\n", server_ip, PORT);
    if (connect(sock, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0) {
        printf("\nConnection Failed \n");
        return -1;
    }
    printf("Connected!\n");

    // 3. 發送訂單 (DELIVER 到 3 樓)
    // 格式必須符合 main.c 的簡單解析邏輯
    char *cmd = "{\"CMD\":\"DELIVER\", \"FLOOR\":3, \"ID\":\"TEST-001\"}";
    send(sock, cmd, strlen(cmd), 0);
    printf("Sent Command: %s\n", cmd);

    // 4. 等待狀態回傳 (Blocking Read)
    char buffer[1024] = {0};
    while (1) {
        int valread = read(sock, buffer, 1024);
        if (valread > 0) {
            buffer[valread] = '\0';
            printf("Received Update from RPi: %s", buffer);
            
            // 如果收到 ARRIVED，測試結束
            if (strstr(buffer, "ARRIVED")) {
                printf("Mission Complete!\n");
                break;
            }
        }
    }

    close(sock);
    return 0;
}