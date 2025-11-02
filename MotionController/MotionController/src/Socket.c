// Socket.c
#include "Socket.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ws2tcpip.h>  // 添加这个头文件以使用INET_ADDRSTRLEN

#pragma comment(lib, "ws2_32.lib")

#define RXDATA_SIZE sizeof(struct RxData)
#define FEEDBACK_SIZE sizeof(CommandFeedback)

// 全局变量声明
struct RxData g_rxData = {0};
int g_bDataReceived = 0;
static int g_sequenceNumber = 0;  // 指令序列号

// 添加保存上一次命令信息的变量
static struct RxData g_lastRxData = {0};
static int g_lastSequenceNumber = 0;

// 发送指令反馈函数实现
int SendCommandFeedback(SOCKET clientSocket, CommandFeedback* feedback) {
    int sendLen = send(clientSocket, (char*)feedback, FEEDBACK_SIZE, 0);
    if(sendLen == SOCKET_ERROR) {
        fprintf(stderr, "Send feedback failed, error code: %zd\n", (size_t)WSAGetLastError());
        return -1;
    }
    return 0;
}

int RunSocketServer(unsigned short usPort, void (*pDataCallback)(struct RxData* pData))
{
    WSADATA wsaData;
    SOCKET serverSocket, clientSocket;
    struct sockaddr_in serverAddr, clientAddr;
    int clientAddrSize;
    struct RxData rxData;
    int recvLen, sendLen;
    char clientIP[INET_ADDRSTRLEN];  // 现在INET_ADDRSTRLEN可以正常使用了
    char response[] = "Server received structure data";
    
    // 初始化Winsock
    if(WSAStartup(MAKEWORD(2,2), &wsaData) != 0)
    {
        fprintf(stderr, "WSAStartup failed, error code: %zd\n", (size_t)WSAGetLastError());
        return 1;
    }

    // 创建套接字
    serverSocket = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(serverSocket == INVALID_SOCKET)
    {
        fprintf(stderr, "Create socket failed, error code: %zd\n", (size_t)WSAGetLastError());
        WSACleanup();
        return 1;
    }

    // 设置服务器地址
    memset(&serverAddr, 0, sizeof(serverAddr));
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    serverAddr.sin_port = htons(usPort);

    // 绑定套接字
    if(bind(serverSocket, (struct sockaddr*)&serverAddr, sizeof(serverAddr)) == SOCKET_ERROR)
    {
        fprintf(stderr, "Bind failed, error code: %zd\n", (size_t)WSAGetLastError());
        closesocket(serverSocket);
        WSACleanup();
        return 1;
    }

    // 开始监听
    if(listen(serverSocket, 5) == SOCKET_ERROR)
    {
        fprintf(stderr, "Listen failed, error code: %zd\n", (size_t)WSAGetLastError());
        closesocket(serverSocket);
        WSACleanup();
        return 1;
    }

    printf("Server started successfully, listening on port %d ...\n", usPort);

    // 接受客户端连接
    clientAddrSize = sizeof(clientAddr);
    clientSocket = accept(serverSocket, (struct sockaddr*)&clientAddr, &clientAddrSize);
    if(clientSocket == INVALID_SOCKET)
    {
        fprintf(stderr, "Accept connection failed, error code: %zd\n", (size_t)WSAGetLastError());
        closesocket(serverSocket);
        WSACleanup();
        return 1;
    }

    inet_ntop(AF_INET, &clientAddr.sin_addr, clientIP, sizeof(clientIP));
    printf("Client %s:%d connected\n", clientIP, ntohs(clientAddr.sin_port));

    // 数据接收循环
    while(1)
    {
        recvLen = recv(clientSocket, (char*)&rxData, RXDATA_SIZE, 0);
        if(recvLen <= 0)
        {
            if(recvLen == 0)
            {
                printf("Client disconnected\n");
            }
            else
            {
                fprintf(stderr, "Receive data failed, error code: %zd\n", (size_t)WSAGetLastError());
            }    
            break;
        }

        if(recvLen < RXDATA_SIZE)
        {
            fprintf(stderr, "Received data incomplete, expected %d bytes, actual %d bytes\n", RXDATA_SIZE, recvLen);
            continue;
        }

        // 更新指令序列号
        g_sequenceNumber++;

        // 发送接收确认反馈（反馈上一次的命令）
        CommandFeedback feedback = {0};
        if (g_lastSequenceNumber > 0) {
            // 如果有上一次的命令，则反馈上一次的命令信息
            feedback.iCMD = g_lastRxData.iCMD;
            feedback.axis = g_lastRxData.axis;
            feedback.sequenceNumber = g_lastSequenceNumber;
            feedback.status = CMD_STATUS_COMPLETED;
            sprintf_s(feedback.message, sizeof(feedback.message), "Command %d completed successfully", g_lastRxData.iCMD);
        } else {
            // 如果没有上一次的命令（第一次），则反馈当前命令的接收确认
            feedback.iCMD = rxData.iCMD;
            feedback.axis = rxData.axis;
            feedback.sequenceNumber = g_sequenceNumber;
            feedback.status = CMD_STATUS_PENDING;
            sprintf_s(feedback.message, sizeof(feedback.message), "Command %d received", rxData.iCMD);
        }
        SendCommandFeedback(clientSocket, &feedback);

        // 保存当前命令为下一次的"上一次命令"
        g_lastRxData = rxData;
        g_lastSequenceNumber = g_sequenceNumber;

        // 将接收到的数据保存到全局变量
        g_rxData = rxData;
        g_bDataReceived = 1;

        // 调用回调函数处理接收到的数据
        if(pDataCallback != NULL)
        {
            pDataCallback(&rxData);
        }
        else
        {
            // 默认处理方式，打印接收到的数据
            printf("\nReceived structure data:\n");
            printf("iCMD: %d\n", rxData.iCMD);
            printf("axis: %d\n", rxData.axis);
            printf("iReserved[0]: %d\n", rxData.iReserved[0]);
            printf("iReserved[1]: %d\n", rxData.iReserved[1]);
            printf("dParamData[0]: %.5f\n", rxData.dParamData[0]);
            printf("dParamData[1]: %.5f\n", rxData.dParamData[1]);
            printf("dParamData[2]: %.5f\n", rxData.dParamData[2]);
            printf("dParamData[3]: %.5f\n", rxData.dParamData[3]);
            printf("dParamData[4]: %.5f\n", rxData.dParamData[4]);
        }

        // 发送执行完成反馈（反馈当前命令）
        feedback.iCMD = rxData.iCMD;
        feedback.axis = rxData.axis;
        feedback.sequenceNumber = g_sequenceNumber;
        feedback.status = CMD_STATUS_COMPLETED;
        sprintf_s(feedback.message, sizeof(feedback.message), "Command %d executed successfully", rxData.iCMD);
        SendCommandFeedback(clientSocket, &feedback);

        // 检查是否需要断开连接
        if(rxData.iCMD == 999)
        {
            printf("Client requested disconnect\n");
            break;
        }
    }

    // 清理资源
    closesocket(clientSocket);
    closesocket(serverSocket);
    WSACleanup();

    return 0;
}