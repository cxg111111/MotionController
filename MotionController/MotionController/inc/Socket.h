// Socket.h
#ifndef SOCKET_H
#define SOCKET_H

// 条件包含Winsock头文件
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include <winsock2.h>
#include <ws2tcpip.h>

// 添加指令执行状态枚举
typedef enum {
    CMD_STATUS_PENDING = 0,     // 等待执行
    CMD_STATUS_EXECUTING = 1,   // 正在执行
    CMD_STATUS_COMPLETED = 2,   // 执行完成
    CMD_STATUS_ERROR = 3        // 执行出错
} CommandStatus;

// 添加指令反馈结构体
typedef struct {
    int iCMD;                   // 指令ID
    int axis;                   // 轴ID
    int sequenceNumber;         // 指令序列号
    CommandStatus status;       // 指令状态
    int errorCode;              // 错误代码
    char message[128];          // 反馈消息
} CommandFeedback;

struct RxData
{
    int iCMD;
    int axis;
    int iReserved[2];
    double dParamData[5];
};

// 全局变量声明
extern struct RxData g_rxData;
extern int g_bDataReceived;

// 添加发送反馈函数声明
int SendCommandFeedback(SOCKET clientSocket, CommandFeedback* feedback);

// 函数声明
int RunSocketServer(unsigned short usPort, void (*pDataCallback)(struct RxData* pData));


#endif