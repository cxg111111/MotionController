#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <process.h>
#include "ThreadControl.h"
#include "Socket.h"

int main()
{
    HANDLE hControlThread, hSocketThread,hCSVWriterThread;
    unsigned short port = 8081;
    
    printf("Starting multi-threaded application\n");
    printf("===================================\n");

    //初始化CSV缓冲区
    InitCSVBuffer();
    
    // 创建Socket线程
    hSocketThread = (HANDLE)_beginthreadex(
        NULL,           // 安全属性
        0,              // 栈大小
        (unsigned int (__stdcall *)(void *))SocketThreadFunction, // 线程函数
        &port,          // 参数
        0,              // 创建标志
        NULL            // 线程ID
    );
    
    if(hSocketThread == NULL)
    {
        fprintf(stderr, "Failed to create socket thread\n");
        return 1;
    }
    
    // 给Socket初始化一些等待时间
    Sleep(1000);
    
    // 创建CSV写入线程
    hCSVWriterThread = (HANDLE)_beginthreadex(
        NULL,           
        0,              
        (unsigned int (__stdcall *)(void *))CSVWriterThreadFunction, 
        NULL,           
        0,              
        NULL            
    );
    
    if(hCSVWriterThread == NULL)
    {
        fprintf(stderr, "Failed to create CSV writer thread\n");
        // 关闭已创建的Socket线程
        if(hSocketThread != NULL)
        {
            WaitForSingleObject(hSocketThread, INFINITE);
            CloseHandle(hSocketThread);
        }
        return 1;
    }

    // 创建控制线程
    hControlThread = (HANDLE)_beginthreadex(
        NULL,           // 安全属性
        0,              // 栈大小
        (unsigned int (__stdcall *)(void *))ControlThreadFunction, // 线程函数
        NULL,           // 参数
        0,              // 创建标志
        NULL            // 线程ID
    );
    
    if(hControlThread == NULL)
    {
        fprintf(stderr, "Failed to create control thread\n");
        // 关闭已创建的线程
        if(hSocketThread != NULL)
        {
            WaitForSingleObject(hSocketThread, INFINITE);
            CloseHandle(hSocketThread);
        }
        if(hCSVWriterThread != NULL)
        {
            WaitForSingleObject(hCSVWriterThread, INFINITE);
            CloseHandle(hCSVWriterThread);
        }
        return 1;
    }
    
    printf("All threads started successfully\n");
    printf("Press Enter to stop application...\n");
    
    // 等待用户输入
    getchar();
    
    // 等待线程结束
    if(hControlThread != NULL)
    {
        WaitForSingleObject(hControlThread, INFINITE);
        CloseHandle(hControlThread);
    }
    
    if(hSocketThread != NULL)
    {
        WaitForSingleObject(hSocketThread, INFINITE);
        CloseHandle(hSocketThread);
    }
    
    // 等待CSV写入线程结束
    if(hCSVWriterThread != NULL)
    {
        WaitForSingleObject(hCSVWriterThread, INFINITE);
        CloseHandle(hCSVWriterThread);
    }
    
    // 清理CSV缓冲区
    CleanupCSVBuffer();
    
    printf("Application completed\n");
    return 0;
}