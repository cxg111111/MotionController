#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <stdio.h>
#include <stdlib.h>
#include <windows.h>
#include <process.h>
#include "ThreadControl.h"
#include "Socket.h"
#include "log.h"  // 添加日志头文件

int main()
{
    // 初始化日志系统
    log_init();
    log_set_level(LOG_TRACE);

    // 添加文件日志输出
    FILE* log_file = fopen("motion_controller.log", "a");
    if (log_file) {
        log_add_fp(log_file, LOG_DEBUG);
    }

    log_info("Starting multi-threaded application");
    log_info("==================================="); 

    HANDLE hControlThread = NULL, hSocketThread = NULL, hCSVWriterThread = NULL;
    unsigned short port = 8081;
    
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
        log_error("Failed to create socket thread");
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
        log_error("Failed to create CSV writer thread");
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
        log_error("Failed to create control thread");
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
        // 清理资源
        if (log_file) {
            fclose(log_file);
            log_file = NULL;
        }
        log_cleanup();
        return 1;
    }
    
    log_info("All threads started successfully");
    log_info("Press Enter to stop application...");
    
    // 等待用户输入
    getchar();

    log_info("Shutting down application...");

    // 等待线程结束
    if(hControlThread != NULL)
    {
        WaitForSingleObject(hControlThread, INFINITE);
        CloseHandle(hControlThread);
        hControlThread = NULL;
    }
    
    if(hSocketThread != NULL)
    {
        WaitForSingleObject(hSocketThread, INFINITE);
        CloseHandle(hSocketThread);
        hSocketThread = NULL;
    }
    
    // 等待CSV写入线程结束
    if(hCSVWriterThread != NULL)
    {
        WaitForSingleObject(hCSVWriterThread, INFINITE);
        CloseHandle(hCSVWriterThread);
        hCSVWriterThread = NULL;
    }
    
    // 清理CSV缓冲区
    CleanupCSVBuffer();
    // 关闭日志文件
    if (log_file) {
        fclose(log_file);
        log_file = NULL;
    }   
    // 清理日志系统
    log_cleanup();
    
    log_info("Application completed");
    return 0;
}