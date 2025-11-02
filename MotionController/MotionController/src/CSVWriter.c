#include "CSVWriter.h"
#include <stdio.h>
#include <windows.h>
#include <process.h>
#include <stdlib.h>
#include "ThreadControl.h"

// CSV数据缓冲区和相关变量
static CSVData g_csvDataBuffer[DATA_BUFFER_SIZE];
static int g_csvBufferHead = 0;
static int g_csvBufferTail = 0;
static int g_csvBufferCount = 0;
static HANDLE g_csvBufferMutex = NULL;
static HANDLE g_csvBufferNotEmpty = NULL;
static HANDLE g_csvBufferNotFull = NULL;
static int g_csvThreadRunning = 0;
static FILE* g_pCSVFile = NULL;

// 初始化CSV缓冲区
void InitCSVBuffer(void) {
    g_csvBufferHead = 0;
    g_csvBufferTail = 0;
    g_csvBufferCount = 0;
    g_csvThreadRunning = 1;
    
    // 创建同步对象
    g_csvBufferMutex = CreateMutex(NULL, FALSE, NULL);
    g_csvBufferNotEmpty = CreateSemaphore(NULL, 0, DATA_BUFFER_SIZE, NULL);
    g_csvBufferNotFull = CreateSemaphore(NULL, DATA_BUFFER_SIZE, DATA_BUFFER_SIZE, NULL);
}

// 清理CSV缓冲区资源
void CleanupCSVBuffer(void) {
    g_csvThreadRunning = 0;
    
    // 释放同步对象
    if (g_csvBufferMutex) {
        CloseHandle(g_csvBufferMutex);
        g_csvBufferMutex = NULL;
    }
    
    if (g_csvBufferNotEmpty) {
        CloseHandle(g_csvBufferNotEmpty);
        g_csvBufferNotEmpty = NULL;
    }
    
    if (g_csvBufferNotFull) {
        CloseHandle(g_csvBufferNotFull);
        g_csvBufferNotFull = NULL;
    }
}

// 设置CSV文件指针
void SetCSVFile(FILE* pFile) {
    g_pCSVFile = pFile;
}

// 将数据写入CSV缓冲区
int WriteCSVDataToBuffer(int step, double time, 
                        double targetPosition[], double actualPosition[],
                        double error[], double controlForce[], int controlMode[]) {
    // 等待缓冲区有空间
    WaitForSingleObject(g_csvBufferNotFull, INFINITE);
    
    // 获取互斥锁
    WaitForSingleObject(g_csvBufferMutex, INFINITE);
    
    // 检查缓冲区是否已满
    if (g_csvBufferCount >= DATA_BUFFER_SIZE) {
        ReleaseMutex(g_csvBufferMutex);
        ReleaseSemaphore(g_csvBufferNotFull, 1, NULL);
        return -1;
    }
    
    // 写入数据到缓冲区
    CSVData* data = &g_csvDataBuffer[g_csvBufferHead];
    data->step = step;
    data->time = time;
    
    for (int axis = 0; axis < AXIS_COUNT; axis++) {
        data->targetPosition[axis] = targetPosition[axis];
        data->actualPosition[axis] = actualPosition[axis];
        data->error[axis] = error[axis];
        data->controlForce[axis] = controlForce[axis];
        data->controlMode[axis] = controlMode[axis];
    }
    
    // 更新缓冲区指针
    g_csvBufferHead = (g_csvBufferHead + 1) % DATA_BUFFER_SIZE;
    g_csvBufferCount++;
    
    // 释放互斥锁
    ReleaseMutex(g_csvBufferMutex);
    
    // 发信号表示缓冲区不为空
    ReleaseSemaphore(g_csvBufferNotEmpty, 1, NULL);
    
    return 0;
}

// CSV写入线程函数
void* CSVWriterThreadFunction(void* param) {
    printf("CSV writer thread started\n");
    
    while (g_csvThreadRunning || g_csvBufferCount > 0) {
        // 等待缓冲区有数据
        if (WaitForSingleObject(g_csvBufferNotEmpty, 100) == WAIT_TIMEOUT) {
            // 超时继续检查条件
            continue;
        }
        
        // 获取互斥锁
        WaitForSingleObject(g_csvBufferMutex, INFINITE);
        
        // 检查缓冲区是否为空
        if (g_csvBufferCount <= 0) {
            ReleaseMutex(g_csvBufferMutex);
            ReleaseSemaphore(g_csvBufferNotEmpty, 1, NULL);
            continue;
        }
        
        // 从缓冲区读取数据
        CSVData data = g_csvDataBuffer[g_csvBufferTail];
        
        // 更新缓冲区指针
        g_csvBufferTail = (g_csvBufferTail + 1) % DATA_BUFFER_SIZE;
        g_csvBufferCount--;
        
        // 释放互斥锁
        ReleaseMutex(g_csvBufferMutex);
        
        // 发信号表示缓冲区有空间了
        ReleaseSemaphore(g_csvBufferNotFull, 1, NULL);
        
        // 写入数据到文件
        if (g_pCSVFile) {
            //fprintf(g_pCSVFile, "%d,%.3f", data.step, data.time);
            for(int axis = 0; axis < AXIS_COUNT; axis++) {
                fprintf(g_pCSVFile, "\n");
                fprintf(g_pCSVFile, "%d,%.3f", data.step, data.time);
                fprintf(g_pCSVFile, ",%.12f,%.15f,%.13f,%.9f,%d",
                        data.targetPosition[axis],
                        data.actualPosition[axis], 
                        data.error[axis], 
                        data.controlForce[axis],
                        data.controlMode[axis]);
            }
            //fprintf(g_pCSVFile, "\n");
            
            // 定期刷新文件以确保数据写入磁盘
            static int flushCounter = 0;
            if (++flushCounter >= 10) {
                fflush(g_pCSVFile);
                flushCounter = 0;
            }
        }
    }
    
    // 最终刷新文件
    if (g_pCSVFile) {
        fflush(g_pCSVFile);
    }
    
    printf("CSV writer thread exiting\n");
    return NULL;
}