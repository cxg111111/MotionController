#ifndef CSV_WRITER_H
#define CSV_WRITER_H

#include <stdio.h>
#include "ThreadControl.h"

#define DATA_BUFFER_SIZE 1000  // 数据缓冲区大小
#define AXIS_COUNT 2  // 轴数改为2

// CSV数据结构
typedef struct{
    int step;
    double time;
    double targetPosition[AXIS_COUNT];
    double actualPosition[AXIS_COUNT];
    double error[AXIS_COUNT];
    double controlForce[AXIS_COUNT];
    int controlMode[AXIS_COUNT];
} CSVData;

// 函数声明
void InitCSVBuffer(void);
void CleanupCSVBuffer(void);
void SetCSVFile(FILE* pFile);
int WriteCSVDataToBuffer(int step, double time, 
                        double targetPosition[], double actualPosition[],
                        double error[], double controlForce[], int controlMode[]);
void* CSVWriterThreadFunction(void* param);

#endif