/**
 * @file FourthOrderTrajectoryPlanning.h
 * @brief 四阶(Snap)轨迹规划器的应用程序接口(API)定义。
 * @brief
 * @brief 本文件定义了实现“逐点式”四阶轨迹规划所需的所有数据结构和函数原型。
 * @brief “逐点式”设计旨在适用于内存有限的嵌入式系统，它通过一个上下文(Context)结构体
 * @brief 来管理状态，使得每次调用都能生成轨迹的下一个数据点，而无需一次性生成并储存整个轨迹数组。
 * @version 1.2
 * @date 2025-10-21
 */

#ifndef FOURTHORDERTRAJECTORYPLANNING_H
#define FOURTHORDERTRAJECTORYPLANNING_H

#include <stdio.h>

/**
 * @brief stPlannerInput: 规划器输入参数结构体
 * @brief 该结构体封装了轨迹规划所需的所有物理约束和设定。
 * @note 变量前缀 'd' 表示 double 类型。
 */
typedef struct {
    double dDistance;      // [m]      要执行的总位移 (s)
    double dVMax;          // [m/s]    运动过程中的最大速度限制 (v_max)
    double dAMax;          // [m/s^2]  运动过程中的最大加速度限制 (a_max)
    double dJMax;          // [m/s^3]  运动过程中的最大加加速度限制 (jerk, j_max)
    double dDMax;          // [m/s^4]  运动过程中的最大加加加加速度限制 (snap, d_max)
    double dSampleTime;    // [s]      轨迹点的采样周期，即每个点之间的时间间隔 (Ts)
    double dTimeLimit;     // [s]      要求完成运动的限制时间。如果 <= 0，则忽略此约束，计算最优时间。
} stPlannerInput;

/**
 * @brief stTrajectoryPoint: 单一轨迹点的状态数据结构
 * @brief 该结构体用于储存轨迹在某一特定时间点的完整运动学状态。
 * @brief 它是 `FourthOrderPlannerGetNextPoint` 函数的主要输出。
 * @note 变量前缀 'd' 表示 double 类型。
 */
typedef struct {
    double dTime;          // [s]      该轨迹点对应的时间
    double dPos;           // [m]      该轨迹点对应的位置
    double dVel;           // [m/s]    该轨迹点对应的速度
    double dAcc;           // [m/s^2]  该轨迹点对应的加速度
    double dJerk;          // [m/s^3]  该轨迹点对应的加加速度
    double dSnap;          // [m/s^4]  该轨迹点对应的加加加加速度
} stTrajectoryPoint;

/**
 * @brief stSegmentBoundaryState: 儲存單個邊界點的運動學狀態
 */
typedef struct {
    double dPos;
    double dVel;
    double dAcc;
    double dJerk;
} stSegmentBoundaryState;

/**
 * @brief stPlannerContext: 规划器上下文（状态管理器）
 * @brief 这是“逐点式”规划器的核心。它像一个状态机，储存了轨迹规划的
 * @brief 所有中间计算结果和当前进度，使得每次调用 GetNextPoint 时都能无缝接续。
 */
typedef struct {
    // --- 输入参数的内部副本 ---
    stPlannerInput stInput; // 储存用户传入的原始规划参数。

    // --- 算法核心：一次性计算出的时间分段参数 ---
    double dTd;             // [s] Snap 从0变化到/从 dDMax 变化到0所需的时间
    double dTj;             // [s] Jerk 以 dJMax 匀速变化的时间
    double dTa;             // [s] 加速度以 dAMax 匀速变化的时间
    double dTv;             // [s] 速度以 dVMax 匀速运动的时间
    double dTotalTime;      // [s] 整个轨迹的总时长

    // --- 时间缩放相关的状态 ---
    int bIsTimeScaled;          // 标志位：是否因时间限制严苛而执行了时间缩放 (0=否, 1=是)
    double dAlphaScaleFactor;   // 若执行了缩放，此为最终的缩放因子 alpha (默认1.0)

    // --- 预计算的边界时间点，用于在 GetNextPoint 中进行高效的区间判断 ---
    double adAccSegBorders[8];  // 加速段7个子阶段的8个边界时间点 (从0开始)
    double adDecSegBorders[8];  // 减速段7个子阶段的8个边界时间点
    double dConstVelStartTime;  // [s] 匀速段的开始时间
    double dDecelStartTime;     // [s] 减速段的开始时间
    
    // --- 新增: 预计算的边界状态 ---
    stSegmentBoundaryState stAccStateAtBorder[8]; // 儲存加速段 0 到 7 結束時的狀態 (共8個點, stAccStateAtBorder[0] 為 t=0 狀態)
    stSegmentBoundaryState stDecStateAtBorder[8]; // 儲存減速段 0 到 7 結束時的狀態 (相對於減速段起點)
    stSegmentBoundaryState stConstVelEndState;    // 儲存勻速段結束時的狀態

    // --- 轨迹生成器的当前状态 ---
    double dCurrentTime;       // [s] 内部计时器，记录下一个待计算点的时间
    int    bIsFinished;        // 标志位，0表示轨迹未完成，1表示已完成
} stPlannerContext;

/**
 * @brief stPlannerDiagnostics: 存储诊断信息与峰值
 * @brief 该结构体用于在规划结束后，统一返回轨迹的各项性能指标。
 */
typedef struct {
    double dTotalTime;         // [s] 最终规划出的总时长
    double dTd, dTj, dTa, dTv; // 最终使用的时间分段参数

    // --- 新增成员，用于清晰地反馈时间缩放结果 ---
    int    bIsTimeScaled;      // 标志位：是否执行了时间缩放 (0=否, 1=是)
    double dAlphaScaleFactor;  // 时间缩放因子 alpha (如果未缩放，则为 1.0)
    double dVMaxEffective;     // [m/s]   缩放后实际生效的最大速度 (等于 alpha * VMax)
    double dAMaxEffective;     // [m/s^2] 缩放后实际生效的最大加速度 (等于 alpha^2 * AMax)
    double dJMaxEffective;     // [m/s^3] 缩放后实际生效的最大Jerk (等于 alpha^3 * JMax)
    double dDMaxEffective;     // [m/s^4] 缩放后实际生效的最大Snap (等于 alpha^4 * DMax)

    // --- 轨迹峰值信息 ---
    double dVPeak;             // [m/s]   实际轨迹的峰值速度
    double dAPeak;             // [m/s^2] 实际轨迹的峰值加速度
    double dJPeak;             // [m/s^3] 实际轨迹的峰值 jerk
    int iExitFlag;             // 退出标志 (0: OK, <0: error)
} stPlannerDiagnostics;


/**
 * @brief      FourthOrderPlannerInit: 初始化轨迹规划器上下文
 * @details    这是轨迹规划的第一步。此函数会根据输入参数执行所有复杂的预计算（例如判断长短行程、二分搜索等），
               并准备好一个包含所有规划参数的上下文结构体。
 * @param[in]  pstInput 指向包含所有物理约束的输入参数结构体，不可为 NULL。
 * @return     成功时，返回一个指向新分配的 `stPlannerContext` 结构体的指针。
 * @return     失败时（例如输入无效或内存分配失败），返回 NULL。
 */
stPlannerContext* FourthOrderPlannerInit(const stPlannerInput *pstInput);

/**
 * @brief         FourthOrderPlannerGetNextPoint: 获取轨迹的下一个数据点
 * @details       这是轨迹规划的第二步。在一个循环中重复调用此函数，每次调用都会计算出轨迹上下一个时间点的完整运动学状态。
 * @param[in,out] pContext 指向由 `FourthOrderPlannerInit` 函数创建的上下文结构体。函数会修改其内部状态（如dCurrentTime）。
 * @param[out]    pPointOutput 指向一个 `stTrajectoryPoint` 结构体，用于储存计算出的数据点。
 * @return        0 - 成功获取一个有效数据点，轨迹尚未结束。
 * @return        1 - 轨迹已到达终点，本次是最后一个有效点或已无更多点。
 */
int FourthOrderPlannerGetNextPoint(stPlannerContext *pContext, stTrajectoryPoint *pPointOutput);

/**
 * @brief      FourthOrderPlannerFree: 释放规划器上下文所占用的内存
 * @details    这是轨迹规划的最后一步。当轨迹生成完毕后，应调用此函数来释放`FourthOrderPlannerInit` 函数所分配的内存。
 * @param[in]  pContext 指向要释放的 `stPlannerContext` 结构体的指针。
 */
void FourthOrderPlannerFree(stPlannerContext *pContext);

#endif // FOURTHORDERTRAJECTORYPLANNING_H