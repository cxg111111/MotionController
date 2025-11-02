/**
 * @file FourthOrderTrajectoryPlanning.c
 * @brief 四阶(Snap)轨迹规划器API的实现 - 最终稳定版 v1.10.1
 * @details
 * - [v1.10.1] 添加 CalculateRampKinematicsInternal 的静态函数原型声明，修复编译错误。
 * - 采用预计算边界点状态的策略，彻底解决数值累计误差。
 * - Init 函数负责一次性精确计算所有时间分段和边界状态。
 * - CalculatePoint 函数基于预计算状态进行分段独立计算。
 * - 确保短行程、时间缩放等情况下的稳定性和精度。
 * @version 1.10.1
 * @date 2025-10-24
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "FourthOrderTrajectoryPlanning.h" // 假设包含了 isfinite 的定义 (math.h)

#ifndef isfinite
#ifdef _MSC_VER // For MSVC
#include <float.h>
#define isfinite _finite
#else // For GCC/Clang etc.
// 使用标准 C99 的 isfinite
#include <math.h>
#endif
#endif

// --- 内部辅助函数原型 ---
static void CalculatePoint(const stPlannerContext *pContext, double dTime, stTrajectoryPoint *pPointOutput);
// [*** 修正 ***] 添加 CalculateRampKinematicsInternal 的原型声明
static void CalculateRampKinematicsInternal(double dTargetV, double dTargetA, double dJMax, double dDMax,
                                            double *pdTd, double *pdTj, double *pdTa, double *pdFinalV, double *pdFinalS);
static void CalculateRampKinematicsForSearch(double dTargetA, double dJMax, double dDMax,
                                             double *pdTd, double *pdTj, double *pdS_ramp, int* pErrorFlag);
static double CalculateOptimalTimeSegments(const stPlannerInput* pInput, double* pdTd, double* pdTj, double* pdTa, double* pdTv, int* pErrorFlag);


/**
 * @brief 初始化轨迹规划器 (包含预计算边界状态)
 * @details
 *   本函数完成以下核心工作：
 *   1. 输入参数合法性检查
 *   2. 分配并初始化上下文结构体
 *   3. 计算最优时间分段（无时间限制时）
 *   4. 若有限制时间，则通过二分搜索缩放因子 alpha 调整物理约束
 *   5. 确定最终时间分段、总时长
 *   6. 预计算加速段与减速段的 8 个边界时间点
 *   7. 预计算每个边界点的完整运动学状态（位置、速度、加速度、Jerk）
 *   8. 验证最终位置与速度误差在容许范围内
 *   9. 初始化内部计时器
 *   整个过程一次性完成，后续 GetNextPoint 只做局部积分，极大地提升精度与实时性。
 */
stPlannerContext* FourthOrderPlannerInit(const stPlannerInput *pstInput) {
    // ===== 步骤 1: 输入验证 =====
    // 检查输入指针是否为空、位移是否为负、所有约束是否为正、采样周期是否有效
    if (!pstInput || pstInput->dDistance < 0.0 || pstInput->dVMax <= 0.0 || pstInput->dAMax <= 0.0 ||
        pstInput->dJMax <= 0.0 || pstInput->dDMax <= 0.0 || pstInput->dSampleTime <= 0.0) {
        fprintf(stderr, "ERROR: Invalid inputs in Init\n"); return NULL;
    }

    // ===== 步骤 2: 分配上下文内存 =====
    stPlannerContext *pContext = (stPlannerContext*)malloc(sizeof(stPlannerContext));
    if (!pContext) { fprintf(stderr, "ERROR: Failed to allocate memory\n"); return NULL; }
    memset(pContext, 0, sizeof(stPlannerContext));
    pContext->stInput = *pstInput;
    pContext->dAlphaScaleFactor = 1.0; pContext->bIsTimeScaled = 0;

    double dTd_final, dTj_final, dTa_final, dTv_final; // 最终的时间分段结果
    double dAlphaFinal = 1.0;
    double dFinalTime = 0.0;
    int errorFlag = 0;

    // ===== 步骤 3: 计算最优时间 =====
    // 调用核心函数计算在无时间限制下的最优时间分段
    double dOptimalTime = CalculateOptimalTimeSegments(pstInput, &dTd_final, &dTj_final, &dTa_final, &dTv_final, &errorFlag);
    if (errorFlag) { fprintf(stderr, "ERROR: Failed to calculate optimal time.\n"); free(pContext); return NULL; }
    double dTimeLimit = pstInput->dTimeLimit;

    // ===== 步骤 4: 处理时间限制 (二分搜索 alpha) =====
    const int MAX_ITERATIONS = 100;
    const double TIME_TOLERANCE = 1e-9;
    const double ALPHA_TOLERANCE = 1e-7;

    if (dTimeLimit > 0 && fabs(dTimeLimit - dOptimalTime) > TIME_TOLERANCE) {
        // --- 需要进行时间缩放 ---
        pContext->bIsTimeScaled = 1;
        double dAlphaLow, dAlphaHigh;
        double dBestAlpha = 1.0, dMinTimeError = 1e300;
        // 记录最佳 alpha 对应的最佳分段
        double dTd_best=dTd_final, dTj_best=dTj_final, dTa_best=dTa_final, dTv_best=dTv_final;

        if (dTimeLimit < dOptimalTime) { // 时间严格, alpha > 1
            dAlphaLow = 1.0; dAlphaHigh = 50.0;
        } else { // 时间宽裕, alpha < 1
            dAlphaLow = 1e-8; dAlphaHigh = 1.0;
        }

        for (int i = 0; i < MAX_ITERATIONS; i++) {
            double dAlphaGuess = (dAlphaLow + dAlphaHigh) / 2.0;
            stPlannerInput stTempInput = *pstInput;
            // 按 alpha 的幂次缩放物理约束
            stTempInput.dVMax = pow(dAlphaGuess, 1.0) * pstInput->dVMax;
            stTempInput.dAMax = pow(dAlphaGuess, 2.0) * pstInput->dAMax;
            stTempInput.dJMax = pow(dAlphaGuess, 3.0) * pstInput->dJMax;
            stTempInput.dDMax = pow(dAlphaGuess, 4.0) * pstInput->dDMax;

            int tempError = 0;
            double dTd_guess, dTj_guess, dTa_guess, dTv_guess;
            double dTempOptimalTime = CalculateOptimalTimeSegments(&stTempInput, &dTd_guess, &dTj_guess, &dTa_guess, &dTv_guess, &tempError);

            if (tempError || !isfinite(dTempOptimalTime)) { // 计算失败或结果无效
                if (dTimeLimit < dOptimalTime) dAlphaHigh = dAlphaGuess;
                else dAlphaLow = dAlphaGuess;
                continue;
            }

            double currentTimeError = dTempOptimalTime - dTimeLimit;

            if (fabs(currentTimeError) < dMinTimeError) {
                dMinTimeError = fabs(currentTimeError);
                dBestAlpha = dAlphaGuess;
                dTd_best = dTd_guess; dTj_best = dTj_guess; dTa_best = dTa_guess; dTv_best = dTv_guess;
            }

            // 根据当前时间误差调整搜索区间
            if (dTimeLimit < dOptimalTime) {
                 if (currentTimeError > 0) dAlphaLow = dAlphaGuess; else dAlphaHigh = dAlphaGuess;
            } else {
                 if (currentTimeError > 0) dAlphaLow = dAlphaGuess; else dAlphaHigh = dAlphaGuess;
            }

            // 收敛条件：alpha 精度或时间误差足够小
            if ((dAlphaHigh - dAlphaLow) < ALPHA_TOLERANCE * fmax(1.0, dAlphaHigh) || fabs(currentTimeError) < TIME_TOLERANCE) {
                break;
            }
        }
        dAlphaFinal = dBestAlpha;
        dFinalTime = dTimeLimit;
        dTd_final = dTd_best; dTj_final = dTj_best; dTa_final = dTa_best; dTv_final = dTv_best;

    } else {
        // --- 无需缩放或恰好相等 ---
        dAlphaFinal = 1.0;
        dFinalTime = dOptimalTime;
        // 使用最优分段 (已在步骤 3 赋值给 _final 变量)
    }

    // ===== 步骤 5: 保存最终确定的 alpha 和时间分段到 Context =====
    pContext->dAlphaScaleFactor = dAlphaFinal;
    pContext->dTd = dTd_final;
    pContext->dTj = dTj_final;
    pContext->dTa = dTa_final;
    pContext->dTv = dTv_final; // 先赋值，下面可能修正

    // ===== 步骤 6: 精确计算最终的 dTe, dTotalTime, dTv =====
    double dTe_final = 4.0 * pContext->dTd + 2.0 * pContext->dTj + pContext->dTa;
    pContext->dTotalTime = dFinalTime;
    pContext->dTv = pContext->dTotalTime - 2.0 * dTe_final;
    if (pContext->dTv < 0.0) pContext->dTv = 0.0;

    // ===== 步骤 7: 预计算边界时间点 =====
    pContext->adAccSegBorders[0] = 0.0;
    double adDurVec[7] = { pContext->dTd, pContext->dTj, pContext->dTd, pContext->dTa, pContext->dTd, pContext->dTj, pContext->dTd };
    for (int i = 0; i < 7; ++i) {
        pContext->adAccSegBorders[i + 1] = pContext->adAccSegBorders[i] + adDurVec[i];
    }
    pContext->dConstVelStartTime = dTe_final;
    pContext->dDecelStartTime = pContext->dTotalTime - dTe_final;
    if (pContext->dDecelStartTime < pContext->dConstVelStartTime - 1e-9) {
         pContext->dDecelStartTime = pContext->dConstVelStartTime;
         pContext->dTv = 0.0;
    } else {
         pContext->dTv = fmax(0.0, pContext->dDecelStartTime - pContext->dConstVelStartTime);
    }
    for (int i = 0; i < 8; ++i) {
        pContext->adDecSegBorders[i] = pContext->dDecelStartTime + pContext->adAccSegBorders[i];
    }

    // ===== 步骤 8: 预计算所有边界点的状态 =====
    double dX_acc = 0.0, dV_acc = 0.0, dA_acc = 0.0, dJ_acc = 0.0;
    stPlannerInput stEffectiveInput = *pstInput;
    // 应用缩放因子得到实际生效的约束
    stEffectiveInput.dVMax = pow(pContext->dAlphaScaleFactor, 1.0) * pstInput->dVMax;
    stEffectiveInput.dAMax = pow(pContext->dAlphaScaleFactor, 2.0) * pstInput->dAMax;
    stEffectiveInput.dJMax = pow(pContext->dAlphaScaleFactor, 3.0) * pstInput->dJMax;
    stEffectiveInput.dDMax = pow(pContext->dAlphaScaleFactor, 4.0) * pstInput->dDMax;
    double dD_eff = stEffectiveInput.dDMax;
    // 加速段 Snap 序列：正向变化
    double adSnapAcc[7] = { dD_eff, 0, -dD_eff, 0, -dD_eff, 0, dD_eff };

    // 初始化加速段第0个边界点（起始点）
    pContext->stAccStateAtBorder[0] = (stSegmentBoundaryState){0.0, 0.0, 0.0, 0.0};
    for (int i = 0; i < 7; ++i) {
        double dDt = adDurVec[i]; double dSnap = adSnapAcc[i];
        if (dDt < 1e-12) { pContext->stAccStateAtBorder[i+1] = pContext->stAccStateAtBorder[i]; continue; }
        double dt2=dDt*dDt, dt3=pow(dDt,3), dt4=pow(dDt,4);
        // 四阶多项式积分更新位置、速度、加速度、Jerk
        dX_acc += dV_acc * dDt + 0.5 * dA_acc * dt2 + (1.0/6.0) * dJ_acc * dt3 + (1.0/24.0) * dSnap * dt4;
        dV_acc += dA_acc * dDt + 0.5 * dJ_acc * dt2 + (1.0/6.0) * dSnap * dt3;
        dA_acc += dJ_acc * dDt + 0.5 * dSnap * dt2;
        dJ_acc += dSnap * dDt;
         if (!isfinite(dX_acc) || !isfinite(dV_acc) || !isfinite(dA_acc) || !isfinite(dJ_acc)) {
            fprintf(stderr, "ERROR: Invalid state during acceleration precomputation at segment %d\n", i);
            free(pContext); return NULL;
        }
        pContext->stAccStateAtBorder[i+1] = (stSegmentBoundaryState){dX_acc, dV_acc, dA_acc, dJ_acc};
    }

    // 匀速段结束状态 = 加速段结束状态 + 匀速段位移
    pContext->stConstVelEndState = pContext->stAccStateAtBorder[7];
    pContext->stConstVelEndState.dPos += pContext->stAccStateAtBorder[7].dVel * pContext->dTv;

    // 减速段状态计算（从匀速段结束点开始）
    double dX_dec = pContext->stConstVelEndState.dPos;
    double dV_dec = pContext->stConstVelEndState.dVel;
    double dA_dec = 0.0; double dJ_dec = 0.0;
    // 减速段 Snap 序列：反向变化
    double adSnapDec[7] = { -dD_eff, 0, dD_eff, 0, dD_eff, 0, -dD_eff };
    pContext->stDecStateAtBorder[0] = pContext->stConstVelEndState;
    for (int i = 0; i < 7; ++i) {
        double dDt = adDurVec[i]; double dSnap = adSnapDec[i];
         if (dDt < 1e-12) { pContext->stDecStateAtBorder[i+1] = pContext->stDecStateAtBorder[i]; continue; }
        double dt2=dDt*dDt, dt3=pow(dDt,3), dt4=pow(dDt,4);
        dX_dec += dV_dec * dDt + 0.5 * dA_dec * dt2 + (1.0/6.0) * dJ_dec * dt3 + (1.0/24.0) * dSnap * dt4;
        dV_dec += dA_dec * dDt + 0.5 * dJ_dec * dt2 + (1.0/6.0) * dSnap * dt3;
        dA_dec += dJ_dec * dDt + 0.5 * dSnap * dt2;
        dJ_dec += dSnap * dDt;
        if (!isfinite(dX_dec) || !isfinite(dV_dec) || !isfinite(dA_dec) || !isfinite(dJ_dec)) {
            fprintf(stderr, "ERROR: Invalid state during deceleration precomputation at segment %d\n", i);
            free(pContext); return NULL;
        }
        pContext->stDecStateAtBorder[i+1] = (stSegmentBoundaryState){dX_dec, dV_dec, dA_dec, dJ_dec};
    }

     // 验证最终位置与速度是否满足要求
     double finalPosError = fabs(pContext->stDecStateAtBorder[7].dPos - pContext->stInput.dDistance);
     double finalVelError = fabs(pContext->stDecStateAtBorder[7].dVel);
     if (finalPosError > 1e-6 || finalVelError > 1e-6) {
         fprintf(stderr, "ERROR: Init - Precalculated final state deviates significantly.\n");
         fprintf(stderr, "       Final Pos: %.9f (Target: %.9f, Error: %.3e)\n", pContext->stDecStateAtBorder[7].dPos, pContext->stInput.dDistance, finalPosError);
         fprintf(stderr, "       Final Vel: %.9f (Error: %.3e)\n", pContext->stDecStateAtBorder[7].dVel, finalVelError);
          free(pContext); return NULL;
     }

    // ===== 步骤 9: 初始化内部状态 =====
    pContext->dCurrentTime = 0.0;
    pContext->bIsFinished = 0;

    return pContext;
}


/**
 * @brief 获取轨迹的下一个点 (增加边界检查)
 * @details
 *   每次调用返回当前时间点对应的完整运动学状态。
 *   通过预计算的边界状态 + 局部积分实现高精度输出。
 *   支持实时逐点调用，适用于嵌入式控制循环。
 */
int FourthOrderPlannerGetNextPoint(stPlannerContext *pContext, stTrajectoryPoint *pPointOutput) {
    if (!pContext || !pPointOutput) { return 1; }
    if (pContext->bIsFinished && pContext->dCurrentTime > pContext->dTotalTime) return 1;

    double dTime = pContext->dCurrentTime;
    const double EPS = 1e-9;

    if (!pContext->bIsFinished && dTime >= pContext->dTotalTime - EPS) {
        dTime = pContext->dTotalTime;
        pContext->bIsFinished = 1;
    } else if (dTime < 0.0) {
        dTime = 0.0;
    }

    CalculatePoint(pContext, dTime, pPointOutput);

    if (!pContext->bIsFinished) {
        pContext->dCurrentTime += pContext->stInput.dSampleTime;
    } else {
        pContext->dCurrentTime = pContext->dTotalTime + pContext->stInput.dSampleTime;
    }

    return 0;
}

/**
 * @brief 释放规划器上下文内存
 */
void FourthOrderPlannerFree(stPlannerContext *pContext) {
    if (pContext) { free(pContext); }
}

/**
 * @brief 根据当前时间计算轨迹点 (核心计算函数)
 * @details
 *   1. 判断当前时间属于加速段、匀速段还是减速段
 *   2. 确定所在子阶段（7段法）
 *   3. 使用预计算的边界状态作为初始值
 *   4. 在当前子阶段内进行局部四阶积分
 *   5. 输出位置、速度、加速度、Jerk、Snap
 */
static void CalculatePoint(const stPlannerContext *pContext, double dTime, stTrajectoryPoint *pPointOutput) {
    // 获取生效的约束
    double dAlpha = pContext->dAlphaScaleFactor;
    double dD = pow(dAlpha, 4.0) * pContext->stInput.dDMax;

    memset(pPointOutput, 0, sizeof(stTrajectoryPoint));
    pPointOutput->dTime = dTime;
    const double EPS = 1e-12; // 內部計算容差

    // 处理 dTime=0
    if (dTime < EPS) {
        stSegmentBoundaryState initialState = pContext->stAccStateAtBorder[0];
        pPointOutput->dPos = initialState.dPos; pPointOutput->dVel = initialState.dVel;
        pPointOutput->dAcc = initialState.dAcc; pPointOutput->dJerk = initialState.dJerk;
        pPointOutput->dSnap = (pContext->dTd > EPS) ? dD : 0.0;
        return;
    }
    // 处理 dTime=dTotalTime
    if (fabs(dTime - pContext->dTotalTime) < EPS) {
        stSegmentBoundaryState finalState = pContext->stDecStateAtBorder[7];
        pPointOutput->dPos = finalState.dPos;
        pPointOutput->dVel = 0.0; pPointOutput->dAcc = 0.0; pPointOutput->dJerk = 0.0; pPointOutput->dSnap = 0.0;
         // 轻微修正，确保完全到达目标
         if (fabs(pPointOutput->dPos - pContext->stInput.dDistance) < 1e-6) {
              pPointOutput->dPos = pContext->stInput.dDistance;
         }
        return;
    }

    // 根据时间判断阶段
    int iSegment = -1;
    double dSegmentStartTime = 0.0;
    stSegmentBoundaryState initialState = {0};
    double currentSnapValue = 0.0;
    const double* snapProfile = NULL;

    if (dTime >= pContext->dConstVelStartTime - EPS && dTime < pContext->dDecelStartTime - EPS) {
         // ========== 阶段 2: 勻速段 ==========
         initialState = pContext->stAccStateAtBorder[7];
         double dTau = dTime - pContext->dConstVelStartTime; if(dTau < 0) dTau = 0;
         pPointOutput->dSnap = 0.0; pPointOutput->dJerk = 0.0; pPointOutput->dAcc  = 0.0;
         pPointOutput->dVel  = initialState.dVel;
         pPointOutput->dPos  = initialState.dPos + initialState.dVel * dTau;
         return;
    }
    else if (dTime >= pContext->dDecelStartTime - EPS) {
        // ========== 阶段 3: 減速段 ==========
        double adSnapDec[7] = { -dD, 0, dD, 0, dD, 0, -dD };
        snapProfile = adSnapDec;
        for (int i = 0; i < 7; ++i) {
            if (dTime >= pContext->adDecSegBorders[i] - EPS && dTime < pContext->adDecSegBorders[i+1] - EPS) {
                iSegment = i;
                initialState = pContext->stDecStateAtBorder[i];
                dSegmentStartTime = pContext->adDecSegBorders[i];
                break;
            }
        }
         // 特殊处理恰好在减速阶段结束点的情況（已被 dTime=dTotalTime 處理）
         if (iSegment == -1 && fabs(dTime - pContext->adDecSegBorders[7]) < EPS) {
             // 應該被 dTime=dTotalTime 分支捕獲，但作為後備
             stSegmentBoundaryState finalState = pContext->stDecStateAtBorder[7];
             pPointOutput->dPos = finalState.dPos; pPointOutput->dVel = 0.0; pPointOutput->dAcc = 0.0;
             pPointOutput->dJerk = 0.0; pPointOutput->dSnap = 0.0;
             return;
         }
    }
    else { // dTime < pContext->dConstVelStartTime - EPS
        // ========== 阶段 1: 加速段 ==========
        double adSnapAcc[7] = { dD, 0, -dD, 0, -dD, 0, dD };
        snapProfile = adSnapAcc;
        for (int i = 0; i < 7; ++i) {
            if (dTime >= pContext->adAccSegBorders[i] - EPS && dTime < pContext->adAccSegBorders[i+1] - EPS) {
                iSegment = i;
                initialState = pContext->stAccStateAtBorder[i];
                dSegmentStartTime = pContext->adAccSegBorders[i];
                break;
            }
        }
         // 特殊处理恰好在加速段结束点
         if (iSegment == -1 && fabs(dTime - pContext->dConstVelStartTime) < EPS) {
             initialState = pContext->stAccStateAtBorder[7];
             pPointOutput->dPos = initialState.dPos; pPointOutput->dVel = initialState.dVel;
             pPointOutput->dAcc = initialState.dAcc; pPointOutput->dJerk = initialState.dJerk;
             pPointOutput->dSnap = adSnapAcc[6]; // 第七段结束时的 Snap
             return;
         }
    }

    // --- 执行选定段内的积分计算 ---
    if (iSegment >= 0 && iSegment < 7 && snapProfile != NULL) {
        double dTau = dTime - dSegmentStartTime; if(dTau < 0) dTau = 0;

        double dSegDuration = 0.0;
        if (snapProfile[0] == dD) { // 加速段
             dSegDuration = pContext->adAccSegBorders[iSegment+1] - pContext->adAccSegBorders[iSegment];
        } else { // 减速段
             dSegDuration = pContext->adDecSegBorders[iSegment+1] - pContext->adDecSegBorders[iSegment];
        }
        if (dTau > dSegDuration + EPS) dTau = fmax(0.0, dSegDuration);

        currentSnapValue = snapProfile[iSegment];
        double dX0 = initialState.dPos, dV0 = initialState.dVel, dA0 = initialState.dAcc, dJ0 = initialState.dJerk;
        double dt2=dTau*dTau, dt3=(dTau>0)?pow(dTau,3):0.0, dt4=(dTau>0)?pow(dTau,4):0.0;

        pPointOutput->dSnap = currentSnapValue;
        pPointOutput->dJerk = dJ0 + currentSnapValue * dTau;
        pPointOutput->dAcc  = dA0 + dJ0 * dTau + 0.5 * currentSnapValue * dt2;
        pPointOutput->dVel  = dV0 + dA0 * dTau + 0.5 * dJ0 * dt2 + (1.0/6.0) * currentSnapValue * dt3;
        pPointOutput->dPos  = dX0 + dV0 * dTau + 0.5 * dA0 * dt2 + (1.0/6.0) * dJ0 * dt3 + (1.0/24.0) * currentSnapValue * dt4;

        if (!isfinite(pPointOutput->dPos)) {
             fprintf(stderr, "ERROR: Invalid state (Pos=%.3e) calculated at T=%.9f, Seg=%d, Tau=%.9f\n",
                     pPointOutput->dPos, dTime, iSegment, dTau);
             // 保持上一个状态或置零
             pPointOutput->dPos = dX0; // 回滚
        }

    } else {
        // 未找到段落，可能因 EPS 容差恰好落在边界外，或 Init 计算有误
        fprintf(stderr, "Warning: CalculatePoint could not determine segment for time %.9f. ConstT=%.9f, DecelT=%.9f\n",
                dTime, pContext->dConstVelStartTime, pContext->dDecelStartTime);
        // 安全赋值：使用最近的边界状态
        if (dTime > pContext->dDecelStartTime) {
             initialState = pContext->stDecStateAtBorder[7]; // 终点
        } else if (dTime > pContext->dConstVelStartTime) {
             initialState = pContext->stAccStateAtBorder[7]; // 加速结束
        } else {
             initialState = pContext->stAccStateAtBorder[0]; // 起点
        }
         pPointOutput->dPos = initialState.dPos; pPointOutput->dVel = initialState.dVel; pPointOutput->dAcc = initialState.dAcc;
         pPointOutput->dJerk = initialState.dJerk; pPointOutput->dSnap = 0;
    }
}


/**
 * @brief 计算给定约束下的最优时间分段 (强化短行程)
 * @details
 *   1. 若位移极小，直接返回 0
 *   2. 计算加速到最大速度所需的位移 S_ramp
 *   3. 若总位移足够长 → 有匀速段
 *   4. 若位移不足 → 无匀速段，通过二分搜索峰值加速度 a_peak 使 2*S_ramp = dS
 *   5. 返回总时间
 */
static double CalculateOptimalTimeSegments(const stPlannerInput* pInput,
                                           double* pdTd, double* pdTj, double* pdTa, double* pdTv, int* pErrorFlag) {
    *pErrorFlag = 0;
    double dS = pInput->dDistance;
    double dVmax = pInput->dVMax;
    double dAmax = pInput->dAMax;
    double dJmax = pInput->dJMax;
    double dDmax = pInput->dDMax;

    if (dS < 1e-12) { *pdTd=0; *pdTj=0; *pdTa=0; *pdTv=0; return 0.0; }

    // 1. 尝试以最大加速度加速到最大速度
    double dSRampVmax = 0.0, dVRampVmax = 0.0;
    double tempTd, tempTj, tempTa;
    // [*** 修正 ***] 使用 Internal 版本
    CalculateRampKinematicsInternal(dVmax, dAmax, dJmax, dDmax, &tempTd, &tempTj, &tempTa, &dVRampVmax, &dSRampVmax);
    if (*pErrorFlag || !isfinite(dSRampVmax)) { // 增加对 Ramp 计算失败的检查
         fprintf(stderr, "ERROR: Initial CalculateRampKinematicsInternal failed.\n");
         *pErrorFlag = 1; return -1.0;
    }

    const double DIST_TOL = 1e-9;

    if (2.0 * dSRampVmax <= dS + DIST_TOL) {
        // 2a. 距离足够长，有匀速段
        *pdTd = tempTd; *pdTj = tempTj; *pdTa = tempTa;
        *pdTv = (dVmax > 1e-12) ? fmax(0.0, (dS - 2.0 * dSRampVmax) / dVmax) : 0.0;
    } else {
        // 2b. 距离不够，无匀速段 (三角波)，二分搜索峰值加速度 a_peak
        *pdTv = 0.0; *pdTa = 0.0;

        double dALow = 0.0, dAHigh = dAmax;
        double dBestAGuess = 0.0;
        double dMinSError = 1e300;
        double dTd_best=0, dTj_best=0;
        int iter;
        int foundValidSolution = 0;

        for (iter = 0; iter < 100; ++iter) {
            double dAGuess = 0.5 * (dALow + dAHigh);
            // [*** 修正 ***] 避免 A 为 0 或负数
            if (dAGuess <= 1e-15) {
                 // 如果下限已为0，且上限也接近0，可能无解或 dS 极小
                 if (dALow < 1e-14 && dAHigh < 1e-12) break;
                 dAGuess = 1e-15; // 尝试一个极小正数
            }

            double dSGuess = 0.0;
            double currentTd, currentTj;
            int rampError = 0;
            CalculateRampKinematicsForSearch(dAGuess, dJmax, dDmax, &currentTd, &currentTj, &dSGuess, &rampError);

            if (rampError || !isfinite(dSGuess)) {
                 if ((dAHigh - dALow) < 1e-9 * dAmax) break;
                 // 如果计算失败，通常意味着 A 太小，增大下限
                 dALow = dAGuess;
                 continue;
            }

            double currentSError = 2.0 * dSGuess - dS;

            if (fabs(currentSError) < dMinSError) {
                dMinSError = fabs(currentSError);
                dBestAGuess = dAGuess;
                dTd_best = currentTd; dTj_best = currentTj;
                foundValidSolution = 1;
            }

            if (currentSError > 0) { dAHigh = dAGuess; } else { dALow = dAGuess; }
            if (fabs(currentSError) < DIST_TOL || (dAHigh - dALow) < 1e-9 * fmax(1.0, dAHigh)) { break; }
        }

        if (!foundValidSolution) {
            fprintf(stderr, "ERROR: Binary search for a_peak failed to find any valid solution (dS=%.3e).\n", dS);
            *pErrorFlag = 1; return -1.0;
        }
        if (dMinSError > 1e-6) {
             fprintf(stderr, "Warning: Binary search for a_peak finished with MinSError = %.3e > 1e-6.\n", dMinSError);
             if (iter >= 99) { fprintf(stderr, "         Max iterations reached.\n"); }
        }

        *pdTd = dTd_best; *pdTj = dTj_best; *pdTa = 0.0;
        *pdTv = 0.0;
    }

    // 3. 返回总时间
    double finalTotalTime = 2.0 * (4.0 * (*pdTd) + 2.0 * (*pdTj) + (*pdTa)) + (*pdTv);
    return fmax(0.0, finalTotalTime);
}


/**
 * @brief 内部辅助函数：计算三角波加速段的 Td, Tj 和位移 S_ramp
 * @details
 *   适用于无 Ta 阶段的短行程加速段（峰值加速度 a_peak < AMax）
 *   通过给定 a_peak 计算所需 Td、Tj 和位移
 */
static void CalculateRampKinematicsForSearch(double dTargetA, double dJMax, double dDMax,
                                             double *pdTd, double *pdTj, double *pdS_ramp, int* pErrorFlag) {
    *pErrorFlag = 0;
    const double EPS = 1e-12;
    *pdTd = 0.0; *pdTj = 0.0; *pdS_ramp = 0.0;

    if (dTargetA <= EPS) { return; }
    dJMax = fmax(EPS, dJMax); dDMax = fmax(EPS, dDMax);

    // 1. 计算 Td 和 Tj (假定 Ta=0)
    double TjCrit = dJMax / dDMax; double ACrit = TjCrit * dJMax;
    if (dTargetA >= ACrit - EPS) {
        *pdTd = TjCrit; *pdTj = (dJMax > EPS) ? fmax(0.0, (dTargetA - ACrit) / dJMax) : 0.0;
    } else {
        *pdTd = sqrt(fmax(0.0, dTargetA / dDMax)); *pdTj = 0.0;
    }
    double dTa = 0.0; // 强制 Ta=0

    *pdTd = fmax(0.0, *pdTd); *pdTj = fmax(0.0, *pdTj);
     if (*pdTd < EPS && *pdTj < EPS) { return; }

    // 2. 积分计算加速段的位移 S_ramp
    double dX0 = 0.0, dV0 = 0.0, dA0 = 0.0, dJ0 = 0.0;
    double adDur[7] = { *pdTd, *pdTj, *pdTd, dTa, *pdTd, *pdTj, *pdTd }; // dTa = 0
    double adD[7] = { dDMax, 0.0, -dDMax, 0.0, -dDMax, 0.0, dDMax };

    for (int i = 0; i < 7; ++i) {
        double dDt = adDur[i]; if (dDt < EPS) continue;
        double dSnap = adD[i];
        double dt2 = dDt*dDt, dt3 = dDt*dt2, dt4 = dt2*dt2;
        dX0 += dV0 * dDt + 0.5 * dA0 * dt2 + (1.0/6.0) * dJ0 * dt3 + (1.0/24.0) * dSnap * dt4;
        dV0 += dA0 * dDt + 0.5 * dJ0 * dt2 + (1.0/6.0) * dSnap * dt3;
        dA0 += dJ0 * dDt + 0.5 * dSnap * dt2;
        dJ0 += dSnap * dDt;

        if (!isfinite(dX0) || !isfinite(dV0) || !isfinite(dA0) || !isfinite(dJ0)) {
             fprintf(stderr, "ERROR: Invalid state in CalculateRampKinematicsForSearch at seg %d (A=%.3e)\n", i, dTargetA);
             *pErrorFlag = 1; *pdS_ramp = NAN; return;
        }
    }
    *pdS_ramp = dX0;
}


/**
 * @brief 内部辅助函数：计算给定 TargetV/A 的加速段运动学参数 (增加保护)
 * @details
 *   给定目标速度 V 和目标加速度 A，计算：
 *   - Td, Tj, Ta
 *   - 最终速度（应等于 V）
 *   - 加速段总位移 S_ramp
 */
static void CalculateRampKinematicsInternal(double dTargetV, double dTargetA,
                                            double dJMax, double dDMax,
                                            double *pdTd, double *pdTj, double *pdTa,
                                            double *pdFinalV, double *pdFinalS) {
    const double EPS = 1e-12;
    *pdTd = 0.0; *pdTj = 0.0; *pdTa = 0.0; *pdFinalV = 0.0; *pdFinalS = 0.0;

    if (dTargetA <= EPS || dTargetV < -EPS) { return; }
    dJMax = fmax(EPS, dJMax); dDMax = fmax(EPS, dDMax);

    // 1. 计算Td和Tj
    double TjCrit = dJMax / dDMax; double ACrit = TjCrit * dJMax;
    if (dTargetA >= ACrit - EPS) {
        *pdTd = TjCrit; *pdTj = (dJMax > EPS) ? fmax(0.0, (dTargetA - ACrit) / dJMax) : 0.0;
    } else {
        *pdTd = sqrt(fmax(0.0, dTargetA / dDMax)); *pdTj = 0.0;
    }

    // 2. 计算Ta
    double vel_jerk_phases = dTargetA * (2.0 * (*pdTd) + (*pdTj));
    if (dTargetV >= vel_jerk_phases - EPS) {
        *pdTa = (dTargetA > EPS) ? fmax(0.0, (dTargetV - vel_jerk_phases) / dTargetA) : 0.0;
    } else {
        *pdTa = 0.0;
    }

    *pdTd = fmax(0.0, *pdTd); *pdTj = fmax(0.0, *pdTj); *pdTa = fmax(0.0, *pdTa);
     if (*pdTd < EPS && *pdTj < EPS && *pdTa < EPS) { return; }

    // 3. 积分计算加速段的终点状态
    double dX0 = 0.0, dV0 = 0.0, dA0 = 0.0, dJ0 = 0.0;
    double adDur[7] = { *pdTd, *pdTj, *pdTd, *pdTa, *pdTd, *pdTj, *pdTd };
    double adD[7] = { dDMax, 0.0, -dDMax, 0.0, -dDMax, 0.0, dDMax };

    for (int i = 0; i < 7; ++i) {
        double dDt = adDur[i]; if (dDt < EPS) continue;
        double dSnap = adD[i];
        double dt2 = dDt*dDt, dt3 = dDt*dt2, dt4 = dt2*dt2;
        dX0 += dV0 * dDt + 0.5 * dA0 * dt2 + (1.0/6.0) * dJ0 * dt3 + (1.0/24.0) * dSnap * dt4;
        dV0 += dA0 * dDt + 0.5 * dJ0 * dt2 + (1.0/6.0) * dSnap * dt3;
        dA0 += dJ0 * dDt + 0.5 * dSnap * dt2;
        dJ0 += dSnap * dDt;
        if (!isfinite(dX0) || !isfinite(dV0) || !isfinite(dA0) || !isfinite(dJ0)) {
            *pdFinalV = NAN; *pdFinalS = NAN; return; // 标记为无效
        }
    }
    *pdFinalV = dV0;
    *pdFinalS = dX0;
}