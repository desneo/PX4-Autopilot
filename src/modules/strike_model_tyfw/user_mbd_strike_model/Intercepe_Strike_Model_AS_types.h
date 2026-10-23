/*
 * File: Intercepe_Strike_Model_AS_types.h
 *
 * Code generated for Simulink model 'Intercepe_Strike_Model_AS'.
 *
 * Model version                  : 1.136
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Thu Oct 22 17:32:43 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef Intercepe_Strike_Model_AS_types_h_
#define Intercepe_Strike_Model_AS_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_STRIKE_PARAM_t_
#define DEFINED_TYPEDEF_FOR_STRIKE_PARAM_t_

/* 打击控制器参数结构体 */
typedef struct {
  /* 滚转通道增益Kp */
  real32_T STRIKE_ROLL_KP;

  /* 滚转通道积分增益Ki */
  real32_T STRIKE_ROLL_KI;

  /* 俯仰通道增益Kp */
  real32_T STRIKE_PITCH_KP;

  /* 俯仰通道积分增益Ki */
  real32_T STRIKE_PITCH_KI;

  /* 误差低通滤波器截止频率(Hz) */
  real32_T STRIKE_LOWPASS_HZ;
} STRIKE_PARAM_t;

#endif

#ifndef DEFINED_TYPEDEF_FOR_SeekerGuidanceBus_
#define DEFINED_TYPEDEF_FOR_SeekerGuidanceBus_

/* 导引头制导控制用反馈信息 Bus (DYT遥测协议提取, 存原始传输值) */
typedef struct {
  /* DYT上电后ms计数值, 字节100~103, U32, 比例尺1, 单位ms */
  uint64_T timestamp;

  /* 自检结果, 字节4: 1正常/2故障/3自检中 */
  uint8_T selfCheckResult;

  /* 工作状态, 字节5: bit0跟踪, bit2除雾, bit4~3视频源, bit6~5算法, bit7推流 */
  uint8_T workStatus;

  /* 跟踪置信度, 字节22, 比例尺10, 传输值/10得0~25.5 */
  real32_T trackConfidence;

  /* 俯仰视线角速度, 字节23~24, S16, 比例尺100, 传输值/100得deg/s */
  real32_T losRatePitch;

  /* 偏航视线角速度, 字节25~26, S16, 比例尺100, 传输值/100得deg/s */
  real32_T losRateYaw;

  /* 俯仰失调角, 字节18~19, S16, 比例尺100, 传输值/100得deg */
  real32_T misalignPitch;

  /* 偏航失调角, 字节20~21, S16, 比例尺100, 传输值/100得deg */
  real32_T misalignYaw;

  /* 俯仰框架角, 字节6~7, S16, 比例尺100, 传输值/100得deg */
  real32_T frameAnglePitch;

  /* 偏航框架角, 字节8~9, S16, 比例尺100, 传输值/100得deg */
  real32_T frameAngleYaw;
} SeekerGuidanceBus;

#endif

#ifndef DEFINED_TYPEDEF_FOR_VehicleCommandBus_
#define DEFINED_TYPEDEF_FOR_VehicleCommandBus_

/* vehicle_command 输出 Bus */
typedef struct {
  /* 时间戳 (us) */
  uint64_T timestamp;

  /* 命令 ID */
  uint32_T command;

  /* 参数1 */
  real32_T param1;

  /* 参数2 */
  real32_T param2;

  /* 参数3 */
  real32_T param3;

  /* 参数4 */
  real32_T param4;

  /* 参数5 */
  real32_T param5;

  /* 参数6 */
  real32_T param6;

  /* 参数7 */
  real32_T param7;

  /* 发送使能, true 时发布 */
  boolean_T cmd_valid;
} VehicleCommandBus;

#endif
#endif                                 /* Intercepe_Strike_Model_AS_types_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
