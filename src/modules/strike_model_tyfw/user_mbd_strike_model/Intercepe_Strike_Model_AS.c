/*
 * File: Intercepe_Strike_Model_AS.c
 *
 * Code generated for Simulink model 'Intercepe_Strike_Model_AS'.
 *
 * Model version                  : 1.148
 * Simulink Coder version         : 25.2 (R2025b) 28-Jul-2025
 * C/C++ source code generated on : Thu Oct 29 14:32:06 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Intercepe_Strike_Model_AS.h"
#include "rtwtypes.h"
#include "Intercepe_Strike_Model_AS_types.h"
#include "Intercepe_Strike_Model_AS_private.h"
#include <math.h>
#include <string.h>

/* Named constants for Chart: '<S2>/Chart' */
#define Intercepe_St_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define Intercepe_Strike_Mod_IN_RECOVER ((uint8_T)3U)
#define Intercepe_Strike_Mode_IN_SEARCH ((uint8_T)4U)
#define Intercepe_Strike_Mode_IN_STRIKE ((uint8_T)5U)
#define Intercepe_Strike_Model__IN_IDLE ((uint8_T)1U)
#define Intercepe_Strike_Model__IN_LOCK ((uint8_T)2U)

/* Exported block parameters */
STRIKE_PARAM_t STRIKE_PARAM = {
  0.2F,
  0.1F,
  0.2F,
  0.1F,
  10.0F,
  1.0F,
  0.2F,
  0.15F,
  0.0045F,
  0.0F
} ;                                    /* Variable: STRIKE_PARAM
                                        * Referenced by: '<S2>/Chart'
                                        * ���ģʽ�����������ṹ�壬֧������ʱ����
                                        */

/* Block signals (default storage) */
B_Intercepe_Strike_Model_AS_T Intercepe_Strike_Model_AS_B;

/* Block states (default storage) */
DW_Intercepe_Strike_Model_AS_T Intercepe_Strike_Model_AS_DW;

/* External inputs (root inport signals with default storage) */
ExtU_Intercepe_Strike_Model_A_T Intercepe_Strike_Model_AS_U;

/* External outputs (root outports fed by signals with default storage) */
ExtY_Intercepe_Strike_Model_A_T Intercepe_Strike_Model_AS_Y;

/* Forward declaration for local functions */
static void Intercepe_Strike_Model_AS_LOCK(boolean_T *cmd_valid, const
  SeekerGuidanceBus *BusConversion_InsertedFor_Chart);
const VehicleCommandBus Intercepe_Strike_Model_AS_rtZVe = { 0ULL,/* timestamp */
  0U,                                  /* command */
  0.0F,                                /* param1 */
  0.0F,                                /* param2 */
  0.0F,                                /* param3 */
  0.0F,                                /* param4 */
  0.0F,                                /* param5 */
  0.0F,                                /* param6 */
  0.0F,                                /* param7 */
  false                                /* cmd_valid */
};

/*
 * Output and update for atomic system:
 *    '<S3>/MATLAB Function'
 *    '<S14>/MATLAB Function'
 *    '<S78>/MATLAB Function'
 */
void Intercepe_Strike_MATLABFunction(real32_T rtu_angles_rad, real32_T
  rtu_angles_rad_k, real32_T rtu_angles_rad_j, real32_T rty_DCM[9])
{
  real32_T c1;
  real32_T c2;
  real32_T c3;
  real32_T s1;
  real32_T s2;
  real32_T s3;
  real32_T tmp;

  /* SignalConversion generated from: '<S9>/ SFunction ' */
  c1 = cosf(rtu_angles_rad);
  s1 = sinf(rtu_angles_rad);
  c2 = cosf(rtu_angles_rad_k);
  s2 = sinf(rtu_angles_rad_k);
  c3 = cosf(rtu_angles_rad_j);
  s3 = sinf(rtu_angles_rad_j);
  rty_DCM[0] = c2 * c1;
  rty_DCM[3] = c2 * s1;
  rty_DCM[6] = -s2;
  tmp = s3 * s2;
  rty_DCM[1] = tmp * c1 - c3 * s1;
  rty_DCM[4] = tmp * s1 + c3 * c1;
  rty_DCM[7] = s3 * c2;
  tmp = c3 * s2;
  rty_DCM[2] = tmp * c1 + s3 * s1;
  rty_DCM[5] = tmp * s1 - s3 * c1;
  rty_DCM[8] = c3 * c2;
}

/*
 * Output and update for action system:
 *    '<S1>/Subsystem6'
 *    '<S1>/Subsystem1'
 */
void Intercepe_Strike_Mod_Subsystem6(boolean_T rtu_cmd_valid, VehicleCommandBus *
  rty_Out1, DW_Subsystem6_Intercepe_Strik_T *localDW)
{
  real_T tmp;

  /* DataTypeConversion: '<S8>/Data Type Conversion' incorporates:
   *  DiscreteIntegrator: '<S8>/Discrete-Time Integrator'
   */
  tmp = fmod(floor(localDW->DiscreteTimeIntegrator_DSTATE),
             1.8446744073709552E+19);

  /* BusAssignment: '<S8>/Bus Assignment' incorporates:
   *  Constant: '<S8>/Constant4'
   *  Constant: '<S8>/Constant5'
   *  Constant: '<S8>/Constant6'
   *  Constant: '<S8>/Constant7'
   *  Constant: '<S8>/Constant8'
   *  Constant: '<S8>/Constant9'
   *  Constant: '<S8>/NAVIGATION_STATE_AUTO_LOITER'
   *  Constant: '<S8>/VEHICLE_CMD_SET_NAV_STATE'
   *  DataTypeConversion: '<S8>/Data Type Conversion'
   */
  rty_Out1->timestamp = tmp < 0.0 ? (uint64_T)-(int64_T)(uint64_T)-tmp :
    (uint64_T)tmp;
  rty_Out1->command = 100001U;
  rty_Out1->param1 = 4.0F;
  rty_Out1->param2 = 0.0F;
  rty_Out1->param3 = 0.0F;
  rty_Out1->param4 = 0.0F;
  rty_Out1->param5 = 0.0F;
  rty_Out1->param6 = 0.0F;
  rty_Out1->param7 = 0.0F;
  rty_Out1->cmd_valid = rtu_cmd_valid;

  /* Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S8>/Constant1'
   */
  localDW->DiscreteTimeIntegrator_DSTATE += 10000.0;
}

/*
 * System initialize for atomic system:
 *    '<S11>/MATLAB Function'
 *    '<S76>/MATLAB Function'
 */
void Intercepe_MATLABFunction_h_Init(DW_MATLABFunction_Intercepe_h_T *localDW)
{
  localDW->x_prev_not_empty = false;
}

/*
 * System reset for atomic system:
 *    '<S11>/MATLAB Function'
 *    '<S76>/MATLAB Function'
 */
void Intercep_MATLABFunction_k_Reset(DW_MATLABFunction_Intercepe_h_T *localDW)
{
  localDW->x_prev_not_empty = false;
}

/*
 * Output and update for atomic system:
 *    '<S11>/MATLAB Function'
 *    '<S76>/MATLAB Function'
 */
void Intercepe_Stri_MATLABFunction_f(real32_T rtu_u, real_T rtu_fc, real_T
  rtu_Ts, real32_T *rty_y, DW_MATLABFunction_Intercepe_h_T *localDW)
{
  real_T C;
  real32_T tmp;
  C = rtu_fc;
  if (!localDW->x_prev_not_empty) {
    localDW->x_prev = rtu_u;
    localDW->x_prev_not_empty = true;
    localDW->y_prev = rtu_u;
  }

  if (rtu_fc < 0.1) {
    C = 0.1;
  }

  C = 6.2831853071795862 * C * rtu_Ts;
  tmp = (real32_T)(C / (C + 2.0));
  *rty_y = (tmp * rtu_u + tmp * localDW->x_prev) - (real32_T)((C - 2.0) / (C +
    2.0)) * localDW->y_prev;
  localDW->x_prev = rtu_u;
  localDW->y_prev = *rty_y;
}

/*
 * Output and update for atomic system:
 *    '<S14>/MATLAB Function1'
 *    '<S78>/MATLAB Function1'
 */
void Intercepe_Strik_MATLABFunction1(const real32_T rtu_angles_rad[3], real32_T
  rty_DCM[9])
{
  real32_T c1;
  real32_T c2;
  real32_T c3;
  real32_T s1;
  real32_T s2;
  real32_T s3;
  c1 = cosf(rtu_angles_rad[0]);
  s1 = sinf(rtu_angles_rad[0]);
  c2 = cosf(rtu_angles_rad[1]);
  s2 = sinf(rtu_angles_rad[1]);
  c3 = cosf(rtu_angles_rad[2]);
  s3 = sinf(rtu_angles_rad[2]);
  rty_DCM[0] = c3 * c2;
  rty_DCM[3] = c3 * s2 * s1 + s3 * c1;
  rty_DCM[6] = -c3 * s2 * c1 + s3 * s1;
  rty_DCM[1] = -s3 * c2;
  rty_DCM[4] = -s3 * s2 * s1 + c3 * c1;
  rty_DCM[7] = s3 * s2 * c1 + c3 * s1;
  rty_DCM[2] = s2;
  rty_DCM[5] = -c2 * s1;
  rty_DCM[8] = c2 * c1;
}

/*
 * System initialize for atomic system:
 *    '<S16>/MATLAB Function1'
 *    '<S16>/MATLAB Function2'
 *    '<S80>/MATLAB Function1'
 *    '<S80>/MATLAB Function2'
 */
void Intercep_MATLABFunction1_a_Init(DW_MATLABFunction1_Intercep_g_T *localDW)
{
  localDW->x_prev_not_empty = false;
}

/*
 * System reset for atomic system:
 *    '<S16>/MATLAB Function1'
 *    '<S16>/MATLAB Function2'
 *    '<S80>/MATLAB Function1'
 *    '<S80>/MATLAB Function2'
 */
void Interce_MATLABFunction1_j_Reset(DW_MATLABFunction1_Intercep_g_T *localDW)
{
  localDW->x_prev_not_empty = false;
}

/*
 * Output and update for atomic system:
 *    '<S16>/MATLAB Function1'
 *    '<S16>/MATLAB Function2'
 *    '<S80>/MATLAB Function1'
 *    '<S80>/MATLAB Function2'
 */
void Intercepe_Str_MATLABFunction1_g(real32_T rtu_u, real32_T rtu_fc, real32_T
  rtu_Ts, real32_T *rty_y, DW_MATLABFunction1_Intercep_g_T *localDW)
{
  real32_T C;
  real32_T tmp;
  C = rtu_fc;
  if (!localDW->x_prev_not_empty) {
    localDW->x_prev = rtu_u;
    localDW->x_prev_not_empty = true;
    localDW->y_prev = rtu_u;
  }

  if (rtu_fc < 0.1F) {
    C = 0.1F;
  }

  C = 6.28318548F * C * rtu_Ts;
  tmp = C / (C + 2.0F);
  *rty_y = (tmp * rtu_u + tmp * localDW->x_prev) - (C - 2.0F) / (C + 2.0F) *
    localDW->y_prev;
  localDW->x_prev = rtu_u;
  localDW->y_prev = *rty_y;
}

/*
 * System initialize for atomic system:
 *    '<S16>/MATLAB Function4'
 *    '<S80>/MATLAB Function4'
 */
void Intercepe__MATLABFunction4_Init(DW_MATLABFunction4_Intercepe__T *localDW)
{
  localDW->x_prev_not_empty = false;
}

/*
 * System reset for atomic system:
 *    '<S16>/MATLAB Function4'
 *    '<S80>/MATLAB Function4'
 */
void Intercepe_MATLABFunction4_Reset(DW_MATLABFunction4_Intercepe__T *localDW)
{
  localDW->x_prev_not_empty = false;
}

/*
 * Output and update for atomic system:
 *    '<S16>/MATLAB Function4'
 *    '<S80>/MATLAB Function4'
 */
void Intercepe_Strik_MATLABFunction4(real32_T rtu_u, real32_T rtu_fc, real32_T
  rtu_Ts, real32_T *rty_y, DW_MATLABFunction4_Intercepe__T *localDW)
{
  real32_T C;
  real32_T tmp;
  C = rtu_fc;
  if (!localDW->x_prev_not_empty) {
    localDW->x_prev = rtu_u;
    localDW->x_prev_not_empty = true;
    localDW->y_prev = rtu_u;
  }

  if (rtu_fc < 0.1F) {
    C = 0.1F;
  }

  C = 6.28318548F * C * rtu_Ts;
  tmp = C / (C + 2.0F);
  *rty_y = (tmp * rtu_u + tmp * localDW->x_prev) - (C - 2.0F) / (C + 2.0F) *
    localDW->y_prev;
  localDW->x_prev = rtu_u;
  localDW->y_prev = *rty_y;
}

/* Function for Chart: '<S2>/Chart' */
static void Intercepe_Strike_Model_AS_LOCK(boolean_T *cmd_valid, const
  SeekerGuidanceBus *BusConversion_InsertedFor_Chart)
{
  uint32_T tmp;
  boolean_T seeker_data_valid;
  boolean_T target_ok;
  Intercepe_Strike_Model_AS_B.guided_state = 3U;
  if (Intercepe_Strike_Model_AS_DW.time_gate >= 0.5F) {
    Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS =
      Intercepe_Strike_Mod_IN_RECOVER;

    /* BusCreator generated from: '<S2>/Chart' */
    Intercepe_Strike_Model_AS_DW.last_timestamp =
      BusConversion_InsertedFor_Chart->timestamp;
    Intercepe_Strike_Model_AS_DW.time_gate = 0.0F;
    Intercepe_Strike_Model_AS_B.guided_state = 4U;
    *cmd_valid = true;
  } else if ((Intercepe_Strike_Model_AS_DW.lock_time >= 5.0F) &&
             Intercepe_Strike_Model_AS_U.bool_request) {
    Intercepe_Strike_Model_AS_DW.durationCounter_1 = 0U;
    Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS =
      Intercepe_Strike_Mode_IN_STRIKE;

    /* BusCreator generated from: '<S2>/Chart' */
    Intercepe_Strike_Model_AS_DW.last_timestamp =
      BusConversion_InsertedFor_Chart->timestamp;
    Intercepe_Strike_Model_AS_DW.time_gate = 0.0F;
    Intercepe_Strike_Model_AS_B.guided_state = 5U;
    *cmd_valid = true;
  } else if ((!Intercepe_Strike_Model_AS_U.bool_search) ||
             (Intercepe_Strike_Model_AS_U.arming_state != 2)) {
    Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS =
      Intercepe_Strike_Model__IN_IDLE;
    Intercepe_Strike_Model_AS_B.guided_state = 1U;
    *cmd_valid = true;
  } else {
    if ((BusConversion_InsertedFor_Chart->timestamp !=
         Intercepe_Strike_Model_AS_DW.last_timestamp) &&
        (BusConversion_InsertedFor_Chart->timestamp > 0ULL)) {
      /* BusCreator generated from: '<S2>/Chart' */
      Intercepe_Strike_Model_AS_DW.last_timestamp =
        BusConversion_InsertedFor_Chart->timestamp;
      Intercepe_Strike_Model_AS_DW.stale_count = 0U;
      seeker_data_valid = true;
    } else {
      tmp = Intercepe_Strike_Model_AS_DW.stale_count + 1U;
      if (Intercepe_Strike_Model_AS_DW.stale_count + 1U > 255U) {
        tmp = 255U;
      }

      Intercepe_Strike_Model_AS_DW.stale_count = (uint8_T)tmp;
      seeker_data_valid = (Intercepe_Strike_Model_AS_DW.stale_count < 4);
    }

    /* BusCreator generated from: '<S2>/Chart' */
    target_ok = (((BusConversion_InsertedFor_Chart->workStatus & 1U) != 0U) &&
                 (BusConversion_InsertedFor_Chart->trackConfidence > 6.0F) &&
                 (BusConversion_InsertedFor_Chart->selfCheckResult == 1));
    if ((!seeker_data_valid) || (!target_ok)) {
      Intercepe_Strike_Model_AS_DW.time_gate += 0.01F;
    } else {
      Intercepe_Strike_Model_AS_DW.time_gate = 0.0F;
    }

    if (seeker_data_valid && target_ok) {
      Intercepe_Strike_Model_AS_DW.lock_time += 0.01F;
    } else {
      Intercepe_Strike_Model_AS_DW.lock_time = 0.0F;
    }

    *cmd_valid = false;
  }
}

/* Model step function */
void Intercepe_Strike_Model_AS_step(void)
{
  real_T tmp;
  int32_T i;
  int32_T i_0;
  int32_T rtb_DCM_tmp;
  int32_T rtb_Transpose1_p_tmp;
  real32_T rtb_DCM[9];
  real32_T rtb_DCM_h[9];
  real32_T rtb_MatrixMultiply2_cf[9];
  real32_T rtb_MatrixMultiply2_fh[9];
  real32_T rtb_Transpose1_b[9];
  real32_T rtb_D2R_i[3];
  real32_T rtb_MatrixMultiply[3];
  real32_T pooled11;
  real32_T rtb_Rollcmd_f;
  real32_T rtb_Saturation;
  real32_T rtb_Sum;
  real32_T rtb_desired_thrust;
  real32_T rtb_lambda_dot_roll_los_filte_a;
  uint32_T tmp_0;
  int8_T rtPrevAction;
  boolean_T seeker_data_valid;

  /* MATLAB Function: '<S3>/MATLAB Function' incorporates:
   *  Inport: '<Root>/current_pitch'
   *  Inport: '<Root>/current_roll'
   *  Inport: '<Root>/current_yaw'
   */
  Intercepe_Strike_MATLABFunction(Intercepe_Strike_Model_AS_U.current_yaw,
    Intercepe_Strike_Model_AS_U.current_pitch,
    Intercepe_Strike_Model_AS_U.current_roll, rtb_DCM);

  /* SignalConversion generated from: '<S3>/MatrixMultiply' incorporates:
   *  Inport: '<Root>/current_vx'
   *  Inport: '<Root>/current_vy'
   *  Inport: '<Root>/current_vz'
   */
  rtb_MatrixMultiply[0] = Intercepe_Strike_Model_AS_U.current_vx;
  rtb_MatrixMultiply[1] = Intercepe_Strike_Model_AS_U.current_vy;
  rtb_MatrixMultiply[2] = Intercepe_Strike_Model_AS_U.current_vz;

  /* Product: '<S3>/MatrixMultiply' */
  rtb_Rollcmd_f = 0.0F;
  rtb_Saturation = 0.0F;
  rtb_desired_thrust = 0.0F;
  for (i = 0; i < 3; i++) {
    pooled11 = rtb_MatrixMultiply[i];
    rtb_Rollcmd_f += rtb_DCM[3 * i] * pooled11;
    rtb_Saturation += rtb_DCM[3 * i + 1] * pooled11;
    rtb_desired_thrust += rtb_DCM[3 * i + 2] * pooled11;
  }

  rtb_MatrixMultiply[2] = rtb_desired_thrust;
  rtb_MatrixMultiply[1] = rtb_Saturation;
  rtb_MatrixMultiply[0] = rtb_Rollcmd_f;

  /* End of Product: '<S3>/MatrixMultiply' */

  /* Chart: '<S2>/Chart' incorporates:
   *  BusCreator generated from: '<S2>/Chart'
   *  Inport: '<Root>/SeekerData'
   *  Inport: '<Root>/arming_state'
   *  Inport: '<Root>/bool_request'
   *  Inport: '<Root>/bool_search'
   */
  if (Intercepe_Strike_Model_AS_DW.is_active_c3_Intercepe_Strike_M == 0) {
    Intercepe_Strike_Model_AS_DW.is_active_c3_Intercepe_Strike_M = 1U;
    Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS =
      Intercepe_Strike_Model__IN_IDLE;
    Intercepe_Strike_Model_AS_B.guided_state = 1U;
    seeker_data_valid = true;
  } else {
    switch (Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS) {
     case Intercepe_Strike_Model__IN_IDLE:
      Intercepe_Strike_Model_AS_B.guided_state = 1U;
      if (Intercepe_Strike_Model_AS_U.bool_search &&
          (Intercepe_Strike_Model_AS_U.arming_state == 2) &&
          (Intercepe_Strike_Model_AS_U.SeekerData.selfCheckResult == 1)) {
        Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS =
          Intercepe_Strike_Mode_IN_SEARCH;
        Intercepe_Strike_Model_AS_DW.last_timestamp =
          Intercepe_Strike_Model_AS_U.SeekerData.timestamp;
        Intercepe_Strike_Model_AS_DW.time_gate = 0.0F;
        Intercepe_Strike_Model_AS_B.guided_state = 2U;
        seeker_data_valid = true;
      } else {
        seeker_data_valid = false;
      }
      break;

     case Intercepe_Strike_Model__IN_LOCK:
      Intercepe_Strike_Model_AS_LOCK(&seeker_data_valid,
        &Intercepe_Strike_Model_AS_U.SeekerData);
      break;

     case Intercepe_Strike_Mod_IN_RECOVER:
      Intercepe_Strike_Model_AS_B.guided_state = 4U;
      if (Intercepe_Strike_Model_AS_DW.time_gate >= 3.0F) {
        Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS =
          Intercepe_Strike_Model__IN_LOCK;
        Intercepe_Strike_Model_AS_DW.last_timestamp =
          Intercepe_Strike_Model_AS_U.SeekerData.timestamp;
        Intercepe_Strike_Model_AS_DW.time_gate = 0.0F;
        Intercepe_Strike_Model_AS_DW.lock_time = 0.0F;
        Intercepe_Strike_Model_AS_B.guided_state = 3U;
        seeker_data_valid = true;
      } else if ((!Intercepe_Strike_Model_AS_U.bool_search) ||
                 (Intercepe_Strike_Model_AS_U.arming_state != 2)) {
        Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS =
          Intercepe_Strike_Model__IN_IDLE;
        Intercepe_Strike_Model_AS_B.guided_state = 1U;
        seeker_data_valid = true;
      } else {
        if ((Intercepe_Strike_Model_AS_U.SeekerData.timestamp !=
             Intercepe_Strike_Model_AS_DW.last_timestamp) &&
            (Intercepe_Strike_Model_AS_U.SeekerData.timestamp > 0ULL)) {
          Intercepe_Strike_Model_AS_DW.last_timestamp =
            Intercepe_Strike_Model_AS_U.SeekerData.timestamp;
          Intercepe_Strike_Model_AS_DW.stale_count = 0U;
          seeker_data_valid = true;
        } else {
          tmp_0 = Intercepe_Strike_Model_AS_DW.stale_count + 1U;
          if (Intercepe_Strike_Model_AS_DW.stale_count + 1U > 255U) {
            tmp_0 = 255U;
          }

          Intercepe_Strike_Model_AS_DW.stale_count = (uint8_T)tmp_0;
          seeker_data_valid = (Intercepe_Strike_Model_AS_DW.stale_count < 4);
        }

        if (seeker_data_valid &&
            (((Intercepe_Strike_Model_AS_U.SeekerData.workStatus & 1U) != 0U) &&
             (Intercepe_Strike_Model_AS_U.SeekerData.trackConfidence > 6.0F) &&
             (Intercepe_Strike_Model_AS_U.SeekerData.selfCheckResult == 1))) {
          Intercepe_Strike_Model_AS_DW.time_gate += 0.01F;
        } else {
          Intercepe_Strike_Model_AS_DW.time_gate = 0.0F;
        }

        seeker_data_valid = false;
      }
      break;

     case Intercepe_Strike_Mode_IN_SEARCH:
      Intercepe_Strike_Model_AS_B.guided_state = 2U;
      if (Intercepe_Strike_Model_AS_DW.time_gate >= 3.0F) {
        Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS =
          Intercepe_Strike_Model__IN_LOCK;
        Intercepe_Strike_Model_AS_DW.last_timestamp =
          Intercepe_Strike_Model_AS_U.SeekerData.timestamp;
        Intercepe_Strike_Model_AS_DW.time_gate = 0.0F;
        Intercepe_Strike_Model_AS_DW.lock_time = 0.0F;
        Intercepe_Strike_Model_AS_B.guided_state = 3U;
        seeker_data_valid = true;
      } else if ((!Intercepe_Strike_Model_AS_U.bool_search) ||
                 (Intercepe_Strike_Model_AS_U.arming_state != 2) ||
                 (Intercepe_Strike_Model_AS_U.SeekerData.selfCheckResult != 1))
      {
        Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS =
          Intercepe_Strike_Model__IN_IDLE;
        Intercepe_Strike_Model_AS_B.guided_state = 1U;
        seeker_data_valid = true;
      } else {
        if ((Intercepe_Strike_Model_AS_U.SeekerData.timestamp !=
             Intercepe_Strike_Model_AS_DW.last_timestamp) &&
            (Intercepe_Strike_Model_AS_U.SeekerData.timestamp > 0ULL)) {
          Intercepe_Strike_Model_AS_DW.last_timestamp =
            Intercepe_Strike_Model_AS_U.SeekerData.timestamp;
          Intercepe_Strike_Model_AS_DW.stale_count = 0U;
          seeker_data_valid = true;
        } else {
          tmp_0 = Intercepe_Strike_Model_AS_DW.stale_count + 1U;
          if (Intercepe_Strike_Model_AS_DW.stale_count + 1U > 255U) {
            tmp_0 = 255U;
          }

          Intercepe_Strike_Model_AS_DW.stale_count = (uint8_T)tmp_0;
          seeker_data_valid = (Intercepe_Strike_Model_AS_DW.stale_count < 4);
        }

        if (seeker_data_valid &&
            (((Intercepe_Strike_Model_AS_U.SeekerData.workStatus & 1U) != 0U) &&
             (Intercepe_Strike_Model_AS_U.SeekerData.trackConfidence > 6.0F) &&
             (Intercepe_Strike_Model_AS_U.SeekerData.selfCheckResult == 1))) {
          Intercepe_Strike_Model_AS_DW.time_gate += 0.01F;
        } else {
          Intercepe_Strike_Model_AS_DW.time_gate = 0.0F;
        }

        seeker_data_valid = false;
      }
      break;

     default:
      /* case IN_STRIKE: */
      Intercepe_Strike_Model_AS_B.guided_state = 5U;
      if ((!Intercepe_Strike_Model_AS_U.bool_request) ||
          (Intercepe_Strike_Model_AS_DW.time_gate >= 0.5F)) {
        seeker_data_valid = true;
      } else if (STRIKE_PARAM.param1 >= 0.5F) {
        if (sqrtf(Intercepe_Strike_Model_AS_U.SeekerData.trackBoxWidth *
                  Intercepe_Strike_Model_AS_U.SeekerData.trackBoxWidth +
                  Intercepe_Strike_Model_AS_U.SeekerData.trackBoxHeight *
                  Intercepe_Strike_Model_AS_U.SeekerData.trackBoxHeight) <
            500.0F) {
          Intercepe_Strike_Model_AS_DW.durationCounter_1 = 0U;
        }

        seeker_data_valid = (Intercepe_Strike_Model_AS_DW.durationCounter_1 >
                             30U);
      } else {
        seeker_data_valid = false;
      }

      if (seeker_data_valid) {
        Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS =
          Intercepe_Strike_Model__IN_IDLE;
        Intercepe_Strike_Model_AS_B.guided_state = 1U;
        seeker_data_valid = true;
      } else {
        if ((Intercepe_Strike_Model_AS_U.SeekerData.timestamp !=
             Intercepe_Strike_Model_AS_DW.last_timestamp) &&
            (Intercepe_Strike_Model_AS_U.SeekerData.timestamp > 0ULL)) {
          Intercepe_Strike_Model_AS_DW.last_timestamp =
            Intercepe_Strike_Model_AS_U.SeekerData.timestamp;
          Intercepe_Strike_Model_AS_DW.stale_count = 0U;
          seeker_data_valid = true;
        } else {
          tmp_0 = Intercepe_Strike_Model_AS_DW.stale_count + 1U;
          if (Intercepe_Strike_Model_AS_DW.stale_count + 1U > 255U) {
            tmp_0 = 255U;
          }

          Intercepe_Strike_Model_AS_DW.stale_count = (uint8_T)tmp_0;
          seeker_data_valid = (Intercepe_Strike_Model_AS_DW.stale_count < 4);
        }

        if ((!seeker_data_valid) ||
            (((Intercepe_Strike_Model_AS_U.SeekerData.workStatus & 1U) == 0U) ||
             (Intercepe_Strike_Model_AS_U.SeekerData.trackConfidence <= 6.0F) ||
             (Intercepe_Strike_Model_AS_U.SeekerData.selfCheckResult != 1))) {
          Intercepe_Strike_Model_AS_DW.time_gate += 0.01F;
        } else {
          Intercepe_Strike_Model_AS_DW.time_gate = 0.0F;
        }

        seeker_data_valid = false;
      }
      break;
    }
  }

  if (sqrtf(Intercepe_Strike_Model_AS_U.SeekerData.trackBoxWidth *
            Intercepe_Strike_Model_AS_U.SeekerData.trackBoxWidth +
            Intercepe_Strike_Model_AS_U.SeekerData.trackBoxHeight *
            Intercepe_Strike_Model_AS_U.SeekerData.trackBoxHeight) >= 178.0F) {
    Intercepe_Strike_Model_AS_DW.durationCounter_1++;
  } else {
    Intercepe_Strike_Model_AS_DW.durationCounter_1 = 0U;
  }

  /* End of Chart: '<S2>/Chart' */

  /* SwitchCase: '<S1>/Switch Case' */
  rtPrevAction = Intercepe_Strike_Model_AS_DW.SwitchCase_ActiveSubsystem;
  Intercepe_Strike_Model_AS_DW.SwitchCase_ActiveSubsystem = -1;
  switch (Intercepe_Strike_Model_AS_B.guided_state) {
   case 1:
    Intercepe_Strike_Model_AS_DW.SwitchCase_ActiveSubsystem = 0;
    break;

   case 2:
    Intercepe_Strike_Model_AS_DW.SwitchCase_ActiveSubsystem = 1;
    break;

   case 3:
    Intercepe_Strike_Model_AS_DW.SwitchCase_ActiveSubsystem = 2;
    break;

   case 4:
    Intercepe_Strike_Model_AS_DW.SwitchCase_ActiveSubsystem = 3;
    break;

   case 5:
    Intercepe_Strike_Model_AS_DW.SwitchCase_ActiveSubsystem = 4;
    break;
  }

  switch (Intercepe_Strike_Model_AS_DW.SwitchCase_ActiveSubsystem) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S1>/Subsystem6' incorporates:
     *  ActionPort: '<S8>/Action Port'
     */
    Intercepe_Strike_Mod_Subsystem6(seeker_data_valid,
      &Intercepe_Strike_Model_AS_B.Merge,
      &Intercepe_Strike_Model_AS_DW.Subsystem6);

    /* End of Outputs for SubSystem: '<S1>/Subsystem6' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S1>/Subsystem1' incorporates:
     *  ActionPort: '<S4>/Action Port'
     */
    Intercepe_Strike_Mod_Subsystem6(seeker_data_valid,
      &Intercepe_Strike_Model_AS_B.Merge,
      &Intercepe_Strike_Model_AS_DW.Subsystem1);

    /* End of Outputs for SubSystem: '<S1>/Subsystem1' */
    break;

   case 2:
    if (Intercepe_Strike_Model_AS_DW.SwitchCase_ActiveSubsystem != rtPrevAction)
    {
      /* InitializeConditions for IfAction SubSystem: '<S1>/Subsystem2' incorporates:
       *  ActionPort: '<S5>/Action Port'
       */
      /* InitializeConditions for SwitchCase: '<S1>/Switch Case' incorporates:
       *  Delay: '<S5>/Delay1'
       *  Delay: '<S5>/Delay3'
       *  DiscreteIntegrator: '<S52>/Integrator'
       *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
       *  RateLimiter: '<S15>/Rate Limiter1'
       *  RateLimiter: '<S15>/Rate Limiter2'
       */
      Intercepe_Strike_Model_AS_DW.PrevY_a = 0.0F;
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_k = 0.0;
      Intercepe_Strike_Model_AS_DW.PrevY_g = 0.0F;
      Intercepe_Strike_Model_AS_DW.icLoad_bl = true;
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE = 0.0F;
      Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState = 2;
      Intercepe_Strike_Model_AS_DW.icLoad_p = true;

      /* End of InitializeConditions for SubSystem: '<S1>/Subsystem2' */

      /* SystemReset for IfAction SubSystem: '<S1>/Subsystem2' incorporates:
       *  ActionPort: '<S5>/Action Port'
       */
      /* SystemReset for SwitchCase: '<S1>/Switch Case' incorporates:
       *  MATLAB Function: '<S11>/MATLAB Function'
       *  MATLAB Function: '<S16>/MATLAB Function1'
       *  MATLAB Function: '<S16>/MATLAB Function2'
       *  MATLAB Function: '<S16>/MATLAB Function4'
       */
      Intercep_MATLABFunction_k_Reset
        (&Intercepe_Strike_Model_AS_DW.sf_MATLABFunction_f);
      Intercepe_MATLABFunction4_Reset
        (&Intercepe_Strike_Model_AS_DW.sf_MATLABFunction4);
      Interce_MATLABFunction1_j_Reset
        (&Intercepe_Strike_Model_AS_DW.sf_MATLABFunction2);
      Interce_MATLABFunction1_j_Reset
        (&Intercepe_Strike_Model_AS_DW.sf_MATLABFunction1_g);

      /* End of SystemReset for SubSystem: '<S1>/Subsystem2' */
    }

    /* Outputs for IfAction SubSystem: '<S1>/Subsystem2' incorporates:
     *  ActionPort: '<S5>/Action Port'
     */
    /* MATLAB Function: '<S14>/MATLAB Function' incorporates:
     *  Inport: '<Root>/current_pitch'
     *  Inport: '<Root>/current_roll'
     *  Inport: '<Root>/current_yaw'
     */
    Intercepe_Strike_MATLABFunction(Intercepe_Strike_Model_AS_U.current_yaw,
      Intercepe_Strike_Model_AS_U.current_pitch,
      Intercepe_Strike_Model_AS_U.current_roll, rtb_DCM_h);
    for (i = 0; i < 3; i++) {
      /* Product: '<S14>/MatrixMultiply' incorporates:
       *  Constant: '<S14>/Constant'
       */
      pooled11 = Intercepe_Strike_Model_A_ConstP.pooled11[3 * i + 1];
      rtb_Rollcmd_f = Intercepe_Strike_Model_A_ConstP.pooled11[3 * i];
      rtb_Saturation = Intercepe_Strike_Model_A_ConstP.pooled11[3 * i + 2];
      for (i_0 = 0; i_0 < 3; i_0++) {
        /* Math: '<S12>/Transpose1' incorporates:
         *  Product: '<S14>/MatrixMultiply'
         */
        rtb_Transpose1_p_tmp = 3 * i + i_0;
        rtb_Transpose1_b[rtb_Transpose1_p_tmp] = rtb_DCM_h[3 * i_0 + i];

        /* Product: '<S14>/MatrixMultiply' incorporates:
         *  Constant: '<S14>/Constant'
         *  Math: '<S14>/Transpose2'
         *  Product: '<S14>/MatrixMultiply2'
         */
        rtb_MatrixMultiply2_cf[rtb_Transpose1_p_tmp] = (rtb_DCM_h[3 * i_0 + 1] *
          pooled11 + rtb_DCM_h[3 * i_0] * rtb_Rollcmd_f) + rtb_DCM_h[3 * i_0 + 2]
          * rtb_Saturation;
      }
    }

    /* Gain: '<S14>/D2R' incorporates:
     *  Constant: '<S14>/phi_g'
     *  Inport: '<Root>/SeekerData'
     */
    rtb_D2R_i[0] = 0.0F;
    rtb_D2R_i[1] = 0.0174532924F *
      Intercepe_Strike_Model_AS_U.SeekerData.frameAnglePitch;
    rtb_D2R_i[2] = 0.0174532924F *
      Intercepe_Strike_Model_AS_U.SeekerData.frameAngleYaw;

    /* MATLAB Function: '<S14>/MATLAB Function1' */
    Intercepe_Strik_MATLABFunction1(rtb_D2R_i, rtb_DCM_h);
    for (i = 0; i < 3; i++) {
      /* Product: '<S14>/MatrixMultiply2' incorporates:
       *  Math: '<S14>/Transpose'
       */
      pooled11 = 0.0F;
      for (i_0 = 0; i_0 < 3; i_0++) {
        pooled11 += Intercepe_Strike_Model_A_ConstB.Transpose_g[3 * i + i_0] *
          rtb_DCM_h[3 * i_0];
      }

      /* Product: '<S11>/MatrixMultiply' incorporates:
       *  Product: '<S14>/MatrixMultiply2'
       */
      rtb_D2R_i[i] = 0.0F;
      rtb_D2R_i[0] += pooled11 * rtb_MatrixMultiply[i];
    }

    /* MATLAB Function: '<S11>/MATLAB Function' incorporates:
     *  Constant: '<S11>/Ts'
     *  Constant: '<S11>/cutoff frequency'
     */
    Intercepe_Stri_MATLABFunction_f(rtb_D2R_i[0], 2.0, 0.01, &rtb_desired_thrust,
      &Intercepe_Strike_Model_AS_DW.sf_MATLABFunction_f);

    /* Saturate: '<S11>/Saturation' */
    if (rtb_desired_thrust > 80.0F) {
      rtb_Rollcmd_f = 80.0F;
    } else if (rtb_desired_thrust < 3.0F) {
      rtb_Rollcmd_f = 3.0F;
    } else {
      rtb_Rollcmd_f = rtb_desired_thrust;
    }

    /* End of Saturate: '<S11>/Saturation' */
    for (i = 0; i < 3; i++) {
      /* Math: '<S14>/Transpose1' incorporates:
       *  Math: '<S12>/Transpose'
       */
      rtb_DCM[3 * i] = rtb_DCM_h[i];

      /* Product: '<S14>/MatrixMultiply1' incorporates:
       *  Math: '<S14>/Transpose1'
       */
      rtb_Transpose1_p_tmp = 3 * i + 1;

      /* Math: '<S14>/Transpose1' incorporates:
       *  Math: '<S12>/Transpose'
       */
      rtb_DCM[rtb_Transpose1_p_tmp] = rtb_DCM_h[i + 3];

      /* Product: '<S14>/MatrixMultiply1' incorporates:
       *  Math: '<S14>/Transpose1'
       */
      rtb_DCM_tmp = 3 * i + 2;

      /* Math: '<S14>/Transpose1' incorporates:
       *  Math: '<S12>/Transpose'
       */
      rtb_DCM[rtb_DCM_tmp] = rtb_DCM_h[i + 6];

      /* Product: '<S14>/MatrixMultiply1' incorporates:
       *  Math: '<S14>/Transpose1'
       *  Product: '<S14>/MatrixMultiply2'
       */
      rtb_Saturation = 0.0F;
      rtb_desired_thrust = 0.0F;
      rtb_lambda_dot_roll_los_filte_a = 0.0F;
      for (i_0 = 0; i_0 < 3; i_0++) {
        pooled11 = rtb_DCM[3 * i + i_0];
        rtb_Saturation += rtb_MatrixMultiply2_cf[3 * i_0] * pooled11;
        rtb_desired_thrust += rtb_MatrixMultiply2_cf[3 * i_0 + 1] * pooled11;
        rtb_lambda_dot_roll_los_filte_a += rtb_MatrixMultiply2_cf[3 * i_0 + 2] *
          pooled11;
      }

      rtb_MatrixMultiply2_fh[rtb_DCM_tmp] = rtb_lambda_dot_roll_los_filte_a;
      rtb_MatrixMultiply2_fh[rtb_Transpose1_p_tmp] = rtb_desired_thrust;
      rtb_MatrixMultiply2_fh[3 * i] = rtb_Saturation;
    }

    /* Trigonometry: '<S14>/Trigonometric Function' incorporates:
     *  Selector: '<S14>/Selector2'
     *  UnaryMinus: '<S14>/Unary Minus'
     */
    if (-rtb_MatrixMultiply2_fh[2] > 1.0F) {
      pooled11 = 1.0F;
    } else if (-rtb_MatrixMultiply2_fh[2] < -1.0F) {
      pooled11 = -1.0F;
    } else {
      pooled11 = -rtb_MatrixMultiply2_fh[2];
    }

    /* MATLAB Function: '<S16>/MATLAB Function4' incorporates:
     *  Constant: '<S16>/DYT frequency'
     *  Constant: '<S16>/cutoff frequency'
     *  Gain: '<S16>/D2R1'
     *  Inport: '<Root>/SeekerData'
     *  Product: '<S16>/Product'
     *  Trigonometry: '<S14>/Trigonometric Function'
     *  Trigonometry: '<S16>/Cos2'
     */
    Intercepe_Strik_MATLABFunction4(0.0174532924F *
      Intercepe_Strike_Model_AS_U.SeekerData.losRateYaw * cosf(asinf(pooled11)),
      1.0F, 0.02F, &rtb_desired_thrust,
      &Intercepe_Strike_Model_AS_DW.sf_MATLABFunction4);

    /* Product: '<S16>/Product2' incorporates:
     *  Gain: '<S16>/D2R2'
     *  Gain: '<S16>/D2R3'
     *  Inport: '<Root>/SeekerData'
     *  Trigonometry: '<S16>/Cos'
     *  Trigonometry: '<S16>/Cos1'
     */
    pooled11 = cosf(0.0174532924F *
                    Intercepe_Strike_Model_AS_U.SeekerData.misalignPitch) * cosf
      (0.0174532924F * Intercepe_Strike_Model_AS_U.SeekerData.misalignYaw);

    /* Saturate: '<S16>/Saturation1' */
    if (pooled11 < 0.9F) {
      pooled11 = 0.9F;
    }

    /* MATLAB Function: '<S16>/MATLAB Function2' incorporates:
     *  Constant: '<S16>/Constant2'
     *  Constant: '<S16>/Constant3'
     *  Saturate: '<S16>/Saturation1'
     */
    Intercepe_Str_MATLABFunction1_g(pooled11, 5.0F, 0.01F, &rtb_Saturation,
      &Intercepe_Strike_Model_AS_DW.sf_MATLABFunction2);

    /* Product: '<S16>/Divide' */
    rtb_lambda_dot_roll_los_filte_a = rtb_desired_thrust / rtb_Saturation;

    /* MATLAB Function: '<S16>/MATLAB Function1' incorporates:
     *  Constant: '<S16>/DYT frequency'
     *  Constant: '<S16>/cutoff frequency'
     *  Gain: '<S16>/D2R'
     *  Inport: '<Root>/SeekerData'
     */
    Intercepe_Str_MATLABFunction1_g(0.0174532924F *
      Intercepe_Strike_Model_AS_U.SeekerData.losRatePitch, 1.0F, 0.02F,
      &rtb_desired_thrust, &Intercepe_Strike_Model_AS_DW.sf_MATLABFunction1_g);

    /* SignalConversion generated from: '<S12>/Matrix Multiply' incorporates:
     *  Constant: '<S10>/Constant'
     */
    rtb_MatrixMultiply[0] = 3.0F;

    /* Product: '<S10>/Product' incorporates:
     *  Constant: '<S10>/Navigation Ratio'
     *  Product: '<S10>/Product1'
     *  UnaryMinus: '<S10>/Unary Minus'
     */
    pooled11 = 3.0F * rtb_Rollcmd_f;

    /* SignalConversion generated from: '<S12>/Matrix Multiply' incorporates:
     *  Product: '<S10>/Product'
     *  Product: '<S10>/Product1'
     *  Product: '<S16>/Divide1'
     *  UnaryMinus: '<S10>/Unary Minus'
     */
    rtb_MatrixMultiply[1] = pooled11 * rtb_lambda_dot_roll_los_filte_a;
    rtb_MatrixMultiply[2] = -(rtb_desired_thrust / rtb_Saturation * pooled11);

    /* Product: '<S12>/Matrix Multiply' incorporates:
     *  Math: '<S12>/Transpose'
     */
    rtb_Rollcmd_f = 0.0F;
    rtb_Saturation = 0.0F;
    rtb_desired_thrust = 0.0F;
    for (i = 0; i < 3; i++) {
      pooled11 = rtb_MatrixMultiply[i];
      rtb_Rollcmd_f += rtb_DCM[3 * i] * pooled11;
      rtb_Saturation += rtb_DCM[3 * i + 1] * pooled11;
      rtb_desired_thrust += rtb_DCM[3 * i + 2] * pooled11;
    }

    rtb_MatrixMultiply[2] = rtb_desired_thrust;
    rtb_MatrixMultiply[1] = rtb_Saturation;
    rtb_MatrixMultiply[0] = rtb_Rollcmd_f;

    /* End of Product: '<S12>/Matrix Multiply' */

    /* Product: '<S12>/Matrix Multiply2' incorporates:
     *  Constant: '<S14>/Constant'
     */
    rtb_Rollcmd_f = 0.0F;
    rtb_Saturation = 0.0F;
    rtb_desired_thrust = 0.0F;
    for (i = 0; i < 3; i++) {
      pooled11 = rtb_MatrixMultiply[i];
      rtb_Rollcmd_f += Intercepe_Strike_Model_A_ConstP.pooled11[3 * i] *
        pooled11;
      rtb_Saturation += Intercepe_Strike_Model_A_ConstP.pooled11[3 * i + 1] *
        pooled11;
      rtb_desired_thrust += Intercepe_Strike_Model_A_ConstP.pooled11[3 * i + 2] *
        pooled11;
    }

    /* End of Product: '<S12>/Matrix Multiply2' */

    /* Sum: '<S15>/Subtract' incorporates:
     *  Constant: '<S15>/Constant'
     *  Math: '<S12>/Transpose1'
     *  Product: '<S12>/Matrix Multiply1'
     */
    for (i = 0; i < 3; i++) {
      rtb_MatrixMultiply[i] = ((rtb_Transpose1_b[i + 3] * rtb_Saturation +
        rtb_Transpose1_b[i] * rtb_Rollcmd_f) + rtb_Transpose1_b[i + 6] *
        rtb_desired_thrust) - Intercepe_Strike_Model_A_ConstP.pooled12[i];
    }

    /* End of Sum: '<S15>/Subtract' */

    /* Sum: '<S15>/Add2' incorporates:
     *  Inport: '<Root>/current_yaw'
     *  Product: '<S15>/Product'
     *  Product: '<S15>/Product1'
     *  Trigonometry: '<S15>/Cos'
     *  Trigonometry: '<S15>/Cos1'
     */
    rtb_desired_thrust = rtb_MatrixMultiply[0] * cosf
      (Intercepe_Strike_Model_AS_U.current_yaw) + rtb_MatrixMultiply[1] * sinf
      (Intercepe_Strike_Model_AS_U.current_yaw);

    /* Trigonometry: '<S15>/Atan1' incorporates:
     *  Inport: '<Root>/current_yaw'
     *  Math: '<S15>/Square'
     *  Math: '<S15>/Square1'
     *  Product: '<S15>/Product2'
     *  Product: '<S15>/Product4'
     *  Sqrt: '<S15>/Sqrt'
     *  Sum: '<S15>/Add'
     *  Sum: '<S15>/Add3'
     *  Trigonometry: '<S15>/Cos2'
     *  Trigonometry: '<S15>/Cos3'
     *  UnaryMinus: '<S15>/Unary Minus'
     */
    rtb_Rollcmd_f = atan2f(-rtb_MatrixMultiply[0] * sinf
      (Intercepe_Strike_Model_AS_U.current_yaw) + rtb_MatrixMultiply[1] * cosf
      (Intercepe_Strike_Model_AS_U.current_yaw), sqrtf(rtb_desired_thrust *
      rtb_desired_thrust + rtb_MatrixMultiply[2] * rtb_MatrixMultiply[2]));

    /* Saturate: '<S15>/Saturation2' */
    if (rtb_Rollcmd_f > 0.87266463F) {
      rtb_Rollcmd_f = 0.87266463F;
    } else if (rtb_Rollcmd_f < -0.87266463F) {
      rtb_Rollcmd_f = -0.87266463F;
    }

    /* End of Saturate: '<S15>/Saturation2' */

    /* RateLimiter: '<S15>/Rate Limiter2' */
    pooled11 = rtb_Rollcmd_f - Intercepe_Strike_Model_AS_DW.PrevY_a;
    if (pooled11 > 0.04F) {
      rtb_Rollcmd_f = Intercepe_Strike_Model_AS_DW.PrevY_a + 0.04F;
    } else if (pooled11 < -0.04F) {
      rtb_Rollcmd_f = Intercepe_Strike_Model_AS_DW.PrevY_a - 0.04F;
    }

    Intercepe_Strike_Model_AS_DW.PrevY_a = rtb_Rollcmd_f;

    /* End of RateLimiter: '<S15>/Rate Limiter2' */

    /* Trigonometry: '<S15>/Atan2' incorporates:
     *  UnaryMinus: '<S15>/Unary Minus1'
     *  UnaryMinus: '<S15>/Unary Minus2'
     */
    rtb_desired_thrust = atan2f(-rtb_desired_thrust, -rtb_MatrixMultiply[2]);

    /* Saturate: '<S15>/Saturation1' */
    if (rtb_desired_thrust > 1.04719758F) {
      rtb_desired_thrust = 1.04719758F;
    } else if (rtb_desired_thrust < -1.04719758F) {
      rtb_desired_thrust = -1.04719758F;
    }

    /* End of Saturate: '<S15>/Saturation1' */

    /* RateLimiter: '<S15>/Rate Limiter1' */
    pooled11 = rtb_desired_thrust - Intercepe_Strike_Model_AS_DW.PrevY_g;
    if (pooled11 > 0.04F) {
      rtb_desired_thrust = Intercepe_Strike_Model_AS_DW.PrevY_g + 0.04F;
    } else if (pooled11 < -0.04F) {
      rtb_desired_thrust = Intercepe_Strike_Model_AS_DW.PrevY_g - 0.04F;
    }

    Intercepe_Strike_Model_AS_DW.PrevY_g = rtb_desired_thrust;

    /* End of RateLimiter: '<S15>/Rate Limiter1' */

    /* Delay: '<S5>/Delay1' incorporates:
     *  Inport: '<Root>/current_posd'
     */
    if (Intercepe_Strike_Model_AS_DW.icLoad_bl) {
      Intercepe_Strike_Model_AS_DW.Delay1_DSTATE_m =
        Intercepe_Strike_Model_AS_U.current_posd;
    }

    /* Sum: '<S5>/Sum1' incorporates:
     *  Delay: '<S5>/Delay1'
     *  Inport: '<Root>/current_posd'
     */
    pooled11 = Intercepe_Strike_Model_AS_DW.Delay1_DSTATE_m -
      Intercepe_Strike_Model_AS_U.current_posd;

    /* Saturate: '<S5>/Saturation2' */
    if (pooled11 > 2.0F) {
      pooled11 = 2.0F;
    } else if (pooled11 < -2.0F) {
      pooled11 = -2.0F;
    }

    /* Sum: '<S5>/Sum7' incorporates:
     *  Inport: '<Root>/current_vz'
     *  Saturate: '<S5>/Saturation2'
     */
    rtb_lambda_dot_roll_los_filte_a = pooled11 -
      Intercepe_Strike_Model_AS_U.current_vz;

    /* DiscreteIntegrator: '<S52>/Integrator' */
    if (((Intercepe_Strike_Model_AS_B.guided_state > 0) &&
         (Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState <= 0)) ||
        ((Intercepe_Strike_Model_AS_B.guided_state <= 0) &&
         (Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState == 1))) {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE = 0.0F;
    }

    /* Sum: '<S61>/Sum' incorporates:
     *  DiscreteIntegrator: '<S52>/Integrator'
     *  Gain: '<S57>/Proportional Gain'
     */
    rtb_Sum = 0.22F * rtb_lambda_dot_roll_los_filte_a +
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE;

    /* Saturate: '<S59>/Saturation' */
    if (rtb_Sum > 0.7F) {
      rtb_Saturation = 0.7F;
    } else if (rtb_Sum < -0.7F) {
      rtb_Saturation = -0.7F;
    } else {
      rtb_Saturation = rtb_Sum;
    }

    /* End of Saturate: '<S59>/Saturation' */

    /* Delay: '<S5>/Delay3' incorporates:
     *  Inport: '<Root>/current_thr'
     */
    if (Intercepe_Strike_Model_AS_DW.icLoad_p) {
      Intercepe_Strike_Model_AS_DW.Delay3_DSTATE_g =
        Intercepe_Strike_Model_AS_U.current_thr;
    }

    /* DataTypeConversion: '<S5>/Data Type Conversion' incorporates:
     *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
     */
    tmp = fmod(floor
               (Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_k),
               1.8446744073709552E+19);

    /* BusAssignment: '<S5>/Bus Assignment' incorporates:
     *  Constant: '<S5>/NAVIGATION_STATE_OFFBOARD'
     *  Constant: '<S5>/VEHICLE_CMD_SET_NAV_STATE'
     *  DataTypeConversion: '<S5>/Data Type Conversion'
     *  Inport: '<Root>/current_yaw'
     *  Merge: '<S1>/Merge'
     */
    Intercepe_Strike_Model_AS_B.Merge.timestamp = tmp < 0.0 ? (uint64_T)
      -(int64_T)(uint64_T)-tmp : (uint64_T)tmp;
    Intercepe_Strike_Model_AS_B.Merge.command = 100001U;
    Intercepe_Strike_Model_AS_B.Merge.param1 = 12.0F;
    Intercepe_Strike_Model_AS_B.Merge.param2 = rtb_Rollcmd_f;
    Intercepe_Strike_Model_AS_B.Merge.param3 = rtb_desired_thrust;
    Intercepe_Strike_Model_AS_B.Merge.param4 =
      Intercepe_Strike_Model_AS_U.current_yaw;

    /* Sum: '<S5>/Sum8' incorporates:
     *  Delay: '<S5>/Delay3'
     *  Gain: '<S5>/Gain7'
     */
    pooled11 = -rtb_Saturation + Intercepe_Strike_Model_AS_DW.Delay3_DSTATE_g;

    /* Saturate: '<S5>/Saturation5' */
    if (pooled11 > 0.7F) {
      /* BusAssignment: '<S5>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param5 = 0.7F;
    } else if (pooled11 < 0.05F) {
      /* BusAssignment: '<S5>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param5 = 0.05F;
    } else {
      /* BusAssignment: '<S5>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param5 = pooled11;
    }

    /* End of Saturate: '<S5>/Saturation5' */

    /* BusAssignment: '<S5>/Bus Assignment' incorporates:
     *  Constant: '<S5>/pass1'
     *  Constant: '<S5>/pass2'
     *  Merge: '<S1>/Merge'
     */
    Intercepe_Strike_Model_AS_B.Merge.param6 = 0.0F;
    Intercepe_Strike_Model_AS_B.Merge.param7 = 0.0F;
    Intercepe_Strike_Model_AS_B.Merge.cmd_valid = seeker_data_valid;

    /* Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S5>/Constant1'
     */
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_k += 10000.0;

    /* Update for Delay: '<S5>/Delay1' */
    Intercepe_Strike_Model_AS_DW.icLoad_bl = false;

    /* Update for DiscreteIntegrator: '<S52>/Integrator' incorporates:
     *  Gain: '<S49>/Integral Gain'
     *  Sum: '<S44>/SumI2'
     *  Sum: '<S44>/SumI4'
     */
    Intercepe_Strike_Model_AS_DW.Integrator_DSTATE += ((rtb_Saturation - rtb_Sum)
      + 0.1F * rtb_lambda_dot_roll_los_filte_a) * 0.01F;
    if (Intercepe_Strike_Model_AS_DW.Integrator_DSTATE > 0.3F) {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE = 0.3F;
    } else if (Intercepe_Strike_Model_AS_DW.Integrator_DSTATE < -0.3F) {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE = -0.3F;
    }

    Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState = (int8_T)
      (Intercepe_Strike_Model_AS_B.guided_state > 0);

    /* End of Update for DiscreteIntegrator: '<S52>/Integrator' */

    /* Update for Delay: '<S5>/Delay3' */
    Intercepe_Strike_Model_AS_DW.icLoad_p = false;

    /* End of Outputs for SubSystem: '<S1>/Subsystem2' */
    break;

   case 3:
    if (Intercepe_Strike_Model_AS_DW.SwitchCase_ActiveSubsystem != rtPrevAction)
    {
      /* InitializeConditions for IfAction SubSystem: '<S1>/Subsystem3' incorporates:
       *  ActionPort: '<S6>/Action Port'
       */
      /* InitializeConditions for SwitchCase: '<S1>/Switch Case' incorporates:
       *  Delay: '<S6>/Delay1'
       *  Delay: '<S6>/Delay2'
       *  Delay: '<S6>/Delay3'
       *  DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
       */
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_m = 0.0;
      Intercepe_Strike_Model_AS_DW.icLoad = true;
      Intercepe_Strike_Model_AS_DW.icLoad_b = true;
      Intercepe_Strike_Model_AS_DW.icLoad_bb = true;

      /* End of InitializeConditions for SubSystem: '<S1>/Subsystem3' */
    }

    /* Outputs for IfAction SubSystem: '<S1>/Subsystem3' incorporates:
     *  ActionPort: '<S6>/Action Port'
     */
    /* Delay: '<S6>/Delay1' incorporates:
     *  Inport: '<Root>/current_posn'
     */
    if (Intercepe_Strike_Model_AS_DW.icLoad) {
      Intercepe_Strike_Model_AS_DW.Delay1_DSTATE =
        Intercepe_Strike_Model_AS_U.current_posn;
    }

    /* Delay: '<S6>/Delay2' incorporates:
     *  Inport: '<Root>/current_pose'
     */
    if (Intercepe_Strike_Model_AS_DW.icLoad_b) {
      Intercepe_Strike_Model_AS_DW.Delay2_DSTATE =
        Intercepe_Strike_Model_AS_U.current_pose;
    }

    /* Delay: '<S6>/Delay3' incorporates:
     *  Inport: '<Root>/current_posd'
     */
    if (Intercepe_Strike_Model_AS_DW.icLoad_bb) {
      Intercepe_Strike_Model_AS_DW.Delay3_DSTATE =
        Intercepe_Strike_Model_AS_U.current_posd;
    }

    /* DataTypeConversion: '<S6>/Data Type Conversion' incorporates:
     *  DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
     */
    tmp = fmod(floor
               (Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_m),
               1.8446744073709552E+19);

    /* BusAssignment: '<S6>/Bus Assignment' incorporates:
     *  Constant: '<S6>/NAVIGATION_STATE_OFFBOARD'
     *  Constant: '<S6>/VEHICLE_CMD_SET_NAV_STATE'
     *  Constant: '<S6>/pass1'
     *  Constant: '<S6>/pass2'
     *  Constant: '<S6>/pass6'
     *  DataTypeConversion: '<S6>/Data Type Conversion'
     *  Delay: '<S6>/Delay1'
     *  Delay: '<S6>/Delay2'
     *  Delay: '<S6>/Delay3'
     *  Merge: '<S1>/Merge'
     */
    Intercepe_Strike_Model_AS_B.Merge.timestamp = tmp < 0.0 ? (uint64_T)
      -(int64_T)(uint64_T)-tmp : (uint64_T)tmp;
    Intercepe_Strike_Model_AS_B.Merge.command = 100001U;
    Intercepe_Strike_Model_AS_B.Merge.param1 = 4.0F;
    Intercepe_Strike_Model_AS_B.Merge.param2 = 0.0872664601F;
    Intercepe_Strike_Model_AS_B.Merge.param3 =
      Intercepe_Strike_Model_AS_DW.Delay1_DSTATE;
    Intercepe_Strike_Model_AS_B.Merge.param4 =
      Intercepe_Strike_Model_AS_DW.Delay2_DSTATE;
    Intercepe_Strike_Model_AS_B.Merge.param5 =
      Intercepe_Strike_Model_AS_DW.Delay3_DSTATE;
    Intercepe_Strike_Model_AS_B.Merge.param6 = 0.0F;
    Intercepe_Strike_Model_AS_B.Merge.param7 = 0.0F;
    Intercepe_Strike_Model_AS_B.Merge.cmd_valid = seeker_data_valid;

    /* Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S6>/Constant1'
     */
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_m += 10000.0;

    /* Update for Delay: '<S6>/Delay1' */
    Intercepe_Strike_Model_AS_DW.icLoad = false;

    /* Update for Delay: '<S6>/Delay2' */
    Intercepe_Strike_Model_AS_DW.icLoad_b = false;

    /* Update for Delay: '<S6>/Delay3' */
    Intercepe_Strike_Model_AS_DW.icLoad_bb = false;

    /* End of Outputs for SubSystem: '<S1>/Subsystem3' */
    break;

   case 4:
    /* Outputs for IfAction SubSystem: '<S1>/Subsystem4' incorporates:
     *  ActionPort: '<S7>/Action Port'
     */
    /* MATLAB Function: '<S78>/MATLAB Function' incorporates:
     *  Inport: '<Root>/current_pitch'
     *  Inport: '<Root>/current_roll'
     *  Inport: '<Root>/current_yaw'
     */
    Intercepe_Strike_MATLABFunction(Intercepe_Strike_Model_AS_U.current_yaw,
      Intercepe_Strike_Model_AS_U.current_pitch,
      Intercepe_Strike_Model_AS_U.current_roll, rtb_DCM_h);
    for (i = 0; i < 3; i++) {
      /* Product: '<S78>/MatrixMultiply' incorporates:
       *  Constant: '<S78>/Constant'
       */
      pooled11 = Intercepe_Strike_Model_A_ConstP.pooled11[3 * i + 1];
      rtb_Rollcmd_f = Intercepe_Strike_Model_A_ConstP.pooled11[3 * i];
      rtb_Saturation = Intercepe_Strike_Model_A_ConstP.pooled11[3 * i + 2];
      for (i_0 = 0; i_0 < 3; i_0++) {
        /* Math: '<S77>/Transpose1' incorporates:
         *  Product: '<S78>/MatrixMultiply'
         */
        rtb_Transpose1_p_tmp = 3 * i + i_0;
        rtb_Transpose1_b[rtb_Transpose1_p_tmp] = rtb_DCM_h[3 * i_0 + i];

        /* Product: '<S78>/MatrixMultiply' incorporates:
         *  Constant: '<S78>/Constant'
         *  Math: '<S78>/Transpose2'
         *  Product: '<S78>/MatrixMultiply2'
         */
        rtb_MatrixMultiply2_cf[rtb_Transpose1_p_tmp] = (rtb_DCM_h[3 * i_0 + 1] *
          pooled11 + rtb_DCM_h[3 * i_0] * rtb_Rollcmd_f) + rtb_DCM_h[3 * i_0 + 2]
          * rtb_Saturation;
      }
    }

    /* Gain: '<S78>/D2R' incorporates:
     *  Constant: '<S78>/phi_g'
     *  Inport: '<Root>/SeekerData'
     */
    rtb_D2R_i[0] = 0.0F;
    rtb_D2R_i[1] = 0.0174532924F *
      Intercepe_Strike_Model_AS_U.SeekerData.frameAnglePitch;
    rtb_D2R_i[2] = 0.0174532924F *
      Intercepe_Strike_Model_AS_U.SeekerData.frameAngleYaw;

    /* MATLAB Function: '<S78>/MATLAB Function1' */
    Intercepe_Strik_MATLABFunction1(rtb_D2R_i, rtb_DCM_h);
    for (i = 0; i < 3; i++) {
      /* Product: '<S78>/MatrixMultiply2' incorporates:
       *  Math: '<S78>/Transpose'
       */
      pooled11 = 0.0F;
      for (i_0 = 0; i_0 < 3; i_0++) {
        pooled11 += Intercepe_Strike_Model_A_ConstB.Transpose[3 * i + i_0] *
          rtb_DCM_h[3 * i_0];
      }

      /* Product: '<S76>/MatrixMultiply' incorporates:
       *  Product: '<S78>/MatrixMultiply2'
       */
      rtb_D2R_i[i] = 0.0F;
      rtb_D2R_i[0] += pooled11 * rtb_MatrixMultiply[i];
    }

    /* MATLAB Function: '<S76>/MATLAB Function' incorporates:
     *  Constant: '<S76>/Ts'
     *  Constant: '<S76>/cutoff frequency'
     */
    Intercepe_Stri_MATLABFunction_f(rtb_D2R_i[0], 2.0, 0.01, &rtb_Saturation,
      &Intercepe_Strike_Model_AS_DW.sf_MATLABFunction_h);

    /* Saturate: '<S76>/Saturation' */
    if (rtb_Saturation > 80.0F) {
      rtb_Rollcmd_f = 80.0F;
    } else if (rtb_Saturation < 3.0F) {
      rtb_Rollcmd_f = 3.0F;
    } else {
      rtb_Rollcmd_f = rtb_Saturation;
    }

    /* End of Saturate: '<S76>/Saturation' */
    for (i = 0; i < 3; i++) {
      /* Math: '<S78>/Transpose1' incorporates:
       *  Math: '<S77>/Transpose'
       */
      rtb_DCM[3 * i] = rtb_DCM_h[i];

      /* Product: '<S78>/MatrixMultiply1' incorporates:
       *  Math: '<S78>/Transpose1'
       */
      rtb_Transpose1_p_tmp = 3 * i + 1;

      /* Math: '<S78>/Transpose1' incorporates:
       *  Math: '<S77>/Transpose'
       */
      rtb_DCM[rtb_Transpose1_p_tmp] = rtb_DCM_h[i + 3];

      /* Product: '<S78>/MatrixMultiply1' incorporates:
       *  Math: '<S78>/Transpose1'
       */
      rtb_DCM_tmp = 3 * i + 2;

      /* Math: '<S78>/Transpose1' incorporates:
       *  Math: '<S77>/Transpose'
       */
      rtb_DCM[rtb_DCM_tmp] = rtb_DCM_h[i + 6];

      /* Product: '<S78>/MatrixMultiply1' incorporates:
       *  Math: '<S78>/Transpose1'
       *  Product: '<S78>/MatrixMultiply2'
       */
      rtb_Saturation = 0.0F;
      rtb_desired_thrust = 0.0F;
      rtb_lambda_dot_roll_los_filte_a = 0.0F;
      for (i_0 = 0; i_0 < 3; i_0++) {
        pooled11 = rtb_DCM[3 * i + i_0];
        rtb_Saturation += rtb_MatrixMultiply2_cf[3 * i_0] * pooled11;
        rtb_desired_thrust += rtb_MatrixMultiply2_cf[3 * i_0 + 1] * pooled11;
        rtb_lambda_dot_roll_los_filte_a += rtb_MatrixMultiply2_cf[3 * i_0 + 2] *
          pooled11;
      }

      rtb_MatrixMultiply2_fh[rtb_DCM_tmp] = rtb_lambda_dot_roll_los_filte_a;
      rtb_MatrixMultiply2_fh[rtb_Transpose1_p_tmp] = rtb_desired_thrust;
      rtb_MatrixMultiply2_fh[3 * i] = rtb_Saturation;
    }

    /* Trigonometry: '<S78>/Trigonometric Function' incorporates:
     *  Selector: '<S78>/Selector2'
     *  UnaryMinus: '<S78>/Unary Minus'
     */
    if (-rtb_MatrixMultiply2_fh[2] > 1.0F) {
      pooled11 = 1.0F;
    } else if (-rtb_MatrixMultiply2_fh[2] < -1.0F) {
      pooled11 = -1.0F;
    } else {
      pooled11 = -rtb_MatrixMultiply2_fh[2];
    }

    /* MATLAB Function: '<S80>/MATLAB Function4' incorporates:
     *  Constant: '<S80>/DYT frequency'
     *  Constant: '<S80>/cutoff frequency'
     *  Gain: '<S80>/D2R1'
     *  Inport: '<Root>/SeekerData'
     *  Product: '<S80>/Product'
     *  Trigonometry: '<S78>/Trigonometric Function'
     *  Trigonometry: '<S80>/Cos2'
     */
    Intercepe_Strik_MATLABFunction4(0.0174532924F *
      Intercepe_Strike_Model_AS_U.SeekerData.losRateYaw * cosf(asinf(pooled11)),
      1.0F, 0.02F, &rtb_Saturation,
      &Intercepe_Strike_Model_AS_DW.sf_MATLABFunction4_h);

    /* Product: '<S80>/Product2' incorporates:
     *  Gain: '<S80>/D2R2'
     *  Gain: '<S80>/D2R3'
     *  Inport: '<Root>/SeekerData'
     *  Trigonometry: '<S80>/Cos'
     *  Trigonometry: '<S80>/Cos1'
     */
    pooled11 = cosf(0.0174532924F *
                    Intercepe_Strike_Model_AS_U.SeekerData.misalignPitch) * cosf
      (0.0174532924F * Intercepe_Strike_Model_AS_U.SeekerData.misalignYaw);

    /* Saturate: '<S80>/Saturation1' */
    if (pooled11 < 0.9F) {
      pooled11 = 0.9F;
    }

    /* MATLAB Function: '<S80>/MATLAB Function2' incorporates:
     *  Constant: '<S80>/Constant2'
     *  Constant: '<S80>/Constant3'
     *  Saturate: '<S80>/Saturation1'
     */
    Intercepe_Str_MATLABFunction1_g(pooled11, 5.0F, 0.01F, &rtb_desired_thrust,
      &Intercepe_Strike_Model_AS_DW.sf_MATLABFunction2_k);

    /* Product: '<S80>/Divide' */
    rtb_lambda_dot_roll_los_filte_a = rtb_Saturation / rtb_desired_thrust;

    /* MATLAB Function: '<S80>/MATLAB Function1' incorporates:
     *  Constant: '<S80>/DYT frequency'
     *  Constant: '<S80>/cutoff frequency'
     *  Gain: '<S80>/D2R'
     *  Inport: '<Root>/SeekerData'
     */
    Intercepe_Str_MATLABFunction1_g(0.0174532924F *
      Intercepe_Strike_Model_AS_U.SeekerData.losRatePitch, 1.0F, 0.02F,
      &rtb_Saturation, &Intercepe_Strike_Model_AS_DW.sf_MATLABFunction1_gf);

    /* SignalConversion generated from: '<S77>/Matrix Multiply' incorporates:
     *  Constant: '<S75>/Constant'
     */
    rtb_MatrixMultiply[0] = 3.0F;

    /* Product: '<S75>/Product' incorporates:
     *  Constant: '<S75>/Navigation Ratio'
     *  Product: '<S75>/Product1'
     *  UnaryMinus: '<S75>/Unary Minus'
     */
    pooled11 = 3.0F * rtb_Rollcmd_f;

    /* SignalConversion generated from: '<S77>/Matrix Multiply' incorporates:
     *  Product: '<S75>/Product'
     *  Product: '<S75>/Product1'
     *  Product: '<S80>/Divide1'
     *  UnaryMinus: '<S75>/Unary Minus'
     */
    rtb_MatrixMultiply[1] = pooled11 * rtb_lambda_dot_roll_los_filte_a;
    rtb_MatrixMultiply[2] = -(rtb_Saturation / rtb_desired_thrust * pooled11);

    /* Product: '<S77>/Matrix Multiply' incorporates:
     *  Math: '<S77>/Transpose'
     */
    rtb_Rollcmd_f = 0.0F;
    rtb_Saturation = 0.0F;
    rtb_desired_thrust = 0.0F;
    for (i = 0; i < 3; i++) {
      pooled11 = rtb_MatrixMultiply[i];
      rtb_Rollcmd_f += rtb_DCM[3 * i] * pooled11;
      rtb_Saturation += rtb_DCM[3 * i + 1] * pooled11;
      rtb_desired_thrust += rtb_DCM[3 * i + 2] * pooled11;
    }

    rtb_MatrixMultiply[2] = rtb_desired_thrust;
    rtb_MatrixMultiply[1] = rtb_Saturation;
    rtb_MatrixMultiply[0] = rtb_Rollcmd_f;

    /* End of Product: '<S77>/Matrix Multiply' */

    /* Product: '<S77>/Matrix Multiply2' incorporates:
     *  Constant: '<S78>/Constant'
     */
    rtb_Rollcmd_f = 0.0F;
    rtb_Saturation = 0.0F;
    rtb_desired_thrust = 0.0F;
    for (i = 0; i < 3; i++) {
      pooled11 = rtb_MatrixMultiply[i];
      rtb_Rollcmd_f += Intercepe_Strike_Model_A_ConstP.pooled11[3 * i] *
        pooled11;
      rtb_Saturation += Intercepe_Strike_Model_A_ConstP.pooled11[3 * i + 1] *
        pooled11;
      rtb_desired_thrust += Intercepe_Strike_Model_A_ConstP.pooled11[3 * i + 2] *
        pooled11;
    }

    /* End of Product: '<S77>/Matrix Multiply2' */

    /* Sum: '<S79>/Subtract' incorporates:
     *  Constant: '<S79>/Constant'
     *  Math: '<S77>/Transpose1'
     *  Product: '<S77>/Matrix Multiply1'
     */
    for (i = 0; i < 3; i++) {
      rtb_MatrixMultiply[i] = ((rtb_Transpose1_b[i + 3] * rtb_Saturation +
        rtb_Transpose1_b[i] * rtb_Rollcmd_f) + rtb_Transpose1_b[i + 6] *
        rtb_desired_thrust) - Intercepe_Strike_Model_A_ConstP.pooled12[i];
    }

    /* End of Sum: '<S79>/Subtract' */

    /* Sum: '<S79>/Add2' incorporates:
     *  Inport: '<Root>/current_yaw'
     *  Product: '<S79>/Product'
     *  Product: '<S79>/Product1'
     *  Trigonometry: '<S79>/Cos'
     *  Trigonometry: '<S79>/Cos1'
     */
    rtb_Saturation = rtb_MatrixMultiply[0] * cosf
      (Intercepe_Strike_Model_AS_U.current_yaw) + rtb_MatrixMultiply[1] * sinf
      (Intercepe_Strike_Model_AS_U.current_yaw);

    /* Math: '<S79>/Square1' incorporates:
     *  Math: '<S79>/Square4'
     */
    rtb_desired_thrust = rtb_MatrixMultiply[2] * rtb_MatrixMultiply[2];

    /* Trigonometry: '<S79>/Atan1' incorporates:
     *  Inport: '<Root>/current_yaw'
     *  Math: '<S79>/Square'
     *  Math: '<S79>/Square1'
     *  Product: '<S79>/Product2'
     *  Product: '<S79>/Product4'
     *  Sqrt: '<S79>/Sqrt'
     *  Sum: '<S79>/Add'
     *  Sum: '<S79>/Add3'
     *  Trigonometry: '<S79>/Cos2'
     *  Trigonometry: '<S79>/Cos3'
     *  UnaryMinus: '<S79>/Unary Minus'
     */
    rtb_Rollcmd_f = atan2f(-rtb_MatrixMultiply[0] * sinf
      (Intercepe_Strike_Model_AS_U.current_yaw) + rtb_MatrixMultiply[1] * cosf
      (Intercepe_Strike_Model_AS_U.current_yaw), sqrtf(rtb_Saturation *
      rtb_Saturation + rtb_desired_thrust));

    /* Saturate: '<S79>/Saturation2' */
    if (rtb_Rollcmd_f > 0.87266463F) {
      rtb_Rollcmd_f = 0.87266463F;
    } else if (rtb_Rollcmd_f < -0.87266463F) {
      rtb_Rollcmd_f = -0.87266463F;
    }

    /* End of Saturate: '<S79>/Saturation2' */

    /* RateLimiter: '<S79>/Rate Limiter2' */
    pooled11 = rtb_Rollcmd_f - Intercepe_Strike_Model_AS_DW.PrevY;
    if (pooled11 > 0.04F) {
      rtb_Rollcmd_f = Intercepe_Strike_Model_AS_DW.PrevY + 0.04F;
    } else if (pooled11 < -0.04F) {
      rtb_Rollcmd_f = Intercepe_Strike_Model_AS_DW.PrevY - 0.04F;
    }

    Intercepe_Strike_Model_AS_DW.PrevY = rtb_Rollcmd_f;

    /* End of RateLimiter: '<S79>/Rate Limiter2' */

    /* Trigonometry: '<S79>/Atan2' incorporates:
     *  UnaryMinus: '<S79>/Unary Minus1'
     *  UnaryMinus: '<S79>/Unary Minus2'
     */
    rtb_Saturation = atan2f(-rtb_Saturation, -rtb_MatrixMultiply[2]);

    /* Saturate: '<S79>/Saturation1' */
    if (rtb_Saturation > 1.04719758F) {
      rtb_Saturation = 1.04719758F;
    } else if (rtb_Saturation < -1.04719758F) {
      rtb_Saturation = -1.04719758F;
    }

    /* End of Saturate: '<S79>/Saturation1' */

    /* RateLimiter: '<S79>/Rate Limiter1' */
    pooled11 = rtb_Saturation - Intercepe_Strike_Model_AS_DW.PrevY_l;
    if (pooled11 > 0.04F) {
      rtb_Saturation = Intercepe_Strike_Model_AS_DW.PrevY_l + 0.04F;
    } else if (pooled11 < -0.04F) {
      rtb_Saturation = Intercepe_Strike_Model_AS_DW.PrevY_l - 0.04F;
    }

    Intercepe_Strike_Model_AS_DW.PrevY_l = rtb_Saturation;

    /* End of RateLimiter: '<S79>/Rate Limiter1' */

    /* DataTypeConversion: '<S7>/Data Type Conversion' incorporates:
     *  DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
     */
    tmp = fmod(floor(Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE),
               1.8446744073709552E+19);

    /* BusAssignment: '<S7>/Bus Assignment' incorporates:
     *  Constant: '<S79>/mass'
     *  Constant: '<S7>/NAVIGATION_STATE_OFFBOARD'
     *  Constant: '<S7>/VEHICLE_CMD_SET_NAV_STATE'
     *  Constant: '<S7>/pass1'
     *  Constant: '<S7>/pass2'
     *  DataTypeConversion: '<S7>/Data Type Conversion'
     *  Gain: '<S79>/Gain'
     *  Inport: '<Root>/current_yaw'
     *  Math: '<S79>/Square2'
     *  Math: '<S79>/Square3'
     *  Merge: '<S1>/Merge'
     *  Product: '<S79>/Product3'
     *  Sqrt: '<S79>/Sqrt1'
     *  Sum: '<S79>/Add1'
     */
    Intercepe_Strike_Model_AS_B.Merge.timestamp = tmp < 0.0 ? (uint64_T)
      -(int64_T)(uint64_T)-tmp : (uint64_T)tmp;
    Intercepe_Strike_Model_AS_B.Merge.command = 100001U;
    Intercepe_Strike_Model_AS_B.Merge.param1 = 12.0F;
    Intercepe_Strike_Model_AS_B.Merge.param2 = rtb_Rollcmd_f;
    Intercepe_Strike_Model_AS_B.Merge.param3 = rtb_Saturation;
    Intercepe_Strike_Model_AS_B.Merge.param4 =
      Intercepe_Strike_Model_AS_U.current_yaw;
    Intercepe_Strike_Model_AS_B.Merge.param5 = sqrtf((rtb_MatrixMultiply[0] *
      rtb_MatrixMultiply[0] + rtb_MatrixMultiply[1] * rtb_MatrixMultiply[1]) +
      rtb_desired_thrust) * 5.0F * 0.0036764706F;
    Intercepe_Strike_Model_AS_B.Merge.param6 = 0.0F;
    Intercepe_Strike_Model_AS_B.Merge.param7 = 0.0F;
    Intercepe_Strike_Model_AS_B.Merge.cmd_valid = seeker_data_valid;

    /* Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S7>/Constant1'
     */
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE += 10000.0;

    /* End of Outputs for SubSystem: '<S1>/Subsystem4' */
    break;
  }

  /* End of SwitchCase: '<S1>/Switch Case' */

  /* Outport: '<Root>/VehicleCommand' incorporates:
   *  Merge: '<S1>/Merge'
   */
  Intercepe_Strike_Model_AS_Y.VehicleCommand = Intercepe_Strike_Model_AS_B.Merge;
}

/* Model initialize function */
void Intercepe_Strike_Model_AS_initialize(void)
{
  /* Registration code */

  /* block I/O */
  (void) memset(((void *) &Intercepe_Strike_Model_AS_B), 0,
                sizeof(B_Intercepe_Strike_Model_AS_T));

  /* states (dwork) */
  (void) memset((void *)&Intercepe_Strike_Model_AS_DW, 0,
                sizeof(DW_Intercepe_Strike_Model_AS_T));

  /* external inputs */
  (void)memset(&Intercepe_Strike_Model_AS_U, 0, sizeof
               (ExtU_Intercepe_Strike_Model_A_T));

  /* external outputs */
  Intercepe_Strike_Model_AS_Y.VehicleCommand = Intercepe_Strike_Model_AS_rtZVe;

  /* Start for SwitchCase: '<S1>/Switch Case' */
  Intercepe_Strike_Model_AS_DW.SwitchCase_ActiveSubsystem = -1;

  /* SystemInitialize for Chart: '<S2>/Chart' */
  Intercepe_Strike_Model_AS_DW.last_timestamp = 0ULL;
  Intercepe_Strike_Model_AS_DW.time_gate = 0.0F;
  Intercepe_Strike_Model_AS_DW.lock_time = 0.0F;
  Intercepe_Strike_Model_AS_B.guided_state = 0U;
  Intercepe_Strike_Model_AS_DW.stale_count = 3U;
  Intercepe_Strike_Model_AS_DW.is_active_c3_Intercepe_Strike_M = 0U;
  Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS =
    Intercepe_St_IN_NO_ACTIVE_CHILD;

  /* SystemInitialize for IfAction SubSystem: '<S1>/Subsystem2' */
  /* InitializeConditions for RateLimiter: '<S15>/Rate Limiter2' */
  Intercepe_Strike_Model_AS_DW.PrevY_a = 0.0F;

  /* InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' */
  Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_k = 0.0;

  /* InitializeConditions for RateLimiter: '<S15>/Rate Limiter1' */
  Intercepe_Strike_Model_AS_DW.PrevY_g = 0.0F;

  /* InitializeConditions for Delay: '<S5>/Delay1' */
  Intercepe_Strike_Model_AS_DW.icLoad_bl = true;

  /* InitializeConditions for DiscreteIntegrator: '<S52>/Integrator' */
  Intercepe_Strike_Model_AS_DW.Integrator_DSTATE = 0.0F;
  Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState = 2;

  /* InitializeConditions for Delay: '<S5>/Delay3' */
  Intercepe_Strike_Model_AS_DW.icLoad_p = true;

  /* SystemInitialize for MATLAB Function: '<S11>/MATLAB Function' */
  Intercepe_MATLABFunction_h_Init
    (&Intercepe_Strike_Model_AS_DW.sf_MATLABFunction_f);

  /* SystemInitialize for MATLAB Function: '<S16>/MATLAB Function4' */
  Intercepe__MATLABFunction4_Init
    (&Intercepe_Strike_Model_AS_DW.sf_MATLABFunction4);

  /* SystemInitialize for MATLAB Function: '<S16>/MATLAB Function2' */
  Intercep_MATLABFunction1_a_Init
    (&Intercepe_Strike_Model_AS_DW.sf_MATLABFunction2);

  /* SystemInitialize for MATLAB Function: '<S16>/MATLAB Function1' */
  Intercep_MATLABFunction1_a_Init
    (&Intercepe_Strike_Model_AS_DW.sf_MATLABFunction1_g);

  /* End of SystemInitialize for SubSystem: '<S1>/Subsystem2' */

  /* SystemInitialize for IfAction SubSystem: '<S1>/Subsystem3' */
  /* InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator' */
  Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_m = 0.0;

  /* InitializeConditions for Delay: '<S6>/Delay1' */
  Intercepe_Strike_Model_AS_DW.icLoad = true;

  /* InitializeConditions for Delay: '<S6>/Delay2' */
  Intercepe_Strike_Model_AS_DW.icLoad_b = true;

  /* InitializeConditions for Delay: '<S6>/Delay3' */
  Intercepe_Strike_Model_AS_DW.icLoad_bb = true;

  /* End of SystemInitialize for SubSystem: '<S1>/Subsystem3' */

  /* SystemInitialize for IfAction SubSystem: '<S1>/Subsystem4' */
  /* InitializeConditions for RateLimiter: '<S79>/Rate Limiter2' */
  Intercepe_Strike_Model_AS_DW.PrevY = 0.0F;

  /* InitializeConditions for RateLimiter: '<S79>/Rate Limiter1' */
  Intercepe_Strike_Model_AS_DW.PrevY_l = 0.0F;

  /* SystemInitialize for MATLAB Function: '<S76>/MATLAB Function' */
  Intercepe_MATLABFunction_h_Init
    (&Intercepe_Strike_Model_AS_DW.sf_MATLABFunction_h);

  /* SystemInitialize for MATLAB Function: '<S80>/MATLAB Function4' */
  Intercepe__MATLABFunction4_Init
    (&Intercepe_Strike_Model_AS_DW.sf_MATLABFunction4_h);

  /* SystemInitialize for MATLAB Function: '<S80>/MATLAB Function2' */
  Intercep_MATLABFunction1_a_Init
    (&Intercepe_Strike_Model_AS_DW.sf_MATLABFunction2_k);

  /* SystemInitialize for MATLAB Function: '<S80>/MATLAB Function1' */
  Intercep_MATLABFunction1_a_Init
    (&Intercepe_Strike_Model_AS_DW.sf_MATLABFunction1_gf);

  /* End of SystemInitialize for SubSystem: '<S1>/Subsystem4' */

  /* SystemInitialize for Merge: '<S1>/Merge' */
  memset(&Intercepe_Strike_Model_AS_B.Merge, 0, sizeof(VehicleCommandBus));
}

/* Model terminate function */
void Intercepe_Strike_Model_AS_terminate(void)
{
  /* (no terminate code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
