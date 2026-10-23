/*
 * File: Intercepe_Strike_Model_AS.c
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
  10.0F
} ;                                    /* Variable: STRIKE_PARAM
                                        * Referenced by:
                                        *   '<S46>/Integral Gain'
                                        *   '<S54>/Proportional Gain'
                                        *   '<S98>/Integral Gain'
                                        *   '<S106>/Proportional Gain'
                                        *   '<S212>/Integral Gain'
                                        *   '<S220>/Proportional Gain'
                                        *   '<S264>/Integral Gain'
                                        *   '<S272>/Proportional Gain'
                                        * 打击模式控制器参数结构体，支持运行时调参
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
 * Output and update for action system:
 *    '<S1>/Subsystem6'
 *    '<S1>/Subsystem1'
 */
void Intercepe_Strike_Mod_Subsystem6(boolean_T rtu_cmd_valid, VehicleCommandBus *
  rty_Out1, DW_Subsystem6_Intercepe_Strik_T *localDW)
{
  real_T tmp;

  /* DataTypeConversion: '<S7>/Data Type Conversion' incorporates:
   *  DiscreteIntegrator: '<S7>/Discrete-Time Integrator'
   */
  tmp = fmod(floor(localDW->DiscreteTimeIntegrator_DSTATE),
             1.8446744073709552E+19);

  /* BusAssignment: '<S7>/Bus Assignment' incorporates:
   *  Constant: '<S7>/Constant4'
   *  Constant: '<S7>/Constant5'
   *  Constant: '<S7>/Constant6'
   *  Constant: '<S7>/Constant7'
   *  Constant: '<S7>/Constant8'
   *  Constant: '<S7>/Constant9'
   *  Constant: '<S7>/NAVIGATION_STATE_AUTO_LOITER'
   *  Constant: '<S7>/VEHICLE_CMD_SET_NAV_STATE'
   *  DataTypeConversion: '<S7>/Data Type Conversion'
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

  /* Update for DiscreteIntegrator: '<S7>/Discrete-Time Integrator' incorporates:
   *  Constant: '<S7>/Constant1'
   */
  localDW->DiscreteTimeIntegrator_DSTATE += 10000.0;
}

/* Function for Chart: '<S2>/Chart' */
static void Intercepe_Strike_Model_AS_LOCK(boolean_T *cmd_valid, const
  SeekerGuidanceBus *BusConversion_InsertedFor_Chart)
{
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
    /* BusCreator generated from: '<S2>/Chart' */
    seeker_data_valid = ((BusConversion_InsertedFor_Chart->timestamp !=
                          Intercepe_Strike_Model_AS_DW.last_timestamp) &&
                         (BusConversion_InsertedFor_Chart->timestamp > 0ULL));
    Intercepe_Strike_Model_AS_DW.last_timestamp =
      BusConversion_InsertedFor_Chart->timestamp;
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
  real32_T rtb_Gain1_a;
  real32_T rtb_Gain1_e;
  real32_T rtb_IntegralGain_hj;
  real32_T rtb_IntegralGain_hr;
  real32_T rtb_Saturation_bm;
  real32_T rtb_Saturation_d;
  real32_T rtb_Saturation_k;
  real32_T rtb_Sum7;
  real32_T rtb_Sum_a;
  real32_T rtb_Sum_c;
  real32_T rtb_Sum_k;
  int8_T rtPrevAction;
  boolean_T cmd_valid;

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
    cmd_valid = true;
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
        cmd_valid = true;
      } else {
        cmd_valid = false;
      }
      break;

     case Intercepe_Strike_Model__IN_LOCK:
      Intercepe_Strike_Model_AS_LOCK(&cmd_valid,
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
        cmd_valid = true;
      } else if ((!Intercepe_Strike_Model_AS_U.bool_search) ||
                 (Intercepe_Strike_Model_AS_U.arming_state != 2)) {
        Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS =
          Intercepe_Strike_Model__IN_IDLE;
        Intercepe_Strike_Model_AS_B.guided_state = 1U;
        cmd_valid = true;
      } else {
        cmd_valid = ((Intercepe_Strike_Model_AS_U.SeekerData.timestamp !=
                      Intercepe_Strike_Model_AS_DW.last_timestamp) &&
                     (Intercepe_Strike_Model_AS_U.SeekerData.timestamp > 0ULL));
        Intercepe_Strike_Model_AS_DW.last_timestamp =
          Intercepe_Strike_Model_AS_U.SeekerData.timestamp;
        if (cmd_valid && (((Intercepe_Strike_Model_AS_U.SeekerData.workStatus &
                            1U) != 0U) &&
                          (Intercepe_Strike_Model_AS_U.SeekerData.trackConfidence
                           > 6.0F) &&
                          (Intercepe_Strike_Model_AS_U.SeekerData.selfCheckResult
                           == 1))) {
          Intercepe_Strike_Model_AS_DW.time_gate += 0.01F;
        } else {
          Intercepe_Strike_Model_AS_DW.time_gate = 0.0F;
        }

        cmd_valid = false;
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
        cmd_valid = true;
      } else if ((!Intercepe_Strike_Model_AS_U.bool_search) ||
                 (Intercepe_Strike_Model_AS_U.arming_state != 2) ||
                 (Intercepe_Strike_Model_AS_U.SeekerData.selfCheckResult != 1))
      {
        Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS =
          Intercepe_Strike_Model__IN_IDLE;
        Intercepe_Strike_Model_AS_B.guided_state = 1U;
        cmd_valid = true;
      } else {
        cmd_valid = ((Intercepe_Strike_Model_AS_U.SeekerData.timestamp !=
                      Intercepe_Strike_Model_AS_DW.last_timestamp) &&
                     (Intercepe_Strike_Model_AS_U.SeekerData.timestamp > 0ULL));
        Intercepe_Strike_Model_AS_DW.last_timestamp =
          Intercepe_Strike_Model_AS_U.SeekerData.timestamp;
        if (cmd_valid && (((Intercepe_Strike_Model_AS_U.SeekerData.workStatus &
                            1U) != 0U) &&
                          (Intercepe_Strike_Model_AS_U.SeekerData.trackConfidence
                           > 6.0F) &&
                          (Intercepe_Strike_Model_AS_U.SeekerData.selfCheckResult
                           == 1))) {
          Intercepe_Strike_Model_AS_DW.time_gate += 0.01F;
        } else {
          Intercepe_Strike_Model_AS_DW.time_gate = 0.0F;
        }

        cmd_valid = false;
      }
      break;

     default:
      /* case IN_STRIKE: */
      Intercepe_Strike_Model_AS_B.guided_state = 5U;
      if ((!Intercepe_Strike_Model_AS_U.bool_request) ||
          (Intercepe_Strike_Model_AS_DW.time_gate >= 0.5F)) {
        Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS =
          Intercepe_Strike_Model__IN_IDLE;
        Intercepe_Strike_Model_AS_B.guided_state = 1U;
        cmd_valid = true;
      } else {
        cmd_valid = ((Intercepe_Strike_Model_AS_U.SeekerData.timestamp !=
                      Intercepe_Strike_Model_AS_DW.last_timestamp) &&
                     (Intercepe_Strike_Model_AS_U.SeekerData.timestamp > 0ULL));
        Intercepe_Strike_Model_AS_DW.last_timestamp =
          Intercepe_Strike_Model_AS_U.SeekerData.timestamp;
        if ((!cmd_valid) || (((Intercepe_Strike_Model_AS_U.SeekerData.workStatus
               & 1U) == 0U) ||
                             (Intercepe_Strike_Model_AS_U.SeekerData.trackConfidence
                              <= 6.0F) ||
                             (Intercepe_Strike_Model_AS_U.SeekerData.selfCheckResult
                              != 1))) {
          Intercepe_Strike_Model_AS_DW.time_gate += 0.01F;
        } else {
          Intercepe_Strike_Model_AS_DW.time_gate = 0.0F;
        }

        cmd_valid = false;
      }
      break;
    }
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
     *  ActionPort: '<S7>/Action Port'
     */
    Intercepe_Strike_Mod_Subsystem6(cmd_valid,
      &Intercepe_Strike_Model_AS_B.Merge,
      &Intercepe_Strike_Model_AS_DW.Subsystem6);

    /* End of Outputs for SubSystem: '<S1>/Subsystem6' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S1>/Subsystem1' incorporates:
     *  ActionPort: '<S3>/Action Port'
     */
    Intercepe_Strike_Mod_Subsystem6(cmd_valid,
      &Intercepe_Strike_Model_AS_B.Merge,
      &Intercepe_Strike_Model_AS_DW.Subsystem1);

    /* End of Outputs for SubSystem: '<S1>/Subsystem1' */
    break;

   case 2:
    if (Intercepe_Strike_Model_AS_DW.SwitchCase_ActiveSubsystem != rtPrevAction)
    {
      /* InitializeConditions for IfAction SubSystem: '<S1>/Subsystem2' incorporates:
       *  ActionPort: '<S4>/Action Port'
       */
      /* InitializeConditions for SwitchCase: '<S1>/Switch Case' incorporates:
       *  Delay: '<S4>/Delay1'
       *  Delay: '<S4>/Delay3'
       *  DiscreteIntegrator: '<S101>/Integrator'
       *  DiscreteIntegrator: '<S10>/Discrete-Time Integrator5'
       *  DiscreteIntegrator: '<S11>/Discrete-Time Integrator5'
       *  DiscreteIntegrator: '<S153>/Integrator'
       *  DiscreteIntegrator: '<S49>/Integrator'
       *  DiscreteIntegrator: '<S4>/Discrete-Time Integrator'
       */
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_k = 0.0;
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LO_o = 1U;
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_ho = 0.0F;
      Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_d = 2;
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_L_ll = 1U;
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_e = 0.0F;
      Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_m = 2;
      Intercepe_Strike_Model_AS_DW.icLoad_bl = true;
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_g = 0.0F;
      Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_h = 2;
      Intercepe_Strike_Model_AS_DW.icLoad_p = true;

      /* End of InitializeConditions for SubSystem: '<S1>/Subsystem2' */
    }

    /* Outputs for IfAction SubSystem: '<S1>/Subsystem2' incorporates:
     *  ActionPort: '<S4>/Action Port'
     */
    /* Gain: '<S8>/Gain1' incorporates:
     *  Inport: '<Root>/SeekerData'
     */
    rtb_Gain1_a = 0.0174532924F *
      Intercepe_Strike_Model_AS_U.SeekerData.misalignYaw;

    /* DiscreteIntegrator: '<S10>/Discrete-Time Integrator5' */
    if (Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LO_o != 0) {
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_k = rtb_Gain1_a;
    }

    /* DiscreteIntegrator: '<S49>/Integrator' */
    if (((Intercepe_Strike_Model_AS_B.guided_state > 0) &&
         (Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_d <= 0)) ||
        ((Intercepe_Strike_Model_AS_B.guided_state <= 0) &&
         (Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_d == 1))) {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_ho = 0.0F;
    }

    /* Sum: '<S58>/Sum' incorporates:
     *  DiscreteIntegrator: '<S10>/Discrete-Time Integrator5'
     *  DiscreteIntegrator: '<S49>/Integrator'
     *  Gain: '<S54>/Proportional Gain'
     */
    rtb_Sum_c = STRIKE_PARAM.STRIKE_ROLL_KP *
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_k +
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_ho;

    /* Saturate: '<S56>/Saturation' */
    if (rtb_Sum_c > 3.8397243F) {
      rtb_Saturation_d = 3.8397243F;
    } else if (rtb_Sum_c < -3.8397243F) {
      rtb_Saturation_d = -3.8397243F;
    } else {
      rtb_Saturation_d = rtb_Sum_c;
    }

    /* End of Saturate: '<S56>/Saturation' */

    /* Gain: '<S9>/Gain1' incorporates:
     *  Inport: '<Root>/SeekerData'
     */
    rtb_Gain1_e = 0.0174532924F *
      Intercepe_Strike_Model_AS_U.SeekerData.misalignPitch;

    /* DiscreteIntegrator: '<S11>/Discrete-Time Integrator5' */
    if (Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_L_ll != 0) {
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTA_ku = rtb_Gain1_e;
    }

    /* DiscreteIntegrator: '<S101>/Integrator' */
    if (((Intercepe_Strike_Model_AS_B.guided_state > 0) &&
         (Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_m <= 0)) ||
        ((Intercepe_Strike_Model_AS_B.guided_state <= 0) &&
         (Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_m == 1))) {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_e = 0.0F;
    }

    /* Sum: '<S110>/Sum' incorporates:
     *  DiscreteIntegrator: '<S101>/Integrator'
     *  DiscreteIntegrator: '<S11>/Discrete-Time Integrator5'
     *  Gain: '<S106>/Proportional Gain'
     */
    rtb_Sum_a = STRIKE_PARAM.STRIKE_PITCH_KP *
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTA_ku +
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_e;

    /* Saturate: '<S108>/Saturation' */
    if (rtb_Sum_a > 3.8397243F) {
      rtb_Saturation_bm = 3.8397243F;
    } else if (rtb_Sum_a < -3.8397243F) {
      rtb_Saturation_bm = -3.8397243F;
    } else {
      rtb_Saturation_bm = rtb_Sum_a;
    }

    /* End of Saturate: '<S108>/Saturation' */

    /* Delay: '<S4>/Delay1' incorporates:
     *  Inport: '<Root>/current_posd'
     */
    if (Intercepe_Strike_Model_AS_DW.icLoad_bl) {
      Intercepe_Strike_Model_AS_DW.Delay1_DSTATE_m =
        Intercepe_Strike_Model_AS_U.current_posd;
    }

    /* Sum: '<S4>/Sum1' incorporates:
     *  Delay: '<S4>/Delay1'
     *  Inport: '<Root>/current_posd'
     */
    rtb_IntegralGain_hr = Intercepe_Strike_Model_AS_DW.Delay1_DSTATE_m -
      Intercepe_Strike_Model_AS_U.current_posd;

    /* Saturate: '<S4>/Saturation2' */
    if (rtb_IntegralGain_hr > 0.7F) {
      rtb_IntegralGain_hr = 0.7F;
    } else if (rtb_IntegralGain_hr < 0.2F) {
      rtb_IntegralGain_hr = 0.2F;
    }

    /* Sum: '<S4>/Sum7' incorporates:
     *  Inport: '<Root>/current_vz'
     *  Saturate: '<S4>/Saturation2'
     */
    rtb_Sum7 = rtb_IntegralGain_hr - Intercepe_Strike_Model_AS_U.current_vz;

    /* DiscreteIntegrator: '<S153>/Integrator' */
    if (((Intercepe_Strike_Model_AS_B.guided_state > 0) &&
         (Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_h <= 0)) ||
        ((Intercepe_Strike_Model_AS_B.guided_state <= 0) &&
         (Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_h == 1))) {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_g = 0.0F;
    }

    /* Sum: '<S162>/Sum' incorporates:
     *  DiscreteIntegrator: '<S153>/Integrator'
     *  Gain: '<S158>/Proportional Gain'
     */
    rtb_Sum_k = 0.22F * rtb_Sum7 +
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_g;

    /* Saturate: '<S160>/Saturation' */
    if (rtb_Sum_k > 0.7F) {
      rtb_Saturation_k = 0.7F;
    } else if (rtb_Sum_k < -0.7F) {
      rtb_Saturation_k = -0.7F;
    } else {
      rtb_Saturation_k = rtb_Sum_k;
    }

    /* End of Saturate: '<S160>/Saturation' */

    /* Delay: '<S4>/Delay3' incorporates:
     *  Inport: '<Root>/current_thr'
     */
    if (Intercepe_Strike_Model_AS_DW.icLoad_p) {
      Intercepe_Strike_Model_AS_DW.Delay3_DSTATE_g =
        Intercepe_Strike_Model_AS_U.current_thr;
    }

    /* DataTypeConversion: '<S4>/Data Type Conversion' incorporates:
     *  DiscreteIntegrator: '<S4>/Discrete-Time Integrator'
     */
    tmp = fmod(floor
               (Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_k),
               1.8446744073709552E+19);

    /* BusAssignment: '<S4>/Bus Assignment' incorporates:
     *  Constant: '<S4>/NAVIGATION_STATE_OFFBOARD'
     *  Constant: '<S4>/VEHICLE_CMD_SET_NAV_STATE'
     *  DataTypeConversion: '<S4>/Data Type Conversion'
     *  Merge: '<S1>/Merge'
     */
    Intercepe_Strike_Model_AS_B.Merge.timestamp = tmp < 0.0 ? (uint64_T)
      -(int64_T)(uint64_T)-tmp : (uint64_T)tmp;
    Intercepe_Strike_Model_AS_B.Merge.command = 100001U;
    Intercepe_Strike_Model_AS_B.Merge.param1 = 12.0F;

    /* Sum: '<S4>/Sum5' incorporates:
     *  Gain: '<S4>/Gain5'
     *  Inport: '<Root>/current_roll'
     *  Saturate: '<S4>/Saturation3'
     */
    rtb_IntegralGain_hr = 0.065F * rtb_Saturation_d +
      Intercepe_Strike_Model_AS_U.current_roll;

    /* Saturate: '<S4>/Saturation' */
    if (rtb_IntegralGain_hr > 0.87266463F) {
      /* BusAssignment: '<S4>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param2 = 0.87266463F;
    } else if (rtb_IntegralGain_hr < -0.87266463F) {
      /* BusAssignment: '<S4>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param2 = -0.87266463F;
    } else {
      /* BusAssignment: '<S4>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param2 = rtb_IntegralGain_hr;
    }

    /* End of Saturate: '<S4>/Saturation' */

    /* Sum: '<S4>/Sum6' incorporates:
     *  Gain: '<S4>/Gain6'
     *  Inport: '<Root>/current_pitch'
     *  Saturate: '<S4>/Saturation4'
     */
    rtb_IntegralGain_hr = 0.065F * rtb_Saturation_bm +
      Intercepe_Strike_Model_AS_U.current_pitch;

    /* Saturate: '<S4>/Saturation1' */
    if (rtb_IntegralGain_hr > 0.87266463F) {
      /* BusAssignment: '<S4>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param3 = 0.87266463F;
    } else if (rtb_IntegralGain_hr < -0.87266463F) {
      /* BusAssignment: '<S4>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param3 = -0.87266463F;
    } else {
      /* BusAssignment: '<S4>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param3 = rtb_IntegralGain_hr;
    }

    /* End of Saturate: '<S4>/Saturation1' */

    /* BusAssignment: '<S4>/Bus Assignment' incorporates:
     *  Inport: '<Root>/current_yaw'
     *  Merge: '<S1>/Merge'
     */
    Intercepe_Strike_Model_AS_B.Merge.param4 =
      Intercepe_Strike_Model_AS_U.current_yaw;

    /* Sum: '<S4>/Sum8' incorporates:
     *  Delay: '<S4>/Delay3'
     *  Gain: '<S4>/Gain7'
     */
    rtb_IntegralGain_hr = -rtb_Saturation_k +
      Intercepe_Strike_Model_AS_DW.Delay3_DSTATE_g;

    /* Saturate: '<S4>/Saturation5' */
    if (rtb_IntegralGain_hr > 0.7F) {
      /* BusAssignment: '<S4>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param5 = 0.7F;
    } else if (rtb_IntegralGain_hr < 0.2F) {
      /* BusAssignment: '<S4>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param5 = 0.2F;
    } else {
      /* BusAssignment: '<S4>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param5 = rtb_IntegralGain_hr;
    }

    /* End of Saturate: '<S4>/Saturation5' */

    /* BusAssignment: '<S4>/Bus Assignment' incorporates:
     *  Constant: '<S4>/pass1'
     *  Constant: '<S4>/pass2'
     *  Merge: '<S1>/Merge'
     */
    Intercepe_Strike_Model_AS_B.Merge.param6 = 0.0F;
    Intercepe_Strike_Model_AS_B.Merge.param7 = 0.0F;
    Intercepe_Strike_Model_AS_B.Merge.cmd_valid = cmd_valid;

    /* Gain: '<S98>/Integral Gain' incorporates:
     *  DiscreteIntegrator: '<S11>/Discrete-Time Integrator5'
     */
    rtb_IntegralGain_hr = STRIKE_PARAM.STRIKE_PITCH_KI *
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTA_ku;

    /* Gain: '<S46>/Integral Gain' incorporates:
     *  DiscreteIntegrator: '<S10>/Discrete-Time Integrator5'
     */
    rtb_IntegralGain_hj = STRIKE_PARAM.STRIKE_ROLL_KI *
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_k;

    /* Update for DiscreteIntegrator: '<S4>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S4>/Constant1'
     */
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_k += 10000.0;

    /* Update for DiscreteIntegrator: '<S10>/Discrete-Time Integrator5' incorporates:
     *  Gain: '<S10>/Gain'
     *  Sum: '<S10>/Sum5'
     */
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LO_o = 0U;
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_k += (rtb_Gain1_a
      - Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_k) *
      125.663704F * 0.0025F;

    /* Update for DiscreteIntegrator: '<S49>/Integrator' incorporates:
     *  Sum: '<S41>/SumI2'
     *  Sum: '<S41>/SumI4'
     */
    Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_ho += ((rtb_Saturation_d -
      rtb_Sum_c) + rtb_IntegralGain_hj) * 0.0025F;
    if (Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_ho > 1.04719758F) {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_ho = 1.04719758F;
    } else if (Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_ho < -1.04719758F)
    {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_ho = -1.04719758F;
    }

    Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_d = (int8_T)
      (Intercepe_Strike_Model_AS_B.guided_state > 0);

    /* End of Update for DiscreteIntegrator: '<S49>/Integrator' */

    /* Update for DiscreteIntegrator: '<S11>/Discrete-Time Integrator5' incorporates:
     *  Gain: '<S11>/Gain'
     *  Sum: '<S11>/Sum5'
     */
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_L_ll = 0U;
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTA_ku += (rtb_Gain1_e
      - Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTA_ku) *
      125.663704F * 0.0025F;

    /* Update for DiscreteIntegrator: '<S101>/Integrator' incorporates:
     *  Sum: '<S93>/SumI2'
     *  Sum: '<S93>/SumI4'
     */
    Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_e += ((rtb_Saturation_bm -
      rtb_Sum_a) + rtb_IntegralGain_hr) * 0.0025F;
    if (Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_e > 1.04719758F) {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_e = 1.04719758F;
    } else if (Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_e < -1.04719758F)
    {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_e = -1.04719758F;
    }

    Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_m = (int8_T)
      (Intercepe_Strike_Model_AS_B.guided_state > 0);

    /* End of Update for DiscreteIntegrator: '<S101>/Integrator' */

    /* Update for Delay: '<S4>/Delay1' */
    Intercepe_Strike_Model_AS_DW.icLoad_bl = false;

    /* Update for DiscreteIntegrator: '<S153>/Integrator' incorporates:
     *  Gain: '<S150>/Integral Gain'
     *  Sum: '<S145>/SumI2'
     *  Sum: '<S145>/SumI4'
     */
    Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_g += ((rtb_Saturation_k -
      rtb_Sum_k) + 0.1F * rtb_Sum7) * 0.0025F;
    if (Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_g > 0.3F) {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_g = 0.3F;
    } else if (Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_g < -0.3F) {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_g = -0.3F;
    }

    Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_h = (int8_T)
      (Intercepe_Strike_Model_AS_B.guided_state > 0);

    /* End of Update for DiscreteIntegrator: '<S153>/Integrator' */

    /* Update for Delay: '<S4>/Delay3' */
    Intercepe_Strike_Model_AS_DW.icLoad_p = false;

    /* End of Outputs for SubSystem: '<S1>/Subsystem2' */
    break;

   case 3:
    if (Intercepe_Strike_Model_AS_DW.SwitchCase_ActiveSubsystem != rtPrevAction)
    {
      /* InitializeConditions for IfAction SubSystem: '<S1>/Subsystem3' incorporates:
       *  ActionPort: '<S5>/Action Port'
       */
      /* InitializeConditions for SwitchCase: '<S1>/Switch Case' incorporates:
       *  Delay: '<S5>/Delay1'
       *  Delay: '<S5>/Delay2'
       *  Delay: '<S5>/Delay3'
       *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
       */
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_m = 0.0;
      Intercepe_Strike_Model_AS_DW.icLoad_d = true;
      Intercepe_Strike_Model_AS_DW.icLoad_b = true;
      Intercepe_Strike_Model_AS_DW.icLoad_bb = true;

      /* End of InitializeConditions for SubSystem: '<S1>/Subsystem3' */
    }

    /* Outputs for IfAction SubSystem: '<S1>/Subsystem3' incorporates:
     *  ActionPort: '<S5>/Action Port'
     */
    /* Delay: '<S5>/Delay1' incorporates:
     *  Inport: '<Root>/current_posn'
     */
    if (Intercepe_Strike_Model_AS_DW.icLoad_d) {
      Intercepe_Strike_Model_AS_DW.Delay1_DSTATE =
        Intercepe_Strike_Model_AS_U.current_posn;
    }

    /* Delay: '<S5>/Delay2' incorporates:
     *  Inport: '<Root>/current_pose'
     */
    if (Intercepe_Strike_Model_AS_DW.icLoad_b) {
      Intercepe_Strike_Model_AS_DW.Delay2_DSTATE =
        Intercepe_Strike_Model_AS_U.current_pose;
    }

    /* Delay: '<S5>/Delay3' incorporates:
     *  Inport: '<Root>/current_posd'
     */
    if (Intercepe_Strike_Model_AS_DW.icLoad_bb) {
      Intercepe_Strike_Model_AS_DW.Delay3_DSTATE_c =
        Intercepe_Strike_Model_AS_U.current_posd;
    }

    /* DataTypeConversion: '<S5>/Data Type Conversion' incorporates:
     *  DiscreteIntegrator: '<S5>/Discrete-Time Integrator'
     */
    tmp = fmod(floor
               (Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_m),
               1.8446744073709552E+19);

    /* BusAssignment: '<S5>/Bus Assignment' incorporates:
     *  Constant: '<S5>/NAVIGATION_STATE_OFFBOARD'
     *  Constant: '<S5>/VEHICLE_CMD_SET_NAV_STATE'
     *  Constant: '<S5>/pass1'
     *  Constant: '<S5>/pass2'
     *  Constant: '<S5>/pass6'
     *  DataTypeConversion: '<S5>/Data Type Conversion'
     *  Delay: '<S5>/Delay1'
     *  Delay: '<S5>/Delay2'
     *  Delay: '<S5>/Delay3'
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
      Intercepe_Strike_Model_AS_DW.Delay3_DSTATE_c;
    Intercepe_Strike_Model_AS_B.Merge.param6 = 0.0F;
    Intercepe_Strike_Model_AS_B.Merge.param7 = 0.0F;
    Intercepe_Strike_Model_AS_B.Merge.cmd_valid = cmd_valid;

    /* Update for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S5>/Constant1'
     */
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_m += 10000.0;

    /* Update for Delay: '<S5>/Delay1' */
    Intercepe_Strike_Model_AS_DW.icLoad_d = false;

    /* Update for Delay: '<S5>/Delay2' */
    Intercepe_Strike_Model_AS_DW.icLoad_b = false;

    /* Update for Delay: '<S5>/Delay3' */
    Intercepe_Strike_Model_AS_DW.icLoad_bb = false;

    /* End of Outputs for SubSystem: '<S1>/Subsystem3' */
    break;

   case 4:
    /* Outputs for IfAction SubSystem: '<S1>/Subsystem4' incorporates:
     *  ActionPort: '<S6>/Action Port'
     */
    /* Trigonometry: '<S6>/Atan1' incorporates:
     *  Gain: '<S171>/Gain1'
     *  Gain: '<S6>/Gain1'
     *  Inport: '<Root>/SeekerData'
     */
    rtb_Gain1_a = atan2f(0.0174532924F *
                         Intercepe_Strike_Model_AS_U.SeekerData.losRatePitch *
                         4.0F, 9.8F);

    /* DiscreteIntegrator: '<S178>/Discrete-Time Integrator5' */
    if (Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LOAD != 0) {
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTATE = rtb_Gain1_a;
    }

    /* Gain: '<S173>/Gain1' incorporates:
     *  Inport: '<Root>/SeekerData'
     */
    rtb_Sum_c = 0.0174532924F *
      Intercepe_Strike_Model_AS_U.SeekerData.misalignPitch;

    /* DiscreteIntegrator: '<S176>/Discrete-Time Integrator5' */
    if (Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LO_l != 0) {
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_i = rtb_Sum_c;
    }

    /* DiscreteIntegrator: '<S267>/Integrator' */
    if (((Intercepe_Strike_Model_AS_B.guided_state > 0) &&
         (Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState <= 0)) ||
        ((Intercepe_Strike_Model_AS_B.guided_state <= 0) &&
         (Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState == 1))) {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE = 0.0F;
    }

    /* Sum: '<S276>/Sum' incorporates:
     *  DiscreteIntegrator: '<S176>/Discrete-Time Integrator5'
     *  DiscreteIntegrator: '<S267>/Integrator'
     *  Gain: '<S272>/Proportional Gain'
     */
    rtb_Saturation_d = STRIKE_PARAM.STRIKE_PITCH_KP *
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_i +
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE;

    /* Saturate: '<S274>/Saturation' */
    if (rtb_Saturation_d > 3.8397243F) {
      rtb_Gain1_e = 3.8397243F;
    } else if (rtb_Saturation_d < -3.8397243F) {
      rtb_Gain1_e = -3.8397243F;
    } else {
      rtb_Gain1_e = rtb_Saturation_d;
    }

    /* End of Saturate: '<S274>/Saturation' */

    /* Gain: '<S174>/Gain1' incorporates:
     *  Inport: '<Root>/SeekerData'
     */
    rtb_Sum_a = 0.0174532924F *
      Intercepe_Strike_Model_AS_U.SeekerData.misalignYaw;

    /* DiscreteIntegrator: '<S175>/Discrete-Time Integrator5' */
    if (Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LO_k != 0) {
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_a = rtb_Sum_a;
    }

    /* DiscreteIntegrator: '<S215>/Integrator' */
    if (((Intercepe_Strike_Model_AS_B.guided_state > 0) &&
         (Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_o <= 0)) ||
        ((Intercepe_Strike_Model_AS_B.guided_state <= 0) &&
         (Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_o == 1))) {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_h = 0.0F;
    }

    /* Sum: '<S224>/Sum' incorporates:
     *  DiscreteIntegrator: '<S175>/Discrete-Time Integrator5'
     *  DiscreteIntegrator: '<S215>/Integrator'
     *  Gain: '<S220>/Proportional Gain'
     */
    rtb_Saturation_bm = STRIKE_PARAM.STRIKE_ROLL_KP *
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_a +
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_h;

    /* Saturate: '<S222>/Saturation' */
    if (rtb_Saturation_bm > 3.8397243F) {
      rtb_Sum7 = 3.8397243F;
    } else if (rtb_Saturation_bm < -3.8397243F) {
      rtb_Sum7 = -3.8397243F;
    } else {
      rtb_Sum7 = rtb_Saturation_bm;
    }

    /* End of Saturate: '<S222>/Saturation' */

    /* Trigonometry: '<S6>/Atan2' incorporates:
     *  Gain: '<S172>/Gain1'
     *  Gain: '<S6>/Gain'
     *  Inport: '<Root>/SeekerData'
     */
    rtb_Sum_k = atan2f(0.0174532924F *
                       Intercepe_Strike_Model_AS_U.SeekerData.losRateYaw * 4.0F,
                       9.8F);

    /* DiscreteIntegrator: '<S177>/Discrete-Time Integrator5' */
    if (Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LO_n != 0) {
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_d = rtb_Sum_k;
    }

    /* RateLimiter: '<S6>/Rate Limiter' incorporates:
     *  Constant: '<S6>/Constant4'
     */
    if (1.0F - Intercepe_Strike_Model_AS_DW.PrevY > 0.005F) {
      /* Saturate: '<S6>/Saturation5' */
      Intercepe_Strike_Model_AS_DW.PrevY += 0.005F;
    } else if (1.0F - Intercepe_Strike_Model_AS_DW.PrevY < -0.0025F) {
      /* Saturate: '<S6>/Saturation5' */
      Intercepe_Strike_Model_AS_DW.PrevY -= 0.0025F;
    } else {
      /* Saturate: '<S6>/Saturation5' */
      Intercepe_Strike_Model_AS_DW.PrevY = 1.0F;
    }

    /* End of RateLimiter: '<S6>/Rate Limiter' */

    /* Delay: '<S6>/Delay3' incorporates:
     *  Inport: '<Root>/current_thr'
     */
    if (Intercepe_Strike_Model_AS_DW.icLoad) {
      Intercepe_Strike_Model_AS_DW.Delay3_DSTATE =
        Intercepe_Strike_Model_AS_U.current_thr;
    }

    /* DataTypeConversion: '<S6>/Data Type Conversion' incorporates:
     *  DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
     */
    tmp = fmod(floor(Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE),
               1.8446744073709552E+19);

    /* BusAssignment: '<S6>/Bus Assignment' incorporates:
     *  Constant: '<S6>/NAVIGATION_STATE_OFFBOARD'
     *  Constant: '<S6>/VEHICLE_CMD_SET_NAV_STATE'
     *  DataTypeConversion: '<S6>/Data Type Conversion'
     *  Merge: '<S1>/Merge'
     */
    Intercepe_Strike_Model_AS_B.Merge.timestamp = tmp < 0.0 ? (uint64_T)
      -(int64_T)(uint64_T)-tmp : (uint64_T)tmp;
    Intercepe_Strike_Model_AS_B.Merge.command = 100001U;
    Intercepe_Strike_Model_AS_B.Merge.param1 = 12.0F;

    /* Sum: '<S6>/Sum' incorporates:
     *  DiscreteIntegrator: '<S177>/Discrete-Time Integrator5'
     *  Gain: '<S6>/Gain5'
     *  Inport: '<Root>/current_roll'
     *  Saturate: '<S6>/Saturation3'
     *  Sum: '<S6>/Sum5'
     */
    rtb_IntegralGain_hr = (0.065F * rtb_Sum7 +
      Intercepe_Strike_Model_AS_U.current_roll) +
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_d;

    /* Saturate: '<S6>/Saturation' */
    if (rtb_IntegralGain_hr > 0.87266463F) {
      /* BusAssignment: '<S6>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param2 = 0.87266463F;
    } else if (rtb_IntegralGain_hr < -0.87266463F) {
      /* BusAssignment: '<S6>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param2 = -0.87266463F;
    } else {
      /* BusAssignment: '<S6>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param2 = rtb_IntegralGain_hr;
    }

    /* End of Saturate: '<S6>/Saturation' */

    /* Sum: '<S6>/Sum1' incorporates:
     *  DiscreteIntegrator: '<S178>/Discrete-Time Integrator5'
     *  Gain: '<S6>/Gain6'
     *  Inport: '<Root>/current_pitch'
     *  Saturate: '<S6>/Saturation4'
     *  Sum: '<S6>/Sum6'
     */
    rtb_IntegralGain_hr = (0.065F * rtb_Gain1_e +
      Intercepe_Strike_Model_AS_U.current_pitch) +
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTATE;

    /* Saturate: '<S6>/Saturation1' */
    if (rtb_IntegralGain_hr > 0.87266463F) {
      /* BusAssignment: '<S6>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param3 = 0.87266463F;
    } else if (rtb_IntegralGain_hr < -0.87266463F) {
      /* BusAssignment: '<S6>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param3 = -0.87266463F;
    } else {
      /* BusAssignment: '<S6>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param3 = rtb_IntegralGain_hr;
    }

    /* End of Saturate: '<S6>/Saturation1' */

    /* BusAssignment: '<S6>/Bus Assignment' incorporates:
     *  Inport: '<Root>/current_yaw'
     *  Merge: '<S1>/Merge'
     */
    Intercepe_Strike_Model_AS_B.Merge.param4 =
      Intercepe_Strike_Model_AS_U.current_yaw;

    /* Sum: '<S6>/Sum4' incorporates:
     *  Delay: '<S6>/Delay3'
     */
    rtb_IntegralGain_hr = Intercepe_Strike_Model_AS_DW.PrevY +
      Intercepe_Strike_Model_AS_DW.Delay3_DSTATE;

    /* Saturate: '<S6>/Saturation5' */
    if (rtb_IntegralGain_hr > 0.7F) {
      /* BusAssignment: '<S6>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param5 = 0.7F;
    } else if (rtb_IntegralGain_hr < 0.2F) {
      /* BusAssignment: '<S6>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param5 = 0.2F;
    } else {
      /* BusAssignment: '<S6>/Bus Assignment' incorporates:
       *  Merge: '<S1>/Merge'
       */
      Intercepe_Strike_Model_AS_B.Merge.param5 = rtb_IntegralGain_hr;
    }

    /* End of Saturate: '<S6>/Saturation5' */

    /* BusAssignment: '<S6>/Bus Assignment' incorporates:
     *  Constant: '<S6>/pass1'
     *  Constant: '<S6>/pass2'
     *  Merge: '<S1>/Merge'
     */
    Intercepe_Strike_Model_AS_B.Merge.param6 = 0.0F;
    Intercepe_Strike_Model_AS_B.Merge.param7 = 0.0F;
    Intercepe_Strike_Model_AS_B.Merge.cmd_valid = cmd_valid;

    /* Gain: '<S212>/Integral Gain' incorporates:
     *  DiscreteIntegrator: '<S175>/Discrete-Time Integrator5'
     */
    rtb_IntegralGain_hr = STRIKE_PARAM.STRIKE_ROLL_KI *
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_a;

    /* Gain: '<S264>/Integral Gain' incorporates:
     *  DiscreteIntegrator: '<S176>/Discrete-Time Integrator5'
     */
    rtb_Saturation_k = STRIKE_PARAM.STRIKE_PITCH_KI *
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_i;

    /* Update for DiscreteIntegrator: '<S178>/Discrete-Time Integrator5' incorporates:
     *  Gain: '<S178>/Gain'
     *  Sum: '<S178>/Sum5'
     */
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LOAD = 0U;
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTATE += (rtb_Gain1_a
      - Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTATE) *
      31.415926F * 0.0025F;

    /* Update for DiscreteIntegrator: '<S176>/Discrete-Time Integrator5' incorporates:
     *  Gain: '<S176>/Gain'
     *  Sum: '<S176>/Sum5'
     */
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LO_l = 0U;
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_i += (rtb_Sum_c -
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_i) *
      125.663704F * 0.0025F;

    /* Update for DiscreteIntegrator: '<S267>/Integrator' incorporates:
     *  Sum: '<S259>/SumI2'
     *  Sum: '<S259>/SumI4'
     */
    Intercepe_Strike_Model_AS_DW.Integrator_DSTATE += ((rtb_Gain1_e -
      rtb_Saturation_d) + rtb_Saturation_k) * 0.0025F;
    if (Intercepe_Strike_Model_AS_DW.Integrator_DSTATE > 1.04719758F) {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE = 1.04719758F;
    } else if (Intercepe_Strike_Model_AS_DW.Integrator_DSTATE < -1.04719758F) {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE = -1.04719758F;
    }

    Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState = (int8_T)
      (Intercepe_Strike_Model_AS_B.guided_state > 0);

    /* End of Update for DiscreteIntegrator: '<S267>/Integrator' */

    /* Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator' incorporates:
     *  Constant: '<S6>/Constant1'
     */
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE += 10000.0;

    /* Update for DiscreteIntegrator: '<S175>/Discrete-Time Integrator5' incorporates:
     *  Gain: '<S175>/Gain'
     *  Sum: '<S175>/Sum5'
     */
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LO_k = 0U;
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_a += (rtb_Sum_a -
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_a) *
      125.663704F * 0.0025F;

    /* Update for DiscreteIntegrator: '<S215>/Integrator' incorporates:
     *  Sum: '<S207>/SumI2'
     *  Sum: '<S207>/SumI4'
     */
    Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_h += ((rtb_Sum7 -
      rtb_Saturation_bm) + rtb_IntegralGain_hr) * 0.0025F;
    if (Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_h > 1.04719758F) {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_h = 1.04719758F;
    } else if (Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_h < -1.04719758F)
    {
      Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_h = -1.04719758F;
    }

    Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_o = (int8_T)
      (Intercepe_Strike_Model_AS_B.guided_state > 0);

    /* End of Update for DiscreteIntegrator: '<S215>/Integrator' */

    /* Update for DiscreteIntegrator: '<S177>/Discrete-Time Integrator5' incorporates:
     *  Gain: '<S177>/Gain'
     *  Sum: '<S177>/Sum5'
     */
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LO_n = 0U;
    Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_d += (rtb_Sum_k -
      Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_DSTAT_d) * 31.415926F
      * 0.0025F;

    /* Update for Delay: '<S6>/Delay3' */
    Intercepe_Strike_Model_AS_DW.icLoad = false;

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
  Intercepe_Strike_Model_AS_DW.is_active_c3_Intercepe_Strike_M = 0U;
  Intercepe_Strike_Model_AS_DW.is_c3_Intercepe_Strike_Model_AS =
    Intercepe_St_IN_NO_ACTIVE_CHILD;

  /* SystemInitialize for IfAction SubSystem: '<S1>/Subsystem2' */
  /* InitializeConditions for DiscreteIntegrator: '<S4>/Discrete-Time Integrator' */
  Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_k = 0.0;

  /* InitializeConditions for DiscreteIntegrator: '<S10>/Discrete-Time Integrator5' */
  Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LO_o = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S49>/Integrator' */
  Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_ho = 0.0F;
  Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_d = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S11>/Discrete-Time Integrator5' */
  Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_L_ll = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S101>/Integrator' */
  Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_e = 0.0F;
  Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_m = 2;

  /* InitializeConditions for Delay: '<S4>/Delay1' */
  Intercepe_Strike_Model_AS_DW.icLoad_bl = true;

  /* InitializeConditions for DiscreteIntegrator: '<S153>/Integrator' */
  Intercepe_Strike_Model_AS_DW.Integrator_DSTATE_g = 0.0F;
  Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_h = 2;

  /* InitializeConditions for Delay: '<S4>/Delay3' */
  Intercepe_Strike_Model_AS_DW.icLoad_p = true;

  /* End of SystemInitialize for SubSystem: '<S1>/Subsystem2' */

  /* SystemInitialize for IfAction SubSystem: '<S1>/Subsystem3' */
  /* InitializeConditions for DiscreteIntegrator: '<S5>/Discrete-Time Integrator' */
  Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator_DSTATE_m = 0.0;

  /* InitializeConditions for Delay: '<S5>/Delay1' */
  Intercepe_Strike_Model_AS_DW.icLoad_d = true;

  /* InitializeConditions for Delay: '<S5>/Delay2' */
  Intercepe_Strike_Model_AS_DW.icLoad_b = true;

  /* InitializeConditions for Delay: '<S5>/Delay3' */
  Intercepe_Strike_Model_AS_DW.icLoad_bb = true;

  /* End of SystemInitialize for SubSystem: '<S1>/Subsystem3' */

  /* SystemInitialize for IfAction SubSystem: '<S1>/Subsystem4' */
  /* InitializeConditions for DiscreteIntegrator: '<S178>/Discrete-Time Integrator5' */
  Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LOAD = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S176>/Discrete-Time Integrator5' */
  Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LO_l = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S267>/Integrator' */
  Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S175>/Discrete-Time Integrator5' */
  Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LO_k = 1U;

  /* InitializeConditions for DiscreteIntegrator: '<S215>/Integrator' */
  Intercepe_Strike_Model_AS_DW.Integrator_PrevResetState_o = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S177>/Discrete-Time Integrator5' */
  Intercepe_Strike_Model_AS_DW.DiscreteTimeIntegrator5_IC_LO_n = 1U;

  /* InitializeConditions for Saturate: '<S6>/Saturation5' incorporates:
   *  RateLimiter: '<S6>/Rate Limiter'
   */
  Intercepe_Strike_Model_AS_DW.PrevY = 0.0F;

  /* InitializeConditions for Delay: '<S6>/Delay3' */
  Intercepe_Strike_Model_AS_DW.icLoad = true;

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
