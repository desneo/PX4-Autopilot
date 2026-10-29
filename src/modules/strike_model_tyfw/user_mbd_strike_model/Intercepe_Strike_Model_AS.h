/*
 * File: Intercepe_Strike_Model_AS.h
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

#ifndef Intercepe_Strike_Model_AS_h_
#define Intercepe_Strike_Model_AS_h_
#ifndef Intercepe_Strike_Model_AS_COMMON_INCLUDES_
#define Intercepe_Strike_Model_AS_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif                          /* Intercepe_Strike_Model_AS_COMMON_INCLUDES_ */

#include "Intercepe_Strike_Model_AS_types.h"
#include <string.h>

/* Block states (default storage) for system '<S1>/Subsystem6' */
typedef struct {
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S8>/Discrete-Time Integrator' */
} DW_Subsystem6_Intercepe_Strik_T;

/* Block states (default storage) for system '<S11>/MATLAB Function' */
typedef struct {
  real32_T x_prev;                     /* '<S11>/MATLAB Function' */
  real32_T y_prev;                     /* '<S11>/MATLAB Function' */
  boolean_T x_prev_not_empty;          /* '<S11>/MATLAB Function' */
} DW_MATLABFunction_Intercepe_h_T;

/* Block states (default storage) for system '<S16>/MATLAB Function1' */
typedef struct {
  real32_T x_prev;                     /* '<S16>/MATLAB Function1' */
  real32_T y_prev;                     /* '<S16>/MATLAB Function1' */
  boolean_T x_prev_not_empty;          /* '<S16>/MATLAB Function1' */
} DW_MATLABFunction1_Intercep_g_T;

/* Block states (default storage) for system '<S16>/MATLAB Function4' */
typedef struct {
  real32_T x_prev;                     /* '<S16>/MATLAB Function4' */
  real32_T y_prev;                     /* '<S16>/MATLAB Function4' */
  boolean_T x_prev_not_empty;          /* '<S16>/MATLAB Function4' */
} DW_MATLABFunction4_Intercepe__T;

/* Block signals (default storage) */
typedef struct {
  VehicleCommandBus Merge;             /* '<S1>/Merge' */
  uint8_T guided_state;                /* '<S2>/Chart' */
} B_Intercepe_Strike_Model_AS_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S7>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator_DSTATE_m;/* '<S6>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator_DSTATE_k;/* '<S5>/Discrete-Time Integrator' */
  uint64_T last_timestamp;             /* '<S2>/Chart' */
  real32_T Delay1_DSTATE;              /* '<S6>/Delay1' */
  real32_T Delay2_DSTATE;              /* '<S6>/Delay2' */
  real32_T Delay3_DSTATE;              /* '<S6>/Delay3' */
  real32_T Delay1_DSTATE_m;            /* '<S5>/Delay1' */
  real32_T Integrator_DSTATE;          /* '<S52>/Integrator' */
  real32_T Delay3_DSTATE_g;            /* '<S5>/Delay3' */
  real32_T time_gate;                  /* '<S2>/Chart' */
  real32_T lock_time;                  /* '<S2>/Chart' */
  real32_T PrevY;                      /* '<S79>/Rate Limiter2' */
  real32_T PrevY_l;                    /* '<S79>/Rate Limiter1' */
  real32_T PrevY_a;                    /* '<S15>/Rate Limiter2' */
  real32_T PrevY_g;                    /* '<S15>/Rate Limiter1' */
  uint32_T durationCounter_1;          /* '<S2>/Chart' */
  int8_T SwitchCase_ActiveSubsystem;   /* '<S1>/Switch Case' */
  int8_T Integrator_PrevResetState;    /* '<S52>/Integrator' */
  uint8_T stale_count;                 /* '<S2>/Chart' */
  uint8_T is_active_c3_Intercepe_Strike_M;/* '<S2>/Chart' */
  uint8_T is_c3_Intercepe_Strike_Model_AS;/* '<S2>/Chart' */
  boolean_T icLoad;                    /* '<S6>/Delay1' */
  boolean_T icLoad_b;                  /* '<S6>/Delay2' */
  boolean_T icLoad_bb;                 /* '<S6>/Delay3' */
  boolean_T icLoad_bl;                 /* '<S5>/Delay1' */
  boolean_T icLoad_p;                  /* '<S5>/Delay3' */
  DW_MATLABFunction4_Intercepe__T sf_MATLABFunction4_h;/* '<S80>/MATLAB Function4' */
  DW_MATLABFunction1_Intercep_g_T sf_MATLABFunction2_k;/* '<S80>/MATLAB Function2' */
  DW_MATLABFunction1_Intercep_g_T sf_MATLABFunction1_gf;/* '<S80>/MATLAB Function1' */
  DW_MATLABFunction_Intercepe_h_T sf_MATLABFunction_h;/* '<S76>/MATLAB Function' */
  DW_MATLABFunction4_Intercepe__T sf_MATLABFunction4;/* '<S16>/MATLAB Function4' */
  DW_MATLABFunction1_Intercep_g_T sf_MATLABFunction2;/* '<S16>/MATLAB Function2' */
  DW_MATLABFunction1_Intercep_g_T sf_MATLABFunction1_g;/* '<S16>/MATLAB Function1' */
  DW_MATLABFunction_Intercepe_h_T sf_MATLABFunction_f;/* '<S11>/MATLAB Function' */
  DW_Subsystem6_Intercepe_Strik_T Subsystem1;/* '<S1>/Subsystem1' */
  DW_Subsystem6_Intercepe_Strik_T Subsystem6;/* '<S1>/Subsystem6' */
} DW_Intercepe_Strike_Model_AS_T;

/* Invariant block signals (default storage) */
typedef struct {
  const real32_T Transpose[9];         /* '<S78>/Transpose' */
  const real32_T Transpose_g[9];       /* '<S14>/Transpose' */
} ConstB_Intercepe_Strike_Model_T;

/* Constant parameters (default storage) */
typedef struct {
  /* Pooled Parameter (Expression: [0 0 1;0 1 0;-1 0 0])
   * Referenced by:
   *   '<S14>/Constant'
   *   '<S78>/Constant'
   */
  real32_T pooled11[9];

  /* Pooled Parameter (Expression: single([0;0;9.81]))
   * Referenced by:
   *   '<S15>/Constant'
   *   '<S79>/Constant'
   */
  real32_T pooled12[3];
} ConstP_Intercepe_Strike_Model_T;

/* External inputs (root inport signals with default storage) */
typedef struct {
  SeekerGuidanceBus SeekerData;        /* '<Root>/SeekerData' */
  boolean_T bool_search;               /* '<Root>/bool_search' */
  boolean_T bool_request;              /* '<Root>/bool_request' */
  uint8_T arming_state;                /* '<Root>/arming_state' */
  real32_T current_roll;               /* '<Root>/current_roll' */
  real32_T current_pitch;              /* '<Root>/current_pitch' */
  real32_T current_yaw;                /* '<Root>/current_yaw' */
  real32_T current_vz;                 /* '<Root>/current_vz' */
  real32_T current_posn;               /* '<Root>/current_posn' */
  real32_T current_pose;               /* '<Root>/current_pose' */
  real32_T current_posd;               /* '<Root>/current_posd' */
  real32_T current_thr;                /* '<Root>/current_thr' */
  real32_T current_vel_mag;            /* '<Root>/current_vel_mag' */
  real32_T current_vx;                 /* '<Root>/current_vx' */
  real32_T current_vy;                 /* '<Root>/current_vy' */
} ExtU_Intercepe_Strike_Model_A_T;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  VehicleCommandBus VehicleCommand;    /* '<Root>/VehicleCommand' */
} ExtY_Intercepe_Strike_Model_A_T;

/* Block signals (default storage) */
extern B_Intercepe_Strike_Model_AS_T Intercepe_Strike_Model_AS_B;

/* Block states (default storage) */
extern DW_Intercepe_Strike_Model_AS_T Intercepe_Strike_Model_AS_DW;

/* External inputs (root inport signals with default storage) */
extern ExtU_Intercepe_Strike_Model_A_T Intercepe_Strike_Model_AS_U;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY_Intercepe_Strike_Model_A_T Intercepe_Strike_Model_AS_Y;

/* External data declarations for dependent source files */
extern const VehicleCommandBus Intercepe_Strike_Model_AS_rtZVe;/* VehicleCommandBus ground */
extern const ConstB_Intercepe_Strike_Model_T Intercepe_Strike_Model_A_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_Intercepe_Strike_Model_T Intercepe_Strike_Model_A_ConstP;

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern STRIKE_PARAM_t STRIKE_PARAM;    /* Variable: STRIKE_PARAM
                                        * Referenced by: '<S2>/Chart'
                                        * 打击模式控制器参数结构体，支持运行时调参
                                        */

/* Model entry point functions */
extern void Intercepe_Strike_Model_AS_initialize(void);
extern void Intercepe_Strike_Model_AS_step(void);
extern void Intercepe_Strike_Model_AS_terminate(void);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S15>/Add1' : Unused code path elimination
 * Block '<S15>/Gain' : Unused code path elimination
 * Block '<S15>/Product3' : Unused code path elimination
 * Block '<S15>/Sqrt1' : Unused code path elimination
 * Block '<S15>/Square2' : Unused code path elimination
 * Block '<S15>/Square3' : Unused code path elimination
 * Block '<S15>/Square4' : Unused code path elimination
 * Block '<S15>/mass' : Unused code path elimination
 * Block '<S5>/Gain' : Eliminated nontunable gain of 1
 * Block '<S44>/Kb' : Eliminated nontunable gain of 1
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'Intercepe_Strike_Model_AS'
 * '<S1>'   : 'Intercepe_Strike_Model_AS/制导控制'
 * '<S2>'   : 'Intercepe_Strike_Model_AS/制导状态机'
 * '<S3>'   : 'Intercepe_Strike_Model_AS/制导控制/NED Vel to Body Vel'
 * '<S4>'   : 'Intercepe_Strike_Model_AS/制导控制/Subsystem1'
 * '<S5>'   : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2'
 * '<S6>'   : 'Intercepe_Strike_Model_AS/制导控制/Subsystem3'
 * '<S7>'   : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4'
 * '<S8>'   : 'Intercepe_Strike_Model_AS/制导控制/Subsystem6'
 * '<S9>'   : 'Intercepe_Strike_Model_AS/制导控制/NED Vel to Body Vel/MATLAB Function'
 * '<S10>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/FOV_Constrained_PN'
 * '<S11>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/Instead of Vc'
 * '<S12>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/LOS_to_NED_Transform'
 * '<S13>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3'
 * '<S14>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/Seeker yaw angle'
 * '<S15>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/Thrust_Vectoring'
 * '<S16>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/transform to LOS'
 * '<S17>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/Instead of Vc/MATLAB Function'
 * '<S18>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Anti-windup'
 * '<S19>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/D Gain'
 * '<S20>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/External Derivative'
 * '<S21>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Filter'
 * '<S22>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Filter ICs'
 * '<S23>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/I Gain'
 * '<S24>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Ideal P Gain'
 * '<S25>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Ideal P Gain Fdbk'
 * '<S26>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Integrator'
 * '<S27>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Integrator ICs'
 * '<S28>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/N Copy'
 * '<S29>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/N Gain'
 * '<S30>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/P Copy'
 * '<S31>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Parallel P Gain'
 * '<S32>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Reset Signal'
 * '<S33>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Saturation'
 * '<S34>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Saturation Fdbk'
 * '<S35>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Sum'
 * '<S36>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Sum Fdbk'
 * '<S37>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tracking Mode'
 * '<S38>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tracking Mode Sum'
 * '<S39>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tsamp - Integral'
 * '<S40>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tsamp - Ngain'
 * '<S41>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/postSat Signal'
 * '<S42>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/preInt Signal'
 * '<S43>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/preSat Signal'
 * '<S44>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Anti-windup/Back Calculation'
 * '<S45>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/D Gain/Disabled'
 * '<S46>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/External Derivative/Disabled'
 * '<S47>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Filter/Disabled'
 * '<S48>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Filter ICs/Disabled'
 * '<S49>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/I Gain/Internal Parameters'
 * '<S50>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Ideal P Gain/Passthrough'
 * '<S51>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Ideal P Gain Fdbk/Disabled'
 * '<S52>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Integrator/Discrete'
 * '<S53>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Integrator ICs/Internal IC'
 * '<S54>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/N Copy/Disabled wSignal Specification'
 * '<S55>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/N Gain/Disabled'
 * '<S56>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/P Copy/Disabled'
 * '<S57>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Parallel P Gain/Internal Parameters'
 * '<S58>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Reset Signal/External Reset'
 * '<S59>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Saturation/Enabled'
 * '<S60>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Saturation Fdbk/Disabled'
 * '<S61>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Sum/Sum_PI'
 * '<S62>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Sum Fdbk/Disabled'
 * '<S63>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tracking Mode/Disabled'
 * '<S64>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tracking Mode Sum/Passthrough'
 * '<S65>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tsamp - Integral/TsSignalSpecification'
 * '<S66>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tsamp - Ngain/Passthrough'
 * '<S67>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/postSat Signal/Forward_Path'
 * '<S68>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/preInt Signal/Internal PreInt'
 * '<S69>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/preSat Signal/Forward_Path'
 * '<S70>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/Seeker yaw angle/MATLAB Function'
 * '<S71>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/Seeker yaw angle/MATLAB Function1'
 * '<S72>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/transform to LOS/MATLAB Function1'
 * '<S73>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/transform to LOS/MATLAB Function2'
 * '<S74>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/transform to LOS/MATLAB Function4'
 * '<S75>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/FOV_Constrained_PN'
 * '<S76>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/Instead of Vc'
 * '<S77>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/LOS_to_NED_Transform'
 * '<S78>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/Seeker yaw angle'
 * '<S79>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/Thrust_Vectoring'
 * '<S80>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/transform to LOS'
 * '<S81>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/Instead of Vc/MATLAB Function'
 * '<S82>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/Seeker yaw angle/MATLAB Function'
 * '<S83>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/Seeker yaw angle/MATLAB Function1'
 * '<S84>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/transform to LOS/MATLAB Function1'
 * '<S85>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/transform to LOS/MATLAB Function2'
 * '<S86>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/transform to LOS/MATLAB Function4'
 * '<S87>'  : 'Intercepe_Strike_Model_AS/制导状态机/Chart'
 */
#endif                                 /* Intercepe_Strike_Model_AS_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
