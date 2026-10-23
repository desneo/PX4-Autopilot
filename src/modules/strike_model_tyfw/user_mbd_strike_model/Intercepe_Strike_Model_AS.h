/*
 * File: Intercepe_Strike_Model_AS.h
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
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S7>/Discrete-Time Integrator' */
} DW_Subsystem6_Intercepe_Strik_T;

/* Block signals (default storage) */
typedef struct {
  VehicleCommandBus Merge;             /* '<S1>/Merge' */
  uint8_T guided_state;                /* '<S2>/Chart' */
} B_Intercepe_Strike_Model_AS_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  real_T DiscreteTimeIntegrator_DSTATE;/* '<S6>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator_DSTATE_m;/* '<S5>/Discrete-Time Integrator' */
  real_T DiscreteTimeIntegrator_DSTATE_k;/* '<S4>/Discrete-Time Integrator' */
  uint64_T last_timestamp;             /* '<S2>/Chart' */
  real32_T DiscreteTimeIntegrator5_DSTATE;/* '<S178>/Discrete-Time Integrator5' */
  real32_T DiscreteTimeIntegrator5_DSTAT_i;/* '<S176>/Discrete-Time Integrator5' */
  real32_T Integrator_DSTATE;          /* '<S267>/Integrator' */
  real32_T DiscreteTimeIntegrator5_DSTAT_a;/* '<S175>/Discrete-Time Integrator5' */
  real32_T Integrator_DSTATE_h;        /* '<S215>/Integrator' */
  real32_T DiscreteTimeIntegrator5_DSTAT_d;/* '<S177>/Discrete-Time Integrator5' */
  real32_T Delay3_DSTATE;              /* '<S6>/Delay3' */
  real32_T Delay1_DSTATE;              /* '<S5>/Delay1' */
  real32_T Delay2_DSTATE;              /* '<S5>/Delay2' */
  real32_T Delay3_DSTATE_c;            /* '<S5>/Delay3' */
  real32_T DiscreteTimeIntegrator5_DSTAT_k;/* '<S10>/Discrete-Time Integrator5' */
  real32_T Integrator_DSTATE_ho;       /* '<S49>/Integrator' */
  real32_T DiscreteTimeIntegrator5_DSTA_ku;/* '<S11>/Discrete-Time Integrator5' */
  real32_T Integrator_DSTATE_e;        /* '<S101>/Integrator' */
  real32_T Delay1_DSTATE_m;            /* '<S4>/Delay1' */
  real32_T Integrator_DSTATE_g;        /* '<S153>/Integrator' */
  real32_T Delay3_DSTATE_g;            /* '<S4>/Delay3' */
  real32_T time_gate;                  /* '<S2>/Chart' */
  real32_T lock_time;                  /* '<S2>/Chart' */
  real32_T PrevY;                      /* '<S6>/Rate Limiter' */
  int8_T SwitchCase_ActiveSubsystem;   /* '<S1>/Switch Case' */
  int8_T Integrator_PrevResetState;    /* '<S267>/Integrator' */
  int8_T Integrator_PrevResetState_o;  /* '<S215>/Integrator' */
  int8_T Integrator_PrevResetState_d;  /* '<S49>/Integrator' */
  int8_T Integrator_PrevResetState_m;  /* '<S101>/Integrator' */
  int8_T Integrator_PrevResetState_h;  /* '<S153>/Integrator' */
  uint8_T is_active_c3_Intercepe_Strike_M;/* '<S2>/Chart' */
  uint8_T is_c3_Intercepe_Strike_Model_AS;/* '<S2>/Chart' */
  uint8_T DiscreteTimeIntegrator5_IC_LOAD;/* '<S178>/Discrete-Time Integrator5' */
  uint8_T DiscreteTimeIntegrator5_IC_LO_l;/* '<S176>/Discrete-Time Integrator5' */
  uint8_T DiscreteTimeIntegrator5_IC_LO_k;/* '<S175>/Discrete-Time Integrator5' */
  uint8_T DiscreteTimeIntegrator5_IC_LO_n;/* '<S177>/Discrete-Time Integrator5' */
  uint8_T DiscreteTimeIntegrator5_IC_LO_o;/* '<S10>/Discrete-Time Integrator5' */
  uint8_T DiscreteTimeIntegrator5_IC_L_ll;/* '<S11>/Discrete-Time Integrator5' */
  boolean_T icLoad;                    /* '<S6>/Delay3' */
  boolean_T icLoad_d;                  /* '<S5>/Delay1' */
  boolean_T icLoad_b;                  /* '<S5>/Delay2' */
  boolean_T icLoad_bb;                 /* '<S5>/Delay3' */
  boolean_T icLoad_bl;                 /* '<S4>/Delay1' */
  boolean_T icLoad_p;                  /* '<S4>/Delay3' */
  DW_Subsystem6_Intercepe_Strik_T Subsystem1;/* '<S1>/Subsystem1' */
  DW_Subsystem6_Intercepe_Strik_T Subsystem6;/* '<S1>/Subsystem6' */
} DW_Intercepe_Strike_Model_AS_T;

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

/*
 * Exported Global Parameters
 *
 * Note: Exported global parameters are tunable parameters with an exported
 * global storage class designation.  Code generation will declare the memory for
 * these parameters and exports their symbols.
 *
 */
extern STRIKE_PARAM_t STRIKE_PARAM;    /* Variable: STRIKE_PARAM
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

/* Model entry point functions */
extern void Intercepe_Strike_Model_AS_initialize(void);
extern void Intercepe_Strike_Model_AS_step(void);
extern void Intercepe_Strike_Model_AS_terminate(void);

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S10>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S11>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S4>/Gain' : Eliminated nontunable gain of 1
 * Block '<S41>/Kb' : Eliminated nontunable gain of 1
 * Block '<S93>/Kb' : Eliminated nontunable gain of 1
 * Block '<S145>/Kb' : Eliminated nontunable gain of 1
 * Block '<S175>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S176>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S177>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S178>/Data Type Conversion' : Eliminate redundant data type conversion
 * Block '<S207>/Kb' : Eliminated nontunable gain of 1
 * Block '<S259>/Kb' : Eliminated nontunable gain of 1
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
 * '<S3>'   : 'Intercepe_Strike_Model_AS/制导控制/Subsystem1'
 * '<S4>'   : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2'
 * '<S5>'   : 'Intercepe_Strike_Model_AS/制导控制/Subsystem3'
 * '<S6>'   : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4'
 * '<S7>'   : 'Intercepe_Strike_Model_AS/制导控制/Subsystem6'
 * '<S8>'   : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/Degrees to Radians'
 * '<S9>'   : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/Degrees to Radians1'
 * '<S10>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/First Order LPF'
 * '<S11>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/First Order LPF1'
 * '<S12>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller'
 * '<S13>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1'
 * '<S14>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3'
 * '<S15>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Anti-windup'
 * '<S16>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/D Gain'
 * '<S17>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/External Derivative'
 * '<S18>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Filter'
 * '<S19>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Filter ICs'
 * '<S20>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/I Gain'
 * '<S21>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Ideal P Gain'
 * '<S22>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Ideal P Gain Fdbk'
 * '<S23>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Integrator'
 * '<S24>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Integrator ICs'
 * '<S25>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/N Copy'
 * '<S26>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/N Gain'
 * '<S27>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/P Copy'
 * '<S28>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Parallel P Gain'
 * '<S29>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Reset Signal'
 * '<S30>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Saturation'
 * '<S31>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Saturation Fdbk'
 * '<S32>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Sum'
 * '<S33>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Sum Fdbk'
 * '<S34>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Tracking Mode'
 * '<S35>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Tracking Mode Sum'
 * '<S36>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Tsamp - Integral'
 * '<S37>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Tsamp - Ngain'
 * '<S38>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/postSat Signal'
 * '<S39>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/preInt Signal'
 * '<S40>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/preSat Signal'
 * '<S41>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Anti-windup/Back Calculation'
 * '<S42>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/D Gain/Disabled'
 * '<S43>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/External Derivative/Disabled'
 * '<S44>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Filter/Disabled'
 * '<S45>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Filter ICs/Disabled'
 * '<S46>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/I Gain/Internal Parameters'
 * '<S47>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Ideal P Gain/Passthrough'
 * '<S48>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S49>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Integrator/Discrete'
 * '<S50>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Integrator ICs/Internal IC'
 * '<S51>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/N Copy/Disabled wSignal Specification'
 * '<S52>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/N Gain/Disabled'
 * '<S53>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/P Copy/Disabled'
 * '<S54>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S55>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Reset Signal/External Reset'
 * '<S56>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Saturation/Enabled'
 * '<S57>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Saturation Fdbk/Disabled'
 * '<S58>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Sum/Sum_PI'
 * '<S59>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Sum Fdbk/Disabled'
 * '<S60>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Tracking Mode/Disabled'
 * '<S61>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S62>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S63>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S64>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/postSat Signal/Forward_Path'
 * '<S65>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/preInt Signal/Internal PreInt'
 * '<S66>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller/preSat Signal/Forward_Path'
 * '<S67>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Anti-windup'
 * '<S68>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/D Gain'
 * '<S69>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/External Derivative'
 * '<S70>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Filter'
 * '<S71>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Filter ICs'
 * '<S72>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/I Gain'
 * '<S73>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Ideal P Gain'
 * '<S74>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Ideal P Gain Fdbk'
 * '<S75>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Integrator'
 * '<S76>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Integrator ICs'
 * '<S77>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/N Copy'
 * '<S78>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/N Gain'
 * '<S79>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/P Copy'
 * '<S80>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Parallel P Gain'
 * '<S81>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Reset Signal'
 * '<S82>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Saturation'
 * '<S83>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Saturation Fdbk'
 * '<S84>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Sum'
 * '<S85>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Sum Fdbk'
 * '<S86>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Tracking Mode'
 * '<S87>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Tracking Mode Sum'
 * '<S88>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Tsamp - Integral'
 * '<S89>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Tsamp - Ngain'
 * '<S90>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/postSat Signal'
 * '<S91>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/preInt Signal'
 * '<S92>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/preSat Signal'
 * '<S93>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Anti-windup/Back Calculation'
 * '<S94>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/D Gain/Disabled'
 * '<S95>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/External Derivative/Disabled'
 * '<S96>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Filter/Disabled'
 * '<S97>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Filter ICs/Disabled'
 * '<S98>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/I Gain/Internal Parameters'
 * '<S99>'  : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Ideal P Gain/Passthrough'
 * '<S100>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S101>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Integrator/Discrete'
 * '<S102>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Integrator ICs/Internal IC'
 * '<S103>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/N Copy/Disabled wSignal Specification'
 * '<S104>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/N Gain/Disabled'
 * '<S105>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/P Copy/Disabled'
 * '<S106>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S107>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Reset Signal/External Reset'
 * '<S108>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Saturation/Enabled'
 * '<S109>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Saturation Fdbk/Disabled'
 * '<S110>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Sum/Sum_PI'
 * '<S111>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Sum Fdbk/Disabled'
 * '<S112>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Tracking Mode/Disabled'
 * '<S113>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S114>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S115>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S116>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/postSat Signal/Forward_Path'
 * '<S117>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/preInt Signal/Internal PreInt'
 * '<S118>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller1/preSat Signal/Forward_Path'
 * '<S119>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Anti-windup'
 * '<S120>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/D Gain'
 * '<S121>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/External Derivative'
 * '<S122>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Filter'
 * '<S123>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Filter ICs'
 * '<S124>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/I Gain'
 * '<S125>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Ideal P Gain'
 * '<S126>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Ideal P Gain Fdbk'
 * '<S127>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Integrator'
 * '<S128>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Integrator ICs'
 * '<S129>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/N Copy'
 * '<S130>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/N Gain'
 * '<S131>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/P Copy'
 * '<S132>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Parallel P Gain'
 * '<S133>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Reset Signal'
 * '<S134>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Saturation'
 * '<S135>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Saturation Fdbk'
 * '<S136>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Sum'
 * '<S137>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Sum Fdbk'
 * '<S138>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tracking Mode'
 * '<S139>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tracking Mode Sum'
 * '<S140>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tsamp - Integral'
 * '<S141>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tsamp - Ngain'
 * '<S142>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/postSat Signal'
 * '<S143>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/preInt Signal'
 * '<S144>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/preSat Signal'
 * '<S145>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Anti-windup/Back Calculation'
 * '<S146>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/D Gain/Disabled'
 * '<S147>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/External Derivative/Disabled'
 * '<S148>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Filter/Disabled'
 * '<S149>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Filter ICs/Disabled'
 * '<S150>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/I Gain/Internal Parameters'
 * '<S151>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Ideal P Gain/Passthrough'
 * '<S152>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Ideal P Gain Fdbk/Disabled'
 * '<S153>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Integrator/Discrete'
 * '<S154>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Integrator ICs/Internal IC'
 * '<S155>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/N Copy/Disabled wSignal Specification'
 * '<S156>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/N Gain/Disabled'
 * '<S157>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/P Copy/Disabled'
 * '<S158>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Parallel P Gain/Internal Parameters'
 * '<S159>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Reset Signal/External Reset'
 * '<S160>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Saturation/Enabled'
 * '<S161>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Saturation Fdbk/Disabled'
 * '<S162>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Sum/Sum_PI'
 * '<S163>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Sum Fdbk/Disabled'
 * '<S164>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tracking Mode/Disabled'
 * '<S165>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tracking Mode Sum/Passthrough'
 * '<S166>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tsamp - Integral/TsSignalSpecification'
 * '<S167>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/Tsamp - Ngain/Passthrough'
 * '<S168>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/postSat Signal/Forward_Path'
 * '<S169>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/preInt Signal/Internal PreInt'
 * '<S170>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem2/PID Controller3/preSat Signal/Forward_Path'
 * '<S171>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/Degrees to Radians'
 * '<S172>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/Degrees to Radians1'
 * '<S173>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/Degrees to Radians2'
 * '<S174>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/Degrees to Radians3'
 * '<S175>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/First Order LPF'
 * '<S176>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/First Order LPF1'
 * '<S177>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/First Order LPF2'
 * '<S178>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/First Order LPF3'
 * '<S179>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller'
 * '<S180>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1'
 * '<S181>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Anti-windup'
 * '<S182>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/D Gain'
 * '<S183>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/External Derivative'
 * '<S184>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Filter'
 * '<S185>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Filter ICs'
 * '<S186>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/I Gain'
 * '<S187>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Ideal P Gain'
 * '<S188>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Ideal P Gain Fdbk'
 * '<S189>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Integrator'
 * '<S190>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Integrator ICs'
 * '<S191>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/N Copy'
 * '<S192>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/N Gain'
 * '<S193>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/P Copy'
 * '<S194>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Parallel P Gain'
 * '<S195>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Reset Signal'
 * '<S196>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Saturation'
 * '<S197>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Saturation Fdbk'
 * '<S198>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Sum'
 * '<S199>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Sum Fdbk'
 * '<S200>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Tracking Mode'
 * '<S201>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Tracking Mode Sum'
 * '<S202>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Tsamp - Integral'
 * '<S203>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Tsamp - Ngain'
 * '<S204>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/postSat Signal'
 * '<S205>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/preInt Signal'
 * '<S206>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/preSat Signal'
 * '<S207>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Anti-windup/Back Calculation'
 * '<S208>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/D Gain/Disabled'
 * '<S209>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/External Derivative/Disabled'
 * '<S210>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Filter/Disabled'
 * '<S211>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Filter ICs/Disabled'
 * '<S212>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/I Gain/Internal Parameters'
 * '<S213>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Ideal P Gain/Passthrough'
 * '<S214>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Ideal P Gain Fdbk/Disabled'
 * '<S215>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Integrator/Discrete'
 * '<S216>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Integrator ICs/Internal IC'
 * '<S217>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/N Copy/Disabled wSignal Specification'
 * '<S218>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/N Gain/Disabled'
 * '<S219>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/P Copy/Disabled'
 * '<S220>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Parallel P Gain/Internal Parameters'
 * '<S221>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Reset Signal/External Reset'
 * '<S222>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Saturation/Enabled'
 * '<S223>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Saturation Fdbk/Disabled'
 * '<S224>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Sum/Sum_PI'
 * '<S225>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Sum Fdbk/Disabled'
 * '<S226>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Tracking Mode/Disabled'
 * '<S227>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Tracking Mode Sum/Passthrough'
 * '<S228>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Tsamp - Integral/TsSignalSpecification'
 * '<S229>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/Tsamp - Ngain/Passthrough'
 * '<S230>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/postSat Signal/Forward_Path'
 * '<S231>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/preInt Signal/Internal PreInt'
 * '<S232>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller/preSat Signal/Forward_Path'
 * '<S233>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Anti-windup'
 * '<S234>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/D Gain'
 * '<S235>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/External Derivative'
 * '<S236>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Filter'
 * '<S237>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Filter ICs'
 * '<S238>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/I Gain'
 * '<S239>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Ideal P Gain'
 * '<S240>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Ideal P Gain Fdbk'
 * '<S241>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Integrator'
 * '<S242>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Integrator ICs'
 * '<S243>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/N Copy'
 * '<S244>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/N Gain'
 * '<S245>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/P Copy'
 * '<S246>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Parallel P Gain'
 * '<S247>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Reset Signal'
 * '<S248>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Saturation'
 * '<S249>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Saturation Fdbk'
 * '<S250>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Sum'
 * '<S251>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Sum Fdbk'
 * '<S252>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Tracking Mode'
 * '<S253>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Tracking Mode Sum'
 * '<S254>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Tsamp - Integral'
 * '<S255>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Tsamp - Ngain'
 * '<S256>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/postSat Signal'
 * '<S257>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/preInt Signal'
 * '<S258>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/preSat Signal'
 * '<S259>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Anti-windup/Back Calculation'
 * '<S260>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/D Gain/Disabled'
 * '<S261>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/External Derivative/Disabled'
 * '<S262>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Filter/Disabled'
 * '<S263>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Filter ICs/Disabled'
 * '<S264>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/I Gain/Internal Parameters'
 * '<S265>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Ideal P Gain/Passthrough'
 * '<S266>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Ideal P Gain Fdbk/Disabled'
 * '<S267>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Integrator/Discrete'
 * '<S268>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Integrator ICs/Internal IC'
 * '<S269>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/N Copy/Disabled wSignal Specification'
 * '<S270>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/N Gain/Disabled'
 * '<S271>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/P Copy/Disabled'
 * '<S272>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Parallel P Gain/Internal Parameters'
 * '<S273>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Reset Signal/External Reset'
 * '<S274>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Saturation/Enabled'
 * '<S275>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Saturation Fdbk/Disabled'
 * '<S276>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Sum/Sum_PI'
 * '<S277>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Sum Fdbk/Disabled'
 * '<S278>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Tracking Mode/Disabled'
 * '<S279>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Tracking Mode Sum/Passthrough'
 * '<S280>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Tsamp - Integral/TsSignalSpecification'
 * '<S281>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/Tsamp - Ngain/Passthrough'
 * '<S282>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/postSat Signal/Forward_Path'
 * '<S283>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/preInt Signal/Internal PreInt'
 * '<S284>' : 'Intercepe_Strike_Model_AS/制导控制/Subsystem4/PID Controller1/preSat Signal/Forward_Path'
 * '<S285>' : 'Intercepe_Strike_Model_AS/制导状态机/Chart'
 */
#endif                                 /* Intercepe_Strike_Model_AS_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
