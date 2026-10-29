/*
 * File: Intercepe_Strike_Model_AS_private.h
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

#ifndef Intercepe_Strike_Model_AS_private_h_
#define Intercepe_Strike_Model_AS_private_h_
#include "rtwtypes.h"
#include "Intercepe_Strike_Model_AS_types.h"
#include "Intercepe_Strike_Model_AS.h"

extern void Intercepe_Strike_MATLABFunction(real32_T rtu_angles_rad, real32_T
  rtu_angles_rad_k, real32_T rtu_angles_rad_j, real32_T rty_DCM[9]);
extern void Intercepe_Strike_Mod_Subsystem6(boolean_T rtu_cmd_valid,
  VehicleCommandBus *rty_Out1, DW_Subsystem6_Intercepe_Strik_T *localDW);
extern void Intercepe_MATLABFunction_h_Init(DW_MATLABFunction_Intercepe_h_T
  *localDW);
extern void Intercep_MATLABFunction_k_Reset(DW_MATLABFunction_Intercepe_h_T
  *localDW);
extern void Intercepe_Stri_MATLABFunction_f(real32_T rtu_u, real_T rtu_fc,
  real_T rtu_Ts, real32_T *rty_y, DW_MATLABFunction_Intercepe_h_T *localDW);
extern void Intercepe_Strik_MATLABFunction1(const real32_T rtu_angles_rad[3],
  real32_T rty_DCM[9]);
extern void Intercep_MATLABFunction1_a_Init(DW_MATLABFunction1_Intercep_g_T
  *localDW);
extern void Interce_MATLABFunction1_j_Reset(DW_MATLABFunction1_Intercep_g_T
  *localDW);
extern void Intercepe_Str_MATLABFunction1_g(real32_T rtu_u, real32_T rtu_fc,
  real32_T rtu_Ts, real32_T *rty_y, DW_MATLABFunction1_Intercep_g_T *localDW);
extern void Intercepe__MATLABFunction4_Init(DW_MATLABFunction4_Intercepe__T
  *localDW);
extern void Intercepe_MATLABFunction4_Reset(DW_MATLABFunction4_Intercepe__T
  *localDW);
extern void Intercepe_Strik_MATLABFunction4(real32_T rtu_u, real32_T rtu_fc,
  real32_T rtu_Ts, real32_T *rty_y, DW_MATLABFunction4_Intercepe__T *localDW);

#endif                                /* Intercepe_Strike_Model_AS_private_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
