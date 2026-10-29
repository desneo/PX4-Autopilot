/*
 * File: Intercepe_Strike_Model_AS_data.c
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

/* Invariant block signals (default storage) */
const ConstB_Intercepe_Strike_Model_T Intercepe_Strike_Model_A_ConstB = {
  { 0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F, -1.0F, 0.0F, 0.0F },/* '<S78>/Transpose' */

  { 0.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F, -1.0F, 0.0F, 0.0F }/* '<S14>/Transpose' */
};

/* Constant parameters (default storage) */
const ConstP_Intercepe_Strike_Model_T Intercepe_Strike_Model_A_ConstP = {
  /* Pooled Parameter (Expression: [0 0 1;0 1 0;-1 0 0])
   * Referenced by:
   *   '<S14>/Constant'
   *   '<S78>/Constant'
   */
  { 0.0F, 0.0F, -1.0F, 0.0F, 1.0F, 0.0F, 1.0F, 0.0F, 0.0F },

  /* Pooled Parameter (Expression: single([0;0;9.81]))
   * Referenced by:
   *   '<S15>/Constant'
   *   '<S79>/Constant'
   */
  { 0.0F, 0.0F, 9.81F }
};

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
