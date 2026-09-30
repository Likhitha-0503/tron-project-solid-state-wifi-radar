/**
  ******************************************************************************
  * @file    polar.h
  * @author  STEdgeAI
  * @date    2026-09-18 20:35:36
  * @brief   Minimal description of the generated c-implemention of the network
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */
#ifndef LL_ATON_POLAR_H
#define LL_ATON_POLAR_H

/******************************************************************************/
#define LL_ATON_POLAR_C_MODEL_NAME        "polar"
#define LL_ATON_POLAR_ORIGIN_MODEL_NAME   "radar_10class_classifier"

/************************** USER ALLOCATED IOs ********************************/
// No user allocated inputs
// No user allocated outputs

/************************** INPUTS ********************************************/
#define LL_ATON_POLAR_IN_NUM        (1)    // Total number of input buffers
// Input buffer 1 -- Input_0_out_0
#define LL_ATON_POLAR_IN_1_ALIGNMENT   (32)
#define LL_ATON_POLAR_IN_1_SIZE_BYTES  (16)

/************************** OUTPUTS *******************************************/
#define LL_ATON_POLAR_OUT_NUM        (1)    // Total number of output buffers
// Output buffer 1 -- Softmax_6_out_0
#define LL_ATON_POLAR_OUT_1_ALIGNMENT   (32)
#define LL_ATON_POLAR_OUT_1_SIZE_BYTES  (40)

#endif /* LL_ATON_POLAR_H */
