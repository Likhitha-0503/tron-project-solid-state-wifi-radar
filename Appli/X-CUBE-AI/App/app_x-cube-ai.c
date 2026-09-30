#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "app_x-cube-ai.h"
#include "ll_aton.h"
#include "ll_aton_NN_interface.h"

/* Explicit declarations bypassing the macro */
extern bool LL_ATON_EC_Network_Init_polar(void);
extern bool LL_ATON_EC_Inference_Init_polar(void);

uint8_t *buffer_in;
uint8_t *buffer_out;

void set_clk_sleep_mode(void)
{
#if defined (CPU_IN_SECURE_STATE)
  __HAL_RCC_DBG_CLK_SLEEP_ENABLE();
#endif
  __HAL_RCC_XSPIPHYCOMP_CLK_SLEEP_ENABLE();

  __HAL_RCC_AXISRAM1_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM2_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM3_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM4_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM5_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_AXISRAM6_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_FLEXRAM_MEM_CLK_SLEEP_ENABLE();
  __HAL_RCC_CACHEAXIRAM_MEM_CLK_SLEEP_ENABLE();
#if defined (CPU_IN_SECURE_STATE)
  __HAL_RCC_RIFSC_CLK_SLEEP_ENABLE();
  __HAL_RCC_RISAF_CLK_SLEEP_ENABLE();
  __HAL_RCC_IAC_CLK_SLEEP_ENABLE();
#endif
  __HAL_RCC_XSPI1_CLK_SLEEP_ENABLE();
  __HAL_RCC_XSPI2_CLK_SLEEP_ENABLE();
  __HAL_RCC_CACHEAXI_CLK_SLEEP_ENABLE();
  __HAL_RCC_NPU_CLK_SLEEP_ENABLE();
  __HAL_RCC_USART1_CLK_SLEEP_ENABLE();
}

void MX_X_CUBE_AI_Init(void)
{
    LL_ATON_EC_Network_Init_polar();
    LL_ATON_EC_Inference_Init_polar();
}

void MX_X_CUBE_AI_Process(void)
{
    /* USER CODE BEGIN 6 */
    /* USER CODE END 6 */
}

#ifdef __cplusplus
}
#endif
