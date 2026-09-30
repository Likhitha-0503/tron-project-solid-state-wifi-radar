/**
  ******************************************************************************
  * @file    mcu_cache.h
  * @brief   Prototypes and inline implementations of MCU cache-handling functions
  ******************************************************************************
  */

#ifndef __MCU_CACHE_H
#define __MCU_CACHE_H

#include "stm32n6xx_hal.h"

__STATIC_FORCEINLINE int mcu_cache_enabled(void) {
#if defined (__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
   if (SCB->CCR & SCB_CCR_DC_Msk) return 1;
#endif
  return 0;
}

__STATIC_INLINE int mcu_cache_enable(void) {
    SCB_EnableDCache();
    return 0;
}

__STATIC_INLINE int mcu_cache_disable(void) {
    SCB_DisableDCache();
    return 0;
}

__STATIC_INLINE int mcu_cache_invalidate(void) {
    SCB_InvalidateDCache();
    return 0;
}

__STATIC_INLINE int mcu_cache_clean(void) {
    SCB_CleanDCache();
    return 0;
}

__STATIC_INLINE int mcu_cache_clean_invalidate(void) {
    SCB_CleanInvalidateDCache();
    return 0;
}

__STATIC_INLINE int mcu_cache_invalidate_range(uint32_t start_addr, uint32_t end_addr) {
    SCB_InvalidateDCache_by_Addr((uint32_t *)start_addr, (int32_t)(end_addr - start_addr));
    return 0;
}

__STATIC_INLINE int mcu_cache_clean_range(uint32_t start_addr, uint32_t end_addr) {
    SCB_CleanDCache_by_Addr((uint32_t *)start_addr, (int32_t)(end_addr - start_addr));
    return 0;
}

__STATIC_INLINE int mcu_cache_clean_invalidate_range(uint32_t start_addr, uint32_t end_addr) {
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)start_addr, (int32_t)(end_addr - start_addr));
    return 0;
}

__STATIC_INLINE void set_mcu_cache_state(uint8_t i_cache_state, uint8_t d_cache_state) {
    if (i_cache_state) SCB_EnableICache(); else SCB_DisableICache();
    if (d_cache_state) SCB_EnableDCache(); else SCB_DisableDCache();
}

#endif // __MCU_CACHE_H
