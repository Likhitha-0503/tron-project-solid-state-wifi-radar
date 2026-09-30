/**
  ******************************************************************************
  * @file    npu_cache.h
  * @brief   Stubs for NPU cache-handling functions
  ******************************************************************************
  */

#ifndef __NPU_CACHE_H
#define __NPU_CACHE_H

#include <stdint.h>
#include "stm32n6xx_hal.h"

__STATIC_INLINE int npu_cache_enabled(void) { return 0; }
__STATIC_INLINE int npu_cache_enable(void) { return 0; }
__STATIC_INLINE int npu_cache_disable(void) { return 0; }
__STATIC_INLINE int npu_cache_invalidate(void) { return 0; }
__STATIC_INLINE int npu_cache_clean(void) { return 0; }
__STATIC_INLINE int npu_cache_clean_invalidate(void) { return 0; }
__STATIC_INLINE int npu_cache_invalidate_range(uint32_t start_addr, uint32_t end_addr) { return 0; }
__STATIC_INLINE int npu_cache_clean_range(uint32_t start_addr, uint32_t end_addr) { return 0; }
__STATIC_INLINE int npu_cache_clean_invalidate_range(uint32_t start_addr, uint32_t end_addr) { return 0; }
__STATIC_INLINE void set_npu_cache_state(uint8_t i_cache_state, uint8_t d_cache_state) {}

#endif // __NPU_CACHE_H
