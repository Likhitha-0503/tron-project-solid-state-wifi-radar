/**
 ******************************************************************************
 * @file    lcd_port.h
 * @brief   Minimal LTDC bring-up for the STM32N6570-DK 5" 800x480 RGB panel
 *          (RK050HR18), HAL only - no ST BSP needed. RGB565, one layer.
 ******************************************************************************
 */
#ifndef LCD_PORT_H
#define LCD_PORT_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define LCD_WIDTH      800U
#define LCD_HEIGHT     480U

/* Framebuffer: start of AXISRAM2 (768 000 bytes of its 1 MB).
 * Requires INTERNAL_RAM_SIZE = 0x000FFC00 in the T-Kernel STM32N6 sysdef.h
 * and a 512K RAM region in the linker script (see README_LCD.md). */
#ifndef LCD_FB_ADDR
#define LCD_FB_ADDR    0x34100000UL
#endif

/** Returns 0 on success, negative on failure (clock / LTDC HAL error). */
int       LCD_Init(void);
uint16_t *LCD_GetFramebuffer(void);

#ifdef __cplusplus
}
#endif
#endif /* LCD_PORT_H */
