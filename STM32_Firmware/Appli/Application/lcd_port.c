/**
 ******************************************************************************
 * @file    lcd_port.c
 * @brief   LTDC bring-up for the STM32N6570-DK (MB1860 RK050HR18 panel).
 *
 * Every constant below comes from ST's own code for this board:
 *   - pins, backlight, panel timing : STM32N6570-DK BSP (stm32n6570_discovery_lcd.c,
 *                                     rk050hr18.h)
 *   - PLL4 / IC16 = 25 MHz pixel clock, RIMC/RISUP settings, layer config :
 *                                     STM32CubeN6 example LTDC_Horizontal_Mirroring
 *
 * Prerequisites (see README_LCD.md):
 *   1. SystemClock_Config() enables PLL4 (HSI, M=1, N=25, P1=P2=1).
 *   2. The MCO1 output on PA8 is removed (PA8 is LTDC_B6).
 ******************************************************************************
 */
#include "stm32n6xx_hal.h"
#include "lcd_port.h"
#include <string.h>

/* RK050HR18 timing (pixels / lines) */
#define HSYNC  4U
#define HBP    4U
#define HFP    4U
#define VSYNC  4U
#define VBP    4U
#define VFP    4U

LTDC_HandleTypeDef hltdc;

uint16_t *LCD_GetFramebuffer(void)
{
    return (uint16_t *)LCD_FB_ADDR;
}

static void lcd_rif_config(void)
{
    __HAL_RCC_RIFSC_CLK_ENABLE();

    RIMC_MasterConfig_t m = {0};
    m.MasterCID = RIF_CID_1;
    m.SecPriv   = RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV;
    HAL_RIF_RIMC_ConfigMasterAttributes(RIF_MASTER_INDEX_LTDC1, &m);

    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDC,   RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
    HAL_RIF_RISC_SetSlaveSecureAttributes(RIF_RISC_PERIPH_INDEX_LTDCL1, RIF_ATTRIBUTE_SEC | RIF_ATTRIBUTE_PRIV);
}

static int lcd_clock_config(void)
{
    /* IC16 <- PLL4 (1600 MHz) / 64 = 25 MHz pixel clock */
    RCC_PeriphCLKInitTypeDef c = {0};
    c.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    c.LtdcClockSelection   = RCC_LTDCCLKSOURCE_IC16;
    c.ICSelection[RCC_IC16].ClockSelection = RCC_ICCLKSOURCE_PLL4;
    c.ICSelection[RCC_IC16].ClockDivider   = 64;
    return (HAL_RCCEx_PeriphCLKConfig(&c) == HAL_OK) ? 0 : -1;
}

static void lcd_gpio_config(void)
{
    GPIO_InitTypeDef g = {0};

    __HAL_RCC_LTDC_CLK_ENABLE();
    __HAL_RCC_LTDC_FORCE_RESET();
    __HAL_RCC_LTDC_RELEASE_RESET();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOQ_CLK_ENABLE();

    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF14_LCD;

    /* PA0 G3, PA1 G2, PA2 B7, PA7 B1, PA8 B6, PA15 R5 */
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOA, &g);
    /* PB2 B2, PB4 R3, PB11 G6, PB12 G5, PB13 CLK, PB14 HSYNC, PB15 G4 */
    g.Pin = GPIO_PIN_2 | GPIO_PIN_4 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &g);
    /* PD8 R7, PD9 R1, PD15 R2 */
    g.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &g);
    /* PE11 VSYNC */
    g.Pin = GPIO_PIN_11;
    HAL_GPIO_Init(GPIOE, &g);
    /* PG0 R0, PG1 G1, PG6 B3, PG8 G7, PG11 R6, PG12 G0, PG15 B0 */
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_6 | GPIO_PIN_8 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOG, &g);
    /* --- CHANGE THIS --- */
    /* PH3 B4, PH4 R4, PH6 B5 */
    g.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_6;
    HAL_GPIO_Init(GPIOH, &g);

    /* Panel control lines (plain outputs, as in the ST BSP) */
    g.Mode = GPIO_MODE_OUTPUT_PP;
    g.Alternate = 0;
    g.Pin = GPIO_PIN_1;                       /* PE1  LCD NRST (configured like ST does) */
    HAL_GPIO_Init(GPIOE, &g);
    g.Pin = GPIO_PIN_3 | GPIO_PIN_6;          /* PQ3 LCD_ONOFF, PQ6 backlight */
    HAL_GPIO_Init(GPIOQ, &g);
    g.Pull = GPIO_PULLUP;
    g.Pin = GPIO_PIN_13;                      /* PG13 display enable */
    HAL_GPIO_Init(GPIOG, &g);
}

int LCD_Init(void)
{
    LTDC_LayerCfgTypeDef layer = {0};

    lcd_rif_config();
    lcd_gpio_config();
    if (lcd_clock_config() != 0) return -1;

    /* black screen before the panel is switched on */
    memset((void *)LCD_FB_ADDR, 0, LCD_WIDTH * LCD_HEIGHT * 2U);

    hltdc.Instance = LTDC;
    hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
    hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
    hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
    hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
    hltdc.Init.HorizontalSync     = HSYNC - 1U;
    hltdc.Init.AccumulatedHBP     = HSYNC + HBP - 1U;
    hltdc.Init.AccumulatedActiveW = HSYNC + LCD_WIDTH + HBP - 1U;
    hltdc.Init.TotalWidth         = HSYNC + LCD_WIDTH + HBP + HFP - 1U;
    hltdc.Init.VerticalSync       = VSYNC - 1U;
    hltdc.Init.AccumulatedVBP     = VSYNC + VBP - 1U;
    hltdc.Init.AccumulatedActiveH = VSYNC + LCD_HEIGHT + VBP - 1U;
    hltdc.Init.TotalHeigh         = VSYNC + LCD_HEIGHT + VBP + VFP - 1U;
    hltdc.Init.Backcolor.Blue  = 0;
    hltdc.Init.Backcolor.Green = 0;
    hltdc.Init.Backcolor.Red   = 0;
    if (HAL_LTDC_Init(&hltdc) != HAL_OK) return -2;

    layer.WindowX0 = 0;
    layer.WindowX1 = LCD_WIDTH;
    layer.WindowY0 = 0;
    layer.WindowY1 = LCD_HEIGHT;
    layer.PixelFormat     = LTDC_PIXEL_FORMAT_RGB565;
    layer.Alpha           = 255;
    layer.Alpha0          = 0;
    layer.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
    layer.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
    layer.FBStartAdress   = LCD_FB_ADDR;
    layer.ImageWidth      = LCD_WIDTH;
    layer.ImageHeight     = LCD_HEIGHT;
    layer.Backcolor.Blue  = 0;
    layer.Backcolor.Green = 0;
    layer.Backcolor.Red   = 0;
    if (HAL_LTDC_ConfigLayer(&hltdc, &layer, LTDC_LAYER_1) != HAL_OK) return -3;

    __HAL_LTDC_ENABLE(&hltdc);

    /* panel on, display enable, backlight 100 % */
    HAL_GPIO_WritePin(GPIOQ, GPIO_PIN_3,  GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOQ, GPIO_PIN_6,  GPIO_PIN_SET);
    return 0;
}

/* Called by hm_update(); a no-op while the CPU D-cache is disabled (as it is
 * in this project today). Kept correct in case the cache is enabled later. */
void hm_cache_clean(void *addr, uint32_t size)
{
    SCB_CleanDCache_by_Addr((uint32_t *)addr, (int32_t)size);
}
