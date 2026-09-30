/**
 ******************************************************************************
 * @file    heatmap.h
 * @brief   WiFi-source heat map renderer (radar fan) for an 800x480 RGB565
 *          framebuffer. Hardware independent: no HAL / RTOS dependency, so the
 *          same file builds for the Cortex-M55 target and for a host PC.
 ******************************************************************************
 */
#ifndef HEATMAP_H
#define HEATMAP_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HM_SCREEN_W   800
#define HM_SCREEN_H   480

/* Maximum distance shown at the outer arc of the fan (metres).
 * Rings are drawn every 2 m. Distances above this are clamped to the arc. */
#ifndef HM_RANGE_M
#define HM_RANGE_M    8.0f
#endif

/* 0: positive angles are drawn to the RIGHT of the sensor axis.
 * 1: positive angles are drawn to the LEFT (flip if your +45 beam is on the
 *    other side of the room). */
#ifndef HM_MIRROR
#define HM_MIRROR     0
#endif

/* Persistence: heat *= HM_DECAY_Q8/256 every frame (224/256 = 0.875). */
#ifndef HM_DECAY_Q8
#define HM_DECAY_Q8   224
#endif

/** One measurement snapshot, published by the localization task. */
typedef struct {
    float   beam_norm[4];   /* smoothed, normalised RSSI per beam, 0..1        */
    int8_t  beam_rssi[4];   /* raw RSSI per beam in dBm (shown as text)        */
    float   theta_deg;      /* Kalman-filtered bearing, +45 .. -45             */
    float   dist_m;         /* Kalman-filtered distance in metres              */
    uint8_t zone;           /* 0..4  ->  Zone 1..5  (+45,+15,0,-15,-45 deg)    */
    uint8_t valid;          /* 0 = no fresh data (heat only decays)            */
} hm_frame_t;

/** Build look-up tables, clear the screen, draw the static background. */
void hm_init(uint16_t *framebuffer);

/** Redraw the heat map. Pass NULL (or frame->valid==0) to only let it fade. */
void hm_update(const hm_frame_t *frame);

/** 8 colour bars + border + gradient. Use for the first LCD bring-up test. */
void hm_test_pattern(uint16_t *framebuffer);

/** Optional hook, called after drawing with the framebuffer range.
 *  Default (weak) implementation does nothing. Override it with
 *  SCB_CleanDCache_by_Addr() if the CPU D-cache is ever enabled. */
void hm_cache_clean(void *addr, uint32_t size);

#ifdef __cplusplus
}
#endif
#endif /* HEATMAP_H */
