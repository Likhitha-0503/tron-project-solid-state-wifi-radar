/* ==================================================================== */
/*  usermain.c - FINAL: SILENT CAPTURE & CATEGORICAL ZONE LOGGING       */
/* ==================================================================== */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "stm32n6xx_hal.h"
#include "main.h"
#include "telemetry_contract.h"
#include "app_x-cube-ai.h"
#include "lcd_port.h"
#include "heatmap.h"

/* ========================================================== */
/* HARDWARE HANDLES & MASTER SWITCHES                         */
/* ========================================================== */
extern SPI_HandleTypeDef hspi5;
extern UART_HandleTypeDef huart1;

#define USE_HARDWARE_NPU 0

#define ENABLE_LCD            1
#define LCD_WAIT_FIRST_FRAME  1
#define LCD_START_TIMEOUT_MS  20000U
#define ENABLE_SPI_DIAG       1

void ARECH_Print(char *msg) {
    HAL_UART_Transmit(&huart1, (uint8_t*)msg, strlen(msg), 1000);
}

uint32_t HAL_GetTick(void) {
    SYSTIM systim;
    if (tk_get_tim(&systim) == E_OK) return systim.lo;
    return 0;
}

/* ========================================================== */
/* MATH & FILTERING                                           */
/* ========================================================== */
#define RSSI_MIN (-70.0f)
#define RSSI_MAX (-20.0f)
#define RSSI_RANGE_INV (1.0f / (RSSI_MAX - RSSI_MIN))

static inline float normalize_rssi(float raw_smoothed_rssi) {
    float normalized = (raw_smoothed_rssi - RSSI_MIN) * RSSI_RANGE_INV;
    if (normalized < 0.0f) return 0.0f;
    if (normalized > 1.0f) return 1.0f;
    return normalized;
}

static float k_est[4]     = {-70.0f, -70.0f, -70.0f, -70.0f};
static float k_err_est[4] = {1.0f, 1.0f, 1.0f, 1.0f};
static const float Q      = 0.05f;
static const float R      = 3.5f;

static int8_t smooth_rssi_by_beam(uint8_t beam, int8_t new_rssi) {
    if (beam > 3) return new_rssi;
    k_err_est[beam] = k_err_est[beam] + Q;
    float kalman_gain = k_err_est[beam] / (k_err_est[beam] + R);
    k_est[beam] = k_est[beam] + kalman_gain * ((float)new_rssi - k_est[beam]);
    k_err_est[beam] = (1.0f - kalman_gain) * k_err_est[beam];
    return (int8_t)k_est[beam];
}

/* ========================================================== */
/* GLOBAL BUFFERS & RTOS OBJECTS                              */
/* ========================================================== */
#define BANK_A 0
#define BANK_B 1

__attribute__((aligned(32))) static float sweep_buffers[2][4];
static int8_t   raw_rssi_buffers[2][4];
static uint8_t  active_write_bank = BANK_A;
static uint8_t  beams_received_mask = 0;

static ID wifi_frame_flg_id;
static ID telemetry_mbf_id;

#define FLG_BANK_A_READY   0x00000001
#define FLG_BANK_B_READY   0x00000002

static volatile uint8_t g_first_sweep = 0;

static void tele_send(const char *buf, size_t bufsz, int len)
{
    if (len <= 0) return;
    if ((size_t)len >= bufsz) len = (int)bufsz - 1;
    if (len > 127) len = 127;
    tk_snd_mbf(telemetry_mbf_id, (void*)buf, (INT)len, TMO_POL);
}
static void tele_puts(const char *s)
{
    tele_send(s, strlen(s) + 1U, (int)strlen(s));
}

#if ENABLE_LCD
typedef struct {
    volatile uint32_t seq;
    float    beam_norm[4];
    int8_t   beam_rssi[4];
    float    theta_deg;
    float    dist_m;
    uint8_t  zone;
} lcd_snapshot_t;
static lcd_snapshot_t lcd_snap;
#endif

/* ========================================================== */
/* TASK 1: CAPTURE                                            */
/* ========================================================== */
LOCAL void high_priority_capture_task(INT stacd, void *exinf)
{
    ARECH_Print("[SYS] Data Capture Task Online.\r\n");
    ARECH_Print("[SYS] Waiting 6 seconds for ESP32 Wi-Fi lock...\r\n");
    tk_dly_tsk(6000);

    ARECH_Print("[SYS] Wi-Fi lock assumed. Initializing CS line (PA3/D10)...\r\n");
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

    ARECH_Print("[SYS] Engaging SPI polling loop (Silent Capture Mode)...\r\n");

    spi_telemetry_packet_t rx_packet;
    uint8_t dummy_tx[8] = {0};
    uint8_t last_processed_beam = 99;

    uint32_t n_poll = 0, n_err = 0, n_bad = 0, n_dup = 0, n_ok = 0;
    uint32_t t_diag = HAL_GetTick();
    memset(&rx_packet, 0, sizeof(rx_packet));

    while(1) {
        tk_dly_tsk(1);

        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
        HAL_StatusTypeDef spi_status = HAL_SPI_TransmitReceive(&hspi5, dummy_tx, (uint8_t*)&rx_packet, 8, 10);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

        n_poll++;
        if (spi_status != HAL_OK) { n_err++; continue; }

#if ENABLE_SPI_DIAG
        uint32_t now = HAL_GetTick();
        if ((now - t_diag) >= 5000U) {
            t_diag = now;
            const uint8_t *rb = (const uint8_t *)&rx_packet;
            char dmsg[128];
            int dl = snprintf(dmsg, sizeof(dmsg),
                "[SPI DIAG] p=%lu ok=%lu e=%lu bad=%lu dup=%lu raw=%02X%02X%02X%02X%02X%02X%02X%02X\r\n",
                (unsigned long)n_poll, (unsigned long)n_ok, (unsigned long)n_err,
                (unsigned long)n_bad, (unsigned long)n_dup,
                (unsigned)rb[0], (unsigned)rb[1], (unsigned)rb[2], (unsigned)rb[3],
                (unsigned)rb[4], (unsigned)rb[5], (unsigned)rb[6], (unsigned)rb[7]);
            tele_send(dmsg, sizeof(dmsg), dl);
        }
#endif

        int valid = (rx_packet.beam_ID < 4) && (rx_packet.rssi_value <= 0) && (rx_packet.rssi_value >= -100);
        if (!valid) { n_bad++; continue; }

        if (rx_packet.beam_ID == last_processed_beam) { n_dup++; continue; }
        last_processed_beam = rx_packet.beam_ID;
        n_ok++;

        int8_t smoothed_rssi = smooth_rssi_by_beam(rx_packet.beam_ID, rx_packet.rssi_value);
        float normalized_rssi = normalize_rssi((float)smoothed_rssi);
        raw_rssi_buffers[active_write_bank][rx_packet.beam_ID] = rx_packet.rssi_value;
        sweep_buffers[active_write_bank][rx_packet.beam_ID] = normalized_rssi;
        beams_received_mask |= (1 << rx_packet.beam_ID);

        /* SILENT PROCESSING: Individual ESP32 frame logs have been removed to stop terminal spam */
        if (beams_received_mask == 0x0F) {
            beams_received_mask = 0;
            g_first_sweep = 1;
            if (active_write_bank == BANK_A) {
                active_write_bank = BANK_B;
                tk_set_flg(wifi_frame_flg_id, FLG_BANK_A_READY);
            } else {
                active_write_bank = BANK_A;
                tk_set_flg(wifi_frame_flg_id, FLG_BANK_B_READY);
            }
        }
    }
}

/* ========================================================== */
/* TASK 2: NPU LOCALIZATION ENGINE & SENSOR FUSION            */
/* ========================================================== */
LOCAL void med_priority_localization_task(INT stacd, void *exinf)
{
    UINT flgptn;
    ARECH_Print("[NPU] 2D Kalman Tracking Engine Online...\r\n");

#if USE_HARDWARE_NPU
    MX_X_CUBE_AI_Init();
    uint8_t* safe_buffer_in = (uint8_t*)0x342e0000UL;
    uint8_t* safe_buffer_out = (uint8_t*)(0x342e0000UL + 1040);
#endif

    while(1) {
        tk_wai_flg(wifi_frame_flg_id, (FLG_BANK_A_READY | FLG_BANK_B_READY),
                   TWF_ORW | TWF_BITCLR, &flgptn, TMO_FEVR);

        uint8_t bank = (flgptn & FLG_BANK_A_READY) ? BANK_A : BANK_B;

        SYSTIM t_start, t_end;
        tk_get_tim(&t_start);

        int max_index = 0;
        float max_prob = 99.0f;
        char dist_buffer[16];
        char *angle_str;

        float hm_theta = 0.0f;
        float hm_dist = 0.0f;

        int8_t max_rssi = -128;
        for (int i = 0; i < 4; i++) {
            if (raw_rssi_buffers[bank][i] > max_rssi) {
                max_rssi = raw_rssi_buffers[bank][i];
            }
        }

        if (max_rssi < -85) {
            continue;
        }

#if USE_HARDWARE_NPU
        memcpy(safe_buffer_in, sweep_buffers[bank], 16);
        MX_X_CUBE_AI_Process();
        float out_data[10];
        memcpy(out_data, safe_buffer_out, 40);

        float current_max = 0.0f;
        for (int i = 0; i < 10; i++) {
            if (out_data[i] > current_max) {
                current_max = out_data[i];
                max_index = i;
            }
        }

        max_prob = current_max * 100.0f;

        hm_dist = (max_index < 5) ? 3.0f : 4.5f;
        if (max_index % 5 == 0) hm_theta = 45.0f;
        else if (max_index % 5 == 1) hm_theta = 15.0f;
        else if (max_index % 5 == 2) hm_theta = 0.0f;
        else if (max_index % 5 == 3) hm_theta = -15.0f;
        else hm_theta = -45.0f;

        snprintf(dist_buffer, sizeof(dist_buffer), "%s", (max_index < 5) ? "<= 3.5m" : "> 3.5m");

#else
        float max_norm = -1.0f;
        int best_beam = 0;
        int second_best = 0;

        for (int i = 0; i < 4; i++) {
            if (sweep_buffers[bank][i] > max_norm) {
                max_norm = sweep_buffers[bank][i];
                second_best = best_beam;
                best_beam = i;
            }
        }

        max_prob = (max_norm > 0) ? (max_norm * 100.0f) : 99.0f;

        float beam_angles[4] = {45.0f, 15.0f, -15.0f, -45.0f};
        float raw_theta = beam_angles[best_beam];
        if (max_norm > 0.1f && (sweep_buffers[bank][second_best] > max_norm * 0.8f)) {
             raw_theta = (beam_angles[best_beam] + beam_angles[second_best]) / 2.0f;
        }

        float PLE = 2.8f;
        float ref_distance = 2.0f;
        float ref_rssi = -45.0f;

        float raw_d = ref_distance * powf(10.0f, (ref_rssi - (float)max_rssi) / (10.0f * PLE));

        if (max_rssi >= -30) {
            raw_d = 0.50f;
        } else {
            if (raw_d > 10.0f) raw_d = 10.0f;
            if (raw_d < 0.5f) raw_d = 0.5f;
        }

        static float dt = 0.1f;
        static float kf_theta = 0.0f, kf_v_theta = 0.0f;
        static float kf_d = 2.0f,     kf_v_d = 0.0f;

        static float P_th[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
        static float P_d[2][2]  = {{1.0f, 0.0f}, {0.0f, 1.0f}};

        float Q_pos = 0.05f;
        float Q_vel = 0.005f;
        float R_th  = 15.0f;
        float R_d   = 2.0f;

        kf_v_theta *= 0.70f;
        kf_v_d *= 0.70f;

        kf_theta = kf_theta + kf_v_theta * dt;
        P_th[0][0] += dt * (P_th[1][0] + P_th[0][1]) + dt * dt * P_th[1][1] + Q_pos;
        P_th[0][1] += dt * P_th[1][1];
        P_th[1][0] += dt * P_th[1][1];
        P_th[1][1] += Q_vel;

        float S_th = P_th[0][0] + R_th;
        float K_th[2] = {P_th[0][0] / S_th, P_th[1][0] / S_th};
        float y_th = raw_theta - kf_theta;

        kf_theta += K_th[0] * y_th;
        kf_v_theta += K_th[1] * y_th;
        P_th[0][0] = (1.0f - K_th[0]) * P_th[0][0];
        P_th[0][1] = (1.0f - K_th[0]) * P_th[0][1];
        P_th[1][0] = -K_th[1] * P_th[0][0] + P_th[1][0];
        P_th[1][1] = -K_th[1] * P_th[0][1] + P_th[1][1];

        kf_d = kf_d + kf_v_d * dt;
        P_d[0][0] += dt * (P_d[1][0] + P_d[0][1]) + dt * dt * P_d[1][1] + Q_pos;
        P_d[0][1] += dt * P_d[1][1];
        P_d[1][0] += dt * P_d[1][1];
        P_d[1][1] += Q_vel;

        float y_d = raw_d - kf_d;

        if (y_d > 4.0f) y_d = 4.0f;
        if (y_d < -4.0f) y_d = -4.0f;

        float S_d = P_d[0][0] + R_d;
        float K_d[2] = {P_d[0][0] / S_d, P_d[1][0] / S_d};

        kf_d += K_d[0] * y_d;
        kf_v_d += K_d[1] * y_d;

        P_d[0][0] = (1.0f - K_d[0]) * P_d[0][0];
        P_d[0][1] = (1.0f - K_d[0]) * P_d[0][1];
        P_d[1][0] = -K_d[1] * P_d[0][0] + P_d[1][0];
        P_d[1][1] = -K_d[1] * P_d[0][1] + P_d[1][1];

        if (kf_d < 0.5f) kf_d = 0.5f;

        if (kf_theta > 30.0f) max_index = 0;
        else if (kf_theta > 5.0f) max_index = 1;
        else if (kf_theta > -5.0f) max_index = 2;
        else if (kf_theta > -30.0f) max_index = 3;
        else max_index = 4;

        /* CATEGORICAL FORMATTING: Limits output to <= 3.5m or > 3.5m based on the Kalman distance */
        snprintf(dist_buffer, sizeof(dist_buffer), "%s", (kf_d <= 3.5f) ? "<= 3.5m" : "> 3.5m");

        hm_theta = kf_theta;
        hm_dist = kf_d;
#endif

        int zone_idx = max_index % 5;
        switch(zone_idx) {
            case 0: angle_str = "+45 deg (Zone 1)"; break;
            case 1: angle_str = "+15 deg (Zone 2)"; break;
            case 2: angle_str = "  0 deg (Zone 3)"; break;
            case 3: angle_str = "-15 deg (Zone 4)"; break;
            case 4: angle_str = "-45 deg (Zone 5)"; break;
            default: angle_str = "Unknown"; break;
        }

        tk_get_tim(&t_end);
        UINT latency_ms = t_end.lo - t_start.lo;

        char msg[128];
        INT len = snprintf(msg, sizeof(msg),
            "[NPU TRACK] Target: %s at %s | Conf: %d%% | Latency: %lu ms\r\n",
            dist_buffer, angle_str, (int)max_prob, (unsigned long)latency_ms);
        tele_send(msg, sizeof(msg), len);

#if ENABLE_LCD
        lcd_snap.seq++;  __DMB();
        for (int i = 0; i < 4; i++) {
            lcd_snap.beam_norm[i] = sweep_buffers[bank][i];
            lcd_snap.beam_rssi[i] = raw_rssi_buffers[bank][i];
        }
        lcd_snap.theta_deg = hm_theta;
        lcd_snap.dist_m    = hm_dist;
        lcd_snap.zone      = (uint8_t)zone_idx;
        __DMB();  lcd_snap.seq++;
#endif
    }
}

/* ========================================================== */
/* TASK 3: TELEMETRY & SERIAL LOGGING                         */
/* ========================================================== */
LOCAL void low_priority_telemetry_task(INT stacd, void *exinf)
{
    char rx_buffer[129];
    INT msg_len;
    ARECH_Print("[SYS] Telemetry Pipeline Armed.\r\n");

    while(1) {
        msg_len = tk_rcv_mbf(telemetry_mbf_id, (void*)rx_buffer, TMO_FEVR);
        if (msg_len > 0) {
            rx_buffer[msg_len] = '\0';
            ARECH_Print(rx_buffer);
        }
    }
}

#if ENABLE_LCD
/* ========================================================== */
/* TASK 4: LCD DISPLAY & HEATMAP RENDER ENGINE                */
/* ========================================================== */
static int lcd_snapshot_read(hm_frame_t *out, uint32_t *seq_out)
{
    for (int tries = 0; tries < 4; tries++) {
        uint32_t s1 = lcd_snap.seq;
        if (s1 == 0 || (s1 & 1U)) continue;
        __DMB();
        for (int i = 0; i < 4; i++) {
            out->beam_norm[i] = lcd_snap.beam_norm[i];
            out->beam_rssi[i] = lcd_snap.beam_rssi[i];
        }
        out->theta_deg = lcd_snap.theta_deg;
        out->dist_m    = lcd_snap.dist_m;
        out->zone      = lcd_snap.zone;
        out->valid     = 1;
        __DMB();
        if (lcd_snap.seq == s1) { *seq_out = s1; return 1; }
    }
    return 0;
}

LOCAL void display_task(INT stacd, void *exinf)
{
#if LCD_WAIT_FIRST_FRAME
    tele_puts("[LCD] Waiting for first SPI sweep before starting LTDC...\r\n");
    {
        UINT waited = 0;
        while (!g_first_sweep && waited < LCD_START_TIMEOUT_MS) {
            tk_dly_tsk(100);
            waited += 100;
        }
        tele_puts(g_first_sweep ? "[LCD] First sweep seen - starting LTDC.\r\n"
                                : "[LCD] Timeout, NO SPI data - starting LTDC anyway.\r\n");
    }
#endif

    tele_puts("[LCD] Initialising LTDC...\r\n");
    if (LCD_Init() != 0) {
        tele_puts("[LCD] LCD_Init FAILED.\r\n");
        for (;;) tk_slp_tsk(TMO_FEVR);
    }

    hm_init(LCD_GetFramebuffer());
    tele_puts("[LCD] Heat map running.\r\n");

    hm_frame_t cur, tmp;
    uint32_t last_seq = 0, seq = 0;
    UINT age_ms = 0;
    int have = 0;

    for (;;) {
        tk_dly_tsk(66);

        if (lcd_snapshot_read(&tmp, &seq) && seq != last_seq) {
            last_seq = seq;
            cur = tmp;
            have = 1;
            age_ms = 0;
        } else {
            age_ms += 66;
        }

        hm_update((have && age_ms < 3000) ? &cur : NULL);
    }
}
#endif

/* ==================================================================== */
/* MAIN RTOS ENTRY POINT                                                */
/* ==================================================================== */
LOCAL ID tskid_capture, tskid_radar, tskid_telemetry;
#if ENABLE_LCD
LOCAL ID tskid_display;
#endif

LOCAL T_CTSK ctsk_capture   = { .itskpri = 10, .stksz = 4096,  .task = high_priority_capture_task,     .tskatr = TA_HLNG | TA_RNG0 };
LOCAL T_CTSK ctsk_radar     = { .itskpri = 5,  .stksz = 32768, .task = med_priority_localization_task, .tskatr = TA_HLNG | TA_RNG0 };
LOCAL T_CTSK ctsk_telemetry = { .itskpri = 20, .stksz = 1024,  .task = low_priority_telemetry_task,    .tskatr = TA_HLNG | TA_RNG0 };
#if ENABLE_LCD
LOCAL T_CTSK ctsk_display   = { .itskpri = 30, .stksz = 8192,  .task = display_task,                   .tskatr = TA_HLNG | TA_RNG0 };
#endif

EXPORT INT usermain(void)
{
    SystemCoreClockUpdate();
    HAL_UART_Init(&huart1);
    __HAL_RCC_CRC_CLK_ENABLE();

    ARECH_Print("\r\n=== µT-Kernel RTOS INITIALIZING ===\r\n");
    char msg[64];
    snprintf(msg, sizeof(msg), "CPU Clock Locked: %lu MHz\r\n", SystemCoreClock / 1000000);
    ARECH_Print(msg);

    T_CFLG cflg;
    cflg.exinf   = NULL;
    cflg.flgatr  = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0;
    wifi_frame_flg_id = tk_cre_flg(&cflg);

    T_CMBF cmbf;
    cmbf.exinf  = NULL;
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = 1024;
    cmbf.maxmsz = 128;
    telemetry_mbf_id = tk_cre_mbf(&cmbf);

    tskid_capture   = tk_cre_tsk(&ctsk_capture);
    tskid_radar     = tk_cre_tsk(&ctsk_radar);
    tskid_telemetry = tk_cre_tsk(&ctsk_telemetry);
#if ENABLE_LCD
    tskid_display   = tk_cre_tsk(&ctsk_display);
#endif

    tk_sta_tsk(tskid_capture, 0);
    tk_sta_tsk(tskid_radar, 0);
    tk_sta_tsk(tskid_telemetry, 0);
#if ENABLE_LCD
    tk_sta_tsk(tskid_display, 0);
#endif

    ARECH_Print("[SYS] RTOS Scheduler Active. Handing over to Tasks.\r\n\n");
    tk_slp_tsk(TMO_FEVR);
    return 0;
}
