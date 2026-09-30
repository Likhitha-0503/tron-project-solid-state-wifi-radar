#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_rom_sys.h"
#include "esp_rom_crc.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_timer.h"
#include "telemetry_contract.h"

static const char *TAG = "SPI_BEAM_AVG";

static const uint8_t TARGET_MAC[6] = {0x68, 0x94, 0x23, 0xAA, 0x54, 0xDB}; //68-94-23-AA-54-DB

/* Set this to the ACTUAL Wi-Fi channel your target device is on.
 * Already confirmed working at channel 11 on your setup. */
#define WIFI_TARGET_CHANNEL 06

#define GPIO_MOSI 23
#define GPIO_MISO 19
#define GPIO_SCLK 18
#define GPIO_CS   27  // Safely moved off the strapping pin

#define GPIO_SWITCH_A 21
#define GPIO_SWITCH_B 22

#define BEAM_WINDOW_MS 200   // time each beam stays active before switching

/* ------------------------------------------------------------------ */
/* SPI SLAVE: KEEP-ALWAYS-LOADED QUEUE                                 */
/*                                                                      */
/* The old code did: queue one transaction -> wait for it to complete   */
/* -> vTaskDelay(5) -> queue the next one. Between "complete" and the  */
/* next "queue", and for the whole 5 ms delay, the slave peripheral    */
/* had NOTHING queued. The STM32 polls every 1 ms with no handshake    */
/* pin, so any poll landing in that gap got the bus idle state back -  */
/* all 0x00 or all 0xFF, which is exactly the garbage seen on the      */
/* STM32 side. This version keeps SPI_TXN_DEPTH transactions queued    */
/* at all times and re-arms each one the instant it completes, so the  */
/* slave is never empty.                                                */
/* ------------------------------------------------------------------ */
#define SPI_TXN_DEPTH 4

static DMA_ATTR WORD_ALIGNED_ATTR spi_telemetry_packet_t pkt_buf[SPI_TXN_DEPTH];
static spi_slave_transaction_t   txn[SPI_TXN_DEPTH];

// Per-beam accumulators
static volatile uint8_t  active_beam = 0;
static volatile int32_t  beam_rssi_sum[4]   = {0, 0, 0, 0};
static volatile uint32_t beam_match_count[4] = {0, 0, 0, 0};
static volatile int8_t   last_good_avg[4]    = {-99, -99, -99, -99}; // holds last window's real average per beam

/* Diagnostics only: total frames the promiscuous callback has SEEN vs.
 * how many actually matched TARGET_MAC, plus SPI transaction throughput,
 * printed on THIS board's own USB serial - independent of the STM32 link. */
static volatile uint32_t g_frames_seen = 0;
static volatile uint32_t g_frames_matched = 0;
static volatile uint32_t g_spi_txn_completed = 0;

static inline void set_rf_switch(uint8_t beam_id) {
    gpio_set_level(GPIO_SWITCH_A, beam_id & 0x01);
    gpio_set_level(GPIO_SWITCH_B, (beam_id >> 1) & 0x01);
}

void wifi_sniffer_cb(void *buf, wifi_promiscuous_pkt_type_t type) {
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;

    if (type != WIFI_PKT_DATA && type != WIFI_PKT_MGMT) {
        return;
    }
    g_frames_seen++;

    uint8_t *payload = pkt->payload;
    uint8_t *transmitter_mac = &payload[10];

    if (memcmp(transmitter_mac, TARGET_MAC, 6) == 0) {
        g_frames_matched++;
        // Attribute this reading to whichever beam is active RIGHT NOW.
        uint8_t b = active_beam;
        beam_rssi_sum[b] += pkt->rx_ctrl.rssi;
        beam_match_count[b]++;
    }
}

/* Prints real, measured data every second on the ESP32's own USB
 * serial - independent of the SPI link to the STM32. */
static void diag_task(void *arg)
{
    (void)arg;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "seen=%lu matched=%lu spi_txn=%lu | avg[0..3]=%d %d %d %d | active_beam=%u",
                 (unsigned long)g_frames_seen, (unsigned long)g_frames_matched,
                 (unsigned long)g_spi_txn_completed,
                 last_good_avg[0], last_good_avg[1], last_good_avg[2], last_good_avg[3],
                 active_beam);
        if (g_frames_matched == 0 && g_frames_seen > 0) {
            ESP_LOGW(TAG, "0 frames matched TARGET_MAC after %lu frames seen - "
                          "check WIFI_TARGET_CHANNEL and TARGET_MAC.",
                          (unsigned long)g_frames_seen);
        }
        if (g_spi_txn_completed == 0) {
            ESP_LOGW(TAG, "0 SPI transactions completed yet - the STM32 has not "
                          "successfully clocked a single full transfer. Check "
                          "wiring/CS/mode before looking at RSSI data.");
        }
    }
}

/* Load the freshest packet into slot i and (re)queue it. */
static void spi_arm_slot(int i, uint32_t sim_time)
{
    pkt_buf[i].timestamp  = sim_time;
    pkt_buf[i].beam_ID    = active_beam;
    pkt_buf[i].rssi_value = last_good_avg[active_beam];
    pkt_buf[i].crc16      = esp_rom_crc16_le(0x0000, (uint8_t*)&pkt_buf[i], 6);

    memset(&txn[i], 0, sizeof(txn[i]));
    txn[i].length    = sizeof(spi_telemetry_packet_t) * 8;
    txn[i].tx_buffer  = (uint8_t*)&pkt_buf[i];
    txn[i].rx_buffer  = NULL;

    esp_err_t qret = spi_slave_queue_trans(SPI2_HOST, &txn[i], portMAX_DELAY);
    if (qret != ESP_OK) {
        ESP_LOGE(TAG, "queue_trans slot %d failed: %d", i, qret);
    }
}

void app_main(void)
{
    esp_err_t ret;

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << GPIO_SWITCH_A) | (1ULL << GPIO_SWITCH_B),
        .pull_down_en = 0,
        .pull_up_en = 0
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_SWITCH_A, 1);
    gpio_set_level(GPIO_SWITCH_B, 1);

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_set_promiscuous_rx_cb(&wifi_sniffer_cb));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_set_channel(WIFI_TARGET_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_LOGI(TAG, "Sniffing channel %d, %d ms beam windows", WIFI_TARGET_CHANNEL, BEAM_WINDOW_MS);

    xTaskCreate(diag_task, "diag_task", 2560, NULL, 3, NULL);

    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = SPI_TXN_DEPTH,
        .flags = 0,
    };

    ret = spi_slave_initialize(SPI2_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "SPI Slave armed. Waiting for STM32 to poll (no handshake pin)...");

    uint32_t simulated_time = 0;
    set_rf_switch(active_beam);
    int64_t last_switch_time = esp_timer_get_time() / 1000;

    /* Pre-load every slot BEFORE the loop starts, so the slave has
     * SPI_TXN_DEPTH transactions queued from the very first STM32 poll. */
    for (int i = 0; i < SPI_TXN_DEPTH; i++) {
        spi_arm_slot(i, simulated_time += 500);
    }

    while (1) {
        int64_t current_time = esp_timer_get_time() / 1000;

        // Check if this beam's window has elapsed - finalize its average, switch beam
        if ((current_time - last_switch_time) >= BEAM_WINDOW_MS) {
            uint8_t b = active_beam;
            uint32_t matches = beam_match_count[b];
            if (matches > 0) {
                int32_t avg = beam_rssi_sum[b] / (int32_t)matches;
                if (avg > -1) avg = -1;      // keep within valid negative range for the packet contract
                if (avg < -99) avg = -99;
                last_good_avg[b] = (int8_t)avg;
            }

            beam_rssi_sum[b] = 0;
            beam_match_count[b] = 0;

            active_beam = (active_beam + 1) % 4;
            set_rf_switch(active_beam);
            last_switch_time = current_time;
        }

        /* Drain any transactions the STM32 has already completed, and
         * IMMEDIATELY re-arm those exact slots with fresh data. This is
         * the part that removes the "empty slave" gap: as long as at
         * least one of the SPI_TXN_DEPTH slots is always either still
         * queued or gets re-queued the instant it finishes, the bus
         * always has real data ready no matter when the STM32 polls. */
        for (int i = 0; i < SPI_TXN_DEPTH; i++) {
            spi_slave_transaction_t *done = NULL;
            esp_err_t gret = spi_slave_get_trans_result(SPI2_HOST, &done, 0);
            if (gret == ESP_OK && done != NULL) {
                g_spi_txn_completed++;
                int slot = (int)(done - txn);   /* which array slot completed */
                spi_arm_slot(slot, simulated_time += 500);
            }
            /* ESP_ERR_TIMEOUT here just means this slot hasn't been
             * polled by the STM32 yet - normal, not an error. */
        }

        vTaskDelay(1);   /* yield briefly to other tasks; the queue stays loaded regardless */
    }
}