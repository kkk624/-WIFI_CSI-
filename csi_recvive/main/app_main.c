/* 接收端：纯混杂监听 + CSI 提取，通过 ESP-NOW 分片发送给汇总节点 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_csi_gain_ctrl.h"

#define CONFIG_LESS_INTERFERENCE_CHANNEL   11
#define CONFIG_WIFI_BANDWIDTH              WIFI_BW40
#define CONFIG_ESP_NOW_PHYMODE             WIFI_PHY_MODE_HT40
#define CONFIG_ESP_NOW_RATE                WIFI_PHY_RATE_MCS0_LGI
#define CONFIG_FORCE_GAIN                  0
#define CONFIG_GAIN_CONTROL                0

#define CSI_TOTAL_LEN   384
#define FRAG_SIZE       247           /* 修改：原248改为247，确保第一片头+数据=250字节 */

#define RECV_TIMEOUT_MS       2000
#define GAIN_RECALC_INTERVAL  10000

static const uint8_t SENDER_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t MY_MAC[]      = {0x1c, 0x00, 0x00, 0x00, 0x00, 0x00};
static const char *TAG = "csi_recv";

/* WiFi 初始化：纯 STA，不连接任何 AP */
static void wifi_init()
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    /* 修改：set_mac 移到 start 之前 */
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, MY_MAC));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, CONFIG_WIFI_BANDWIDTH));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
}

/* ESP-NOW 初始化 */
static void wifi_esp_now_init()
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));

    esp_now_peer_info_t peer = {
        .channel   = CONFIG_LESS_INTERFERENCE_CHANNEL,
        .ifidx     = WIFI_IF_STA,
        .encrypt   = false,
    };
    memcpy(peer.peer_addr, SENDER_MAC, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer));

    esp_now_rate_config_t rate_config = {
        .phymode = CONFIG_ESP_NOW_PHYMODE,
        .rate    = CONFIG_ESP_NOW_RATE,
        .ersu    = false,
        .dcm     = false
    };
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer.peer_addr, &rate_config));
}

static volatile uint32_t s_last_recv_tick = 0;
static volatile int s_total_count = 0;
static volatile int s_espnow_fail_count = 0;
static volatile int s_promisc_count = 0;

static void wifi_promiscuous_rx_cb(void *buf, wifi_promiscuous_pkt_type_t type)
{
    wifi_promiscuous_pkt_t *pkt = (wifi_promiscuous_pkt_t *)buf;
    s_promisc_count++;
    if (s_promisc_count % 50 == 0) {
        ESP_LOGI(TAG, "Promisc pkt: %d, type=%d, rssi=%d, src_mac=%02x:%02x:%02x:%02x:%02x:%02x",
                 s_promisc_count, type, pkt->rx_ctrl.rssi,
                 pkt->payload[6], pkt->payload[7], pkt->payload[8],
                 pkt->payload[9], pkt->payload[10], pkt->payload[11]);
    }
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) return;
    if (memcmp(info->mac, SENDER_MAC, 6) != 0) return;

    s_last_recv_tick = xTaskGetTickCount();
    s_total_count++;

    static int s_count = 0;
    s_count++;
    if (s_count % 100 == 0) {
        ESP_LOGI(TAG, "CSI triggered %d times, espnow_fail=%d", s_count, s_espnow_fail_count);
    }

    uint8_t raw_buf[CSI_TOTAL_LEN];
    memcpy(raw_buf, info->buf, info->len);

    /* 分片发送：修改 FRAG_SIZE=247，确保第一片 3+247=250 字节不超限 */
    uint8_t frag_buf[3 + FRAG_SIZE];
    uint16_t total_len = info->len;

    frag_buf[0] = 0;
    memcpy(frag_buf + 1, &total_len, 2);
    memcpy(frag_buf + 3, raw_buf, FRAG_SIZE);
    esp_err_t ret0 = esp_now_send(SENDER_MAC, frag_buf, 3 + FRAG_SIZE);
    if (ret0 != ESP_OK) {
        s_espnow_fail_count++;
        ESP_LOGW(TAG, "Frag0 send fail: %d", ret0);
    }

    frag_buf[0] = 1;
    int remain = total_len - FRAG_SIZE;
    memcpy(frag_buf + 3, raw_buf + FRAG_SIZE, remain);
    esp_err_t ret1 = esp_now_send(SENDER_MAC, frag_buf, 3 + remain);
    if (ret1 != ESP_OK) {
        s_espnow_fail_count++;
        ESP_LOGW(TAG, "Frag1 send fail: %d", ret1);
    }
}

/* CSI 初始化（关闭合并和滤波） */
static void wifi_csi_init()
{
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    esp_wifi_set_promiscuous_rx_cb(wifi_promiscuous_rx_cb);
    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = true,
        .ltf_merge_en      = false,
        .channel_filter_en = false,
        .manu_scale        = true,
        .shift             = false,
    };
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, NULL));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI(TAG, "CSI recv init start");

    wifi_init();
    wifi_esp_now_init();
    wifi_csi_init();

    ESP_LOGI(TAG, "CSI recv init done");
    // 启动后关闭日志
    esp_log_level_set("*", ESP_LOG_NONE);

    s_last_recv_tick = xTaskGetTickCount();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        uint32_t now = xTaskGetTickCount();

        if ((now - s_last_recv_tick) * portTICK_PERIOD_MS > RECV_TIMEOUT_MS) {
            ESP_LOGW(TAG, "No CSI data for %d ms, re-init CSI", RECV_TIMEOUT_MS);
            esp_wifi_set_csi(false);
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_wifi_set_csi(true);
            s_last_recv_tick = now;
        }
    }
}