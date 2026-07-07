/* 发送端/汇总节点：ESP-NOW 发送测量包 + 接收 CSI 分片并串口输出 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "driver/gpio.h"           // 如需调试 LED 可加入
#include "nvs_flash.h"
#include "esp_mac.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_now.h"

#define CONFIG_LESS_INTERFERENCE_CHANNEL   11
#define CONFIG_WIFI_BANDWIDTH              WIFI_BW40
#define CONFIG_ESP_NOW_PHYMODE             WIFI_PHY_MODE_HT40
#define CONFIG_ESP_NOW_RATE                WIFI_PHY_RATE_MCS0_LGI
#define CONFIG_SEND_FREQUENCY              50
#define CONFIG_UART_BAUDRATE               921600

static const uint8_t CONFIG_CSI_SEND_MAC[] = {0x1a, 0x00, 0x00, 0x00, 0x00, 0x00};
static const uint8_t CONFIG_CSI_RECV_MAC[] = {0x1c, 0x00, 0x00, 0x00, 0x00, 0x00};  // 接收端 MAC
static const char *TAG = "csi_send";

#define CSI_TOTAL_LEN   384
#define FRAG_SIZE       247           // 3 + 247 = 250，不超 ESP-NOW 限制
#define FRAG_TIMEOUT_MS 100

typedef struct {
    uint8_t mac[6];
    uint8_t buf[CSI_TOTAL_LEN];
    uint8_t received_frags;
    uint16_t total_len;
    uint32_t first_frag_tick;
} csi_frag_ctx_t;

typedef struct {
    uint8_t mac[6];
    uint8_t buf[CSI_TOTAL_LEN];
    uint32_t seq;
} csi_output_item_t;

static csi_frag_ctx_t frag_ctx = {0};
static bool frag_ctx_valid = false;
static QueueHandle_t s_output_queue = NULL;
static volatile uint32_t s_csi_frame_count = 0;
static volatile uint32_t s_frag_discard_count = 0;

/* 可选：用 GPIO 指示收包状态 */
#define DEBUG_LED_GPIO  GPIO_NUM_2
static void debug_led_init(void) {
    gpio_reset_pin(DEBUG_LED_GPIO);
    gpio_set_direction(DEBUG_LED_GPIO, GPIO_MODE_OUTPUT);
}
static void debug_led_toggle(void) {
    static int level = 0;
    gpio_set_level(DEBUG_LED_GPIO, level);
    level = !level;
}

/* ESP-NOW 接收回调：处理 CSI 分片 */
static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data, int len)
{
    if (len < 3) return;
    uint8_t frag_id = data[0];
    uint16_t total_len;
    memcpy(&total_len, data + 1, 2);
    if (total_len != CSI_TOTAL_LEN) return;

    const uint8_t *payload = data + 3;
    int payload_len = len - 3;
    uint32_t now = xTaskGetTickCountFromISR();

    if (!frag_ctx_valid) {
        memcpy(frag_ctx.mac, info->src_addr, 6);
        frag_ctx.first_frag_tick = now;
        frag_ctx_valid = true;
    } else if (memcmp(frag_ctx.mac, info->src_addr, 6) != 0) {
        return;
    } else {
        uint32_t elapsed = (now - frag_ctx.first_frag_tick) * portTICK_PERIOD_MS;
        if (elapsed > FRAG_TIMEOUT_MS) {
            memset(&frag_ctx, 0, sizeof(frag_ctx));
            memcpy(frag_ctx.mac, info->src_addr, 6);
            frag_ctx.first_frag_tick = now;
            frag_ctx_valid = true;
            s_frag_discard_count++;
        }
    }

    if (frag_id == 0) {
        if (payload_len != FRAG_SIZE) return;
        memcpy(frag_ctx.buf, payload, FRAG_SIZE);
        frag_ctx.received_frags |= 1;
        frag_ctx.total_len = total_len;
    } else if (frag_id == 1) {
        if (payload_len != (CSI_TOTAL_LEN - FRAG_SIZE)) return;
        memcpy(frag_ctx.buf + FRAG_SIZE, payload, payload_len);
        frag_ctx.received_frags |= 2;
        frag_ctx.total_len = total_len;
    }

    if (frag_ctx.received_frags == 3) {
        if (s_output_queue != NULL) {
            csi_output_item_t item;
            memcpy(item.mac, frag_ctx.mac, 6);
            memcpy(item.buf, frag_ctx.buf, CSI_TOTAL_LEN);
            item.seq = s_csi_frame_count++;
            BaseType_t higher_prio_task_woken = pdFALSE;
            xQueueSendFromISR(s_output_queue, &item, &higher_prio_task_woken);
            if (higher_prio_task_woken == pdTRUE) {
                portYIELD_FROM_ISR();
            }
        }
        debug_led_toggle();   // 每组装完一帧翻转一次 LED
        static uint32_t s_assembled = 0;
        if (++s_assembled % 50 == 0) {
            ESP_LOGI(TAG, "Assembled %"PRIu32" frames, discarded=%"PRIu32, s_assembled, s_frag_discard_count);
        }
        memset(&frag_ctx, 0, sizeof(frag_ctx));
        frag_ctx_valid = false;
    }
}

static void uart_output_task(void *arg)
{
    csi_output_item_t item;
    static uint32_t seq = 0;
    static char line_buf[3072];

    while (1) {
        if (xQueueReceive(s_output_queue, &item, pdMS_TO_TICKS(1000)) == pdPASS) {
            int pos = 0;
            int n = snprintf(line_buf, sizeof(line_buf),
                             "CSI_DATA,%" PRIu32 "," MACSTR ",0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,%" PRIu16 ",0,\"[",
                             seq++,
                             MAC2STR(item.mac),
                             CSI_TOTAL_LEN);
            if (n <= 0 || n >= (int)sizeof(line_buf)) continue;
            pos = n;

            for (int i = 0; i < CSI_TOTAL_LEN; i++) {
                int written = snprintf(line_buf + pos, sizeof(line_buf) - pos,
                                       "%d,", (int8_t)item.buf[i]);
                if (written <= 0 || written >= (int)(sizeof(line_buf) - pos)) {
                    ESP_LOGE(TAG, "line buffer overflow");
                    pos = 0;
                    break;
                }
                pos += written;
            }
            if (pos == 0) continue;

            if (pos > 0 && line_buf[pos - 1] == ',') pos--;
            int tail = snprintf(line_buf + pos, sizeof(line_buf) - pos, "]\"\n");
            if (tail <= 0 || tail >= (int)(sizeof(line_buf) - pos)) continue;
            pos += tail;

            int w = uart_write_bytes(UART_NUM_0, (const char *)line_buf, pos);
            if (w < 0) ESP_LOGE(TAG, "uart write err");
            uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(100));
            static uint32_t s_output_cnt = 0;
            if (++s_output_cnt % 50 == 0) {
                ESP_LOGI(TAG, "UART output %"PRIu32" frames", s_output_cnt);
            }
        }
    }
}

static void wifi_init()
{
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // 在 start 前设置 MAC
    ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, CONFIG_CSI_SEND_MAC));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, CONFIG_WIFI_BANDWIDTH));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_BELOW));
}

static void wifi_esp_now_init()
{
    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_set_pmk((uint8_t *)"pmk1234567890123"));
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

    // 1. 添加广播 peer（用于发送测量包）
    esp_now_peer_info_t peer_broadcast = {
        .channel   = CONFIG_LESS_INTERFERENCE_CHANNEL,
        .ifidx     = WIFI_IF_STA,
        .encrypt   = false,
        .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    };
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_broadcast));

    // 为广播 peer 设置固定 HT40 速率，确保包包含 HT-LTF 以触发 CSI
    {
        esp_now_rate_config_t rate_config = {
            .phymode = CONFIG_ESP_NOW_PHYMODE,
            .rate    = CONFIG_ESP_NOW_RATE,
            .ersu    = false,
            .dcm     = false
        };
        esp_err_t ret = esp_now_set_peer_rate_config(peer_broadcast.peer_addr, &rate_config);
        ESP_LOGI(TAG, "Set broadcast rate config: %d", ret);
    }

    // 2. 【关键】添加接收端 MAC 为 peer，才能收到它发回的分片
    esp_now_peer_info_t peer_recv = {
        .channel   = CONFIG_LESS_INTERFERENCE_CHANNEL,
        .ifidx     = WIFI_IF_STA,
        .encrypt   = false,
    };
    memcpy(peer_recv.peer_addr, CONFIG_CSI_RECV_MAC, 6);
    ESP_ERROR_CHECK(esp_now_add_peer(&peer_recv));

    // 可选：为接收端 peer 设置固定速率（与接收端一致）
    esp_now_rate_config_t rate_config = {
        .phymode = CONFIG_ESP_NOW_PHYMODE,
        .rate    = CONFIG_ESP_NOW_RATE,
        .ersu    = false,
        .dcm     = false
    };
    ESP_ERROR_CHECK(esp_now_set_peer_rate_config(peer_recv.peer_addr, &rate_config));
}

void app_main()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 日志输出到 USB Serial JTAG，UART0 专用于 CSI 数据
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("csi_send", ESP_LOG_INFO);
    ESP_LOGI(TAG, "CSI send init start");

    // 安全重装 UART0
    if (uart_is_driver_installed(UART_NUM_0)) {
        uart_driver_delete(UART_NUM_0);
    }
    uart_driver_install(UART_NUM_0, 2048, 4096, 0, NULL, 0);
    uart_set_baudrate(UART_NUM_0, CONFIG_UART_BAUDRATE);
    uart_set_word_length(UART_NUM_0, UART_DATA_8_BITS);
    uart_set_parity(UART_NUM_0, UART_PARITY_DISABLE);
    uart_set_stop_bits(UART_NUM_0, UART_STOP_BITS_1);
    uart_set_hw_flow_ctrl(UART_NUM_0, UART_HW_FLOWCTRL_DISABLE, 0);

    debug_led_init();

    s_output_queue = xQueueCreate(20, sizeof(csi_output_item_t));
    xTaskCreate(uart_output_task, "uart_out", 8192, NULL, 1, NULL);

    wifi_init();
    wifi_esp_now_init();

    ESP_LOGI(TAG, "CSI send init done, starting broadcast...");
    // 启动后关闭日志，避免干扰 UART0 的 CSI 数据
    esp_log_level_set("*", ESP_LOG_NONE);
    esp_log_level_set("csi_send", ESP_LOG_NONE);

    const uint8_t broadcast_mac[] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    uint32_t send_count = 0;
    for (uint32_t count = 0; ; ++count) {
        esp_err_t ret = esp_now_send(broadcast_mac, (const uint8_t *)&count, sizeof(count));
        send_count++;
        if (send_count % 100 == 0) {
            ESP_LOGI(TAG, "Broadcast sent %"PRIu32" pkts, last_ret=%d", send_count, ret);
        }
        usleep(1000 * 1000 / CONFIG_SEND_FREQUENCY);
    }
}