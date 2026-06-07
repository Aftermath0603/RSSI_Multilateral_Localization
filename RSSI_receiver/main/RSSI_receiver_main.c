#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_timer.h"

/* BLE Includes (NimBLE) */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

/* --- 宏定义 --- */
// WiFi 相关
#define TARGET_SSID         "TX_C3_AP"
#define SCAN_CHANNEL        6
#define SCAN_INTERVAL_MS    1000    // 扫描间隔
#define RSSI_WINDOW_SIZE    10      // 滑动平均窗口大小

// 距离转换参数 (PL(d) = PL(d0) - 10*n*lg(d/d0))
#define DIST_D0             1.0f    // 参考距离 (m)
#define RSSI_D0             -45.0f  // 1米处的 RSSI (dBm)
#define PATH_LOSS_N         3.0f    // 路径衰减因子 n

// 接收器编号及坐标配置 (d1/d2/d3 对应 A/B/C)
// 接收器 A: (0, 0), 发送 d1
// 接收器 B: (L, 0), 发送 d2
// 接收器 C: (0, L), 发送 d3
#define RECEIVER_ID         'A'     // 'A', 'B', 或 'C'
#if RECEIVER_ID == 'A'
    #define DEVICE_NAME     "C3_RECEIVER_A"
#elif RECEIVER_ID == 'B'
    #define DEVICE_NAME     "C3_RECEIVER_B"
#else
    #define DEVICE_NAME     "C3_RECEIVER_C"
#endif

static const char *TAG = DEVICE_NAME;

/* --- 全局变量 --- */
static float g_rssi_history[RSSI_WINDOW_SIZE];
static int g_rssi_index = 0;
static int g_rssi_count = 0;
static float g_current_dist = 0.0f;
static uint16_t g_dist_char_handle;
static uint16_t g_conn_handle = 0;
static bool g_ble_connected = false;

/* --- 函数声明 --- */
float rssi_to_dist(int rssi);
float update_moving_average(int rssi);
static void wifi_scan_task(void *pvParameters);
static void ble_app_advertise(void);

/* --- 距离转换公式实现 --- */
float rssi_to_dist(int rssi) {
    // d = d0 * 10^((rssi_d0 - rssi) / (10 * n))
    float exponent = (RSSI_D0 - (float)rssi) / (10.0f * PATH_LOSS_N);
    return DIST_D0 * powf(10.0f, exponent);
}

/* --- 滑动平均滤波 --- */
float update_moving_average(int rssi) {
    g_rssi_history[g_rssi_index] = (float)rssi;
    g_rssi_index = (g_rssi_index + 1) % RSSI_WINDOW_SIZE;
    if (g_rssi_count < RSSI_WINDOW_SIZE) g_rssi_count++;

    float sum = 0;
    for (int i = 0; i < g_rssi_count; i++) {
        sum += g_rssi_history[i];
    }
    return sum / g_rssi_count;
}

/* --- BLE GATT 服务回调与配置 --- */
static int ble_svc_gatt_handler(uint16_t conn_handle, uint16_t attr_handle,
                                struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (attr_handle == g_dist_char_handle) {
        return os_mbuf_append(ctxt->om, &g_current_dist, sizeof(g_current_dist));
    }
    return 0;
}

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0x181A), // Environmental Sensing Service
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0x2A58), // Analog Value
                .access_cb = ble_svc_gatt_handler,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &g_dist_char_handle,
            },
            {0}
        },
    },
    {0}
};

static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            ESP_LOGI(TAG, "BLE Connected; conn_handle=%d", event->connect.conn_handle);
            g_conn_handle = event->connect.conn_handle;
            g_ble_connected = true;
            break;
        case BLE_GAP_EVENT_DISCONNECT:
            ESP_LOGI(TAG, "BLE Disconnected; restarting advertising");
            g_ble_connected = false;
            ble_app_advertise();
            break;
        case BLE_GAP_EVENT_ADV_COMPLETE:
            ESP_LOGI(TAG, "BLE Adv Complete");
            ble_app_advertise();
            break;
    }
    return 0;
}

static void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    memset(&fields, 0, sizeof(fields));
    fields.name = (uint8_t *)DEVICE_NAME;
    fields.name_len = strlen(DEVICE_NAME);
    fields.name_is_complete = 1;
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    ble_gap_adv_set_fields(&fields);
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

void ble_host_task(void *param) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* --- WiFi 扫描逻辑 --- */
static void wifi_scan_task(void *pvParameters) {
    wifi_scan_config_t scan_config = {
        .ssid = NULL,
        .bssid = NULL,
        .channel = SCAN_CHANNEL,
        .show_hidden = false,
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .scan_time.active.min = 100,
        .scan_time.active.max = 250
    };

    while (1) {
        esp_wifi_scan_start(&scan_config, true);
        
        uint16_t ap_count = 0;
        esp_wifi_scan_get_ap_num(&ap_count);
        if (ap_count > 0) {
            wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * ap_count);
            ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&ap_count, ap_list));
            
            bool found = false;
            for (int i = 0; i < ap_count; i++) {
                if (strcmp((char *)ap_list[i].ssid, TARGET_SSID) == 0) {
                    float avg_rssi = update_moving_average(ap_list[i].rssi);
                    g_current_dist = rssi_to_dist((int)avg_rssi);
                    
                    ESP_LOGI(TAG, "Target: %s, RSSI: %d, Avg RSSI: %.1f, Dist: %.2f m", 
                             TARGET_SSID, ap_list[i].rssi, avg_rssi, g_current_dist);
                    
                    // 通过 BLE 发送
                    if (g_ble_connected) {
                        struct os_mbuf *om = ble_hs_mbuf_from_flat(&g_current_dist, sizeof(g_current_dist));
                        int rc = ble_gattc_notify_custom(g_conn_handle, g_dist_char_handle, om);
                        ESP_LOGI(TAG, "BLE Send Status: %s", rc == 0 ? "Success" : "Fail");
                    } else {
                        ESP_LOGI(TAG, "BLE Not Connected");
                    }
                    found = true;
                    break;
                }
            }
            if (!found) {
                ESP_LOGW(TAG, "Target SSID '%s' not found on channel %d", TARGET_SSID, SCAN_CHANNEL);
            }
            free(ap_list);
        }
        
        vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL_MS));
    }
}

void app_main(void) {
    // 1. 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. 初始化 WiFi (STA 模式)
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());

    // 3. 初始化 BLE (NimBLE)
    ESP_ERROR_CHECK(nimble_port_init());
    ble_svc_gap_device_name_set(DEVICE_NAME);
    ble_svc_gap_init();
    ble_svc_gatt_init();
    ESP_ERROR_CHECK(ble_gatts_count_cfg(gatt_svr_svcs));
    ESP_ERROR_CHECK(ble_gatts_add_svcs(gatt_svr_svcs));
    
    nimble_port_freertos_init(ble_host_task);
    ble_app_advertise();

    // 4. 启动扫描任务
    xTaskCreate(wifi_scan_task, "wifi_scan_task", 4096, NULL, 5, NULL);
    
    ESP_LOGI(TAG, "Receiver %c Initialized. Monitoring SSID: %s on Ch: %d", RECEIVER_ID, TARGET_SSID, SCAN_CHANNEL);
}
