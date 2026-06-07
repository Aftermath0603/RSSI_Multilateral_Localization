#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

/* --- 参数配置宏 --- */
#define AP_SSID          "TX_C3_AP"    // 广播的 SSID
#define AP_PASSWORD      ""            // 密码为空表示开放网络
#define AP_CHANNEL       6             // WiFi 信道
#define BEACON_INTERVAL  100           // 信标帧间隔 (ms), 范围 100~60000

static const char *TAG = "RSSI_EMITTER";

/**
 * @brief 初始化 SoftAP 模式
 * 
 * 配置 ESP32-C3 作为接入点持续广播信标帧。
 */
void wifi_init_softap(void)
{
    // 1. 初始化底层 TCP/IP 堆栈
    ESP_ERROR_CHECK(esp_netif_init());
    
    // 2. 创建默认事件循环
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    
    // 3. 创建默认 SoftAP 网络接口
    esp_netif_create_default_wifi_ap();

    // 4. 初始化 WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 5. 配置 SoftAP 参数
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .channel = AP_CHANNEL,
            .password = AP_PASSWORD,
            .max_connection = 4,         // 最大允许连接数（虽然不需要连接）
            .authmode = WIFI_AUTH_OPEN,  // 开放加密方式
            .beacon_interval = BEACON_INTERVAL, // 设置信标间隔
        },
    };

    // 6. 设置 WiFi 模式为 AP
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    
    // 7. 应用配置
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    
    // 8. 启动 WiFi
    ESP_ERROR_CHECK(esp_wifi_start());

    // 输出启动信息
    printf("\n==================================\n");
    printf("SSID: %s\n", AP_SSID);
    printf("Channel: %d\n", AP_CHANNEL);
    printf("Beacon Interval: %d ms\n", BEACON_INTERVAL);
    printf("==================================\n\n");
}

void app_main(void)
{
    // 初始化 NVS（WiFi 存储配置需要）
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 启动 SoftAP
    // 注意：未调用任何蓝牙相关初始化函数，蓝牙将保持关闭状态
    wifi_init_softap();

    // 循环打印发送状态
    while (1) {
        printf("beacon sending...\n");
        vTaskDelay(pdMS_TO_TICKS(2000)); // 每 2 秒打印一次状态，避免串口过于频繁
    }
}
