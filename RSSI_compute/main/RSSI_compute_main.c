#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "nvs_flash.h"

/* BLE NimBLE includes */
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"

static const char *TAG = "RSSI_COMPUTE";

/* ========================== 宏定义配置 ========================== */
#define L               1.0f    // 等腰直角三角形直角边长度 (单位: 米)
#define SCAN_DURATION_MS 10000   // 扫描持续时间
#define UPDATE_PERIOD_MS 2000    // 定位计算与打印周期
#define MAX_DISTANCE    10.0f   // 合理的最大距离范围 (单位: 米)

// 接收器名称定义，用于区分 d1, d2, d3
#define DEVICE_NAME_A   "C3_RECEIVER_A"
#define DEVICE_NAME_B   "C3_RECEIVER_B"
#define DEVICE_NAME_C   "C3_RECEIVER_C"

/* ========================== 全局变量 ========================== */
typedef struct {
    float distance;
    bool updated;
} receiver_data_t;

static receiver_data_t g_receivers[3] = {0}; // 0:A, 1:B, 2:C
static uint16_t g_conn_handles[3] = {BLE_HS_CONN_HANDLE_NONE, BLE_HS_CONN_HANDLE_NONE, BLE_HS_CONN_HANDLE_NONE};

/* ========================== 矩阵运算与定位算法 ========================== */

/**
 * @brief 2x2 矩阵求逆
 * @param mat 输入矩阵 [a, b; c, d]
 * @param inv 输出逆矩阵
 * @return 成功返回 true，行列式为 0 返回 false
 */
bool mat_inv_2x2(float mat[2][2], float inv[2][2]) {
    float det = mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
    if (fabs(det) < 1e-6) return false;
    float inv_det = 1.0f / det;
    inv[0][0] =  mat[1][1] * inv_det;
    inv[0][1] = -mat[0][1] * inv_det;
    inv[1][0] = -mat[1][0] * inv_det;
    inv[1][1] =  mat[0][0] * inv_det;
    return true;
}

/**
 * @brief 最小二乘法多边定位
 * 根据三个锚点坐标 (0,0), (L,0), (0,L) 和对应的距离计算坐标 (x, y)
 * 
 * 构建线性方程组 AX = B:
 * 1. (x-0)^2 + (y-0)^2 = d1^2
 * 2. (x-L)^2 + (y-0)^2 = d2^2
 * 3. (x-0)^2 + (y-L)^2 = d3^2
 * 
 * 方程1减方程3: 2yL = d1^2 - d3^2 + L^2  =>  0*x + 2L*y = d1^2 - d3^2 + L^2
 * 方程2减方程3: -2xL + 2yL = d2^2 - d3^2  => -2L*x + 2L*y = d2^2 - d3^2
 * 
 * 矩阵 A = [[0, 2L], [-2L, 2L]]
 * 向量 B = [[d1^2 - d3^2 + L^2], [d2^2 - d3^2]]
 * 
 * @param d1 接收器 A 的距离
 * @param d2 接收器 B 的距离
 * @param d3 接收器 C 的距离
 * @param x 输出 x 坐标
 * @param y 输出 y 坐标
 */
void trilateration(float d1, float d2, float d3, float *x, float *y) {
    // 1. 构建线性方程组 AX = B
    // 锚点坐标: A(0,0), B(L,0), C(0,L)
    // 根据 (x-xi)^2 + (y-yi)^2 = di^2 展开并作差消除二次项:
    // 方程(1)-(3): 0*x + 2L*y = d1^2 - d3^2 + L^2
    // 方程(2)-(3): -2L*x + 2L*y = d2^2 - d3^2
    float A[2][2] = {
        {0, 2 * L},
        {-2 * L, 2 * L}
    };
    
    float B[2] = {
        d1 * d1 - d3 * d3 + L * L,
        d2 * d2 - d3 * d3
    };

    // 2. 计算 A 的转置 AT
    float AT[2][2] = {
        {A[0][0], A[1][0]},
        {A[0][1], A[1][1]}
    };

    // 3. 计算 ATA = A^T * A (正规方程组左侧)
    float ATA[2][2];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            ATA[i][j] = 0;
            for (int k = 0; k < 2; k++) {
                ATA[i][j] += AT[i][k] * A[k][j];
            }
        }
    }

    // 4. 对 ATA 求逆
    float ATA_inv[2][2];
    if (!mat_inv_2x2(ATA, ATA_inv)) {
        ESP_LOGE(TAG, "矩阵 ATA 不可逆，无法求解！");
        return;
    }

    // 5. 计算 (A^T A)^-1 * A^T
    float ATA_inv_AT[2][2];
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            ATA_inv_AT[i][j] = 0;
            for (int k = 0; k < 2; k++) {
                ATA_inv_AT[i][j] += ATA_inv[i][k] * AT[k][j];
            }
        }
    }

    // 6. 最终求解 X = (A^T A)^-1 * A^T * B
    *x = ATA_inv_AT[0][0] * B[0] + ATA_inv_AT[0][1] * B[1];
    *y = ATA_inv_AT[1][0] * B[0] + ATA_inv_AT[1][1] * B[1];
}

/* ========================== BLE 逻辑 ========================== */

// 处理收到的数据
static void handle_received_distance(int index, float dist) {
    if (dist < 0 || dist > MAX_DISTANCE) {
        ESP_LOGW(TAG, "Invalid distance received from %d: %.2f", index, dist);
        return;
    }
    g_receivers[index].distance = dist;
    g_receivers[index].updated = true;
}

static int ble_central_gap_event(struct ble_gap_event *event, void *arg);

/**
 * 扫描并尝试连接目标设备
 */
static void ble_central_scan(void) {
    struct ble_gap_disc_params disc_params;
    int rc;

    disc_params.filter_duplicates = 1;
    disc_params.passive = 0;
    disc_params.itvl = 0;
    disc_params.window = 0;
    disc_params.filter_policy = 0;
    disc_params.limited = 0;

    rc = ble_gap_disc(0, SCAN_DURATION_MS, &disc_params, ble_central_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGE(TAG, "Error initiating GAP discovery; rc=%d", rc);
    }
}

static int ble_central_gap_event(struct ble_gap_event *event, void *arg) {
    struct ble_hs_adv_fields fields;
    int rc;

    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        rc = ble_hs_adv_parse_fields(&fields, event->disc.data, event->disc.length_data);
        if (rc != 0) return 0;

        if (fields.name != NULL) {
            int index = -1;
            if (strncmp((char *)fields.name, DEVICE_NAME_A, fields.name_len) == 0) index = 0;
            else if (strncmp((char *)fields.name, DEVICE_NAME_B, fields.name_len) == 0) index = 1;
            else if (strncmp((char *)fields.name, DEVICE_NAME_C, fields.name_len) == 0) index = 2;

            if (index != -1 && g_conn_handles[index] == BLE_HS_CONN_HANDLE_NONE) {
                ESP_LOGI(TAG, "Found target device: %.*s, connecting...", fields.name_len, fields.name);
                ble_gap_disc_cancel();
                
                // 将 index 作为参数传递给连接回调
                int *p_index = malloc(sizeof(int));
                *p_index = index;
                rc = ble_gap_connect(0, &event->disc.addr, 30000, NULL, ble_central_gap_event, p_index);
                if (rc != 0) {
                    ESP_LOGE(TAG, "Error connecting; rc=%d", rc);
                    free(p_index);
                }
            }
        }
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        if (arg != NULL) {
            int index = *(int *)arg;
            if (event->connect.status == 0) {
                ESP_LOGI(TAG, "Connected to receiver %d successfully", index);
                g_conn_handles[index] = event->connect.conn_handle;
            } else {
                ESP_LOGE(TAG, "Connection to receiver %d failed; status=%d", index, event->connect.status);
                free(arg);
                ble_central_scan();
            }
        }
        return 0;

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected; reason=%d", event->disconnect.reason);
        for (int i = 0; i < 3; i++) {
            if (g_conn_handles[i] == event->disconnect.conn.conn_handle) {
                g_conn_handles[i] = BLE_HS_CONN_HANDLE_NONE;
                ESP_LOGI(TAG, "Receiver %d disconnected", i);
            }
        }
        if (arg != NULL) free(arg);
        ble_central_scan();
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX:
        // 假设从机通过 Notify 发送 4 字节的 float 距离值
        if (OS_MBUF_PKTLEN(event->notify_rx.om) == sizeof(float)) {
            float dist;
            ble_hs_mbuf_to_flat(event->notify_rx.om, &dist, sizeof(float), NULL);
            
            // 根据 conn_handle 确定是哪个接收器
            for (int i = 0; i < 3; i++) {
                if (g_conn_handles[i] == event->notify_rx.conn_handle) {
                    handle_received_distance(i, dist);
                    break;
                }
            }
        }
        return 0;

    default:
        return 0;
    }
}

static void ble_central_on_stack_reset(int reason) {
    ESP_LOGE(TAG, "NimBLE stack reset; reason=%d", reason);
}

static void ble_central_on_stack_sync(void) {
    ESP_LOGI(TAG, "NimBLE stack synced");
    ble_central_scan();
}

void ble_central_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/* ========================== 主任务逻辑 ========================== */

void compute_task(void *pvParameters) {
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(UPDATE_PERIOD_MS));

        // 检查是否收到了所有三个接收器的数据
        if (g_receivers[0].updated && g_receivers[1].updated && g_receivers[2].updated) {
            float x, y;
            float d1 = g_receivers[0].distance;
            float d2 = g_receivers[1].distance;
            float d3 = g_receivers[2].distance;

            trilateration(d1, d2, d3, &x, &y);

            printf("\n----------------------------------------\n");
            printf("距离数据：d1=%.3f m, d2=%.3f m, d3=%.3f m\n", d1, d2, d3);
            printf("计算结果：x=%.3f, y=%.3f\n", x, y);
            printf("----------------------------------------\n");

            // 重置更新标志，等待下一轮数据
            g_receivers[0].updated = false;
            g_receivers[1].updated = false;
            g_receivers[2].updated = false;
        } else {
            ESP_LOGW(TAG, "Waiting for data from all receivers... (A:%d, B:%d, C:%d)", 
                     g_receivers[0].updated, g_receivers[1].updated, g_receivers[2].updated);
        }
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

    // 2. 初始化 NimBLE
    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.reset_cb = ble_central_on_stack_reset;
    ble_hs_cfg.sync_cb = ble_central_on_stack_sync;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    // 初始化 GAP 服务
    ble_svc_gap_init();

    // 启动 NimBLE 主机任务
    nimble_port_freertos_init(ble_central_host_task);

    // 3. 启动定位计算任务
    xTaskCreate(compute_task, "compute_task", 4096, NULL, 5, NULL);
}
