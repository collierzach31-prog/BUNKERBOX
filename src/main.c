#include <stdio.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_err.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

static const char *TAG = "BunkerBOX";

// -----------------------------------------------------------------------------
// I2C / MPU6050 configuration
// -----------------------------------------------------------------------------
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SCL_IO           22
#define I2C_MASTER_SDA_IO           21
#define I2C_MASTER_FREQ_HZ          100000
#define I2C_MASTER_TX_BUF_DISABLE   0
#define I2C_MASTER_RX_BUF_DISABLE   0

#define MPU6050_SENSOR_ADDR         0x68
#define MPU6050_PWR_MGMT_1_REG_ADDR 0x6B
#define MPU6050_ACCEL_CONFIG_ADDR   0x1C
#define MPU6050_ACCEL_XOUT_H        0x3B

// -----------------------------------------------------------------------------
// BLE configuration
// -----------------------------------------------------------------------------
#define PROFILE_APP_ID              0
#define GATTS_SERVICE_UUID          0xFFE0
#define GATTS_CHAR_UUID             0xFFE1
#define SAMPLE_NOTIFICATION_BYTES   6

static uint16_t gatts_service_handle;
static esp_gatt_if_t gatts_if_global = ESP_GATT_IF_NONE;
static uint16_t conn_id = 0;
static uint16_t notify_handle = 0;
static bool is_connected = false;

static uint8_t adv_config_done = 0;
#define ADV_CONFIG_FLAG      (1 << 0)

static uint8_t raw_adv_data[] = {
    0x02, 0x01, 0x06,                 // Flags
    0x03, 0x03, 0xE0, 0xFF,           // 0xFFE0 service UUID
    0x0A, 0x09, 'B','U','N','K','E','R','B','O','X'
};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// -----------------------------------------------------------------------------
static esp_err_t i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    return i2c_driver_install(I2C_MASTER_NUM,
                              conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE,
                              0);
}

static esp_err_t mpu6050_write(uint8_t reg, uint8_t value) {
    uint8_t buf[2] = {reg, value};
    return i2c_master_write_to_device(I2C_MASTER_NUM,
                                      MPU6050_SENSOR_ADDR,
                                      buf,
                                      sizeof(buf),
                                      pdMS_TO_TICKS(1000));
}

static esp_err_t mpu6050_read(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM,
                                        MPU6050_SENSOR_ADDR,
                                        &reg,
                                        1,
                                        data,
                                        len,
                                        pdMS_TO_TICKS(1000));
}

// -----------------------------------------------------------------------------
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= ~ADV_CONFIG_FLAG;
        if (adv_config_done == 0) {
            ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
        }
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Failed to start advertising");
        } else {
            ESP_LOGI(TAG, "Advertising as BUNKERBOX (HM-10 compatible)");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(TAG, "Failed to stop advertising");
        }
        break;
    default:
        break;
    }
}

static void handle_connect(const esp_ble_gatts_cb_param_t *param, esp_gatt_if_t gatts_if) {
    conn_id = param->connect.conn_id;
    gatts_if_global = gatts_if;
    is_connected = true;
    ESP_LOGI(TAG,
             "Central connected: %02X:%02X:%02X:%02X:%02X:%02X",
             param->connect.remote_bda[0],
             param->connect.remote_bda[1],
             param->connect.remote_bda[2],
             param->connect.remote_bda[3],
             param->connect.remote_bda[4],
             param->connect.remote_bda[5]);
}

static void handle_disconnect(void) {
    is_connected = false;
    ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
    ESP_LOGI(TAG, "Central disconnected; advertising restarted");
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event,
                                        esp_gatt_if_t gatts_if,
                                        esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        esp_ble_gap_set_device_name("BUNKERBOX");
        ESP_ERROR_CHECK(esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data)));
        adv_config_done |= ADV_CONFIG_FLAG;

        esp_gatt_srvc_id_t service_id = {
            .is_primary = true,
            .id = {
                .inst_id = 0,
                .uuid = {
                    .len = ESP_UUID_LEN_16,
                    .uuid = {.uuid16 = GATTS_SERVICE_UUID}
                }
            }
        };
        ESP_ERROR_CHECK(esp_ble_gatts_create_service(gatts_if, &service_id, 6));
        break;
    }
    case ESP_GATTS_CREATE_EVT:
        gatts_service_handle = param->create.service_handle;
        ESP_ERROR_CHECK(esp_ble_gatts_start_service(gatts_service_handle));

        esp_bt_uuid_t char_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = GATTS_CHAR_UUID}
        };
        ESP_ERROR_CHECK(esp_ble_gatts_add_char(gatts_service_handle,
                                               &char_uuid,
                                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                               ESP_GATT_CHAR_PROP_BIT_READ |
                                               ESP_GATT_CHAR_PROP_BIT_WRITE |
                                               ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                               NULL,
                                               NULL));
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
        notify_handle = param->add_char.attr_handle;

        esp_bt_uuid_t descr_uuid = {
            .len = ESP_UUID_LEN_16,
            .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}
        };
        ESP_ERROR_CHECK(esp_ble_gatts_add_char_descr(gatts_service_handle,
                                                     &descr_uuid,
                                                     ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                     NULL,
                                                     NULL));
        break;
    case ESP_GATTS_CONNECT_EVT:
        handle_connect(param, gatts_if);
        break;
    case ESP_GATTS_DISCONNECT_EVT:
        handle_disconnect();
        break;
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event,
                                esp_gatt_if_t gatts_if,
                                esp_ble_gatts_cb_param_t *param) {
    if (event == ESP_GATTS_REG_EVT && param->reg.status != ESP_GATT_OK) {
        ESP_LOGE(TAG, "GATTS registration failed: 0x%X", param->reg.status);
        return;
    }

    if (event == ESP_GATTS_REG_EVT) {
        gatts_profile_event_handler(event, gatts_if, param);
        return;
    }

    if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gatts_if_global) {
        gatts_profile_event_handler(event, gatts_if, param);
    }
}

// -----------------------------------------------------------------------------
static void sensor_loop(void) {
    uint8_t raw_buf[SAMPLE_NOTIFICATION_BYTES];
    uint8_t notify_buf[SAMPLE_NOTIFICATION_BYTES];

    while (true) {
        if (mpu6050_read(MPU6050_ACCEL_XOUT_H, raw_buf, sizeof(raw_buf)) == ESP_OK) {
            notify_buf[0] = raw_buf[1];
            notify_buf[1] = raw_buf[0];
            notify_buf[2] = raw_buf[3];
            notify_buf[3] = raw_buf[2];
            notify_buf[4] = raw_buf[5];
            notify_buf[5] = raw_buf[4];

            if (is_connected && notify_handle != 0) {
                esp_err_t err = esp_ble_gatts_send_indicate(gatts_if_global,
                                                             conn_id,
                                                             notify_handle,
                                                             sizeof(notify_buf),
                                                             notify_buf,
                                                             false);
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "Notify failed: %s", esp_err_to_name(err));
                }
            }
        } else {
            ESP_LOGW(TAG, "MPU6050 read failed");
        }

        static uint8_t heartbeat = 0;
        if (!is_connected && heartbeat++ % 50 == 0) {
            ESP_LOGI(TAG, "Advertising... waiting for subscriber");
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

void app_main(void) {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_ERROR_CHECK(mpu6050_write(MPU6050_PWR_MGMT_1_REG_ADDR, 0));
    ESP_ERROR_CHECK(mpu6050_write(MPU6050_ACCEL_CONFIG_ADDR, 0x18));
    ESP_LOGI(TAG, "MPU6050 ready (±16g)");

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));
    ESP_ERROR_CHECK(esp_bluedroid_init());
    ESP_ERROR_CHECK(esp_bluedroid_enable());

    ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
    ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
    ESP_ERROR_CHECK(esp_ble_gatts_app_register(PROFILE_APP_ID));

    sensor_loop();
}