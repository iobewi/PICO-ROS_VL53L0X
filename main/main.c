#include <stdlib.h>
#include "wifi_connect.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "picoros.h"
#include "picoserdes.h"

#include "vl53l0x.h"   // ESP-IDF_VL53L0X

static const char *TAG = "vl53l0x_picoros";

// -----------------------------------------------------------------------------
// WiFi Configuration (identique à l’exemple joystick)
// -----------------------------------------------------------------------------
#define WIFI_SSID          "yourWIFI SSID"
#define WIFI_PASS          "password"
#define WIFI_MAXIMUM_RETRY 5

// -----------------------------------------------------------------------------
// VL53L0X configuration
// -----------------------------------------------------------------------------
#define VL53L0X_SDA_GPIO        8
#define VL53L0X_SCL_GPIO        9
#define VL53L0X_I2C_FREQ_HZ     400000
#define VL53L0X_TIMING_BUDGET   33000  // µs

#define PUBLISH_PERIOD_MS      50      // 20 Hz

// -----------------------------------------------------------------------------
// Pico-ROS objects
// -----------------------------------------------------------------------------
static pico_publisher_t publisher;

// -----------------------------------------------------------------------------
// VL53L0X driver state
// -----------------------------------------------------------------------------
static vl53l0x_dev_t vl53_dev = {
    .addr_7b = 0x29
};

static uint16_t last_mm = 0;
static bool valid_sample = false;

// -----------------------------------------------------------------------------
// VL53L0X acquisition + publish task
// -----------------------------------------------------------------------------
static void publish_vl53_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Init I2C + VL53L0X");

    ESP_ERROR_CHECK(
        vl53l0x_i2c_master_init(
            VL53L0X_SDA_GPIO,
            VL53L0X_SCL_GPIO,
            VL53L0X_I2C_FREQ_HZ
        )
    );

    ESP_ERROR_CHECK(
        vl53l0x_init(&vl53_dev, VL53L0X_TIMING_BUDGET)
    );

    while (true) {

        uint16_t mm = 0;
        esp_err_t err = vl53l0x_read_mm(&vl53_dev, &mm);

        if (err == ESP_OK) {
            last_mm = mm;
            valid_sample = true;
        } else {
            ESP_LOGW(TAG, "VL53L0X read failed");
            valid_sample = false;
        }

        if (valid_sample) {
            // ---- Serialize UInt16 (std_msgs/UInt16)
            uint8_t buffer[8];
            size_t len = 0;

            picoserdes_uint16_serialize(buffer, &len, last_mm);

            pico_publish(&publisher, buffer, len);

            ESP_LOGI(TAG, "Published: %u mm", last_mm);
        }

        vTaskDelay(pdMS_TO_TICKS(PUBLISH_PERIOD_MS));
    }
}

// -----------------------------------------------------------------------------
// app_main (structure IDENTIQUE joystick)
// -----------------------------------------------------------------------------
void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_LOGI(TAG, "Connecting to WiFi...");
    wifi_connect_init(WIFI_SSID, WIFI_PASS, WIFI_MAXIMUM_RETRY);

    ESP_LOGI(TAG, "Init Pico-ROS");
    picoros_init();

    publisher = pico_publisher_create(
        "vl53l0x/mm",
        "std_msgs/msg/UInt16"
    );

    xTaskCreate(
        publish_vl53_task,
        "publish_vl53_task",
        4096,
        NULL,
        5,
        NULL
    );
}
