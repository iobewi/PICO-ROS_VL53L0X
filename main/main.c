#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

#include "picoros.h"
#include "picoserdes.h"

#include "vl53l0x.h"   // ESP-IDF_VL53L0X

static const char *TAG = "vl53l0x_picoros";

#define DEG2RAD(x) ((x) * (float)M_PI / 180.0f)
#define ROS_RANGE_INFRARED 1

static const float VL53_FOV_RAD   = DEG2RAD(25.0f);
static const float VL53_MIN_M     = 0.005f;
static const float VL53_MAX_M     = 2.0f;

static const float VL53_SIGMA_BASE_M  = 0.01f;   // Erreur absolue minimale (m)
static const float VL53_SIGMA_REL      = 0.02f;    /// Erreur relative (% de la distance)

// -----------------------------------------------------------------------------
// VL53L0X configuration
// -----------------------------------------------------------------------------
#define VL53L0X_SDA_GPIO        8
#define VL53L0X_SCL_GPIO        9
#define VL53L0X_I2C_FREQ_HZ     400000
#define VL53L0X_TIMING_BUDGET   33000  // µs

#define PUBLISH_PERIOD_MS      50      // 20 Hz

// Pico-ROS config
#define TOPIC_NAME                  "vl53l0x"
#define MODE                        "client"
#define ROUTER_ADDRESS              "serial/20.21#baudrate=115200"

// ROS node
picoros_node_t node = {
    .name = "vl53l0x_picoros",
};

// Publisher
picoros_publisher_t pub_vl53l0x = {
    .topic = {
        .name = TOPIC_NAME,
        .type = ROSTYPE_NAME(ros_Range),
        .rihs_hash = ROSTYPE_HASH(ros_Range),
    },
};

// Buffer for publication, used from this thread
#define PUB_BUF_SIZE 1024
uint8_t pub_buf[PUB_BUF_SIZE];

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

            z_clock_t clk = z_clock_now();
            float range_m = ((float)last_mm) / 1000.0f;

            float sigma_m = VL53_SIGMA_BASE_M +
                            VL53_SIGMA_REL * range_m;

            float variance_m2 = sigma_m * sigma_m;

            ros_Range vl53 = {
                .header.stamp.nanosec = clk.tv_nsec, 
                .header.stamp.sec = clk.tv_sec,
                .header.frame_id = "esp32-vl53",
                .radiation_type = ROS_RANGE_INFRARED,
                .field_of_view = VL53_FOV_RAD,
                .min_range = VL53_MIN_M,
                .max_range = VL53_MAX_M,
                .range =  range_m,
                .variance = variance_m2,
            };

            size_t len = ps_serialize(pub_buf, &vl53, PUB_BUF_SIZE);
            if (len > 0){
                picoros_publish(&pub_vl53l0x, pub_buf, len);
            }
            else{
                ESP_LOGE(node.name, "ros_vl53 message serialization error.");
            }

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
    ESP_LOGI(TAG, "Init Pico-ROS");

    // Init Pico-ROS
    picoros_interface_t ifx = {
        .mode = MODE,
        .locator = ROUTER_ADDRESS,
    };

    ESP_LOGI(node.name, "Starting pico-ros interface %s %s\n", ifx.mode, ifx.locator );
    while (picoros_interface_init(&ifx) == PICOROS_NOT_READY){
        ESP_LOGI(node.name, "Waiting RMW init...\n");
        z_sleep_s(1);
    }
    
    ESP_LOGI(node.name, "Starting Pico-ROS node %s domain:%lu\n", node.name, node.domain_id);
    picoros_node_init(&node);
    
    ESP_LOGI(node.name, "Declaring publisher on %s\n", pub_vl53l0x.topic.name);
    picoros_publisher_declare(&node, &pub_vl53l0x);

    xTaskCreate(
        publish_vl53_task,
        "publish_vl53_task",
        4096,
        NULL,
        5,
        NULL
    );
}
