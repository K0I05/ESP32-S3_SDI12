/**
 * @file main.c
 * @brief explain
 *
 * This example takes the parameters 
 *
 * board: (1) ESP32-S3-­WROOM­-1-N16R8 | (2) ESP32-WROOM-32UE (ESP32 DevKit V4)
 * 
 * CTRL + SHIFT + P
 * pio run -t menufconfig
 * k & l keys for up or down
 * OR
 * PowerShell prompt: C:\Users\lavco\.platformio\penv\Scripts\platformio.exe run -t menuconfig
 * 
 */
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <unistd.h>
#include <limits.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>

#include <esp_system.h>
#include <esp_timer.h>
#include <esp_event.h>
#include <esp_log.h>

#include <nvs.h>
#include <nvs_flash.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <freertos/event_groups.h>

#include <sdi12_master.h>

#include <driver/uart.h>
#include <driver/gpio.h>

//
#define I2C_0_TASK_NAME          "i2c_0_tsk"
#define I2C_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)
#define I2C_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define APP_TAG                  "SDI-12 [APP]"

// https://randomnerdtutorials.com/esp32-pinout-reference-gpios/

// macros


static inline void vTaskDelaySecUntil(TickType_t *previousWakeTime, const uint sec) {
    const TickType_t xFrequency = ((sec * 1000) / portTICK_PERIOD_MS);
    vTaskDelayUntil( previousWakeTime, xFrequency );  
}

static void i2c_0_task( void *pvParameters ) {
    TickType_t xLastWakeTime = xTaskGetTickCount ();
    
    sdi12_master_config_t sdi12_master_cfg = SDI12_MASTER_CONFIG_DEFAULT;
    sdi12_master_handle_t sdi12_master_hdl = NULL;
    
    /* attempt to instantiate sdi-12 handle */
    sdi12_master_init(&sdi12_master_cfg, &sdi12_master_hdl);
    if(sdi12_master_hdl == NULL) {
        ESP_LOGE(APP_TAG, "sdi12_master_init failed");
        esp_restart();
    }

    ESP_LOGI(APP_TAG, "sdi-12 master fw version: %s", sdi12_master_get_fw_version());

    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## SDI-12 - START #########################");

        /* address query */
        char sensor_addrs;
        esp_err_t result = sdi12_master_address_query(sdi12_master_hdl, &sensor_addrs);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "sdi12_master_address_query failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "sdi-12 sensor found: %c", sensor_addrs);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
        
        /* acknowledge active */
        bool active = false;
        result = sdi12_master_acknowledge_active(sdi12_master_hdl, '0', &active);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "sdi12_master_acknowledge_active failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "sdi-12 sensor active: %s", active ? "true" : "false");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));

        /* send identification */
        sdi12_master_sensor_identification_t sdi12_identification;
        result = sdi12_master_send_identification(sdi12_master_hdl, '0', &sdi12_identification);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "sdi12_master_send_identification failed (%s)", esp_err_to_name(result));
        } else {
            ESP_LOGI(APP_TAG, "sdi-12 version:        %s", sdi12_identification.sdi12_version);
            ESP_LOGI(APP_TAG, "vendor identification: %s", sdi12_identification.vendor_identification);
            ESP_LOGI(APP_TAG, "sensor model:          %s", sdi12_identification.sensor_model);
            ESP_LOGI(APP_TAG, "sensor version:        %s", sdi12_identification.sensor_version);
            ESP_LOGI(APP_TAG, "sensor information:    %s", sdi12_identification.sensor_information);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));

        /* recorder m command */
        float* values;
        uint8_t size;
        result = sdi12_master_recorder(sdi12_master_hdl, '0', SDI12_MASTER_M_COMMAND, &values, &size);
        if(result != ESP_OK) {
            ESP_LOGE(APP_TAG, "sdi12_master_recorder failed (%s)", esp_err_to_name(result));
        } else {
            for(int i = 0; i < size; i++) {
                ESP_LOGI(APP_TAG, "sdi-12 sensor response value: %.2f", values[i]);
            }
        }

        ESP_LOGI(APP_TAG, "######################## SDI-12 - END ###########################");

        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 30 );
    }

    // free up task resources and remove task from stack
    sdi12_master_delete( sdi12_master_hdl );
    vTaskDelete( NULL );
}


void app_main( void ) {
    ESP_LOGI(APP_TAG, "Startup..");
    ESP_LOGI(APP_TAG, "Free memory: %lu bytes", esp_get_free_heap_size());
    ESP_LOGI(APP_TAG, "IDF version: %s", esp_get_idf_version());

    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set(APP_TAG, ESP_LOG_VERBOSE);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK( nvs_flash_erase() );
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );
    
    xTaskCreatePinnedToCore( 
        i2c_0_task, 
        I2C_0_TASK_NAME, 
        I2C_0_TASK_STACK_SIZE, 
        NULL, 
        I2C_0_TASK_PRIORITY, 
        NULL, 
        APP_CPU_NUM );
}