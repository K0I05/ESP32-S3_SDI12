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
#define SDI12_0_TASK_NAME          "sdi12_0_tsk"
#define SDI12_0_TASK_STACK_SIZE    (configMINIMAL_STACK_SIZE * 4)
#define SDI12_0_TASK_PRIORITY      (tskIDLE_PRIORITY + 2)

#define APP_TAG                  "SDI-12 [APP]"

// https://randomnerdtutorials.com/esp32-pinout-reference-gpios/

// macros

typedef struct hy_wdc6se_s {
    float wind_speed;
    float wind_direction;
    float air_temperature;
    float relative_humidity;
    float barometric_pressure;
    int precipitation_status;
    float precipitation_intensity;
    float illuminance;
    float solar_radiation;
} hy_wdc6se_t;

static inline void sdi12_x_hy_wdc6se_print(hy_wdc6se_t hy_wdc6se) {
    //ESP_LOGI(APP_TAG, "hy-wdc6se wind speed:           %.2f m/s", hy_wdc6se.wind_speed);
    //ESP_LOGI(APP_TAG, "hy-wdc6se wind direction:       %.2f°", hy_wdc6se.wind_direction);
    ESP_LOGI(APP_TAG, "hy-wdc6se air temperature:      %.2f°C", hy_wdc6se.air_temperature);
    ESP_LOGI(APP_TAG, "hy-wdc6se relative humidity:    %.2f %%", hy_wdc6se.relative_humidity);
    //ESP_LOGI(APP_TAG, "hy-wdc6se barometric pressure:  %.2f hPa", hy_wdc6se.barometric_pressure);
    //ESP_LOGI(APP_TAG, "hy-wdc6se precipitation status: %i", hy_wdc6se.precipitation_status);
    //ESP_LOGI(APP_TAG, "hy-wdc6se precipitation rate:   %.2f mm/hr", hy_wdc6se.precipitation_intensity);
    //ESP_LOGI(APP_TAG, "hy-wdc6se illuminance:          %.2f lux", hy_wdc6se.illuminance);
    //ESP_LOGI(APP_TAG, "hy-wdc6se solar radiation:      %.2f W/m2", hy_wdc6se.solar_radiation);
}

static inline void vTaskDelaySecUntil(TickType_t *previousWakeTime, const uint sec) {
    const TickType_t xFrequency = ((sec * 1000) / portTICK_PERIOD_MS);
    vTaskDelayUntil( previousWakeTime, xFrequency );  
}

static void sdi12_0_cr6( sdi12_master_handle_t sdi12_master_hdl ) {
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
    //result = sdi12_master_recorder(sdi12_master_hdl, '0', SDI12_MASTER_M_COMMAND, &values, &size);
    result = sdi12_master_recorder(sdi12_master_hdl, '0', SDI12_MASTER_C_COMMAND, &values, &size);
    if(result != ESP_OK) {
        ESP_LOGE(APP_TAG, "sdi12_master_recorder failed (%s)", esp_err_to_name(result));
    } else {
        for(int i = 0; i < size; i++) {
            ESP_LOGI(APP_TAG, "sdi-12 sensor response value: %.2f", values[i]);
        }
    }
}

static void sdi12_x_hy_wdc6se_measure( sdi12_master_handle_t sdi12_master_hdl, hy_wdc6se_t* hy_wdc6se ) {
    esp_err_t result;
    float* values;
    uint8_t size;
    
    /* recorder m command 
    result = sdi12_master_recorder(sdi12_master_hdl, 'x', SDI12_MASTER_M_COMMAND, &values, &size);
    if(result != ESP_OK || size != 5) {
        ESP_LOGE(APP_TAG, "sdi12_master_recorder M failed (%s)", esp_err_to_name(result));
        hy_wdc6se->wind_speed     = NAN;
        hy_wdc6se->wind_direction = NAN;
    } else {
        hy_wdc6se->wind_speed     = values[0];
        hy_wdc6se->wind_direction = values[1];
    }
    */
    /* recorder m1 command */
    result = sdi12_master_recorder(sdi12_master_hdl, 'x', SDI12_MASTER_M1_COMMAND, &values, &size);
    if(result != ESP_OK || size != 3) {
        ESP_LOGE(APP_TAG, "sdi12_master_recorder M1 failed (%s)", esp_err_to_name(result));
        hy_wdc6se->air_temperature   = NAN;
        hy_wdc6se->relative_humidity = NAN;
    } else {
        hy_wdc6se->air_temperature   = values[0];
        hy_wdc6se->relative_humidity = values[1];
    }

    /* recorder m2 command
    result = sdi12_master_recorder(sdi12_master_hdl, 'x', SDI12_MASTER_M2_COMMAND, &values, &size);
    if(result != ESP_OK || size != 3) {
        ESP_LOGE(APP_TAG, "sdi12_master_recorder M2 failed (%s)", esp_err_to_name(result));
        hy_wdc6se->barometric_pressure = NAN;
    } else {
        hy_wdc6se->barometric_pressure = values[0];
    }
    */
    /* recorder m4 command
    result = sdi12_master_recorder(sdi12_master_hdl, 'x', SDI12_MASTER_M4_COMMAND, &values, &size);
    if(result != ESP_OK || size != 4) {
        ESP_LOGE(APP_TAG, "sdi12_master_recorder M4 failed (%s)", esp_err_to_name(result));
        hy_wdc6se->precipitation_status    = 0;
        hy_wdc6se->precipitation_intensity = NAN;
    } else {
        hy_wdc6se->precipitation_status    = values[0];
        hy_wdc6se->precipitation_intensity = values[1];
    }
    */

    /* recorder m8 command 
    result = sdi12_master_recorder(sdi12_master_hdl, 'x', SDI12_MASTER_M8_COMMAND, &values, &size);
    if(result != ESP_OK || size != 6) {
        ESP_LOGE(APP_TAG, "sdi12_master_recorder M8 failed (%s) %u", esp_err_to_name(result), size);
        hy_wdc6se->illuminance     = NAN;
        hy_wdc6se->solar_radiation = NAN;
    } else {
        hy_wdc6se->illuminance     = values[0];
        hy_wdc6se->solar_radiation = values[1];
    }
    */
}

static void sdi12_x_hy_wdc6se_info( sdi12_master_handle_t sdi12_master_hdl ) {
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
    result = sdi12_master_acknowledge_active(sdi12_master_hdl, 'x', &active);
    if(result != ESP_OK) {
        ESP_LOGE(APP_TAG, "sdi12_master_acknowledge_active failed (%s)", esp_err_to_name(result));
    } else {
        ESP_LOGI(APP_TAG, "sdi-12 sensor active: %s", active ? "true" : "false");
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    /* send identification */
    sdi12_master_sensor_identification_t sdi12_identification;
    result = sdi12_master_send_identification(sdi12_master_hdl, 'x', &sdi12_identification);
    if(result != ESP_OK) {
        ESP_LOGE(APP_TAG, "sdi12_master_send_identification failed (%s)", esp_err_to_name(result));
    } else {
        ESP_LOGI(APP_TAG, "sdi-12 version:        %s", sdi12_identification.sdi12_version);
        ESP_LOGI(APP_TAG, "vendor identification: %s", sdi12_identification.vendor_identification);
        ESP_LOGI(APP_TAG, "sensor model:          %s", sdi12_identification.sensor_model);
        ESP_LOGI(APP_TAG, "sensor version:        %s", sdi12_identification.sensor_version);
        ESP_LOGI(APP_TAG, "sensor information:    %s", sdi12_identification.sensor_information);
    }
}

static void sdi12_x_hy_wdc6se( sdi12_master_handle_t sdi12_master_hdl, bool info, bool measure ) {
    if(info) sdi12_x_hy_wdc6se_info(sdi12_master_hdl);
    if(measure) {
        hy_wdc6se_t hy_wdc6se;
        sdi12_x_hy_wdc6se_measure(sdi12_master_hdl, &hy_wdc6se);
        sdi12_x_hy_wdc6se_print(hy_wdc6se);
    }
}

static void sdi12_0_task( void *pvParameters ) {
    TickType_t xLastWakeTime = xTaskGetTickCount ();
    
    sdi12_master_config_t sdi12_master_cfg = SDI12_MASTER_CONFIG_DEFAULT;
    sdi12_master_handle_t sdi12_master_hdl = NULL;
    
    /* attempt to instantiate sdi-12 handle */
    sdi12_master_init(&sdi12_master_cfg, &sdi12_master_hdl);
    if(sdi12_master_hdl == NULL) {
        ESP_LOGE(APP_TAG, "sdi12_master_init failed");
        esp_restart();
    }

    bool show_info = true;

    ESP_LOGI(APP_TAG, "sdi-12 master fw version: %s", sdi12_master_get_fw_version());

    // task loop entry point
    for ( ;; ) {
        ESP_LOGI(APP_TAG, "######################## SDI-12 - START #########################");

        //sdi12_x_hy_wdc6se( sdi12_master_hdl, show_info, true );
        show_info = false;

        sdi12_0_cr6( sdi12_master_hdl );

        ESP_LOGI(APP_TAG, "######################## SDI-12 - END ###########################");

        // pause the task per defined wait period
        vTaskDelaySecUntil( &xLastWakeTime, 60 );
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
        sdi12_0_task, 
        SDI12_0_TASK_NAME, 
        SDI12_0_TASK_STACK_SIZE, 
        NULL, 
        SDI12_0_TASK_PRIORITY, 
        NULL, 
        APP_CPU_NUM );
}