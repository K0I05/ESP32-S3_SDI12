/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/**
 * @file sdi12_master.c
 *
 * ESP-IDF driver for SDI-12 sensors through an LTC2873 RS-232/RS-485 transceiver.
 *
 * Ported from esp-open-rtos
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#include "include/sdi12_master.h"
#include <string.h>
#include <stdio.h>
#include <sdkconfig.h>
#include <esp_types.h>
#include <esp_log.h>
#include <esp_check.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/**
 * possible BMP280 registers
 */


#define SDI12_MASTER_POWERUP_DELAY_MS      UINT16_C(25)  /*!< */
#define SDI12_MASTER_APPSTART_DELAY_MS     UINT16_C(25)
#define SDI12_MASTER_CMD_DELAY_MS          UINT16_C(5)

/*
 * macro definitions
*/
#define ESP_TIMEOUT_CHECK(start, len) ((uint64_t)(esp_timer_get_time() - (start)) >= (len))
#define ESP_ARG_CHECK(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

/*
* static constant declarations
*/
static const char *TAG = "sdi12_master";

static const uint8_t sdi12_master_send_data_commands[] = {
    SDI12_MASTER_D0_COMMAND,
    SDI12_MASTER_D1_COMMAND,
    SDI12_MASTER_D2_COMMAND,
    SDI12_MASTER_D3_COMMAND,
    SDI12_MASTER_D4_COMMAND,
    SDI12_MASTER_D5_COMMAND,
    SDI12_MASTER_D6_COMMAND,
    SDI12_MASTER_D7_COMMAND,
    SDI12_MASTER_D8_COMMAND,
    SDI12_MASTER_D9_COMMAND
};

uint8_t sdi12_master_char_to_dec(const char c) {
    if ((c >= '0') && (c <= '9')) return c - '0';
    if ((c >= 'a') && (c <= 'z')) return c - 'a' + 10;
    if ((c >= 'A') && (c <= 'Z')) {
        return c - 'A' + 37;
    } else {
        return c;
    }
}

char sdi12_master_dec_to_char(const uint8_t b) {
    if (b <= 9) return b + '0';
    if ((b >= 10) && (b <= 36)) return b + 'a' - 10;
    if ((b >= 37) && (b <= 62)) {
        return b + 'A' - 37;
    } else {
        return b;
    }
}

/**
 * @brief Precision delay function for mark and break serial transmissions.  This function 
 * should not be used for extended delays.  Consider using `vTaskDelay` for extended delays 
 * to avoid triggering task watchdog events.
 * 
 * @param delay_us Delay in microseconds.
 */
static inline void sdi12_master_delay(const uint32_t delay_us) {
    int64_t end_time = esp_timer_get_time() + (int64_t)delay_us;
    while(esp_timer_get_time() < end_time);
}

/**
 * @brief Writes a mark signal to the SDI-12 bus.
 * 
 * @param handle SDI-12 master handle.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
static inline esp_err_t sdi12_master_mark(sdi12_master_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( gpio_set_level(handle->dev_config.uart_tx_io_num, 1), TAG, "Unable to set tx gpio level, mark failed" );

    sdi12_master_delay(SDI12_MASTER_MARK_US);

    return ESP_OK;
}

/**
 * @brief Writes a break signal to the SDI-12 bus.
 * 
 * @param handle SDI-12 master handle.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
static inline esp_err_t sdi12_master_break(sdi12_master_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( gpio_set_level(handle->dev_config.uart_tx_io_num, 0), TAG, "Unable to set tx gpio level, break failed" );
    ESP_RETURN_ON_ERROR( gpio_set_level(handle->dev_config.dc2364a_de_io_num, 1), TAG, "Unable to set de gpio level, break failed" );
    ESP_RETURN_ON_ERROR( gpio_set_level(handle->dev_config.dc2364a_re_io_num, 1), TAG, "Unable to set re gpio level, break failed" );

    sdi12_master_delay(SDI12_MASTER_BREAK_US);

    return ESP_OK;
}

/**
 * @brief Initializes SDI-12 master GPIO pins and levels.
 * 
 * @param handle SDI-12 master handle.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
static inline esp_err_t sdi12_master_gpio_init(sdi12_master_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* mask bits for gpio pins */
    uint64_t pin_bit_mask = ((1ULL << handle->dev_config.dc2364a_mode_io_num) | (1ULL << handle->dev_config.dc2364a_de_io_num) |
                             (1ULL << handle->dev_config.dc2364a_re_io_num)   | (1ULL << handle->dev_config.dc2364a_te_io_num) |
                             (1ULL << handle->dev_config.dc2364a_io_io_num)   | (1ULL << handle->dev_config.uart_tx_io_num));

    /* set gpio configuration */
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = pin_bit_mask,
        .pull_down_en = 0,
        .pull_up_en   = 0,
    };
    ESP_RETURN_ON_ERROR( gpio_config(&io_conf), TAG, "Unable to configure gpio, setup failed" );

    /* initialize sdi-12 gpio levels */
    ESP_RETURN_ON_ERROR( gpio_set_level(handle->dev_config.dc2364a_te_io_num, 1), TAG, "Unable to set te gpio level, setup failed" );
    ESP_RETURN_ON_ERROR( gpio_set_level(handle->dev_config.dc2364a_mode_io_num, 1), TAG, "Unable to set mode gpio level, setup failed" );
    ESP_RETURN_ON_ERROR( gpio_set_level(handle->dev_config.dc2364a_io_io_num, 1), TAG, "Unable to set io gpio level, setup failed" );

    //gpio_dump_io_configuration(stdout, pin_bit_mask);

    return ESP_OK;
}

static inline esp_err_t sdi12_master_uart_enable(sdi12_master_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    // sdi-12: init uart 1200bps, 7 bits, even parity, 1 stop bit
    uart_config_t uart_config = (uart_config_t) { 
        .baud_rate  = SDI12_MASTER_UART_BAUD_RATE,
        .data_bits  = UART_DATA_7_BITS,
        .parity     = UART_PARITY_EVEN,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    int intr_alloc_flags = 0;

    /* configure uart */
    ESP_RETURN_ON_ERROR( uart_driver_install(handle->dev_config.uart_port, SDI12_MASTER_UART_RX_BUFFER_SIZE * 2, 0, 0, NULL, intr_alloc_flags), TAG, "unable to install uart drive, uart enable failed");
    ESP_RETURN_ON_ERROR( uart_param_config(handle->dev_config.uart_port, &uart_config), TAG, "unable to configure uart parameters, uart enable failed");
    ESP_RETURN_ON_ERROR( uart_set_pin(handle->dev_config.uart_port, handle->dev_config.uart_tx_io_num, handle->dev_config.uart_rx_io_num, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE), TAG, "unable to set uart pins, uart enable failed");

    return ESP_OK;
}

static inline esp_err_t sdi12_master_uart_disable(sdi12_master_handle_t handle) {
    /* validate arguments */
    ESP_ARG_CHECK( handle );

    ESP_RETURN_ON_ERROR( uart_driver_delete(handle->dev_config.uart_port), TAG, "unable to delete uart drive, uart disable failed");

    return ESP_OK;
}

static inline const char* sdi12_master_measurement_command_string(const char address, const sdi12_master_measurement_commands_t command) {
    char *cmd = (char *)malloc(6 * sizeof(char));

    switch(command) {
        case SDI12_MASTER_M_COMMAND:
            cmd[0] = address;
            cmd[1] = 'M';
            cmd[2] = '!';
            cmd[3] = '\0';
            return cmd;
        case SDI12_MASTER_M1_COMMAND:
            cmd[0] = address;
            cmd[1] = 'M';
            cmd[2] = '1';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_M2_COMMAND:
            cmd[0] = address;
            cmd[1] = 'M';
            cmd[2] = '2';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_M3_COMMAND:
            cmd[0] = address;
            cmd[1] = 'M';
            cmd[2] = '3';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_M4_COMMAND:
            cmd[0] = address;
            cmd[1] = 'M';
            cmd[2] = '4';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_M5_COMMAND:
            cmd[0] = address;
            cmd[1] = 'M';
            cmd[2] = '5';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_M6_COMMAND:
            cmd[0] = address;
            cmd[1] = 'M';
            cmd[2] = '6';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_M7_COMMAND:
            cmd[0] = address;
            cmd[1] = 'M';
            cmd[2] = '7';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_M8_COMMAND:
            cmd[0] = address;
            cmd[1] = 'M';
            cmd[2] = '8';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_M9_COMMAND:
            cmd[0] = address;
            cmd[1] = 'M';
            cmd[2] = '9';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        default:
            return NULL;
    };
}

static inline const char* sdi12_master_send_data_command_string(const char address, const sdi12_master_send_data_commands_t command) {
    char *cmd = (char *)malloc(6 * sizeof(char));

    switch(command) {
        case SDI12_MASTER_D0_COMMAND:
            cmd[0] = address;
            cmd[1] = 'D';
            cmd[2] = '0';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_D1_COMMAND:
            cmd[0] = address;
            cmd[1] = 'D';
            cmd[2] = '1';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_D2_COMMAND:
            cmd[0] = address;
            cmd[1] = 'D';
            cmd[2] = '2';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_D3_COMMAND:
            cmd[0] = address;
            cmd[1] = 'D';
            cmd[2] = '3';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_D4_COMMAND:
            cmd[0] = address;
            cmd[1] = 'D';
            cmd[2] = '4';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_D5_COMMAND:
            cmd[0] = address;
            cmd[1] = 'D';
            cmd[2] = '5';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_D6_COMMAND:
            cmd[0] = address;
            cmd[1] = 'D';
            cmd[2] = '6';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_D7_COMMAND:
            cmd[0] = address;
            cmd[1] = 'D';
            cmd[2] = '7';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_D8_COMMAND:
            cmd[0] = address;
            cmd[1] = 'D';
            cmd[2] = '8';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        case SDI12_MASTER_D9_COMMAND:
            cmd[0] = address;
            cmd[1] = 'D';
            cmd[2] = '9';
            cmd[3] = '!';
            cmd[4] = '\0';
            return cmd;
        default:
            return NULL;
    };
}

/**
 * @brief Determines the SDI-12 measurement mode from the measurement command.
 * 
 * @param command SDI-12 measurement command.
 * @return sdi12_master_measurement_modes_t SDI-12 measurement start mode.
 */
static inline sdi12_master_measurement_modes_t sdi12_master_measurement_mode(const sdi12_master_measurement_commands_t command) {
    switch(command) {
        case SDI12_MASTER_M_COMMAND:
        case SDI12_MASTER_M1_COMMAND:
        case SDI12_MASTER_M2_COMMAND:
        case SDI12_MASTER_M3_COMMAND:
        case SDI12_MASTER_M4_COMMAND:
        case SDI12_MASTER_M5_COMMAND:
        case SDI12_MASTER_M6_COMMAND:
        case SDI12_MASTER_M7_COMMAND:
        case SDI12_MASTER_M8_COMMAND:
        case SDI12_MASTER_M9_COMMAND:
            return SDI12_MASTER_MODE_QUEUED;
        case SDI12_MASTER_MC_COMMAND:
        case SDI12_MASTER_MC1_COMMAND:
        case SDI12_MASTER_MC2_COMMAND:
        case SDI12_MASTER_MC3_COMMAND:
        case SDI12_MASTER_MC4_COMMAND:
        case SDI12_MASTER_MC5_COMMAND:
        case SDI12_MASTER_MC6_COMMAND:
        case SDI12_MASTER_MC7_COMMAND:
        case SDI12_MASTER_MC8_COMMAND:
        case SDI12_MASTER_MC9_COMMAND:
            return SDI12_MASTER_MODE_QUEUED_CRC;
        case SDI12_MASTER_C_COMMAND:
        case SDI12_MASTER_C1_COMMAND:
        case SDI12_MASTER_C2_COMMAND:
        case SDI12_MASTER_C3_COMMAND:
        case SDI12_MASTER_C4_COMMAND:
        case SDI12_MASTER_C5_COMMAND:
        case SDI12_MASTER_C6_COMMAND:
        case SDI12_MASTER_C7_COMMAND:
        case SDI12_MASTER_C8_COMMAND:
        case SDI12_MASTER_C9_COMMAND:
            return SDI12_MASTER_MODE_CONCURRENT;
        case SDI12_MASTER_CC_COMMAND:
        case SDI12_MASTER_CC1_COMMAND:
        case SDI12_MASTER_CC2_COMMAND:
        case SDI12_MASTER_CC3_COMMAND:
        case SDI12_MASTER_CC4_COMMAND:
        case SDI12_MASTER_CC5_COMMAND:
        case SDI12_MASTER_CC6_COMMAND:
        case SDI12_MASTER_CC7_COMMAND:
        case SDI12_MASTER_CC8_COMMAND:
        case SDI12_MASTER_CC9_COMMAND:
            return SDI12_MASTER_MODE_CONCURRENT_CRC;
        case SDI12_MASTER_R0_COMMAND:
        case SDI12_MASTER_R1_COMMAND:
        case SDI12_MASTER_R2_COMMAND:
        case SDI12_MASTER_R3_COMMAND:
        case SDI12_MASTER_R4_COMMAND:
        case SDI12_MASTER_R5_COMMAND:
        case SDI12_MASTER_R6_COMMAND:
        case SDI12_MASTER_R7_COMMAND:
        case SDI12_MASTER_R8_COMMAND:
        case SDI12_MASTER_R9_COMMAND:
            return SDI12_MASTER_MODE_CONTINUOUS;
        case SDI12_MASTER_RC0_COMMAND:
        case SDI12_MASTER_RC1_COMMAND:
        case SDI12_MASTER_RC2_COMMAND:
        case SDI12_MASTER_RC3_COMMAND:
        case SDI12_MASTER_RC4_COMMAND:
        case SDI12_MASTER_RC5_COMMAND:
        case SDI12_MASTER_RC6_COMMAND:
        case SDI12_MASTER_RC7_COMMAND:
        case SDI12_MASTER_RC8_COMMAND:
        case SDI12_MASTER_RC9_COMMAND:
            return SDI12_MASTER_MODE_CONTINUOUS_CRC;
        default:
            return SDI12_MASTER_MODE_QUEUED;
    };
}

/**
 * @brief Sends a measurement queuing command to the SDI-12 sensor and parses the response.
 * Continuous measurement commands are not supported or handled by this function.
 * 
 * @param[in] handle SDI-12 master handle.
 * @param[in] address SDI-12 sensor address.
 * @param[in] command SDI-12 measurement command to send and process.
 * @param[out] queue SDI-12 measurement queuing structure (data ready delay and number of values).
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
static inline esp_err_t sdi12_master_measurement(sdi12_master_handle_t handle, const char address, const sdi12_master_measurement_commands_t command, sdi12_master_measurement_queue_t *const queue) {
    sdi12_master_measurement_queue_t out_queue;
    const char* response;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* determine measurement mode */
    sdi12_master_measurement_modes_t mode = sdi12_master_measurement_mode(command);

    /* validate measurement mode */
    if(mode == SDI12_MASTER_MODE_CONTINUOUS || mode == SDI12_MASTER_MODE_CONTINUOUS_CRC) {
        /* continuous and continuous crc are not supported or handled by this function */
        return ESP_ERR_NOT_SUPPORTED;
    }

    /* build measurement command */
    const char* cmd = sdi12_master_measurement_command_string(address, command);

    // start measurement command response status code
    ESP_RETURN_ON_ERROR(sdi12_master_send_command(handle, cmd, &response), TAG, "start measurement command failed");

    /* validate response size */
    ESP_RETURN_ON_FALSE((strnlen(response, SDI12_MASTER_M_CMD_RESPONSE_MAX_SIZE) < SDI12_MASTER_M_CMD_RESPONSE_MAX_SIZE), ESP_ERR_INVALID_SIZE, TAG, "response length cannot exceed %u characters, start measurement command failed", SDI12_MASTER_M_CMD_RESPONSE_MAX_SIZE);

    /* validate address */
    ESP_RETURN_ON_FALSE(((char)response[0] == address), ESP_ERR_INVALID_RESPONSE, TAG, "sdi-12 address is incorrect, start measurement command failed");

    /* validate if concurrent atttnn vs queued atttn and parse response */
    if(mode == SDI12_MASTER_MODE_CONCURRENT || mode == SDI12_MASTER_MODE_CONCURRENT_CRC) {
        const char delay[] = { (char)response[1], (char)response[2], (char)response[3], '\0' };
        const char vals[]  = { (char)response[4], (char)response[5], '\0' };

        out_queue.data_ready_delay = (uint8_t)atoi(delay);
        out_queue.number_of_values = (uint8_t)atoi(vals);
    } else {
        const char delay[] = { (char)response[1], (char)response[2], (char)response[3], '\0' };
        const char vals[]  = { (char)response[4], '\0' };

        out_queue.data_ready_delay = (uint8_t)atoi(delay);
        out_queue.number_of_values = (uint8_t)atoi(vals);
    }

    /* set output parameter */
    *queue = out_queue;
    
    return ESP_OK;
}

/**
 * @brief Parses an SDI-12 D0..D9 command response from device with <CR> and <LF> characters removed (i.e. 0+3.14+2.718+1.414).
 * 
 * @param response SDI-12 D0..D9 command response from device to parse.
 * @param[out] values Parsed SDI-12 measurement values parsed from response.
 * @param[out] size Number of SDI-12 measurement values parsed from response.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
static inline esp_err_t sdi12_master_parse_d_response(const char* response, float **const values, size_t *const size) {
    uint8_t rsp_char_index = 0;
    uint8_t tok_count      = 0;
    uint8_t rsp_len        = strnlen(response, SDI12_MASTER_RESPONSE_MAX_SIZE);
    float*  out_values     = (float *)calloc(SDI12_MASTER_D_CMD_VALUES_MAX_SIZE, sizeof(float));

    /* validate response size */
    ESP_RETURN_ON_FALSE((strnlen(response, SDI12_MASTER_RESPONSE_MAX_SIZE) < SDI12_MASTER_RESPONSE_MAX_SIZE), ESP_ERR_INVALID_SIZE, TAG, "response length cannot exceed %u characters, parse d response failed", SDI12_MASTER_RESPONSE_MAX_SIZE);

    do {
        /* check for token delimiter in the response string */
        if(response[rsp_char_index] == '+' || response[rsp_char_index] == '-') {
            bool    tok_end        = false;
            uint8_t tok_char_index = 0;
            char    token[SDI12_MASTER_DATA_VALUE_MAX_SIZE];
            /* token found - preserve signing (+/-) */
            token[tok_char_index] = response[rsp_char_index];
            do {
                /* advance to the next char in the response string */
                token[tok_char_index++] = response[rsp_char_index++];
                /* check for next token delimiter in the response string */
                if((rsp_char_index + 1) == rsp_len || response[rsp_char_index + 1] == '+' || response[rsp_char_index + 1] == '-') {
                    /* if we landed here, the token was extracted from the response */
                    /* string, parse token to a float data-type and exit this loop */
                    token[tok_char_index++] = '\0';
                    out_values[tok_count++] = (float)atof(token);
                    tok_end = true;
                }
            } while (tok_end == false);
        }
        rsp_char_index++;
    } while (rsp_char_index < rsp_len);

    *size   = tok_count;
    *values = out_values;

    return ESP_OK;
}

esp_err_t sdi12_master_init(const sdi12_master_config_t *sdi12_master_config, sdi12_master_handle_t *sdi12_master_handle) {
    esp_err_t ret = ESP_OK;

    /* validate arguments */
    ESP_ARG_CHECK( sdi12_master_handle );

    /* validate memory availability for handle */
    sdi12_master_handle_t out_handle;
    out_handle = (sdi12_master_handle_t)calloc(1, sizeof(*out_handle));
    ESP_GOTO_ON_FALSE(out_handle, ESP_ERR_NO_MEM, err, TAG, "no memory for sdi12 master device for init");

    /* copy configuration */
    out_handle->dev_config = *sdi12_master_config;

    /* set output parameter */
    *sdi12_master_handle = out_handle;

    return ESP_OK;
    err:
        return ret;
}

esp_err_t sdi12_master_send_command(sdi12_master_handle_t handle, const char* command, const char **const response) {
    /* validate arguments */
    ESP_ARG_CHECK( handle && command && response );

    /* validate command size */
    ESP_RETURN_ON_FALSE((strnlen(command, SDI12_MASTER_COMMAND_MAX_SIZE) < SDI12_MASTER_COMMAND_MAX_SIZE), ESP_ERR_INVALID_SIZE, TAG, "command length cannot exceed %u characters, send command failed", SDI12_MASTER_COMMAND_MAX_SIZE);

    /* attempt to initialize gpio pins and levels for ltc2873 */
    ESP_RETURN_ON_ERROR( sdi12_master_gpio_init(handle), TAG, "unable to setup gpio pins, init failed");

    /* send break */
    ESP_RETURN_ON_ERROR(sdi12_master_break(handle), TAG, "unable to send break, send command failed");

    /* send mark */
    ESP_RETURN_ON_ERROR(sdi12_master_mark(handle), TAG, "unable to send master mark, send command failed");

    /* enable uart */
    if(uart_is_driver_installed(handle->dev_config.uart_port) == false) {
        ESP_RETURN_ON_ERROR(sdi12_master_uart_enable(handle), TAG, "unable to enable uart, send command failed");
    }

    /* send command */
    for(int i = 0; i < strnlen(command, SDI12_MASTER_COMMAND_MAX_SIZE); i++) {
        uart_write_bytes(handle->dev_config.uart_port, &command[i], 1);
    }

    // command transaction delay (byte time * command length)
    sdi12_master_delay( (SDI12_MASTER_MARK_US * strnlen(command, SDI12_MASTER_COMMAND_MAX_SIZE)) );

    // set de and re direction to read
    ESP_RETURN_ON_ERROR( gpio_set_level(handle->dev_config.dc2364a_de_io_num, 0), TAG, "unable to set de gpio level, send command failed" );
    ESP_RETURN_ON_ERROR( gpio_set_level(handle->dev_config.dc2364a_re_io_num, 0), TAG, "unable to set re gpio level, send command failed" );

    // init end time and flags, sensor must complete the transmission within 810-ms (total response time)
    int64_t end_time    = esp_timer_get_time() + (int64_t)SDI12_MASTER_MAX_TOTAL_RESPONSE_TIME_US;
    bool    end_command = false;

    // configure temporary buffers for the output response and incoming data from uart
    char* out_response = (char *)calloc(SDI12_MASTER_RESPONSE_MAX_SIZE, sizeof(char));
    //uint8_t* rx_buffer = (uint8_t *)calloc(SDI12_MASTER_RESPONSE_MAX_SIZE, sizeof(uint8_t));
    uint8_t rx_buffer[SDI12_MASTER_RESPONSE_MAX_SIZE] = { 0 };
    
    // poll for sensor response otherwise a timeout will be raised
    while ((esp_timer_get_time() < end_time) && (end_command == false)) {
        //
        // read uart bytes and concat response string
        //
        uint8_t rx_read_len = uart_read_bytes(handle->dev_config.uart_port, &rx_buffer, SDI12_MASTER_RESPONSE_MAX_SIZE, 100 / portTICK_PERIOD_MS);
        //
        // iterate one byte at a time and remove <CR><LF> chars
        //
        if(rx_read_len > 0) {
            uint8_t response_index = 0;
            for(uint8_t i = 0; i < rx_read_len; i++) {
                char c = (char)rx_buffer[i];
                // concat response without <CR><LF>
                if(c != '\n' || c != '\r') {
                    out_response[response_index++] = c;
                }
                // validate end of response
                if(c == '\r') {
                    out_response[response_index++] = '\0';
                    end_command = true;
                }
            }
        }
    }

    /* disable uart */
    ESP_RETURN_ON_ERROR(sdi12_master_uart_disable(handle), TAG, "unable to disable uart, send command failed");

    /* validate sensor responded within the max total response time */
    ESP_RETURN_ON_FALSE(end_command, ESP_ERR_TIMEOUT, TAG, "response timed out, send command failed");

    /* delay a little before next command */
    vTaskDelay(pdMS_TO_TICKS(SDI12_MASTER_DELAY_AFTER_TRANSMIT_US / 1000));

    /* set output parameters */
    *response = out_response;
    
    return ESP_OK;
}

esp_err_t sdi12_master_recorder(sdi12_master_handle_t handle, const char address, const sdi12_master_measurement_commands_t command, float **const values, size_t *const size) {
    sdi12_master_measurement_queue_t queue;
    uint8_t values_index    = 0;
    uint8_t values_counter  = 0;
    uint8_t send_data_index = 0;
    
    /* validate arguments */
    ESP_ARG_CHECK( handle && values && size );

    /* 
        handle measurement modes: queued, queued crc, concurrent, 
        concurrent crc, continuous and continuous crc

        sdi12_master_measurement_modes_t
    */

    /* send measurement command */
    ESP_RETURN_ON_ERROR(sdi12_master_measurement(handle, address, command, &queue), TAG, "measurement command unsuccessful, recorder failed");

    /* delay before requesting measurement values */
    vTaskDelay(pdMS_TO_TICKS(queue.data_ready_delay * 1000));

    /* instantiate output values based on expected number of values */
    float* out_values = (float *)calloc(queue.number_of_values, sizeof(float));

    /* 
        determine number of send data commands (D0..D9) required to 
        collect all measurement values (i.e. queue.number_of_values)
    */
    do {
        float* vals = NULL;
        size_t vals_size = 0;
        const char* response;

        /* retrieve send data command type from send data index (D0..D9) */
        sdi12_master_send_data_commands_t send_data = sdi12_master_send_data_commands[send_data_index++];

        /* build send data command from d command index (D0..D9) */
        const char* send_data_cmd = sdi12_master_send_data_command_string(address, send_data);

        /* send data command and wait for d command response */
        ESP_RETURN_ON_ERROR(sdi12_master_send_command(handle, send_data_cmd, &response), TAG, "send data command unsuccessful, recorder failed");

        /* parse measurement values from d command response */
        ESP_RETURN_ON_ERROR(sdi12_master_parse_d_response(response, &vals, &vals_size), TAG, "parse d response unsuccessful, recorder failed");

        /* set output measurement values */
        for(uint8_t i = 0; i < vals_size; i++) {
            out_values[values_index++] = vals[i];
        }

        /* increment measurement values counter */
        values_counter = values_counter + vals_size;

    } while (values_counter < queue.number_of_values);

    /* set output parameters */
    *values = out_values;
    *size   = queue.number_of_values;

    return ESP_OK;
}

esp_err_t sdi12_master_acknowledge_active(sdi12_master_handle_t handle, const char address, bool *const active) {
    const char* response;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    // build acknowledge active command: a!
    const char command[] = { address, '!', '\0' };

    // return acknowledge active command response status code
    ESP_RETURN_ON_ERROR(sdi12_master_send_command(handle, command, &response), TAG, "acknowledge active command failed");

    /* validate response: a */

    /* validate response size */
    ESP_RETURN_ON_FALSE((strnlen(response, SDI12_MASTER_AK_CMD_RESPONSE_MAX_SIZE) < SDI12_MASTER_AK_CMD_RESPONSE_MAX_SIZE), ESP_ERR_INVALID_SIZE, TAG, "response length cannot exceed %u characters, acknowledge active command failed", SDI12_MASTER_AK_CMD_RESPONSE_MAX_SIZE);

    /* set output parameter */
    if((char)response[0] == address) {
        *active = true;
    } else {
        *active = false;
    }

    return ESP_OK;
}

esp_err_t sdi12_master_send_identification(sdi12_master_handle_t handle, const char address, sdi12_master_sensor_identification_t *const identification) {
    const char* response;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    // build send identification command: aI!
    const char command[] = { address, 'I', '!', '\0' };

    // return send identification command response status code
    ESP_RETURN_ON_ERROR(sdi12_master_send_command(handle, command, &response), TAG, "send identification command failed");

    /* validate and parse response: allccccccccmmmmmmvvvxxx...xx */

    /* validate response size */
    ESP_RETURN_ON_FALSE((strnlen(response, SDI12_MASTER_I_CMD_RESPONSE_MAX_SIZE) < SDI12_MASTER_I_CMD_RESPONSE_MAX_SIZE), ESP_ERR_INVALID_SIZE, TAG, "response length cannot exceed %u characters, send identification command failed", SDI12_MASTER_I_CMD_RESPONSE_MAX_SIZE);

    /* validate address */
    ESP_RETURN_ON_FALSE(((char)response[0] == address), ESP_ERR_INVALID_RESPONSE, TAG, "sdi-12 address is incorrect, send identification command failed");

    /* parse information */
    sdi12_master_sensor_identification_t out_ident;
    out_ident.sdi12_version[0]         = (char)response[1];
    out_ident.sdi12_version[1]         = '.';
    out_ident.sdi12_version[2]         = (char)response[2];
    out_ident.sdi12_version[3]         = '\0';
    out_ident.vendor_identification[0] = (char)response[3];
    out_ident.vendor_identification[1] = (char)response[4];
    out_ident.vendor_identification[2] = (char)response[5];
    out_ident.vendor_identification[3] = (char)response[6];
    out_ident.vendor_identification[4] = (char)response[7];
    out_ident.vendor_identification[5] = (char)response[8];
    out_ident.vendor_identification[6] = (char)response[9];
    out_ident.vendor_identification[7] = (char)response[10];
    out_ident.vendor_identification[8] = '\0';
    out_ident.sensor_model[0]          = (char)response[11];
    out_ident.sensor_model[1]          = (char)response[12];
    out_ident.sensor_model[2]          = (char)response[13];
    out_ident.sensor_model[3]          = (char)response[14];
    out_ident.sensor_model[4]          = (char)response[15];
    out_ident.sensor_model[5]          = '\0';
    out_ident.sensor_version[0]        = (char)response[16];
    out_ident.sensor_version[1]        = (char)response[17];
    out_ident.sensor_version[2]        = (char)response[18];
    out_ident.sensor_version[3]        = '\0';
    out_ident.sensor_information[0]    = (char)response[19];
    out_ident.sensor_information[1]    = (char)response[20];
    out_ident.sensor_information[2]    = (char)response[21];
    out_ident.sensor_information[3]    = (char)response[22];
    out_ident.sensor_information[4]    = (char)response[23];
    out_ident.sensor_information[5]    = (char)response[24];
    out_ident.sensor_information[6]    = (char)response[25];
    out_ident.sensor_information[7]    = (char)response[26];
    out_ident.sensor_information[8]    = (char)response[27];
    out_ident.sensor_information[9]    = (char)response[28];
    out_ident.sensor_information[10]   = (char)response[29];
    out_ident.sensor_information[11]   = (char)response[30];
    out_ident.sensor_information[12]   = (char)response[31];
    out_ident.sensor_information[13]   = '\0';

    /* set output parameter */
    *identification = out_ident;

    return ESP_OK;
}

esp_err_t sdi12_master_change_address(sdi12_master_handle_t handle, const char address, const char new_address) {
    const char* response;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    // build change address command: aAb!
    const char command[] = { address, 'A', new_address, '!', '\0' };

    // return change address command response status code
    ESP_RETURN_ON_ERROR(sdi12_master_send_command(handle, command, &response), TAG, "change address command failed");

    /* validate response: b */

    /* validate response size */
    ESP_RETURN_ON_FALSE((strnlen(response, SDI12_MASTER_A_CMD_RESPONSE_MAX_SIZE) < SDI12_MASTER_A_CMD_RESPONSE_MAX_SIZE), ESP_ERR_INVALID_SIZE, TAG, "response length cannot exceed %u characters, change address command failed", SDI12_MASTER_A_CMD_RESPONSE_MAX_SIZE);

    /* validate address */
    ESP_RETURN_ON_FALSE(((char)response[0] == new_address), ESP_ERR_INVALID_RESPONSE, TAG, "sdi-12 address does not match new address, change address command failed");

    return ESP_OK;
}

esp_err_t sdi12_master_address_query(sdi12_master_handle_t handle, char *const address) {
    const char* response;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* build address query command: ?! */
    const char command[] = { '?', '!', '\0' };

    // address query command response status code
    ESP_RETURN_ON_ERROR(sdi12_master_send_command(handle, command, &response), TAG, "address query command failed");

    /* validate response: a */

    /* validate response size */
    ESP_RETURN_ON_FALSE((strnlen(response, SDI12_MASTER_Q_CMD_RESPONSE_MAX_SIZE) < SDI12_MASTER_Q_CMD_RESPONSE_MAX_SIZE), ESP_ERR_INVALID_SIZE, TAG, "response length cannot exceed %u characters, address query command failed", SDI12_MASTER_Q_CMD_RESPONSE_MAX_SIZE);

    /* set output parameter */
    *address = (char)response[0];

    return ESP_OK;
}

esp_err_t sdi12_master_start_measurement(sdi12_master_handle_t handle, const char address, sdi12_master_measurement_queue_t *const queue) {
    const char* response;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* build start measurement command: aM! */
    const char command[] = { address, 'M', '!', '\0' };

    // start measurement command response status code
    ESP_RETURN_ON_ERROR(sdi12_master_send_command(handle, command, &response), TAG, "start measurement command failed");

    /* validate response size */
    ESP_RETURN_ON_FALSE((strnlen(response, SDI12_MASTER_M_CMD_RESPONSE_MAX_SIZE) < SDI12_MASTER_M_CMD_RESPONSE_MAX_SIZE), ESP_ERR_INVALID_SIZE, TAG, "response length cannot exceed %u characters, start measurement command failed", SDI12_MASTER_M_CMD_RESPONSE_MAX_SIZE);

    /* validate address */
    ESP_RETURN_ON_FALSE(((char)response[0] == address), ESP_ERR_INVALID_RESPONSE, TAG, "sdi-12 address is incorrect, start measurement command failed");

    /* parse information */
    sdi12_master_measurement_queue_t out_queue;

    const char delay[] = { (char)response[1], (char)response[2], (char)response[3], '\0' };
    const char vals[]  = { (char)response[4], '\0' };

    out_queue.data_ready_delay = (uint8_t)atoi(delay);
    out_queue.number_of_values = (uint8_t)atoi(vals);

    /* set output parameter */
    *queue = out_queue;

    return ESP_OK;
}

esp_err_t sdi12_master_abort_measurement(sdi12_master_handle_t handle, const char address) {
    const char* response;

    /* validate arguments */
    ESP_ARG_CHECK( handle );

    /* build abort measurement command: a<CR><LF> */
    const char command[] = { address, 0x0D, 0x0A, '\0' };

    // abort measurement command response status code
    ESP_RETURN_ON_ERROR(sdi12_master_send_command(handle, command, &response), TAG, "start measurement command failed");

    return ESP_OK;
}
