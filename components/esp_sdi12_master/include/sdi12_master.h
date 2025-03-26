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
 * @file sdi12_master.h
 * @defgroup drivers sdi12 master
 * @{
 *
 * ESP-IDF driver for sdi12 sensors
 *
 * Copyright (c) 2024 Eric Gionet (gionet.c.eric@gmail.com)
 *
 * MIT Licensed as described in the file LICENSE
 */
#ifndef __SDI12_H__
#define __SDI12_H__

#include <stdint.h>
#include <stdbool.h>
#include <esp_err.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <esp_rom_gpio.h>
#include <soc/uart_periph.h>
#include "sdi12_master_version.h"


#ifdef __cplusplus
extern "C" {
#endif

/*

ESP32-S3 GPIO Pin-Out Reference: https://randomnerdtutorials.com/esp32-pinout-reference-gpios/

ESP32-S3 GPIO Summary:
- ESP32-S3 GPIO Usable Pins: 04, 13, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33
- ESP32-S3 UART 0 GPIO Pins: 01 (Tx), 03 (Rx)
- ESP32-S3 UART 2 GPIO Pins: 17 (Tx), 16 (Rx)

*/

#define SDI12_MASTER_UART_RX_BUFFER_SIZE            (1024)        /*!< uart receive maximum buffer size */
#define SDI12_MASTER_UART_PORT_NUM                  (UART_NUM_2)  /*!< uart port number */
#define SDI12_MASTER_TXD_IO_NUM                     (GPIO_NUM_17) /*!< uart ttl transmit */
#define SDI12_MASTER_RXD_IO_NUM                     (GPIO_NUM_16) /*!< uart ttl receive */
#define SDI12_MASTER_RTS_IO_NUM                     (UART_PIN_NO_CHANGE)
#define SDI12_MASTER_CTS_IO_NUM                     (UART_PIN_NO_CHANGE)
#define SDI12_MASTER_IO_IO_NUM                      (GPIO_NUM_4)  /*!< logic supply ref voltage (dev board logic supply) */
#define SDI12_MASTER_DE_IO_NUM                      (GPIO_NUM_13) /*!< in rs485 mode a logic low disables rs-485 driver and high enables driver, in rs 232 a logic high enables fast mode (1Mbps) and low enables slow mode (250kbps) */
#define SDI12_MASTER_RE_IO_NUM                      (GPIO_NUM_18) /*!< logic high disables the rs485 receiver and low enables the receiver */
#define SDI12_MASTER_TE_IO_NUM                      (GPIO_NUM_19) /*!< logic low for 120-ohm termination or high for unterminated */
#define SDI12_MASTER_MODE_IO_NUM                    (GPIO_NUM_21) /*!< logic low for rs-232 and high for rs-485 */



/**
 * Send identification command aI! maximum number of characters for
 * sensor identification structure fields
 */
#define SDI12_MASTER_I_SENSOR_VER_MAX_SIZE 4        /*!< sensor identification structure, sdi-12 version number, version 1.4 is encoded as 14 (3-char + `\0` char) */
#define SDI12_MASTER_I_VENDOR_IDENT_MAX_SIZE 9      /*!< sensor identification structure, vendor identification (8-char + `\0` char) */
#define SDI12_MASTER_I_SENSOR_MODEL_MAX_SIZE 7      /*!< sensor identification structure, sensor model number (6-char + `\0` char) */
#define SDI12_MASTER_I_SENSOR_VER_MAX_SIZE 4        /*!< sensor identification structure, sensor version number (3-char + `\0` char) */
#define SDI12_MASTER_I_SENSOR_INFO_MAX_SIZE 14      /*!< sensor identification structure, sensor information (13-char + `\0` char) i.e. serial number, other */


/*
 * SDI12 Master macros
*/

/**
 * @brief Macro that initializes the `sdi12_master_config_t` 
 * configuration structure to default settings.
 */
#define SDI12_MASTER_CONFIG_DEFAULT {                                           \
        .uart_port           = SDI12_MASTER_UART_PORT_NUM,                      \
        .uart_tx_io_num      = SDI12_MASTER_TXD_IO_NUM,                         \
        .uart_rx_io_num      = SDI12_MASTER_RXD_IO_NUM,                         \
        .dc2364a_io_io_num   = SDI12_MASTER_IO_IO_NUM,                          \
        .dc2364a_de_io_num   = SDI12_MASTER_DE_IO_NUM,                          \
        .dc2364a_re_io_num   = SDI12_MASTER_RE_IO_NUM,                          \
        .dc2364a_te_io_num   = SDI12_MASTER_TE_IO_NUM,                          \
        .dc2364a_mode_io_num = SDI12_MASTER_MODE_IO_NUM }


/*
 * SDI12 enumerator and structure declarations
*/

/**
 * @brief SDI-12 master measurement base commands (M, MC, C, CC, R, and RC) 
 * enumerator.  This command is used to start measurement(s) onboard the SDI-12 
 * sensor which returns measurement queuing information (atttn or atttnn) from
 * queued or concurrent measurement commands or measurement values (like a D 
 * command response) from continuous measurement command when send data or 
 * measurement values are requested from the SDI-12 sensor.  See table 5 - 'The 
 * SDI-12 basic command/response set' in the SDI-12 specification document.
 * 
 * @note CRC measurement commands (MC, CC, and RC)  are not supported at this time.
 */
typedef enum sdi12_master_measurement_base_commands_e {
    SDI12_MASTER_M_COMMAND = 0, /*!< start measurement command, aM! (response is formatted as atttn) */
    SDI12_MASTER_M1_COMMAND,    /*!< start additional measurement command, aM1! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_M2_COMMAND,    /*!< start additional measurement command, aM2! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_M3_COMMAND,    /*!< start additional measurement command, aM3! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_M4_COMMAND,    /*!< start additional measurement command, aM4! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_M5_COMMAND,    /*!< start additional measurement command, aM5! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_M6_COMMAND,    /*!< start additional measurement command, aM6! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_M7_COMMAND,    /*!< start additional measurement command, aM7! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_M8_COMMAND,    /*!< start additional measurement command, aM8! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_M9_COMMAND,    /*!< start additional measurement command, aM9! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_MC_COMMAND,    /*!< start measurement command with CRC, aMC! (response is formatted as atttn) */
    SDI12_MASTER_MC1_COMMAND,   /*!< start additional measurement command with CRC, aMC1! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_MC2_COMMAND,   /*!< start additional measurement command with CRC, aMC2! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_MC3_COMMAND,   /*!< start additional measurement command with CRC, aMC3! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_MC4_COMMAND,   /*!< start additional measurement command with CRC, aMC4! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_MC5_COMMAND,   /*!< start additional measurement command with CRC, aMC5! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_MC6_COMMAND,   /*!< start additional measurement command with CRC, aMC6! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_MC7_COMMAND,   /*!< start additional measurement command with CRC, aMC7! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_MC8_COMMAND,   /*!< start additional measurement command with CRC, aMC8! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_MC9_COMMAND,   /*!< start additional measurement command with CRC, aMC9! (response is formatted as atttn), as defined by the sensor manufacturer */
    SDI12_MASTER_C_COMMAND,     /*!< start concurrent measurement command, aC! (response is formatted as atttnn) */
    SDI12_MASTER_C1_COMMAND,    /*!< start additional concurrent measurement command, aC1! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_C2_COMMAND,    /*!< start additional concurrent measurement command, aC2! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_C3_COMMAND,    /*!< start additional concurrent measurement command, aC3! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_C4_COMMAND,    /*!< start additional concurrent measurement command, aC4! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_C5_COMMAND,    /*!< start additional concurrent measurement command, aC5! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_C6_COMMAND,    /*!< start additional concurrent measurement command, aC6! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_C7_COMMAND,    /*!< start additional concurrent measurement command, aC7! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_C8_COMMAND,    /*!< start additional concurrent measurement command, aC8! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_C9_COMMAND,    /*!< start additional concurrent measurement command, aC9! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_CC_COMMAND,    /*!< start concurrent measurement command with CRC, aCC! (response is formatted as atttnn) */
    SDI12_MASTER_CC1_COMMAND,   /*!< start additional concurrent measurement command with CRC, aCC1! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_CC2_COMMAND,   /*!< start additional concurrent measurement command with CRC, aCC2! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_CC3_COMMAND,   /*!< start additional concurrent measurement command with CRC, aCC3! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_CC4_COMMAND,   /*!< start additional concurrent measurement command with CRC, aCC4! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_CC5_COMMAND,   /*!< start additional concurrent measurement command with CRC, aCC5! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_CC6_COMMAND,   /*!< start additional concurrent measurement command with CRC, aCC6! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_CC7_COMMAND,   /*!< start additional concurrent measurement command with CRC, aCC7! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_CC8_COMMAND,   /*!< start additional concurrent measurement command with CRC, aCC8! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_CC9_COMMAND,   /*!< start additional concurrent measurement command with CRC, aCC9! (response is formatted as atttnn), as defined by the sensor manufacturer */
    SDI12_MASTER_R0_COMMAND,    /*!< start continuous measurement command, aR0! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_R1_COMMAND,    /*!< start continuous measurement command, aR1! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_R2_COMMAND,    /*!< start continuous measurement command, aR2! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_R3_COMMAND,    /*!< start continuous measurement command, aR3! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_R4_COMMAND,    /*!< start continuous measurement command, aR4! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_R5_COMMAND,    /*!< start continuous measurement command, aR5! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_R6_COMMAND,    /*!< start continuous measurement command, aR6! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_R7_COMMAND,    /*!< start continuous measurement command, aR7! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_R8_COMMAND,    /*!< start continuous measurement command, aR8! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_R9_COMMAND,    /*!< start continuous measurement command, aR9! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_RC0_COMMAND,   /*!< start continuous measurement command with CRC, aR0! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_RC1_COMMAND,   /*!< start continuous measurement command with CRC, aR1! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_RC2_COMMAND,   /*!< start continuous measurement command with CRC, aR2! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_RC3_COMMAND,   /*!< start continuous measurement command with CRC, aR3! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_RC4_COMMAND,   /*!< start continuous measurement command with CRC, aR4! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_RC5_COMMAND,   /*!< start continuous measurement command with CRC, aR5! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_RC6_COMMAND,   /*!< start continuous measurement command with CRC, aR6! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_RC7_COMMAND,   /*!< start continuous measurement command with CRC, aR7! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_RC8_COMMAND,   /*!< start continuous measurement command with CRC, aR8! (response is formatted like the D command), as defined by the sensor manufacturer */
    SDI12_MASTER_RC9_COMMAND    /*!< start continuous measurement command with CRC, aR9! (response is formatted like the D command), as defined by the sensor manufacturer */
} sdi12_master_measurement_base_commands_t;

/**
 * @brief SDI-12 master send data (`D`) base commands enumerator.  This command is used 
 * in sequence of measurement start commands (M, MC, C, and CC) to retrieve measurement 
 * values once the measurements have been processed and are ready for collection. See 
 * measurement queue structure `sdi12_master_measurement_queue_t` that is returned from 
 * measurement start commands (M, MC, C, and CC).  See table 5 - 'The  SDI-12 basic 
 * command/response set' in the SDI-12 specification document.
 */
typedef enum sdi12_master_send_data_base_commands_e {
    SDI12_MASTER_D0_COMMAND = 0,/*!< send data command, aD0!, (response is formatted as a<values><CR><LF> or a<values><CRC><CR><LF> [see measurement commands `sdi12_master_measurement_commands_t`]) */
    SDI12_MASTER_D1_COMMAND,    /*!< send data command, aD1!, (response is formatted as a<values><CR><LF> or a<values><CRC><CR><LF> [see measurement commands `sdi12_master_measurement_commands_t`]) */
    SDI12_MASTER_D2_COMMAND,    /*!< send data command, aD2!, (response is formatted as a<values><CR><LF> or a<values><CRC><CR><LF> [see measurement commands `sdi12_master_measurement_commands_t`]) */
    SDI12_MASTER_D3_COMMAND,    /*!< send data command, aD3!, (response is formatted as a<values><CR><LF> or a<values><CRC><CR><LF> [see measurement commands `sdi12_master_measurement_commands_t`]) */
    SDI12_MASTER_D4_COMMAND,    /*!< send data command, aD4!, (response is formatted as a<values><CR><LF> or a<values><CRC><CR><LF> [see measurement commands `sdi12_master_measurement_commands_t`]) */
    SDI12_MASTER_D5_COMMAND,    /*!< send data command, aD5!, (response is formatted as a<values><CR><LF> or a<values><CRC><CR><LF> [see measurement commands `sdi12_master_measurement_commands_t`]) */
    SDI12_MASTER_D6_COMMAND,    /*!< send data command, aD6!, (response is formatted as a<values><CR><LF> or a<values><CRC><CR><LF> [see measurement commands `sdi12_master_measurement_commands_t`]) */
    SDI12_MASTER_D7_COMMAND,    /*!< send data command, aD7!, (response is formatted as a<values><CR><LF> or a<values><CRC><CR><LF> [see measurement commands `sdi12_master_measurement_commands_t`]) */
    SDI12_MASTER_D8_COMMAND,    /*!< send data command, aD8!, (response is formatted as a<values><CR><LF> or a<values><CRC><CR><LF> [see measurement commands `sdi12_master_measurement_commands_t`]) */
    SDI12_MASTER_D9_COMMAND     /*!< send data command, aD9!, (response is formatted as a<values><CR><LF> or a<values><CRC><CR><LF> [see measurement commands `sdi12_master_measurement_commands_t`]) */
} sdi12_master_send_data_base_commands_t;

/**
 * @brief SDI-12 master measurement modes enumerator.  Queued measurement 
 * commands return a measurement queuing structure `sdi12_master_measurement_queue_t`.  
 * See measurement queue structure `sdi12_master_measurement_queue_t` for 
 * further details.
 */
typedef enum sdi12_master_measurement_modes_e {
    SDI12_MASTER_MODE_QUEUED = 0,           /*<! queued measurements */
    SDI12_MASTER_MODE_QUEUED_CRC,           /*<! queued measurements and request crc */
    SDI12_MASTER_MODE_CONCURRENT,           /*<! queued concurrent measurements */
    SDI12_MASTER_MODE_CONCURRENT_CRC,       /*<! queued concurrent measurements and request crc */
    SDI12_MASTER_MODE_CONTINUOUS,           /*<! continuous measurements */
    SDI12_MASTER_MODE_CONTINUOUS_CRC,       /*<! continuous measurements and request crc */
    SDI12_MASTER_MODE_QUEUED_VERIFICATION   /*<! queued verification */
} sdi12_master_measurement_modes_t;

/**
 * @brief SDI-12 master send identification (i.e. format allccccccccmmmmmmvvvxxx...xx / 13 NRSYSINC 100000 1.2 101) 
 * structure.  See `sdi12_master_send_identification` function for further details.
 */
typedef struct sdi12_master_sensor_identification_s {
    char sdi12_version[SDI12_MASTER_I_SENSOR_VER_MAX_SIZE];             /*!< sdi-12 version number, version 1.4 is encoded as 14 (3-char + `\0` char) */
    char vendor_identification[SDI12_MASTER_I_VENDOR_IDENT_MAX_SIZE];   /*!< vendor identification (8-char + `\0` char) */
    char sensor_model[SDI12_MASTER_I_SENSOR_MODEL_MAX_SIZE];            /*!< sensor model number (6-char + `\0` char) */
    char sensor_version[SDI12_MASTER_I_SENSOR_VER_MAX_SIZE];            /*!< sensor version number (3-char + `\0` char) */
    char sensor_information[SDI12_MASTER_I_SENSOR_INFO_MAX_SIZE];       /*!< sensor information (13-char + `\0` char) i.e. serial number, other */
} sdi12_master_sensor_identification_t;

/**
 * @brief SDI-12 master measurement queue structure.  The measurement queue 
 * structure is returned from measurement commands (M, MC, C, and CC) or
 * from a verification command (V) issued to the SDI-12 sensor.
 */
typedef struct sdi12_master_measurement_queue_s {
    uint8_t data_ready_delay;  /*!< time in seconds before the measurement(s) is ready */
    uint8_t number_of_values;  /*!< number of measurement values the sensor is expected to return (1 to 9) */
} sdi12_master_measurement_queue_t;

/**
 * @brief SDI-12 master configuration structure.
 */
typedef struct sdi12_master_config_s {
    uint16_t   uart_port;               /*!< uart port number */
    gpio_num_t uart_tx_io_num;          /*!< uart tx pin */
    gpio_num_t uart_rx_io_num;          /*!< uart rx pin */
    gpio_num_t dc2364a_io_io_num;       /*!< logic supply ref voltage (dev board logic supply) */
    gpio_num_t dc2364a_de_io_num;       /*!< in rs485 mode a logic low disables rs-485 driver and high enables driver, in rs 232 a logic high enables fast mode (1Mbps) and low enables slow mode (250kbps) */
    gpio_num_t dc2364a_re_io_num;       /*!< logic high disables the rs485 receiver and low enables the receiver */
    gpio_num_t dc2364a_te_io_num;       /*!< logic low for 120-ohm termination or high for unterminated */
    gpio_num_t dc2364a_mode_io_num;     /*!< logic low for rs-232 and high for rs-485 */
} sdi12_master_config_t;

/**
 * @brief SDI-12 master context structure.
 */
struct sdi12_master_context_t {
    sdi12_master_config_t   dev_config;
};

/**
 * @brief SDI-12 master context structure definition.
 */
typedef struct sdi12_master_context_t sdi12_master_context_t;

/**
 * @brief SDI-12 master handle structure definition.
 */
typedef struct sdi12_master_context_t *sdi12_master_handle_t;

/**
 * @brief Converts allowable address characters ('0'-'9', 'a'-'z', 'A'-'Z') to a
 * decimal number between 0 and 61 (inclusive) to cover the 62 possible
 * addresses.
 * 
 * @param c Address character to convert.
 * @return uint8_t Converted decimal number.
 */
uint8_t sdi12_master_char_to_dec(const char c);

/**
 * @brief Maps a decimal number between 0 and 61 (inclusive) to allowable
 * address characters '0'-'9', 'a'-'z', 'A'-'Z'
 * 
 * @param b Decimal number to map.
 * @return char Mapped character.
 */
char sdi12_master_dec_to_char(const uint8_t b);

/**
 * @brief Initializes the SDI-12 master handle.
 * 
 * @param[in] sdi12_master_config SDI-12 master configuration.
 * @param[out] sdi12_master_handle SDI-12 master handle.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
esp_err_t sdi12_master_init(const sdi12_master_config_t *sdi12_master_config, sdi12_master_handle_t *sdi12_master_handle);

/**
 * @brief Sends a command to the SDI-12 sensor and retrieves the response.
 * 
 * @param[in] handle SDI-12 master handle.
 * @param[in] command SDI-12 command to send and process.
 * @param[out] response Response from SDI-12 sensor with <CR><LF> characters removed.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
esp_err_t sdi12_master_send_command(sdi12_master_handle_t handle, const char* command, const char **const  response);

/**
 * @brief Sends a measurement command to the SDI-12 sensor, waits for 
 * the sensor to process the requested command, parses the measurement 
 * values, and returns the measurements as an array of float values 
 * with size of array values.
 * 
 * @param[in] handle SDI-12 master handle.
 * @param[in] address SDI-12 sensor address.
 * @param[in] command SDI-12 measurement based command.
 * @param[out] values Array of values parsed from the SDI-12 data send response.
 * @param[out] size Number of values parsed from the SDI-12 data send response.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
esp_err_t sdi12_master_recorder(sdi12_master_handle_t handle, const char address, const sdi12_master_measurement_base_commands_t command, float **const values, uint8_t *const size);

/**
 * @brief Sends an acknowledge active command to the SDI-12 sensor.
 * 
 * @param[in] handle SDI-12 master handle.
 * @param[in] address SDI-12 sensor address.
 * @param[out] active SDI-12 sensor is active when true.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
esp_err_t sdi12_master_acknowledge_active(sdi12_master_handle_t handle, const char address, bool *const active);

/**
 * @brief Sends an identification request command to the SDI-12 sensor.  This function 
 * returns sensor information that is stored onboard the SDI-12 sensor as defined by 
 * the `sdi12_master_sensor_identification_t` structure.  See `sdi12_master_sensor_identification_t` 
 * structure for more details.
 * 
 * Sample information returned from the sensor identification structure:
 * 
 * `SDI-12 Version:        1.4`
 * 
 * `Vendor Identification: Campbell`
 * 
 * `Sensor Model:          CR6`
 * 
 * `Sensor Version:         01`
 * 
 * `Sensor Information:    4Std.14.01`
 * 
 * @param[in] handle SDI-12 master handle.
 * @param[in] address SDI-12 sensor address.
 * @param[out] identification SDI-12 sensor identification structure.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
esp_err_t sdi12_master_send_identification(sdi12_master_handle_t handle, const char address, sdi12_master_sensor_identification_t *const identification);

/**
 * @brief Sends a change address command to change the address of the SDI-12 sensor.
 * 
 * @param handle SDI-12 master handle.
 * @param address SDI-12 sensor address.
 * @param new_address New SDI-12 sensor address.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
esp_err_t sdi12_master_change_address(sdi12_master_handle_t handle, const char address, const char new_address);

/**
 * @brief Sends an address query command on the SDI-12 bus to determine the address of an SDI-12 sensor.
 * 
 * @param handle SDI-12 master handle.
 * @param[out] address SDI-12 sensor address.
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if handle is NULL.
 */
esp_err_t sdi12_master_address_query(sdi12_master_handle_t handle, char *const address);

/**
 * @brief Removes an SDI-12 master from uart and frees handle.
 *
 * @param[in] handle SDI-12 master handle.
 * @return esp_err_t ESP_OK on success.
 */
esp_err_t sdi12_master_delete(sdi12_master_handle_t handle);

/**
 * @brief Converts SDI-12 master firmware version numbers (major, minor, patch, build) into a string.
 * 
 * @return char* SDI-12 master firmware version as a string that is formatted as X.X.X (e.g. 4.0.0).
 */
const char* sdi12_master_get_fw_version(void);

/**
 * @brief Converts SDI-12 master firmware version numbers (major, minor, patch) into an integer value.
 * 
 * @return int32_t SDI-12 master firmware version number.
 */
int32_t sdi12_master_get_fw_version_number(void);


#ifdef __cplusplus
}
#endif

/**@}*/

#endif  // __SDI12_H__
