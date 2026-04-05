/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "driver/rmt_encoder.h"
#include "driver/rmt_tx.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Throttle representation in DShot protocol
 */
typedef struct {
    uint16_t throttle;  /*!< Throttle value */
    bool telemetry_req; /*!< Telemetry request */
} dshot_esc_throttle_t;

/**
 * @brief Type of Dshot ESC encoder configuration
 */
typedef struct {
    uint32_t resolution;    /*!< Encoder resolution, in Hz */
    uint32_t baud_rate;     /*!< Dshot protocol runs at several different baud rates, e.g. DSHOT300 = 300k baud rate */
    uint32_t post_delay_us; /*!< Delay time after one Dshot frame, in microseconds */
} dshot_esc_encoder_config_t;

/**
 * @brief Create RMT encoder for encoding Dshot ESC frame into RMT symbols
 *
 * @param[in] config Encoder configuration
 * @param[out] ret_encoder Returned encoder handle
 * @return
 *      - ESP_ERR_INVALID_ARG for any invalid arguments
 *      - ESP_ERR_NO_MEM out of memory when creating Dshot ESC encoder
 *      - ESP_OK if creating encoder successfully
 */
esp_err_t rmt_new_dshot_esc_encoder(const dshot_esc_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

typedef struct {
    rmt_encoder_t base;
    rmt_encoder_handle_t copy_encoder; // Добавьте эту строку
    rmt_symbol_word_t sym_one;   // Заранее рассчитанный символ "1"
    rmt_symbol_word_t sym_zero;  // Заранее рассчитанный символ "0"
    rmt_symbol_word_t sym_delay; // Символ паузы (post_delay)
} dshot_fast_encoder_t;

// esp_err_t rmt_new_fast_dshot_esc_encoder(const dshot_esc_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

// size_t rmt_encode_fast_dshot_esc(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
//                                           const void *primary_data, size_t data_size, 
//                                           rmt_encode_state_t *ret_state);
#ifdef __cplusplus
}
#endif
