#include "wt_alldef.h"
#include "dshot_esc_encoder.h"
extern  dshot_esc_throttle_t dshot_signal[4];
extern rmt_encoder_handle_t esc_dshot_encoder[4];
//extern rmt_encoder_handle_t my_fast_dshot_encoder[4];
extern rmt_channel_handle_t esc_dshot_tx_channel[4];
extern rmt_transmit_config_t esc_dshot_tx_config;

void dshot_esc_control_init(uint8_t engine_0_pin, uint8_t engine_1_pin, uint8_t engine_2_pin, uint8_t engine_3_pin, int loop_count) {
//Проверяем первый ли это запуск инициализации (созданы ли уже каналы). Если нет — инициализируем железо
    if (esc_dshot_tx_channel[0] == NULL) {
        uint8_t pins[4] = {engine_0_pin, engine_1_pin, engine_2_pin, engine_3_pin};
        for (int i = 0; i < 4; i++) {
//создаем 4 канала RMT, привязываем номера ног            
            rmt_tx_channel_config_t tx_chan_config = {
                .clk_src = RMT_CLK_SRC_DEFAULT,
                .gpio_num = (gpio_num_t)pins[i],
                .mem_block_symbols = 48,
                .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
                .trans_queue_depth = 10,
            };
            ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &esc_dshot_tx_channel[i]));

//создаем 4 энкодера (все одинаковые)            
            dshot_esc_encoder_config_t encoder_config = {
                .resolution = DSHOT_ESC_RESOLUTION_HZ,
                .baud_rate = DSHOT_BITRATE,
//это дополнительный кусок прибавляемый к пакету. В бескончеом режиме (loop_count = -1) эта пауза между отправками
//в одиночном режиме это "хвост" прибавляемый к каждому пакету. Пока он не пройдет транзакция считается не завершенной
                .post_delay_us = 100,  
            };
            ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, &esc_dshot_encoder[i]));
            //rmt_new_fast_dshot_esc_encoder(&encoder_config, &my_fast_dshot_encoder[i]);

//включаем канал            
            ESP_ERROR_CHECK(rmt_enable(esc_dshot_tx_channel[i]));
        }
    }

//Если каналы уже есть, просто меняем режим работы (loop_count)
// Обязательно disable/enable, чтобы сбросить текущую передачу, если она была циклической
    for (int i = 0; i < 4; i++) {
        rmt_disable(esc_dshot_tx_channel[i]);
        esc_dshot_tx_config.loop_count = loop_count; 
        rmt_enable(esc_dshot_tx_channel[i]);

//Если включили бесконечный режим стартуем бесконечную нулевую отправку
        if (loop_count == -1) {
            dshot_signal[i].throttle = 0;
            rmt_transmit(esc_dshot_tx_channel[i], esc_dshot_encoder[i], &dshot_signal[i], sizeof(dshot_signal[i]), &esc_dshot_tx_config);
            //rmt_transmit(esc_dshot_tx_channel[i], my_fast_dshot_encoder[i], &dshot_signal[i], sizeof(dshot_signal[i]), &esc_dshot_tx_config);        
        }
    }
}


void dshot_esc_control_update(float* engines_signal)
{
    for (uint8_t i = 0; i<4; i++)
    { 
        if (engines_signal[i] < 0) dshot_signal[i].throttle = 0;
        else      
        {
        dshot_signal[i].throttle = (uint16_t)(engines_signal[i] * 1999.0f + 48.0f);
        if (dshot_signal[i].throttle > 2047) dshot_signal[i].throttle = 2047;
        }
        //rmt_transmit(esc_dshot_tx_channel[i], my_fast_dshot_encoder[i], &dshot_signal[i], sizeof(dshot_signal[i]), &esc_dshot_tx_config);
        ESP_ERROR_CHECK(rmt_transmit(esc_dshot_tx_channel[i], esc_dshot_encoder[i], &dshot_signal[i], sizeof(dshot_signal[i]), &esc_dshot_tx_config));
    }
}
