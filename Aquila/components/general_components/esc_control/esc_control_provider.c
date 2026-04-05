#include "wt_alldef.h"
#include "stdio.h"
#include "pwm.h"
#include "dshot.h"
#include "driver/ledc.h"
#include "dshot_esc_encoder.h"

//в зависимости от выбранного алгоритма объявляем и инициализируем структуры того или иного механизма выдачи сигналов на ESC
// обязательно static, чтобы сохранялось между вызовами
#ifdef USING_PWM_ESC_CONTROL
    ledc_timer_config_t engine_pwm_timer;
    ledc_channel_config_t engine_pwm_channel[4];
#endif

#ifdef USING_DSHOT_ESC_CONTROL
    volatile rmt_channel_handle_t esc_dshot_tx_channel[4] = {NULL}; // хэндлы 4 каналов для Dshot RMT, значение присваиваются при инициализации
    volatile rmt_encoder_handle_t esc_dshot_encoder[4] = {NULL};    //хэндлы энкодеров для 4-х каналов, передаются в RMT_transmit 
    //volatile rmt_encoder_handle_t my_fast_dshot_encoder[4]  = {NULL};
    volatile rmt_sync_manager_handle_t esc_dshot_synchro = {NULL};
    volatile rmt_transmit_config_t esc_dshot_tx_config = {          //общий для всех 4-х каналов
        .loop_count = -1,           //при первичной инициализации запускаем в бесконечном цикле слать нулевой пакет чтобы не пищал 
        .flags.eot_level = 0,
        .flags.queue_nonblocking = 1,

    };
    volatile dshot_esc_throttle_t dshot_signal[4] = {0}; 
#endif

//параметр нужен только для dshot: -1 активирует бесконечную отправку пакетов, 0 - единичную
//для PWM игнорируется
void esc_control_initialize(int8_t loop_count)
{
#ifdef USING_PWM_ESC_CONTROL
    pwm_esc_control_init(ENGINE_OUTPUT_0_PIN, ENGINE_OUTPUT_1_PIN, ENGINE_OUTPUT_2_PIN, ENGINE_OUTPUT_3_PIN);
#endif

#ifdef USING_DSHOT_ESC_CONTROL
    dshot_esc_control_init(ENGINE_OUTPUT_0_PIN, ENGINE_OUTPUT_1_PIN, ENGINE_OUTPUT_2_PIN, ENGINE_OUTPUT_3_PIN, loop_count);
#endif
}


void esc_control_update(float* engines_signal)
{
#ifdef USING_PWM_ESC_CONTROL
    pwm_esc_control_update(engines_signal);
#endif

#ifdef USING_DSHOT_ESC_CONTROL
    dshot_esc_control_update(engines_signal);
#endif
}