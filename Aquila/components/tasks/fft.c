//стандартные библиотеки
#include "freertos/FreeRTOS.h"
#include "dsps_fft2r.h"
#include "dsps_view.h"
#include "dsps_wind_hann.h"
#include "math.h"
#include "freertos/ringbuf.h"
#include <string.h>
#include "esp_log.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "filters.h"

//окно медианного фильтра доминирующей частоты FFT (нечетное!)
#define FFT_MEDIAN_FILTER_LENGTH (5)

extern  char *TAG_FFT;
extern RingbufHandle_t fft_rb_handle;
extern QueueHandle_t fft_to_main_queue;
//переменные ниже __attribute__((aligned(16))) для более эффективной работы библиотеки dsp 
static float __attribute__((aligned(16))) overlap_buffer[FFT_WINDOW_LENGTH]; // Локальное хранилище для выборки данных от датчика, по которой делаем FFT 
float __attribute__((aligned(16))) fft_input[FFT_WINDOW_LENGTH * 2]; // Подается на вход FFT, Формат: [re, im, re, im...]. im = 0
float __attribute__((aligned(16))) hann_window[FFT_WINDOW_LENGTH];  //хранилище коэффициентов для сглаживания спектра
float* new_data;
size_t item_size;
uint32_t frame_counter = 0;
float fft_median_filter_pool[FFT_MEDIAN_FILTER_LENGTH];
//индекс начиная с которого ищем доминирующую частоту вибраций. Вибрации ниже 20Гц считаем рабочими и игнорируем
//индекс = минимальная частота * ширина окна / частота сэмплирования = 20 * 512 / 1000 = 10.24
const uint16_t start_bin = (uint16_t)(20.0f * FFT_WINDOW_LENGTH / IMU_SAMPLING_FREQ_HZ);

//задача вычисления доминирующей частоты вибраций для последующего использования в фильтрации
//FFT производится по выборте длиной FFT_WINDOW_LENGTH (512), которая пополняется кусками по FFT_HOP_SIZE (128)
//то есть каждые 128 сэмплов main_flying_cycle шлет через кольцевой буфер 128 новых отсчетов с IMU
//эти 128 отсчетов добавляются в состав окна и по обновленному окну идет FFT
//вычисленная пиковая частота отправляется в main_flying_cycle через очередь
void fft(void* pvParameters)
{
  ESP_LOGI(TAG_FFT,"инициализируем FFT");
  ESP_ERROR_CHECK (dsps_fft2r_init_fc32(NULL, FFT_WINDOW_LENGTH)); 
  
  ESP_LOGI(TAG_FFT,"готовим однократно оконную функцию (Hann)");
  dsps_wind_hann_f32(hann_window, FFT_WINDOW_LENGTH);
  
  ESP_LOGI(TAG_FFT,"очищаем буфер от мусора");
  memset(overlap_buffer, 0, sizeof(overlap_buffer));

  while(1) 
  {
//ждем команды от main_flying_cycle
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//копируем содержимое кольцевого буфера
    new_data = (float*)xRingbufferReceive(fft_rb_handle, &item_size, 0);
    if (new_data != NULL) 
    {
       if (item_size != FFT_HOP_SIZE * sizeof(float)) ESP_LOGW("FFT", "Некорректный размер пакета: %d", item_size); 
       else         
       {
          frame_counter++;
//Сдвигаем старые данные в локальном буфере влево
          memmove(overlap_buffer, &overlap_buffer[FFT_HOP_SIZE], (FFT_WINDOW_LENGTH - FFT_HOP_SIZE) * sizeof(float));           
//Копируем в конец свежеполученные 128 сэмплов 
          memcpy(&overlap_buffer[FFT_WINDOW_LENGTH - FFT_HOP_SIZE], new_data, FFT_HOP_SIZE * sizeof(float));
        }
//Готовим данные для FFT - копируем в fft_input выборку данных,умноженную на окно Ханна, добавляя после каждого im = 0
        for (uint16_t i = 0; i < FFT_WINDOW_LENGTH; i++) 
        {
          fft_input[2*i] = overlap_buffer[i] * hann_window[i];
          fft_input[2*i + 1] = 0;
        }
//очищаем ringbuffer
        vRingbufferReturnItem(fft_rb_handle, (void*)new_data);
//главное действие - считаем FFT
        dsps_fft2r_fc32(fft_input, FFT_WINDOW_LENGTH);
//упорядочиваем результат (Bit-reversal)
        dsps_bit_rev_fc32(fft_input, FFT_WINDOW_LENGTH);
//далее идет поиск пиков на спектрограмме
        float max_magnitude = 0;
        uint16_t peak_index = 0;
//переводим комплексные числа в значения амплитуд.
//начинаем считать с 10го отсчета, чтобы игнорировать постоянную составляющую и медленные частоты (до 20Гц)
        for (uint16_t i = 0; i < FFT_WINDOW_LENGTH / 2; i++) 
        {
          float re = fft_input[2*i];
          float im = fft_input[2*i + 1];         
// Считаем модуль вектора и нормализуем
// Множитель 2.0 для компенсации энергии (кроме отсчета 0)
// Множитель 2.0 для компенсации окна Ханна
          float magnitude = (sqrtf(re * re + im * im) / FFT_WINDOW_LENGTH) * 2.0f * 2.0f;
// magnitude[i] соответствует частоте (i * Fsampling / FFT_WINDOW_LENGTH)
#ifdef PRINT_FFT_RESULTS          
          if ((frame_counter % 2) == 0) printf("%.3f ", magnitude);
#endif
          if (magnitude > max_magnitude) 
          {
              max_magnitude = magnitude;
              peak_index = i;
          }
        }
#ifdef PRINT_FFT_RESULTS  
          if ((frame_counter % 2) == 0) printf("\n");
#endif
//Вычисляем частоту пика с учетом реальной частоты сэмплирования
          float peak_frequency = (float)peak_index * (float)IMU_SAMPLING_FREQ_HZ / (float)FFT_WINDOW_LENGTH;
//применяем медианный фильтр для пиковой частоты для фильтрации случайных шумов
          peak_frequency = median_filter(fft_median_filter_pool, peak_frequency, FFT_MEDIAN_FILTER_LENGTH);
//если вычисленная максимальная частота менее 20Гц ее воспринимаем как постоянный сигнал и исключаем из наблюдения
//если амплитуда пика слишком мала ее игнорируем (надо смотреть экспериментрально)
          if ((peak_frequency < 20.0f) || (max_magnitude < 0.01f)) peak_frequency = 0;
          //printf("Пик: %.4f Гц, Амплитуда: %.4f\n", peak_frequency, max_magnitude);
          xQueueSend(fft_to_main_queue, &peak_frequency, 0); 
    }        
  }
}