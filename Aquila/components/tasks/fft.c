//стандартные библиотеки
#include "freertos/FreeRTOS.h"
#include "dsps_fft2r.h"
#include "dsps_view.h"
#include "dsps_wind_hann.h"
#include "math.h"
#include "freertos/ringbuf.h"
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"

//собственные библиотеки
#include "wt_alldef.h"
#include "fft_aux.h"

extern  char *TAG_FFT;
extern RingbufHandle_t fft_rb_handle;
extern QueueHandle_t fft_to_main_queue;
//переменные ниже __attribute__((aligned(16))) для более эффективной работы библиотеки dsp 
static imu_fft_data_t __attribute__((aligned(16))) overlap_buffer[FFT_WINDOW_LENGTH]; // Локальное хранилище для выборки данных от датчика, по которой делаем FFT 
float __attribute__((aligned(16))) fft_input[FFT_WINDOW_LENGTH * 2]; // Подается на вход FFT, Формат: [re, im, re, im...]
float __attribute__((aligned(16))) hann_window[FFT_WINDOW_LENGTH];   //хранилище коэффициентов для сглаживания спектра
imu_fft_data_t* new_data;
size_t item_size;
uint32_t frame_counter = 0;
//индекс начиная с которого ищем доминирующую частоту вибраций. Вибрации ниже 20Гц считаем рабочими и игнорируем
//индекс = минимальная частота * ширина окна / частота сэмплирования = 20 * 512 / 1000 = 10.24
const uint16_t start_bin = (uint16_t)(20.0f * FFT_WINDOW_LENGTH / IMU_SAMPLING_FREQ_HZ);

float __attribute__((aligned(16))) magnitudes_squared_x[FFT_WINDOW_LENGTH / 2]; //для хранения рассчитанных амплитуд по X
float __attribute__((aligned(16))) magnitudes_squared_z[FFT_WINDOW_LENGTH / 2]; //для хранения рассчитанных амплитуд по Z

typedef struct {
    float magnitude;
    uint16_t index;
} fft_peak_t;

fft_peak_t candidates_x[32]; // Хранилище для найденных локальных максимумов по X
fft_peak_t candidates_z[32]; // Хранилище для найденных локальных максимумов по z

//задача вычисления доминирующей частоты вибраций для последующего использования в фильтрации
//FFT производится по выборте длиной FFT_WINDOW_LENGTH (512), которая пополняется кусками по FFT_HOP_SIZE (128)
//то есть каждые 128 сэмплов main_flying_cycle шлет через кольцевой буфер 128 новых отсчетов с IMU
//эти 128 отсчетов добавляются в состав окна и по обновленному окну идет FFT
//по результатам FFT находятся 3 максимальный частоты вибрации, а именно
//сначала просто находим 32 пика по критерию что он выше соседей справа и слева
//затем прогоняем этот массив и выбрасываем мелкие локальные максимумы, в итоге на выход отдаем топ-3 пика
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
    new_data = (imu_fft_data_t*)xRingbufferReceive(fft_rb_handle, &item_size, 0);
    if (new_data != NULL) 
    {
       if (item_size != FFT_HOP_SIZE * sizeof(imu_fft_data_t)) ESP_LOGW("FFT", "Некорректный размер пакета: %d", item_size); 
       else         
       {
          frame_counter++;
//Сдвигаем старые данные в локальном буфере влево
          memmove(overlap_buffer, &overlap_buffer[FFT_HOP_SIZE], (FFT_WINDOW_LENGTH - FFT_HOP_SIZE) * sizeof(imu_fft_data_t));           
//Копируем в конец свежеполученные 128 сэмплов 
          memcpy(&overlap_buffer[FFT_WINDOW_LENGTH - FFT_HOP_SIZE], new_data, FFT_HOP_SIZE * sizeof(imu_fft_data_t));
        }
//Готовим данные для FFT - копируем в fft_input выборку данных,умноженную на окно Ханна, добавляя после каждого im = 0
//так как минимые составляющие входного сигнала нулевые а функция их ожидает на вход - впихиваем туда колебания по X и Z
        for (uint16_t i = 0; i < FFT_WINDOW_LENGTH; i++) 
        {
          fft_input[2*i] = overlap_buffer[i].x * hann_window[i];
          fft_input[2*i + 1] = overlap_buffer[i].z * hann_window[i];
        }
//очищаем ringbuffer
        vRingbufferReturnItem(fft_rb_handle, (void*)new_data);
//главное действие - считаем FFT
        dsps_fft2r_fc32(fft_input, FFT_WINDOW_LENGTH);
//упорядочиваем результат (Bit-reversal)
        dsps_bit_rev_fc32(fft_input, FFT_WINDOW_LENGTH);

//далее идет пересчет в амплитуды и поиск пиков на спектрограмме
//переводим комплексные числа в значения амплитуд.
        for (uint16_t i = 0; i < FFT_WINDOW_LENGTH / 2; i++) 
        {
 //индекс зеркального элемента (для i=0 зеркалом будет 0, для остальных N-i)
        uint16_t j = (i == 0) ? 0 : (FFT_WINDOW_LENGTH - i);

//извлекаем компоненты из комплексного массива
        float ReA = fft_input[2*i];     // Real часть i-го бина
        float ImA = fft_input[2*i + 1]; // Imag часть i-го бина
        float ReB = fft_input[2*j];     // Real часть зеркального бина
        float ImB = fft_input[2*j + 1]; // Imag часть зеркального бина

//разделяем компоненты оси X (была в Re части fft_input)
        float x_re = (ReA + ReB) * 0.5f;
        float x_im = (ImA - ImB) * 0.5f;
        magnitudes_squared_x[i] = (x_re * x_re + x_im * x_im);

//разделяем компоненты оси Z (была в Im части fft_input)
        float z_re = (ImA + ImB) * 0.5f;
        float z_im = (ReB - ReA) * 0.5f;
        magnitudes_squared_z[i] = (z_re * z_re + z_im * z_im);
        }
//запускаем функцию, которая из линейки амплитуд выбирает 3 самые мощные, исключая при этом близлежащие
//в результате в структуре peaks_x лежат 3 доминирующие частоты
        fft_1_axis_result_t peaks_x = calculate_fft_peaks(magnitudes_squared_x, IMU_SAMPLING_FREQ_HZ, FFT_WINDOW_LENGTH);
        fft_1_axis_result_t peaks_z = calculate_fft_peaks(magnitudes_squared_z, IMU_SAMPLING_FREQ_HZ, FFT_WINDOW_LENGTH);
        
//копируем данные в общую структуру (в X и Y копируем результаты X, Z идет как Z)
        fft_3_axis_result_t all_peaks;
        memcpy(all_peaks.fft_top_frequencies[0], peaks_x.fft_top_frequencies, sizeof(float) * 3);
        memcpy(all_peaks.fft_top_frequencies[1], peaks_x.fft_top_frequencies, sizeof(float) * 3);
        memcpy(all_peaks.fft_top_frequencies[2], peaks_z.fft_top_frequencies, sizeof(float) * 3);

        //printf("%0.3f, %0.3f, %0.3f\n",peaks_x.fft_top_frequencies[0], peaks_x.fft_top_frequencies[1], peaks_x.fft_top_frequencies[2]);
//отправляем результаты расчета в main_flying_cycle          
        xQueueSend(fft_to_main_queue, &all_peaks, 0); 
//опционально выводим на печать квадраты амплитуд, вычисление корня из них и умножение на 4 на принимающей стороне
#ifdef PRINT_FFT_RESULTS          
        if ((frame_counter % 2) == 0) 
        {         
        for (uint16_t i = 0; i < (FFT_WINDOW_LENGTH / 2); i++) printf("%.3f ", magnitudes_squared_x[i]);
        printf("\n");
        }
#endif
    }        
  }
}