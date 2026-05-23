//функция принимающая на вход массив комплексных амплитуд
//производит поиск локальных максимумов, сортирует локальные максимумы по амплитуде,
//выбрасывает близко расположенные значения и возвращает три самые мощные частоты

// Вспомогательная структура для кандидатов

#include <string.h>
#include <math.h>
#include "esp_dsp.h"
#include <inttypes.h>
#include "wt_alldef.h"


typedef struct {
    float magnitude;
    uint16_t index;
} fft_peak_t;

fft_1_axis_result_t calculate_fft_peaks(float* magnitudes_sq, float sampling_freq, uint16_t window_len) {
//до 32 ло4альных максимумов
    fft_peak_t candidates[32];
    uint8_t cand_count = 0;
    fft_1_axis_result_t result = {0};

//поиск локальных максимумов (отсекаем первые 10 бинов)
    for (uint16_t i = 10; i < (window_len / 2) - 1 && cand_count < 32; i++) {
        if (magnitudes_sq[i] > magnitudes_sq[i-1] && magnitudes_sq[i] > magnitudes_sq[i+1]) {
            candidates[cand_count].magnitude = magnitudes_sq[i];
            candidates[cand_count].index = i;
            cand_count++;
        }
    }
//если ничего не нашли - возвращаем нули
    if (cand_count == 0) return result;

//если что-то нашли - сортируем найденное пузырьком по убыванию амплитуды
    for (uint8_t i = 0; i < cand_count - 1; i++) {
        for (uint8_t j = 0; j < cand_count - i - 1; j++) {
            if (candidates[j].magnitude < candidates[j+1].magnitude) {
                fft_peak_t temp = candidates[j];
                candidates[j] = candidates[j+1];
                candidates[j+1] = temp;
            }
        }
    }

//выбор ТОП-3 с учетом минимальной дистанции (NMS)
    float bin_width = sampling_freq / window_len;
//минимальная дистанция в бинах для того чтобы 2 частоты признать независимыми
    uint8_t min_distance_bins = (uint8_t)((float)FFT_PEAKS_MIN_DIST_HZ / bin_width);
    fft_peak_t top_3_peaks[3] = {0};
    uint8_t found_count = 0;
//проверяем найденные частоты не находятся ли они слишком близко друг к другу
    for (uint8_t i = 0; i < cand_count && found_count < 3; i++) {
        bool too_close = false;
        for (uint8_t j = 0; j < found_count; j++) {
            if (abs((int)candidates[i].index - (int)top_3_peaks[j].index) <= min_distance_bins) 
            {
                too_close = true;
                break;
            }
        }
        if (!too_close) {
            top_3_peaks[found_count] = candidates[i];
            found_count++;
        }
    }

//расчет финальных частот и фильтрация по порогам
    for (uint8_t i = 0; i < found_count; i++) {
        float freq = top_3_peaks[i].index * bin_width;
//восстановление магнитуды для проверки порога, х2 из-за окна Ханна, х2 за компенсацию что анализировали половину спектра
        float final_mag = (sqrtf(top_3_peaks[i].magnitude) / window_len) * 4.0f;
        //printf("%f %f ", freq,final_mag);
//частоты ниже 20 и со слишком малой амплитудой не учитываем (они остаются нулями)
        if (freq >= 20.0f && final_mag >= 0.5f) {
            result.fft_top_frequencies[i] = freq;
        }
    }
    //printf("\n");

    return result;
}


void update_filter_system_states_and_coeffs_3_axis(
                          float new_peaks_from_fft[3][3], // [Ось][3 пика]
                          float current_freqs[3][3],      // [Ось][3 текущих]
                          float notch_coeffs[3][3][5],    // [Ось][3 фильтра][5 коэф]
                          float accel_states[3][3][2], 
                          float gyro_states[3][3][2], 
                          float accel_states_2[3][3][2], 
                          float gyro_states_2[3][3][2], 
                          float sampling_freq_hz) 
{
    // Проходим по каждой оси (X, Y, Z)
    for (int a = 0; a < 3; a++) {
        bool peak_assigned[3] = {false, false, false};
        bool slot_updated[3] = {false, false, false};

        // 1. Трекинг и сглаживание для конкретной оси
        for (int s = 0; s < 3; s++) {//каждую из трех текущих пиковых частот
            int best_p = -1;
            float min_dist = 50.0f; 
            for (int p = 0; p < 3; p++) {//сравниваем с каждой из вновь прибывших
                if (peak_assigned[p]) continue;
                float dist = fabsf(new_peaks_from_fft[a][p] - current_freqs[a][s]);
//ищем расстояние между текущей частотой и тремя вновь прибывшими
//и выделяем ту, которая имеет минимальное расстояние, индекс этой частоты сохраняем в best_p
                if (dist < min_dist) { min_dist = dist; best_p = p; }
            }
//если нашли частоту которая расположена менее чем в 50Гц от текущей
            if (best_p != -1) {
//подползаем текущей к старой неспеша
                current_freqs[a][s] = (new_peaks_from_fft[a][best_p] * FREQ_SMOOTH_ALPHA) + 
                                      (current_freqs[a][s] * (1.0f - FREQ_SMOOTH_ALPHA));
                peak_assigned[best_p] = true;
                slot_updated[s] = true;
            }
        }

//обработка новых пиков для конкретной оси
        for (int p = 0; p < 3; p++) {
            if (!peak_assigned[p]) {
                for (int s = 0; s < 3; s++) {
                    if (!slot_updated[s]) {
                        float delta = fabsf(new_peaks_from_fft[a][p] - current_freqs[a][s]);
                        current_freqs[a][s] = new_peaks_from_fft[a][p];
                        slot_updated[s] = true;

                        if (delta > MIN_DIST_FOR_RESET) {
                            // Сброс состояний только для этой оси и этого слота
                            memset(accel_states[a][s], 0, sizeof(float) * 2);
                            memset(gyro_states[a][s], 0, sizeof(float) * 2);
                            memset(accel_states_2[a][s], 0, sizeof(float) * 2);
                            memset(gyro_states_2[a][s], 0, sizeof(float) * 2);
                        }
                        break;
                    }
                }
            }
        }

//пересчет коэффициентов для конкретной оси
//если частота ниже 20Гц фильтр ее не фильтруем (всепропускающие коэффициенты)
        for (int s = 0; s < 3; s++) {
            float norm_f = current_freqs[a][s] / sampling_freq_hz;
            if (norm_f < 0.02f) { 
                notch_coeffs[a][s][0] = 1.0f;
                notch_coeffs[a][s][1] = 0.0f;
                notch_coeffs[a][s][2] = 0.0f;
                notch_coeffs[a][s][3] = 0.0f;
                notch_coeffs[a][s][4] = 0.0f;
            } else dsps_biquad_gen_notch_f32(notch_coeffs[a][s], norm_f, -60.0f, (float)FFT_NOTCH_FILTERS_Q);    
            
        }
    }
}



void apply_series_of_filters_to_imu_data_3_axis (
    const float* imu_input_data, 
    float* imu_output_data, 
    float notch_coeffs[3][3][5],   // Теперь здесь [Ось][Фильтр][Коэффициенты]
    float* lpf_coeffs, 
    float notch_states[3][3][2], //[2]
    float lpf_states[3][2]) //[2]
{
    // Для каждой из трех осей (X, Y, Z)
    for (uint8_t axis = 0; axis < 3; axis++) {
        float val = imu_input_data[axis];

//последовательно пропускаем через 3 фильтра, уникальных для этой оси
        for (uint8_t i = 0; i < 3; i++) 
        {
//используем коэффициенты ИМЕННО для этой оси: notch_coeffs[axis][i]
            dsps_biquad_f32(&val, &val, 1, notch_coeffs[axis][i], notch_states[axis][i]);
            dsps_biquad_f32(&val, &val, 1, notch_coeffs[axis][i], notch_states[axis][i]);
        }
//пропукаем повторно (увеличиваем порядок фильтра)        
        // for (uint8_t i = 0; i < 3; i++) 
        // {
        //     dsps_biquad_f32(&val, &val, 1, notch_coeffs[axis][i], notch_states[axis][i]);
        // }

        // Применяем общий ФНЧ (коэффициенты lpf_coeffs одинаковые, состояния уникальные)
        dsps_biquad_f32(&val, &val, 1, lpf_coeffs, lpf_states[axis]);
        //dsps_biquad_f32(&val, &val, 1, lpf_coeffs, lpf_states[axis]);
        
        imu_output_data[axis] = val;    
    }
}