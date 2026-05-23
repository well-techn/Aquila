#ifndef FFT_AUX
#define FFT_AUX

fft_1_axis_result_t calculate_fft_peaks(float* magnitudes_sq, float sampling_freq, uint16_t window_len);

void update_filter_system_states_and_coeffs_3_axis(
                          float new_peaks_from_fft[3][3], // [Ось][3 пика]
                          float current_freqs[3][3],      // [Ось][3 текущих]
                          float notch_coeffs[3][3][5],    // [Ось][3 фильтра][5 коэф]
                          float accel_states[3][3][2], 
                          float gyro_states[3][3][2], 
                          float accel_states_2[3][3][2], 
                          float gyro_states_2[3][3][2], 
                          float sampling_freq_hz);

void apply_series_of_filters_to_imu_data_3_axis (
    const float* imu_input_data, 
    float* imu_output_data, 
    float notch_coeffs[3][3][5],   // Теперь здесь [Ось][Фильтр][Коэффициенты]
    float* lpf_coeffs, 
    float notch_states[3][3][2], 
    float lpf_states[3][2]);

#endif
