#include "Fusion.h"
#include "vqf.h"
#include "wt_alldef.h"
#include "stdio.h"


//в зависимости от выбранного алгоритма объявляем и инициализируем структуры того или иного фильтра
// обязательно static, чтобы сохранялось между вызовами
#ifdef USING_MADGWICK
  static FusionBias_t bias;  
  static FusionAhrs_t ahrs;  
  //устанавливаем настройки
  const FusionAhrsSettings_t settings = {
      .convention = FusionConventionNed,                            //Earth axes convention (NWU, ENU, or NED).
      .gain = 0.2f,                                                 //Determines the influence of the gyroscope relative to other sensors. A value of zero will disable startup and the acceleration and magnetic rejection features. A value of 0.5 is appropriate for most applications.
      .gyroscopeRange = (float)(IMU_GYRO_FULL_SCALE_DPS),           //Gyroscope range (in degrees per second). Angular rate recovery will activate if the gyroscope measurement exceeds 98% of this value. A value of zero will disable this feature. The value should be set to the range specified in the gyroscope datasheet.
      .accelerationRejection = 10.0f,                               //Threshold (in degrees) used by the acceleration rejection feature. A value of zero will disable this feature. A value of 10 degrees is appropriate for most applications.
      .magneticRejection = 10.0f,                                   //Threshold (in degrees) used by the magnetic rejection feature. A value of zero will disable the feature. A value of 10 degrees is appropriate for most applications.
      .recoveryTriggerPeriod = 5 * 1000,                            //5 * sample rate  Acceleration and magnetic recovery trigger period (in samples). A value of zero will disable the acceleration and magnetic rejection features. A period of 5 seconds is appropriate for most applications.
  };
#endif

#ifdef USING_VQF
    static const vqf_real_t gyrTs = 1.0f / (float)IMU_SAMPLING_FREQ_HZ;   //sampling_time гироскопа
    static const vqf_real_t accTs = 1.0f / (float)IMU_SAMPLING_FREQ_HZ;   //sampling_time акселерометра
    static const vqf_real_t magTs = 0.005f;                               //sampling_time магнетометра 
#endif


void ahrs_initialize(void)
{
#ifdef USING_MADGWICK
    FusionBiasInitialise(&bias, IMU_SAMPLING_FREQ_HZ); 
    FusionAhrsInitialise(&ahrs);
    FusionAhrsSetSettings(&ahrs, &settings);
#endif

#ifdef USING_VQF
    initVqf(gyrTs, accTs, magTs);
#endif
}

//в эту функцию данные подаются в градусах в секунду
void ahrs_update(float* accel_data, float* gyro_data, float* magn_data, float delta_time, float* q_result)
{
#ifdef USING_MADGWICK
    FusionVector_t gyroscope = {.array = {gyro_data[0], gyro_data[1], gyro_data[2]}};
    FusionVector_t accelerometer = {.array = {accel_data[0], accel_data[1], accel_data[2]}};
    float fragmented_time = 0;
//если есть данные от магнетометра
    if (magn_data != NULL) 
    {
        FusionVector_t magnetometer = {.array = {magn_data[0], magn_data[1],magn_data[2]}};
        FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, delta_time);   
    }
//если нет данных от магнетометра
    else
    {
        fragmented_time = delta_time / (float)NUMBER_OF_MADGWICK_CYCLES_WO_MAGNETOMETER;
        for (uint8_t i = 0; i < NUMBER_OF_MADGWICK_CYCLES_WO_MAGNETOMETER; i++)
        {
            FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, fragmented_time); 
        }
    }
//копируем результат расчета     
    q_result[0] = ahrs.quaternion.element.w;
    q_result[1] = ahrs.quaternion.element.x;
    q_result[2] = ahrs.quaternion.element.y;
    q_result[3] = ahrs.quaternion.element.z;

#elif defined USING_VQF
//переводим градусы в секунду в радианы в секунду
    float gyro_data_rads[3] = {
        gyro_data[0] * (float)M_PI / 180.0f, 
        gyro_data[1] * (float)M_PI / 180.0f, 
        gyro_data[2] * (float)M_PI / 180.0f
    };
//инвертируем показания акселерометра, так как Fusion и VQF понимают вектро гравитации по-разному
    float accel_data_inv[3] = {
        -accel_data[0], 
        -accel_data[1], 
        -accel_data[2]
    };
//запускаем алгоритмы вычисления кватерниона ориентации      
    updateGyr((vqf_real_t*)gyro_data_rads);
    updateAcc((vqf_real_t*)accel_data_inv);
    if (magn_data == NULL) getQuat6D((vqf_real_t*)q_result);
    else 
    {
    updateMag(magn_data);
    getQuat9D((vqf_real_t*)q_result);
    }
#endif
}

