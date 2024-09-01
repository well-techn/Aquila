/*
при включении пульта первые выданные данные АЦП некорректны, поставить задержку перед выдачей первых данных
проверить производительность (параметр CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS  в шестеренке по rtos )
https://github.com/espressif/esp-idf/blob/56123c52aaa08f1b53350c7af30c91320b352ef4/examples/system/freertos/real_time_stats/main/real_time_stats_example_main.c#L30-L44
vTaskGetRunTimeStats

посмотреть esc фильтр на предмет окна


FreeRTOS to ensure that the space allocated is indeed adequate, and     //UBaseType_t uxTaskGetStackHighWaterMark( TaskHandle_t xTask );
that RAM is not being wasted unnecessarily. Section 12.3, Stack         ////log_i( "fDoTheHumidityThing high watermark %d",  uxTaskGetStackHighWaterMark( NULL ) );
Overflow, contains information on how to query the maximum stack
space that has actually been used by a task.

check defines for PMW9685


how to get height and course from GPS                       can get height from GGA
to adapt send pwm data to more convinient way
performance monitor                                          vTaskGetRunTimeStats

            static char cBuffer[ 512 ];
              ( cBuffer );
            в SDK config набрать stats для отображения связанных переменных

to get rid of all global vars
extract course and other usefull data from GPS

check assignation of interrupts to cores - interrupts have to be created from tasks pinned to core
check vTaskNotifyGive instead of semaphores https://microsin.net/programming/arm/freertos-counting-semaphores-and-task-notifications.html

*/