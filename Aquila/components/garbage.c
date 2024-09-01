/*


enum {
            IMU_calibration,
            IMU_coeff_calculation,
            MAG_calibration,
            MAG_coeff_calculation,
            flying
};
int state;

state = IMU_calibration;

minimize all  ctrl+K, ctrl+0
Shift+Atl+F auto format
изменения

статические задачи
введены коды ошибок и задача их отображения


#include <inttypes.h>
uint64_t t;
printf("%" PRIu64 "\n", t);

  fflush(stdout);
  esp_restart();


printing char value in hex  printf ("%x\n",var);

printf("received %d bytes from GPS\n" , received_length);

checking value from i2c register
printf("%04x \n", i2c_read_byte_from_address_x16(0x52, 0x010F));

vTaskDelay(1000/portTICK_PERIOD_MS);
ets_delay_us(10);

start_time = get_time();
printf ("%lld\n",get_time() - start_time);

PMW3901_get_raw_image(PMW_raw_image);
  for (i=0;i<1225;i++) 
  {
    if ((i%35 == 0) && (i > 0)) printf ("\n");
    printf ("%d ",PMW_raw_image[i]); 
  }

  /* 
  ESP_LOGI(TAG_INIT,"Checking communication with PMW3901.....");
  PMW3901_communication_check ();
  ESP_LOGI(TAG_INIT,"Configuring PMW3901");
  PMW3901_config ();

  PMW3901_get_raw_image(PMW_raw_image);
  for (i=0;i<1225;i++) 
  {
    if ((i%35 == 0) && (i > 0)) printf ("\n");
    printf ("%d ",PMW_raw_image[i]); 
  }
  do
  {
    vTaskDelay(5000/portTICK_PERIOD_MS);
    read_data_from_PMW();
    //PMW_X_coordinate += (int16_t)(0.021935 * ((float)VL53L1x_distance / 10.0) * (float)PMW_deltaX);
    PMW_X_coordinate += (int16_t)(0.021935 * 68 * (float)PMW_deltaX);
    printf("D=%d, dX=%d\n",VL53L1x_distance,PMW_deltaX);
  } while (1);



%с	Символ типа char
%d	Десятичное число целого типа со знаком  (%ld) 
%i	Десятичное число целого типа со знаком
%е	Научная нотация (е нижнего регистра)
%Е	Научная нотация (Е верхнего регистра)
%f	Десятичное число с плавающей точкой  (sprintf(aa, "%0.7f", a);)
%g	Использует код %е или %f — тот из них, который короче (при использовании %g используется е нижнего регистра)
%G	Использует код %Е или %f — тот из них, который короче (при использовании %G используется Е верхнего регистра)
%о	Восьмеричное целое число без знака
%s	Строка символов
%u	Десятичное число целого типа без знака (%lu)
%х	Шестнадцатиричное целое число без знака (буквы нижнего регистра) "%02x for leading zeros"
%Х	Шестнадцатиричное целое число без знака (буквы верхнего регистра)
%р	Выводит на экран значение указателя
%n	Ассоциированный аргумент — это указатель на переменную целого типа, в которую помещено количество символов, записанных на данный момент
%%	Выводит символ %


#include "esp_log.h"
static const char *TAG = "MAIN";
void app_main(void) {
ESP_LOGE(TAG, "This is error log");
ESP_LOGW(TAG, "This is warning log");
ESP_LOGI(TAG, "This is info log");
ESP_LOGD(TAG, "This is debug log");
ESP_LOGV(TAG, "This is verbos log");
}

esp_log_level_set()

//esp_fill_random(logs_buffer, LOGS_BYTES_PER_STRING);

random_number = esp_random();
p_to_uint8 = &random_number;

  pointer_to_mpu_temp = &mpu_temp;

  printf ("MPU_temp address is %p\n",  &mpu_temp);
  printf("%p\n", pointer_to_mpu_temp);
  pointer_to_mpu_temp++;
  printf("this is incresed pointer %p\n", pointer_to_mpu_temp);

  printf("%x\n", (uint8_t) *pointer_to_mpu_temp);
  printf("%x\n", (uint8_t) *(pointer_to_mpu_temp-1));
  printf("%x\n", (uint8_t) *(pointer_to_mpu_temp-2));
  printf("%x\n", (uint8_t) *(pointer_to_mpu_temp-3));
  //printf("%x\n", *(p+3));



p_to_uint8 = &test_uint8;
p_to_uint16 = &test_uint16;
p_to_uint32 = &test_uint32;
p_to_float = &test_float;
p_to_double = &test_double;

printf("variable test_uint8 = %d located at address %p\n", test_uint8, p_to_uint8);
printf("variable test_uint16 = %d located at address %p\n", test_uint16, p_to_uint16);
printf("variable test_uint32 = %lu located at address %p\n", test_uint32, p_to_uint32);
printf("variable test_float = %0.12f located at address %p\n", test_float, p_to_float);
printf("variable test_double= %f located at address %p\n", test_double, p_to_double);
printf ("*******************************\n");


printf("variable test_uint8 = %d located at address %p\n", test_uint8, p_to_uint8);
printf("variable test_uint16 = %d located at address %p\n", test_uint16, p_to_uint16);
printf("variable test_uint32 = %lu located at address %p\n", test_uint32, p_to_uint32);
printf("variable test_float = %f located at address %p\n", test_float, p_to_float);
printf("variable test_double= %f located at address %p\n", test_double, p_to_double);

test_uint8 = *p_to_uint8+1;
test_uint16 = *p_to_uint16+1;
test_uint32 =*p_to_uint32+1;
test_float = *p_to_float+1;
test_double = *p_to_double+1;

printf("variable test_uint8 = %d located at address %p\n", test_uint8, p_to_uint8);
printf("variable test_uint16 = %d located at address %p\n", test_uint16, p_to_uint16);
printf("variable test_uint32 = %lu located at address %p\n", test_uint32, p_to_uint32);
printf("variable test_float = %f located at address %p\n", test_float, p_to_float);
printf("variable test_double= %f located at address %p\n", test_double, p_to_double);
printf ("*******************************\n");

p_to_uint8 = &test_float;
printf("variable test_float= %f located at address %p\n", test_float, p_to_uint8);
float_by_bytes[0] = *p_to_uint8;
float_by_bytes[1] = *(p_to_uint8+1);
float_by_bytes[2] = *(p_to_uint8+2);
float_by_bytes[3] = *(p_to_uint8+3);
printf("byte_1 is %x byte_2 is %x byte_3 is %x byte_4 is %x\n", float_by_bytes[0],float_by_bytes[1],float_by_bytes[2],float_by_bytes[3]);

p_to_float = float_by_bytes;
reconstructed_float = *p_to_float;
printf("variable reconstructed_float = %0.12f\n", reconstructed_float);



 //sprintf(logs_buffer[i_logs_buf],"%10lu,%6d,%6d,%6d,%6d,%6d,%6d,%0.4f,%0.4f,%0.4f,%0.4f,%0.1f,%0.1f,%0.1f,%lu,%lu,%lu,%lu",timestamp,accel_raw[0],accel_raw[1],accel_raw[2],gyro_raw[0],gyro_raw[1],gyro_raw[2],q0,q1,q2,q3,pitch,roll,yaw,engine[0],engine[1],engine[2],engine[3]);
 
  
 static void  gpio_interrupt_handler(void *args)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR( Semaphore_9255, &xHigherPriorityTaskWoken );
  }

void configure_pin_for_interrupt (void)
{
  ESP_ERROR_CHECK(gpio_reset_pin(MPU9255_interrupt_pin));
  gpio_config_t gpio = {
    .pin_bit_mask = 1ULL<<MPU9255_interrupt_pin,
    .mode = GPIO_MODE_INPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_ENABLE,
    .intr_type = GPIO_INTR_POSEDGE
  }; 
  ESP_ERROR_CHECK(gpio_config(&gpio));
  ESP_ERROR_CHECK(gpio_install_isr_service(0));
  ESP_ERROR_CHECK(gpio_isr_handler_add(MPU9255_interrupt_pin, gpio_interrupt_handler, (void*) MPU9255_interrupt_pin));
}     

*/












