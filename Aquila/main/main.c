#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "init.h"

extern TaskHandle_t task_handle_init;

void app_main(void) {

  xTaskCreatePinnedToCore(init, "init", 8912, NULL, 5, &task_handle_init, 1);
 
}



