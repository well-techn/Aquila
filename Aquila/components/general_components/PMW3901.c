#include "PMW3901.h"
#include "wt_spi.h" 
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "wt_alldef.h"
#include <rom/ets_sys.h>
#include "driver/gpio.h"
#include "esp_log.h"

extern spi_device_handle_t PMW3901;
extern char *TAG_PMW;

uint8_t PMW3901_read_address(uint8_t address)
{
  uint8_t value = 0;
  gpio_set_level(GPIO_CS_PMW3901,0);
  ets_delay_us(1);
  SPI_write_byte (PMW3901, 0, NULL, 0, NULL, 0, address);
  ets_delay_us(35);
  value = SPI_read_byte (PMW3901,  0, NULL, 0, NULL, NULL);
  ets_delay_us(1);
  gpio_set_level(GPIO_CS_PMW3901, 1);
  //printf ("read value from 0x%02x is 0x%02x\n",address, value);
  ets_delay_us(50);

  return value;
}

void PMW3901_write_address(uint8_t address, uint8_t value)
{
  gpio_set_level(GPIO_CS_PMW3901,0);
  ets_delay_us(1);
  SPI_write_byte (PMW3901, 0, 0, 8, address | 0x80, 0, value);
  ets_delay_us(1);
  gpio_set_level(GPIO_CS_PMW3901, 1);
  ets_delay_us(50);
}

esp_err_t PMW3901_communication_check(void)
{
    esp_err_t err = ESP_FAIL;
    uint8_t product_ID = 0;
    uint8_t inv_product_ID = 0;

    gpio_set_level(GPIO_CS_PMW3901, 1);
    ets_delay_us(50);
    PMW3901_write_address (PMW3901_POWER_UP_RESET,  0x5A);
    ets_delay_us(1200);
    
    product_ID = PMW3901_read_address (PMW3901_PRODUCT_ID);
    ets_delay_us(1000);
    inv_product_ID = PMW3901_read_address (PMW3901_INV_PRODUCT_ID);
        if ((product_ID == 0x49) && (inv_product_ID == 0xB6)) 
        {//printf("PMW3901 is online\n");
        ESP_LOGI(TAG_PMW,"PMW3901 is online\n");
        err = ESP_OK;}
    else ESP_LOGI(TAG_PMW,"PMW3901 is offline, returned values are %d and %d\n", product_ID, inv_product_ID);
    //printf("PMW3901 is offline, returned values are %d and %d\n", product_ID, inv_product_ID); 
    return err;
}

void PMW3901_read_motion_burst(uint8_t* buffer)
{
  gpio_set_level(GPIO_CS_PMW3901,0);
  ets_delay_us(1);
  SPI_write_byte (PMW3901, 0, NULL, 0, NULL, 0, PMW3901_MOTION_BURST);
  ets_delay_us(35);
  SPI_read_bytes (PMW3901, 0, NULL, 0, NULL, 0, buffer, 12);
  ets_delay_us(1);
  gpio_set_level(GPIO_CS_PMW3901,1);
  ets_delay_us(1);
}

void PMW3901_config(void)  
{
  uint8_t counter = 0;
  uint8_t value, C1, C2;  
  //delay 40 ms after power up

  gpio_set_level(GPIO_CS_PMW3901, 1);
  ets_delay_us(50);
  PMW3901_write_address (PMW3901_POWER_UP_RESET,  0x5A);
  ets_delay_us(1200);
  
  value = PMW3901_read_address (0x02);
  value = PMW3901_read_address (0x03);
  value = PMW3901_read_address (0x04);
  value = PMW3901_read_address (0x05);
  value = PMW3901_read_address (0x06);

  PMW3901_write_address (0x7F, 0x00);
  PMW3901_write_address (0x55, 0x01);
  PMW3901_write_address (0x50, 0x07);
  PMW3901_write_address (0x7F, 0x0E);

  PMW3901_write_address (0x43, 0x10);
  
  do 
  {
    value = PMW3901_read_address (0x47);
    counter++;
  }
  while ((value != 0x08) && (counter < 3));

  if (value !=0x08) printf( "cannot get right value from 0x47, read value is %02x\n", value);

  value = PMW3901_read_address (0x67);
  if (value & 0x80) PMW3901_write_address (0x48, 0x04); 
  else PMW3901_write_address (0x48, 0x02);

  PMW3901_write_address (0x7F, 0x00);
  PMW3901_write_address (0x51, 0x7B);
  PMW3901_write_address (0x50, 0x00);
  PMW3901_write_address (0x55, 0x0E);
  PMW3901_write_address (0x7F, 0x0E);

  value = PMW3901_read_address (0x73);

  if (value == 0x00)
  {
    C1 = PMW3901_read_address (0x70);
    if (C1 <= 28) C1+=14;
    else C1+=11;
    if (C1 > 0x3F) C1 = 0x3F;

    C2 = PMW3901_read_address (0x71);
    C2 = (C2 * 45) / 100;

    PMW3901_write_address (0x7F, 0x00);
    PMW3901_write_address (0x61, 0xAD);
    PMW3901_write_address (0x51, 0x70);
    PMW3901_write_address (0x7F, 0x0E);
    PMW3901_write_address (0x70, C1);
    PMW3901_write_address (0x71, C2);
  }

  PMW3901_write_address (0x7F, 0x00);
  PMW3901_write_address (0x61, 0xAD);
  PMW3901_write_address (0x7F, 0x03);
  PMW3901_write_address (0x40, 0x00);
  PMW3901_write_address (0x7F, 0x05);
  PMW3901_write_address (0x41, 0xB3);
  PMW3901_write_address (0x43, 0xF1); 
  PMW3901_write_address (0x45, 0x14);   
  PMW3901_write_address (0x5B, 0x32);   
  PMW3901_write_address (0x5F, 0x34);  
  PMW3901_write_address (0x7B, 0x08);  
  PMW3901_write_address (0x7F, 0x06);  
  PMW3901_write_address (0x44, 0x1B);  
  PMW3901_write_address (0x40, 0xBF);   
  PMW3901_write_address (0x4E, 0x3F);   
  PMW3901_write_address (0x7F, 0x08);   
  PMW3901_write_address (0x65, 0x20);  
  PMW3901_write_address (0x6A, 0x18);   
  PMW3901_write_address (0x7F, 0x09);   
  PMW3901_write_address (0x4F, 0xAF);   
  PMW3901_write_address (0x5F, 0x40);  
  PMW3901_write_address (0x48, 0x80);  
  PMW3901_write_address (0x49, 0x80);  
  PMW3901_write_address (0x57, 0x77);  
  PMW3901_write_address (0x60, 0x78);  
  PMW3901_write_address (0x61, 0x78);  
  PMW3901_write_address (0x62, 0x08);  
  PMW3901_write_address (0x63, 0x50);  
  PMW3901_write_address (0x7F, 0x0A);  
  PMW3901_write_address (0x45, 0x60);  
  PMW3901_write_address (0x7F, 0x00);  
  PMW3901_write_address (0x4D, 0x11);  
  PMW3901_write_address (0x55, 0x80);  
  PMW3901_write_address (0x74, 0x1F);  
  PMW3901_write_address (0x75, 0x1F);  
  PMW3901_write_address (0x4A, 0x78);   
  PMW3901_write_address (0x4B, 0x78);   
  PMW3901_write_address (0x44, 0x08);  
  PMW3901_write_address (0x45, 0x50);
  PMW3901_write_address (0x64, 0xFF);
  PMW3901_write_address (0x65, 0x1F);  
  PMW3901_write_address (0x7F, 0x14);  
  PMW3901_write_address (0x65, 0x60);  
  PMW3901_write_address (0x66, 0x08);  
  PMW3901_write_address (0x63, 0x78);  
  PMW3901_write_address (0x7F, 0x15);  
  PMW3901_write_address (0x48, 0x58);  
  PMW3901_write_address (0x7F, 0x07);   
  PMW3901_write_address (0x41, 0x0D);  
  PMW3901_write_address (0x43, 0x14);  
  PMW3901_write_address (0x4B, 0x0E);  
  PMW3901_write_address (0x45, 0x0F);  
  PMW3901_write_address (0x44, 0x42);  
  PMW3901_write_address (0x4C, 0x80);  
  PMW3901_write_address (0x7F, 0x10); 
  PMW3901_write_address (0x5B, 0x02);  
  PMW3901_write_address (0x7F, 0x07);   
  PMW3901_write_address (0x40, 0x41);   
  PMW3901_write_address (0x70, 0x00);
  
  ets_delay_us(10000);

  PMW3901_write_address (0x32, 0x44);  
  PMW3901_write_address (0x7F, 0x07);  
  PMW3901_write_address (0x40, 0x40);  
  PMW3901_write_address (0x7F, 0x06); 
  PMW3901_write_address (0x62, 0xF0); 
  PMW3901_write_address (0x63, 0x00);  
  PMW3901_write_address (0x7F, 0x0D); 
  PMW3901_write_address (0x48, 0xC0);  
  PMW3901_write_address (0x6F, 0xD5);  
  PMW3901_write_address (0x7F, 0x00);  
  PMW3901_write_address (0x5B, 0xA0);  
  PMW3901_write_address (0x4E, 0xA8);  
  PMW3901_write_address (0x5A, 0x50);  
  PMW3901_write_address (0x40, 0x80);

  ESP_LOGI(TAG_PMW,"Finished PMW3901 configuration\n");

}  

void PMW3901_get_raw_image(uint8_t* FBuffer)
{
  //power up reset
  gpio_set_level(GPIO_CS_PMW3901, 1);
  ets_delay_us(50);
  PMW3901_write_address (PMW3901_POWER_UP_RESET,  0x5A);
  ets_delay_us(1200);
  //entering the mode
  PMW3901_write_address (0x7F, 0x07);
  PMW3901_write_address (0x41, 0x1D);
  PMW3901_write_address (0x4C, 0x00);
  PMW3901_write_address (0x7F, 0x08);
  PMW3901_write_address (0x6A, 0x38);
  PMW3901_write_address (0x7F, 0x00);
  PMW3901_write_address (0x55, 0x04);
  PMW3901_write_address (0x40, 0x80);
  PMW3901_write_address (0x4D, 0x11);



  PMW3901_write_address (0x70, 0x00);
  PMW3901_write_address (0x58, 0xFF);

  uint8_t temp, check = 0;

  do 
  { // keep reading
    temp = PMW3901_read_address (PMW3901_RAWDATA_GRAB_STATUS); 
    temp = (temp  >> 6) & 0b00000011;
    check++; 
  } while(temp != 0x03); 
  printf("number of cycles is %d\n", check);
  vTaskDelay(500/portTICK_PERIOD_MS);

  int count = 0;
  uint8_t a; //temp value for reading register
  //uint8_t b; //temp value for second register
  uint8_t hold; //holding value for checking bits
  uint8_t mask = 0x03; //mask 
  uint8_t pixel = 0; //temp holding value for pixel
  uint8_t pixel_is_ready = 0;

    for (int ii = 0; ii < 1225; ii++) 
    {
      pixel_is_ready = 0;
      do {
        a = PMW3901_read_address (PMW3901_RAWDATA_GRAB); //read register
        hold = a >> 6; //right shift to leave top two bits for ease of check.
        if (hold == 0x01) 
        { 
          pixel = a; //set pixel to a
          pixel = pixel << 2; //push left to 7:2
          //printf("high part is %d ", pixel);
          //if data is upper 6 bits
        }

        if (hold == 0x02) 
        { 
          a = (a >> 2) & 0b00000011;
          //printf("low part is %d ", a);
          pixel += a; //set pixel to a
          //printf("overal pixel is %d\n", pixel);
          FBuffer[count++] = pixel; //put temp value in fbuffer array
          pixel_is_ready = 1;
        }
        
      }
      while(!(pixel_is_ready));
    }




    
    /*
    hold = a >> 6; //right shift to leave top two bits for ease of check.
  if (hold == 0x01) 
        { 
          pixel = a; //set pixel to a
          pixel = pixel << 2; //push left to 7:2
          //printf("high part is %d ", pixel);
          //if data is upper 6 bits
        }

        if (hold == 0x02) 
        { 
          a = (a >> 2) & 0b00000011;
          //printf("low part is %d ", a);
          pixel += a; //set pixel to a
          //printf("overal pixel is %d\n", pixel);
          FBuffer[count++] = pixel; //put temp value in fbuffer array
          pixel_is_ready = 1;
        }


pixel = 0;
        if ((a & 0b11000000) == 0b01000000)  
        {
          pixel &= ~0b11111100;
          pixel |= (a & 0b00111111) << 2;         //# Held in 5:0
        }
        if ((a & 0b11000000) == 0b10000000)
        {  
          pixel &= ~0b00000011;
          pixel |= (a & 0b00001100) >> 2;  //# Held in 3:2
          FBuffer[count++] = pixel;
          pixel_is_ready = 1;
        }  





















   */


   
 
  //exiting the mode
  PMW3901_write_address (0x7F, 0x00);
  PMW3901_write_address (0x4D, 0x11);
  PMW3901_write_address (0x40, 0x80);
  PMW3901_write_address (0x55, 0x80);
  PMW3901_write_address (0x7F, 0x08);
  PMW3901_write_address (0x6A, 0x18);
  PMW3901_write_address (0x7F, 0x07);
  PMW3901_write_address (0x41, 0x0D);
  PMW3901_write_address (0x4C, 0x80);
  PMW3901_write_address (0x7F, 0x00);
  
}