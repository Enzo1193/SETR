/**
 * Application entry point.
 */

 #include "nvs_flash.h"
 #include "http_server.h"
 #include "wifi_app.h"
 #include "driver/gpio.h"
 #include "freertos/FreeRTOS.h"
 #include "freertos/task.h"
 #include "freertos/queue.h"
 
 #include "led_rgb.h"
 #include "uart.h"
 #include "adc.h"
 #include "isr.h"
 #include "http_server.h"
 
 #include "servo.h"
 #include "esp_log.h"
 #include "time.h"

 
 
 #define BLINK_GPIO				2
 
 
 static const char *TAG = "Main";
 
 LED LED1;
 LED LED2;

 
 
 void app_main(void)
 {
     adc_config();
     adc_config2();

     servo_init();
     servo_set_angle(0);
     init_uart0();

     LED1 = (LED){
         .CHANEL_R = LEDC_CHANNEL_0, .CHANEL_G = LEDC_CHANNEL_1, .CHANEL_B = LEDC_CHANNEL_2,
         .PIN_R = 5, .PIN_G = 18, .PIN_B = 19,
         .Duty_R = 50, .Duty_G = 50, .Duty_B = 50
     };
     LED2 = (LED){
         .CHANEL_R = LEDC_CHANNEL_3, .CHANEL_G = LEDC_CHANNEL_4, .CHANEL_B = LEDC_CHANNEL_5,
         .PIN_R = 14, .PIN_G = 13, .PIN_B = 12,
         .Duty_R = 0, .Duty_G = 0, .Duty_B = 0
     };
     configurar_led(&LED1);
     configurar_led(&LED2);
     ESP_LOGI(TAG, "LEDs initialized successfully");
 
     // Initialize NVS
     esp_err_t ret = nvs_flash_init();
     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
     {
         ESP_ERROR_CHECK(nvs_flash_erase());
         ret = nvs_flash_init();
     }
     ESP_ERROR_CHECK(ret);
 
     // Start Wifi
 
     wifi_app_start();
     http_server_start();
     isr_install();

    

 }
 
         