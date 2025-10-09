#include <stdio.h>
#include "driver/ledc.h"
#include "esp_err.h"
#include "library_ledc.c"

void app_main(void)
{
    
    //Forma uno de llenar los datos de la estructura
    LED LED1;
    //establecer canales para cada Color
    LED1.CHANEL_R = LEDC_CHANNEL_0;
    LED1.CHANEL_G = LEDC_CHANNEL_1;
    LED1.CHANEL_B = LEDC_CHANNEL_2;
    //establecer pin de salida
    LED1.PIN_R = 5;
    LED1.PIN_G = 18;
    LED1.PIN_B = 19;
    //establecer % de color led
    LED1.Duty_R= 0;
    LED1.Duty_G = 255;
    LED1.Duty_B = 201;
    configurar_led(&LED1);

    //Forma dos de llenar los datos de la estructura
  
    LED LED2 = {
        //establecer canales para cada Color
        .CHANEL_R = LEDC_CHANNEL_3,  
        .CHANEL_G = LEDC_CHANNEL_4,  
        .CHANEL_B = LEDC_CHANNEL_5,  
        //establecer pin de salida
        .PIN_R = 21,    
        .PIN_G = 22,    
        .PIN_B = 23,
        //establecer % de color led
        .Duty_R= 1,
        .Duty_G = 2,
        .Duty_B = 3
    };
    configurar_led(&LED2);

   // ledc_set_duty(LEDC_LOW_SPEED_MODE, LED1.CHANEL_R, 20);
   //ledc_update_duty(LEDC_LOW_SPEED_MODE, LED1.CHANEL_R);


}