#include "rgb_control.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include "driver/uart.h"


#define UART_PORT      UART_NUM_0
#define UART_RX_SIZE   1024
#define UART_TX_SIZE   256
#define BUF_SIZE       64

#define COLOR_VALUE_LED1     200    
#define INTENSITY_LED1       10   

#define COLOR_VALUE_LED2     255  
#define INTENSITY_LED2       10    

static void uart_event_task(void *arg)
{
    uint8_t data[BUF_SIZE];

    while (1) {
        int len = uart_read_bytes(UART_PORT, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);

        if (len > 0) {
            while (len > 0 && (data[len - 1] == '\r' || data[len - 1] == '\n')) {
                len--;
            }
            data[len] = '\0';

            uint8_t led_id, color, intensity;
            if (sscanf((char *)data, "%hhu %hhu %hhu", &led_id, &color, &intensity) == 3) {
                if ((led_id == 1 || led_id == 2) && color <= 255 && intensity <= 100) {
                    if (led_id == 1)
                        rgb_set_single(1, color, intensity);
                    else
                        rgb_set_single(2, color, intensity);

                    char msg[64];
                    int n = snprintf(msg, sizeof(msg),
                                     "LED%d -> Color=%u Intensity=%u%%\r\n",
                                     led_id, color, intensity);
                    uart_write_bytes(UART_PORT, msg, n);
                } else {
                    const char *err = "Format: <led 1|2> <color 0–255> <intensity 0–100>\r\n";
                    uart_write_bytes(UART_PORT, err, strlen(err));
                }
            } else {
                const char *fmt = "Format: <led 1|2> <color 0–255> <intensity 0–100>\r\n";
                uart_write_bytes(UART_PORT, fmt, strlen(fmt));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void app_main(void)
{
    rgb_init();
    vTaskDelay(pdMS_TO_TICKS(100));  // Sincroniza LEDC

    // 🔹 Configura manualmente ambos LEDs
    rgb_set_single(1, COLOR_VALUE_LED1, INTENSITY_LED1);
    printf("LED1 -> Color=%u | Intensity=%u%%\n", COLOR_VALUE_LED1, INTENSITY_LED1);

    rgb_set_single(2, COLOR_VALUE_LED2, INTENSITY_LED2);
    printf("LED2 -> Color=%u | Intensity=%u%%\n", COLOR_VALUE_LED2, INTENSITY_LED2);

    // 🔹 UART opcional (por si quieres enviar comandos después)
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    ESP_ERROR_CHECK(uart_param_config(UART_PORT, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_PORT, UART_RX_SIZE, UART_TX_SIZE, 0, NULL, 0));

    uart_flush_input(UART_PORT);
    uart_set_mode(UART_PORT, UART_MODE_UART);

    const char *start_msg =
        "\r\nSystem ready. Format: <led 1|2> <color 0–255> <intensity 0–100>\r\n";
    uart_write_bytes(UART_PORT, start_msg, strlen(start_msg));

    // 🟢 Mantiene activa la lectura UART (sin bloquear)
    xTaskCreate(uart_event_task, "uart_event_task", 6144, NULL, 5, NULL);
}