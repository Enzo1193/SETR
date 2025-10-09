// schedule_register.c
#include "schedule_register.h"
#include "nvs_flash.h"
#include "esp_err.h"
#include <string.h>

void save_schedule_register(uint8_t register_num, ScheduleRegister *register_data) {
    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READWRITE, &nvs_handle));

    char key[16];
    sprintf(key, "schedule_%d", register_num);

    ESP_ERROR_CHECK(nvs_set_blob(nvs_handle, key, register_data, sizeof(ScheduleRegister)));
    ESP_ERROR_CHECK(nvs_commit(nvs_handle));
    nvs_close(nvs_handle);
}

void load_schedule_register(uint8_t register_num, ScheduleRegister *register_data) {
    nvs_handle_t nvs_handle;
    ESP_ERROR_CHECK(nvs_open("storage", NVS_READONLY, &nvs_handle));

    char key[16];
    sprintf(key, "schedule_%d", register_num);

    size_t required_size;
    ESP_ERROR_CHECK(nvs_get_blob(nvs_handle, key, NULL, &required_size));
    ESP_ERROR_CHECK(nvs_get_blob(nvs_handle, key, register_data, &required_size));
    nvs_close(nvs_handle);
}
