#pragma once
#include <stdint.h>
#include "esp_err.h"

typedef struct {
    uint8_t seg;
    uint8_t min;
    uint8_t hora;
    uint8_t dia;
    uint8_t mes;
    uint8_t anio;
} ds1307_time_t;

esp_err_t ds1307_init(ds1307_time_t *hora_inicial);
esp_err_t ds1307_get_time(ds1307_time_t *t);
esp_err_t ds1307_set_time(ds1307_time_t *t);
