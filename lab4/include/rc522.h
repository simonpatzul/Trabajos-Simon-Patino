#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

typedef struct {
    uint8_t bytes[10];
    uint8_t size;
} rc522_uid_t;

esp_err_t rc522_init(void);
bool rc522_read_uid(rc522_uid_t *uid);
void rc522_halt(void);
