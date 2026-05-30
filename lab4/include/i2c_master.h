#pragma once
#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

esp_err_t i2c_master_init(void);
esp_err_t i2c_master_write_to(uint8_t addr, uint8_t *data, size_t len);
esp_err_t i2c_master_read_reg(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len);
