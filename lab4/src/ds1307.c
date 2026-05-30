#include "ds1307.h"
#include "i2c_master.h"

#define DS1307_ADDR 0x68
#define REG_SEG     0x00  // registro segundos, bit7 es CH (clock halt)

// convierte BCD a decimal
static uint8_t bcd_a_dec(uint8_t bcd)
{
    return (bcd >> 4) * 10 + (bcd & 0x0F);
}

// convierte decimal a BCD
static uint8_t dec_a_bcd(uint8_t dec)
{
    return ((dec / 10) << 4) | (dec % 10);
}

esp_err_t ds1307_set_time(ds1307_time_t *t)
{
    uint8_t buf[8];

    buf[0] = REG_SEG;
    buf[1] = dec_a_bcd(t->seg) & 0x7F;  // CH=0 para que el oscilador corra
    buf[2] = dec_a_bcd(t->min);
    buf[3] = dec_a_bcd(t->hora) & 0x3F; // modo 24h
    buf[4] = 0x01;                        // dia semana (no lo usamos)
    buf[5] = dec_a_bcd(t->dia);
    buf[6] = dec_a_bcd(t->mes);
    buf[7] = dec_a_bcd(t->anio);

    return i2c_master_write_to(DS1307_ADDR, buf, 8);
}

esp_err_t ds1307_get_time(ds1307_time_t *t)
{
    uint8_t buf[7];
    esp_err_t ret = i2c_master_read_reg(DS1307_ADDR, REG_SEG, buf, 7);
    if (ret != ESP_OK) return ret;

    t->seg  = bcd_a_dec(buf[0] & 0x7F);  // quitar bit CH
    t->min  = bcd_a_dec(buf[1]);
    t->hora = bcd_a_dec(buf[2] & 0x3F);  // quitar bits de modo 12/24
    // buf[3] es dia semana, lo ignoramos
    t->dia  = bcd_a_dec(buf[4]);
    t->mes  = bcd_a_dec(buf[5]);
    t->anio = bcd_a_dec(buf[6]);

    return ESP_OK;
}

esp_err_t ds1307_init(ds1307_time_t *hora_inicial)
{
    uint8_t seg_reg = 0;
    esp_err_t ret = i2c_master_read_reg(DS1307_ADDR, REG_SEG, &seg_reg, 1);
    if (ret != ESP_OK) return ret;

    // si CH=1 significa que el oscilador estaba apagado (sin bateria o primera vez)
    if (seg_reg & 0x80) {
        ret = ds1307_set_time(hora_inicial);
    }
    return ret;
}
