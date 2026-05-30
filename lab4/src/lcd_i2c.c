#include "lcd_i2c.h"
#include "i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// direccion I2C del PCF8574
#define LCD_ADDR  0x27

// bits del expansor: P0=RS, P1=RW, P2=EN, P3=BL, P4-P7=D4-D7
#define RS  0x01
#define RW  0x02
#define EN  0x04
#define BL  0x08  // backlight siempre encendido

static void escribir_expansor(uint8_t dato)
{
    i2c_master_write_to(LCD_ADDR, &dato, 1);
}

static void pulso_enable(uint8_t dato)
{
    uint8_t b;

    b = dato | EN;
    escribir_expansor(b);
    vTaskDelay(pdMS_TO_TICKS(1));

    b = dato & ~EN;
    escribir_expansor(b);
    vTaskDelay(pdMS_TO_TICKS(1));
}

// manda solo 4 bits (nibble alto del byte que arma el PCF8574)
static void enviar_nibble(uint8_t nibble, uint8_t rs)
{
    uint8_t dato = ((nibble & 0x0F) << 4) | BL;
    if (rs) dato |= RS;
    pulso_enable(dato);
}

// manda byte completo en dos nibbles (primero el alto)
static void enviar_byte(uint8_t valor, uint8_t rs)
{
    enviar_nibble(valor >> 4, rs);
    enviar_nibble(valor & 0x0F, rs);
}

void lcd_init(void)
{
    vTaskDelay(pdMS_TO_TICKS(50));

    // secuencia de reset segun datasheet HD44780
    enviar_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(5));
    enviar_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
    enviar_nibble(0x03, 0);
    vTaskDelay(pdMS_TO_TICKS(1));

    // cambiar a 4 bits
    enviar_nibble(0x02, 0);
    vTaskDelay(pdMS_TO_TICKS(1));

    // configurar: 4bit, 2 lineas, 5x8
    enviar_byte(0x28, 0);
    vTaskDelay(pdMS_TO_TICKS(1));

    // apagar display
    enviar_byte(0x08, 0);
    vTaskDelay(pdMS_TO_TICKS(1));

    // limpiar
    enviar_byte(0x01, 0);
    vTaskDelay(pdMS_TO_TICKS(2));

    // modo entrada: cursor avanza a la derecha
    enviar_byte(0x06, 0);
    vTaskDelay(pdMS_TO_TICKS(1));

    // prender display, sin cursor ni parpadeo
    enviar_byte(0x0C, 0);
    vTaskDelay(pdMS_TO_TICKS(1));
}

void lcd_clear(void)
{
    enviar_byte(0x01, 0);
    vTaskDelay(pdMS_TO_TICKS(2));
}

void lcd_set_cursor(uint8_t col, uint8_t row)
{
    // fila 0 empieza en 0x00, fila 1 en 0x40
    uint8_t offset = (row == 0) ? 0x00 : 0x40;
    enviar_byte(0x80 | (offset + col), 0);
    vTaskDelay(pdMS_TO_TICKS(1));
}

void lcd_print_char(char c)
{
    enviar_byte((uint8_t)c, 1);
}

void lcd_print(const char *str)
{
    while (*str) {
        lcd_print_char(*str);
        str++;
    }
}
