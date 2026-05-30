#include "rc522.h"
#include "pines.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "rc522";

#define REG_COMMAND      0x01
#define REG_COM_IRQ      0x04
#define REG_DIV_IRQ      0x05
#define REG_ERROR        0x06
#define REG_FIFO_DATA    0x09
#define REG_FIFO_LEVEL   0x0A
#define REG_BIT_FRAMING  0x0D
#define REG_COLL         0x0E
#define REG_MODE         0x11
#define REG_TX_CONTROL   0x14
#define REG_TX_ASK       0x15
#define REG_CRC_L        0x22
#define REG_CRC_H        0x21
#define REG_T_MODE       0x2A
#define REG_T_PRESC      0x2B
#define REG_T_RELOAD_H   0x2C
#define REG_T_RELOAD_L   0x2D
#define REG_VERSION      0x37

#define CMD_IDLE         0x00
#define CMD_CALC_CRC     0x03
#define CMD_TRANSCEIVE   0x0C
#define CMD_RESET        0x0F

#define PICC_REQA        0x26
#define PICC_ANTICOLL    0x93
#define PICC_HALT        0x50

static spi_device_handle_t spi;

static void rc522_escribir(uint8_t reg, uint8_t val)
{
    uint8_t tx[2] = { (reg << 1) & 0x7E, val };
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = NULL };
    spi_device_polling_transmit(spi, &t);
}

static uint8_t rc522_leer(uint8_t reg)
{
    uint8_t tx[2] = { ((reg << 1) & 0x7E) | 0x80, 0x00 };
    uint8_t rx[2] = {0, 0};
    spi_transaction_t t = { .length = 16, .tx_buffer = tx, .rx_buffer = rx };
    spi_device_polling_transmit(spi, &t);
    return rx[1];
}

static void poner_bits(uint8_t reg, uint8_t mask)
{
    rc522_escribir(reg, rc522_leer(reg) | mask);
}

static void quitar_bits(uint8_t reg, uint8_t mask)
{
    rc522_escribir(reg, rc522_leer(reg) & ~mask);
}

static void rc522_reset(void)
{
    rc522_escribir(REG_COMMAND, CMD_RESET);
    vTaskDelay(pdMS_TO_TICKS(50));
}

static void calcular_crc(uint8_t *datos, uint8_t len, uint8_t *salida)
{
    rc522_escribir(REG_COMMAND, CMD_IDLE);
    rc522_escribir(REG_DIV_IRQ, 0x04);
    rc522_escribir(REG_FIFO_LEVEL, 0x80);

    for (int i = 0; i < len; i++) {
        rc522_escribir(REG_FIFO_DATA, datos[i]);
    }
    rc522_escribir(REG_COMMAND, CMD_CALC_CRC);

    for (int i = 0; i < 150; i++) {
        if (rc522_leer(REG_DIV_IRQ) & 0x04) break;
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    rc522_escribir(REG_COMMAND, CMD_IDLE);

    salida[0] = rc522_leer(REG_CRC_L);
    salida[1] = rc522_leer(REG_CRC_H);
}

static bool transceive(uint8_t *tx_buf, uint8_t tx_len,
                        uint8_t *rx_buf, uint8_t *rx_len,
                        uint8_t ultimos_bits)
{
    rc522_escribir(REG_COMMAND, CMD_IDLE);
    rc522_escribir(REG_COM_IRQ, 0x7F);
    rc522_escribir(REG_FIFO_LEVEL, 0x80);

    for (int i = 0; i < tx_len; i++) {
        rc522_escribir(REG_FIFO_DATA, tx_buf[i]);
    }

    rc522_escribir(REG_BIT_FRAMING, ultimos_bits);
    rc522_escribir(REG_COMMAND, CMD_TRANSCEIVE);
    poner_bits(REG_BIT_FRAMING, 0x80); // StartSend

    bool ok = false;
    for (int i = 0; i < 200; i++) {
        uint8_t irq = rc522_leer(REG_COM_IRQ);
        if (irq & 0x01) break;
        if (irq & 0x30) {
            ok = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    quitar_bits(REG_BIT_FRAMING, 0x80);

    if (!ok) return false;
    if (rc522_leer(REG_ERROR) & 0x1B) return false;

    uint8_t nivel = rc522_leer(REG_FIFO_LEVEL);
    if (rx_len) *rx_len = nivel;
    if (rx_buf) {
        for (int i = 0; i < nivel && i < 20; i++) {
            rx_buf[i] = rc522_leer(REG_FIFO_DATA);
        }
    }
    return true;
}

esp_err_t rc522_init(void)
{
    gpio_config_t io;
    io.pin_bit_mask = 1ULL << PIN_RC522_RST;
    io.mode = GPIO_MODE_OUTPUT;
    io.pull_up_en = GPIO_PULLUP_DISABLE;
    io.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&io);
    gpio_set_level(PIN_RC522_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    spi_bus_config_t bus;
    bus.mosi_io_num = PIN_RC522_MOSI;
    bus.miso_io_num = PIN_RC522_MISO;
    bus.sclk_io_num = PIN_RC522_SCK;
    bus.quadwp_io_num = -1;
    bus.quadhd_io_num = -1;
    bus.max_transfer_sz = 0;

    spi_device_interface_config_t dev;
    dev.clock_speed_hz = 5000000;
    dev.mode = 0;
    dev.spics_io_num = PIN_RC522_SS;
    dev.queue_size = 7;
    dev.pre_cb = NULL;
    dev.post_cb = NULL;

    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &bus, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) return ret;

    ret = spi_bus_add_device(SPI3_HOST, &dev, &spi);
    if (ret != ESP_OK) return ret;

    rc522_reset();

    // timer de timeout (valores del ejemplo de Nordic/Espressif)
    rc522_escribir(REG_T_MODE,     0x80);
    rc522_escribir(REG_T_PRESC,    0xA9);
    rc522_escribir(REG_T_RELOAD_H, 0x03);
    rc522_escribir(REG_T_RELOAD_L, 0xE8);

    rc522_escribir(REG_TX_ASK, 0x40);
    rc522_escribir(REG_MODE,   0x3D); // CRC preset 0x6363
    poner_bits(REG_TX_CONTROL, 0x03);

    uint8_t ver = rc522_leer(REG_VERSION);
    ESP_LOGI(TAG, "RC522 version: 0x%02X", ver);

    return ESP_OK;
}

bool rc522_read_uid(rc522_uid_t *uid)
{
    uint8_t reqa = PICC_REQA;
    uint8_t atqa[2] = {0};
    uint8_t atqa_len = 0;

    rc522_escribir(REG_COLL, 0x80);

    // REQA en 7 bits, sin CRC
    if (!transceive(&reqa, 1, atqa, &atqa_len, 7)) return false;
    if (atqa_len < 2) return false;

    uint8_t anticoll[2] = { PICC_ANTICOLL, 0x20 };
    uint8_t uid_buf[5] = {0};
    uint8_t uid_len = 0;

    if (!transceive(anticoll, 2, uid_buf, &uid_len, 0)) return false;
    if (uid_len < 5) return false;

    // BCC es XOR de los 4 bytes del UID
    uint8_t bcc = uid_buf[0] ^ uid_buf[1] ^ uid_buf[2] ^ uid_buf[3];
    if (bcc != uid_buf[4]) return false;

    // SELECT con CRC
    uint8_t sel[9];
    sel[0] = PICC_ANTICOLL;
    sel[1] = 0x70;
    memcpy(&sel[2], uid_buf, 5);

    uint8_t crc[2];
    calcular_crc(sel, 7, crc);
    sel[7] = crc[0];
    sel[8] = crc[1];

    uint8_t sak[3] = {0};
    uint8_t sak_len = 0;
    if (!transceive(sel, 9, sak, &sak_len, 0)) return false;
    if (sak_len < 1) return false;

    uid->size = 4;
    memcpy(uid->bytes, uid_buf, 4);
    return true;
}

void rc522_halt(void)
{
    uint8_t halt[4];
    halt[0] = PICC_HALT;
    halt[1] = 0x00;

    uint8_t crc[2];
    calcular_crc(halt, 2, crc);
    halt[2] = crc[0];
    halt[3] = crc[1];

    transceive(halt, 4, NULL, NULL, 0);
}
