#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "pines.h"
#include "panel_hmi.h"
#include "i2c_master.h"
#include "lcd_i2c.h"
#include "ds1307.h"
#include "rc522.h"
#include "ble_nus.h"

static const char *TAG = "main";

volatile estado_t g_estado    = ESTADO_BLOQUEADO;
char              g_msg_ble[17] = {0};
bool              g_hay_mensaje = false;

#define NUM_UIDS_AUTH 2
static const uint8_t UIDS_AUTH[NUM_UIDS_AUTH][4] = {
    { 0xDE, 0xAD, 0xBE, 0xEF },
    { 0xAB, 0xCD, 0x12, 0x34 }
};

static SemaphoreHandle_t mutex;

// rellena con espacios para que no queden residuos en el LCD
static void lcd_linea(const char *str)
{
    char buf[17];
    snprintf(buf, sizeof(buf), "%-16s", str);
    lcd_print(buf);
}

static void pantalla_bloqueado(void)
{
    lcd_set_cursor(0, 0);
    lcd_linea("Panel bloqueado ");
    lcd_set_cursor(0, 1);
    lcd_linea("Acerque credencial");
}

static bool uid_es_valido(rc522_uid_t *uid)
{
    int i;
    for (i = 0; i < NUM_UIDS_AUTH; i++) {
        if (memcmp(uid->bytes, UIDS_AUTH[i], 4) == 0) return true;
    }
    return false;
}

static void buzzer_on(void)  { gpio_set_level(PIN_BUZZER, 1); }
static void buzzer_off(void) { gpio_set_level(PIN_BUZZER, 0); }

static void leds_apagar(void)
{
    gpio_set_level(PIN_LED_ROJO,  0);
    gpio_set_level(PIN_LED_VERDE, 0);
    gpio_set_level(PIN_LED_AZUL,  0);
}

static void accion_acceso_concedido(void)
{
    ds1307_time_t t;
    char hora[17];

    leds_apagar();
    gpio_set_level(PIN_LED_VERDE, 1);

    ds1307_get_time(&t);
    snprintf(hora, sizeof(hora), "%02d:%02d:%02d", t.hora, t.min, t.seg);

    lcd_set_cursor(0, 0);
    lcd_linea("Acceso concedido");
    lcd_set_cursor(0, 1);
    lcd_linea(hora);

    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(500));
    buzzer_off();
    vTaskDelay(pdMS_TO_TICKS(500));

    gpio_set_level(PIN_LED_VERDE, 0);

    memset(g_msg_ble, 0, sizeof(g_msg_ble));
    g_hay_mensaje = false;

    ds1307_get_time(&t);
    snprintf(hora, sizeof(hora), "%02d:%02d:%02d", t.hora, t.min, t.seg);

    lcd_set_cursor(0, 0);
    lcd_linea("Sin mensajes    ");
    lcd_set_cursor(0, 1);
    lcd_linea(hora);

    gpio_set_level(PIN_LED_AZUL, 1);
    g_estado = ESTADO_ACTIVO;
}

static void accion_acceso_denegado(void)
{
    int i;

    lcd_set_cursor(0, 0);
    lcd_linea("Acceso denegado ");
    lcd_set_cursor(0, 1);
    lcd_linea("UID no registrado");

    buzzer_on();
    for (i = 0; i < 3; i++) {
        gpio_set_level(PIN_LED_ROJO, 1);
        vTaskDelay(pdMS_TO_TICKS(300));
        gpio_set_level(PIN_LED_ROJO, 0);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    buzzer_off();

    pantalla_bloqueado();
    gpio_set_level(PIN_LED_ROJO, 1);
}

static void accion_cierre_sesion(void)
{
    g_estado = ESTADO_BLOQUEADO;

    buzzer_on();
    vTaskDelay(pdMS_TO_TICKS(500));
    buzzer_off();

    leds_apagar();
    gpio_set_level(PIN_LED_ROJO, 1);
    pantalla_bloqueado();
}

static void tarea_rfid(void *pvParam)
{
    rc522_uid_t uid;
    bool tarjeta_antes = false;

    while (1) {
        bool hay_tarjeta = rc522_read_uid(&uid);

        if (hay_tarjeta && !tarjeta_antes) {
            ESP_LOGI(TAG, "UID leido: %02X %02X %02X %02X",
                     uid.bytes[0], uid.bytes[1], uid.bytes[2], uid.bytes[3]);

            xSemaphoreTake(mutex, portMAX_DELAY);

            if (g_estado == ESTADO_BLOQUEADO) {
                if (uid_es_valido(&uid)) {
                    accion_acceso_concedido();
                } else {
                    accion_acceso_denegado();
                }
            } else {
                if (uid_es_valido(&uid)) {
                    accion_cierre_sesion();
                }
            }

            xSemaphoreGive(mutex);
            rc522_halt();
        }

        tarjeta_antes = hay_tarjeta;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static void tarea_lcd(void *pvParam)
{
    ds1307_time_t t;
    char hora[17];

    while (1) {
        if (g_estado == ESTADO_ACTIVO) {
            xSemaphoreTake(mutex, portMAX_DELAY);

            lcd_set_cursor(0, 0);
            if (g_hay_mensaje) {
                lcd_linea(g_msg_ble);
            } else {
                lcd_linea("Sin mensajes    ");
            }

            ds1307_get_time(&t);
            snprintf(hora, sizeof(hora), "%02d:%02d:%02d", t.hora, t.min, t.seg);
            lcd_set_cursor(0, 1);
            lcd_linea(hora);

            xSemaphoreGive(mutex);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    gpio_config_t pines_salida;
    pines_salida.pin_bit_mask = (1ULL << PIN_LED_ROJO)  |
                                (1ULL << PIN_LED_VERDE) |
                                (1ULL << PIN_LED_AZUL)  |
                                (1ULL << PIN_BUZZER);
    pines_salida.mode = GPIO_MODE_OUTPUT;
    pines_salida.pull_up_en = GPIO_PULLUP_DISABLE;
    pines_salida.pull_down_en = GPIO_PULLDOWN_DISABLE;
    pines_salida.intr_type = GPIO_INTR_DISABLE;
    gpio_config(&pines_salida);

    leds_apagar();
    buzzer_off();

    // NVS requerido por el stack BLE
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        nvs_flash_erase();
        nvs_flash_init();
    }

    i2c_master_init();
    lcd_init();

    // si el RTC no tiene bateria arranca con esta hora
    ds1307_time_t hora_inicial;
    hora_inicial.seg  = 0;
    hora_inicial.min  = 0;
    hora_inicial.hora = 12;
    hora_inicial.dia  = 1;
    hora_inicial.mes  = 1;
    hora_inicial.anio = 25;

    ret = ds1307_init(&hora_inicial);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "RTC no responde: %s", esp_err_to_name(ret));
        lcd_set_cursor(0, 0);
        lcd_linea("Error RTC!      ");
    }

    ble_nus_init();
    rc522_init();

    pantalla_bloqueado();
    gpio_set_level(PIN_LED_ROJO, 1);

    mutex = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(tarea_rfid, "rfid", 4096, NULL, 5, NULL, 1);
    xTaskCreatePinnedToCore(tarea_lcd,  "lcd",  2048, NULL, 3, NULL, 1);
}
