#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

// =====================================================
// DISPLAY 7 SEGMENTOS
// =====================================================
#define SEG_A 13
#define SEG_B 18
#define SEG_C 14
#define SEG_D 27
#define SEG_E 26
#define SEG_F 25
#define SEG_G 33

#define COM1 19
#define COM2 21
#define COM3 22

// =====================================================
// BOTONES Y LEDS (PINES RECOMENDADOS)
// BOTONES EN GPIO35 Y GPIO39 CON PULL-UP EXTERNO 10k
// =====================================================
#define BTN_IZQ   35
#define BTN_DER   39

#define LED_ROJO  4
#define LED_VERDE 5

// =====================================================
// PUENTE H CON TRANSISTORES P / N
// S1 y S2 = lado alto (P) -> activo en BAJO
// S3 y S4 = lado bajo (N) -> activo en ALTO
// =====================================================
#define MOTOR_S1 32
#define MOTOR_S2 23
#define MOTOR_S3 16
#define MOTOR_S4 17

// =====================================================
// POTENCIOMETRO
// =====================================================
#define POT_ADC_CH ADC_CHANNEL_6   // GPIO34

// =====================================================
// PWM
// =====================================================
#define PWM_FREQ_HZ       500
#define PWM_RESOLUTION    LEDC_TIMER_12_BIT
#define PWM_MAX_DUTY      4095
#define PWM_MODE          LEDC_LOW_SPEED_MODE
#define PWM_TIMER         LEDC_TIMER_0
#define PWM_CH_S3         LEDC_CHANNEL_0
#define PWM_CH_S4         LEDC_CHANNEL_1

// =====================================================
// AJUSTES
// =====================================================
#define SAMPLE_PERIOD_US      20000
#define DEBOUNCE_US           180000
#define DEADTIME_MS           5
#define LOOP_DELAY_MS         1
#define RAMP_STEP_UP          80
#define RAMP_STEP_DOWN        120

// Display
#define SEG_ON   1
#define SEG_OFF  0
#define COM_ON   0
#define COM_OFF  1

// Puente H
#define HS_ON    0
#define HS_OFF   1
#define LS_ON    1
#define LS_OFF   0

typedef enum {
    DIR_STOP = 0,
    DIR_IZQ,
    DIR_DER
} direccion_t;

// =====================================================
// GLOBALES
// =====================================================
static adc_oneshot_unit_handle_t adc_handle;

static volatile int disp1 = 0;
static volatile int disp2 = 0;
static volatile int disp3 = 0;

static volatile int64_t anti_izq = 0;
static volatile int64_t anti_der = 0;
static volatile direccion_t requested_dir = DIR_STOP;

static direccion_t active_dir = DIR_STOP;
static uint32_t current_duty = 0;
static uint32_t target_duty = 0;

static const uint8_t digitos[10] = {
    0b00111111, // 0
    0b00000110, // 1
    0b01011011, // 2
    0b01001111, // 3
    0b01100110, // 4
    0b01101101, // 5
    0b01111101, // 6
    0b00000111, // 7
    0b01111111, // 8
    0b01101111  // 9
};

// =====================================================
// DISPLAY
// =====================================================
static void all_digits_off(void)
{
    gpio_set_level(COM1, COM_OFF);
    gpio_set_level(COM2, COM_OFF);
    gpio_set_level(COM3, COM_OFF);
}

static void set_segments(uint8_t pattern)
{
    gpio_set_level(SEG_A, (pattern & (1 << 0)) ? SEG_ON : SEG_OFF);
    gpio_set_level(SEG_B, (pattern & (1 << 1)) ? SEG_ON : SEG_OFF);
    gpio_set_level(SEG_C, (pattern & (1 << 2)) ? SEG_ON : SEG_OFF);
    gpio_set_level(SEG_D, (pattern & (1 << 3)) ? SEG_ON : SEG_OFF);
    gpio_set_level(SEG_E, (pattern & (1 << 4)) ? SEG_ON : SEG_OFF);
    gpio_set_level(SEG_F, (pattern & (1 << 5)) ? SEG_ON : SEG_OFF);
    gpio_set_level(SEG_G, (pattern & (1 << 6)) ? SEG_ON : SEG_OFF);
}

static void update_display_values(int porcentaje)
{
    if (porcentaje < 0) porcentaje = 0;
    if (porcentaje > 100) porcentaje = 100;

    disp1 = porcentaje / 100;
    disp2 = (porcentaje / 10) % 10;
    disp3 = porcentaje % 10;
}

static void multiplex_display_step(void)
{
    static int idx = 0;

    all_digits_off();

    switch (idx) {
        case 0:
            set_segments(digitos[disp1]);
            gpio_set_level(COM1, COM_ON);
            break;
        case 1:
            set_segments(digitos[disp2]);
            gpio_set_level(COM2, COM_ON);
            break;
        default:
            set_segments(digitos[disp3]);
            gpio_set_level(COM3, COM_ON);
            break;
    }

    idx++;
    if (idx >= 3) idx = 0;
}

// =====================================================
// PWM / PUENTE H
// =====================================================
static void pwm_set_s3(uint32_t duty)
{
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
    ledc_set_duty(PWM_MODE, PWM_CH_S3, duty);
    ledc_update_duty(PWM_MODE, PWM_CH_S3);
}

static void pwm_set_s4(uint32_t duty)
{
    if (duty > PWM_MAX_DUTY) duty = PWM_MAX_DUTY;
    ledc_set_duty(PWM_MODE, PWM_CH_S4, duty);
    ledc_update_duty(PWM_MODE, PWM_CH_S4);
}

static void bridge_all_off(void)
{
    gpio_set_level(MOTOR_S1, HS_OFF);
    gpio_set_level(MOTOR_S2, HS_OFF);
    pwm_set_s3(0);
    pwm_set_s4(0);
}

static void update_leds(direccion_t dir)
{
    switch (dir) {
        case DIR_IZQ:
            gpio_set_level(LED_ROJO, 1);
            gpio_set_level(LED_VERDE, 0);
            break;

        case DIR_DER:
            gpio_set_level(LED_ROJO, 0);
            gpio_set_level(LED_VERDE, 1);
            break;

        default:
            gpio_set_level(LED_ROJO, 0);
            gpio_set_level(LED_VERDE, 0);
            break;
    }
}

static void apply_bridge(direccion_t dir, uint32_t duty)
{
    if (dir == DIR_STOP || duty == 0) {
        bridge_all_off();
        update_leds(DIR_STOP);
        return;
    }

    switch (dir) {
        case DIR_IZQ:
            // diagonal S1 + S4(PWM)
            gpio_set_level(MOTOR_S1, HS_ON);
            gpio_set_level(MOTOR_S2, HS_OFF);
            pwm_set_s3(0);
            pwm_set_s4(duty);
            update_leds(DIR_IZQ);
            break;

        case DIR_DER:
            // diagonal S2 + S3(PWM)
            gpio_set_level(MOTOR_S1, HS_OFF);
            gpio_set_level(MOTOR_S2, HS_ON);
            pwm_set_s4(0);
            pwm_set_s3(duty);
            update_leds(DIR_DER);
            break;

        default:
            bridge_all_off();
            update_leds(DIR_STOP);
            break;
    }
}

static uint32_t ramp_towards(uint32_t current, uint32_t target)
{
    if (current < target) {
        uint32_t diff = target - current;
        current += (diff > RAMP_STEP_UP) ? RAMP_STEP_UP : diff;
    } else if (current > target) {
        uint32_t diff = current - target;
        current -= (diff > RAMP_STEP_DOWN) ? RAMP_STEP_DOWN : diff;
    }
    return current;
}

// =====================================================
// ADC
// =====================================================
static void init_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, POT_ADC_CH, &ch_cfg));
}

// =====================================================
// ISR BOTONES
// =====================================================
static void IRAM_ATTR isr_izq(void *arg)
{
    int64_t now = esp_timer_get_time();
    if (now - anti_izq > DEBOUNCE_US) {
        requested_dir = DIR_IZQ;
        anti_izq = now;
    }
}

static void IRAM_ATTR isr_der(void *arg)
{
    int64_t now = esp_timer_get_time();
    if (now - anti_der > DEBOUNCE_US) {
        requested_dir = DIR_DER;
        anti_der = now;
    }
}

// =====================================================
// INIT
// =====================================================
static void init_outputs(void)
{
    gpio_config_t out_cfg = {
        .pin_bit_mask =
            (1ULL << SEG_A) |
            (1ULL << SEG_B) |
            (1ULL << SEG_C) |
            (1ULL << SEG_D) |
            (1ULL << SEG_E) |
            (1ULL << SEG_F) |
            (1ULL << SEG_G) |
            (1ULL << COM1) |
            (1ULL << COM2) |
            (1ULL << COM3) |
            (1ULL << LED_VERDE) |
            (1ULL << LED_ROJO) |
            (1ULL << MOTOR_S1) |
            (1ULL << MOTOR_S2),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&out_cfg);

    all_digits_off();
    set_segments(0);
    gpio_set_level(LED_VERDE, 0);
    gpio_set_level(LED_ROJO, 0);

    gpio_set_level(MOTOR_S1, HS_OFF);
    gpio_set_level(MOTOR_S2, HS_OFF);
}

static void init_buttons(void)
{
    gpio_config_t in_cfg = {
        .pin_bit_mask = (1ULL << BTN_IZQ) | (1ULL << BTN_DER),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,   // GPIO35 y 39 no tienen pull-up interno
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    gpio_config(&in_cfg);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(BTN_IZQ, isr_izq, NULL);
    gpio_isr_handler_add(BTN_DER, isr_der, NULL);
}

static void init_pwm(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = PWM_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&timer_cfg);

    ledc_channel_config_t ch_s3 = {
        .gpio_num = MOTOR_S3,
        .speed_mode = PWM_MODE,
        .channel = PWM_CH_S3,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch_s3);

    ledc_channel_config_t ch_s4 = {
        .gpio_num = MOTOR_S4,
        .speed_mode = PWM_MODE,
        .channel = PWM_CH_S4,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ch_s4);

    pwm_set_s3(0);
    pwm_set_s4(0);
}

// =====================================================
// MAIN
// =====================================================
void app_main(void)
{
    init_outputs();
    init_buttons();
    init_adc();
    init_pwm();

    requested_dir = DIR_STOP;
    active_dir = DIR_STOP;
    current_duty = 0;
    target_duty = 0;
    update_display_values(0);

    int adc_raw = 0;
    int porcentaje = 0;
    int64_t last_sample = 0;

    printf("Sistema iniciado\n");
    printf("PWM = %d Hz\n", PWM_FREQ_HZ);

    while (1) {
        int64_t now = esp_timer_get_time();

        multiplex_display_step();

        if ((now - last_sample) >= SAMPLE_PERIOD_US) {
            adc_oneshot_read(adc_handle, POT_ADC_CH, &adc_raw);

            porcentaje = 100 - ((adc_raw * 100) / 4095);
            if (porcentaje < 0) porcentaje = 0;
            if (porcentaje > 100) porcentaje = 100;

            update_display_values(porcentaje);
            target_duty = (porcentaje * PWM_MAX_DUTY) / 100;

            last_sample = now;
        }

        if (requested_dir != active_dir) {
            if (current_duty > 0) {
                current_duty = ramp_towards(current_duty, 0);
                apply_bridge(active_dir, current_duty);
            } else {
                bridge_all_off();
                vTaskDelay(pdMS_TO_TICKS(DEADTIME_MS));
                active_dir = requested_dir;
            }
        } else {
            if (active_dir == DIR_STOP) {
                current_duty = ramp_towards(current_duty, 0);
            } else {
                current_duty = ramp_towards(current_duty, target_duty);
            }
            apply_bridge(active_dir, current_duty);
        }

        vTaskDelay(pdMS_TO_TICKS(LOOP_DELAY_MS));
    }
}