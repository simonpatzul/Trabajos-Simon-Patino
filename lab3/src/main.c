#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/adc.h"
#include "driver/ledc.h"

#include "esp_timer.h"

// =====================================================
// SENSORES
// =====================================================

#define LM35_CHANNEL        ADC1_CHANNEL_6   // GPIO 34
#define LDR_CHANNEL         ADC1_CHANNEL_7   // GPIO 35

#define LDR_INVERTIDA       1
#define TEMP_INVERTIDA      1

#define ADC_MAX_RAW         4095.0f
#define ADC_MAX_MV          3300.0f

// =====================================================
// CALIBRACION FOTORESISTENCIA
// =====================================================

#define LDR_MIN_REAL_PERCENT   66.0f
#define LDR_MAX_REAL_PERCENT   100.0f

// =====================================================
// CALIBRACION TEMPERATURA
// =====================================================

// Ajustado porque en ambiente marcaba 9.4 C.
// Ambiente real aproximado: 20 C.
#define TEMP_SENSOR_AMBIENTE       315.0f
#define TEMP_REAL_AMBIENTE_C       20.0f

// Punto caliente estimado
#define TEMP_SENSOR_CALIENTE       100.0f
#define TEMP_REAL_CALIENTE_C       60.0f

#define TEMP_FILTRO_ALPHA          0.20f

// =====================================================
// MOTOR TB6612FNG
// =====================================================

#define AIN1 GPIO_NUM_19
#define AIN2 GPIO_NUM_18
#define PWMA GPIO_NUM_5

#define BIN1 GPIO_NUM_22
#define BIN2 GPIO_NUM_21
#define PWMB GPIO_NUM_4

#define STBY GPIO_NUM_23

#define INVERTIR_MOTOR 1
#define SECUENCIA 0

#define MOTOR_RAMP_STEP_SPS 20
#define MOTOR_RAMP_TIME_MS  50

// =====================================================
// ACTUADORES
// =====================================================

#define HEATER_GPIO GPIO_NUM_25

// GPIO25 = 3.3V prende
// GPIO25 = 0V apaga
#define HEATER_ON_LEVEL 1

#define LED_POWER_GPIO GPIO_NUM_26

// Luz salida 100% -> LED prende
// Luz salida 0%   -> LED apaga
#define LED_PWM_INVERTIDA 0

// =====================================================
// PWM LUZ DE POTENCIA
// =====================================================

#define LEDC_MODE       LEDC_LOW_SPEED_MODE
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_RES        LEDC_TIMER_10_BIT
#define LEDC_FREQ_HZ    1000
#define LEDC_MAX_DUTY   1023
// =====================================================
// VARIABLES GLOBALES
// =====================================================

typedef enum {
    MOTOR_STOP = 0,
    MOTOR_HORARIO = 1,
    MOTOR_ANTIHORARIO = -1
} motor_dir_t;

volatile float temp_control = 28.0f;

volatile int motor_target_sps = 0;
volatile motor_dir_t motor_target_dir = MOTOR_STOP;

volatile int motor_actual_sps = 0;
volatile motor_dir_t motor_actual_dir = MOTOR_STOP;

int step_index = 0;

esp_timer_handle_t motor_timer = NULL;
bool motor_timer_running = false;

float temperatura_filtrada = -1000.0f;

int secuencias[4][4][2] = {
    {
        { 1,  1},
        {-1,  1},
        {-1, -1},
        { 1, -1}
    },
    {
        { 1,  1},
        { 1, -1},
        {-1, -1},
        {-1,  1}
    },
    {
        { 1, -1},
        {-1, -1},
        {-1,  1},
        { 1,  1}
    },
    {
        {-1,  1},
        {-1, -1},
        { 1, -1},
        { 1,  1}
    }
};

// =====================================================
// PROTOTIPOS
// =====================================================

void configurar_gpio(void);
void configurar_adc(void);
void configurar_pwm_luz(void);
void configurar_timer_motor(void);

int leer_adc_filtrado(adc1_channel_t canal);
float leer_temperatura_celsius(void);
float leer_luz_porcentaje(void);

void set_luz_porcentaje(float porcentaje);
void set_heater(int estado);

void set_bobina_a(int estado);
void set_bobina_b(int estado);
void aplicar_paso(int a, int b);
void paso_adelante(void);
void paso_atras(void);
void motor_off(void);

void motor_timer_callback(void *arg);
void actualizar_timer_motor(int sps, motor_dir_t dir);

void tarea_control(void *arg);
void tarea_serial(void *arg);
void tarea_rampa_motor(void *arg);

void procesar_comando(char *linea);

// =====================================================
// APP MAIN
// =====================================================

void app_main(void)
{
    configurar_gpio();
    configurar_adc();
    configurar_pwm_luz();
    configurar_timer_motor();

    gpio_set_level(STBY, 1);
    gpio_set_level(PWMA, 1);
    gpio_set_level(PWMB, 1);

    set_heater(0);
    set_luz_porcentaje(0);

    printf("\n=====================================\n");
    printf("VERSION FINAL AJUSTADA TEMP 315\n");
    printf("LABORATORIO 3 - SISTEMA DOMOTICO ESP32\n");
    printf("Temperatura en GPIO34\n");
    printf("Fotoresistencia en GPIO35\n");
    printf("Lampara calefactora en GPIO25\n");
    printf("Luz de potencia PWM en GPIO26\n");
    printf("HEATER_ON_LEVEL: %d\n", HEATER_ON_LEVEL);
    printf("LED_PWM_INVERTIDA: %d\n", LED_PWM_INVERTIDA);
    printf("TEMP_SENSOR_AMBIENTE: %.1f\n", TEMP_SENSOR_AMBIENTE);
    printf("Comando serial: SET_TEMP:XX\n");
    printf("Temperatura de control inicial: %.1f C\n", temp_control);
    printf("=====================================\n\n");

    xTaskCreate(tarea_control, "tarea_control", 4096, NULL, 4, NULL);
    xTaskCreate(tarea_serial, "tarea_serial", 4096, NULL, 3, NULL);
    xTaskCreate(tarea_rampa_motor, "tarea_rampa_motor", 4096, NULL, 3, NULL);
}

// =====================================================
// CONFIGURACION
// =====================================================

void configurar_gpio(void)
{
    gpio_reset_pin(HEATER_GPIO);

    gpio_config_t io_conf = {
        .pin_bit_mask =
            (1ULL << AIN1) |
            (1ULL << AIN2) |
            (1ULL << PWMA) |
            (1ULL << BIN1) |
            (1ULL << BIN2) |
            (1ULL << PWMB) |
            (1ULL << STBY) |
            (1ULL << HEATER_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&io_conf);

    gpio_set_level(STBY, 0);
    gpio_set_level(PWMA, 0);
    gpio_set_level(PWMB, 0);

    gpio_set_level(HEATER_GPIO, 0);

    motor_off();
}

void configurar_adc(void)
{
    adc1_config_width(ADC_WIDTH_BIT_12);

    adc1_config_channel_atten(LM35_CHANNEL, ADC_ATTEN_DB_11);
    adc1_config_channel_atten(LDR_CHANNEL, ADC_ATTEN_DB_11);
}

void configurar_pwm_luz(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_RES,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ledc_timer_config(&timer);

    ledc_channel_config_t led_power = {
        .gpio_num = LED_POWER_GPIO,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };

    ledc_channel_config(&led_power);
}

void configurar_timer_motor(void)
{
    esp_timer_create_args_t timer_args = {
        .callback = &motor_timer_callback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "motor_step_timer"
    };

    esp_timer_create(&timer_args, &motor_timer);
}

// =====================================================
// LECTURA ADC FILTRADA
// =====================================================

int leer_adc_filtrado(adc1_channel_t canal)
{
    const int N = 25;
    int muestras[N];

    for (int i = 0; i < N; i++)
    {
        muestras[i] = adc1_get_raw(canal);
        vTaskDelay(1);
    }

    for (int i = 0; i < N - 1; i++)
    {
        for (int j = i + 1; j < N; j++)
        {
            if (muestras[j] < muestras[i])
            {
                int temp = muestras[i];
                muestras[i] = muestras[j];
                muestras[j] = temp;
            }
        }
    }

    int suma = 0;
    int contador = 0;

    for (int i = 5; i < 20; i++)
    {
        suma += muestras[i];
        contador++;
    }

    return suma / contador;
}

// =====================================================
// SENSOR TEMPERATURA CALIBRADO
// =====================================================

float leer_temperatura_celsius(void)
{
    int raw = leer_adc_filtrado(LM35_CHANNEL);

    float voltaje_mv = ((float)raw * ADC_MAX_MV) / ADC_MAX_RAW;

    float lectura_sensor;

#if TEMP_INVERTIDA
    lectura_sensor = (ADC_MAX_MV - voltaje_mv) / 10.0f;
#else
    lectura_sensor = voltaje_mv / 10.0f;
#endif

    float temperatura =
        TEMP_REAL_AMBIENTE_C +
        ((lectura_sensor - TEMP_SENSOR_AMBIENTE) *
        (TEMP_REAL_CALIENTE_C - TEMP_REAL_AMBIENTE_C)) /
        (TEMP_SENSOR_CALIENTE - TEMP_SENSOR_AMBIENTE);

    if (temperatura < 0.0f)
    {
        temperatura = 0.0f;
    }

    if (temperatura > 100.0f)
    {
        temperatura = 100.0f;
    }

    if (temperatura_filtrada < -100.0f)
    {
        temperatura_filtrada = temperatura;
    }
    else
    {
        temperatura_filtrada =
            (TEMP_FILTRO_ALPHA * temperatura) +
            ((1.0f - TEMP_FILTRO_ALPHA) * temperatura_filtrada);
    }

    printf("DEBUG TEMP -> RAW:%d | V:%.1f mV | Sensor:%.1f | T:%.1f C\n",
           raw,
           voltaje_mv,
           lectura_sensor,
           temperatura_filtrada);

    return temperatura_filtrada;
}

// =====================================================
// FOTORESISTENCIA CALIBRADA
// =====================================================

float leer_luz_porcentaje(void)
{
    int raw = leer_adc_filtrado(LDR_CHANNEL);

    float porcentaje = ((float)raw * 100.0f) / ADC_MAX_RAW;

#if LDR_INVERTIDA
    porcentaje = 100.0f - porcentaje;
#endif

    porcentaje =
        ((porcentaje - LDR_MIN_REAL_PERCENT) * 100.0f) /
        (LDR_MAX_REAL_PERCENT - LDR_MIN_REAL_PERCENT);

    if (porcentaje < 0.0f)
    {
        porcentaje = 0.0f;
    }

    if (porcentaje > 100.0f)
    {
        porcentaje = 100.0f;
    }

    return porcentaje;
}

// =====================================================
// LUZ Y CALEFACCION
// =====================================================

void set_luz_porcentaje(float porcentaje)
{
    if (porcentaje < 0.0f)
    {
        porcentaje = 0.0f;
    }

    if (porcentaje > 100.0f)
    {
        porcentaje = 100.0f;
    }

#if LED_PWM_INVERTIDA
    float porcentaje_pwm = 100.0f - porcentaje;
#else
    float porcentaje_pwm = porcentaje;
#endif

    uint32_t duty = (uint32_t)((porcentaje_pwm * LEDC_MAX_DUTY) / 100.0f);

    ledc_set_duty(LEDC_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CHANNEL_0);
}

void set_heater(int estado)
{
    if (estado)
    {
        gpio_set_level(HEATER_GPIO, 1);
    }
    else
    {
        gpio_set_level(HEATER_GPIO, 0);
    }
}

// =====================================================
// MOTOR TB6612FNG
// =====================================================

void set_bobina_a(int estado)
{
    if (estado == 1)
    {
        gpio_set_level(AIN1, 1);
        gpio_set_level(AIN2, 0);
    }
    else if (estado == -1)
    {
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 1);
    }
    else
    {
        gpio_set_level(AIN1, 0);
        gpio_set_level(AIN2, 0);
    }
}

void set_bobina_b(int estado)
{
    if (estado == 1)
    {
        gpio_set_level(BIN1, 1);
        gpio_set_level(BIN2, 0);
    }
    else if (estado == -1)
    {
        gpio_set_level(BIN1, 0);
        gpio_set_level(BIN2, 1);
    }
    else
    {
        gpio_set_level(BIN1, 0);
        gpio_set_level(BIN2, 0);
    }
}

void aplicar_paso(int a, int b)
{
    set_bobina_a(a);
    set_bobina_b(b);
}

void paso_adelante(void)
{
    int a = secuencias[SECUENCIA][step_index][0];
    int b = secuencias[SECUENCIA][step_index][1];

    aplicar_paso(a, b);

    step_index++;

    if (step_index >= 4)
    {
        step_index = 0;
    }
}

void paso_atras(void)
{
    int a = secuencias[SECUENCIA][step_index][0];
    int b = secuencias[SECUENCIA][step_index][1];

    aplicar_paso(a, b);

    step_index--;

    if (step_index < 0)
    {
        step_index = 3;
    }
}

void motor_off(void)
{
    set_bobina_a(0);
    set_bobina_b(0);
}

// =====================================================
// TIMER MOTOR
// =====================================================

void motor_timer_callback(void *arg)
{
    motor_dir_t dir = motor_actual_dir;

    if (motor_actual_sps <= 0 || dir == MOTOR_STOP)
    {
        motor_off();
        return;
    }

    int usar_adelante;

    if (dir == MOTOR_HORARIO)
    {
        usar_adelante = 1;
    }
    else
    {
        usar_adelante = 0;
    }

#if INVERTIR_MOTOR
    usar_adelante = !usar_adelante;
#endif

    if (usar_adelante)
    {
        paso_adelante();
    }
    else
    {
        paso_atras();
    }
}

void actualizar_timer_motor(int sps, motor_dir_t dir)
{
    if (motor_timer_running)
    {
        esp_timer_stop(motor_timer);
        motor_timer_running = false;
    }

    motor_actual_sps = sps;
    motor_actual_dir = dir;

    if (sps <= 0 || dir == MOTOR_STOP)
    {
        motor_off();
        return;
    }

    int periodo_us = 1000000 / sps;

    esp_timer_start_periodic(motor_timer, periodo_us);
    motor_timer_running = true;
}

// =====================================================
// RAMPA MOTOR
// =====================================================

void tarea_rampa_motor(void *arg)
{
    int sps_actual_local = 0;
    motor_dir_t dir_actual_local = MOTOR_STOP;

    int sps_timer_anterior = -1;
    motor_dir_t dir_timer_anterior = MOTOR_STOP;

    while (1)
    {
        int sps_objetivo = motor_target_sps;
        motor_dir_t dir_objetivo = motor_target_dir;

        if (sps_objetivo <= 0 || dir_objetivo == MOTOR_STOP)
        {
            sps_actual_local = 0;
            dir_actual_local = MOTOR_STOP;
        }
        else
        {
            if (dir_actual_local != dir_objetivo)
            {
                dir_actual_local = dir_objetivo;
                sps_actual_local = 100;
            }

            if (sps_actual_local < sps_objetivo)
            {
                sps_actual_local += MOTOR_RAMP_STEP_SPS;

                if (sps_actual_local > sps_objetivo)
                {
                    sps_actual_local = sps_objetivo;
                }
            }
            else if (sps_actual_local > sps_objetivo)
            {
                sps_actual_local -= MOTOR_RAMP_STEP_SPS;

                if (sps_actual_local < sps_objetivo)
                {
                    sps_actual_local = sps_objetivo;
                }
            }
        }

        if (sps_actual_local != sps_timer_anterior || dir_actual_local != dir_timer_anterior)
        {
            actualizar_timer_motor(sps_actual_local, dir_actual_local);

            sps_timer_anterior = sps_actual_local;
            dir_timer_anterior = dir_actual_local;
        }

        vTaskDelay(pdMS_TO_TICKS(MOTOR_RAMP_TIME_MS));
    }
}

// =====================================================
// CONTROL PRINCIPAL
// =====================================================

void tarea_control(void *arg)
{
    while (1)
    {
        float temperatura = leer_temperatura_celsius();
        float luz_ambiente = leer_luz_porcentaje();
        float tc = temp_control;

        int calefaccion = 0;
        int pasos_s = 0;
        motor_dir_t direccion = MOTOR_STOP;

        const char *modo = "ESTABLE";

        if (temperatura < (tc - 1.0f))
        {
            calefaccion = 1;
            pasos_s = 100;
            direccion = MOTOR_HORARIO;
            modo = "CALEFACCION";
        }
        else if (temperatura <= (tc + 1.0f))
        {
            calefaccion = 0;
            pasos_s = 0;
            direccion = MOTOR_STOP;
            modo = "ZONA MUERTA";
        }
        else if (temperatura < (tc + 3.0f))
        {
            calefaccion = 0;
            pasos_s = 100;
            direccion = MOTOR_ANTIHORARIO;
            modo = "VENTILACION BAJA";
        }
        else if (temperatura <= (tc + 5.0f))
        {
            calefaccion = 0;
            pasos_s = 300;
            direccion = MOTOR_ANTIHORARIO;
            modo = "VENTILACION MEDIA";
        }
        else
        {
            calefaccion = 0;
            pasos_s = 600;
            direccion = MOTOR_ANTIHORARIO;
            modo = "VENTILACION ALTA";
        }

        set_heater(calefaccion);

        motor_target_sps = pasos_s;
        motor_target_dir = direccion;

        float luz_salida = 0.0f;

        if (luz_ambiente < 20.0f)
        {
            luz_salida = 100.0f;
        }
        else if (luz_ambiente < 30.0f)
        {
            luz_salida = 80.0f;
        }
        else if (luz_ambiente < 40.0f)
        {
            luz_salida = 60.0f;
        }
        else if (luz_ambiente < 60.0f)
        {
            luz_salida = 50.0f;
        }
        else if (luz_ambiente < 80.0f)
        {
            luz_salida = 30.0f;
        }
        else
        {
            luz_salida = 0.0f;
        }

        set_luz_porcentaje(luz_salida);

        printf("Tc: %.1f C | T: %.1f C | Luz amb: %.1f %% | Luz salida: %.0f %% | Calefactor: %s | Motor objetivo: %d steps/s | Motor actual: %d steps/s | Modo: %s\n",
               tc,
               temperatura,
               luz_ambiente,
               luz_salida,
               calefaccion ? "ON" : "OFF",
               pasos_s,
               motor_actual_sps,
               modo);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

// =====================================================
// SERIAL
// =====================================================

void tarea_serial(void *arg)
{
    char linea[64];

    while (1)
    {
        if (fgets(linea, sizeof(linea), stdin) != NULL)
        {
            linea[strcspn(linea, "\r\n")] = 0;
            procesar_comando(linea);
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void procesar_comando(char *linea)
{
    if (strncmp(linea, "SET_TEMP:", 9) == 0)
    {
        float nueva_temp = atof(&linea[9]);

        if (nueva_temp > 0.0f && nueva_temp < 80.0f)
        {
            temp_control = nueva_temp;
            printf("Nueva temperatura de control: %.1f C\n", temp_control);
        }
        else
        {
            printf("Valor invalido. Usa por ejemplo: SET_TEMP:28\n");
        }
    }
    else
    {
        printf("Comando no reconocido. Usa: SET_TEMP:XX\n");
    }
}