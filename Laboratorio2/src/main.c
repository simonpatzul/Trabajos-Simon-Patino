#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_random.h"

// ==================== PINES ====================
static const int ROW_PINS[6]   = {33, 32, 23, 22, 21,  0};
static const int COL_V_PINS[6] = {13, 14, 26, 19,  5, 16};
static const int COL_R_PINS[6] = {12, 27, 25, 18, 17,  4};

#define BTN_LEFT   2    // SALTAR
#define BTN_RIGHT 15    // INICIAR / REINICIAR

// ==================== TAMAÑO DISPLAY ====================
#define GRID_ROWS 6
#define GRID_COLS 6

// ==================== POLARIDAD ====================
// ESTA VERSION ASUME:
// - FILAS activas en HIGH
// - COLUMNAS activas en LOW
#define ROW_ON   1
#define ROW_OFF  0

#define COL_ON   0
#define COL_OFF  1

// ==================== TIMINGS ====================
#define DISPLAY_PERIOD_US 2000
#define GAME_TICK_MS      110
#define DEBOUNCE_MS        50

// ==================== JUEGO ====================
#define DINO_COL        1
#define GROUND_ROW      5
#define MAX_OBSTACLES   3

typedef enum {
    STATE_WAIT_START = 0,
    STATE_RUNNING,
    STATE_GAME_OVER
} GameState;

typedef struct {
    bool active;
    int  col;
    int  height;   // 1 o 2 píxeles
} Obstacle;

typedef struct {
    int pin;
    bool last_stable;
    bool current;
    TickType_t last_change;
} Button;

// ==================== GLOBALES ====================
// framebuffer:
// 0 = apagado
// 1 = verde
// 2 = rojo
// 3 = naranja (verde + rojo)
static volatile uint8_t g_fb[GRID_ROWS][GRID_COLS];
static volatile int disp_row = 0;

static GameState g_state = STATE_WAIT_START;
static Obstacle  g_obs[MAX_OBSTACLES];

static Button btn_left  = {BTN_LEFT,  true, true, 0};
static Button btn_right = {BTN_RIGHT, true, true, 0};

static int  g_score = 0;
static int  g_dino_bottom_row = GROUND_ROW;
static bool g_jumping = false;
static int  g_jump_idx = 0;
static bool g_blink = false;

// Secuencia del salto (fila inferior del dinosaurio)
static const int JUMP_SEQ[] = {5, 4, 3, 2, 3, 4, 5};
#define JUMP_SEQ_LEN (sizeof(JUMP_SEQ) / sizeof(JUMP_SEQ[0]))

// ==================== FRAMEBUFFER ====================
static inline void fb_clear(void) {
    memset((void*)g_fb, 0, sizeof(g_fb));
}

static inline void fb_set(int r, int c, uint8_t color) {
    if (r >= 0 && r < GRID_ROWS && c >= 0 && c < GRID_COLS) {
        g_fb[r][c] = color;
    }
}

// ==================== DISPLAY ISR ====================
static void IRAM_ATTR display_isr(void *arg) {
    int prev = (disp_row + GRID_ROWS - 1) % GRID_ROWS;

    // Apaga fila anterior
    gpio_set_level(ROW_PINS[prev], ROW_OFF);

    // Apaga todas las columnas
    for (int c = 0; c < GRID_COLS; c++) {
        gpio_set_level(COL_V_PINS[c], COL_OFF);
        gpio_set_level(COL_R_PINS[c], COL_OFF);
    }

    int r = disp_row;

    // Carga columnas según framebuffer
    for (int c = 0; c < GRID_COLS; c++) {
        uint8_t pix = g_fb[r][c];

        if (pix == 1 || pix == 3) {
            gpio_set_level(COL_V_PINS[c], COL_ON);
        }
        if (pix == 2 || pix == 3) {
            gpio_set_level(COL_R_PINS[c], COL_ON);
        }
    }

    // Enciende la fila actual
    gpio_set_level(ROW_PINS[r], ROW_ON);

    disp_row = (disp_row + 1) % GRID_ROWS;
}

// ==================== GPIO INIT ====================
static void gpio_init_all(void) {
    for (int i = 0; i < GRID_ROWS; i++) {
        gpio_reset_pin(ROW_PINS[i]);
        gpio_set_direction(ROW_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_level(ROW_PINS[i], ROW_OFF);
    }

    for (int i = 0; i < GRID_COLS; i++) {
        gpio_reset_pin(COL_V_PINS[i]);
        gpio_set_direction(COL_V_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_level(COL_V_PINS[i], COL_OFF);

        gpio_reset_pin(COL_R_PINS[i]);
        gpio_set_direction(COL_R_PINS[i], GPIO_MODE_OUTPUT);
        gpio_set_level(COL_R_PINS[i], COL_OFF);
    }

    gpio_reset_pin(BTN_LEFT);
    gpio_set_direction(BTN_LEFT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN_LEFT, GPIO_PULLUP_ONLY);

    gpio_reset_pin(BTN_RIGHT);
    gpio_set_direction(BTN_RIGHT, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN_RIGHT, GPIO_PULLUP_ONLY);
}

// ==================== BOTONES ====================
static bool btn_pressed(Button *b) {
    bool raw = gpio_get_level(b->pin);
    TickType_t now = xTaskGetTickCount();

    if (raw != b->current) {
        b->current = raw;
        b->last_change = now;
    }

    if ((now - b->last_change) >= pdMS_TO_TICKS(DEBOUNCE_MS)) {
        if (b->last_stable != b->current) {
            b->last_stable = b->current;
            if (!b->last_stable) { // activo en LOW
                return true;
            }
        }
    }

    return false;
}

// ==================== LOGICA DEL JUEGO ====================
static void clear_obstacles(void) {
    memset(g_obs, 0, sizeof(g_obs));
}

static void reset_run(void) {
    g_score = 0;
    g_dino_bottom_row = GROUND_ROW;
    g_jumping = false;
    g_jump_idx = 0;
    clear_obstacles();
    g_state = STATE_RUNNING;
}

static int obstacle_speed_ms(void) {
    int v = 420 - (g_score / 3) * 25;
    if (v < 140) v = 140;
    return v;
}

static int spawn_delay_ms(void) {
    int base = 1200 - (g_score / 2) * 35;
    if (base < 450) base = 450;
    return base + (esp_random() % 350);
}

static void spawn_obstacle(void) {
    // Evita que nazcan demasiado pegados
    for (int i = 0; i < MAX_OBSTACLES; i++) {
        if (g_obs[i].active && g_obs[i].col >= GRID_COLS - 2) {
            return;
        }
    }

    for (int i = 0; i < MAX_OBSTACLES; i++) {
        if (!g_obs[i].active) {
            g_obs[i].active = true;
            g_obs[i].col = GRID_COLS - 1;
            g_obs[i].height = ((esp_random() % 100) < 35) ? 2 : 1;
            return;
        }
    }
}

static void update_jump(void) {
    if (!g_jumping) {
        g_dino_bottom_row = GROUND_ROW;
        return;
    }

    g_dino_bottom_row = JUMP_SEQ[g_jump_idx];
    g_jump_idx++;

    if (g_jump_idx >= JUMP_SEQ_LEN) {
        g_jump_idx = 0;
        g_jumping = false;
        g_dino_bottom_row = GROUND_ROW;
    }
}

static void move_obstacles(void) {
    for (int i = 0; i < MAX_OBSTACLES; i++) {
        if (!g_obs[i].active) continue;

        g_obs[i].col--;

        if (g_obs[i].col < 0) {
            g_obs[i].active = false;
            g_score++;
        }
    }
}

static bool check_collision(void) {
    int dino_r1 = g_dino_bottom_row;
    int dino_r2 = g_dino_bottom_row - 1;

    for (int i = 0; i < MAX_OBSTACLES; i++) {
        if (!g_obs[i].active) continue;
        if (g_obs[i].col != DINO_COL) continue;

        for (int h = 0; h < g_obs[i].height; h++) {
            int obs_row = GROUND_ROW - h;
            if (obs_row == dino_r1 || obs_row == dino_r2) {
                return true;
            }
        }
    }

    return false;
}

// ==================== RENDER ====================
static void render_wait_start(void) {
    fb_clear();

    // Dino
    fb_set(4, DINO_COL, 3);
    fb_set(5, DINO_COL, 3);

    // Obstáculo de muestra
    fb_set(5, 4, 2);
    fb_set(4, 4, 2);

    // Indicador de inicio
    if (g_blink) {
        fb_set(1, 2, 1);
        fb_set(1, 3, 1);
    }
}

static void render_game_over(void) {
    fb_clear();

    // X roja
    fb_set(1, 1, 2);
    fb_set(2, 2, 2);
    fb_set(3, 3, 2);
    fb_set(1, 3, 2);
    fb_set(3, 1, 2);

    // Puntaje en la última fila (máximo 6)
    int dots = g_score;
    if (dots > 6) dots = 6;

    for (int i = 0; i < dots; i++) {
        fb_set(5, i, 1);
    }
}

static void render_running(void) {
    fb_clear();

    // Puntaje mod 6 en la fila superior
    int dots = g_score % 6;
    for (int i = 0; i < dots; i++) {
        fb_set(0, i, 1);
    }

    // Dino
    fb_set(g_dino_bottom_row,     DINO_COL, 3);
    fb_set(g_dino_bottom_row - 1, DINO_COL, 3);

    // Obstáculos
    for (int i = 0; i < MAX_OBSTACLES; i++) {
        if (!g_obs[i].active) continue;

        fb_set(GROUND_ROW, g_obs[i].col, 2);
        if (g_obs[i].height >= 2) {
            fb_set(GROUND_ROW - 1, g_obs[i].col, 2);
        }
    }
}

// ==================== TASK DISPLAY ====================
static void display_task(void *pvParam) {
    esp_timer_handle_t timer;

    const esp_timer_create_args_t args = {
        .callback = &display_isr,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "disp"
    };

    esp_timer_create(&args, &timer);
    esp_timer_start_periodic(timer, DISPLAY_PERIOD_US);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ==================== TASK GAME ====================
static void game_task(void *pvParam) {
    TickType_t last_logic = xTaskGetTickCount();
    TickType_t last_move  = xTaskGetTickCount();
    TickType_t next_spawn = xTaskGetTickCount() + pdMS_TO_TICKS(1000);
    TickType_t last_blink = xTaskGetTickCount();

    while (1) {
        TickType_t now = xTaskGetTickCount();

        bool jump_press  = btn_pressed(&btn_left);
        bool start_press = btn_pressed(&btn_right);

        // Parpadeo
        if ((now - last_blink) >= pdMS_TO_TICKS(300)) {
            g_blink = !g_blink;
            last_blink = now;
        }

        if (g_state == STATE_WAIT_START) {
            if (jump_press || start_press) {
                reset_run();
                last_logic = now;
                last_move  = now;
                next_spawn = now + pdMS_TO_TICKS(800);
            }

            render_wait_start();
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        if (g_state == STATE_GAME_OVER) {
            if (jump_press || start_press) {
                reset_run();
                last_logic = now;
                last_move  = now;
                next_spawn = now + pdMS_TO_TICKS(800);
            }

            render_game_over();
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        // RUNNING
        if (jump_press && !g_jumping) {
            g_jumping = true;
            g_jump_idx = 0;
        }

        if ((now - last_logic) >= pdMS_TO_TICKS(GAME_TICK_MS)) {
            update_jump();
            last_logic = now;
        }

        if ((now - last_move) >= pdMS_TO_TICKS(obstacle_speed_ms())) {
            move_obstacles();
            last_move = now;

            if (check_collision()) {
                g_state = STATE_GAME_OVER;
                continue;
            }
        }

        if (now >= next_spawn) {
            spawn_obstacle();
            next_spawn = now + pdMS_TO_TICKS(spawn_delay_ms());
        }

        if (check_collision()) {
            g_state = STATE_GAME_OVER;
            continue;
        }

        render_running();
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ==================== MAIN ====================
void app_main(void) {
    gpio_init_all();
    fb_clear();

    xTaskCreatePinnedToCore(display_task, "display_task", 2048, NULL, 10, NULL, 1);
    xTaskCreatePinnedToCore(game_task,    "game_task",    4096, NULL,  5, NULL, 0);
}