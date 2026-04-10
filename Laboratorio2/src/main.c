#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_random.h"

static const int ROW_PINS[6]   = {33, 32, 23, 22, 21, 0};
static const int COL_V_PINS[6] = {13, 14, 26, 19, 5, 16};
static const int COL_R_PINS[6] = {12, 27, 25, 18, 17, 4};

#define BTN_LEFT  2
#define BTN_RIGHT 15

#define GRID_ROWS 6
#define GRID_COLS 6

#define DINO_COL 1
#define GROUND_ROW 0

#define DISPLAY_PERIOD_US 2000

static uint8_t g_fb[GRID_ROWS][GRID_COLS];

static void fb_clear(void){ memset(g_fb,0,sizeof(g_fb)); }

static void fb_set(int r,int c,uint8_t color){
    if(r>=0&&r<GRID_ROWS&&c>=0&&c<GRID_COLS){
        int nr=c;
        int nc=GRID_ROWS-1-r;
        g_fb[nr][nc]=color;
    }
}

static volatile int disp_row=0;

static void IRAM_ATTR display_isr(void *arg){
    int prev=(disp_row+GRID_ROWS-1)%GRID_ROWS;
    gpio_set_level(ROW_PINS[prev],0);

    for(int c=0;c<GRID_COLS;c++){
        gpio_set_level(COL_V_PINS[c],0);
        gpio_set_level(COL_R_PINS[c],0);
    }

    int r=disp_row;
    for(int c=0;c<GRID_COLS;c++){
        uint8_t p=g_fb[r][c];
        if(p==1) gpio_set_level(COL_V_PINS[c],1);
        if(p==2) gpio_set_level(COL_R_PINS[c],1);
    }

    gpio_set_level(ROW_PINS[r],1);
    disp_row=(disp_row+1)%GRID_ROWS;
}

static void gpio_init_all(void){
    for(int i=0;i<GRID_ROWS;i++){
        gpio_set_direction(ROW_PINS[i],GPIO_MODE_OUTPUT);
        gpio_set_level(ROW_PINS[i],0);
    }

    for(int i=0;i<GRID_COLS;i++){
        gpio_set_direction(COL_V_PINS[i],GPIO_MODE_OUTPUT);
        gpio_set_direction(COL_R_PINS[i],GPIO_MODE_OUTPUT);
    }

    gpio_set_direction(BTN_LEFT,GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN_LEFT,GPIO_PULLUP_ONLY);

    gpio_set_direction(BTN_RIGHT,GPIO_MODE_INPUT);
    gpio_set_pull_mode(BTN_RIGHT,GPIO_PULLUP_ONLY);
}

#define DEBOUNCE_MS 50

typedef struct{
    int pin;
    bool last_stable;
    bool current;
    TickType_t last_change;
}Button;

static Button btn_left={BTN_LEFT,true,true,0};
static Button btn_right={BTN_RIGHT,true,true,0};

static bool btn_pressed(Button *b){
    bool raw=gpio_get_level(b->pin);
    TickType_t now=xTaskGetTickCount();

    if(raw!=b->current){
        b->current=raw;
        b->last_change=now;
    }

    if((now-b->last_change)>=pdMS_TO_TICKS(DEBOUNCE_MS)){
        if(b->last_stable!=b->current){
            b->last_stable=b->current;
            if(!b->last_stable) return true;
        }
    }
    return false;
}

typedef struct{
    int col;
    int height;
    bool active;
}Obstacle;

static Obstacle obs[6];

static int dino_row=0;
static int velocity=0;
static bool jumping=false;

typedef enum{
    PLAY,
    GAME_OVER
}State;

static State state=PLAY;

static void spawn_obs(){
    for(int i=0;i<6;i++){
        if(!obs[i].active){
            obs[i].active=true;
            obs[i].col=GRID_COLS-1;
            obs[i].height=(rand()%2)+1; // 1 o 2 píxeles
            return;
        }
    }
}

static void render(){
    fb_clear();

    fb_set(dino_row,DINO_COL,1);

    for(int i=0;i<6;i++){
        if(obs[i].active){
            for(int h=0;h<obs[i].height;h++){
                fb_set(GROUND_ROW+h,obs[i].col,2);
            }
        }
    }
}

static bool collision(){
    for(int i=0;i<6;i++){
        if(obs[i].active && obs[i].col==DINO_COL){
            for(int h=0;h<obs[i].height;h++){
                if(dino_row==GROUND_ROW+h)
                    return true;
            }
        }
    }
    return false;
}

static void game_task(void *pv){
    TickType_t last_move=0;
    TickType_t last_spawn=0;

    while(1){
        TickType_t now=xTaskGetTickCount();

        if(state==PLAY){

            if(btn_pressed(&btn_left) && !jumping){
                jumping=true;
                velocity=3;
            }

            if(jumping){
                dino_row+=velocity;
                velocity--;

                if(dino_row<=0){
                    dino_row=0;
                    jumping=false;
                }

                if(dino_row>5) dino_row=5;
            }

            if((now-last_move)>=pdMS_TO_TICKS(300)){
                for(int i=0;i<6;i++){
                    if(obs[i].active){
                        obs[i].col--;
                        if(obs[i].col<0)
                            obs[i].active=false;
                    }
                }
                last_move=now;
            }

            if((now-last_spawn)>=pdMS_TO_TICKS(1200)){
                spawn_obs();
                last_spawn=now;
            }

            if(collision()) state=GAME_OVER;

            render();
        }

        else{
            fb_clear();
            for(int i=0;i<6;i++) fb_set(i,i,2);

            if(btn_pressed(&btn_right)){
                memset(obs,0,sizeof(obs));
                dino_row=0;
                velocity=0;
                jumping=false;
                state=PLAY;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(60));
    }
}

static void display_task(void *pv){
    esp_timer_handle_t timer;

    esp_timer_create_args_t args={.callback=display_isr};
    esp_timer_create(&args,&timer);
    esp_timer_start_periodic(timer,DISPLAY_PERIOD_US);

    while(1) vTaskDelay(pdMS_TO_TICKS(1000));
}

void app_main(void){
    srand(esp_random());
    gpio_init_all();
    fb_clear();

    xTaskCreatePinnedToCore(display_task,"display",2048,NULL,10,NULL,1);
    xTaskCreatePinnedToCore(game_task,"game",4096,NULL,5,NULL,0);
}