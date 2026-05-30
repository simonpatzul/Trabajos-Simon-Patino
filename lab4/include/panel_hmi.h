#pragma once
#include <stdbool.h>

typedef enum {
    ESTADO_BLOQUEADO = 0,
    ESTADO_ACTIVO
} estado_t;

extern volatile estado_t g_estado;
extern char              g_msg_ble[17];
extern bool              g_hay_mensaje;
