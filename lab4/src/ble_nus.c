#include "ble_nus.h"
#include "panel_hmi.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "ble";

// UUIDs del servicio NUS (Nordic UART Service) en little-endian
// base: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
static const ble_uuid128_t SVC_UUID = BLE_UUID128_INIT(
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,
    0x93,0xF3,0xA3,0xB5,0x01,0x00,0x40,0x6E
);
// 6E400002 -> RX (el cliente escribe aqui)
static const ble_uuid128_t RX_UUID = BLE_UUID128_INIT(
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,
    0x93,0xF3,0xA3,0xB5,0x02,0x00,0x40,0x6E
);
// 6E400003 -> TX (notificaciones hacia el cliente)
static const ble_uuid128_t TX_UUID = BLE_UUID128_INIT(
    0x9E,0xCA,0xDC,0x24,0x0E,0xE5,0xA9,0xE0,
    0x93,0xF3,0xA3,0xB5,0x03,0x00,0x40,0x6E
);

static uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t tx_handle   = 0;

static int gap_cb(struct ble_gap_event *event, void *arg);

static int rx_callback(uint16_t ch, uint16_t ah,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (g_estado != ESTADO_ACTIVO) return 0;

    uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
    if (len > 16) len = 16;

    memset(g_msg_ble, 0, sizeof(g_msg_ble));
    ble_hs_mbuf_to_flat(ctxt->om, g_msg_ble, len, NULL);
    g_msg_ble[len] = '\0';
    g_hay_mensaje = true;
    return 0;
}

static int tx_callback(uint16_t ch, uint16_t ah,
                       struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    return 0;
}

static const struct ble_gatt_svc_def tabla_gatt[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &SVC_UUID.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &RX_UUID.u,
                .access_cb = rx_callback,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
            },
            {
                .uuid = &TX_UUID.u,
                .access_cb = tx_callback,
                .val_handle = &tx_handle,
                .flags = BLE_GATT_CHR_F_NOTIFY,
            },
            { 0 }
        },
    },
    { 0 }
};

static void iniciar_advertising(void)
{
    struct ble_hs_adv_fields campos = {0};
    struct ble_gap_adv_params params = {0};
    uint8_t tipo_addr;

    ble_hs_id_infer_auto(0, &tipo_addr);

    campos.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    campos.name = (const uint8_t *)"PanelHMI";
    campos.name_len = 8;
    campos.name_is_complete = 1;
    ble_gap_adv_set_fields(&campos);

    params.conn_mode = BLE_GAP_CONN_MODE_UND;
    params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    int r = ble_gap_adv_start(tipo_addr, NULL, BLE_HS_FOREVER, &params, gap_cb, NULL);
    if (r != 0) {
        ESP_LOGE(TAG, "error al anunciar: %d", r);
    }
}

static int gap_cb(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            conn_handle = event->connect.conn_handle;
            ESP_LOGI(TAG, "cliente conectado");
        } else {
            conn_handle = BLE_HS_CONN_HANDLE_NONE;
            iniciar_advertising();
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "cliente desconectado");
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        iniciar_advertising();
        break;
    default:
        break;
    }
    return 0;
}

static void on_sync(void)
{
    iniciar_advertising();
}

static void on_reset(int reason)
{
    ESP_LOGE(TAG, "ble reset: %d", reason);
}

static void tarea_nimble(void *arg)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

void ble_nus_init(void)
{
    nimble_port_init();

    ble_hs_cfg.sync_cb  = on_sync;
    ble_hs_cfg.reset_cb = on_reset;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_gatts_count_cfg(tabla_gatt);
    ble_gatts_add_svcs(tabla_gatt);

    ble_svc_gap_device_name_set("PanelHMI");
    nimble_port_freertos_init(tarea_nimble);
}

void ble_nus_send(const char *str)
{
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) return;
    if (!str || str[0] == '\0') return;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(str, strlen(str));
    if (om != NULL) {
        ble_gattc_notify_custom(conn_handle, tx_handle, om);
    }
}
