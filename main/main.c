/*
 * ESP32-S3 <-> ArduPilot MAVLink2 bridge (ESP-IDF v5.x)
 *
 * - UART1 on the GPIO matrix (pick free pins), 3.3V TTL straight to a FC TELEM port
 * - Sends HEARTBEAT at 1 Hz  (FC ignores commands from a link it thinks is dead)
 * - Parses RX: locks target sysid/compid from the FC's HEARTBEAT, logs COMMAND_ACK
 * - Demo command: MAV_CMD_REQUEST_MESSAGE (harmless, returns an ACK -> proves the pipe)
 *
 * MAVLink C headers (c_library_v2, ardupilotmega dialect) must be added as a component.
 * See notes below the file for CMakeLists + FC-side SERIALn params.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "ardupilotmega/mavlink.h"   /* superset of common/ — safe for ArduPilot */

static const char *TAG = "mav";

/* ---- wiring / link config ---- */
#define MAV_UART        UART_NUM_1
#define PIN_TX          17          /* S3: avoid strapping (0,3,45,46) & USB (19,20) */
#define PIN_RX          18
#define MAV_BAUD        115200      /* must match SERIALn_BAUD on the FC */
#define RX_BUF_SIZE     1024
#define TX_BUF_SIZE     512

/* ---- our identity on the bus ---- */
#define OWN_SYSID       1                              /* same as vehicle -> treated as onboard */
#define OWN_COMPID      MAV_COMP_ID_ONBOARD_COMPUTER   /* 191 */

/* learned from the FC's heartbeat before we send anything targeted */
static uint8_t  g_target_sys  = 0;
static uint8_t  g_target_comp = 0;
static bool     g_link_up     = false;

static void uart_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = MAV_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(MAV_UART, RX_BUF_SIZE, TX_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MAV_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(MAV_UART, PIN_TX, PIN_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void mav_send(const mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    uart_write_bytes(MAV_UART, (const char *)buf, len);
}

static void send_heartbeat(void)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(OWN_SYSID, OWN_COMPID, &msg,
        MAV_TYPE_ONBOARD_CONTROLLER,  /* we are a companion, not a GCS or a vehicle */
        MAV_AUTOPILOT_INVALID,        /* companions must advertise INVALID */
        0,                            /* base_mode */
        0,                            /* custom_mode */
        MAV_STATE_ACTIVE);
    mav_send(&msg);
}

/* Demo: ask the FC to emit AUTOPILOT_VERSION once. Returns a COMMAND_ACK, moves nothing. */
static void request_autopilot_version(void)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(OWN_SYSID, OWN_COMPID, &msg,
        g_target_sys, g_target_comp,
        MAV_CMD_REQUEST_MESSAGE,          /* 512 */
        0,                                /* confirmation */
        (float)MAVLINK_MSG_ID_AUTOPILOT_VERSION, /* param1 = requested msg id (148) */
        0, 0, 0, 0, 0, 0);
    mav_send(&msg);
    ESP_LOGI(TAG, "-> REQUEST_MESSAGE(AUTOPILOT_VERSION) to %u/%u",
             g_target_sys, g_target_comp);
}

static void handle_message(const mavlink_message_t *msg)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT: {
        /* Lock onto the autopilot component only (compid 1), not other GCS/companions */
        if (!g_link_up && msg->compid == MAV_COMP_ID_AUTOPILOT1) {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(msg, &hb);
            g_target_sys  = msg->sysid;
            g_target_comp = msg->compid;
            g_link_up     = true;
            ESP_LOGI(TAG, "FC heartbeat: sys=%u comp=%u type=%u ap=%u -> link up",
                     g_target_sys, g_target_comp, hb.type, hb.autopilot);
        }
        break;
    }

    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(msg, &ack);
        ESP_LOGI(TAG, "COMMAND_ACK cmd=%u result=%u", ack.command, ack.result);
        /* Real code: on MAV_RESULT_TEMPORARILY_REJECTED -> back off and retry */
        break;
    }

    case MAVLINK_MSG_ID_AUTOPILOT_VERSION: {
        mavlink_autopilot_version_t v;
        mavlink_msg_autopilot_version_decode(msg, &v);
        ESP_LOGI(TAG, "AUTOPILOT_VERSION flight_sw=0x%08lx", (unsigned long)v.flight_sw_version);
        break;
    }

    default:
        break;
    }
}

static void mavlink_task(void *arg)
{
    uart_init();

    mavlink_message_t rx_msg;
    mavlink_status_t  rx_status;
    uint8_t buf[256];

    int64_t last_hb_us  = 0;
    int64_t last_req_us = 0;

    while (1) {
        /* --- RX: drain whatever is available, parse byte by byte --- */
        int n = uart_read_bytes(MAV_UART, buf, sizeof(buf), pdMS_TO_TICKS(20));
        for (int i = 0; i < n; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &rx_msg, &rx_status)) {
                handle_message(&rx_msg);
            }
        }

        int64_t now = esp_timer_get_time();

        /* --- 1 Hz heartbeat --- */
        if (now - last_hb_us >= 1000000) {
            send_heartbeat();
            last_hb_us = now;
        }

        /* --- demo command every 5 s, only once the FC is known --- */
        if (g_link_up && now - last_req_us >= 5000000) {
            request_autopilot_version();
            last_req_us = now;
        }
    }
}

void app_main(void)
{
    xTaskCreate(mavlink_task, "mavlink", 4096, NULL, 5, NULL);
}
