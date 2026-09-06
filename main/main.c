/*
 * ESP32-S3 <-> ArduPilot MAVLink2 bridge (ESP-IDF v5.x)
 *
 * - UART1 on the GPIO matrix (pick free pins), 3.3V TTL straight to a FC TELEM port
 * - Sends HEARTBEAT at 1 Hz  (FC ignores commands from a link it thinks is dead)
 * - Parses RX: locks target sysid/compid from the FC's HEARTBEAT, logs COMMAND_ACK
 * - Once the FC is seen, sends the hardcoded command list below, one every 2s
 *
 * MAVLink C headers (c_library_v2, ardupilotmega dialect) must be added as a component.
 * See notes below the file for CMakeLists + FC-side SERIALn params.
 */

#include <string.h>
#include <math.h>
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
static uint8_t  g_target_sys   = 0;
static uint8_t  g_target_comp  = 0;
static bool     g_link_up      = false;
static uint32_t g_flight_mode  = 0;   /* HEARTBEAT.custom_mode, refreshed every heartbeat */

/* ArduCopter custom_mode -> human name (see ardupilotmega.h enum COPTER_MODE) */
static const char *copter_mode_name(uint32_t mode)
{
    switch (mode) {
    case COPTER_MODE_STABILIZE: return "STABILIZE";
    case COPTER_MODE_ACRO:      return "ACRO";
    case COPTER_MODE_ALT_HOLD:  return "ALT_HOLD";
    case COPTER_MODE_AUTO:      return "AUTO";
    case COPTER_MODE_GUIDED:    return "GUIDED";
    case COPTER_MODE_LOITER:    return "LOITER";
    case COPTER_MODE_RTL:       return "RTL";
    case COPTER_MODE_LAND:      return "LAND";
    case COPTER_MODE_POSHOLD:   return "POSHOLD";
    case COPTER_MODE_BRAKE:     return "BRAKE";
    default:                    return "OTHER";
    }
}

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

/* One MAV_CMD_* to send via COMMAND_LONG, with its 7 params. */
typedef struct {
    uint16_t cmd;
    float    param[7];
} mav_cmd_t;

/* ---- hardcoded command sequence -> edit this to whatever the FC should do ---- */
static const mav_cmd_t g_commands[] = {
    /* ask the FC to emit AUTOPILOT_VERSION once (harmless, proves the pipe) */
    { MAV_CMD_REQUEST_MESSAGE, { MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0 } },
    /* ask the FC to emit SYS_STATUS once */
    { MAV_CMD_REQUEST_MESSAGE, { MAVLINK_MSG_ID_SYS_STATUS, 0, 0, 0, 0, 0, 0 } },
    /* stream ATTITUDE (roll/pitch/yaw + timestamp) continuously at 1 Hz */
    { MAV_CMD_SET_MESSAGE_INTERVAL, { MAVLINK_MSG_ID_ATTITUDE, 1000000, 0, 0, 0, 0, 0 } },
};
#define NUM_COMMANDS (sizeof(g_commands) / sizeof(g_commands[0]))

static void send_command(const mav_cmd_t *c)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(OWN_SYSID, OWN_COMPID, &msg,
        g_target_sys, g_target_comp,
        c->cmd, 0 /* confirmation */,
        c->param[0], c->param[1], c->param[2], c->param[3],
        c->param[4], c->param[5], c->param[6]);
    mav_send(&msg);
    ESP_LOGI(TAG, "-> command %u to %u/%u", c->cmd, g_target_sys, g_target_comp);
}

static void handle_message(const mavlink_message_t *msg)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT: {
        /* Only the autopilot component (compid 1), not other GCS/companions on the bus */
        if (msg->compid != MAV_COMP_ID_AUTOPILOT1) {
            break;
        }
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(msg, &hb);
        g_flight_mode = hb.custom_mode;   /* refreshed every heartbeat, not just the first */

        if (!g_link_up) {
            g_target_sys  = msg->sysid;
            g_target_comp = msg->compid;
            g_link_up     = true;
            ESP_LOGI(TAG, "FC heartbeat: sys=%u comp=%u type=%u ap=%u -> link up",
                     g_target_sys, g_target_comp, hb.type, hb.autopilot);
        }
        break;
    }

    case MAVLINK_MSG_ID_ATTITUDE: {
        mavlink_attitude_t att;
        mavlink_msg_attitude_decode(msg, &att);
        ESP_LOGI(TAG, "ATTITUDE t=%lu ms roll=%.1f pitch=%.1f yaw=%.1f deg mode=%s",
                 (unsigned long)att.time_boot_ms,
                 att.roll  * (180.0f / (float)M_PI),
                 att.pitch * (180.0f / (float)M_PI),
                 att.yaw   * (180.0f / (float)M_PI),
                 copter_mode_name(g_flight_mode));
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
    int64_t last_cmd_us = 0;
    size_t  next_cmd    = 0;

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

        /* --- walk the hardcoded command list, one every 2 s, once the FC is known --- */
        if (g_link_up && next_cmd < NUM_COMMANDS && now - last_cmd_us >= 2000000) {
            send_command(&g_commands[next_cmd]);
            next_cmd++;
            last_cmd_us = now;
        }
    }
}

void app_main(void)
{
    xTaskCreate(mavlink_task, "mavlink", 4096, NULL, 5, NULL);
}
