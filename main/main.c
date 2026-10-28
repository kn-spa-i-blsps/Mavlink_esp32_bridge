/*
 * ESP32-S3 MAVLink <-> WiFi bridge (ESP-IDF)
 *
 * Two boards running this same firmware replace a pair of telemetry radios:
 *
 *   laptop --USB-- [GROUND: WiFi AP] ~~WiFi/UDP~~ [DRONE: WiFi STA] --UART-- flight controller
 *
 * The relay is transparent: raw bytes are passed through without parsing MAVLink,
 * so QGroundControl / Mission Planner talk to the FC as if wired to it.
 *
 * Role is picked at build time (CONFIG_MAV_ROLE_GROUND / CONFIG_MAV_ROLE_DRONE).
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"

#if CONFIG_MAV_ROLE_GROUND
#include "driver/usb_serial_jtag.h"
#else
#include "driver/uart.h"
#include "driver/gpio.h"
#endif

static const char *TAG = "bridge";

/* ---- wiring / link config ---- */
#define MAV_UART        UART_NUM_1
#define PIN_TX          43          /* XIAO ESP32S3 D6 (labelled TX); GPIO17/18 are not broken out there */
#define PIN_RX          44          /* XIAO ESP32S3 D7 (labelled RX) */
#define MAV_BAUD        115200      /* must match SERIALn_BAUD on the FC */
#define SERIAL_BUF_SIZE 1024

#define GROUND_IP       "192.168.4.1"   /* default address of the ESP32 access point */

/* ---- serial side: USB to the laptop (ground) or UART to the FC (drone) ---- */
#if CONFIG_MAV_ROLE_GROUND

static void serial_init(void)
{
    usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    cfg.rx_buffer_size = SERIAL_BUF_SIZE;
    cfg.tx_buffer_size = SERIAL_BUF_SIZE;
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&cfg));
}

static int serial_read(uint8_t *buf, size_t len, TickType_t wait)
{
    return usb_serial_jtag_read_bytes(buf, len, wait);
}

static void serial_write(const uint8_t *buf, size_t len)
{
    /* short timeout so a closed laptop port can't stall the relay */
    usb_serial_jtag_write_bytes(buf, len, pdMS_TO_TICKS(20));
}

#else

static void serial_init(void)
{
    const uart_config_t cfg = {
        .baud_rate = MAV_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(MAV_UART, SERIAL_BUF_SIZE, SERIAL_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MAV_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(MAV_UART, PIN_TX, PIN_RX,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(gpio_set_pull_mode(PIN_RX, GPIO_PULLUP_ONLY));   /* idle high if nothing drives RX */
}

static int serial_read(uint8_t *buf, size_t len, TickType_t wait)
{
    return uart_read_bytes(MAV_UART, buf, len, wait);
}

static void serial_write(const uint8_t *buf, size_t len)
{
    uart_write_bytes(MAV_UART, (const char *)buf, len);
}

#endif

/* ---- WiFi: ground is the access point, drone joins it ---- */
static void wifi_event_handler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
#if CONFIG_MAV_ROLE_GROUND
    if (id == WIFI_EVENT_AP_STACONNECTED) {
        ESP_LOGI(TAG, "drone joined the access point");
    } else if (id == WIFI_EVENT_AP_STADISCONNECTED) {
        ESP_LOGW(TAG, "drone left the access point");
    }
#else
    if (id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (id == WIFI_EVENT_STA_CONNECTED) {
        ESP_LOGI(TAG, "joined '%s'", CONFIG_MAV_WIFI_SSID);
    } else if (id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGW(TAG, "WiFi lost, reconnecting");
        esp_wifi_connect();   /* keep trying forever: the link may drop in flight */
    }
#endif
}

static void wifi_start(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    const wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));

    wifi_config_t cfg = { 0 };
#if CONFIG_MAV_ROLE_GROUND
    esp_netif_create_default_wifi_ap();
    strlcpy((char *)cfg.ap.ssid, CONFIG_MAV_WIFI_SSID, sizeof(cfg.ap.ssid));
    strlcpy((char *)cfg.ap.password, CONFIG_MAV_WIFI_PASS, sizeof(cfg.ap.password));
    cfg.ap.ssid_len       = strlen(CONFIG_MAV_WIFI_SSID);
    cfg.ap.channel        = 1;
    cfg.ap.authmode       = WIFI_AUTH_WPA2_PSK;
    cfg.ap.max_connection = 1;   /* only the drone board may join */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &cfg));
#else
    esp_netif_create_default_wifi_sta();
    strlcpy((char *)cfg.sta.ssid, CONFIG_MAV_WIFI_SSID, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, CONFIG_MAV_WIFI_PASS, sizeof(cfg.sta.password));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));
#endif
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));   /* power save adds latency */
}

/* ---- UDP side ---- */
static int                g_sock = -1;
static struct sockaddr_in g_peer;
static volatile bool      g_peer_known = false;

static void udp_init(void)
{
    g_sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    ESP_ERROR_CHECK(g_sock < 0 ? ESP_FAIL : ESP_OK);

    const struct sockaddr_in local = {
        .sin_family      = AF_INET,
        .sin_port        = htons(CONFIG_MAV_UDP_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };
    ESP_ERROR_CHECK(bind(g_sock, (const struct sockaddr *)&local, sizeof(local)) < 0
                    ? ESP_FAIL : ESP_OK);

#if !CONFIG_MAV_ROLE_GROUND
    /* the drone always sends to the ground board; the ground board learns the
     * drone's address from the first packet it receives */
    g_peer.sin_family = AF_INET;
    g_peer.sin_port   = htons(CONFIG_MAV_UDP_PORT);
    inet_pton(AF_INET, GROUND_IP, &g_peer.sin_addr);
    g_peer_known = true;
#endif
}

/* serial side -> WiFi */
static void serial_to_udp_task(void *arg)
{
    uint8_t buf[256];
#if !CONFIG_MAV_ROLE_GROUND
    uint32_t rx_bytes = 0;
    TickType_t last_report = xTaskGetTickCount();
#endif
    for (;;) {
        int n = serial_read(buf, sizeof(buf), pdMS_TO_TICKS(10));
#if !CONFIG_MAV_ROLE_GROUND
        if (n > 0) {
            rx_bytes += n;
        }
        if (xTaskGetTickCount() - last_report >= pdMS_TO_TICKS(2000)) {
            if (rx_bytes) {
                ESP_LOGI(TAG, "FC -> %u bytes in the last 2 s", (unsigned)rx_bytes);
            } else {
                ESP_LOGW(TAG, "nothing from the FC in the last 2 s (wiring? SERIALn params?) RX pin=%d",
                         gpio_get_level(PIN_RX));
            }
            rx_bytes = 0;
            last_report = xTaskGetTickCount();
        }
#endif
        if (n > 0 && g_peer_known) {
            sendto(g_sock, buf, n, 0, (const struct sockaddr *)&g_peer, sizeof(g_peer));
        }
    }
}

/* WiFi -> serial side */
static void udp_to_serial_task(void *arg)
{
    uint8_t buf[512];
    for (;;) {
        struct sockaddr_in from;
        socklen_t from_len = sizeof(from);
        int n = recvfrom(g_sock, buf, sizeof(buf), 0, (struct sockaddr *)&from, &from_len);
        if (n <= 0) {
            continue;
        }
#if !CONFIG_MAV_ROLE_GROUND
        /* only the ground board may talk to the FC */
        if (from.sin_addr.s_addr != g_peer.sin_addr.s_addr) {
            continue;
        }
#endif
#if CONFIG_MAV_ROLE_GROUND
        if (!g_peer_known) {
            ESP_LOGI(TAG, "peer found");
        }
        g_peer       = from;
        g_peer_known = true;
#endif
#if !CONFIG_MAV_ROLE_GROUND
        static int logged = 0;
        if (logged < 5) {   /* first few packets only: proves WiFi -> UART TX is reached */
            ESP_LOGI(TAG, "UDP -> UART TX: %d bytes", n);
            logged++;
        }
#endif
        serial_write(buf, n);
    }
}

void app_main(void)
{
#if CONFIG_MAV_ROLE_GROUND
    ESP_LOGI(TAG, "role: GROUND (access point '%s', USB serial to laptop)", CONFIG_MAV_WIFI_SSID);
#else
    ESP_LOGI(TAG, "role: DRONE (joins '%s', UART to flight controller)", CONFIG_MAV_WIFI_SSID);
#endif
    serial_init();
    wifi_start();
    udp_init();

    xTaskCreate(serial_to_udp_task, "ser2udp", 4096, NULL, 5, NULL);
    xTaskCreate(udp_to_serial_task, "udp2ser", 4096, NULL, 5, NULL);
}
