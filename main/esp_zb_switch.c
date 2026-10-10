#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "esp_wifi.h"
#include "esp_now.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "driver/gpio.h"
#include "esp_zb_switch.h"

#define MESH_SERIAL_USE_USB_SERIAL_JTAG 1

#if MESH_SERIAL_USE_USB_SERIAL_JTAG
#include "driver/usb_serial_jtag.h"
#else
#include "driver/uart.h"
#endif

#if !MESH_SERIAL_USE_USB_SERIAL_JTAG
#define MESH_UART_NUM UART_NUM_1
#define MESH_UART_TX_GPIO GPIO_NUM_17
#define MESH_UART_RX_GPIO GPIO_NUM_16
#define MESH_UART_BAUD 115200
#endif

static esp_err_t mesh_serial_init(void)
{
#if MESH_SERIAL_USE_USB_SERIAL_JTAG
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };
    return usb_serial_jtag_driver_install(&cfg);
#else
    uart_config_t cfg = {
        .baud_rate = MESH_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(MESH_UART_NUM, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MESH_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(MESH_UART_NUM,
                                 MESH_UART_TX_GPIO,
                                 MESH_UART_RX_GPIO,
                                 UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));
    return ESP_OK;
#endif
}

static int mesh_serial_read_byte(void)
{
    uint8_t ch = 0;
#if MESH_SERIAL_USE_USB_SERIAL_JTAG
    int n = usb_serial_jtag_read_bytes(&ch, 1, pdMS_TO_TICKS(10));
#else
    int n = uart_read_bytes(MESH_UART_NUM, &ch, 1, pdMS_TO_TICKS(10));
#endif
    return (n == 1) ? (int)ch : -1;
}

static void mesh_serial_write(const char *s)
{
    if (!s) return;
#if MESH_SERIAL_USE_USB_SERIAL_JTAG
    usb_serial_jtag_write_bytes((const uint8_t *)s, strlen(s), pdMS_TO_TICKS(100));
#else
    uart_write_bytes(MESH_UART_NUM, s, strlen(s));
#endif
}

static void mesh_serial_writef(const char *fmt, ...)
{
    char buf[160];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    mesh_serial_write(buf);
}

static const char *TAG = "COORDINATOR_ESPNOW";

// Seeed Studio XIAO ESP32-C6 RF Switch Control
// GPIO 3: RF switch enable (Active LOW - driving LOW powers ON the RF switch)
// GPIO 14: Antenna select (LOW = Onboard ceramic antenna, HIGH = External U.FL)
#define XIAO_RF_SWITCH_PWR_GPIO  GPIO_NUM_3
#define XIAO_RF_SWITCH_SEL_GPIO  GPIO_NUM_14

void mesh_coordinator_set_rf_pins(int pwr, int sel)
{
    gpio_reset_pin(XIAO_RF_SWITCH_PWR_GPIO);
    gpio_set_direction(XIAO_RF_SWITCH_PWR_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(XIAO_RF_SWITCH_PWR_GPIO, pwr);

    gpio_reset_pin(XIAO_RF_SWITCH_SEL_GPIO);
    gpio_set_direction(XIAO_RF_SWITCH_SEL_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(XIAO_RF_SWITCH_SEL_GPIO, sel);

    vTaskDelay(pdMS_TO_TICKS(20));
    ESP_LOGI(TAG, "RF Switch set: PWR(GPIO3)=%d, SEL(GPIO14)=%d", pwr, sel);
}

static void xiao_rf_switch_init(void)
{
    mesh_coordinator_set_rf_pins(0, 0); // PWR=0 (ON), SEL=0 (Ceramic antenna)
}

#define ESPNOW_WIFI_CHANNEL 1
#define MESH_TEXT_LEN 18
#define MESH_LINE_LEN 80

typedef struct
{
    char source[MESH_TEXT_LEN];
    char target[MESH_TEXT_LEN];
    char cmd[MESH_TEXT_LEN];
    int16_t value;
    int16_t value2;
    char text[MESH_LINE_LEN];
} mesh_msg_t;

typedef struct __attribute__((packed))
{
    char source[MESH_TEXT_LEN];
    char target[MESH_TEXT_LEN];
    char cmd[MESH_TEXT_LEN];
    int16_t value;
    int16_t value2;
    char text[MESH_LINE_LEN];
} espnow_frame_t;

typedef struct
{
    uint8_t mac[ESP_NOW_ETH_ALEN];
    uint64_t ieee;
    const char *name;
    uint16_t short_addr;
    uint8_t endpoint;
    bool online;
    int16_t last_lqi;
    int16_t remote_lqi;
    TickType_t last_seen_tick;
    bool fresh_ping_expected;
    bool fresh_ping_seen;
} DeviceEntry;

static DeviceEntry devices[] = {
    {{0xB4, 0x3A, 0x45, 0x8A, 0xC6, 0x40}, 0xb43a45fffe8ac640ULL, "Kitchen", 0x0001, 10, false, 0, -1, 0, false, false},
    {{0xB4, 0x3A, 0x45, 0x8A, 0xC7, 0xC0}, 0xb43a45fffe8ac7c0ULL, "Garage",  0x0002, 10, false, 0, -1, 0, false, false},
    {{0x58, 0xE6, 0xC5, 0x1A, 0xE8, 0xA0}, 0x58e6c5fffe1ae8a0ULL, "Santafe", 0x0003, 10, false, 0, -1, 0, false, false},
    {{0x58, 0xE6, 0xC5, 0x1A, 0xDD, 0xD0}, 0x58e6c5fffe1addd0ULL, "aa",      0x0004, 10, false, 0, -1, 0, false, false},
    {{0x58, 0xE6, 0xC5, 0x13, 0x6E, 0xEC}, 0x58e6c5fffe136eecULL, "bb",      0x0005, 10, false, 0, -1, 0, false, false},
    {{0x00, 0x12, 0x4B, 0x01, 0xCC, 0xCC}, 0x00124B0001CCCCCCULL, "Boiler",  0x0006, 10, false, 0, -1, 0, false, false},
};
#define DEVICE_COUNT (sizeof(devices) / sizeof(devices[0]))

static const uint8_t s_broadcast_mac[ESP_NOW_ETH_ALEN] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

#define MESH_RX_QUEUE_LEN 16
#define MESH_TX_QUEUE_LEN 16

static QueueHandle_t g_mesh_rx_queue = NULL;
static QueueHandle_t g_mesh_tx_queue = NULL;

#define FRESH_NETWORK_REPORT_TIMEOUT_MS 3000
static volatile bool g_fresh_report_active = false;
static volatile int g_fresh_report_expected = 0;
static volatile int g_fresh_report_received = 0;

static inline int16_t rssi_to_lqi(int8_t rssi)
{
    if (rssi <= -100) return 0;
    if (rssi >= -30)  return 255;
    return (int16_t)(((rssi + 100) * 255) / 70);
}

static bool ensure_peer_exists(const uint8_t *mac)
{
    if (!mac) return false;
    if (esp_now_is_peer_exist(mac)) {
        return true;
    }
    esp_now_peer_info_t peer = {0};
    memcpy(peer.peer_addr, mac, ESP_NOW_ETH_ALEN);
    peer.channel = ESPNOW_WIFI_CHANNEL;
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    esp_err_t err = esp_now_add_peer(&peer);
    if (err != ESP_OK && err != ESP_ERR_ESPNOW_EXIST) {
        ESP_LOGE(TAG, "Failed to add peer: %s", esp_err_to_name(err));
        return false;
    }

    esp_now_rate_config_t rate_cfg = {
        .phymode = WIFI_PHY_MODE_LR,
        .rate = WIFI_PHY_RATE_LORA_250K,
        .ersu = false,
        .dcm = false,
    };
    esp_now_set_peer_rate_config(mac, &rate_cfg);
    return true;
}

static DeviceEntry *find_device_by_name(const char *name)
{
    if (!name) return NULL;
    for (size_t i = 0; i < DEVICE_COUNT; i++) {
        if (strcasecmp(devices[i].name, name) == 0) {
            return &devices[i];
        }
    }
    return NULL;
}

static DeviceEntry *find_device_by_mac(const uint8_t *mac)
{
    if (!mac) return NULL;
    for (size_t i = 0; i < DEVICE_COUNT; i++) {
        if (memcmp(devices[i].mac, mac, ESP_NOW_ETH_ALEN) == 0) {
            return &devices[i];
        }
    }
    return NULL;
}

static void strip_line_endings(char *s)
{
    if (!s) return;
    size_t len = strlen(s);
    while (len > 0 && (s[len - 1] == '\r' || s[len - 1] == '\n')) {
        s[len - 1] = '\0';
        len--;
    }
}

static esp_err_t mesh_init_queues(void)
{
    if (!g_mesh_rx_queue) {
        g_mesh_rx_queue = xQueueCreate(MESH_RX_QUEUE_LEN, sizeof(mesh_msg_t));
    }
    if (!g_mesh_tx_queue) {
        g_mesh_tx_queue = xQueueCreate(MESH_TX_QUEUE_LEN, sizeof(mesh_msg_t));
    }
    ESP_RETURN_ON_FALSE(g_mesh_rx_queue && g_mesh_tx_queue, ESP_FAIL, TAG, "Failed to create mesh queues");
    return ESP_OK;
}

bool enqueue_outgoing(mesh_msg_t msg)
{
    if (!g_mesh_tx_queue) return false;
    return (xQueueSend(g_mesh_tx_queue, &msg, 0) == pdTRUE);
}

bool dequeue_incoming(mesh_msg_t *msg, TickType_t timeout)
{
    if (!g_mesh_rx_queue || !msg) return false;
    return (xQueueReceive(g_mesh_rx_queue, msg, timeout) == pdTRUE);
}

static void parse_console_line_to_mesh_msg(const char *line, mesh_msg_t *msg)
{
    if (!line || !msg) return;
    memset(msg, 0, sizeof(*msg));
    strncpy(msg->text, line, sizeof(msg->text) - 1);

    char target[MESH_TEXT_LEN] = {0};
    char cmd[MESH_TEXT_LEN] = {0};
    int value = 0;
    int value2 = 0;

    int fields = sscanf(line, "%17s %17s %d %d", target, cmd, &value, &value2);
    if (fields >= 1) {
        strncpy(msg->target, target, sizeof(msg->target) - 1);
    }
    if (fields >= 2) {
        strncpy(msg->cmd, cmd, sizeof(msg->cmd) - 1);
    }
    if (fields >= 3) {
        msg->value = (int16_t)value;
    }
    if (fields >= 4) {
        msg->value2 = (int16_t)value2;
    }
}

static const char *device_route_string(const DeviceEntry *dev)
{
    return (dev && dev->online && dev->short_addr != 0xFFFF) ? "YES" : "NO";
}

static const char *device_rf_status_string(const DeviceEntry *dev)
{
    if (!dev || !dev->online || dev->short_addr == 0xFFFF) {
        return "UNKNOWN";
    }
    int weakest_lqi = dev->last_lqi;
    if (dev->remote_lqi >= 0 && dev->remote_lqi < weakest_lqi) {
        weakest_lqi = dev->remote_lqi;
    }
    if (weakest_lqi < 50) return "WEAK";
    if (weakest_lqi < 100) return "MARGINAL";
    return "OK";
}

static void print_network_report(void)
{
    mesh_serial_write("\r\n..... recieved GiveNetworkReport --------\r\n\r\n");
    mesh_serial_write("*********************NETWORK_REPORT_BEGIN\r\n\r\n");
    mesh_serial_write("NAME,IEEE,SHORT,ONLINE,TYPE,LQI,REMOTE_LQI,RF_STATUS,AGE,ROUTE\r\n");

    uint32_t now = xTaskGetTickCount();

    for (size_t i = 0; i < DEVICE_COUNT; i++) {
        DeviceEntry *dev = &devices[i];
        uint32_t age_sec = 0;
        if (dev->last_seen_tick != 0) {
            age_sec = (now - dev->last_seen_tick) / configTICK_RATE_HZ;
        }

        const char *type = dev->online ? "END_DEVICE" : "UNKNOWN";
        const char *route = device_route_string(dev);
        const char *rf_status = device_rf_status_string(dev);

        char remote_lqi_str[8];
        if (dev->remote_lqi >= 0) {
            snprintf(remote_lqi_str, sizeof(remote_lqi_str), "%d", dev->remote_lqi);
        } else {
            snprintf(remote_lqi_str, sizeof(remote_lqi_str), "-");
        }

        mesh_serial_writef(
            "%s,0x%016llX,0x%04X,%d,%s,%d,%s,%s,%lu,%s\r\n",
            dev->name,
            (unsigned long long)dev->ieee,
            dev->short_addr,
            dev->online ? 1 : 0,
            type,
            dev->last_lqi,
            remote_lqi_str,
            rf_status,
            (unsigned long)age_sec,
            route);
    }

    mesh_serial_write("*********************NETWORK_REPORT_END\r\n\r\n");
    mesh_serial_write("      ----------------------\r\n");
    mesh_serial_write("      |   RF_STATUS_LEGEND  |\r\n");
    mesh_serial_write("      ----------------------\r\n");
    mesh_serial_write("MIN_LQI,MAX_LQI,RF_STATUS\r\n");
    mesh_serial_write("0,49,WEAK\r\n");
    mesh_serial_write("50,99,MARGINAL\r\n");
    mesh_serial_write("100,255,OK\r\n");
    mesh_serial_write("offline,offline,UNKNOWN\r\n\r\n");
    mesh_serial_write("RF_STATUS_LEGEND_END\r\n");
}

static void ping_network(void)
{
    mesh_serial_write("PING_NETWORK_BEGIN\r\n");
    for (size_t i = 0; i < DEVICE_COUNT; i++) {
        DeviceEntry *dev = &devices[i];
        if (dev->mac[0] == 0) continue;

        mesh_msg_t msg = {0};
        strncpy(msg.target, dev->name, sizeof(msg.target) - 1);
        strncpy(msg.text, "PING", sizeof(msg.text) - 1);

        if (enqueue_outgoing(msg)) {
            mesh_serial_writef("PING %s SENT SHORT=0x%04X\r\n", dev->name, dev->short_addr);
            ESP_LOGI(TAG, "PingNetwork: queued PING to %s", dev->name);
        } else {
            mesh_serial_writef("PING %s ERROR=TX_QUEUE_FULL\r\n", dev->name);
            ESP_LOGW(TAG, "PingNetwork: TX queue full for %s", dev->name);
        }
    }
    mesh_serial_write("PING_NETWORK_END\r\n");
}

static void fresh_network_report_task(void *arg)
{
    TickType_t start_tick = xTaskGetTickCount();
    TickType_t timeout_ticks = pdMS_TO_TICKS(FRESH_NETWORK_REPORT_TIMEOUT_MS);

    while ((xTaskGetTickCount() - start_tick) < timeout_ticks) {
        if (g_fresh_report_expected <= 0 || g_fresh_report_received >= g_fresh_report_expected) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    mesh_serial_writef("FRESH_REPORT_RESPONSES RECEIVED=%d EXPECTED=%d\r\n",
                       g_fresh_report_received,
                       g_fresh_report_expected);

    for (size_t i = 0; i < DEVICE_COUNT; i++) {
        DeviceEntry *dev = &devices[i];
        if (dev->fresh_ping_expected && !dev->fresh_ping_seen) {
            dev->online = false;
            dev->last_lqi = 0;
            dev->remote_lqi = -1;
            ESP_LOGW(TAG, "Fresh report: no PONG from %s, marking offline", dev->name);
        }
        dev->fresh_ping_expected = false;
        dev->fresh_ping_seen = false;
    }

    print_network_report();
    g_fresh_report_active = false;
    vTaskDelete(NULL);
}

static void start_fresh_network_report(void)
{
    if (g_fresh_report_active) {
        mesh_serial_write("error: fresh network report already running\r\n");
        return;
    }

    int expected = 0;
    for (size_t i = 0; i < DEVICE_COUNT; i++) {
        DeviceEntry *dev = &devices[i];
        dev->fresh_ping_expected = true;
        dev->fresh_ping_seen = false;
        expected++;
    }

    g_fresh_report_expected = expected;
    g_fresh_report_received = 0;
    g_fresh_report_active = true;

    mesh_serial_write("FRESH_NETWORK_REPORT_BEGIN\r\n");
    ping_network();

    if (xTaskCreate(fresh_network_report_task, "fresh_report_task", 3072, NULL, 4, NULL) != pdPASS) {
        mesh_serial_write("error: failed to start fresh report task\r\n");
        g_fresh_report_active = false;
    }
}

static void reset_network_discovery(void)
{
    mesh_serial_write("RESET_NETWORK_BEGIN\r\n");
    for (size_t i = 0; i < DEVICE_COUNT; i++) {
        DeviceEntry *dev = &devices[i];
        dev->online = false;
        dev->last_lqi = 0;
        dev->remote_lqi = -1;
        dev->last_seen_tick = 0;
        dev->fresh_ping_expected = false;
        dev->fresh_ping_seen = false;
    }
    ping_network();
    mesh_serial_write("RESET_NETWORK_END\r\n");
}

static void reset_coordinator(void)
{
    mesh_serial_write("RESET_COORDINATOR_BEGIN\r\n");
    mesh_serial_write("RESET_COORDINATOR_RESTARTING\r\n");
    vTaskDelay(pdMS_TO_TICKS(200));
    esp_restart();
}

static void espnow_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len)
{
    if (!recv_info || !data || len <= 0) return;

    ESP_LOGI(TAG, "ESP-NOW RX: from %02x:%02x:%02x:%02x:%02x:%02x len=%d",
             recv_info->src_addr[0], recv_info->src_addr[1], recv_info->src_addr[2],
             recv_info->src_addr[3], recv_info->src_addr[4], recv_info->src_addr[5], len);

    DeviceEntry *dev = find_device_by_mac(recv_info->src_addr);
    int16_t lqi = 0;
    if (recv_info->rx_ctrl) {
        lqi = rssi_to_lqi(recv_info->rx_ctrl->rssi);
    }

    if (dev) {
        dev->online = true;
        dev->last_seen_tick = xTaskGetTickCount();
        dev->last_lqi = lqi;
        ESP_LOGI(TAG, "Device %s ONLINE (LQI=%d)", dev->name, lqi);
    }

    mesh_msg_t rx = {0};

    if (len == sizeof(espnow_frame_t)) {
        const espnow_frame_t *frame = (const espnow_frame_t *)data;
        if (dev) {
            strncpy(rx.source, dev->name, sizeof(rx.source) - 1);
        } else if (frame->source[0] != '\0') {
            strncpy(rx.source, frame->source, sizeof(rx.source) - 1);
        } else {
            strncpy(rx.source, "unknown", sizeof(rx.source) - 1);
        }
        strncpy(rx.target, frame->target, sizeof(rx.target) - 1);
        strncpy(rx.cmd, frame->cmd, sizeof(rx.cmd) - 1);
        rx.value = frame->value;
        rx.value2 = frame->value2;
        strncpy(rx.text, frame->text, sizeof(rx.text) - 1);
        ESP_LOGI(TAG, "ESP-NOW RX frame: src='%s' tgt='%s' cmd='%s' text='%s'",
                 rx.source, rx.target, rx.cmd, rx.text);
    } else {
        size_t copy_len = (size_t)len < (sizeof(rx.text) - 1) ? (size_t)len : (sizeof(rx.text) - 1);
        memcpy(rx.text, data, copy_len);
        rx.text[copy_len] = '\0';
        if (dev) {
            strncpy(rx.source, dev->name, sizeof(rx.source) - 1);
        } else {
            strncpy(rx.source, "unknown", sizeof(rx.source) - 1);
        }
        ESP_LOGI(TAG, "ESP-NOW RX text: src='%s' text='%s'", rx.source, rx.text);
    }

    rx.value = lqi;

    if (strncmp(rx.text, "PONG", 4) == 0 && dev) {
        int remote_lqi = -1;
        char *p = strstr(rx.text, "LQI=");
        if (p) {
            remote_lqi = atoi(p + 4);
        }
        dev->remote_lqi = remote_lqi;
        dev->fresh_ping_seen = true;
    }

    if (g_mesh_rx_queue) {
        if (xQueueSend(g_mesh_rx_queue, &rx, 0) != pdTRUE) {
            ESP_LOGW(TAG, "RX queue full, dropping text from %s", rx.source);
        }
    }
}

static void espnow_send_cb(const esp_now_send_info_t *tx_info, esp_now_send_status_t status)
{
    ESP_LOGI(TAG, "ESP-NOW TX cb: status=%d (%s)", status, status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

static void serial_console_task(void *arg)
{
    char line[MESH_LINE_LEN] = {0};
    size_t pos = 0;
    bool last_was_cr = false;

    ESP_LOGI(TAG, "PuTTY bridge interface ready: <target> <text>");
    mesh_serial_write("\r\nCoordinator bridge ready. Use: <DeviceName> <text>\r\n");
    mesh_serial_write("Examples: Kitchen blink 3  |  Garage ChickenSoup88\r\n");
    mesh_serial_write("Commands: GiveNetworkReport, PingNetwork, ResetNetwork, ResetCoordinator\r\n\r\ncoordinator> ");

    while (1) {
        int ch = mesh_serial_read_byte();
        if (ch < 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (ch == '\n' && last_was_cr) {
            last_was_cr = false;
            continue;
        }
        last_was_cr = (ch == '\r');

        if (ch == '\r' || ch == '\n') {
            if (pos == 0) {
                mesh_serial_write("\r\ncoordinator> ");
                continue;
            }

            line[pos] = '\0';
            strip_line_endings(line);
            mesh_serial_writef("\r\n> %s\r\n", line);

            if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
                mesh_serial_write("\r\nCommands: GiveNetworkReport, PingNetwork, ResetNetwork, ResetCoordinator, scan, rf <pwr> <sel>\r\n");
                mesh_serial_write("Usage: <DeviceName> <command/text> (e.g. Kitchen blink 3)\r\ncoordinator> ");
                pos = 0;
                memset(line, 0, sizeof(line));
                continue;
            }

            if (strncmp(line, "rf ", 3) == 0) {
                int pwr = 0, sel = 0;
                if (sscanf(line + 3, "%d %d", &pwr, &sel) == 2) {
                    mesh_coordinator_set_rf_pins(pwr, sel);
                    mesh_serial_writef("RF pins set: PWR=%d, SEL=%d\r\n", pwr, sel);
                }
                pos = 0;
                memset(line, 0, sizeof(line));
                continue;
            }

            if (strcmp(line, "scan") == 0) {
                mesh_serial_write("Scanning Wi-Fi...\r\n");
                wifi_scan_config_t scan_cfg = {0};
                esp_err_t err = esp_wifi_scan_start(&scan_cfg, true);
                if (err == ESP_OK) {
                    uint16_t ap_count = 0;
                    esp_wifi_scan_get_ap_num(&ap_count);
                    mesh_serial_writef("Found %d APs\r\n", ap_count);
                    if (ap_count > 0) {
                        wifi_ap_record_t *ap_list = malloc(sizeof(wifi_ap_record_t) * ap_count);
                        if (ap_list) {
                            esp_wifi_scan_get_ap_records(&ap_count, ap_list);
                            for (int i = 0; i < ap_count && i < 10; i++) {
                                mesh_serial_writef("  SSID: %s, RSSI: %d, Chan: %d\r\n",
                                                   ap_list[i].ssid, ap_list[i].rssi, ap_list[i].primary);
                            }
                            free(ap_list);
                        }
                    }
                } else {
                    mesh_serial_writef("Scan failed: %s\r\n", esp_err_to_name(err));
                }
                pos = 0;
                memset(line, 0, sizeof(line));
                continue;
            }

            if (strcmp(line, "GiveNetworkReport") == 0) {
                start_fresh_network_report();
                pos = 0;
                memset(line, 0, sizeof(line));
                continue;
            }

            if (strcmp(line, "PingNetwork") == 0) {
                ping_network();
                pos = 0;
                memset(line, 0, sizeof(line));
                continue;
            }

            if (strcmp(line, "ResetNetwork") == 0) {
                reset_network_discovery();
                pos = 0;
                memset(line, 0, sizeof(line));
                continue;
            }

            if (strcmp(line, "ResetCoordinator") == 0) {
                reset_coordinator();
                pos = 0;
                memset(line, 0, sizeof(line));
                continue;
            }

            mesh_msg_t msg = {0};
            parse_console_line_to_mesh_msg(line, &msg);

            if (msg.target[0] == '\0') {
                mesh_serial_write("error: missing target\r\n");
                ESP_LOGW(TAG, "Serial line ignored: missing target");
            } else if (!enqueue_outgoing(msg)) {
                mesh_serial_write("error: TX queue full\r\n");
                ESP_LOGW(TAG, "TX queue full, dropping serial line: %s", line);
            } else {
                mesh_serial_write("queued\r\n");
                ESP_LOGI(TAG, "SERIAL TX queued: %s", line);
            }

            pos = 0;
            memset(line, 0, sizeof(line));
        }
        else if (ch == 8 || ch == 127) {
            if (pos > 0) {
                pos--;
                line[pos] = '\0';
                mesh_serial_write("\b \b");
            }
        }
        else if (pos < sizeof(line) - 1) {
            line[pos++] = (char)ch;
            char echo[2] = {(char)ch, 0};
            mesh_serial_write(echo);
        }
        else {
            mesh_serial_write("\r\nerror: line too long\r\n");
            pos = 0;
            memset(line, 0, sizeof(line));
        }
    }
}

static void mesh_tx_task(void *arg)
{
    mesh_msg_t msg;

    while (1) {
        if (xQueueReceive(g_mesh_tx_queue, &msg, portMAX_DELAY) == pdTRUE) {
            if (msg.target[0] == '\0') continue;

            DeviceEntry *dev = find_device_by_name(msg.target);
            const uint8_t *dest_mac = dev ? dev->mac : s_broadcast_mac;
            ensure_peer_exists(dest_mac);

            espnow_frame_t frame = {0};
            strncpy(frame.source, "coordinator", sizeof(frame.source) - 1);
            strncpy(frame.target, msg.target, sizeof(frame.target) - 1);
            strncpy(frame.cmd, msg.cmd, sizeof(frame.cmd) - 1);
            frame.value = msg.value;
            frame.value2 = msg.value2;
            strncpy(frame.text, msg.text, sizeof(frame.text) - 1);

            ESP_LOGI(TAG, "TX task: sending to %02x:%02x:%02x:%02x:%02x:%02x target='%s' text='%s'",
                     dest_mac[0], dest_mac[1], dest_mac[2], dest_mac[3], dest_mac[4], dest_mac[5],
                     frame.target, frame.text);

            esp_err_t err = esp_now_send(dest_mac, (const uint8_t *)&frame, sizeof(frame));
            ESP_LOGI(TAG, "esp_now_send returned %s (%d)", esp_err_to_name(err), err);

            if (err != ESP_OK) {
                ESP_LOGW(TAG, "esp_now_send to %s failed: %s, trying broadcast", msg.target, esp_err_to_name(err));
                ensure_peer_exists(s_broadcast_mac);
                esp_now_send(s_broadcast_mac, (const uint8_t *)&frame, sizeof(frame));
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

static void app_msg_task(void *arg)
{
    mesh_msg_t msg;

    while (1) {
        if (dequeue_incoming(&msg, portMAX_DELAY)) {
            if (msg.text[0] != '\0') {
                ESP_LOGI(TAG, "SERIAL RX from %s: %s", msg.source, msg.text);
                if (strcmp(msg.text, "ACK") == 0) {
                    mesh_serial_writef("< %s: [ACK]\r\n", msg.source);
                }
                else if (strncmp(msg.text, "PONG", 4) == 0) {
                    int remote_lqi = -1;
                    char *p = strstr(msg.text, "LQI=");
                    if (p) {
                        remote_lqi = atoi(p + 4);
                    }
                    mesh_serial_writef("< %s: [PONG LQI=%d REMOTE_LQI=%d]\r\n",
                                       msg.source,
                                       msg.value,
                                       remote_lqi);
                    if (g_fresh_report_active) {
                        g_fresh_report_received++;
                    }
                }
                else {
                    mesh_serial_writef("< %s: %s\r\n", msg.source, msg.text);
                }

                if (strcmp(msg.text, "ACK") != 0 &&
                    strncmp(msg.text, "PONG", 4) != 0 &&
                    msg.source[0] != '\0') {
                    mesh_msg_t ack = {0};
                    strncpy(ack.target, msg.source, sizeof(ack.target) - 1);
                    strncpy(ack.text, "ACK", sizeof(ack.text) - 1);
                    enqueue_outgoing(ack);
                }
            }
            else if (strcmp(msg.cmd, "button") == 0) {
                ESP_LOGI(TAG, "%s Button Pressed", msg.source);
                mesh_serial_writef("< %s: button %d\r\n", msg.source, msg.value);
            }
            else {
                ESP_LOGI(TAG, "APP RX source='%s' cmd='%s' value=%d",
                         msg.source, msg.cmd, msg.value);
                mesh_serial_writef("< %s: %s %d\r\n", msg.source, msg.cmd, msg.value);
            }
        }
    }
}

void app_main(void)
{
    ESP_ERROR_CHECK(mesh_init_queues());
    ESP_ERROR_CHECK(mesh_serial_init());

    mesh_serial_write("\r\n--- ESP-NOW Coordinator Bridge ---\r\n");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    esp_err_t netif_err = esp_netif_init();
    if (netif_err != ESP_OK && netif_err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(netif_err);
    }

    esp_err_t evt_err = esp_event_loop_create_default();
    if (evt_err != ESP_OK && evt_err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(evt_err);
    }

    // Power on Seeed Studio XIAO ESP32-C6 RF switch and select ceramic antenna
    xiao_rf_switch_init();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(ESPNOW_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N | WIFI_PROTOCOL_LR));
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(80)); // 80 * 0.25 dBm = 20 dBm (Maximum RF TX Power)
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));

    // Re-assert RF switch after Wi-Fi start
    xiao_rf_switch_init();

    int8_t max_tx_power = 0;
    esp_wifi_get_max_tx_power(&max_tx_power);
    ESP_LOGI(TAG, "Coordinator Wi-Fi Config: Max TX Power=%d (0.25dBm units, %.2fdBm), LR Mode enabled",
             max_tx_power, max_tx_power * 0.25f);

    ESP_ERROR_CHECK(esp_now_init());
    ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));
    ESP_ERROR_CHECK(esp_now_register_send_cb(espnow_send_cb));

    ensure_peer_exists(s_broadcast_mac);
    for (size_t i = 0; i < DEVICE_COUNT; i++) {
        if (devices[i].mac[0] != 0) {
            ensure_peer_exists(devices[i].mac);
        }
    }

    ESP_LOGI(TAG, "ESP-NOW Coordinator initialized on Channel %d", ESPNOW_WIFI_CHANNEL);

    xTaskCreate(mesh_tx_task, "mesh_tx_task", 3072, NULL, 5, NULL);
    xTaskCreate(app_msg_task, "app_msg_task", 3072, NULL, 5, NULL);
    xTaskCreate(serial_console_task, "serial_console_task", 3072, NULL, 5, NULL);

    // Initial ping across the network to immediately discover online nodes
    vTaskDelay(pdMS_TO_TICKS(100));
    ping_network();
}

