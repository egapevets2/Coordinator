#include "mesh_serial.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// =======================================================
// CONFIG: choose transport
// =======================================================
#define MESH_SERIAL_USE_USB_SERIAL_JTAG 1   // 1 = USB (now), 0 = UART (later)

// =======================================================
// USB SERIAL/JTAG IMPLEMENTATION
// =======================================================
#if MESH_SERIAL_USE_USB_SERIAL_JTAG

#include "driver/usb_serial_jtag.h"

esp_err_t mesh_serial_init(void)
{
    usb_serial_jtag_driver_config_t cfg = {
        .tx_buffer_size = 1024,
        .rx_buffer_size = 1024,
    };

    return usb_serial_jtag_driver_install(&cfg);
}

int mesh_serial_read_byte(void)
{
    uint8_t ch;
    int n = usb_serial_jtag_read_bytes(&ch, 1, pdMS_TO_TICKS(10));
    return (n == 1) ? ch : -1;
}

void mesh_serial_write(const char *s)
{
    if (!s) return;
    usb_serial_jtag_write_bytes((const uint8_t *)s, strlen(s), pdMS_TO_TICKS(100));
}

#else

// =======================================================
// UART IMPLEMENTATION (future RX/TX pins)
// =======================================================

#include "driver/uart.h"
#include "driver/gpio.h"

#define MESH_UART_NUM      UART_NUM_1
#define MESH_UART_TX_GPIO  GPIO_NUM_17
#define MESH_UART_RX_GPIO  GPIO_NUM_16
#define MESH_UART_BAUD     115200

esp_err_t mesh_serial_init(void)
{
    uart_config_t cfg = {
        .baud_rate = MESH_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(MESH_UART_NUM, 1024, 1024, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(MESH_UART_NUM, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(
        MESH_UART_NUM,
        MESH_UART_TX_GPIO,
        MESH_UART_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    return ESP_OK;
}

int mesh_serial_read_byte(void)
{
    uint8_t ch;
    int n = uart_read_bytes(MESH_UART_NUM, &ch, 1, pdMS_TO_TICKS(10));
    return (n == 1) ? ch : -1;
}

void mesh_serial_write(const char *s)
{
    if (!s) return;
    uart_write_bytes(MESH_UART_NUM, s, strlen(s));
}

#endif

// =======================================================
// COMMON printf helper
// =======================================================

void mesh_serial_writef(const char *fmt, ...)
{
    char buf[160];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    mesh_serial_write(buf);
}