#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <stdio.h>

#define UART_DEVICE_NODE DT_NODELABEL(uart1)
#define MSG_SIZE 128

K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 4, 4);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static char rx_buf[MSG_SIZE];
static int rx_buf_pos = 0;

void serial_cb(const struct device *dev, void *user_data)
{
    uint8_t c;

    if (!uart_irq_update(uart_dev) || !uart_irq_rx_ready(uart_dev)) {
        return;
    }

    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        printk("%c", c);

        if (c == '\r' || c == '\n') {
            if (rx_buf_pos > 0) {
                rx_buf[rx_buf_pos] = '\0';
                k_msgq_put(&uart_msgq, rx_buf, K_NO_WAIT);
                rx_buf_pos = 0;
            }
        } else {
            if (rx_buf_pos < MSG_SIZE - 1) {
                rx_buf[rx_buf_pos++] = c;
            }
        }
    }
}

void uart_send(const char *data)
{
    for (size_t i = 0; i < strlen(data); i++) {
        uart_poll_out(uart_dev, data[i]);
    }
}

bool send_at_and_check(const char *cmd, char *response_out)
{
    char resp_buf[MSG_SIZE] = {0};

    printk("Sending: %s", cmd);
    uart_send(cmd);

    while (1) {
        if (k_msgq_get(&uart_msgq, &resp_buf, K_SECONDS(5)) == 0) {
            printk("\nResponse: %s\n", resp_buf);
            if (strstr(resp_buf, "+CME ERROR: 516")) {
                return false;
            } else if (strstr(resp_buf, "+QGPSLOC:")) {
                if (response_out) {
                    strcpy(response_out, resp_buf);
                }
                return true;
            }
        } else {
            printk("No response or timeout.\n");
            return false;
        }
    }
}

void extract_lat_lon(const char *gps_response, char *latitude, char *longitude)
{
    const char *start = strstr(gps_response, "+QGPSLOC:");
    if (!start) {
        strcpy(latitude, "N/A");
        strcpy(longitude, "N/A");
        return;
    }

    start += strlen("+QGPSLOC:");
    while (*start == ' ') start++;

    const char *comma = strchr(start, ',');
    if (!comma) return;
    start = comma + 1;

    comma = strchr(start, ',');
    if (!comma) return;
    strncpy(latitude, start, comma - start);
    latitude[comma - start] = '\0';
    start = comma + 1;

    comma = strchr(start, ',');
    if (!comma) return;
    strncpy(longitude, start, comma - start);
    longitude[comma - start] = '\0';
}

void main(void)
{
    printk("EC200U GPS Test Start\n");

    if (!device_is_ready(uart_dev)) {
        printk("UART device not ready!\n");
        return;
    }

    uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    k_sleep(K_SECONDS(2));

    send_at_and_check("AT\r\n", NULL);
    send_at_and_check("AT+QGPS=1\r\n", NULL);

    char gps_response[MSG_SIZE] = {0};
    char latitude[20], longitude[20];
    int attempts = 0;
    bool got_fix = false;

    while (attempts < 12) {  // Retry up to 60 seconds
        k_sleep(K_SECONDS(5));
        if (send_at_and_check("AT+QGPSLOC?\r\n", gps_response)) {
            extract_lat_lon(gps_response, latitude, longitude);
            printk("Latitude: %s\n", latitude);
            printk("Longitude: %s\n", longitude);
            got_fix = true;
            break;
        }
        attempts++;
    }

    if (!got_fix) {
        printk("No GPS fix after waiting.\n");
    }

    send_at_and_check("AT+QGPSEND\r\n", NULL);

    printk("GPS Test Complete\n");
}