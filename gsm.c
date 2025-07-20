#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/shell/shell.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_core.h>
#include <zephyr/devicetree.h>
#include <zephyr/net/ppp.h>
#include "gsm.h"
#include "aws.h"
#include <zephyr/drivers/modem/gsm_ppp.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

/* GPIO led code */
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/pm.h>
#include <zephyr/device.h>

#define LED0_NODE DT_ALIAS(led3)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* GPS related defines */
#define UART_DEVICE_NODE DT_NODELABEL(uart1)
#define MSG_SIZE 128

/* GPS message queue and UART device */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 4, 4);
static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);
static char rx_buf[MSG_SIZE];
static int rx_buf_pos = 0;

/* GSM PPP defines */
#define GSM_MODEM_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_gsm_ppp)
#define UART_NODE DT_BUS(GSM_MODEM_NODE)

static const struct device *const gsm_dev = DEVICE_DT_GET(GSM_MODEM_NODE);
static struct net_mgmt_event_callback mgmt_cb;

/* GPS UART callback function */
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

/* Send data via UART */
void uart_send(const char *data)
{
    for (size_t i = 0; i < strlen(data); i++) {
        uart_poll_out(uart_dev, data[i]);
    }
}

/* Send AT command and check response */
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

/* Extract latitude and longitude from GPS response */
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

/* GPS initialization and data fetching */
bool init_and_fetch_gps(char *latitude, char *longitude)
{
    printk("EC200U GPS Initialization Start\n");

    if (!device_is_ready(uart_dev)) {
        printk("UART device not ready for GPS!\n");
        return false;
    }

    uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    k_sleep(K_SECONDS(2));

    // Initialize GPS
    send_at_and_check("AT\r\n", NULL);
    send_at_and_check("AT+QGPS=1\r\n", NULL);

    char gps_response[MSG_SIZE] = {0};
    int attempts = 0;
    bool got_fix = false;

    printk("Waiting for GPS fix...\n");
    while (attempts < 12) {  // Retry up to 60 seconds
        k_sleep(K_SECONDS(5));
        if (send_at_and_check("AT+QGPSLOC?\r\n", gps_response)) {
            extract_lat_lon(gps_response, latitude, longitude);
            printk("GPS Fix Acquired - Latitude: %s, Longitude: %s\n", latitude, longitude);
            got_fix = true;
            break;
        }
        attempts++;
        printk("GPS attempt %d/12...\n", attempts);
    }

    if (!got_fix) {
        printk("No GPS fix after waiting 60 seconds.\n");
        strcpy(latitude, "N/A");
        strcpy(longitude, "N/A");
    }

    // Stop GPS to save power
    send_at_and_check("AT+QGPSEND\r\n", NULL);
    printk("GPS fetch complete\n");
    
    return got_fix;
}

static void event_handler(struct net_mgmt_event_callback *cb,
			  uint32_t mgmt_event, struct net_if *iface)
{
	ARG_UNUSED(cb);
	ARG_UNUSED(iface);

	if ((mgmt_event & (NET_EVENT_L4_CONNECTED
			   | NET_EVENT_L4_DISCONNECTED)) != mgmt_event) {
		return;
	}

	if (mgmt_event == NET_EVENT_L4_CONNECTED) {
		printk("Network connected");
		printk("LED 4 IS HIGH\n");
        on_net_event_l4_connected();
		gpio_pin_configure_dt(&led, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
		gpio_pin_set_dt(&led, 1);
		K_MSEC(100);
		return;
	}

	if (mgmt_event == NET_EVENT_L4_DISCONNECTED) {
		printk("Network disconnected");
		printk("LED 4 IS LOW\n");
        on_net_event_l4_disconnected();
		gpio_pin_configure_dt(&led, GPIO_OUTPUT | GPIO_ACTIVE_LOW);
		gpio_pin_set_dt(&led, 0);
		K_MSEC(100);
		k_msleep(30);
		sys_reboot();
		return;
	}
}

static void modem_on_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	printk("GSM modem on callback fired");
}

static void modem_off_cb(const struct device *dev, void *user_data)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	printk("GSM modem off callback fired");
}

void init_gsm(void)
{
    printk("Init... gsm\n");

	const struct device *uart_dev_gsm = DEVICE_DT_GET(UART_NODE);

	/* Optional register modem power callbacks */
	gsm_ppp_register_modem_power_callback(gsm_dev, modem_on_cb, modem_off_cb, NULL);

	printk("Board '%s' APN '%s' UART '%s' device %p (%s)",
		CONFIG_BOARD, CONFIG_MODEM_GSM_APN,
		uart_dev_gsm->name, uart_dev_gsm, gsm_dev->name);

	net_mgmt_init_event_callback(&mgmt_cb, event_handler,
				     NET_EVENT_L4_CONNECTED |
				     NET_EVENT_L4_DISCONNECTED);
	net_mgmt_add_event_callback(&mgmt_cb);

    printk("GSM Initialized\n");
    
    // GPS integration point - fetch GPS data after GSM initialization
    char latitude[20], longitude[20];
    bool gps_success = init_and_fetch_gps(latitude, longitude);
    
    if (gps_success) {
        printk("GPS data successfully fetched: Lat=%s, Lon=%s\n", latitude, longitude);
        // You can store these coordinates for later use in AWS or other functions
    } else {
        printk("GPS fetch failed, continuing without GPS data\n");
    }
    
    printk("Proceeding with network operations...\n");
}