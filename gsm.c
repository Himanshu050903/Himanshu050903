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

/* GSM PPP defines */
#define GSM_MODEM_NODE DT_COMPAT_GET_ANY_STATUS_OKAY(zephyr_gsm_ppp)
#define UART_NODE DT_BUS(GSM_MODEM_NODE)

static const struct device *const gsm_dev = DEVICE_DT_GET(GSM_MODEM_NODE);
static const struct device *const uart_dev = DEVICE_DT_GET(UART_NODE);
static struct net_mgmt_event_callback mgmt_cb;

/* GPS related variables */
#define MSG_SIZE 128
static char rx_buf[MSG_SIZE];
static int rx_buf_pos = 0;
static bool gps_response_ready = false;
static char gps_response_buffer[MSG_SIZE];

/* GPS UART callback function for GSM UART */
void gps_serial_cb(const struct device *dev, void *user_data)
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
                
                // Check if this is a GPS response
                if (strstr(rx_buf, "+QGPSLOC:") || strstr(rx_buf, "OK") || strstr(rx_buf, "ERROR")) {
                    strcpy(gps_response_buffer, rx_buf);
                    gps_response_ready = true;
                }
                rx_buf_pos = 0;
            }
        } else {
            if (rx_buf_pos < MSG_SIZE - 1) {
                rx_buf[rx_buf_pos++] = c;
            }
        }
    }
}

/* Send data via GSM UART */
void gps_uart_send(const char *data)
{
    for (size_t i = 0; i < strlen(data); i++) {
        uart_poll_out(uart_dev, data[i]);
    }
}

/* Send AT command and wait for response */
bool gps_send_at_and_check(const char *cmd, char *response_out)
{
    printk("GPS Command: %s", cmd);
    gps_response_ready = false;
    
    // Clear any pending responses first
    k_msleep(100);
    gps_response_ready = false;
    
    gps_uart_send(cmd);

    // Wait for response with longer timeout for GPS commands
    int timeout_loops = (strstr(cmd, "QGPSLOC") != NULL) ? 100 : 50;  // 10s for GPS, 5s for others
    
    for (int i = 0; i < timeout_loops; i++) {
        k_msleep(100);
        if (gps_response_ready) {
            printk("GPS Response: %s\n", gps_response_buffer);
            
            // For GPS location command, specifically look for +QGPSLOC response
            if (strstr(cmd, "QGPSLOC") && strstr(gps_response_buffer, "+QGPSLOC:")) {
                if (response_out) {
                    strcpy(response_out, gps_response_buffer);
                }
                return true;
            }
            // For GPS location command, check for specific errors
            else if (strstr(cmd, "QGPSLOC") && strstr(gps_response_buffer, "+CME ERROR: 516")) {
                printk("GPS not fixed yet (normal)\n");
                return false;
            }
            // For other commands, accept OK
            else if (strstr(gps_response_buffer, "OK")) {
                return true;
            } 
            // For any command, check for general errors
            else if (strstr(gps_response_buffer, "ERROR")) {
                return false;
            }
            
            // Reset and continue waiting for the right response
            gps_response_ready = false;
        }
    }
    
    printk("GPS Command timeout after %d attempts\n", timeout_loops);
    return false;
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

/* GPS initialization and data fetching using GSM UART */
bool init_and_fetch_gps(char *latitude, char *longitude)
{
    printk("=== GPS INITIALIZATION START ===\n");

    if (!device_is_ready(uart_dev)) {
        printk("UART device not ready for GPS!\n");
        return false;
    }

    // Set up UART callback for GPS communication
    uart_irq_callback_user_data_set(uart_dev, gps_serial_cb, NULL);
    uart_irq_rx_enable(uart_dev);

    k_sleep(K_SECONDS(1));

    // Initialize GPS
    if (!gps_send_at_and_check("AT\r\n", NULL)) {
        printk("GPS AT command failed\n");
        return false;
    }

    k_sleep(K_SECONDS(1));

    if (!gps_send_at_and_check("AT+QGPS=1\r\n", NULL)) {
        printk("GPS start command failed\n");
        return false;
    }

    printk("GPS started, waiting for fix...\n");
    k_sleep(K_SECONDS(2));

    char gps_response[MSG_SIZE] = {0};
    int attempts = 0;
    bool got_fix = false;

    while (attempts < 10) {  // Try for 50 seconds
        if (gps_send_at_and_check("AT+QGPSLOC?\r\n", gps_response)) {
            extract_lat_lon(gps_response, latitude, longitude);
            if (strcmp(latitude, "N/A") != 0 && strcmp(longitude, "N/A") != 0) {
                printk("*** GPS FIX ACQUIRED ***\n");
                printk("Latitude: %s\n", latitude);
                printk("Longitude: %s\n", longitude);
                got_fix = true;
                break;
            }
        }
        attempts++;
        printk("GPS attempt %d/10...\n", attempts);
        k_sleep(K_SECONDS(5));
    }

    if (!got_fix) {
        printk("No GPS fix after 50 seconds\n");
        strcpy(latitude, "N/A");
        strcpy(longitude, "N/A");
    }

    // Stop GPS to save power
    gps_send_at_and_check("AT+QGPSEND\r\n", NULL);
    printk("=== GPS FETCH COMPLETE ===\n");
    
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

	/* Optional register modem power callbacks */
	gsm_ppp_register_modem_power_callback(gsm_dev, modem_on_cb, modem_off_cb, NULL);

	printk("Board '%s' APN '%s' UART '%s' device %p (%s)",
		CONFIG_BOARD, CONFIG_MODEM_GSM_APN,
		uart_dev->name, uart_dev, gsm_dev->name);

    printk("GSM Hardware Initialized\n");
    
    // Wait for GSM modem to fully initialize and settle
    printk("Waiting for GSM modem to stabilize completely...\n");
    k_sleep(K_SECONDS(30));  // Wait for GSM to fully settle (longer delay)
    
    // *** GPS INTEGRATION POINT ***
    // Fetch GPS data AFTER GSM has stabilized
    printk("GSM stabilized, starting GPS fetch...\n");
    char latitude[20], longitude[20];
    bool gps_success = init_and_fetch_gps(latitude, longitude);
    
    if (gps_success) {
        printk("*** GPS SUCCESS: Lat=%s, Lon=%s ***\n", latitude, longitude);
    } else {
        printk("GPS fetch failed, continuing without GPS data\n");
    }
    
    // Now set up network event callbacks
	net_mgmt_init_event_callback(&mgmt_cb, event_handler,
				     NET_EVENT_L4_CONNECTED |
				     NET_EVENT_L4_DISCONNECTED);
	net_mgmt_add_event_callback(&mgmt_cb);

    printk("Network callbacks initialized, GSM init complete\n");
}