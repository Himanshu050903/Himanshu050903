#ifndef __GSM_H__
#define __GSM_H__

#include <stdbool.h>

void init_gsm(void);
void start_sim_polling(void);

/* GPS related functions */
bool init_and_fetch_gps(char *latitude, char *longitude);
void extract_lat_lon(const char *gps_response, char *latitude, char *longitude);
bool send_at_and_check(const char *cmd, char *response_out);
void uart_send(const char *data);

#endif