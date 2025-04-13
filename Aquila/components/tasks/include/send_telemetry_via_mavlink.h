#ifndef SEND_TELEMETRY_VIA_MAVLINK_H
#define SEND_TELEMETRY_VIA_MAVLINK_H

void mavlink_uart_config(void);
void send_telemetry_via_mavlink(void * pvParameters);

#endif