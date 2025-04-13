#include <inttypes.h>
#include "mavlink_crc.h"
#include "freertos/FreeRTOS.h"


#include "wt_alldef.h"
#include "mavlink_wt.h"
#include <string.h>


void prepare_heartbeat(mavlink_heartbeat_t* heartbeat, uint8_t* seq)
{ 
  heartbeat -> header.seq = (*seq)++;
  heartbeat -> CRC = MAVLINK_MSG_ID_HEARTBEAT_CRC;
  uint16_t CRC = crc_calculate((uint8_t*)heartbeat + 1, sizeof(mavlink_heartbeat_t) - 2);
  heartbeat -> CRC = CRC;
}  

void prepare_status(mavlink_sys_status_t* status, uint8_t* seq) 
{
  status -> header.seq = (*seq)++;
  status -> CRC = MAVLINK_MSG_ID_SYS_STATUS_CRC;
  uint16_t CRC = crc_calculate((uint8_t*)status + 1, sizeof(mavlink_sys_status_t) - 2);
  status -> CRC = CRC;
}

void prepare_rssi(mavlink_radio_status_t* radio_status, uint8_t* seq) 
{
  radio_status -> header.seq = (*seq)++;
  radio_status -> CRC = MAVLINK_MSG_ID_RADIO_STATUS_CRC;
  uint16_t CRC = crc_calculate((uint8_t*)radio_status + 1, sizeof(mavlink_radio_status_t) - 2);
  radio_status -> CRC = CRC;
}


void prepare_attitude(mavlink_attitude_t* attitude, uint8_t* seq)
{
  attitude -> header.seq = (*seq)++;
  attitude -> CRC = MAVLINK_MSG_ID_ATTITUDE_CRC;
  uint16_t CRC = crc_calculate((uint8_t*)attitude + 1, sizeof(mavlink_attitude_t) - 2);
  attitude -> CRC = CRC;
}

void prepare_battery(mavlink_battery_status_t* battery, uint8_t* seq)
{
  battery -> header.seq = (*seq)++;
  battery -> CRC = MAVLINK_MSG_ID_BATTERY_STATUS_CRC;
  uint16_t CRC = crc_calculate((uint8_t*)battery + 1, sizeof(mavlink_battery_status_t) - 2);
  battery -> CRC = CRC;
}

void prepare_gps(mavlink_global_position_int_t* gps, uint8_t* seq)
{
  gps -> header.seq = (*seq)++;
  gps -> CRC = MAVLINK_MSG_ID_GLOBAL_POSITION_INT_CRC;
  uint16_t CRC = crc_calculate((uint8_t*)gps + 1, sizeof(mavlink_global_position_int_t) - 2);
  gps -> CRC = CRC;
}

void prepare_gps_raw(mavlink_gps_raw_int_t* gps_raw, uint8_t* seq)
{
  gps_raw -> header.seq = (*seq)++;
  gps_raw -> CRC = MAVLINK_MSG_ID_GPS_RAW_INT_CRC;
  uint16_t CRC = crc_calculate((uint8_t*)gps_raw + 1, sizeof(mavlink_gps_raw_int_t) - 2);
  gps_raw -> CRC = CRC;
}

void prepare_rc_channels(mavlink_rc_channels_raw_t* rc_channels, uint8_t* seq)
{
  rc_channels -> header.seq = (*seq)++;
  rc_channels -> CRC = MAVLINK_MSG_ID_RC_CHANNELS_RAW_CRC;
  uint16_t CRC = crc_calculate((uint8_t*)rc_channels + 1, sizeof(mavlink_rc_channels_raw_t) - 2);
  rc_channels -> CRC = CRC;
}

void prepare_vfr_hud(mavlink_vfr_hud_t* vfr_hud, uint8_t* seq)
{
  vfr_hud -> header.seq = (*seq)++;
  vfr_hud -> CRC = MAVLINK_MSG_ID_VFR_HUD_CRC;
  uint16_t CRC = crc_calculate((uint8_t*)vfr_hud + 1, sizeof(mavlink_vfr_hud_t) - 2);
  vfr_hud -> CRC = CRC;
}