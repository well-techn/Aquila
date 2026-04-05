
#ifndef MAVLINK_WT_H
#define MAVLINK_WT_H

#define MAVLINK_MY_SYS_ID           (2)     //Quadrotor         
#define MAVLINK_MY_COMP_ID          (191)  //MAV_COMP_ID_ONBOARD_COMPUTER 

#define MAVLINK_V1_MESSAGE_HEADER 0xFE

//структура общей части всех пакетов
typedef struct {                      //The maximum packet length is 263 bytes for full payload.
    uint8_t magic;   ///< protocol magic marker
    uint8_t len;     ///< Length of payload
    uint8_t seq;     ///< Sequence of packet
    uint8_t sysid;   ///< ID of message sender system/aircraft
    uint8_t compid;  ///< ID of the message sender component
    uint8_t msgid;   ///< ID of message in payload
    } __attribute__((packed)) mavlink_header_t;

//для HEARTBEAT
#define MAVLINK_MSG_ID_HEARTBEAT 0
#define MAVLINK_MSG_ID_HEARTBEAT_LEN 9
#define MAVLINK_MSG_ID_HEARTBEAT_CRC 50

typedef struct {
    mavlink_header_t header;
    uint32_t custom_mode; /*<  A bitfield for use for autopilot-specific flags*/
    uint8_t type; 
    uint8_t autopilot; 
    uint8_t base_mode; /*<  System mode bitmap.*/
    uint8_t system_status; /*<  System status flag.*/
    uint8_t mavlink_version; 
    uint16_t CRC;
   } __attribute__((packed)) mavlink_heartbeat_t;

//для SYS_STATUS
#define MAVLINK_MSG_ID_SYS_STATUS 1
#define MAVLINK_MSG_ID_SYS_STATUS_LEN 31
#define MAVLINK_MSG_ID_SYS_STATUS_CRC 124

typedef struct {
     mavlink_header_t header; 
     uint32_t onboard_control_sensors_present; 
     uint32_t onboard_control_sensors_enabled; 
     uint32_t onboard_control_sensors_health; 
     uint16_t load; 
     uint16_t voltage_battery; /*< [mV] Battery voltage, UINT16_MAX: Voltage not sent by autopilot*/
     int16_t current_battery; /*< [cA] Battery current, -1: Current not sent by autopilot*/
     uint16_t drop_rate_comm; /*< [c%] Communication drop rate, (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
     uint16_t errors_comm; /*<  Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)*/
     uint16_t errors_count1; /*<  Autopilot-specific errors*/
     uint16_t errors_count2; /*<  Autopilot-specific errors*/
     uint16_t errors_count3; /*<  Autopilot-specific errors*/
     uint16_t errors_count4; /*<  Autopilot-specific errors*/
     int8_t battery_remaining; /*< [%] Battery energy remaining, -1: Battery remaining energy not sent by autopilot*/
     uint16_t CRC;
    } __attribute__((packed)) mavlink_sys_status_t;

//для radio_status
#define MAVLINK_MSG_ID_RADIO_STATUS 109
#define MAVLINK_MSG_ID_RADIO_STATUS_LEN 9
#define MAVLINK_MSG_ID_RADIO_STATUS_CRC 185

typedef struct {
    mavlink_header_t header;
    uint16_t rxerrors; /*<  Count of radio packet receive errors (since boot).*/
    uint16_t fixed; /*<  Count of error corrected radio packets (since boot).*/
    uint8_t rssi; /*<  Local (message sender) received signal strength indication in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown.*/
    uint8_t remrssi; /*<  Remote (message receiver) signal strength indication in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown.*/
    uint8_t txbuf; /*< [%] Remaining free transmitter buffer space.*/
    uint8_t noise; /*<  Local background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], UINT8_MAX: invalid/unknown.*/
    uint8_t remnoise; /*<  Remote background noise level. These are device dependent RSSI values (scale as approx 2x dB on SiK radios). Values: [0-254], UINT8_MAX: invalid/unknown.*/
    uint16_t CRC;    
} __attribute__((packed)) mavlink_radio_status_t;

//для attitude
#define MAVLINK_MSG_ID_ATTITUDE 30
#define MAVLINK_MSG_ID_ATTITUDE_LEN 28
#define MAVLINK_MSG_ID_ATTITUDE_CRC 39

typedef struct {
    mavlink_header_t header;
    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
    float roll; /*< [rad] Roll angle (-pi..+pi)*/
    float pitch; /*< [rad] Pitch angle (-pi..+pi)*/
    float yaw; /*< [rad] Yaw angle (-pi..+pi)*/
    float rollspeed; /*< [rad/s] Roll angular speed*/
    float pitchspeed; /*< [rad/s] Pitch angular speed*/
    float yawspeed; /*< [rad/s] Yaw angular speed*/
    uint16_t CRC;    
} __attribute__((packed)) mavlink_attitude_t;

//для BATTERY_STATUS
#define MAVLINK_MSG_ID_BATTERY_STATUS 147
#define MAVLINK_MSG_ID_BATTERY_STATUS_LEN 36
#define MAVLINK_MSG_ID_BATTERY_STATUS_CRC 154

typedef struct {
    mavlink_header_t header;
    int32_t current_consumed; /*< [mAh] Consumed charge, -1: autopilot does not provide consumption estimate*/
    int32_t energy_consumed; /*< [hJ] Consumed energy, -1: autopilot does not provide energy consumption estimate*/
    int16_t temperature; /*< [cdegC] Temperature of the battery. INT16_MAX for unknown temperature.*/
    uint16_t voltages[10]; /*< [mV] Battery voltage of cells 1 to 10 (see voltages_ext for cells 11-14). Cells in this field above the valid cell count for this battery should have the UINT16_MAX value. If individual cell voltages are unknown or not measured for this battery, then the overall battery voltage should be filled in cell 0, with all others set to UINT16_MAX. If the voltage of the battery is greater than (UINT16_MAX - 1), then cell 0 should be set to (UINT16_MAX - 1), and cell 1 to the remaining voltage. This can be extended to multiple cells if the total voltage is greater than 2 * (UINT16_MAX - 1).*/
    int16_t current_battery; /*< [cA] Battery current, -1: autopilot does not measure the current*/
    uint8_t id; /*<  Battery ID*/
    uint8_t battery_function; /*<  Function of the battery*/
    uint8_t type; /*<  Type (chemistry) of the battery*/
    int8_t battery_remaining; /*< [%] Remaining battery energy. Values: [0-100], -1: autopilot does not estimate the remaining battery.*/
    uint16_t CRC;    
} __attribute__((packed)) mavlink_battery_status_t;

//для GLOBAL_POSITION_INT
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_LEN 28
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT_CRC 10

typedef struct {
    mavlink_header_t header;
    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
    int32_t lat; /*< [degE7] Latitude, expressed*/
    int32_t lon; /*< [degE7] Longitude, expressed*/
    int32_t alt; /*< [mm] Altitude (MSL). Note that virtually all GPS modules provide both WGS84 and MSL.*/
    int32_t relative_alt; /*< [mm] Altitude above home*/
    int16_t vx; /*< [cm/s] Ground X Speed (Latitude, positive north)*/
    int16_t vy; /*< [cm/s] Ground Y Speed (Longitude, positive east)*/
    int16_t vz; /*< [cm/s] Ground Z Speed (Altitude, positive down)*/
    uint16_t hdg; /*< [cdeg] Vehicle heading (yaw angle), 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
    uint16_t CRC;    
} __attribute__((packed)) mavlink_global_position_int_t;   

//для GLOBAL_POSITION_INT
#define MAVLINK_MSG_ID_GPS_RAW_INT 24
#define MAVLINK_MSG_ID_GPS_RAW_INT_LEN 30
#define MAVLINK_MSG_ID_GPS_RAW_INT_CRC 24

typedef struct {
    mavlink_header_t header;
    uint64_t time_usec; /*< [us] Timestamp (UNIX Epoch time or time since system boot). The receiving end can infer timestamp format (since 1.1.1970 or since system boot) by checking for the magnitude of the number.*/
    int32_t lat; /*< [degE7] Latitude (WGS84, EGM96 ellipsoid)*/
    int32_t lon; /*< [degE7] Longitude (WGS84, EGM96 ellipsoid)*/
    int32_t alt; /*< [mm] Altitude (MSL). Positive for up. Note that virtually all GPS modules provide the MSL altitude in addition to the WGS84 altitude.*/
    uint16_t eph; /*<  GPS HDOP horizontal dilution of position (unitless * 100). If unknown, set to: UINT16_MAX*/
    uint16_t epv; /*<  GPS VDOP vertical dilution of position (unitless * 100). If unknown, set to: UINT16_MAX*/
    uint16_t vel; /*< [cm/s] GPS ground speed. If unknown, set to: UINT16_MAX*/
    uint16_t cog; /*< [cdeg] Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX*/
    uint8_t fix_type; /*<  GPS fix type.*/
    uint8_t satellites_visible; /*<  Number of satellites visible. If unknown, set to UINT8_MAX*/
    uint16_t CRC;    
} __attribute__((packed)) mavlink_gps_raw_int_t;


//для RC_CHANNELS
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW 35
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_LEN 22
#define MAVLINK_MSG_ID_RC_CHANNELS_RAW_CRC 244

typedef struct {
    mavlink_header_t header;
    uint32_t time_boot_ms; /*< [ms] Timestamp (time since system boot).*/
    uint16_t chan1_raw; /*< [us] RC channel 1 value.*/
    uint16_t chan2_raw; /*< [us] RC channel 2 value.*/
    uint16_t chan3_raw; /*< [us] RC channel 3 value.*/
    uint16_t chan4_raw; /*< [us] RC channel 4 value.*/
    uint16_t chan5_raw; /*< [us] RC channel 5 value.*/
    uint16_t chan6_raw; /*< [us] RC channel 6 value.*/
    uint16_t chan7_raw; /*< [us] RC channel 7 value.*/
    uint16_t chan8_raw; /*< [us] RC channel 8 value.*/
    uint8_t port; /*<  Servo output port (set of 8 outputs = 1 port). Flight stacks running on Pixhawk should use: 0 = MAIN, 1 = AUX.*/
    uint8_t rssi; /*<  Receive signal strength indicator in device-dependent units/scale. Values: [0-254], UINT8_MAX: invalid/unknown.*/
    uint16_t CRC;  
   } __attribute__((packed)) mavlink_rc_channels_raw_t;


//для RC_VFR_HUD
#define MAVLINK_MSG_ID_VFR_HUD 74
#define MAVLINK_MSG_ID_VFR_HUD_LEN 20
#define MAVLINK_MSG_ID_VFR_HUD_CRC 20

typedef struct {
    mavlink_header_t header;
    float airspeed; /*< Current airspeed in m/s*/
    float groundspeed; /*< Current ground speed in m/s*/
    float alt; /*< Current altitude (MSL), in meters*/
    float climb; /*< Current climb rate in meters/second*/
    int16_t heading; /*< Current heading in degrees, in compass units (0..360, 0=north)*/
    uint16_t throttle; /*< Current throttle setting in integer percent, 0 to 100*/
    uint16_t CRC;
   } __attribute__((packed)) mavlink_vfr_hud_t;


void prepare_heartbeat(mavlink_heartbeat_t* heartbeat, uint8_t* seq);
void prepare_status(mavlink_sys_status_t* status, uint8_t* seq);
void prepare_rssi(mavlink_radio_status_t* radio_status, uint8_t* seq);
void prepare_attitude(mavlink_attitude_t* attitude, uint8_t* seq);
void prepare_battery(mavlink_battery_status_t* battery, uint8_t* seq);
void prepare_gps(mavlink_global_position_int_t* gps, uint8_t* seq);
void prepare_gps_raw(mavlink_gps_raw_int_t* gps_raw, uint8_t* seq);
void prepare_rc_channels(mavlink_rc_channels_raw_t* rc_channels, uint8_t* seq);
void prepare_vfr_hud(mavlink_vfr_hud_t* vfr_hud, uint8_t* seq);

#endif

