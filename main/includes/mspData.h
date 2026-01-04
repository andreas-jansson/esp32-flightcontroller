#pragma once

#include <stdint.h>
#include <map>
#include <string>

namespace Msp{
    constexpr inline uint8_t maxMspPayload{200};

    constexpr inline uint8_t start_sym{'$'};
    constexpr inline uint8_t version_1{'M'};
    constexpr inline uint8_t version_2{'X'};
    constexpr inline uint8_t dir_vtx_to_fc{'<'};
    constexpr inline uint8_t dir_fc_to_vtx{'>'};
    constexpr inline uint8_t error{'!'};

    constexpr inline uint8_t start_sym_idx{0};
    constexpr inline uint8_t version_idx{1};
    constexpr inline uint8_t dir_idx{2};
    constexpr inline uint8_t size_idx{3};
    constexpr inline uint8_t cmd_idx{4};
    constexpr inline uint8_t payload_idx{5};




     typedef enum Cmd{
        MSP_API_VERSION = 1,
        MSP_FC_VARIANT,    //out message
        MSP_FC_VERSION,    //out message
        MSP_BOARD_INFO,    //out message
        MSP_BUILD_INFO,    //out message
        MSP_NAME = 10,     //out message          Returns user set board name - betaflight
        MSP_SET_NAME = 11,               //in message           Sets board name - betaflight

        MSP_BATTERY_CONFIG = 32,   // out message: Get battery configuration
        MSP_SET_BATTERY_CONFIG = 33,   // in message:  Set battery configuration
        MSP_MODE_RANGES = 34,   // out message: Returns all mode ranges
        MSP_SET_MODE_RANGE = 35,   // in message:  Sets a single mode range
        MSP_FEATURE_CONFIG = 36,   // out message: Get feature configuration
        MSP_SET_FEATURE_CONFIG = 37,   // in message:  Set feature configuration
        MSP_BOARD_ALIGNMENT_CONFIG = 38,   // out message: Get board alignment configuration
        MSP_SET_BOARD_ALIGNMENT_CONFIG = 39,   // in message:  Set board alignment configuration
        MSP_CURRENT_METER_CONFIG = 40,   // out message: Get current meter configuration
        MSP_SET_CURRENT_METER_CONFIG = 41,   // in message:  Set current meter configuration
        MSP_MIXER_CONFIG = 42,   // out message: Get mixer configuration
        MSP_SET_MIXER_CONFIG = 43,   // in message:  Set mixer configuration
        MSP_RX_CONFIG = 44,   // out message: Get RX configuration
        MSP_SET_RX_CONFIG = 45,   // in message:  Set RX configuration
        MSP_LED_COLORS = 46,   // out message: Get LED colors
        MSP_SET_LED_COLORS = 47,   // in message:  Set LED colors
        MSP_LED_STRIP_CONFIG = 48,   // out message: Get LED strip configuration
        MSP_SET_LED_STRIP_CONFIG = 49,   // in message:  Set LED strip configuration
        MSP_RSSI_CONFIG = 50,   // out message: Get RSSI configuration
        MSP_SET_RSSI_CONFIG = 51,   // in message:  Set RSSI configuration
        MSP_ADJUSTMENT_RANGES = 52,   // out message: Get adjustment ranges
        MSP_SET_ADJUSTMENT_RANGE = 53,   // in message:  Set adjustment range
        MSP_CF_SERIAL_CONFIG = 54,   // out message: Get Cleanflight serial configuration
        MSP_SET_CF_SERIAL_CONFIG = 55,   // in message:  Set Cleanflight serial configuration
        MSP_VOLTAGE_METER_CONFIG = 56,   // out message: Get voltage meter configuration
        MSP_SET_VOLTAGE_METER_CONFIG = 57,   // in message:  Set voltage meter configuration
        MSP_SONAR_ALTITUDE = 58,   // out message: Get sonar altitude [cm]
        MSP_PID_CONTROLLER = 59,   // out message: Get PID controller
        MSP_SET_PID_CONTROLLER = 60,   // in message:  Set PID controller
        MSP_ARMING_CONFIG = 61,   // out message: Get arming configuration
        MSP_SET_ARMING_CONFIG = 62,   // in message:  Set arming configuration

        // Baseflight MSP commands (64-89)
        MSP_RX_MAP = 64,   // out message: Get RX map (also returns number of channels total)
        MSP_SET_RX_MAP = 65,   // in message:  Set RX map, numchannels to set comes from MSP_RX_MAP
        MSP_REBOOT = 68,   // in message:  Reboot settings
        MSP_DATAFLASH_SUMMARY = 70,   // out message: Get description of dataflash chip
        MSP_DATAFLASH_READ = 71,   // out message: Get content of dataflash chip
        MSP_DATAFLASH_ERASE = 72,   // in message:  Erase dataflash chip
        MSP_FAILSAFE_CONFIG = 75,   // out message: Get failsafe settings
        MSP_SET_FAILSAFE_CONFIG = 76,   // in message:  Set failsafe settings
        MSP_RXFAIL_CONFIG = 77,   // out message: Get RX failsafe settings
        MSP_SET_RXFAIL_CONFIG = 78,   // in message:  Set RX failsafe settings
        MSP_SDCARD_SUMMARY = 79,   // out message: Get SD card state
        MSP_BLACKBOX_CONFIG = 80,   // out message: Get blackbox settings
        MSP_SET_BLACKBOX_CONFIG = 81,   // in message:  Set blackbox settings
        MSP_TRANSPONDER_CONFIG = 82,   // out message: Get transponder settings
        MSP_SET_TRANSPONDER_CONFIG = 83,   // in message:  Set transponder settings
        MSP_OSD_CONFIG = 84,   // out message: Get OSD settings
        MSP_SET_OSD_CONFIG = 85,   // in message:  Set OSD settings
        MSP_OSD_CHAR_READ = 86,   // out message: Get OSD characters
        MSP_OSD_CHAR_WRITE = 87,   // in message:  Set OSD characters
        MSP_VTX_CONFIG = 88,   // out message: Get VTX settings
        MSP_SET_VTX_CONFIG = 89,   // in message:  Set VTX settings

        // Betaflight Additional Commands (90-99)
        MSP_ADVANCED_CONFIG = 90,   // out message: Get advanced configuration
        MSP_SET_ADVANCED_CONFIG = 91,   // in message:  Set advanced configuration
        MSP_FILTER_CONFIG = 92,   // out message: Get filter configuration
        MSP_SET_FILTER_CONFIG = 93,   // in message:  Set filter configuration
        MSP_PID_ADVANCED = 94,   // out message: Get advanced PID settings
        MSP_SET_PID_ADVANCED = 95,   // in message:  Set advanced PID settings
        MSP_SENSOR_CONFIG = 96,   // out message: Get sensor configuration
        MSP_SET_SENSOR_CONFIG = 97,   // in message:  Set sensor configuration
        MSP_CAMERA_CONTROL = 98,   // in/out message: Camera control
        MSP_SET_ARMING_DISABLED = 99,   // in message:  Enable/disable arming

        // Multiwii original MSP commands (101-139)
        MSP_STATUS = 101,  // out message: Cycletime & errors_count & sensor present & box activation & current setting number
        MSP_RAW_IMU = 102,  // out message: 9 DOF
        MSP_SERVO = 103,  // out message: Servos
        MSP_MOTOR = 104,  // out message: Motors
        MSP_RC = 105,  // out message: RC channels and more
        MSP_RAW_GPS = 106,  // out message: Fix, numsat, lat, lon, alt, speed, ground course
        MSP_COMP_GPS = 107,  // out message: Distance home, direction home
        MSP_ATTITUDE = 108,  // out message: 2 angles 1 heading
        MSP_ALTITUDE = 109,  // out message: Altitude, variometer
        MSP_ANALOG = 110,  // out message: Vbat, powermetersum, rssi if available on RX
        MSP_RC_TUNING = 111,  // out message: RC rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
        MSP_PID = 112,  // out message: P I D coeff (9 are used currently)
        MSP_BOXNAMES = 116,  // out message: The aux switch names
        MSP_PIDNAMES = 117,  // out message: The PID names
        MSP_WP = 118,  // out message: Get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
        MSP_BOXIDS = 119,  // out message: Get the permanent IDs associated to BOXes
        MSP_SERVO_CONFIGURATIONS = 120,  // out message: All servo configurations
        MSP_NAV_STATUS = 121,  // out message: Returns navigation status
        MSP_NAV_CONFIG = 122,  // out message: Returns navigation parameters
        MSP_MOTOR_3D_CONFIG = 124,  // out message: Settings needed for reversible ESCs
        MSP_RC_DEADBAND = 125,  // out message: Deadbands for yaw alt pitch roll
        MSP_SENSOR_ALIGNMENT = 126,  // out message: Orientation of acc,gyro,mag
        MSP_LED_STRIP_MODECOLOR = 127,  // out message: Get LED strip mode_color settings
        MSP_VOLTAGE_METERS = 128,  // out message: Voltage (per meter)
        MSP_CURRENT_METERS = 129,  // out message: Amperage (per meter)
        MSP_BATTERY_STATE = 130,  // out message: Connected/Disconnected, Voltage, Current Used
        MSP_MOTOR_CONFIG = 131,  // out message: Motor configuration (min/max throttle, etc)
        MSP_GPS_CONFIG = 132,  // out message: GPS configuration
        MSP_COMPASS_CONFIG = 133,  // out message: Compass configuration
        MSP_ESC_SENSOR_DATA = 134,  // out message: Extra ESC data from 32-Bit ESCs (Temperature, RPM)
        MSP_GPS_RESCUE = 135,  // out message: GPS Rescue angle, returnAltitude, descentDistance, groundSpeed, sanityChecks and minSats
        MSP_GPS_RESCUE_PIDS = 136,  // out message: GPS Rescue throttleP and velocity PIDS + yaw P
        MSP_VTXTABLE_BAND = 137,  // out message: VTX table band/channel data
        MSP_VTXTABLE_POWERLEVEL = 138,  // out message: VTX table powerLevel data
        MSP_MOTOR_TELEMETRY = 139,  // out message: Per-motor telemetry data (RPM, packet stats, ESC temp, etc.)

        // Simplified tuning commands (140-145)
        MSP_SIMPLIFIED_TUNING = 140,  // out message: Get simplified tuning values and enabled state
        MSP_SET_SIMPLIFIED_TUNING = 141,  // in message:  Set simplified tuning positions and apply calculated tuning
        MSP_CALCULATE_SIMPLIFIED_PID = 142,  // out message: Calculate PID values based on sliders without saving
        MSP_CALCULATE_SIMPLIFIED_GYRO = 143,  // out message: Calculate gyro filter values based on sliders without saving
        MSP_CALCULATE_SIMPLIFIED_DTERM = 144,  // out message: Calculate D term filter values based on sliders without saving
        MSP_VALIDATE_SIMPLIFIED_TUNING = 145,  // out message: Returns array of true/false showing which simplified tuning groups match values

        // Additional non-MultiWii commands (150-166)
        MSP_STATUS_EX = 150,  // out message: Cycletime, errors_count, CPU load, sensor present etc
        MSP_UID = 160,  // out message: Unique device ID
        MSP_GPSSVINFO = 164,  // out message: Get Signal Strength (only U-Blox)
        MSP_GPSSTATISTICS = 166,  // out message: Get GPS debugging data

        // OSD specific commands (180-189)
        MSP_OSD_VIDEO_CONFIG = 180,  // out message: Get OSD video settings
        MSP_SET_OSD_VIDEO_CONFIG = 181,  // in message:  Set OSD video settings
        MSP_DISPLAYPORT = 182,  // out message: External OSD displayport mode
        MSP_COPY_PROFILE = 183,  // in message:  Copy settings between profiles
        MSP_BEEPER_CONFIG = 184,  // out message: Get beeper configuration
        MSP_SET_BEEPER_CONFIG = 185,  // in message:  Set beeper configuration
        MSP_SET_TX_INFO = 186,  // in message:  Set runtime information from TX lua scripts
        MSP_TX_INFO = 187,  // out message: Get runtime information for TX lua scripts
        MSP_SET_OSD_CANVAS = 188,  // in message:  Set OSD canvas size COLSxROWS
        MSP_OSD_CANVAS = 189,  // out message: Get OSD canvas size COLSxROWS

        // Set commands (200-229)
        MSP_SET_RAW_RC = 200,  // in message:  8 rc chan
        MSP_SET_RAW_GPS = 201,  // in message:  Fix, numsat, lat, lon, alt, speed
        MSP_SET_PID = 202,  // in message:  P I D coeff (9 are used currently)
        MSP_SET_RC_TUNING = 204,  // in message:  RC rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID, yaw expo
        MSP_ACC_CALIBRATION = 205,  // in message:  No param - calibrate accelerometer
        MSP_MAG_CALIBRATION = 206,  // in message:  No param - calibrate magnetometer
        MSP_RESET_CONF = 208,  // in message:  No param - reset settings
        MSP_SET_WP = 209,  // in message:  Sets a given WP (WP#,lat, lon, alt, flags)
        MSP_SELECT_SETTING = 210,  // in message:  Select setting number (0-2)
        MSP_SET_HEADING = 211,  // in message:  Define a new heading hold direction
        MSP_SET_SERVO_CONFIGURATION = 212,  // in message:  Servo settings
        MSP_SET_MOTOR = 214,  // in message:  PropBalance function
        MSP_SET_NAV_CONFIG = 215,  // in message:  Sets nav config parameters
        MSP_SET_MOTOR_3D_CONFIG = 217,  // in message:  Settings needed for reversible ESCs
        MSP_SET_RC_DEADBAND = 218,  // in message:  Deadbands for yaw alt pitch roll
        MSP_SET_RESET_CURR_PID = 219,  // in message:  Reset current PID profile to defaults
        MSP_SET_SENSOR_ALIGNMENT = 220,  // in message:  Set the orientation of acc,gyro,mag
        MSP_SET_LED_STRIP_MODECOLOR = 221,  // in message:  Set LED strip mode_color settings
        MSP_SET_MOTOR_CONFIG = 222,  // in message:  Motor configuration (min/max throttle, etc)
        MSP_SET_GPS_CONFIG = 223,  // in message:  GPS configuration
        MSP_SET_COMPASS_CONFIG = 224,  // in message:  Compass configuration
        MSP_SET_GPS_RESCUE = 225,  // in message:  Set GPS Rescue parameters
        MSP_SET_GPS_RESCUE_PIDS = 226,  // in message:  Set GPS Rescue PID values
        MSP_SET_VTXTABLE_BAND = 227,  // in message:  Set vtxTable band/channel data
        MSP_SET_VTXTABLE_POWERLEVEL = 228,  // in message:  Set vtxTable powerLevel data

        // Multiple MSP and special commands (230-255)
        MSP_MULTIPLE_MSP = 230,  // out message: Request multiple MSPs in one request
        MSP_MODE_RANGES_EXTRA = 238,  // out message: Extra mode range data
        MSP_SET_ACC_TRIM = 239,  // in message:  Set acc angle trim values
        MSP_ACC_TRIM = 240,  // out message: Get acc angle trim values
        MSP_SERVO_MIX_RULES = 241,  // out message: Get servo mixer configuration
        MSP_SET_SERVO_MIX_RULE = 242,  // in message:  Set servo mixer configuration
        MSP_SET_PASSTHROUGH = 245,  // in message:  Set passthrough to peripherals
        MSP_SET_RTC = 246,  // in message:  Set the RTC clock
        MSP_RTC = 247,  // out message: Get the RTC clock
        MSP_SET_BOARD_INFO = 248,  // in message:  Set the board information
        MSP_SET_SIGNATURE = 249,  // in message:  Set the signature of the board and serial number
        MSP_EEPROM_WRITE = 250,  // in message:  Write settings to EEPROM
        MSP_RESERVE_1 = 251,  // reserved for system usage
        MSP_RESERVE_2 = 252,  // reserved for system usage
        MSP_DEBUGMSG = 253,  // out message: debug string buffer
        MSP_DEBUG = 254,  // out message: debug1,debug2,debug3,debug4
        MSP_V2_FRAME = 255,  // MSPv2 payload indicator
    }cmd_e;


    typedef enum MspType{
        COMMAND,
        REQUEST,
        RESPONSE,
    } msp_type_e;

    struct cmd_item{
        cmd_e id{};
        std::string name;
        std::string desc;  
        msp_type_e type{};
    };

        
    typedef enum DpCmd{
        MSP_DP_HEARTBEAT,
        MSP_DP_RELEASE,
        MSP_DP_CLEAR_SCREEN,
        MSP_DP_WRITE_STRING,
        MSP_DP_DRAW_SCREEN,
        MSP_DP_OPTIONS,
        MSP_DP_SYS,

     }dp_cmd_e;


    inline std::map<cmd_e, cmd_item> cmds{

    { MSP_API_VERSION,                 { MSP_API_VERSION,                 "MSP_API_VERSION",                 "out message" } }, // 1
    { MSP_FC_VARIANT,                  { MSP_FC_VARIANT,                  "MSP_FC_VARIANT",                  "out message" } }, // 2
    { MSP_FC_VERSION,                  { MSP_FC_VERSION,                  "MSP_FC_VERSION",                  "out message" } }, // 3
    { MSP_BOARD_INFO,                  { MSP_BOARD_INFO,                  "MSP_BOARD_INFO",                  "out message" } }, // 4
    { MSP_BUILD_INFO,                  { MSP_BUILD_INFO,                  "MSP_BUILD_INFO",                  "out message" } }, // 5

    { MSP_NAME,                        { MSP_NAME,                        "MSP_NAME",                        "out message: Returns user set board name - betaflight" } }, // 10
    { MSP_SET_NAME,                    { MSP_SET_NAME,                    "MSP_SET_NAME",                    "in message: Sets board name - betaflight" } }, // 11

    { MSP_BATTERY_CONFIG,              { MSP_BATTERY_CONFIG,              "MSP_BATTERY_CONFIG",              "out message: Get battery configuration" } }, // 32
    { MSP_SET_BATTERY_CONFIG,          { MSP_SET_BATTERY_CONFIG,          "MSP_SET_BATTERY_CONFIG",          "in message: Set battery configuration" } }, // 33
    { MSP_MODE_RANGES,                 { MSP_MODE_RANGES,                 "MSP_MODE_RANGES",                 "out message: Returns all mode ranges" } }, // 34
    { MSP_SET_MODE_RANGE,              { MSP_SET_MODE_RANGE,              "MSP_SET_MODE_RANGE",              "in message: Sets a single mode range" } }, // 35
    { MSP_FEATURE_CONFIG,              { MSP_FEATURE_CONFIG,              "MSP_FEATURE_CONFIG",              "out message: Get feature configuration" } }, // 36
    { MSP_SET_FEATURE_CONFIG,          { MSP_SET_FEATURE_CONFIG,          "MSP_SET_FEATURE_CONFIG",          "in message: Set feature configuration" } }, // 37
    { MSP_BOARD_ALIGNMENT_CONFIG,      { MSP_BOARD_ALIGNMENT_CONFIG,      "MSP_BOARD_ALIGNMENT_CONFIG",      "out message: Get board alignment configuration" } }, // 38
    { MSP_SET_BOARD_ALIGNMENT_CONFIG,  { MSP_SET_BOARD_ALIGNMENT_CONFIG,  "MSP_SET_BOARD_ALIGNMENT_CONFIG",  "in message: Set board alignment configuration" } }, // 39
    { MSP_CURRENT_METER_CONFIG,        { MSP_CURRENT_METER_CONFIG,        "MSP_CURRENT_METER_CONFIG",        "out message: Get current meter configuration" } }, // 40
    { MSP_SET_CURRENT_METER_CONFIG,    { MSP_SET_CURRENT_METER_CONFIG,    "MSP_SET_CURRENT_METER_CONFIG",    "in message: Set current meter configuration" } }, // 41
    { MSP_MIXER_CONFIG,                { MSP_MIXER_CONFIG,                "MSP_MIXER_CONFIG",                "out message: Get mixer configuration" } }, // 42
    { MSP_SET_MIXER_CONFIG,            { MSP_SET_MIXER_CONFIG,            "MSP_SET_MIXER_CONFIG",            "in message: Set mixer configuration" } }, // 43
    { MSP_RX_CONFIG,                   { MSP_RX_CONFIG,                   "MSP_RX_CONFIG",                   "out message: Get RX configuration" } }, // 44
    { MSP_SET_RX_CONFIG,               { MSP_SET_RX_CONFIG,               "MSP_SET_RX_CONFIG",               "in message: Set RX configuration" } }, // 45
    { MSP_LED_COLORS,                  { MSP_LED_COLORS,                  "MSP_LED_COLORS",                  "out message: Get LED colors" } }, // 46
    { MSP_SET_LED_COLORS,              { MSP_SET_LED_COLORS,              "MSP_SET_LED_COLORS",              "in message: Set LED colors" } }, // 47
    { MSP_LED_STRIP_CONFIG,            { MSP_LED_STRIP_CONFIG,            "MSP_LED_STRIP_CONFIG",            "out message: Get LED strip configuration" } }, // 48
    { MSP_SET_LED_STRIP_CONFIG,        { MSP_SET_LED_STRIP_CONFIG,        "MSP_SET_LED_STRIP_CONFIG",        "in message: Set LED strip configuration" } }, // 49
    { MSP_RSSI_CONFIG,                 { MSP_RSSI_CONFIG,                 "MSP_RSSI_CONFIG",                 "out message: Get RSSI configuration" } }, // 50
    { MSP_SET_RSSI_CONFIG,             { MSP_SET_RSSI_CONFIG,             "MSP_SET_RSSI_CONFIG",             "in message: Set RSSI configuration" } }, // 51
    { MSP_ADJUSTMENT_RANGES,           { MSP_ADJUSTMENT_RANGES,           "MSP_ADJUSTMENT_RANGES",           "out message: Get adjustment ranges" } }, // 52
    { MSP_SET_ADJUSTMENT_RANGE,        { MSP_SET_ADJUSTMENT_RANGE,        "MSP_SET_ADJUSTMENT_RANGE",        "in message: Set adjustment range" } }, // 53
    { MSP_CF_SERIAL_CONFIG,            { MSP_CF_SERIAL_CONFIG,            "MSP_CF_SERIAL_CONFIG",            "out message: Get Cleanflight serial configuration" } }, // 54
    { MSP_SET_CF_SERIAL_CONFIG,        { MSP_SET_CF_SERIAL_CONFIG,        "MSP_SET_CF_SERIAL_CONFIG",        "in message: Set Cleanflight serial configuration" } }, // 55
    { MSP_VOLTAGE_METER_CONFIG,        { MSP_VOLTAGE_METER_CONFIG,        "MSP_VOLTAGE_METER_CONFIG",        "out message: Get voltage meter configuration" } }, // 56
    { MSP_SET_VOLTAGE_METER_CONFIG,    { MSP_SET_VOLTAGE_METER_CONFIG,    "MSP_SET_VOLTAGE_METER_CONFIG",    "in message: Set voltage meter configuration" } }, // 57
    { MSP_SONAR_ALTITUDE,              { MSP_SONAR_ALTITUDE,              "MSP_SONAR_ALTITUDE",              "out message: Get sonar altitude [cm]" } }, // 58
    { MSP_PID_CONTROLLER,              { MSP_PID_CONTROLLER,              "MSP_PID_CONTROLLER",              "out message: Get PID controller" } }, // 59
    { MSP_SET_PID_CONTROLLER,          { MSP_SET_PID_CONTROLLER,          "MSP_SET_PID_CONTROLLER",          "in message: Set PID controller" } }, // 60
    { MSP_ARMING_CONFIG,               { MSP_ARMING_CONFIG,               "MSP_ARMING_CONFIG",               "out message: Get arming configuration" } }, // 61
    { MSP_SET_ARMING_CONFIG,           { MSP_SET_ARMING_CONFIG,           "MSP_SET_ARMING_CONFIG",           "in message: Set arming configuration" } }, // 62

    // Baseflight MSP commands (64-89)
    { MSP_RX_MAP,                      { MSP_RX_MAP,                      "MSP_RX_MAP",                      "out message: Get RX map (also returns number of channels total)" } }, // 64
    { MSP_SET_RX_MAP,                  { MSP_SET_RX_MAP,                  "MSP_SET_RX_MAP",                  "in message: Set RX map, numchannels to set comes from MSP_RX_MAP" } }, // 65
    { MSP_REBOOT,                      { MSP_REBOOT,                      "MSP_REBOOT",                      "in message: Reboot settings" } }, // 68
    { MSP_DATAFLASH_SUMMARY,           { MSP_DATAFLASH_SUMMARY,           "MSP_DATAFLASH_SUMMARY",           "out message: Get description of dataflash chip" } }, // 70
    { MSP_DATAFLASH_READ,              { MSP_DATAFLASH_READ,              "MSP_DATAFLASH_READ",              "out message: Get content of dataflash chip" } }, // 71
    { MSP_DATAFLASH_ERASE,             { MSP_DATAFLASH_ERASE,             "MSP_DATAFLASH_ERASE",             "in message: Erase dataflash chip" } }, // 72
    { MSP_FAILSAFE_CONFIG,             { MSP_FAILSAFE_CONFIG,             "MSP_FAILSAFE_CONFIG",             "out message: Get failsafe settings" } }, // 75
    { MSP_SET_FAILSAFE_CONFIG,         { MSP_SET_FAILSAFE_CONFIG,         "MSP_SET_FAILSAFE_CONFIG",         "in message: Set failsafe settings" } }, // 76
    { MSP_RXFAIL_CONFIG,               { MSP_RXFAIL_CONFIG,               "MSP_RXFAIL_CONFIG",               "out message: Get RX failsafe settings" } }, // 77
    { MSP_SET_RXFAIL_CONFIG,           { MSP_SET_RXFAIL_CONFIG,           "MSP_SET_RXFAIL_CONFIG",           "in message: Set RX failsafe settings" } }, // 78
    { MSP_SDCARD_SUMMARY,              { MSP_SDCARD_SUMMARY,              "MSP_SDCARD_SUMMARY",              "out message: Get SD card state" } }, // 79
    { MSP_BLACKBOX_CONFIG,             { MSP_BLACKBOX_CONFIG,             "MSP_BLACKBOX_CONFIG",             "out message: Get blackbox settings" } }, // 80
    { MSP_SET_BLACKBOX_CONFIG,         { MSP_SET_BLACKBOX_CONFIG,         "MSP_SET_BLACKBOX_CONFIG",         "in message: Set blackbox settings" } }, // 81
    { MSP_TRANSPONDER_CONFIG,          { MSP_TRANSPONDER_CONFIG,          "MSP_TRANSPONDER_CONFIG",          "out message: Get transponder settings" } }, // 82
    { MSP_SET_TRANSPONDER_CONFIG,      { MSP_SET_TRANSPONDER_CONFIG,      "MSP_SET_TRANSPONDER_CONFIG",      "in message: Set transponder settings" } }, // 83
    { MSP_OSD_CONFIG,                  { MSP_OSD_CONFIG,                  "MSP_OSD_CONFIG",                  "out message: Get OSD settings" } }, // 84
    { MSP_SET_OSD_CONFIG,              { MSP_SET_OSD_CONFIG,              "MSP_SET_OSD_CONFIG",              "in message: Set OSD settings" } }, // 85
    { MSP_OSD_CHAR_READ,               { MSP_OSD_CHAR_READ,               "MSP_OSD_CHAR_READ",               "out message: Get OSD characters" } }, // 86
    { MSP_OSD_CHAR_WRITE,              { MSP_OSD_CHAR_WRITE,              "MSP_OSD_CHAR_WRITE",              "in message: Set OSD characters" } }, // 87
    { MSP_VTX_CONFIG,                  { MSP_VTX_CONFIG,                  "MSP_VTX_CONFIG",                  "out message: Get VTX settings" } }, // 88
    { MSP_SET_VTX_CONFIG,              { MSP_SET_VTX_CONFIG,              "MSP_SET_VTX_CONFIG",              "in message: Set VTX settings" } }, // 89

    // Betaflight Additional Commands (90-99)
    { MSP_ADVANCED_CONFIG,             { MSP_ADVANCED_CONFIG,             "MSP_ADVANCED_CONFIG",             "out message: Get advanced configuration" } }, // 90
    { MSP_SET_ADVANCED_CONFIG,         { MSP_SET_ADVANCED_CONFIG,         "MSP_SET_ADVANCED_CONFIG",         "in message: Set advanced configuration" } }, // 91
    { MSP_FILTER_CONFIG,               { MSP_FILTER_CONFIG,               "MSP_FILTER_CONFIG",               "out message: Get filter configuration" } }, // 92
    { MSP_SET_FILTER_CONFIG,           { MSP_SET_FILTER_CONFIG,           "MSP_SET_FILTER_CONFIG",           "in message: Set filter configuration" } }, // 93
    { MSP_PID_ADVANCED,                { MSP_PID_ADVANCED,                "MSP_PID_ADVANCED",                "out message: Get advanced PID settings" } }, // 94
    { MSP_SET_PID_ADVANCED,            { MSP_SET_PID_ADVANCED,            "MSP_SET_PID_ADVANCED",            "in message: Set advanced PID settings" } }, // 95
    { MSP_SENSOR_CONFIG,               { MSP_SENSOR_CONFIG,               "MSP_SENSOR_CONFIG",               "out message: Get sensor configuration" } }, // 96
    { MSP_SET_SENSOR_CONFIG,           { MSP_SET_SENSOR_CONFIG,           "MSP_SET_SENSOR_CONFIG",           "in message: Set sensor configuration" } }, // 97
    { MSP_CAMERA_CONTROL,              { MSP_CAMERA_CONTROL,              "MSP_CAMERA_CONTROL",              "in/out message: Camera control" } }, // 98
    { MSP_SET_ARMING_DISABLED,         { MSP_SET_ARMING_DISABLED,         "MSP_SET_ARMING_DISABLED",         "in message: Enable/disable arming" } }, // 99

    // Multiwii original MSP commands (101-139)
    { MSP_STATUS,                      { MSP_STATUS,                      "MSP_STATUS",                      "out message: Cycletime & errors_count & sensor present & box activation & current setting number" } }, // 101
    { MSP_RAW_IMU,                     { MSP_RAW_IMU,                     "MSP_RAW_IMU",                     "out message: 9 DOF" } }, // 102
    { MSP_SERVO,                       { MSP_SERVO,                       "MSP_SERVO",                       "out message: Servos" } }, // 103
    { MSP_MOTOR,                       { MSP_MOTOR,                       "MSP_MOTOR",                       "out message: Motors" } }, // 104
    { MSP_RC,                          { MSP_RC,                          "MSP_RC",                          "out message: RC channels and more" } }, // 105
    { MSP_RAW_GPS,                     { MSP_RAW_GPS,                     "MSP_RAW_GPS",                     "out message: Fix, numsat, lat, lon, alt, speed, ground course" } }, // 106
    { MSP_COMP_GPS,                    { MSP_COMP_GPS,                    "MSP_COMP_GPS",                    "out message: Distance home, direction home" } }, // 107
    { MSP_ATTITUDE,                    { MSP_ATTITUDE,                    "MSP_ATTITUDE",                    "out message: 2 angles 1 heading" } }, // 108
    { MSP_ALTITUDE,                    { MSP_ALTITUDE,                    "MSP_ALTITUDE",                    "out message: Altitude, variometer" } }, // 109
    { MSP_ANALOG,                      { MSP_ANALOG,                      "MSP_ANALOG",                      "out message: Vbat, powermetersum, rssi if available on RX" } }, // 110
    { MSP_RC_TUNING,                   { MSP_RC_TUNING,                   "MSP_RC_TUNING",                   "out message: RC rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID" } }, // 111
    { MSP_PID,                         { MSP_PID,                         "MSP_PID",                         "out message: P I D coeff (9 are used currently)" } }, // 112
    { MSP_BOXNAMES,                    { MSP_BOXNAMES,                    "MSP_BOXNAMES",                    "out message: The aux switch names" } }, // 116
    { MSP_PIDNAMES,                    { MSP_PIDNAMES,                    "MSP_PIDNAMES",                    "out message: The PID names" } }, // 117
    { MSP_WP,                          { MSP_WP,                          "MSP_WP",                          "out message: Get a WP; WP# in payload; returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold" } }, // 118
    { MSP_BOXIDS,                      { MSP_BOXIDS,                      "MSP_BOXIDS",                      "out message: Get the permanent IDs associated to BOXes" } }, // 119
    { MSP_SERVO_CONFIGURATIONS,        { MSP_SERVO_CONFIGURATIONS,        "MSP_SERVO_CONFIGURATIONS",        "out message: All servo configurations" } }, // 120
    { MSP_NAV_STATUS,                  { MSP_NAV_STATUS,                  "MSP_NAV_STATUS",                  "out message: Returns navigation status" } }, // 121
    { MSP_NAV_CONFIG,                  { MSP_NAV_CONFIG,                  "MSP_NAV_CONFIG",                  "out message: Returns navigation parameters" } }, // 122
    { MSP_MOTOR_3D_CONFIG,             { MSP_MOTOR_3D_CONFIG,             "MSP_MOTOR_3D_CONFIG",             "out message: Settings needed for reversible ESCs" } }, // 124
    { MSP_RC_DEADBAND,                 { MSP_RC_DEADBAND,                 "MSP_RC_DEADBAND",                 "out message: Deadbands for yaw alt pitch roll" } }, // 125
    { MSP_SENSOR_ALIGNMENT,            { MSP_SENSOR_ALIGNMENT,            "MSP_SENSOR_ALIGNMENT",            "out message: Orientation of acc,gyro,mag" } }, // 126
    { MSP_LED_STRIP_MODECOLOR,         { MSP_LED_STRIP_MODECOLOR,         "MSP_LED_STRIP_MODECOLOR",         "out message: Get LED strip mode_color settings" } }, // 127
    { MSP_VOLTAGE_METERS,              { MSP_VOLTAGE_METERS,              "MSP_VOLTAGE_METERS",              "out message: Voltage (per meter)" } }, // 128
    { MSP_CURRENT_METERS,              { MSP_CURRENT_METERS,              "MSP_CURRENT_METERS",              "out message: Amperage (per meter)" } }, // 129
    { MSP_BATTERY_STATE,               { MSP_BATTERY_STATE,               "MSP_BATTERY_STATE",               "out message: Connected/Disconnected, Voltage, Current Used" } }, // 130
    { MSP_MOTOR_CONFIG,                { MSP_MOTOR_CONFIG,                "MSP_MOTOR_CONFIG",                "out message: Motor configuration (min/max throttle, etc)" } }, // 131
    { MSP_GPS_CONFIG,                  { MSP_GPS_CONFIG,                  "MSP_GPS_CONFIG",                  "out message: GPS configuration" } }, // 132
    { MSP_COMPASS_CONFIG,              { MSP_COMPASS_CONFIG,              "MSP_COMPASS_CONFIG",              "out message: Compass configuration" } }, // 133
    { MSP_ESC_SENSOR_DATA,             { MSP_ESC_SENSOR_DATA,             "MSP_ESC_SENSOR_DATA",             "out message: Extra ESC data (Temp, RPM)" } }, // 134
    { MSP_GPS_RESCUE,                  { MSP_GPS_RESCUE,                  "MSP_GPS_RESCUE",                  "out message: GPS Rescue params (angle/alt/descent/speed/checks/minSats)" } }, // 135
    { MSP_GPS_RESCUE_PIDS,             { MSP_GPS_RESCUE_PIDS,             "MSP_GPS_RESCUE_PIDS",             "out message: GPS Rescue throttle/velocity PIDs + yaw P" } }, // 136
    { MSP_VTXTABLE_BAND,               { MSP_VTXTABLE_BAND,               "MSP_VTXTABLE_BAND",               "out message: VTX table band/channel data" } }, // 137
    { MSP_VTXTABLE_POWERLEVEL,         { MSP_VTXTABLE_POWERLEVEL,         "MSP_VTXTABLE_POWERLEVEL",         "out message: VTX table powerLevel data" } }, // 138
    { MSP_MOTOR_TELEMETRY,             { MSP_MOTOR_TELEMETRY,             "MSP_MOTOR_TELEMETRY",             "out message: Per-motor telemetry (RPM, packet stats, temp, etc.)" } }, // 139

    // Simplified tuning commands (140-145)
    { MSP_SIMPLIFIED_TUNING,           { MSP_SIMPLIFIED_TUNING,           "MSP_SIMPLIFIED_TUNING",           "out message: Get simplified tuning values and enabled state" } }, // 140
    { MSP_SET_SIMPLIFIED_TUNING,       { MSP_SET_SIMPLIFIED_TUNING,       "MSP_SET_SIMPLIFIED_TUNING",       "in message: Set simplified tuning positions and apply calculated tuning" } }, // 141
    { MSP_CALCULATE_SIMPLIFIED_PID,    { MSP_CALCULATE_SIMPLIFIED_PID,    "MSP_CALCULATE_SIMPLIFIED_PID",    "out message: Calculate PID values based on sliders without saving" } }, // 142
    { MSP_CALCULATE_SIMPLIFIED_GYRO,   { MSP_CALCULATE_SIMPLIFIED_GYRO,   "MSP_CALCULATE_SIMPLIFIED_GYRO",   "out message: Calculate gyro filter values based on sliders without saving" } }, // 143
    { MSP_CALCULATE_SIMPLIFIED_DTERM,  { MSP_CALCULATE_SIMPLIFIED_DTERM,  "MSP_CALCULATE_SIMPLIFIED_DTERM",  "out message: Calculate D term filter values based on sliders without saving" } }, // 144
    { MSP_VALIDATE_SIMPLIFIED_TUNING,  { MSP_VALIDATE_SIMPLIFIED_TUNING,  "MSP_VALIDATE_SIMPLIFIED_TUNING",  "out message: Returns array of true/false for simplified tuning group matches" } }, // 145

    // Additional non-MultiWii commands (150-166)
    { MSP_STATUS_EX,                   { MSP_STATUS_EX,                   "MSP_STATUS_EX",                   "out message: Cycletime, errors_count, CPU load, sensor present etc" } }, // 150
    { MSP_UID,                         { MSP_UID,                         "MSP_UID",                         "out message: Unique device ID" } }, // 160
    { MSP_GPSSVINFO,                   { MSP_GPSSVINFO,                   "MSP_GPSSVINFO",                   "out message: Get Signal Strength (only U-Blox)" } }, // 164
    { MSP_GPSSTATISTICS,               { MSP_GPSSTATISTICS,               "MSP_GPSSTATISTICS",               "out message: Get GPS debugging data" } }, // 166

    // OSD specific commands (180-189)
    { MSP_OSD_VIDEO_CONFIG,            { MSP_OSD_VIDEO_CONFIG,            "MSP_OSD_VIDEO_CONFIG",            "out message: Get OSD video settings" } }, // 180
    { MSP_SET_OSD_VIDEO_CONFIG,        { MSP_SET_OSD_VIDEO_CONFIG,        "MSP_SET_OSD_VIDEO_CONFIG",        "in message: Set OSD video settings" } }, // 181
    { MSP_DISPLAYPORT,                 { MSP_DISPLAYPORT,                 "MSP_DISPLAYPORT",                 "out message: External OSD displayport mode" } }, // 182
    { MSP_COPY_PROFILE,                { MSP_COPY_PROFILE,                "MSP_COPY_PROFILE",                "in message: Copy settings between profiles" } }, // 183
    { MSP_BEEPER_CONFIG,               { MSP_BEEPER_CONFIG,               "MSP_BEEPER_CONFIG",               "out message: Get beeper configuration" } }, // 184
    { MSP_SET_BEEPER_CONFIG,           { MSP_SET_BEEPER_CONFIG,           "MSP_SET_BEEPER_CONFIG",           "in message: Set beeper configuration" } }, // 185
    { MSP_SET_TX_INFO,                 { MSP_SET_TX_INFO,                 "MSP_SET_TX_INFO",                 "in message: Set runtime information from TX lua scripts" } }, // 186
    { MSP_TX_INFO,                     { MSP_TX_INFO,                     "MSP_TX_INFO",                     "out message: Get runtime information for TX lua scripts" } }, // 187
    { MSP_SET_OSD_CANVAS,              { MSP_SET_OSD_CANVAS,              "MSP_SET_OSD_CANVAS",              "in message: Set OSD canvas size COLSxROWS" } }, // 188
    { MSP_OSD_CANVAS,                  { MSP_OSD_CANVAS,                  "MSP_OSD_CANVAS",                  "out message: Get OSD canvas size COLSxROWS" } }, // 189

    // Set commands (200-229)
    { MSP_SET_RAW_RC,                  { MSP_SET_RAW_RC,                  "MSP_SET_RAW_RC",                  "in message: 8 rc chan" } }, // 200
    { MSP_SET_RAW_GPS,                 { MSP_SET_RAW_GPS,                 "MSP_SET_RAW_GPS",                 "in message: Fix, numsat, lat, lon, alt, speed" } }, // 201
    { MSP_SET_PID,                     { MSP_SET_PID,                     "MSP_SET_PID",                     "in message: P I D coeff (9 are used currently)" } }, // 202
    { MSP_SET_RC_TUNING,               { MSP_SET_RC_TUNING,               "MSP_SET_RC_TUNING",               "in message: RC rate/expo/rates/dyn throttle PID/yaw expo" } }, // 204
    { MSP_ACC_CALIBRATION,             { MSP_ACC_CALIBRATION,             "MSP_ACC_CALIBRATION",             "in message: No param - calibrate accelerometer" } }, // 205
    { MSP_MAG_CALIBRATION,             { MSP_MAG_CALIBRATION,             "MSP_MAG_CALIBRATION",             "in message: No param - calibrate magnetometer" } }, // 206
    { MSP_RESET_CONF,                  { MSP_RESET_CONF,                  "MSP_RESET_CONF",                  "in message: No param - reset settings" } }, // 208
    { MSP_SET_WP,                      { MSP_SET_WP,                      "MSP_SET_WP",                      "in message: Sets a given WP (WP#, lat, lon, alt, flags)" } }, // 209
    { MSP_SELECT_SETTING,              { MSP_SELECT_SETTING,              "MSP_SELECT_SETTING",              "in message: Select setting number (0-2)" } }, // 210
    { MSP_SET_HEADING,                 { MSP_SET_HEADING,                 "MSP_SET_HEADING",                 "in message: Define a new heading hold direction" } }, // 211
    { MSP_SET_SERVO_CONFIGURATION,     { MSP_SET_SERVO_CONFIGURATION,     "MSP_SET_SERVO_CONFIGURATION",     "in message: Servo settings" } }, // 212
    { MSP_SET_MOTOR,                   { MSP_SET_MOTOR,                   "MSP_SET_MOTOR",                   "in message: PropBalance function" } }, // 214
    { MSP_SET_NAV_CONFIG,              { MSP_SET_NAV_CONFIG,              "MSP_SET_NAV_CONFIG",              "in message: Sets nav config parameters" } }, // 215
    { MSP_SET_MOTOR_3D_CONFIG,         { MSP_SET_MOTOR_3D_CONFIG,         "MSP_SET_MOTOR_3D_CONFIG",         "in message: Settings needed for reversible ESCs" } }, // 217
    { MSP_SET_RC_DEADBAND,             { MSP_SET_RC_DEADBAND,             "MSP_SET_RC_DEADBAND",             "in message: Deadbands for yaw alt pitch roll" } }, // 218
    { MSP_SET_RESET_CURR_PID,          { MSP_SET_RESET_CURR_PID,          "MSP_SET_RESET_CURR_PID",          "in message: Reset current PID profile to defaults" } }, // 219
    { MSP_SET_SENSOR_ALIGNMENT,        { MSP_SET_SENSOR_ALIGNMENT,        "MSP_SET_SENSOR_ALIGNMENT",        "in message: Set the orientation of acc,gyro,mag" } }, // 220
    { MSP_SET_LED_STRIP_MODECOLOR,     { MSP_SET_LED_STRIP_MODECOLOR,     "MSP_SET_LED_STRIP_MODECOLOR",     "in message: Set LED strip mode_color settings" } }, // 221
    { MSP_SET_MOTOR_CONFIG,            { MSP_SET_MOTOR_CONFIG,            "MSP_SET_MOTOR_CONFIG",            "in message: Motor configuration (min/max throttle, etc)" } }, // 222
    { MSP_SET_GPS_CONFIG,              { MSP_SET_GPS_CONFIG,              "MSP_SET_GPS_CONFIG",              "in message: Set GPS configuration" } }, // 223
    { MSP_SET_COMPASS_CONFIG,          { MSP_SET_COMPASS_CONFIG,          "MSP_SET_COMPASS_CONFIG",          "in message: Set compass configuration" } }, // 224
    { MSP_SET_GPS_RESCUE,              { MSP_SET_GPS_RESCUE,              "MSP_SET_GPS_RESCUE",              "in message: Set GPS Rescue parameters" } }, // 225
    { MSP_SET_GPS_RESCUE_PIDS,         { MSP_SET_GPS_RESCUE_PIDS,         "MSP_SET_GPS_RESCUE_PIDS",         "in message: Set GPS Rescue PID values" } }, // 226
    { MSP_SET_VTXTABLE_BAND,           { MSP_SET_VTXTABLE_BAND,           "MSP_SET_VTXTABLE_BAND",           "in message: Set vtxTable band/channel data" } }, // 227
    { MSP_SET_VTXTABLE_POWERLEVEL,     { MSP_SET_VTXTABLE_POWERLEVEL,     "MSP_SET_VTXTABLE_POWERLEVEL",     "in message: Set vtxTable powerLevel data" } }, // 228

    // Multiple MSP and special commands (230-255)
    { MSP_MULTIPLE_MSP,                { MSP_MULTIPLE_MSP,                "MSP_MULTIPLE_MSP",                "out message: Request multiple MSPs in one request" } }, // 230
    { MSP_MODE_RANGES_EXTRA,           { MSP_MODE_RANGES_EXTRA,           "MSP_MODE_RANGES_EXTRA",           "out message: Extra mode range data" } }, // 238
    { MSP_SET_ACC_TRIM,                { MSP_SET_ACC_TRIM,                "MSP_SET_ACC_TRIM",                "in message: Set acc angle trim values" } }, // 239
    { MSP_ACC_TRIM,                    { MSP_ACC_TRIM,                    "MSP_ACC_TRIM",                    "out message: Get acc angle trim values" } }, // 240
    { MSP_SERVO_MIX_RULES,             { MSP_SERVO_MIX_RULES,             "MSP_SERVO_MIX_RULES",             "out message: Get servo mixer configuration" } }, // 241
    { MSP_SET_SERVO_MIX_RULE,          { MSP_SET_SERVO_MIX_RULE,          "MSP_SET_SERVO_MIX_RULE",          "in message: Set servo mixer configuration" } }, // 242
    { MSP_SET_PASSTHROUGH,             { MSP_SET_PASSTHROUGH,             "MSP_SET_PASSTHROUGH",             "in message: Set passthrough to peripherals" } }, // 245
    { MSP_SET_RTC,                     { MSP_SET_RTC,                     "MSP_SET_RTC",                     "in message: Set the RTC clock" } }, // 246
    { MSP_RTC,                         { MSP_RTC,                         "MSP_RTC",                         "out message: Get the RTC clock" } }, // 247
    { MSP_SET_BOARD_INFO,              { MSP_SET_BOARD_INFO,              "MSP_SET_BOARD_INFO",              "in message: Set the board information" } }, // 248
    { MSP_SET_SIGNATURE,               { MSP_SET_SIGNATURE,               "MSP_SET_SIGNATURE",               "in message: Set the signature of the board and serial number" } }, // 249
    { MSP_EEPROM_WRITE,                { MSP_EEPROM_WRITE,                "MSP_EEPROM_WRITE",                "in message: Write settings to EEPROM" } }, // 250
    { MSP_RESERVE_1,                   { MSP_RESERVE_1,                   "MSP_RESERVE_1",                   "reserved for system usage" } }, // 251
    { MSP_RESERVE_2,                   { MSP_RESERVE_2,                   "MSP_RESERVE_2",                   "reserved for system usage" } }, // 252
    { MSP_DEBUGMSG,                    { MSP_DEBUGMSG,                    "MSP_DEBUGMSG",                    "out message: debug string buffer" } }, // 253
    { MSP_DEBUG,                       { MSP_DEBUG,                       "MSP_DEBUG",                       "out message: debug1,debug2,debug3,debug4" } }, // 254
    { MSP_V2_FRAME,                    { MSP_V2_FRAME,                    "MSP_V2_FRAME",                    "MSPv2 payload indicator" } }, // 255
    };




    struct msp_status_t {
        uint16_t cycleTime;
        uint16_t i2cErrorCounter;
        uint16_t sensor;                    // MSP_STATUS_SENSOR_...
        uint32_t flightModeFlags;           // see getActiveModes()
        uint8_t  configProfileIndex;        // Connection Mode
    } __attribute__ ((packed));

    // MSP_STATUS_EX reply
    struct msp_status_ex_t {
      uint16_t cycleTime;
      uint16_t i2cErrorCounter;
      uint16_t sensor;                    // MSP_STATUS_SENSOR_...
      uint32_t flightModeFlags;           // see getActiveModes()
      uint8_t  configProfileIndex;
      uint16_t averageSystemLoadPercent;  // 0...100
      uint16_t armingFlags;
      uint8_t  accCalibrationAxisFlags;
    } __attribute__ ((packed));

    struct msp_rc_t {
    uint16_t channelValue[11]; // TODO check chanels used
    } __attribute__ ((packed));

    // MSP_ANALOG reply
    struct msp_analog_t {
      uint16_t  vbat;     // Converted to int x 100
      uint16_t mAhDrawn; // milliamp hours drawn from battery
      uint16_t rssi;     // 0..1023
      int16_t  amperage; // send amperage in 0.01 A steps, range is -320A to 320A  
    } __attribute__ ((packed));

    // MSP_RC_TUNING reply
    struct msp_rc_tuning_t {
    uint8_t  rcRate8;  // no longer used
    uint8_t  rcExpo8;
    uint8_t  rates[3]; // R,P,Y
    uint8_t  dynThrPID;
    uint8_t  thrMid8;
    uint8_t  thrExpo8;
    uint16_t tpa_breakpoint;
    uint8_t  rcYawExpo8;  
} __attribute__ ((packed));

// MSP_PID reply
struct msp_pid_t {
  uint8_t roll[3];     // 0=P, 1=I, 2=D
  uint8_t pitch[3];    // 0=P, 1=I, 2=D
  uint8_t yaw[3];      // 0=P, 1=I, 2=D
  uint8_t pos_z[3];    // 0=P, 1=I, 2=D
  uint8_t pos_xy[3];   // 0=P, 1=I, 2=D
  uint8_t vel_xy[3];   // 0=P, 1=I, 2=D
  uint8_t surface[3];  // 0=P, 1=I, 2=D
  uint8_t level[3];    // 0=P, 1=I, 2=D
  uint8_t heading[3];  // 0=P, 1=I, 2=D
  uint8_t vel_z[3];    // 0=P, 1=I, 2=D
} __attribute__ ((packed));


// MSP_FC_VERSION reply
struct msp_fc_version_t {
  uint8_t versionMajor;
  uint8_t versionMinor;
  uint8_t versionPatchLevel;
} __attribute__ ((packed));

// MSP_FILTER_CONFIG based on betaflight
struct msp_filter_config {
  uint8_t gyro_lpf1_static_hz;
  uint16_t dterm_lpf1_static_hz;
  uint16_t yaw_lowpass_hz;
  uint16_t gyro_soft_notch_hz_1;
  uint16_t gyro_soft_notch_cutoff_1;
  uint16_t dterm_notch_hz;
  uint16_t dterm_notch_cutoff;
  uint16_t gyro_soft_notch_hz_2;
  uint16_t gyro_soft_notch_cutoff_2;
  uint8_t dterm_lpf1_type;
  uint8_t gyro_hardware_lpf;
  uint8_t gyro_32khz_hardware_lpf; // DEPRECATED: gyro_32khz_hardware_lpf
  uint16_t gyro_lpf1_static_hz_2;
  uint16_t gyro_lpf2_static_hz;
  uint8_t gyro_lpf1_type;
  uint8_t gyro_lpf2_type;
  uint16_t dterm_lpf2_static_hz;
        // Added in MSP API 1.41
  uint8_t dterm_lpf2_type;
#if defined(USE_DYN_LPF)
        uint16_t gyro_lpf1_dyn_min_hz;
        uint16_t gyro_lpf1_dyn_max_hz;
        uint16_t dterm_lpf1_dyn_min_hz;
        uint16_t dterm_lpf1_dyn_max_hz;
#else
        uint16_t filler1;
        uint16_t filler2;
        uint16_t filler3;
        uint16_t filler4;
#endif
        // Added in MSP API 1.42
#if defined(USE_DYN_NOTCH_FILTER)
        uint8_t filler;  // DEPRECATED 1.43: dyn_notch_range
        uint8_t filler;  // DEPRECATED 1.44: dyn_notch_width_percent
        uint16_t dyn_notch_q;
        uint16_t dyn_notch_min_hz;
#else
        uint8_t filler5;
        uint8_t filler6;
        uint16_t filler7;
        uint16_t filler8;
#endif
#if defined(USE_RPM_FILTER)
        uint8_t rpm_filter_harmonics;
        uint8_t rpm_filter_min_hz;
#else
        uint8_t filler9;
        uint8_t filler10;
#endif
#if defined(USE_DYN_NOTCH_FILTER)
        // Added in MSP API 1.43
        uint16_t dyn_notch_max_hz;
#else
        uint16_t filler11;
#endif
#if defined(USE_DYN_LPF)
        // Added in MSP API 1.44
        sbufWriteU8(dst, currentPidProfile->dterm_lpf1_dyn_expo);
#else
        uint8_t filler12;
#endif
#if defined(USE_DYN_NOTCH_FILTER)
        sbufWriteU8(dst, dynNotchConfig()->dyn_notch_count);
#else
        uint8_t filler13;
#endif
} __attribute__ ((packed));

        struct msp_pid_advanced{
                uint16_t filler1;
                uint16_t filler2;
                uint16_t filler3; // was pidProfile.yaw_p_limit
                uint8_t filler4; // reserved
                uint8_t filler5; // was vbatPidCompensation
        #if defined(USE_FEEDFORWARD)
                uint8_t feedforward_transition;
        #else
                uint8_t filler6; 
        #endif
                uint8_t filler7; // was low byte of currentPidProfile->dtermSetpointWeight
                uint8_t filler8; // reserved
                uint8_t filler9; // reserved
                uint8_t filler10; // reserved
                uint16_t rateAccelLimit;
                uint16_t yawRateAccelLimit;
                uint8_t angle_limit;
                uint8_t filler11; // was pidProfile.levelSensitivity
                uint16_t filler12; // was currentPidProfile->itermThrottleThreshold
                uint16_t anti_gravity_gain;
                uint16_t filler13; // was currentPidProfile->dtermSetpointWeight
                uint8_t iterm_rotation;
                uint8_t filler14; // was currentPidProfile->smart_feedforward
        #if defined(USE_ITERM_RELAX)
                uint8_t iterm_relax;
                uint8_t iterm_relax_type;
        #else
                uint8_t filler15;
                uint8_t filler16;
        #endif
        #if defined(USE_ABSOLUTE_CONTROL)
                uint8_t abs_control_gain;
        #else
                uint8_t filler17;
        #endif
        #if defined(USE_THROTTLE_BOOST)
                uint8_t throttle_boost;
        #else
                uint8_t filler18;
        #endif
        #if defined(USE_ACRO_TRAINER)
                uint8_t acro_trainer_angle_limit;
        #else
                uint8_t filler19;
        #endif
                uint16_t pidRoll; // roll float?
                uint16_t pidPitch; // pitch float? 
                uint16_t pidYaw; // yaw float?
                uint8_t filler20; // was currentPidProfile->antiGravityMode
        #ifdef USE_D_MAX
                uint8_t d_max_roll);
                uint8_t d_max_pitch);
                uint8_t d_max_yaw);
                uint8_t d_max_gain);
                uint8_t d_max_advance);
        #else
                uint8_t filler21;
                uint8_t filler22;
                uint8_t filler23;
                uint8_t filler24;
                uint8_t filler25;
        #endif
        #if defined(USE_INTEGRATED_YAW_CONTROL)
                sbufWriteU8(dst, currentPidProfile->use_integrated_yaw);
                sbufWriteU8(dst, currentPidProfile->integrated_yaw_relax);
        #else
                uint8_t filler26;
                uint8_t filler27;
        #endif
        #if defined(USE_ITERM_RELAX)
                // Added in MSP API 1.42
                uint8_t iterm_relax_cutoff;
        #else
                uint8_t filler28;
        #endif
                // Added in MSP API 1.43
                uint8_t motor_output_limit;
                uint8_t auto_profile_cell_count;
        #if defined(USE_DYN_IDLE)
                uint8_t dyn_idle_min_rpm;
        #else
                uint8_t filler29;
        #endif
                // Added in MSP API 1.44
        #if defined(USE_FEEDFORWARD)
                uint8_t feedforward_averaging;
                uint8_t feedforward_smooth_factor;
                uint8_t feedforward_boost;
                uint8_t feedforward_max_rate_limit;
                uint8_t feedforward_jitter_factor;
        #else
                uint8_t filler30;
                uint8_t filler31;
                uint8_t filler32;
                uint8_t filler33;
                uint8_t filler34;
        #endif
        #if defined(USE_BATTERY_VOLTAGE_SAG_COMPENSATION)
                sbufWriteU8(dst, currentPidProfile->vbat_sag_compensation);
        #else
                uint8_t filler35;
        #endif
        #if defined(USE_THRUST_LINEARIZATION)
                sbufWriteU8(dst, currentPidProfile->thrustLinearization);
        #else
                uint8_t filler36;
        #endif
                uint8_t tpa_mode;
                uint8_t tpa_rate;
                uint16_t tpa_breakpoint;   // was currentControlRateProfile->tpa_breakpoint
        } __attribute__ ((packed));
        
        struct displayPortMspCommand_s {
            uint8_t command;
            uint8_t row;
            uint8_t col;
            uint8_t attribute;
            uint8_t data[64]; // OSD_CHAR_BYTES 
        } __attribute__((packed));
        
//MSP_BATTERY_STATE
struct msp_battery_state_s{
    uint8_t cellCount;
    uint16_t capacityMah;
    uint8_t voltageLegacy; 
    uint16_t mahDrawn;
    int16_t current;
    uint8_t alerts;
    uint16_t voltage;
}__attribute__((packed));



// from betaflight
typedef enum {
// ARM flag
BOXARM = 0,
// FLIGHT_MODE
BOXANGLE,
BOXHORIZON,
BOXMAG,
BOXALTHOLD,
BOXHEADFREE,
BOXCHIRP,
BOXPASSTHRU,
BOXFAILSAFE,
BOXPOSHOLD,
BOXGPSRESCUE,
BOXID_FLIGHTMODE_LAST = BOXGPSRESCUE,

// When new flight modes are added, the parameter group version for 'modeActivationConditions' in src/main/fc/rc_modes.c has to be incremented to ensure that the RC modes configuration is reset.

// RCMODE flags
BOXANTIGRAVITY,
BOXHEADADJ,
BOXCAMSTAB,
BOXBEEPERON,
BOXLEDLOW,
BOXCALIB,
BOXOSD,
BOXTELEMETRY,
BOXSERVO1,
BOXSERVO2,
BOXSERVO3,
BOXBLACKBOX,
BOXAIRMODE,
BOX3D,
BOXFPVANGLEMIX,
BOXBLACKBOXERASE,
BOXCAMERA1,
BOXCAMERA2,
BOXCAMERA3,
BOXCRASHFLIP,
BOXPREARM,
BOXBEEPGPSCOUNT,
BOXVTXPITMODE,
BOXPARALYZE,
BOXUSER1,
BOXUSER2,
BOXUSER3,
BOXUSER4,
BOXPIDAUDIO,
BOXACROTRAINER,
BOXVTXCONTROLDISABLE,
BOXLAUNCHCONTROL,
BOXMSPOVERRIDE,
BOXSTICKCOMMANDDISABLE,
BOXBEEPERMUTE,
BOXREADY,
BOXLAPTIMERRESET,
CHECKBOX_ITEM_COUNT
} boxId_e;
}
