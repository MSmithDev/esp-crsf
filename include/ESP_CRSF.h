#include "stdio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_intr_alloc.h"
#include <string.h>
#include "freertos/queue.h"
#include "freertos/semphr.h"

/**
 * @brief struct to hold the configuration of the CRSF
 *
 * @param uart_num the uart controller number to use
 * @param tx_pin the tx pin of the esp uart
 * @param rx_pin the rx pin of the esp uart
 *
 */
typedef struct
{
    uint8_t uart_num;
    uint8_t tx_pin;
    uint8_t rx_pin;
} crsf_config_t;




/**
 * @brief structure for handling 16 channels of data, 11 bits each. Which channel is used depends on transmitter setting
 *
 * @return typedef struct
 */
typedef struct __attribute__((packed))
{
    unsigned ch1 : 11;
    unsigned ch2 : 11;
    unsigned ch3 : 11;
    unsigned ch4 : 11;
    unsigned ch5 : 11;
    unsigned ch6 : 11;
    unsigned ch7 : 11;
    unsigned ch8 : 11;
    unsigned ch9 : 11;
    unsigned ch10 : 11;
    unsigned ch11 : 11;
    unsigned ch12 : 11;
    unsigned ch13 : 11;
    unsigned ch14 : 11;
    unsigned ch15 : 11;
    unsigned ch16 : 11;
} crsf_channels_t;

/**
 * @brief struct for battery data telemetry
 *
 * @param voltage the voltage of the battery in 10*V (1 = 0.1V)
 * @param current the current of the battery in 10*A (1 = 0.1A)
 * @param capacity the capacity of the battery in mah
 * @param remaining the remaining percentage of the battery
 *
 */
typedef struct __attribute__((packed))
{
    unsigned voltage : 16;  // V * 10 big endian
    unsigned current : 16;  // A * 10 big endian
    unsigned capacity : 24; // mah big endian
    unsigned remaining : 8; // %
} crsf_battery_t;

/**
 * @brief struct for GPS data telemetry
 *
 * @param latitude int32 the latitude of the GPS in degree / 10,000,000 big endian
 * @param longitude int32 the longitude of the GPS in degree / 10,000,000 big endian
 * @param groundspeed uint16 the groundspeed of the GPS in km/h / 10 big endian
 * @param heading uint16 the heading of the GPS in degree/100 big endian
 * @param altitude uint16 the altitude of the GPS in meters, +1000m big endian
 * @param satellites uint8 the number of satellites
 *
 */
typedef struct __attribute__((packed))
{
    int32_t latitude;     // degree / 10,000,000 big endian
    int32_t longitude;    // degree / 10,000,000 big endian
    uint16_t groundspeed; // km/h / 10 big endian
    uint16_t heading;     // GPS heading, degree/100 big endian
    uint16_t altitude;    // meters, +1000m big endian
    uint8_t satellites;   // satellites
} crsf_gps_t;






typedef struct __attribute__((packed)) {
    uint8_t byte0;
    uint8_t byte1;
    uint8_t byte2;
} int24_t;
// Helper functions to convert between int32_t and int24_t
static inline int32_t int24_to_int32(int24_t val) {
    int32_t result = (val.byte2 << 16) | (val.byte1 << 8) | val.byte0;
    // Sign extend if negative (bit 23 is set)
    if (result & 0x800000) {
        result |= 0xFF000000;
    }
    return result;
}

static inline int24_t int32_to_int24(int32_t val) {
    int24_t result;
    result.byte0 = val & 0xFF;
    result.byte1 = (val >> 8) & 0xFF;
    result.byte2 = (val >> 16) & 0xFF;
    return result;
}




/**
 * @brief struct for RPM data telemetry
 *
 * @param rpm_source_id identifies the source of the RPM data (e.g., 0 = Motor 1, 1 = Motor 2, etc.)
 * @param rpm_value array of 1 - 19 RPM values with negative ones representing the motor spinning in reverse
 *
 */
typedef struct __attribute__((packed))
{
    uint8_t    rpm_source_id;  // Identifies the source of the RPM data (e.g., 0 = Motor 1, 1 = Motor 2, etc.)
    int24_t    rpm_value[];      // 1 - 19 RPM values with negative ones representing the motor spinning in reverse
} crsf_rpm_t;

/**
 * @brief struct for temperature data telemetry
 *
 * @param temp_source_id identifies the source of the temperature data (e.g., 0 = FC including all ESCs, 1 = Ambient, etc.)
 * @param temp_value array of up to 20 temperature values in deci-degree (tenths of a degree) Celsius (e.g., 250 = 25.0°C, -50 = -5.0°C)
 *
 */
typedef struct __attribute__((packed))
{
    uint8_t temp_source_id; // Identifies the source of the temperature data (e.g., 0 = FC including all ESCs, 1 = Ambient, etc.)
    int16_t temp_value[];  // up to 20 temperature values in deci-degree (tenths of a degree) Celsius (e.g., 250 = 25.0°C, -50 = -5.0°C)
} crsf_temp_t;


/**
 * @brief struct for link statistics received from the transmitter
 * @param up_rssi_ant1 Uplink RSSI Antenna 1 (dBm * -1)
 * @param up_rssi_ant2 Uplink RSSI Antenna 2 (dBm * -1)
 * @param up_link_quality Uplink Package success rate / Link quality (%)
 * @param up_snr Uplink SNR (dB)
 * @param active_antenna number of currently best antenna
 * @param rf_profile enum {4fps = 0 , 50fps, 150fps}
 * @param up_rf_power enum {0mW = 0, 10mW, 25mW, 100mW,
 * 
 * @param down_rssi Downlink RSSI (dBm * -1)
 * @param down_link_quality Downlink Package success rate / Link quality (%)
 * @param down_snr Downlink SNR (dB)
 */
typedef struct __attribute__((packed))
{
    uint8_t up_rssi_ant1;      // Uplink RSSI Antenna 1 (dBm * -1)
    uint8_t up_rssi_ant2;      // Uplink RSSI Antenna 2 (dBm * -1)
    uint8_t up_link_quality;   // Uplink Package success rate / Link quality (%)
    int8_t up_snr;             // Uplink SNR (dB)
    uint8_t active_antenna;    // number of currently best antenna
    uint8_t rf_profile;        // enum {4fps = 0 , 50fps, 150fps}
    uint8_t up_rf_power;       // enum {0mW = 0, 10mW, 25mW, 100mW,
                               
    uint8_t down_rssi;         // Downlink RSSI (dBm * -1)
    uint8_t down_link_quality; // Downlink Package success rate / Link quality (%)
    int8_t down_snr;           // Downlink SNR (dB)
} crsf_link_statistics_t;

typedef enum
{
    CRSF_TYPE_CHANNELS = 0x16,
    CRSF_TYPE_BATTERY = 0x08,
    CRSF_TYPE_GPS = 0x02,
    CRSF_TYPE_ALTITUDE = 0x09,
    CRSF_TYPE_ATTITUDE = 0x1E,
    CRSF_TYPE_RPM = 0x0C,
    CRSF_TYPE_TEMP= 0x0D,
    CRSF_TYPE_LINK_STATISTICS = 0x14
} crsf_type_t;

typedef enum
{
    CRSF_DEST_FC = 0xC8,
    CRSF_DEST_RADIO = 0xEA
} crsf_dest_t;

/**
 * @brief setup CRSF communication
 *
 * @param config pointer to config of CRSF communication
 */
void CRSF_init(crsf_config_t *config);

/**
 * @brief copy latest 16 channel data received to the pointer
 *
 * @param channels pointer to receiver buffer
 */
void CRSF_receive_channels(crsf_channels_t *channels);

/**
 * @brief send battery data telemetry
 *
 * @param dest destination (to send back to transmitter destination is CRSF_DEST_FC)
 * @param payload pointer to the battery data
 */
void CRSF_send_battery_data(crsf_dest_t dest, crsf_battery_t *payload);

/**
 * @brief send gps data telemetry
 *
 * @param dest destination (to send back to transmitter destination is CRSF_DEST_FC)
 * @param payload pointer to the gps data
 */
void CRSF_send_gps_data(crsf_dest_t dest, crsf_gps_t *payload);

void CRSF_send_rpm_values(crsf_dest_t dest, uint8_t source_id, int32_t *rpm_values, size_t num_values);

void CRSF_send_temp_data(crsf_dest_t dest, crsf_temp_t *payload, size_t num_temps);

/**
 * @brief get the latest link statistics received
 *
 * @return crsf_link_stats_rx_t the latest link statistics received
 */
crsf_link_statistics_t CRSF_get_link_statistics();
