#include <stdio.h>
#include "ESP_CRSF.h"
#include "byteswap.h"
#include "freertos/timers.h"


#define RX_BUF_SIZE 1024 // UART buffer size

// CRC8 lookup table (poly 0xd5)
static uint8_t crc8_table[256] = {0};

void generate_CRC(uint8_t poly)
{
  for (int idx = 0; idx < 256; ++idx)
  {
    uint8_t crc = idx;
    for (int shift = 0; shift < 8; ++shift)
    {
      crc = (crc << 1) ^ ((crc & 0x80) ? poly : 0);
    }
    crc8_table[idx] = crc & 0xff;
  }
}

// Function to calculate CRC8 checksum
uint8_t crc8(const uint8_t *data, uint8_t len)
{
  uint8_t crc = 0;
  while (len--)
  {
    crc = crc8_table[crc ^ *data++];
  }

  return crc;
}

SemaphoreHandle_t xMutex;

static int uart_num = 1;
static QueueHandle_t uart_queue;
crsf_channels_t received_channels = {0};
crsf_battery_t received_battery = {0};
crsf_link_statistics_t received_link_statistics = {0};

static bool failsafe_flag = true; // Failsafe flag
static TimerHandle_t failsafe_timer = NULL; // Watchdog timer

static void rx_task(void *arg)
{
  uart_event_t event;
  uint8_t *dtmp = (uint8_t *)malloc(RX_BUF_SIZE);
  for (;;)
  {
    // Waiting for UART event.
    if (xQueueReceive(uart_queue, (void *)&event, (TickType_t)portMAX_DELAY))
    {
      bzero(dtmp, RX_BUF_SIZE);
      if (event.type == UART_DATA)
      {
        // ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
        uart_read_bytes(uart_num, dtmp, event.size, portMAX_DELAY);

        // extract length and type
        uint8_t type = dtmp[2];
        uint8_t length = dtmp[1];
        uint8_t dest = dtmp[0];

        // read the rest of the frame
        uint8_t payload_length = length - 2;
        uint8_t payload[payload_length];

        for (int i = 0; i < payload_length; i++)
        {
          payload[i] = dtmp[i + 3];
        }

 
        switch (type)
        {
          case CRSF_TYPE_CHANNELS:
            xSemaphoreTake(xMutex, portMAX_DELAY);
            received_channels = *(crsf_channels_t *)payload;
            xSemaphoreGive(xMutex);

            // Reset the failsafe timer
            if (failsafe_timer != NULL) {
                xTimerReset(failsafe_timer, 0);
            }

            // Clear the failsafe flag
            failsafe_flag = false;

            break;

          case CRSF_TYPE_LINK_STATISTICS:
            xSemaphoreTake(xMutex, portMAX_DELAY);
            received_link_statistics = *(crsf_link_statistics_t *)payload;
            xSemaphoreGive(xMutex);
            break;
        }
      }
    }
  }
  free(dtmp);
  dtmp = NULL;
  vTaskDelete(NULL);
}

// Timer callback to set the failsafe flag
static void failsafe_timer_callback(TimerHandle_t xTimer) {
    failsafe_flag = true; // Set the failsafe flag
}

void CRSF_init(crsf_config_t *config) {
    generate_CRC(0xd5);

    uart_num = config->uart_num;

    // Begin UART communication with RX
    uart_config_t uart_config = {
        .baud_rate = 420000,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(config->uart_num, &uart_config);
    uart_set_pin(uart_num, config->tx_pin, config->rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    ESP_ERROR_CHECK(uart_driver_install(uart_num, RX_BUF_SIZE, RX_BUF_SIZE, 10, &uart_queue, 0));

    // Create semaphore
    xMutex = xSemaphoreCreateMutex();

    // Create task
    xTaskCreate(rx_task, "uart_rx_task", 1024 * 4, NULL, configMAX_PRIORITIES - 1, NULL);

    // Create and start the failsafe timer
    failsafe_timer = xTimerCreate("FailsafeTimer", pdMS_TO_TICKS(500), pdFALSE, NULL, failsafe_timer_callback);
    if (failsafe_timer != NULL) {
        xTimerStart(failsafe_timer, 0);
    }
}

// receive uart data frame
void CRSF_receive_channels(crsf_channels_t *channels)
{
  xSemaphoreTake(xMutex, portMAX_DELAY);
  *channels = received_channels;
  xSemaphoreGive(xMutex);
}
/**
 * @brief function sends payload to a destination using uart
 *
 * @param payload pointer to payload of given crsf_type_t
 * @param destination destination for payload, typically CRSF_DEST_FC
 * @param type type of data contained in payload
 * @param payload_length length of the payload type
 */
void CRSF_send_payload(const void *payload, crsf_dest_t destination, crsf_type_t type, uint8_t payload_length)
{
    // The +4 accounts for the two bytes on both ends of the packet: 2 + [payload_length] + 2
    uint8_t packet[payload_length + 4];

    packet[0] = destination;
    packet[1] = payload_length + 2; // Size of payload + type + CRC
    packet[2] = type;

    memcpy(&packet[3], payload, payload_length);

    // Calculate CRC
    unsigned char checksum = crc8(&packet[2], payload_length + 1);
    packet[payload_length + 3] = checksum;

    // Send frame
    uart_write_bytes(uart_num, packet, payload_length + 4);
}

void CRSF_send_battery_data(crsf_dest_t dest, crsf_battery_t *payload)
{
  crsf_battery_t *payload_proc = 0;
  // processed payload
  payload_proc = (crsf_battery_t *)payload;
  payload_proc->voltage = __bswap16(payload_proc->voltage);
  payload_proc->current = __bswap16(payload_proc->current);
  payload_proc->capacity = __bswap16(payload_proc->capacity) << 8;

  CRSF_send_payload(payload_proc, dest, CRSF_TYPE_BATTERY, sizeof(crsf_battery_t));
}

void CRSF_send_gps_data(crsf_dest_t dest, crsf_gps_t *payload)
{
  crsf_gps_t *payload_proc = 0;
  // processed payload
  payload_proc = (crsf_gps_t *)payload;
  payload_proc->latitude = __bswap32(payload_proc->latitude);
  payload_proc->longitude = __bswap32(payload_proc->longitude);
  payload_proc->groundspeed = __bswap16(payload_proc->groundspeed);
  payload_proc->heading = __bswap16(payload_proc->heading);
  payload_proc->altitude = __bswap16(payload_proc->altitude);

  CRSF_send_payload(payload_proc, dest, CRSF_TYPE_GPS, sizeof(crsf_gps_t));
}

inline uint32_t bswap24(uint32_t value) {
    // Swap only the lower 24 bits
    return ((value & 0x0000FF) << 16) | 
           ((value & 0x00FF00)) | 
           ((value & 0xFF0000) >> 16);
}

void CRSF_send_rpm_values(crsf_dest_t dest, uint8_t source_id, int32_t *rpm_values, size_t num_values)
{
    // Limit to maximum supported RPM values
    if (num_values > 19) {
        num_values = 19;
    }

    // Allocate memory for the complete packet
    size_t packet_size = sizeof(crsf_rpm_t) + (num_values * sizeof(int24_t));
    crsf_rpm_t *rpm_packet = malloc(packet_size);
    if (!rpm_packet) {
        ESP_LOGE("CRSF", "Failed to allocate memory for RPM packet");
        return;
    }

    // Set source ID
    rpm_packet->rpm_source_id = source_id;

    // Convert and copy RPM values with byte swapping
    for (size_t i = 0; i < num_values; i++) {
        rpm_packet->rpm_value[i] = int32_to_int24(bswap24(rpm_values[i]));
    }

    // Send the data
    CRSF_send_payload(rpm_packet, dest, CRSF_TYPE_RPM, packet_size);

    // Clean up
    free(rpm_packet);
}

void CRSF_send_temp_data(crsf_dest_t dest, crsf_temp_t *payload, size_t num_temps)
{
    // Calculate the actual payload size
    size_t payload_size = sizeof(uint8_t) + (num_temps * sizeof(int16_t));

    // Process endianness for each temperature value
    for (size_t i = 0; i < num_temps; i++) {
        payload->temp_value[i] = __bswap16(payload->temp_value[i]);
    }

    // Send the data
    CRSF_send_payload(payload, dest, CRSF_TYPE_TEMP, payload_size);
}

crsf_link_statistics_t CRSF_get_link_statistics()
{
  xSemaphoreTake(xMutex, portMAX_DELAY);
  crsf_link_statistics_t stats = received_link_statistics;
  xSemaphoreGive(xMutex);
  return stats;
}



// Function to check if the system is in failsafe
bool CRSF_is_failsafe() {
    return failsafe_flag;
}
