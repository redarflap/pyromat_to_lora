#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"
#include "lora.h"

/* --------------------- Definitions and static variables ------------------ */
// Example Configuration
#define RX_TASK_PRIO 8
#define PEER_CAN_ID 0x1
#define EXAMPLE_TAG "TWAI Master"
#define LORA_NODE_ID 0x01 // ID Pyromat Oven

typedef enum
{
  CAN_LISTEN = 1,
  CAN_DONE = 5
} twai_states_t;

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = {.mode = TWAI_MODE_NORMAL,
                                               .tx_io = CONFIG_EXAMPLE_TX_GPIO_NUM,
                                               .rx_io = CONFIG_EXAMPLE_RX_GPIO_NUM,
                                               .clkout_io = TWAI_IO_UNUSED,
                                               .bus_off_io = TWAI_IO_UNUSED,
                                               .tx_queue_len = 0,
                                               .rx_queue_len = 5,
                                               .alerts_enabled = TWAI_ALERT_NONE,
                                               .clkout_divider = 0};

static twai_states_t twai_state = CAN_LISTEN;

struct __attribute__((__packed__)) OvenData_s
{
  uint16_t nodeId;

  uint16_t tempSystem;
  uint16_t tempReturn;
  uint16_t tempBufferTop;
  uint16_t tempBufferMiddle;
  uint16_t tempBufferBottom;
  uint16_t tempExhaust;
  uint16_t ventilatorRpm;
  uint16_t clockH;
  uint16_t clockM;
  uint16_t clockS;

  int16_t restO2;
  uint16_t airInletPrimary;
  uint16_t airInletSecondary;
  uint16_t valveReturnMixing;
  uint16_t valveBuffer;
  uint16_t targetRestO2;
  uint16_t targetSystemTemp;
} const OvenData_default = {LORA_NODE_ID, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

typedef struct OvenData_s OvenData;
typedef union OvenDataConverter_u
{
  OvenData data;
  uint8_t bytes[sizeof(OvenData)];
} OvenDataConverter;

static OvenDataConverter ovenData;

// static uint64_t previousData[5000] = {0};

/* --------------------------- Tasks and Functions -------------------------- */

void parseData(twai_message_t *msg)
{
  if (!msg)
    return;
  uint16_t index = (msg->data[1] << 8) | msg->data[0];
  switch (index)
  {
  case 0x1e:
    ovenData.data.restO2 = msg->data[6] | (msg->data[7] << 8);
    return;

  case 0x27:

    return;
  case 0x42:
    ovenData.data.clockM = msg->data[4] | (msg->data[5] << 8);
    return;
  case 0x44:
    ovenData.data.clockH = msg->data[4] | (msg->data[5] << 8);
    ovenData.data.clockS = msg->data[2] | (msg->data[3] << 8);
    return;

  case 0x66:
    ovenData.data.tempExhaust = msg->data[2] | (msg->data[3] << 8);
    ovenData.data.ventilatorRpm = msg->data[6] | (msg->data[7] << 8);
    return;
  case 0x7b:
    ovenData.data.valveReturnMixing = msg->data[6] | (msg->data[7] << 8); // check
    return;
  case 0x7e:
    ovenData.data.valveBuffer = msg->data[4] | (msg->data[5] << 8);
    return;
  case 0x8a:
    ovenData.data.airInletPrimary = msg->data[2] | (msg->data[3] << 8); // range anpassen  90=offen
    return;
  case 0x8c:
    ovenData.data.targetRestO2 = msg->data[6] | (msg->data[7] << 8);
    return;
  case 0x8d:
    ovenData.data.tempReturn = msg->data[6] | (msg->data[7] << 8); //  check
    return;
  case 0x92:
    ovenData.data.airInletSecondary = msg->data[4] | (msg->data[5] << 8); // check 92   30 34 30
    return;
  case 0x9c:
    ovenData.data.tempSystem = msg->data[6] | (msg->data[7] << 8); // target min 6-7    9e 832 850
    return;
  case 0x9f:
    ovenData.data.targetSystemTemp = msg->data[2] | (msg->data[3] << 8);
    return;
  case 0x171:
    ovenData.data.tempBufferTop = msg->data[4] | (msg->data[5] << 8);
    ovenData.data.tempBufferMiddle = msg->data[6] | (msg->data[7] << 8);
    return;
  case 0x174:
    ovenData.data.tempBufferBottom = msg->data[2] | (msg->data[3] << 8);
    return;
  // case 0x172:
  //   ovenData.data.tempBufferTop = msg->data[2] | (msg->data[3] << 8);
  //   ovenData.data.tempBufferMiddle = msg->data[4] | (msg->data[5] << 8);
  //   ovenData.data.tempBufferBottom = msg->data[6] | (msg->data[7] << 8);
  //   return;
  default:
    break;
  }

  // ESP_LOGI(EXAMPLE_TAG, "Clock: %d %d %d", ovenData.data.clockH, ovenData.data.clockM, ovenData.data.clockS);
}

static void twai_receive_task(void *arg)
{
  while (twai_state != CAN_DONE)
  {
    // Listen for messages
    twai_message_t rx_msg;
    twai_receive(&rx_msg, portMAX_DELAY);

        // uint16_t from = 99;
    // uint16_t to = 101;
    // uint16_t v1 = (rx_msg.data[3] << 8) | (rx_msg.data[2]);
    // uint16_t v2 = (rx_msg.data[5] << 8) | (rx_msg.data[4]);
    // uint16_t v3 = (rx_msg.data[7] << 8) | (rx_msg.data[6]);

    // if ((v1 > from && v1 < to) || (v2 > from && v2 < to) || (v3 > from && v3 < to))
    //   ESP_LOGI(EXAMPLE_TAG, "CANRX, %04x - %04u %04u %04u",
    //            // rx_msg.identifier,
    //            //  rx_msg.data_length_code,
    //            (rx_msg.data[1] << 8) | (rx_msg.data[0]),
    //            (rx_msg.data[3] << 8) | (rx_msg.data[2]),
    //            (rx_msg.data[5] << 8) | (rx_msg.data[4]),
    //            (rx_msg.data[7] << 8) | (rx_msg.data[6]));

    if (rx_msg.identifier > 0x200 && rx_msg.identifier < 0x600)
    {
      parseData(&rx_msg);
      vTaskDelay(pdMS_TO_TICKS(5));

      // uint16_t index = ((rx_msg.data[1] << 8) | rx_msg.data[0]);
      // if (index < 5000)
      // {
      //   uint64_t newData = ((uint64_t)((rx_msg.data[3] << 8) | (rx_msg.data[2]))) << 32 |
      //                      ((rx_msg.data[5] << 8) | (rx_msg.data[4])) << 16 |
      //                      (rx_msg.data[7] << 8) | (rx_msg.data[6]);

      //   uint16_t idx = (rx_msg.data[1] << 8) | rx_msg.data[0];

      //   if (previousData[index] != 0 && previousData[index] != newData

      //       && idx != 0x42 && idx != 0x43 && idx != 0x44)
      //   {
      //     ESP_LOGI(EXAMPLE_TAG, "%x New Data: %u %u %u",
      //              (rx_msg.data[1] << 8) | rx_msg.data[0],
      //              (rx_msg.data[3] << 8) | (rx_msg.data[2]),
      //              (rx_msg.data[5] << 8) | (rx_msg.data[4]),
      //              (rx_msg.data[7] << 8) | (rx_msg.data[6]));

      //     if (false) // previousData[index] != 0 && previousData[index] != newData)
      //     {

      //       ESP_LOGI(EXAMPLE_TAG, "%x New Data: %u %u %u => %u %u %u",
      //                (rx_msg.data[1] << 8) | rx_msg.data[0],
      //                (uint16_t)(previousData[index] >> 32 & 0xFFFF),
      //                (uint16_t)(previousData[index] >> 16 & 0xFFFF),
      //                (uint16_t)(previousData[index] & 0xFFFF),
      //                (rx_msg.data[3] << 8) | (rx_msg.data[2]),
      //                (rx_msg.data[5] << 8) | (rx_msg.data[4]),
      //                (rx_msg.data[7] << 8) | (rx_msg.data[6]));
      //     }
      //   }

      // previousData[index] = newData;
      // }
    }
  }
  vTaskDelete(NULL);
}

void lora_tx_task()
{
  ESP_LOGI(pcTaskGetName(NULL), "Start");

  while (1)
  {
    lora_send_packet(ovenData.bytes, sizeof(ovenData.bytes));
    ovenData.data = OvenData_default;

    // ESP_LOGI(pcTaskGetName(NULL), "packet sent... %x%x %x%x %x%x %x%x %x%x %x%x %x%x %x%x",
    //          ovenData.bytes[0],
    //          ovenData.bytes[1],
    //          ovenData.bytes[2],
    //          ovenData.bytes[3],
    //          ovenData.bytes[4],
    //          ovenData.bytes[5],
    //          ovenData.bytes[6],
    //          ovenData.bytes[7],
    //          ovenData.bytes[8],
    //          ovenData.bytes[9],
    //          ovenData.bytes[10],
    //          ovenData.bytes[11],
    //          ovenData.bytes[12],
    //          ovenData.bytes[13],
    //          ovenData.bytes[14],
    //          ovenData.bytes[15]
    // );

    int lost = lora_packet_lost();
    if (lost != 0)
    {
      ESP_LOGW(pcTaskGetName(NULL), "%d packets lost", lost);
    }
    vTaskDelay(pdMS_TO_TICKS(30000));
  } // end while
}

void app_main(void)
{
  ovenData.data = OvenData_default;

  // Create tasks, queues, and semaphores
  xTaskCreatePinnedToCore(twai_receive_task, "TWAI_rx", 4096, NULL, RX_TASK_PRIO, NULL, 1);

  // Install TWAI driver
  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
  ESP_LOGI(EXAMPLE_TAG, "TWAI Driver installed");
  ESP_ERROR_CHECK(twai_start());
  ESP_LOGI(EXAMPLE_TAG, "TWAI Driver started");

  if (lora_init() == 0)
  {
    ESP_LOGE(pcTaskGetName(NULL), "Does not recognize the LoRa module");
    while (1)
    {
      vTaskDelay(1);
    }
  }
  ESP_LOGI(pcTaskGetName(NULL), "LoRa Frequency is 868MHz");
  lora_set_frequency(868e6); // 868MHz

  lora_enable_crc();

  int cr = 5;
  int bw = 7;
  int sf = 7;

  lora_set_coding_rate(cr);
  ESP_LOGI(pcTaskGetName(NULL), "coding_rate=%d", cr);

  lora_set_bandwidth(bw);
  ESP_LOGI(pcTaskGetName(NULL), "bandwidth=%d", bw);

  lora_set_spreading_factor(sf);
  ESP_LOGI(pcTaskGetName(NULL), "spreading_factor=%d", sf);

  vTaskDelay(pdMS_TO_TICKS(20000));
  xTaskCreate(&lora_tx_task, "LoRa TX", 1024 * 3, NULL, 5, NULL);

  while (1)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
  }

  // Uninstall TWAI driver
  ESP_ERROR_CHECK(twai_stop());
  ESP_LOGI(EXAMPLE_TAG, "TWAI Driver stopped");
  ESP_ERROR_CHECK(twai_driver_uninstall());
  ESP_LOGI(EXAMPLE_TAG, "TWAI Driver uninstalled");
}
