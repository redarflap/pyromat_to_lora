/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

/*
 * The following example demonstrates a master node in a TWAI network. The master
 * node is responsible for initiating and stopping the transfer of data messages.
 * The example will execute multiple iterations, with each iteration the master
 * node will do the following:
 * 1) Start the TWAI driver
 * 2) Repeatedly send ping messages until a ping response from slave is received
 * 3) Send start command to slave and receive data messages from slave
 * 4) Send stop command to slave and wait for stop response from slave
 * 5) Stop the TWAI driver
 */
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

/* --------------------------- Tasks and Functions -------------------------- */

void parseData(twai_message_t *msg)
{
  if (!msg)
    return;
  uint16_t index = (msg->data[1] << 8) | msg->data[0];
  switch (index)
  {
  case 0x1e:
    ovenData.data.restO2 = msg->data[6] | (msg->data[7] << 8); // ok
    return;
  case 0x21:
    ovenData.data.tempReturn = msg->data[4] | (msg->data[5] << 8); // ok check
    ovenData.data.tempSystem = msg->data[6] | (msg->data[7] << 8); // nok
    return;
  case 0x24:
    ovenData.data.tempBufferTop = msg->data[4] | (msg->data[5] << 8);    // nok
    ovenData.data.tempBufferMiddle = msg->data[6] | (msg->data[7] << 8); // nok
    return;
  case 0x27:
    ovenData.data.tempBufferBottom = msg->data[2] | (msg->data[3] << 8); // nok
    return;
  case 0x42:
    ovenData.data.clockM = msg->data[4] | (msg->data[5] << 8); // ?
    ovenData.data.clockS = msg->data[6] | (msg->data[7] << 8); // ?
    return;
  case 0x45:
    ovenData.data.clockH = msg->data[2] | (msg->data[3] << 8); // ?
    return;
  case 0x66:
    ovenData.data.tempExhaust = msg->data[2] | (msg->data[3] << 8);   // ok check
    ovenData.data.ventilatorRpm = msg->data[6] | (msg->data[7] << 8); // ok
    return;
  case 0x7b:
    ovenData.data.valveReturnMixing = msg->data[6] | (msg->data[7] << 8); // check
    return;
  case 0x7e:
    ovenData.data.valveBuffer = msg->data[4] | (msg->data[5] << 8);
    return;
  case 0x87:
    ovenData.data.airInletPrimary = msg->data[6] | (msg->data[7] << 8); // check
    return;
  case 0x8d:
    ovenData.data.targetRestO2 = msg->data[4] | (msg->data[5] << 8); // check
    return;
  case 0x93:
    ovenData.data.airInletSecondary = msg->data[2] | (msg->data[3] << 8); // check
    return;
  case 0x9f:
    ovenData.data.targetSystemTemp = msg->data[2] | (msg->data[3] << 8); // ok check
    return;
  default:
    break;
  }
}

static void twai_receive_task(void *arg)
{
  while (twai_state != CAN_DONE)
  {
    // Listen for messages
    twai_message_t rx_msg;
    twai_receive(&rx_msg, portMAX_DELAY);

    if (usb_serial_jtag_is_connected())
    {

      ESP_LOGI(EXAMPLE_TAG, "CANRX ID: %lx, dlc: %d, %lx %ld %ld %ld",
               rx_msg.identifier,
               rx_msg.data_length_code,
               (msg->data[1] << 8) | msg->data[0],
               (msg->data[3] << 8) | msg->data[2],
               (msg->data[5] << 8) | msg->data[4],
               (msg->data[7] << 8) | msg->data[6], );
    }

    // if (rx_msg.data[0] == 0x42)
    //   ESP_LOGI(EXAMPLE_TAG, "CANRX ID: %lx, dlc: %d, %x %x %x %x %x %x %x %x",
    //            rx_msg.identifier,
    //            rx_msg.data_length_code,
    //            rx_msg.data[0],
    //            rx_msg.data[1],
    //            rx_msg.data[2],
    //            rx_msg.data[3],
    //            rx_msg.data[4],
    //            rx_msg.data[5],
    //            rx_msg.data[6],
    //            rx_msg.data[7]);

    if (rx_msg.identifier > 0x200 && rx_msg.identifier < 0x600)
    {
      parseData(&rx_msg);
      vTaskDelay(pdMS_TO_TICKS(5));
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
