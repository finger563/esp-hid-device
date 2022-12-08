#include "sdkconfig.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "nvs_flash.h"

#include "format.hpp"
#include "task.hpp"

extern "C" {
#include "esp_hid_device.h"
}

using namespace std::chrono_literals;

static const char *TAG = "HID_DEV_BLE";
std::atomic<bool> is_connected_to_host = false;

static QueueHandle_t gpio_evt_queue;
static void gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

const unsigned char hidapiReportMap[] = { //8 bytes input, 8 bytes feature
    0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined 0xFF00)
    0x0A, 0x00, 0x01,  // Usage (0x0100)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x08,        //   Report Count (8)
    0x09, 0x01,        //   Usage (0x01)
    0x82, 0x02, 0x01,  //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Buffered Bytes)
    0x95, 0x08,        //   Report Count (8)
    0x09, 0x02,        //   Usage (0x02)
    0xB2, 0x02, 0x01,  //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile,Buffered Bytes)
    0x95, 0x08,        //   Report Count (8)
    0x09, 0x03,        //   Usage (0x03)
    0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,              // End Collection

    // 38 bytes
};

const unsigned char mediaReportMap[] = {
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x03,        //   Report ID (3)
    0x09, 0x02,        //   Usage (Numeric Key Pad)
    0xA1, 0x02,        //   Collection (Logical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x0A,        //     Usage Maximum (0x0A)
    0x15, 0x01,        //     Logical Minimum (1)
    0x25, 0x0A,        //     Logical Maximum (10)
    0x75, 0x04,        //     Report Size (4)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x00,        //     Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection
    0x05, 0x0C,        //   Usage Page (Consumer)
    0x09, 0x86,        //   Usage (Channel)
    0x15, 0xFF,        //   Logical Minimum (-1)
    0x25, 0x01,        //   Logical Maximum (1)
    0x75, 0x02,        //   Report Size (2)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x46,        //   Input (Data,Var,Rel,No Wrap,Linear,Preferred State,Null State)
    0x09, 0xE9,        //   Usage (Volume Increment)
    0x09, 0xEA,        //   Usage (Volume Decrement)
    0x15, 0x00,        //   Logical Minimum (0)
    0x75, 0x01,        //   Report Size (1)
    0x95, 0x02,        //   Report Count (2)
    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0xE2,        //   Usage (Mute)
    0x09, 0x30,        //   Usage (Power)
    0x09, 0x83,        //   Usage (Recall Last)
    0x09, 0x81,        //   Usage (Assign Selection)
    0x09, 0xB0,        //   Usage (Play)
    0x09, 0xB1,        //   Usage (Pause)
    0x09, 0xB2,        //   Usage (Record)
    0x09, 0xB3,        //   Usage (Fast Forward)
    0x09, 0xB4,        //   Usage (Rewind)
    0x09, 0xB5,        //   Usage (Scan Next Track)
    0x09, 0xB6,        //   Usage (Scan Previous Track)
    0x09, 0xB7,        //   Usage (Stop)
    0x15, 0x01,        //   Logical Minimum (1)
    0x25, 0x0C,        //   Logical Maximum (12)
    0x75, 0x04,        //   Report Size (4)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x00,        //   Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0x09, 0x80,        //   Usage (Selection)
    0xA1, 0x02,        //   Collection (Logical)
    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (0x01)
    0x29, 0x03,        //     Usage Maximum (0x03)
    0x15, 0x01,        //     Logical Minimum (1)
    0x25, 0x03,        //     Logical Maximum (3)
    0x75, 0x02,        //     Report Size (2)
    0x81, 0x00,        //     Input (Data,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              //   End Collection
    0x81, 0x03,        //   Input (Const,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
    0xC0,              // End Collection
};

static esp_hid_raw_report_map_t ble_report_maps[] = {
    {
        .data = hidapiReportMap,
        .len = sizeof(hidapiReportMap)
    },
    {
        .data = mediaReportMap,
        .len = sizeof(mediaReportMap)
    }
};

static esp_hid_device_config_t ble_hid_config = {
    .vendor_id          = 0x16C0,
    .product_id         = 0x05DF,
    .version            = 0x0100,
    .device_name        = "Backbone Two",
    .manufacturer_name  = "Backbone",
    .serial_number      = "1234567890",
    .report_maps        = ble_report_maps,
    .report_maps_len    = 2
};

void ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data) {
  esp_hidd_event_t event = (esp_hidd_event_t)id;
  esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

  switch (event) {
  case ESP_HIDD_START_EVENT: {
    ESP_LOGI(TAG, "START");
    esp_hid_ble_gap_adv_start();
    break;
  }
  case ESP_HIDD_CONNECT_EVENT: {
    ESP_LOGI(TAG, "CONNECT");
    // ble_hid_task_start_up();//todo: this should be on auth_complete (in GAP)
    is_connected_to_host = true;
    break;
  }
  case ESP_HIDD_PROTOCOL_MODE_EVENT: {
    ESP_LOGI(TAG, "PROTOCOL MODE[%u]: %s", param->protocol_mode.map_index, param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
    break;
  }
  case ESP_HIDD_CONTROL_EVENT: {
    ESP_LOGI(TAG, "CONTROL[%u]: %sSUSPEND", param->control.map_index, param->control.control ? "EXIT_" : "");
    break;
  }
  case ESP_HIDD_OUTPUT_EVENT: {
    ESP_LOGI(TAG, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:", param->output.map_index, esp_hid_usage_str(param->output.usage), param->output.report_id, param->output.length);
    ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
    break;
  }
  case ESP_HIDD_FEATURE_EVENT: {
    ESP_LOGI(TAG, "FEATURE[%u]: %8s ID: %2u, Len: %d, Data:", param->feature.map_index, esp_hid_usage_str(param->feature.usage), param->feature.report_id, param->feature.length);
    ESP_LOG_BUFFER_HEX(TAG, param->feature.data, param->feature.length);
    break;
  }
  case ESP_HIDD_DISCONNECT_EVENT: {
    ESP_LOGI(TAG, "DISCONNECT: %s", esp_hid_disconnect_reason_str(esp_hidd_dev_transport_get(param->disconnect.dev), param->disconnect.reason));
    // ble_hid_task_shut_down();
    is_connected_to_host = false;
    esp_hid_ble_gap_adv_start();
    break;
  }
  case ESP_HIDD_STOP_EVENT: {
    ESP_LOGI(TAG, "STOP");
    break;
  }
  default:
    break;
  }
  return;
}

extern "C" void app_main(void) {
  esp_err_t err;
  espp::Logger logger({.tag = TAG, .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup");

  // initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    logger.warn("Erasing NVS flash...");
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // set up the ble hid device
#if HID_DEV_MODE == HIDD_IDLE_MODE
  logger.error("Please turn on BT HID device or BLE!");
  return;
#endif

  logger.info("setting hid gap, mode: {}", (int)HID_DEV_MODE);
  ret = esp_hid_gap_init(HID_DEV_MODE);
  ESP_ERROR_CHECK( ret );

#if CONFIG_BT_BLE_ENABLED
  ret = esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_GENERIC, ble_hid_config.device_name);
  ESP_ERROR_CHECK( ret );

  if ((ret = esp_ble_gatts_register_callback(esp_hidd_gatts_event_handler)) != ESP_OK) {
    logger.error("GATTS register callback failed: {} - '{}'", ret, esp_err_to_name(ret));
    return;
  }
  logger.info("setting ble device");
  esp_hidd_dev_t *hid_dev;
  ESP_ERROR_CHECK(
                  esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE, ble_hidd_event_callback, &hid_dev));
#endif

  // set up the gpio we'll use - we're going to set an ISR on the gpio to look
  // for any edge and then have the ISR push to a FreeRTOS queue - this will
  // allow us to easily block until the queue has data, meaning the pin was
  // pressed.
  static constexpr size_t RECV_GPIO = 21;
  // create the gpio event queue
  gpio_evt_queue = xQueueCreate(2, sizeof(uint32_t));
  // setup gpio interrupts for boot button and mute button
  gpio_config_t io_conf;
  memset(&io_conf, 0, sizeof(io_conf));
  // interrupt on any edge since we're looking for a toggle
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  io_conf.pin_bit_mask = (1<<RECV_GPIO);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
  gpio_config(&io_conf);
  //install gpio isr service
  gpio_install_isr_service(0);
  gpio_isr_handler_add((gpio_num_t)RECV_GPIO, gpio_isr_handler, (void*) RECV_GPIO);

  // now that we're good let's start sending data and measuring latency
  int num_reports_sent = 0;
  float total_elapsed_seconds = 0;
  float max_elapsed_seconds = 0;
  float min_elapsed_seconds = 100;
  auto report_period = 50ms;
  uint32_t io_num = 0;
  while (true) {
    if (is_connected_to_host) {
      auto num_waiting = uxQueueMessagesWaiting(gpio_evt_queue);
      if (num_waiting) {
        logger.warn("Queue already has {} entries!", (int)num_waiting);
      }
      auto start = std::chrono::high_resolution_clock::now();
      // send the hid report (with the opposite of the pin state)
      esp_hidd_send_consumer_value(hid_dev, HID_CONSUMER_VOLUME_UP, true);
      // wait until pin is toggled (notified by ISR)
      if (xQueueReceive(gpio_evt_queue, &io_num, 100 / portTICK_PERIOD_MS) == pdTRUE) {
        // see if it's the pin we're looking for (it shouldn't be anything else)
        if (io_num == RECV_GPIO) {
          auto end = std::chrono::high_resolution_clock::now();
          // measure execution time (latency)
          float elapsed_seconds = std::chrono::duration<float>(end-start).count();
          max_elapsed_seconds = std::max(elapsed_seconds, max_elapsed_seconds);
          min_elapsed_seconds = std::min(elapsed_seconds, min_elapsed_seconds);
          // update average execution time (latency)
          total_elapsed_seconds += elapsed_seconds;
          num_reports_sent++;
          float average_latency = total_elapsed_seconds / (float)num_reports_sent;
          // print
          logger.info("Latency: {:.3f}s ({:.3f}s) ({:.3f}s - {:.3f}s)",
                      elapsed_seconds, average_latency, min_elapsed_seconds, max_elapsed_seconds);
        } else {
          logger.warn("For some reason, we got a different gpio interrupt!");
        }
      } else {
        logger.warn("For some reason we couldn't get anything from the queue!");
      }
      // try to get exactly the desired report rate
      std::this_thread::sleep_until(start + report_period);
    } else {
      logger.warn("Not connected, waiting for BLE HID Host connection...");
      // so just wait a little longer..
      std::this_thread::sleep_for(1s);
    }
  }
}
