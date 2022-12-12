#include "sdkconfig.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "format.hpp"
#include "task.hpp"

#include "ble_gamepad.hpp"

using namespace std::chrono_literals;

static QueueHandle_t gpio_evt_queue;
static void gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

extern "C" void app_main(void) {
  esp_err_t err;
  espp::Logger logger({.tag = "Backbone Two", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup");

  // create the gamepad
  BleGamepad ble_gamepad("Backbone Two", "Backbone", 95);
  BleGamepadConfiguration ble_gamepad_config;
  ble_gamepad_config.setAutoReport(false); // normally true
  // and start advertising
  ble_gamepad.begin(&ble_gamepad_config);

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
    if (ble_gamepad.isConnected()) {
      auto num_waiting = uxQueueMessagesWaiting(gpio_evt_queue);
      if (num_waiting) {
        logger.warn("Queue already has {} entries!", (int)num_waiting);
      }
      auto start = std::chrono::high_resolution_clock::now();
      // send the hid report
      ble_gamepad.press(BUTTON_5);
      ble_gamepad.sendReport();
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
      printf("\x1B[1A"); // go up a line
      printf("\x1B[2K\r"); // erase the line
      logger.warn("[{}] Not connected, waiting for BLE HID Host connection...", std::chrono::high_resolution_clock::now());
      // so just wait a little longer..
      std::this_thread::sleep_for(1s);
    }
  }
}
