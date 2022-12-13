#include "sdkconfig.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_pm.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

#include "format.hpp"
#include "task.hpp"

#include "ble_gamepad.hpp"

#define TEST_LATENCY 0
#define TEST_POWER   1

using namespace std::chrono_literals;

#if TEST_LATENCY
static QueueHandle_t gpio_evt_queue;
static void IRAM_ATTR gpio_isr_handler(void *arg) {
  uint32_t gpio_num = (uint32_t)arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}
#endif

extern "C" void app_main(void) {
  esp_err_t err;
  espp::Logger logger({.tag = "Backbone Two", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup");

#if TEST_POWER
  esp_pm_config_esp32_t pm_config = {
    .max_freq_mhz = 240, // e.g. 80, 160, 240
    .min_freq_mhz = 40, // e.g. 40
    .light_sleep_enable = true, // enable light sleep
  };
  ESP_ERROR_CHECK( esp_pm_configure(&pm_config) );
#endif

  // create the gamepad
  BleGamepad ble_gamepad("Backbone Two", "Backbone", 95);
  BleGamepadConfiguration ble_gamepad_config;
  ble_gamepad_config.setAutoReport(false); // normally true
  // and start advertising
  ble_gamepad.begin(&ble_gamepad_config);

#if TEST_LATENCY
  // set up the gpio we'll use
  static constexpr size_t RECV_GPIO = 26;
  // create the gpio event queue
  gpio_evt_queue = xQueueCreate(2, sizeof(uint32_t));
  // setup gpio interrupts for boot button and mute button
  gpio_config_t io_conf;
  memset(&io_conf, 0, sizeof(io_conf));
  // interrupt on any edge since we're looking for a toggle
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  io_conf.pin_bit_mask = (1<<RECV_GPIO);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&io_conf);

  //install gpio isr service
  gpio_install_isr_service(0);
  gpio_isr_handler_add((gpio_num_t)RECV_GPIO, gpio_isr_handler, (void*) RECV_GPIO);
  // esp_sleep_enable_gpio_wakeup();

  // variables for measuring latency
  uint32_t io_num = 0;
  int num_reports_sent = 0;
  float total_elapsed_seconds = 0;
  float max_elapsed_seconds = 0;
  float min_elapsed_seconds = 100;
#endif

  // now lets start sending reports
  auto report_period = 50ms;
  auto report_period_us = std::chrono::duration_cast<std::chrono::microseconds>(report_period).count();
  while (true) {
    if (ble_gamepad.isConnected()) {
      auto start = std::chrono::high_resolution_clock::now();
#if TEST_POWER
      // send the hid report
      ble_gamepad.press(BUTTON_5);
      ble_gamepad.sendReport();
      // set the wakeup timer
      auto end = std::chrono::high_resolution_clock::now();
      auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
      esp_sleep_enable_timer_wakeup(report_period_us - elapsed_us);
      // and sleep again till we need to wakeup
      esp_light_sleep_start();
#elif TEST_LATENCY
      auto num_waiting = uxQueueMessagesWaiting(gpio_evt_queue);
      if (num_waiting) {
        logger.warn("Queue already has {} entries!", (int)num_waiting);
      }
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
          printf("\x1B[1A"); // go up a line
          printf("\x1B[2K\r"); // erase the line
          logger.info("[{}] Latency: {:.3f}s ({:.3f}s) ({:.3f}s - {:.3f}s)",
                      start,
                      elapsed_seconds,
                      average_latency, min_elapsed_seconds, max_elapsed_seconds);
        } else {
          logger.warn("For some reason, we got a different gpio interrupt!");
        }
      } else {
        logger.warn("For some reason we couldn't get anything from the queue!");
      }
      // try to get exactly the desired report rate
      std::this_thread::sleep_until(start + report_period);
#endif
    } else {
      // printf("\x1B[1A"); // go up a line
      // printf("\x1B[2K\r"); // erase the line
      logger.warn("[{}] Not connected, waiting for BLE HID Host connection...",
                  std::chrono::high_resolution_clock::now());
      // so just wait a little longer..
      std::this_thread::sleep_for(1s);
    }
  }
}
