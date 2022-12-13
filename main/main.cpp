#include "sdkconfig.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_sleep.h"
#include "driver/gpio.h"

#include "format.hpp"
#include "task.hpp"

#include "ble_gamepad.hpp"

using namespace std::chrono_literals;

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

  // set up the gpio we'll use
  static constexpr size_t RECV_GPIO = 26;
  // setup gpio interrupts for boot button and mute button
  gpio_config_t io_conf;
  memset(&io_conf, 0, sizeof(io_conf));
  // interrupt on any edge since we're looking for a toggle
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.pin_bit_mask = (1<<RECV_GPIO);
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  gpio_config(&io_conf);

  // now that we're good let's start sending data and measuring latency
  int num_reports_sent = 0;
  float total_elapsed_seconds = 0;
  float max_elapsed_seconds = 0;
  float min_elapsed_seconds = 100;
  auto report_period = 50ms;
  auto report_period_us = std::chrono::duration_cast<std::chrono::microseconds>(report_period).count();
  uint32_t io_num = 0;
  while (true) {
    if (ble_gamepad.isConnected()) {
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
      auto start = std::chrono::high_resolution_clock::now();
      // get the current level (to know what level should cause wakeup)
      auto current_level = gpio_get_level((gpio_num_t)RECV_GPIO);
      auto wakeup_level = current_level == 1 ? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL;
      // logger.info("waking up for level {}", wakeup_level);
      gpio_wakeup_enable((gpio_num_t)RECV_GPIO, wakeup_level);
      esp_sleep_enable_gpio_wakeup();
      static constexpr auto gpio_timeout = 30 * 1000; // 100ms (in us)
      esp_sleep_enable_timer_wakeup(gpio_timeout);
      // send the hid report
      ble_gamepad.press(BUTTON_5);
      ble_gamepad.sendReport();
      // see
      // https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/sleep_modes.html
      // which says that before entering light / deep sleep modes, the
      // application must disable WiFi and Bluetooth
      // esp_bt_controller_disable();
      esp_light_sleep_start();
      switch (esp_sleep_get_wakeup_cause()) {
      case ESP_SLEEP_WAKEUP_GPIO: {
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
        // printf("\x1B[1A"); // go up a line
        // printf("\x1B[2K\r"); // erase the line
        logger.info("[{}] Latency: {:.3f}s ({:.3f}s) ({:.3f}s - {:.3f}s)",
                    start,
                    elapsed_seconds,
                    average_latency, min_elapsed_seconds, max_elapsed_seconds);
      } break;
      case ESP_SLEEP_WAKEUP_TIMER:
        // logger.info("Couldn't get gpio wakeup signal!");
        break;
      default:
        // logger.warn("Unknown wakeup source!");
        break;
      }
      // set the wakeup timer
      auto end = std::chrono::high_resolution_clock::now();
      auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(end-start).count();
      esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
      esp_sleep_enable_timer_wakeup(report_period_us - elapsed_us);
      // and sleep again till we need to wakeup
      esp_light_sleep_start();
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
