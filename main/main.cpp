#include "sdkconfig.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string.h>

#include "oneshot_adc.hpp"
#include "controller.hpp"
#include "logger.hpp"
#include "task.hpp"

#include "ble_gamepad.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  esp_err_t err;
  espp::Logger logger({.tag = "LodeStone", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup");

  // NOTE: controller mapping:
  // * R1 -> A0 (GPIO18)
  // * R2 -> A1 (GPIO17)
  // * X  -> A2 (GPIO9)
  // * Y  -> A3 (GPIO8)
  // * Rx -> SDA (GPIO7 / ADC1_CH6)
  // * Ry -> SCL (GPIO6 / ADC1_CH5)
  // * Menu -> TX (GPIO5)
  // * Home -> RX (GPIO16)
  // * R3 -> SCK (GPIO36)
  // * A  -> MI (GPIO37)
  // * B  -> MO (GPIO35)

  // create the controller (digital buttons) object
  espp::Controller controller(espp::Controller::DualConfig{
      // buttons short to ground, so they are active low. this will enable the
      // GPIO_PULLUP and invert the logic
      .active_low = true,
      .gpio_a = 37, // MI on Qt Py S3, A on BB
      .gpio_b = 35, // MO on Qt Py S3, B on BB
      .gpio_x = 9, // A2 on Qt Py S3, X on BB
      .gpio_y = 8, // A3 on Qt Py S3, Y on BB
      .gpio_start = 16,  // RX on Qt Py S3, HOME on BB
      .gpio_select = 5, // TX on Qt Py S3, MENU on BB
      .gpio_joystick_select = 36, // SCK on Qt Py S3, R3 on BB
      .log_level = espp::Logger::Verbosity::WARN
    });

  // create the adc confgiuration for the Joystick
  std::vector<espp::AdcConfig> channels{
    {
      // Ry
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_5,
      .attenuation = ADC_ATTEN_DB_11
    },
    {
      // Rx
      .unit = ADC_UNIT_1,
      .channel = ADC_CHANNEL_6,
      .attenuation = ADC_ATTEN_DB_11
    }
  };
  auto& ry = channels[0];
  auto& rx = channels[1];
  espp::OneshotAdc adc({
      .unit = ADC_UNIT_1,
      .channels = channels,
    });

  // create the gamepad
  BleGamepad ble_gamepad("LodeStone", "Backbone", 95);
  BleGamepadConfiguration ble_gamepad_config;
  ble_gamepad_config.setAutoReport(false); // normally true
  // and start advertising
  ble_gamepad.begin(&ble_gamepad_config);

  // now that we're good let's start sending data
  auto report_period = 50ms;
  while (true) {
    if (ble_gamepad.isConnected()) {
      auto start = std::chrono::high_resolution_clock::now();
      // read the state of the controller
      controller.update();
      // send the hid report
      ble_gamepad.press(BUTTON_5);
      ble_gamepad.sendReport();
      // try to get exactly the desired report rate
      std::this_thread::sleep_until(start + report_period);
    } else {
      printf("\x1B[1A"); // go up a line
      printf("\x1B[2K\r"); // erase the line
      logger.warn("[{}] Not connected, waiting for BLE HID Host connection...",
                  std::chrono::high_resolution_clock::now());
      // so just wait a little longer..
      std::this_thread::sleep_for(1s);
    }
  }
}
