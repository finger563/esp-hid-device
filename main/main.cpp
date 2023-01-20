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

  // NOTE: the QtPy S3 has a NeoPixel on GPIO39 (power for it is GPIO38)

  // NOTE: controller mapping:
  // * R2 -> A0 (GPIO18 / ADC2_CH7)
  // * R1 -> A1 (GPIO17 / ADC2_CH6)
  // * X  -> MI (GPIO37)
  // * Y  -> A3 (GPIO8)
  // * Rx -> SDA (GPIO7 / ADC1_CH6)
  // * Ry -> SCL (GPIO6 / ADC1_CH5)
  // * Menu -> TX (GPIO5)
  // * Home -> RX (GPIO16)
  // * R3 -> SCK (GPIO36)
  // * A  -> MO (GPIO35)
  // * B  -> A2 (GPIO9)

  // create the controller (digital buttons) object
  espp::Controller controller(espp::Controller::DualConfig{
      // buttons short to ground, so they are active low. this will enable the
      // GPIO_PULLUP and invert the logic
      .active_low = true,
      .gpio_a = 35,
      .gpio_b = 9,
      .gpio_x = 37,
      .gpio_y = 8,
      .gpio_start = 16,
      .gpio_select = 5,
      .gpio_joystick_select = 36,
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
  auto& ry_channel = channels[0];
  auto& rx_channel = channels[1];
  espp::OneshotAdc adc({
      .unit = ADC_UNIT_1,
      .channels = channels,
    });

  while (true) {
    float rx = 0;
    float ry = 0;
    auto maybe_rx = adc.read_mv(rx_channel.channel);
    if (maybe_rx.has_value()) {
      rx = maybe_rx.value();
    }
    auto maybe_ry = adc.read_mv(ry_channel.channel);
    if (maybe_ry.has_value()) {
      ry = maybe_ry.value();
    }
    fmt::print("Joystick:\n"
               "\tX:  {:.3f}\n"
               "\tY:  {:.3f}\n",
               rx,
               ry);
    controller.update();
    bool is_a_pressed = controller.is_pressed(espp::Controller::Button::A);
    bool is_b_pressed = controller.is_pressed(espp::Controller::Button::B);
    bool is_x_pressed = controller.is_pressed(espp::Controller::Button::X);
    bool is_y_pressed = controller.is_pressed(espp::Controller::Button::Y);
    bool is_select_pressed = controller.is_pressed(espp::Controller::Button::SELECT);
    bool is_start_pressed = controller.is_pressed(espp::Controller::Button::START);
    bool is_joystick_select_pressed = controller.is_pressed(espp::Controller::Button::JOYSTICK_SELECT);
    fmt::print("Controller buttons:\n"
               "\tA:      {}\n"
               "\tB:      {}\n"
               "\tX:      {}\n"
               "\tY:      {}\n"
               "\tMenu:   {}\n"
               "\tHome:   {}\n"
               "\tR3:     {}\n",
               is_a_pressed,
               is_b_pressed,
               is_x_pressed,
               is_y_pressed,
               is_select_pressed,
               is_start_pressed,
               is_joystick_select_pressed
               );
    std::this_thread::sleep_for(100ms);
  }

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
