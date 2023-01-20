#include "sdkconfig.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string.h>

#include "driver/i2c.h"

#include "ads1x15.hpp"
#include "controller.hpp"
#include "joystick.hpp"
#include "logger.hpp"
#include "mcp23x17.hpp"
#include "oneshot_adc.hpp"
#include "task.hpp"

#include "ble_gamepad.hpp"

#include "left_gamepad.hpp"

#define I2C_NUM         (I2C_NUM_1)
#define I2C_SCL_IO      (GPIO_NUM_40)
#define I2C_SDA_IO      (GPIO_NUM_41)
#define I2C_FREQ_HZ     (400 * 1000)
#define I2C_TIMEOUT_MS  (10)

using namespace std::chrono_literals;

// NOTE: RIGHT controller mapping:
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

// The channels on the ADS1015 that the left joystick and left trigger are wired
// into.
static constexpr int LX_CHANNEL = 1;
static constexpr int LY_CHANNEL = 2;
static constexpr int L2_CHANNEL = 0;

bool operator !=(const espp::Controller::State& lhs, const espp::Controller::State& rhs) {
  return
    lhs.a != rhs.a ||
    lhs.b != rhs.b ||
    lhs.x != rhs.x ||
    lhs.y != rhs.y ||
    lhs.select != rhs.select ||
    lhs.start != rhs.start ||
    lhs.up != rhs.up ||
    lhs.down != rhs.down ||
    lhs.left != rhs.left ||
    lhs.right != rhs.right ||
    lhs.joystick_select != rhs.joystick_select;
}

extern "C" void app_main(void) {
  esp_err_t err;
  espp::Logger logger({.tag = "LodeStone", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup");

  // make the I2C that we'll use to communicate
  i2c_config_t i2c_cfg;
  logger.info("initializing i2c driver...");
  memset(&i2c_cfg, 0, sizeof(i2c_cfg));
  i2c_cfg.sda_io_num = I2C_SDA_IO;
  i2c_cfg.scl_io_num = I2C_SCL_IO;
  i2c_cfg.mode = I2C_MODE_MASTER;
  i2c_cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
  i2c_cfg.master.clk_speed = I2C_FREQ_HZ;
  err = i2c_param_config(I2C_NUM, &i2c_cfg);
  if (err != ESP_OK) logger.error("config i2c failed");
  err = i2c_driver_install(I2C_NUM, I2C_MODE_MASTER,  0, 0, 0);
  if (err != ESP_OK) logger.error("install i2c driver failed");

  logger.info("Making MCP23017 lambda functions!");
  // make some lambda functions we'll use to read/write to the mcp23x17
  auto mcp23x17_write = [](uint8_t reg_addr, uint8_t value) {
    uint8_t data[] = {reg_addr, value};
    i2c_master_write_to_device(I2C_NUM,
                               espp::Mcp23x17::ADDRESS,
                               data,
                               2,
                               I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
  };

  auto mcp23x17_read = [](uint8_t reg_addr) -> uint8_t{
    uint8_t data;
    i2c_master_write_read_device(I2C_NUM,
                                 espp::Mcp23x17::ADDRESS,
                                 &reg_addr,
                                 1,
                                 &data,
                                 1,
                                 I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    return data;
  };

  logger.info("Making MCP23017");
  // now make the mcp23x17 which handles GPIO
  espp::Mcp23x17 mcp23x17({
      .port_a_direction_mask = (1 << 0),   // input on A0
      .port_a_interrupt_mask = (1 << 0),   // interrupt on A0
      .port_b_direction_mask = (1 << 7),   // input on B7
      .port_b_interrupt_mask = (1 << 7),   // interrupt on B7
      .write = mcp23x17_write,
      .read = mcp23x17_read,
      .log_level = espp::Logger::Verbosity::WARN
    });
  // set pull up on the input pins
  mcp23x17.set_pull_up(espp::Mcp23x17::Port::A, (1<<0));
  mcp23x17.set_pull_up(espp::Mcp23x17::Port::B, (1<<7));

  logger.info("Making ADS1015 lambda functions!");
    // make some lambda functions we'll use to read/write to the i2c adc
    auto ads_write = [](uint8_t reg_addr, uint16_t value) {
      uint8_t write_buf[3] = {reg_addr, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
      i2c_master_write_to_device(I2C_NUM,
                                 espp::Ads1x15::ADDRESS,
                                 write_buf,
                                 3,
                                 I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    };
    auto ads_read = [](uint8_t reg_addr) -> uint16_t {
      uint8_t read_data[2];
      i2c_master_write_read_device(I2C_NUM,
                                   espp::Ads1x15::ADDRESS,
                                   &reg_addr,
                                   1, // size of addr
                                   read_data,
                                   2,
                                   I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
      return (read_data[0] << 8) | read_data[1];
    };

    logger.info("Making ADS1015");
    // make the actual ads class
    espp::Ads1x15 ads(espp::Ads1x15::Ads1015Config{
        .write = ads_write,
        .read = ads_read
      });

    auto read_left_joystick = [&ads](float *x, float *y) -> bool {
      // this will be in mv
      auto x_mv = ads.sample_mv(LX_CHANNEL);
      auto y_mv = ads.sample_mv(LY_CHANNEL);
      // convert [0, 3300]mV to approximately [-1.0f, 1.0f]
      *x = (float)(x_mv) / 1700.0f - 1.0f;
      *y = (float)(y_mv) / 1700.0f - 1.0f;
      return true;
    };
    espp::Joystick left_joystick({
        .x_calibration = {.center = 0.0f, .deadband = 0.2f, .minimum = -1.0f, .maximum = 1.0f},
        .y_calibration = {.center = 0.0f, .deadband = 0.2f, .minimum = -1.0f, .maximum = 1.0f},
        .get_values = read_left_joystick,
      });

  // NOTE: the QtPy S3 has a NeoPixel on GPIO39 (power for it is GPIO38)

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

  // create the adc confgiuration for the right Joystick
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
  espp::OneshotAdc adc({
      .unit = ADC_UNIT_1,
      .channels = channels,
    });

  auto read_right_joystick = [&adc, &channels](float *x, float *y) -> bool {
    // this will be in mv
    auto maybe_x_mv = adc.read_mv(channels[0].channel);
    auto maybe_y_mv = adc.read_mv(channels[1].channel);
    if (maybe_x_mv.has_value() && maybe_y_mv.has_value()) {
      auto x_mv = maybe_x_mv.value();
      auto y_mv = maybe_y_mv.value();
      // convert [0, 3300]mV to approximately [-1.0f, 1.0f]
      *x = (float)(x_mv) / 1700.0f - 1.0f;
      *y = (float)(y_mv) / 1700.0f - 1.0f;
      return true;
    }
    return false;
  };
  espp::Joystick right_joystick({
      .x_calibration = {.center = 0.0f, .deadband = 0.2f, .minimum = -1.0f, .maximum = 1.0f},
      .y_calibration = {.center = 0.0f, .deadband = 0.2f, .minimum = -1.0f, .maximum = 1.0f},
      .get_values = read_right_joystick,
    });


  while (false) {
    controller.update();
    right_joystick.update();
    left_joystick.update();
    // read the left buttons (d-pad, etc.) using MCP23017
    auto mcp_port_a_pins = mcp23x17.get_pins(espp::Mcp23x17::Port::A);
    auto mcp_port_b_pins = mcp23x17.get_pins(espp::Mcp23x17::Port::B);
    auto left_gamepad_state = get_left_gamepad_state(mcp_port_a_pins, mcp_port_b_pins);
    logger.info("Left buttons:\n"
               "\tUp:      {}\n"
               "\tDown:    {}\n"
               "\tLeft:    {}\n"
               "\tRight:   {}\n"
               "\tCapture: {}\n"
               "\tOptions: {}\n"
               "\tL1:      {}\n"
               "\tL3:      {}",
               (bool)left_gamepad_state.up,
               (bool)left_gamepad_state.down,
               (bool)left_gamepad_state.left,
               (bool)left_gamepad_state.right,
               (bool)left_gamepad_state.capture,
               (bool)left_gamepad_state.options,
               (bool)left_gamepad_state.l1,
               (bool)left_gamepad_state.l3
               );


    auto right_position = right_joystick.position();
    auto left_position = left_joystick.position();
    logger.info("Joystick Left: {}", left_position.to_string());
    logger.info("Joystick Right: {}", right_position.to_string());

    bool is_a_pressed = controller.is_pressed(espp::Controller::Button::A);
    bool is_b_pressed = controller.is_pressed(espp::Controller::Button::B);
    bool is_x_pressed = controller.is_pressed(espp::Controller::Button::X);
    bool is_y_pressed = controller.is_pressed(espp::Controller::Button::Y);
    bool is_select_pressed = controller.is_pressed(espp::Controller::Button::SELECT);
    bool is_start_pressed = controller.is_pressed(espp::Controller::Button::START);
    bool is_joystick_select_pressed = controller.is_pressed(espp::Controller::Button::JOYSTICK_SELECT);
    logger.info("Right buttons:\n"
               "\tA:      {}\n"
               "\tB:      {}\n"
               "\tX:      {}\n"
               "\tY:      {}\n"
               "\tMenu:   {}\n"
               "\tHome:   {}\n"
               "\tR3:     {}",
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

  // create the gamepad (third parameter is starting battery level)
  BleGamepad ble_gamepad("LodeStone", "Backbone", 95);
  BleGamepadConfiguration ble_gamepad_config;
  ble_gamepad_config.setAutoReport(false); // normally true
  // and start advertising
  ble_gamepad.begin(&ble_gamepad_config);

  // now that we're good let's start sending data
  auto report_period = 50ms;
  // set the previous state
  espp::Controller::State previous_right_state;
  LeftGamepadState previous_left_state;
  controller.update();
  previous_right_state = controller.get_state();
  while (true) {
    if (ble_gamepad.isConnected()) {
      auto start = std::chrono::high_resolution_clock::now();
      // read the analog (right side)
      right_joystick.update();
      // read the left analog stick using ADS1x15
      // left_joystick.update();
      // read the left buttons (d-pad, etc.) using MCP23017
      auto a_pins = mcp23x17.get_pins(espp::Mcp23x17::Port::A);
      auto b_pins = mcp23x17.get_pins(espp::Mcp23x17::Port::B);
      auto left_gamepad_state = get_left_gamepad_state(a_pins, b_pins);
      // read the state of the controller
      controller.update();
      auto current_right_state = controller.get_state();
      bool state_changed =
        current_right_state != previous_right_state ||
        left_gamepad_state != previous_left_state;
      previous_right_state = current_right_state;
      previous_left_state = left_gamepad_state;

      // build the report by setting the gamepad state
      bool is_a_pressed = controller.is_pressed(espp::Controller::Button::A);
      bool is_b_pressed = controller.is_pressed(espp::Controller::Button::B);
      bool is_x_pressed = controller.is_pressed(espp::Controller::Button::X);
      bool is_y_pressed = controller.is_pressed(espp::Controller::Button::Y);
      bool is_select_pressed = controller.is_pressed(espp::Controller::Button::SELECT);
      bool is_start_pressed = controller.is_pressed(espp::Controller::Button::START);
      bool is_joystick_select_pressed = controller.is_pressed(espp::Controller::Button::JOYSTICK_SELECT);
      if (is_a_pressed) {
        ble_gamepad.press(BUTTON_1);
      } else {
        ble_gamepad.release(BUTTON_1);
      }
      if (is_b_pressed) {
        ble_gamepad.press(BUTTON_2);
      } else {
        ble_gamepad.release(BUTTON_2);
      }
      if (is_x_pressed) {
        ble_gamepad.press(BUTTON_3);
      } else {
        ble_gamepad.release(BUTTON_3);
      }
      if (is_y_pressed) {
        ble_gamepad.press(BUTTON_3);
      } else {
        ble_gamepad.release(BUTTON_3);
      }
      if (is_select_pressed) {
        ble_gamepad.pressSelect();
      } else {
        ble_gamepad.releaseSelect();
      }
      if (is_start_pressed) {
        ble_gamepad.pressStart();
      } else {
        ble_gamepad.releaseStart();
      }

      // set the d-pad (HAT1)
      ble_gamepad.setHat1(left_gamepad_state.get_hat_value());

      // set the Rx / Ry analog stick values in the report
      auto right_int = right_joystick.position() * 16384.0f + espp::Vector2f(16384.0f, 16384.0f);
      ble_gamepad.setRightThumb((int16_t)right_int.x(), (int16_t)right_int.y());

      // set the Lx / Ly analog stick values in the report
      auto left_int = left_joystick.position() * 16384.0f + espp::Vector2f(16384.0f, 16384.0f);
      ble_gamepad.setLeftThumb((int16_t)left_int.x(), (int16_t)left_int.y());

      // TODO: read the left trigger (ads)
      // TODO: set the left trigger in gamepad
      ble_gamepad.setLeftTrigger(0);

      // TODO: read the right trigger (adc)
      // TODO: set the right trigger in gamepad
      ble_gamepad.setRightTrigger(0);

      // send the hid report, only if the button state changed?
      if (state_changed) {
        logger.info("State changed, sending report!");
        ble_gamepad.sendReport();
      }
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
