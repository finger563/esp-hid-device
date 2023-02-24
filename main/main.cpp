#include "sdkconfig.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string.h>

#include "driver/i2c.h"
#include "esp_sleep.h"
#include "esp_pm.h"

#include "logger.hpp"
#include "task.hpp"

#include "gamepad.hpp"

#define I2C_NUM         (I2C_NUM_1)
#define I2C_SCL_IO      (GPIO_NUM_22)
#define I2C_SDA_IO      (GPIO_NUM_21)
#define I2C_FREQ_HZ     (400 * 1000)
#define I2C_TIMEOUT_MS  (10)

using namespace std::chrono_literals;

bool operator ==(const espp::Vector2d<float>& lhs, const espp::Vector2d<float>& rhs) {
  return
    lhs.x() == rhs.x() &&
    lhs.y() == rhs.y();
}

extern "C" void app_main(void) {
  esp_err_t err;
  espp::Logger logger({.tag = "LodeStone", .level = espp::Logger::Verbosity::INFO});
  logger.info("Bootup");

#if CONFIG_PM_ENABLE
    // Configure dynamic frequency scaling:
    // maximum and minimum frequencies are set in sdkconfig,
    // automatic light sleep is enabled if tickless idle support is enabled.
    esp_pm_config_esp32_t pm_config = {
            .max_freq_mhz = 240,
            .min_freq_mhz = 40,
            .light_sleep_enable = true
    };
    ESP_ERROR_CHECK( esp_pm_configure(&pm_config) );
#endif // CONFIG_PM_ENABLE

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

  // make some lambda functions we'll use to read/write to the devices on the i2c bus
  auto i2c_write = [](uint8_t dev_addr, uint8_t *data, size_t data_len) {
    i2c_master_write_to_device(I2C_NUM,
                               dev_addr,
                               data,
                               data_len,
                               I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
  };

  auto i2c_read = [](uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len) {
    i2c_master_write_read_device(I2C_NUM,
                                 dev_addr,
                                 &reg_addr,
                                 1,
                                 data,
                                 data_len,
                                 I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
  };

  Gamepad gamepad({
      .bus_write = i2c_write,
      .bus_read = i2c_read,
      .left_io_address = espp::Aw9523::DEFAULT_ADDRESS,
      .left_adc_address = espp::Ads1x15::DEFAULT_ADDRESS,
      .right_io_address = espp::Aw9523::DEFAULT_ADDRESS,
      .right_adc_address = espp::Ads1x15::DEFAULT_ADDRESS,
    });

  // NOTE: for debugging pure controller functionality
  while (false) {
    gamepad.update();
    logger.info("{}", gamepad.to_string());
    std::this_thread::sleep_for(100ms);
  }

  // create the gamepad (third parameter is starting battery level)
  BleGamepad ble_gamepad("LodeStone", "Backbone", 95);
  BleGamepadConfiguration ble_gamepad_config;
  ble_gamepad_config.setAutoReport(false); // normally true
  // and start advertising
  ble_gamepad.begin(&ble_gamepad_config);

  // now that we're good let's start sending data
  auto report_period = 500ms;
  // set the previous state
  ButtonState previous_state = gamepad.get_button_state();
  while (true) {
    if (ble_gamepad.isConnected()) {
      auto start = std::chrono::high_resolution_clock::now();

      // NOTE: if you run this without the ADS connected, it will fail
      gamepad.update();

      // NOTE: for debugging....
      logger.info("{}", gamepad.to_string());

      auto right_position = gamepad.get_right_joystick_position();
      auto left_position = gamepad.get_left_joystick_position();
      auto button_state = gamepad.get_button_state();
      auto l2 = gamepad.get_trigger_l2();
      auto r2 = gamepad.get_trigger_r2();

      // build the report by setting the gamepad state
      if (button_state.a) {
        ble_gamepad.press(BUTTON_1);
      } else {
        ble_gamepad.release(BUTTON_1);
      }
      if (button_state.b) {
        ble_gamepad.press(BUTTON_2);
      } else {
        ble_gamepad.release(BUTTON_2);
      }
      if (button_state.x) {
        ble_gamepad.press(BUTTON_4);
      } else {
        ble_gamepad.release(BUTTON_4);
      }
      if (button_state.y) {
        ble_gamepad.press(BUTTON_5);
      } else {
        ble_gamepad.release(BUTTON_5);
      }
      if (button_state.menu) {
        ble_gamepad.pressSelect();
      } else {
        ble_gamepad.releaseSelect();
      }
      if (button_state.home) {
        ble_gamepad.pressStart();
      } else {
        ble_gamepad.releaseStart();
      }

      // set the d-pad (HAT1)
      ble_gamepad.setHat1(button_state.get_hat_value());

      // set the Rx / Ry analog stick values in the report
      auto right_int = right_position * 16384.0f + espp::Vector2f(16384.0f, 16384.0f);
      ble_gamepad.setRightThumb((int16_t)-right_int.x(), (int16_t)-right_int.y());

      // set the Lx / Ly analog stick values in the report
      auto left_int = left_position * 16384.0f + espp::Vector2f(16384.0f, 16384.0f);
      ble_gamepad.setLeftThumb((int16_t)-left_int.x(), (int16_t)-left_int.y());

      // set the left trigger in gamepad
      ble_gamepad.setLeftTrigger((int16_t)(255.0f * l2));

      // set the right trigger in gamepad
      ble_gamepad.setRightTrigger((int16_t)(255.0f * r2));

      // send the hid report, only if the button state changed?
      bool state_changed = previous_state != button_state;
      if (state_changed) {
        logger.info("State changed, sending report!");
        ble_gamepad.sendReport();
      }
      previous_state = button_state;
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
