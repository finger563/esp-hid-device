#pragma once

#include "ble_gamepad.hpp"

#include "ads1x15.hpp"
#include "aw9523.hpp"
#include "logger.hpp"
#include "joystick.hpp"

#include "button_state.hpp"

class Gamepad {
public:
  /**
   * @brief Function to write bytes to the device.
   * @param dev_addr Address of the device to write to.
   * @param data Pointer to array of bytes to write.
   * @param data_len Number of data bytes to write.
   */
  typedef std::function<void(uint8_t dev_addr, uint8_t *data, size_t data_len)> bus_write_fn;

  /**
   * @brief Function to read bytes from the device.
   * @param dev_addr Address of the device to write to.
   * @param reg_addr Register address to read from.
   * @param data Pointer to array of bytes to read into.
   * @param data_len Number of data bytes to read.
   */
  typedef std::function<void(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, size_t data_len)> bus_read_fn;

  /**
   * @brief Configuration structure for the gamepad.
   */
  struct Config {
    bus_write_fn bus_write; ///< Function to be used to write bytes to the I2C bus.
    bus_read_fn bus_read; ///< Function to be used to read bytes from the I2C bus.
    uint8_t left_io_address; ///< Device address for the left-side IO expander
    uint8_t left_adc_address; ///< Device address for the left-side ADC expander
    uint8_t right_io_address; ///< Device address for the right-side IO expander
    uint8_t right_adc_address; ///< Device address for the right-side ADC expander
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Verbosity for the gamepad logger
  };

  /**
   * @brief Initialize the gamepad to talk to IO expanders for the left/right
   *        sides.
   * @param config The configuration structure for the gamepad.
   */
  Gamepad(const Config& config)
    : left_io_({
        .device_address = config.left_io_address,
        .port_0_direction_mask = LEFT_PORT_0_PIN_MASK,
        .port_1_direction_mask = LEFT_PORT_1_PIN_MASK,
        .write = config.bus_write,
        .read = config.bus_read}),
      left_adc_(espp::Ads1x15::Ads1015Config{
        .device_address = config.left_adc_address,
        .write = config.bus_write,
        .read = config.bus_read}),
      right_io_({
        .device_address = config.right_io_address,
        .port_0_direction_mask = RIGHT_PORT_0_PIN_MASK,
        .port_1_direction_mask = RIGHT_PORT_1_PIN_MASK,
        .write = config.bus_write,
        .read = config.bus_read}),
      right_adc_(espp::Ads1x15::Ads1015Config{
        .device_address = config.right_adc_address,
        .write = config.bus_write,
        .read = config.bus_read}),
      left_joystick_({
          .x_calibration = {.center=0.0f, .deadband=0.2f, .minimum=-1.0f, .maximum=1.0f},
            .y_calibration = {.center=0.0f, .deadband=0.2f, .minimum=-1.0f, .maximum=1.0f},
            .get_values = [this](float *x, float *y) -> bool {
              auto x_mv = left_adc_.sample_mv(LX_CHANNEL);
              auto y_mv = left_adc_.sample_mv(LY_CHANNEL);
              // convert [0, 3300]mV to approximately [-1.0f, 1.0f]
              *x = (float)(x_mv) / 1700.0f - 1.0f;
              *y = (float)(y_mv) / 1700.0f - 1.0f;
              return true;
            }
            }),
      right_joystick_({
          .x_calibration = {.center=0.0f, .deadband=0.2f, .minimum=-1.0f, .maximum=1.0f},
            .y_calibration = {.center=0.0f, .deadband=0.2f, .minimum=-1.0f, .maximum=1.0f},
            .get_values = [this](float *x, float *y) -> bool {
              auto x_mv = right_adc_.sample_mv(RX_CHANNEL);
              auto y_mv = right_adc_.sample_mv(RY_CHANNEL);
              // convert [0, 3300]mV to approximately [-1.0f, 1.0f]
              *x = (float)(x_mv) / 1700.0f - 1.0f;
              *y = (float)(y_mv) / 1700.0f - 1.0f;
              return true;
            }
            }),
      logger_({.tag = "Gamepad", .level = config.log_level}) {
    // no LEDs on port 0, LEDs are on port 1.
    right_io_.configure_led(espp::Aw9523::Port::PORT1, LED_PIN_MASK);
  }

  /**
   * @brief Get the most recent state of the gamepad at the last time update()
   *        was called.
   * @return The state of each input.
   */
  ButtonState get_button_state() const { return button_state_; }

  /**
   * @brief Get the most recent left joystick position at the last time update()
   *        was called.
   * @return The most recent left joystick position, as a floating point
   *         2-vector.
   */
  espp::Vector2f get_left_joystick_position() const { return left_joystick_.position(); }

  /**
   * @brief Get the most recent right joystick position at the last time update()
   *        was called.
   * @return The most recent right joystick position, as a floating point
   *         2-vector.
   */
  espp::Vector2f get_right_joystick_position() const { return right_joystick_.position(); }

  /**
   * @brief Get the most recent L2 trigger value at the last time update() was
   *        called.
   * @return L2 value in the range [0,1]
   */
  float get_trigger_l2() const { return trigger_l2_; }

  /**
   * @brief Get the most recent R2 trigger value at the last time update() was
   *        called.
   * @return r2 value in the range [0,1]
   */
  float get_trigger_r2() const { return trigger_r2_; }

  /**
   * @brief Read the IO/ADC expanders to get the latest state, updating the
   *        internal state variables.
   */
  void update() {
    logger_.info("update");
    // get the latest port data from left/right sides
    auto left_pins = left_io_.get_pins();
    auto right_pins = right_io_.get_pins();
    uint8_t right_port_0 = right_pins & 0xff;
    uint8_t right_port_1 = (right_pins >> 8) & 0xff;
    uint8_t left_port_0 = left_pins & 0xff;
    uint8_t left_port_1 = (left_pins >> 8) & 0xff;
    // update the button state variables
    button_state_.a = right_port_0 & A_BIT;
    button_state_.b = right_port_0 & B_BIT;
    button_state_.x = right_port_0 & X_BIT;
    button_state_.y = right_port_0 & Y_BIT;
    button_state_.r1 = right_port_0 & R1_BIT;
    button_state_.r3 = right_port_0 & R3_BIT;
    button_state_.home = right_port_1 & HOME_BIT;
    button_state_.menu = right_port_1 & MENU_BIT;
    button_state_.up = left_port_0 & UP_BIT;
    button_state_.down = left_port_0 & DOWN_BIT;
    button_state_.left = left_port_0 & LEFT_BIT;
    button_state_.right = left_port_0 & RIGHT_BIT;
    button_state_.l1 = left_port_0 & L1_BIT;
    button_state_.l3 = left_port_0 & L3_BIT;
    button_state_.capture = left_port_1 & CAPTURE_BIT;
    button_state_.options = left_port_1 & OPTIONS_BIT;
    // get the latest joystick data from left/right sides
    left_joystick_.update();
    right_joystick_.update();
    // get the latest trigger data from left/right sides
    auto l2_mv = left_adc_.sample_mv(L2_CHANNEL);
    auto r2_mv = right_adc_.sample_mv(R2_CHANNEL);
    // update the trigger state
    // convert [0, 3300]mV to [0.0f, 1.0f]
    trigger_l2_ = (float)(l2_mv) / 3300.0f;
    trigger_r2_ = (float)(r2_mv) / 3300.0f;
  }

  /**
   * @brief Set's the RGB LED color / brightness.
   * @param r Brightness of red
   * @param g Brightness of green
   * @param b Brightness of blue
   */
  void set_rgb(uint8_t r, uint8_t g, uint8_t b) {
    logger_.info("Setting RGB to [{}, {}, {}]", r, g, b);
    // NOTE: we left shift the LED bits by 8 since they are on Port 1 and are
    //       therefore in the high byte.
    right_io_.led(LED_R_BIT << 8, r);
    right_io_.led(LED_G_BIT << 8, g);
    right_io_.led(LED_B_BIT << 8, b);
  }

  std::string to_string() {
    return fmt::format("Gamepad State:\n"
                       "\tUp:      {}\n"
                       "\tDown:    {}\n"
                       "\tLeft:    {}\n"
                       "\tRight:   {}\n"
                       "\tCapture: {}\n"
                       "\tOptions: {}\n"
                       "\tL1:      {}\n"
                       "\tL2:      {:.2f}\n"
                       "\tL3:      {}\n"
                       "\tLX:      {:.2f}\n"
                       "\tLY:      {:.2f}\n"
                       "-------------\n"
                       "\tA:       {}\n"
                       "\tB:       {}\n"
                       "\tX:       {}\n"
                       "\tY:       {}\n"
                       "\tMenu:    {}\n"
                       "\tHome:    {}\n"
                       "\tR1:      {}\n"
                       "\tR2:      {:.2f}\n"
                       "\tR3:      {}\n"
                       "\tRX:      {:.2f}\n"
                       "\tRY:      {:.2f}",
                       (bool)button_state_.up,
                       (bool)button_state_.down,
                       (bool)button_state_.left,
                       (bool)button_state_.right,
                       (bool)button_state_.capture,
                       (bool)button_state_.options,
                       (bool)button_state_.l1,
                       trigger_l2_,
                       (bool)button_state_.l3,
                       left_joystick_.x(),
                       left_joystick_.y(),
                       // right side
                       (bool)button_state_.a,
                       (bool)button_state_.b,
                       (bool)button_state_.x,
                       (bool)button_state_.y,
                       (bool)button_state_.menu,
                       (bool)button_state_.home,
                       (bool)button_state_.r1,
                       trigger_r2_,
                       (bool)button_state_.r3,
                       right_joystick_.x(),
                       right_joystick_.y());
  }

protected:
  // PORT 0 of the right AW9523
  static constexpr int A_BIT  = (1<<0);
  static constexpr int B_BIT  = (1<<1);
  static constexpr int X_BIT  = (1<<2);
  static constexpr int Y_BIT  = (1<<3);
  static constexpr int R1_BIT = (1<<4);
  static constexpr int R3_BIT = (1<<5);
  static constexpr uint8_t RIGHT_PORT_0_PIN_MASK = A_BIT | B_BIT | X_BIT | Y_BIT | R1_BIT | R3_BIT;

  // PORT 1 of the right AW9523
  static constexpr int HOME_BIT    = (1<<0);
  static constexpr int MENU_BIT    = (1<<1);
  static constexpr int LED_R_BIT   = (1<<5);
  static constexpr int LED_G_BIT   = (1<<6);
  static constexpr int LED_B_BIT   = (1<<7);
  static constexpr uint8_t RIGHT_PORT_1_PIN_MASK = HOME_BIT | MENU_BIT;
  static constexpr int LED_PIN_MASK= ~(LED_R_BIT | LED_G_BIT | LED_B_BIT); // 1=gpio, 0=led

  // PORT 0 of the left AW9523
  static constexpr int UP_BIT    = (1<<0);
  static constexpr int DOWN_BIT  = (1<<1);
  static constexpr int LEFT_BIT  = (1<<2);
  static constexpr int RIGHT_BIT = (1<<3);
  static constexpr int L1_BIT    = (1<<4);
  static constexpr int L3_BIT    = (1<<5);
  static constexpr uint8_t LEFT_PORT_0_PIN_MASK = UP_BIT | DOWN_BIT | LEFT_BIT | RIGHT_BIT | L1_BIT | L3_BIT;

  // PORT 1 of the left AW9523
  static constexpr int CAPTURE_BIT = (1<<7);
  static constexpr int OPTIONS_BIT = (1<<6);
  static constexpr uint8_t LEFT_PORT_1_PIN_MASK = CAPTURE_BIT | OPTIONS_BIT;

  // The channels on the ADS1015 that the left joystick and left trigger are wired
  // into.
  static constexpr int LX_CHANNEL = 0;
  static constexpr int LY_CHANNEL = 1;
  static constexpr int L2_CHANNEL = 3;

  // The channels on the ADS1015 that the right joystick and right trigger are wired
  // into.
  static constexpr int RX_CHANNEL = 0;
  static constexpr int RY_CHANNEL = 1;
  static constexpr int R2_CHANNEL = 3;

  // I/O
  espp::Aw9523 left_io_;
  espp::Ads1x15 left_adc_;
  espp::Aw9523 right_io_;
  espp::Ads1x15 right_adc_;

  // state
  ButtonState button_state_;
  espp::Joystick left_joystick_;
  espp::Joystick right_joystick_;
  float trigger_l2_{0};
  float trigger_r2_{0};

  espp::Logger logger_;
};
