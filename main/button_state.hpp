#pragma once

#include "ble_gamepad.hpp"

/**
 *  @brief Represents the current digital state of the gamepad (what buttons
 *  are pressed).
 */
struct ButtonState {
  // right side
  uint32_t a : 1; ///< A button
  uint32_t b : 1; ///< B button
  uint32_t x : 1; ///< X button
  uint32_t y : 1; ///< Y button
  uint32_t r1 : 1; ///< R1 (bumper) button
  uint32_t r3 : 1; ///< R3 (right stick press) button
  uint32_t home : 1; ///< Home button
  uint32_t menu : 1; ///< Menu button
  // left side
  uint32_t up : 1; ///< D-pad up button
  uint32_t down : 1; ///< D-pad down button
  uint32_t left : 1; ///< D-pad left button
  uint32_t right : 1; ///< D-pad right button
  uint32_t l1 : 1; ///< L1 (bumper) button
  uint32_t l3 : 1; ///< L3 (left stick press) button
  uint32_t capture : 1; ///< Capture button
  uint32_t options : 1; ///< Options button

  /**
   * @brief Turn the d-pad up/down/left/right buttons into a a HAT value.
   * @return One of the 8 possible HAT values corresponding to the d-pad.
   */
  int get_hat_value() const {
    uint8_t bitmask =
      (up << 0) |
      (down << 1) |
      (left << 2) |
      (right << 3);
    switch (bitmask) {
    case 0:
    default:
      return HAT_CENTERED;
    case 1:
      return HAT_UP;
    case 2:
      return HAT_DOWN;
    case 3:
      // impossible, both up and down
      return HAT_CENTERED;
    case 4:
      return HAT_LEFT;
    case 5:
      return HAT_UP_LEFT;
    case 6:
      return HAT_DOWN_LEFT;
    case 7:
      // impossible, both up/down
      return HAT_CENTERED;
    case 8:
      return HAT_RIGHT;
    case 9:
      return HAT_UP_RIGHT;
    case 10:
      return HAT_DOWN_RIGHT;
      // other impossible states:
    case 11:
    case 12:
    case 13:
    case 14:
    case 15:
      return HAT_CENTERED;
    }
  }

  friend bool operator==(const ButtonState& lhs, const ButtonState& rhs) {
    return
      lhs.home == rhs.home &&
      lhs.menu == rhs.menu &&
      lhs.a == rhs.a &&
      lhs.b == rhs.b &&
      lhs.x == rhs.x &&
      lhs.y == rhs.y &&
      lhs.r1 == rhs.r1 &&
      lhs.r3 == rhs.r3 &&
      lhs.capture == rhs.capture &&
      lhs.options == rhs.options &&
      lhs.up == rhs.up &&
      lhs.down == rhs.down &&
      lhs.left == rhs.left &&
      lhs.right == rhs.right &&
      lhs.l1 == rhs.l1 &&
      lhs.l3 == rhs.l3;
  }
};
