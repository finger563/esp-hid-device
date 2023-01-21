#pragma once

#include "ble_gamepad.hpp"

struct LeftGamepadState {
  uint32_t up : 1;
  uint32_t down : 1;
  uint32_t left : 1;
  uint32_t right : 1;
  uint32_t l1 : 1;
  uint32_t l3 : 1;
  uint32_t capture : 1;
  uint32_t options : 1;

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
};

bool operator !=(const LeftGamepadState& lhs, const LeftGamepadState& rhs) {
  return
    lhs.capture != rhs.capture ||
    lhs.options != rhs.options ||
    lhs.up != rhs.up ||
    lhs.down != rhs.down ||
    lhs.left != rhs.left ||
    lhs.right != rhs.right ||
    lhs.l3 != rhs.l3 ||
    lhs.l1 != rhs.l1;
}

// PORT B of the MCP23017
static constexpr int CAPTURE_BIT = (1<<7);
static constexpr int OPTIONS_BIT = (1<<6);
static constexpr int L1_BIT      = (1<<5);
static constexpr int L3_BIT      = (1<<4);
static constexpr int PORT_B_PIN_MASK = CAPTURE_BIT | OPTIONS_BIT | L1_BIT | L3_BIT;

// PORT A of the MCP23017
static constexpr int UP_BIT    = (1<<0);
static constexpr int DOWN_BIT  = (1<<1);
static constexpr int LEFT_BIT  = (1<<2);
static constexpr int RIGHT_BIT = (1<<3);
static constexpr int PORT_A_PIN_MASK = UP_BIT | DOWN_BIT | LEFT_BIT | RIGHT_BIT;

LeftGamepadState get_left_gamepad_state(uint8_t port_a, uint8_t port_b) {
  LeftGamepadState state;
  state.up = port_a & UP_BIT;
  state.down = port_a & DOWN_BIT;
  state.left = port_a & LEFT_BIT;
  state.right = port_a & RIGHT_BIT;
  state.capture = port_b & CAPTURE_BIT;
  state.options = port_b & OPTIONS_BIT;
  state.l1 = port_b & L1_BIT;
  state.l3 = port_b & L3_BIT;
  return state;
}
