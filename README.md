# esp-hid-device

Simple test for a BLE HID Device to send input reports and measure latency (how
long the input report takes to send, be received, and for the receiver to toggle
a pin connected back to this device / sending board).

## Videos

### Sending input report (size = 2B)
NOTE: Receiver is top window, sender is bottom window

https://user-images.githubusercontent.com/213467/206505413-78ade628-ae2e-4f3d-8b6c-05b6673ec763.mp4

### Sending input report (size = 20B)
NOTE: Receiver is top window, sender is bottom window

https://user-images.githubusercontent.com/213467/206505448-c10c6750-ec82-46f1-8efe-8c1f2774aa75.mp4

## Related

* [ESP-IDF BT/BLE HID Device Example](https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/esp_hid_device)
* [ESP BLE Gamepad](https://github.com/lemmingDev/ESP32-BLE-Gamepad)
* [ESP HID Host](https://github.com/finger563/esp-hid-host)
