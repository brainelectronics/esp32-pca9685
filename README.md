# ESP32 PCA9685 PWM/Servo Driver Library

---

This is a example for the 16-channel PWM & Servo driver PCA9585

#### Connections
| ESP32 pin | PCA9685 | Notes |
| --------- | ------- | ----- |
| Any output pin | SCL | currenty pin 4 is used |
| Any output pin | SDA | currenty pin 5 is used |
| GND | GND  | Power supply ground |
| 3.3V | Vcc  | Power supply positive |

---

#### How to build

Configure your esp32 build environment as for **esp-idf examples**

Clone the repository

`git clone https://github.com/brainelectronics/esp32-pca9685`

Execute menuconfig and configure your Serial flash config and other settings. Included *sdkconfig.defaults* sets some defaults to be used.

`make menuconfig`

Make and flash the example.

`make all && make flash`

---
