PIR_WISUN — Wi-SUN Sensor Node with PIR and Vibration Detection
Wi-SUN FAN 1.1 sensor node for EFR32xG28 (BRD2705A) that detects motion and vibration and transmits JSON data over Wi-SUN UDP to a Border Router.
Features:

Wi-SUN FAN 1.1 FFN (Full Function Node) automatic connection to Border Router
PIR motion detection via YS312 sensor with adaptive EMA-based algorithm
Vibration detection via MPU-6500 (MPU9265) IMU sensor
Adjustable sensitivity via potentiometer (IADC)
60-second warmup period for baseline calibration
JSON UDP packets sent to Border Router on motion/vibration detection
LED and buzzer alert on detection event
FreeRTOS task architecture, CMake build system

Hardware: Silicon Labs BRD2705A (EFR32xG28 Explorer Kit)
Sensors:

YS312 PIR sensor → PC01 (DOCI)
MPU-6500 IMU → PC03/PC04 (I2C0 SDA/SCL)
Potentiometer → PA05 (IADC0)
Buzzer → PC02
LED1 → on-board