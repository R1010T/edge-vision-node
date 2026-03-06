/*
 * camera_pins.h
 * OV2640 pin map for Freenove ESP32-S3 WROOM dev board.
 * Pin assignments taken from the Freenove pinout diagram.
 * PWDN and RESET are not connected on this board (sensor manages its own reset).
 */

#pragma once

#define CAM_PIN_PWDN    -1   // not connected
#define CAM_PIN_RESET   -1   // not connected
#define CAM_PIN_XCLK    15
#define CAM_PIN_SIOD     4   // SCCB data
#define CAM_PIN_SIOC     5   // SCCB clock

#define CAM_PIN_D7      16   // Y9 (MSB)
#define CAM_PIN_D6      17   // Y8
#define CAM_PIN_D5      18   // Y7
#define CAM_PIN_D4      12   // Y6
#define CAM_PIN_D3      10   // Y5
#define CAM_PIN_D2       8   // Y4
#define CAM_PIN_D1       9   // Y3
#define CAM_PIN_D0      11   // Y2 (LSB)

#define CAM_PIN_VSYNC    6
#define CAM_PIN_HREF     7
#define CAM_PIN_PCLK    13
