#include "cameras.h"
#include <Arduino.h>

// Two ESP32-CAM modules with OV2640, controlled over UART.
// Teensy 4.1: ground camera Serial2 (RX2/TX2), top camera Serial3 (RX3/TX3).
// StateLogic: camera1 at apogee, camera2 at payload release.
// ESP32-CAM sketch listens on its Serial and reacts to:
//  - "START\n" to begin writing frames to SD
//  - "STOP\n" to stop recording

static HardwareSerial& CAM1_SERIAL = Serial2;  // ground
static HardwareSerial& CAM2_SERIAL = Serial3;  // top

static bool camerasInitialized = false;
static bool camera1Recording = false;
static bool camera2Recording = false;

void initCameras() {
    // Match camera_tester.ino: ESP32-CAM uses Serial at 115200 baud
    CAM1_SERIAL.begin(115200);
    CAM2_SERIAL.begin(115200);
    camerasInitialized = true;
    camera1Recording = false;
    camera2Recording = false;
}

void startCamera1Recording() {
    if (!camerasInitialized || camera1Recording) return;
    CAM1_SERIAL.print("START\n");
    camera1Recording = true;
}

void stopCamera1Recording() {
    if (!camerasInitialized || !camera1Recording) return;
    CAM1_SERIAL.print("STOP\n");
    camera1Recording = false;
}

void startCamera2Recording() {
    if (!camerasInitialized || camera2Recording) return;
    CAM2_SERIAL.print("START\n");
    camera2Recording = true;
}

void stopCamera2Recording() {
    if (!camerasInitialized || !camera2Recording) return;
    CAM2_SERIAL.print("STOP\n");
    camera2Recording = false;
}

