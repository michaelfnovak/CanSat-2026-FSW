#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"
#include <SPI.h>

// ========================
// Camera Pins for Boscam OV2640
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ========================
// SD card pins
#define SD_CS 4  // Chip select
#define SD_MOSI 15
#define SD_MISO 2
#define SD_SCK 14

// ========================
// Globals
bool recording = false;
String filename = "";
int fileIndex = 0;

// ========================
// Setup camera
void initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;  // Medium size for SD
  config.jpeg_quality = 12;
  config.fb_count = 1;

  if(esp_camera_init(&config) != ESP_OK){
    Serial.println("Camera init failed!");
    while(true){ delay(1000); }
  }
  Serial.println("Camera ready.");
}

// ========================
// Setup SD
void initSD() {
  if(!SD_MMC.begin("/sdcard", true)){
    Serial.println("SD Card Mount Failed");
    return;
  }
  uint8_t cardType = SD_MMC.cardType();
  if(cardType == CARD_NONE){
    Serial.println("No SD card attached");
    return;
  }
  Serial.println("SD Card initialized.");
}

// ========================
// Setup
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32-CAM ready for Teensy control...");

  initCamera();
  initSD();
}

// ========================
// Loop
void loop() {
  // Check Teensy commands
  if(Serial.available()){
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if(cmd == "START"){
      recording = true;
      filename = "/video_" + String(fileIndex++) + ".jpg";
      Serial.println("Recording started: " + filename);
    } else if(cmd == "STOP"){
      recording = false;
      Serial.println("Recording stopped.");
    }
  }

  // Capture frame if recording
  if(recording){
    camera_fb_t * fb = esp_camera_fb_get();
    if(!fb){
      Serial.println("Camera capture failed!");
      return;
    }

    File file = SD_MMC.open(filename, FILE_WRITE);
    if(file){
      file.write(fb->buf, fb->len);
      file.close();
      Serial.println("Frame saved: " + filename);
    } else {
      Serial.println("Failed to open file!");
    }
    esp_camera_fb_return(fb);

    delay(100); // ~10 FPS, adjust as needed
  }
}