#ifndef CAMERAS_H
#define CAMERAS_H

// Initialize camera serial links (ESP32-CAM modules)
void initCameras();

// Camera 1: payload separation camera
void startCamera1Recording();
void stopCamera1Recording();

// Camera 2: payload/egg release camera
void startCamera2Recording();
void stopCamera2Recording();

#endif // CAMERAS_H

