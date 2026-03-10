#ifndef CAMERAS_H
#define CAMERAS_H

// Initialize camera serial links (ESP32-CAM modules)
void initCameras();

// Start recording on both cameras
void startCamerasRecording();

// Stop recording on both cameras
void stopCamerasRecording();

#endif // CAMERAS_H

