#ifndef XBEE_H
#define XBEE_H

// Initialize XBee communication module
void initXBee();

// Check if XBee is connected/ready
bool xbeeReady();

// Send data via XBee
void xbeeSend(const uint8_t* data, size_t length);

// Receive data via XBee (non-blocking)
bool xbeeReceive(uint8_t* buffer, size_t* length);

// Update XBee communication (call periodically)
void updateXBee();

#endif // XBEE_H
