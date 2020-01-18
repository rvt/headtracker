#pragma once

#include <hwheadtrack.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

//volatile bool HWHeadTrackmpu6050_mpuInterrupt = false;         // indicates whether MPU interrupt pin has gone high
class MPU6050;

/**
 * 6DOF MPU6050 HeadTracker
 */
class HWHeadTrackmpu6050 final : public HWHeadTrack {
private:
    MPU6050* mpu;
    bool _dmpReady = false;             // Indicate that the DMP is ready
    uint16_t packetSize;                        // expected DMP packet size (default is 42 bytes)
    uint8_t fifoBuffer[64];                     // FIFO storage buffer
    uint16_t fifoCount;                         // count of all bytes currently in FIFO
    HWHeadTrack_Orientation pOrientation;
    bool calibrated = false;
public:
    HWHeadTrackmpu6050();

    virtual bool isReady() const {
        return _dmpReady && !calibrated;
    }
    virtual void setup(JsonObject json);
    virtual bool loop();
    virtual void calibrate(JsonObject& jsonObject);
    virtual HWHeadTrack_Orientation getOrientation() const {
        return pOrientation;
    }
    virtual String name() const {
        return "mpu6050";
    }
};
