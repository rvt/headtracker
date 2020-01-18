#pragma once

#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

struct HWHeadTrack_Orientation {
    float yaw;
    float pitch;
    float roll;
    float x;
    float y;
    float z;
};

class HWHeadTrack {
public:
    virtual bool isReady() const = 0;
    virtual void setup(JsonObject json) = 0;
    virtual bool loop() = 0;
    virtual void calibrate(JsonObject& jsonObject) = 0;
    virtual HWHeadTrack_Orientation getOrientation() const;
    virtual String name() const;
};
