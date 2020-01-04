#pragma once

struct __attribute__((packed)) OpenTrack {
    double x;
    double y;
    double z;
    double yaw;
    double pitch;
    double roll;
};

struct __attribute__((packed)) FreePie {
    uint8_t pad;
    uint8_t mask;
    float yaw;
    float pitch;
    float roll;
    float x;
    float y;
    float z;
    float a;
    float b;
    float c;
    float d;
    float e;
    float f;
};