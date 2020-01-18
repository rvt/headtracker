#pragma once

#include <hwheadtrack.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include "mpu9250.h"

/**
 * 6DOF MPU6050 HeadTracker
 */
class HWHeadTrackmpu9250 final : public HWHeadTrack {
private:
    MPU9250* mpu;
    HWHeadTrack_Orientation pOrientation;

    // MPU9250 Configuration
    // Specify sensor full scale
    /* Choices are:
    *  Gscale: GFS_250 == 250 dps, GFS_500 DPS == 500 dps, GFS_1000 == 1000 dps, and GFS_2000DPS == 2000 degrees per second gyro full scale
    *  Ascale: AFS_2G == 2 g, AFS_4G == 4 g, AFS_8G == 8 g, and AFS_16G == 16 g accelerometer full scale
    *  Mscale: MFS_14BITS == 0.6 mG per LSB and MFS_16BITS == 0.15 mG per LSB
    *  Mmode: Mmode == M_8Hz for 8 Hz data rate or Mmode = M_100Hz for 100 Hz data rate
    *  (1 + sampleRate) is a simple divisor of the fundamental 1000 kHz rate of the gyro and accel, so
    *  sampleRate = 0x00 means 1 kHz sample rate for both accel and gyro, 0x04 means 200 Hz, etc.
    */
    uint8_t Gscale = GFS_250DPS, Ascale = AFS_2G, Mscale = MFS_16BITS, Mmode = M_100Hz, sampleRate = 0x04;
    float aRes, gRes, mRes;      // scale resolutions per LSB for the sensors
    // global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
    float GyroMeasError = PI * (40.0f / 180.0f);   // gyroscope measurement error in rads/s (start at 40 deg/s)
    float GyroMeasDrift = PI * (0.0f  / 180.0f);   // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    float beta = sqrtf(3.0f / 4.0f) * GyroMeasError;   // compute beta
    float zeta = sqrtf(3.0f / 4.0f) * GyroMeasDrift;   // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

    // Variables used for in between measurements
    float   magCalibration[3] = {0, 0, 0};  // Factory mag calibration and mag bias
    float   temperature;    // Stores the MPU9250 internal chip temperature in degrees Celsius

    // These can be measured once and entered here or can be calculated each time the device is powered on
    float   gyroBias[3] = {0.96, -0.21, 0.12}, accelBias[3] = {0.00299, -0.00916, 0.00952};
    float   magBias[3] = {71.04, 122.43, -36.90}, magScale[3]  = {1.01, 1.03, 0.96}; // Bias corrections for gyro and accelerometer

    uint32_t lastUpdate = 0;                   // used to calculate integration interval

    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};    // vector to hold quaternion
    float eInt[3] = {0.0f, 0.0f, 0.0f};       // vector to hold integral error for Mahony method

    bool _dmpReady = false;
    bool calibrated = false;

public:
    HWHeadTrackmpu9250();

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
        return "mpu9250";
    }
};
