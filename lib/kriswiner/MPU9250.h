/* 06/16/2017 Copyright Tlera Corporation
 *
 *  Created by Kris Winer
 *
 Demonstrate basic MPU-9250 functionality including parameterizing the register addresses, initializing the sensor,
 getting properly scaled accelerometer, gyroscope, and magnetometer data out.
 Addition of 9 DoF sensor fusion using open source Madgwick and Mahony filter algorithms.
 Sketch runs on the 3.3 V Dragonfly STM32L476 Breakout Board.

 Library may be used freely and without limit with attribution.

*/

#pragma once

#include "Arduino.h"
#include "Wire.h"

// Set initial input parameters
#define  AFS_2G  0
#define  AFS_4G  1
#define  AFS_8G  2
#define  AFS_16G 3

#define  MFS_14BITS  0 // 0.6 mG per LSB
#define  MFS_16BITS  1    // 0.15 mG per LSB

#define M_8Hz   0x02
#define M_100Hz 0x06

#define  GFS_250DPS  0
#define  GFS_500DPS  1
#define  GFS_1000DPS 2
#define  GFS_2000DPS 3

// Define I2C addresses of MPU9250
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69   // Device address when ADO = 1
#define AK8963_ADDRESS  0x0C   //  Address of magnetometer
#else
#define MPU9250_ADDRESS 0x68   // Device address when ADO = 0
#define AK8963_ADDRESS  0x0C   //  Address of magnetometer
#endif


class MPU9250 {
public:
    MPU9250(uint8_t intPin);
    uint8_t getMPU9250ID();
    uint8_t getAK8963CID();
    void resetMPU9250();
    void initMPU9250(uint8_t Ascale, uint8_t Gscale, uint8_t sampleRate);
    void initAK8963(uint8_t Mscale, uint8_t Mmode, float* destination);
    float getAres(uint8_t Ascale);
    float getGres(uint8_t Gscale);
    float getMres(uint8_t Mscale);
    void magcalMPU9250(float* dest1, float* dest2, float mRes);
    void calibrateMPU9250(float* dest1, float* dest2);
    void SelfTest(float* destination);
    void readMPU9250Data(int16_t* destination);
    void readAccelData(int16_t* destination);
    void readGyroData(int16_t* destination);
    bool checkNewAccelGyroData();
    bool checkNewMagData();
    void readMagData(int16_t* destination);
    int16_t readGyroTempData();
    void gyromagSleep();
    void gyromagWake(uint8_t Mmode);
    void accelWakeOnMotion();
    bool checkWakeOnMotion();
    //  void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
    void I2Cscan();
    void    writeByte(uint8_t address, uint8_t subAddress, uint8_t data);
    uint8_t readByte(uint8_t address, uint8_t subAddress);
    void    readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest);
private:
    uint8_t _intPin;
    uint8_t _Mmode;
    float _fuseROMx;
    float _fuseROMy;
    float _fuseROMz;
    float _magCalibration[3];
};

