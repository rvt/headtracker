#include "hwheadtrackmpu6050.h"
#include <MPU6050_6Axis_MotionApps_V6_12.h>

#define INTERRUPT_PIN                 15 // use pin 15 on ESP8266

volatile bool HWHeadTrackmpu6050_mpuInterrupt = false;

void IRAM_ATTR HWHeadTrackmpu6050_dmpDataReady() {
    HWHeadTrackmpu6050_mpuInterrupt = true;
}

HWHeadTrackmpu6050::HWHeadTrackmpu6050() {
    mpu = new MPU6050();
}

bool HWHeadTrackmpu6050::loop() {

    // if programming failed, don't try to do anything
    if (!isReady() || calibrated) {
        return false;
    }

    // wait for MPU interrupt or extra packet(s) available
    if (!HWHeadTrackmpu6050_mpuInterrupt && fifoCount < packetSize) {
        return false;
    }

    // reset interrupt flag and get INT_STATUS byte
    HWHeadTrackmpu6050_mpuInterrupt = false;
    uint8_t mpuIntStatus = mpu->getIntStatus();

    // get current FIFO count
    fifoCount = mpu->getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu->resetFIFO();
        Serial.println(F("FIFO overflow!"));

    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) {
            fifoCount = mpu->getFIFOCount();
        }

        // read a packet from FIFO
        mpu->getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        float ypr[3];
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorFloat gravity;    // [x, y, z]            gravity vector

        mpu->dmpGetQuaternion(&q, fifoBuffer);
        mpu->dmpGetGravity(&gravity, &q);
        mpu->dmpGetYawPitchRoll(ypr, &q, &gravity);

        //float euler[3];         // [psi, theta, phi]    Euler angle container
        //mpu->dmpGetEuler(euler, &q);

        // Copy to last so we can use this in our webserver
        pOrientation.yaw = ypr[0];
        pOrientation.pitch = ypr[1];
        pOrientation.roll = ypr[2];
        pOrientation.x = 0.0f;
        pOrientation.y = 0.0f;
        pOrientation.z = 0.0f;

        //        sendTracker();

        /*Serial.print("ypr\t");
        Serial.print(pOrientation[0] * 180/3.141);
        Serial.print("\t");
        Serial.print(pOrientation[1] * 180/3.141);
        Serial.print("\t");
        Serial.print(pOrientation[2] * 180/3.141);
        Serial.print("\t");
        Serial.println("");*/
        return true;
    }

    return false;
}

void HWHeadTrackmpu6050::setup(JsonObject json) {
    // initialize device
    if (isReady() || calibrated) {
        return;
    }

    Serial.println(F("Initializing MPU6050..."));
    pinMode(INTERRUPT_PIN, INPUT);
    mpu->initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu->testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    uint8_t devStatus = mpu->dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {

        if (json.containsKey("XGO")) {
            mpu->setXGyroOffset(json["XGO"].as<int>());
            mpu->setYGyroOffset(json["YGO"].as<int>());
            mpu->setZGyroOffset(json["ZGO"].as<int>());
            mpu->setXAccelOffset(json["XAO"].as<int>());
            mpu->setYAccelOffset(json["YAO"].as<int>());
            mpu->setZAccelOffset(json["ZAO"].as<int>());
            Serial.println(F("Using saved calibration."));
        } else {
            // Set some defaults to might be okish....
            mpu->setXGyroOffset(51);
            mpu->setYGyroOffset(8);
            mpu->setZGyroOffset(21);
            mpu->setXAccelOffset(1150);
            mpu->setYAccelOffset(-50);
            mpu->setZAccelOffset(1060);
        }

        // Perform calibration
        //mpu->CalibrateAccel(6);
        //mpu->CalibrateGyro(6);

        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu->setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), HWHeadTrackmpu6050_dmpDataReady, RISING);
        mpu->getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        _dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu->dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void HWHeadTrackmpu6050::calibrate(JsonObject& jsonObject) {
    //JsonObject j;
    //if (!isReady()) return j;
    _dmpReady = false;
    calibrated = true;

    int16_t xgo = mpu->getXGyroOffset();
    int16_t ygo = mpu->getYGyroOffset();
    int16_t zgo = mpu->getZGyroOffset();
    int16_t xao = mpu->getXAccelOffset();
    int16_t yao = mpu->getYAccelOffset();
    int16_t zao = mpu->getZAccelOffset();
    mpu->reset();

    MPU6050 calMpu;
    delay(100);
    calMpu.initialize();

    // Set offset based on last measurement
    calMpu.setXGyroOffset(xgo);
    calMpu.setYGyroOffset(ygo);
    calMpu.setZGyroOffset(zgo);
    calMpu.setXAccelOffset(xao);
    calMpu.setYAccelOffset(yao);
    calMpu.setZAccelOffset(zao);

    // Do calibration
    calMpu.CalibrateAccel(6);
    calMpu.CalibrateGyro(6);
    calMpu.CalibrateAccel(1);
    calMpu.CalibrateGyro(1);
    calMpu.CalibrateAccel(1);
    calMpu.CalibrateGyro(1);

    // Store config
    jsonObject["XGO"] = calMpu.getXGyroOffset();
    jsonObject["YGO"] = calMpu.getYGyroOffset();
    jsonObject["ZGO"] = calMpu.getZGyroOffset();
    jsonObject["XAO"] = calMpu.getXAccelOffset();
    jsonObject["YAO"] = calMpu.getYAccelOffset();
    jsonObject["ZAO"] = calMpu.getZAccelOffset();
}
