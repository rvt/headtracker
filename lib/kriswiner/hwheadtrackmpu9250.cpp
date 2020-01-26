
#include "hwheadtrackmpu9250.h"

extern "C" {
#include "quaternionFilters.h"
}

#define INTERRUPT_PIN                 15 // use pin 15 on ESP8266

#define SerialDebug false
#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

volatile bool HWHeadTrackmpu9250_mpuInterrupt = false;

void ICACHE_RAM_ATTR HWHeadTrackmpu9250_dmpDataReady() {
    HWHeadTrackmpu9250_mpuInterrupt = true;
}

HWHeadTrackmpu9250::HWHeadTrackmpu9250() {
    mpu = new MPU9250(INTERRUPT_PIN);
}

void HWHeadTrackmpu9250::setup(JsonObject json) {
    // initialize device only if we are not yet ready
    if (isReady() || calibrated) {
        return;
    }

    Serial.println(F("Initializing MPU9250..."));
    mpu->I2Cscan(); // should detect BME280 at 0x77, MPU9250 at 0x71

    /* Configure the MPU9250 */
    // Read the WHO_AM_I register, this is a good test of communication
    Serial.println("MPU9250 9-axis motion sensor...");
    uint8_t c = mpu->getMPU9250ID();

    if (c == 0x71) { // WHO_AM_I should always be 0x71 for MPU9250, 0x73 for MPU9255
        Serial.println("MPU9250 is online...");

        mpu->resetMPU9250(); // start by resetting MPU9250

        float   SelfTest[6];    // holds results of gyro and accelerometer self test
        mpu->SelfTest(SelfTest); // Start by performing self test and reporting values
        Serial.print("x-axis self test: acceleration trim within : ");
        Serial.print(SelfTest[0], 1);
        Serial.println("% of factory value");
        Serial.print("y-axis self test: acceleration trim within : ");
        Serial.print(SelfTest[1], 1);
        Serial.println("% of factory value");
        Serial.print("z-axis self test: acceleration trim within : ");
        Serial.print(SelfTest[2], 1);
        Serial.println("% of factory value");
        Serial.print("x-axis self test: gyration trim within : ");
        Serial.print(SelfTest[3], 1);
        Serial.println("% of factory value");
        Serial.print("y-axis self test: gyration trim within : ");
        Serial.print(SelfTest[4], 1);
        Serial.println("% of factory value");
        Serial.print("z-axis self test: gyration trim within : ");
        Serial.print(SelfTest[5], 1);
        Serial.println("% of factory value");
        delay(1000);

        // get sensor resolutions, only need to do this once
        aRes = mpu->getAres(Ascale);
        gRes = mpu->getGres(Gscale);
        mRes = mpu->getMres(Mscale);

        if (json.containsKey("AB0")) {
            accelBias[0] = json["AB0"].as<float>();
            accelBias[1] = json["AB1"].as<float>();
            accelBias[2] = json["AB2"].as<float>();

            gyroBias[0] = json["GB0"].as<float>();
            gyroBias[1] = json["GB1"].as<float>();
            gyroBias[2] = json["GB2"].as<float>();

            magBias[0] = json["MB0"].as<float>();
            magBias[1] = json["MB1"].as<float>();
            magBias[2] = json["MB2"].as<float>();

            magScale[0] = json["MS0"].as<float>();
            magScale[1] = json["MS1"].as<float>();
            magScale[2] = json["MS2"].as<float>();
            magScale = {1.01, 1.03, 0.96};

            Serial.println(F("Using saved calibration."));
        } else {
            accelBias = {0.00299, -0.00916, 0.00952};
            gyroBias = {0.96, -0.21, 0.12};
            magBias = {71.04, 122.43, -36.90};
            magScale = {1.01, 1.03, 0.96};
        }

        // Comment out if using pre-measured, pre-stored offset biases

        mpu->initMPU9250(Ascale, Gscale, sampleRate);

        Serial.println("MPU9250 initialized for active data mode...."); // Initialize device for active mode read of acclerometer, gyroscope, and temperature

        // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
        byte d = mpu->getAK8963CID();  // Read WHO_AM_I register for AK8963
        Serial.print("AK8963 ");
        Serial.print("I AM ");
        Serial.print(d, HEX);
        Serial.print(" I should be ");
        Serial.println(0x48, HEX);
        delay(1000);

        // Get magnetometer calibration from AK8963 ROM
        mpu->initAK8963(Mscale, Mmode, magCalibration.data());
        Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

        pinMode(INTERRUPT_PIN, INPUT);
        attachInterrupt(INTERRUPT_PIN, HWHeadTrackmpu9250_dmpDataReady, RISING);  // define interrupt for intPin output of MPU9250
        _dmpReady = true;
        Serial.print("DMP Ready!");
    } else {
        Serial.print("Could not connect to MPU9250: 0x");
        Serial.println(c, HEX);
    }
}

bool HWHeadTrackmpu9250::loop() {
    if (!isReady() || calibrated) {
        return false;
    }

    // If intPin goes high, either all data registers have new data
    if (HWHeadTrackmpu9250_mpuInterrupt == true) {  // On interrupt, read data
        HWHeadTrackmpu9250_mpuInterrupt = false;     // reset newData flag

        int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
        int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

        mpu->readMPU9250Data(MPU9250Data); // INT cleared on any read

        float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
        float pitch, yaw, roll;                   // absolute orientation
        float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components

        // Now we'll calculate the accleration value into actual g's
        ax = (float)MPU9250Data[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
        ay = (float)MPU9250Data[1] * aRes - accelBias[1];
        az = (float)MPU9250Data[2] * aRes - accelBias[2];

        temperature = ((float) MPU9250Data[3]) / 333.87f + 21.0f; // Gyro chip temperature in degrees Centigrade

        // Calculate the gyro value into actual degrees per second
        gx = (float)MPU9250Data[4] * gRes - gyroBias[0]; // get actual gyro value, this depends on scale being set
        gy = (float)MPU9250Data[5] * gRes - gyroBias[1];
        gz = (float)MPU9250Data[6] * gRes - gyroBias[2];

        //    if( mpu->checkNewMagData() == true) { // wait for magnetometer data ready bit to be set
        mpu->readMagData(magCount);  // Read the x/y/z adc values

        // Calculate the magnetometer values in milliGauss
        // Include factory calibration per data sheet and user environmental corrections
        mx = (float)magCount[0] * mRes * magCalibration[0] - magBias[0]; // get actual magnetometer value, this depends on scale being set
        my = (float)magCount[1] * mRes * magCalibration[1] - magBias[1];
        mz = (float)magCount[2] * mRes * magCalibration[2] - magBias[2];
        mx *= magScale[0];
        my *= magScale[1];
        mz *= magScale[2];
        //    }

        /*{
            // Debug perpose
            // Drop in a tool like https://app.rawgraphs.io
            // to check for calibration
            static int ccc = 0;

            if (ccc++ % 20 == 0) {
                Serial.print(gz, 2);
                Serial.print(",");
                Serial.print(mx, 2);
                Serial.print(",");
                Serial.print(my, 2);
                Serial.print(",");
                Serial.print(mz, 2);
                Serial.println();
            }
        }*/

        uint32_t now = micros();
        float deltat = ((float)(now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
        lastUpdate = now;

        /*
        MadgwickQuaternionUpdate(q, deltat, beta,
                                 ax, ay, az,
                                 gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f,
                                 my, mx, -mz);*/
        MahonyQuaternionUpdate(q, eInt, deltat,
                               Ki, Kp,
                               ax, ay, az,
                               gx * PI / 180.0f, gy * PI / 180.0f, gz * PI / 180.0f,
                               my,  mx, -mz);

        a12 =   2.0f * (q[1] * q[2] + q[0] * q[3]);
        a22 =   q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3];
        a31 =   2.0f * (q[0] * q[1] + q[2] * q[3]);
        a32 =   2.0f * (q[1] * q[3] - q[0] * q[2]);
        a33 =   q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3];
        pitch = -asinf(a32);
        roll  = atan2f(a31, a33);
        yaw   = atan2f(a12, a22);
        //pitch *= 180.0f / PI;
        //yaw   *= 180.0f / PI;
        //roll  *= 180.0f / PI;

        if (yaw < 0) {
            yaw   += 2 * PI;  // Ensure yaw stays between 0 and 360
        }

        //        lin_ax = ax + a31;
        //        lin_ay = ay + a32;
        //        lin_az = az - a33;

        // Copy to last so we can use this in our webserver
        pOrientation.yaw = yaw;
        pOrientation.pitch = pitch;
        pOrientation.roll = roll;
        pOrientation.x = 0.0f;
        pOrientation.y = 0.0f;
        pOrientation.z = 0.0f;
        return true;
    }

    return false;
}

void HWHeadTrackmpu9250::calibrate(JsonObject& jsonObject) {
    _dmpReady = false;
    calibrated = true;

    // Reset device and init MPU to start gyro and accell calibration
    mpu->resetMPU9250(); // start by resetting MPU9250
    mpu->initMPU9250(Ascale, Gscale, sampleRate);
    Serial.println("About to start calibrating gyro and accelerator, keep device flat and still!!!");
    Serial.println("Starting....");
    uint32_t start = micros();
    mpu->calibrateMPU9250(gyroBias.data(), accelBias.data()); // Calibrate gyro and accelerometers, load biases in bias registers
    Serial.print("Finnished in ");
    Serial.print((float)(micros() - start) / 1000000, 1);
    Serial.println(" seconds");

    // Reset again, then start calibration of magnetic compess
    // NoteÑ If we don´t reset and init the device, the magnet compass calbration will fail
    mpu->resetMPU9250(); // start by resetting MPU9250
    mpu->initMPU9250(Ascale, Gscale, sampleRate);
    delay(5000);

    // Start calibration fo compass
    Serial.println("About to start calibrating magnetic compass, Wave device in a figure eight until done!");
    mpu->initAK8963(Mscale, Mmode, magCalibration.data());
    Serial.println("Starting....");
    start = micros();
    mpu->magcalMPU9250(magBias.data(), magScale.data(), mRes);

    Serial.print("Finnished in ");
    Serial.print((float)(micros() - start) / 1000000, 1);
    Serial.println(" seconds");

    jsonObject["AB0"] = accelBias[0];
    jsonObject["AB1"] = accelBias[1];
    jsonObject["AB2"] = accelBias[2];

    jsonObject["GB0"] = gyroBias[0];
    jsonObject["GB1"] = gyroBias[1];
    jsonObject["GB2"] = gyroBias[2];

    jsonObject["MB0"] = magBias[0];
    jsonObject["MB1"] = magBias[1];
    jsonObject["MB2"] = magBias[2];

    jsonObject["MS0"] = magScale[0];
    jsonObject["MS1"] = magScale[1];
    jsonObject["MS2"] = magScale[2];
}
