
#include "hwheadtrackmpu9250.h"

extern "C" {
#include "MadgwickFilter.h"
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
    // initialize device
    if (isReady()) {
        return;
    }

    Serial.println(F("Initializing MPU9250..."));
    pinMode(INTERRUPT_PIN, INPUT);


    mpu->I2Cscan(); // should detect BME280 at 0x77, MPU9250 at 0x71

    /* Configure the MPU9250 */
    // Read the WHO_AM_I register, this is a good test of communication
    Serial.println("MPU9250 9-axis motion sensor...");
    uint8_t c = mpu->getMPU9250ID();
    Serial.print("MPU9250 ");
    Serial.print("I AM ");
    Serial.print(c, HEX);
    Serial.print(" I should be ");
    Serial.println(0x71, HEX);
    delay(1000);

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

        // Comment out if using pre-measured, pre-stored offset biases
        mpu->calibrateMPU9250(gyroBias, accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
        Serial.println("accel biases (mg)");
        Serial.println(1000.*accelBias[0]);
        Serial.println(1000.*accelBias[1]);
        Serial.println(1000.*accelBias[2]);
        Serial.println("gyro biases (dps)");
        Serial.println(gyroBias[0]);
        Serial.println(gyroBias[1]);
        Serial.println(gyroBias[2]);
        delay(1000);

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
        mpu->initAK8963Slave(Mscale, Mmode, magCalibration);
        Serial.println("AK8963 initialized for active data mode...."); // Initialize device for active mode read of magnetometer

        // Comment out if using pre-measured, pre-stored offset biases
        //  mpu->magcalMPU9250(magBias, magScale, mRes);
        Serial.println("AK8963 mag biases (mG)");
        Serial.println(magBias[0]);
        Serial.println(magBias[1]);
        Serial.println(magBias[2]);
        Serial.println("AK8963 mag scale (mG)");
        Serial.println(magScale[0]);
        Serial.println(magScale[1]);
        Serial.println(magScale[2]);
        delay(2000); // add delay to see results before serial spew of data

        if (SerialDebug) {
            Serial.println("Calibration values: ");
            Serial.print("X-Axis sensitivity adjustment value ");
            Serial.println(magCalibration[0], 2);
            Serial.print("Y-Axis sensitivity adjustment value ");
            Serial.println(magCalibration[1], 2);
            Serial.print("Z-Axis sensitivity adjustment value ");
            Serial.println(magCalibration[2], 2);

        }

        attachInterrupt(INTERRUPT_PIN, HWHeadTrackmpu9250_dmpDataReady, RISING);  // define interrupt for intPin output of MPU9250
        _dmpReady = true;
    } else {
        Serial.print("Could not connect to MPU9250: 0x");
        Serial.println(c, HEX);

    }
}

bool HWHeadTrackmpu9250::loop() {
    // If intPin goes high, either all data registers have new data
    if (HWHeadTrackmpu9250_mpuInterrupt == true) {  // On interrupt, read data
        HWHeadTrackmpu9250_mpuInterrupt = false;     // reset newData flag

        int16_t MPU9250Data[7]; // used to read all 14 bytes at once from the MPU9250 accel/gyro
        int16_t magCount[3];    // Stores the 16-bit signed magnetometer sensor output

        mpu->readMPU9250Data(MPU9250Data); // INT cleared on any read

        float ax, ay, az, gx, gy, gz, mx, my, mz; // variables to hold latest sensor data values
        float lin_ax, lin_ay, lin_az;             // linear acceleration (acceleration with gravity component subtracted)
        float pitch, yaw, roll;                   // absolute orientation
        float a12, a22, a31, a32, a33;            // rotation matrix coefficients for Euler angles and gravity components

        // Now we'll calculate the accleration value into actual g's
        ax = (float)MPU9250Data[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
        ay = (float)MPU9250Data[1] * aRes - accelBias[1];
        az = (float)MPU9250Data[2] * aRes - accelBias[2];

        // Calculate the gyro value into actual degrees per second
        gx = (float)MPU9250Data[4] * gRes; // get actual gyro value, this depends on scale being set
        gy = (float)MPU9250Data[5] * gRes;
        gz = (float)MPU9250Data[6] * gRes;

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

        float sum = 0.0f;          // integration interval for both filter schemes

        for (uint8_t i = 0; i < 10; i++) { // iterate a fixed number of times per data read cycle
            uint32_t now = micros();
            float deltat = ((now - lastUpdate) / 1000000.0f); // set integration time by time elapsed since last filter update
            lastUpdate = now;

            sum += deltat; // sum for averaging filter update rate

            MadgwickQuaternionUpdate(q, beta, deltat,
                                     -ax, +ay, +az,
                                     gx * PI / 180.0f, -gy * PI / 180.0f, -gz * PI / 180.0f,
                                     my,  -mx, mz);
            /*             MahonyQuaternionUpdate(q, eInt, deltat,
                                               Ki, Kp,
                                               -ax, +ay, +az,
                                               gx * PI / 180.0f, -gy * PI / 180.0f, -gz * PI / 180.0f,
                                               my,  -mx, mz); */
        }


        /* end of MPU9250 interrupt handling */


        if (SerialDebug) {
            Serial.print("ax = ");
            Serial.print((int)1000 * ax);
            Serial.print(" ay = ");
            Serial.print((int)1000 * ay);
            Serial.print(" az = ");
            Serial.print((int)1000 * az);
            Serial.println(" mg");
            Serial.print("gx = ");
            Serial.print(gx, 2);
            Serial.print(" gy = ");
            Serial.print(gy, 2);
            Serial.print(" gz = ");
            Serial.print(gz, 2);
            Serial.println(" deg/s");
            Serial.print("mx = ");
            Serial.print((int)mx);
            Serial.print(" my = ");
            Serial.print((int)my);
            Serial.print(" mz = ");
            Serial.print((int)mz);
            Serial.println(" mG");

            Serial.print("q0 = ");
            Serial.print(q[0]);
            Serial.print(" qx = ");
            Serial.print(q[1]);
            Serial.print(" qy = ");
            Serial.print(q[2]);
            Serial.print(" qz = ");
            Serial.println(q[3]);

            temperature = ((float) MPU9250Data[3]) / 333.87f + 21.0f; // Gyro chip temperature in degrees Centigrade
            // Print temperature in degrees Centigrade
            Serial.print("Gyro temperature is ");
            Serial.print(temperature, 1);
            Serial.println(" degrees C"); // Print T values to tenths of s degree C
        }

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
        //yaw   += 13.8f; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04

        if (yaw < 0) {
            yaw   += 2 * PI;  // Ensure yaw stays between 0 and 360
        }

        //roll  *= 180.0f / PI;
        lin_ax = ax + a31;
        lin_ay = ay + a32;
        lin_az = az - a33;

        // Copy to last so we can use this in our webserver
        pOrientation.yaw = yaw;
        pOrientation.pitch = pitch;
        pOrientation.roll = roll;
        pOrientation.x = 0.0f;
        pOrientation.y = 0.0f;
        pOrientation.z = 0.0f;

        if (SerialDebug) {
            Serial.print("Yaw, Pitch, Roll: ");
            Serial.print(yaw, 2);
            Serial.print(", ");
            Serial.print(pitch, 2);
            Serial.print(", ");
            Serial.println(roll, 2);

            Serial.print("Grav_x, Grav_y, Grav_z: ");
            Serial.print(-a31 * 1000.0f, 2);
            Serial.print(", ");
            Serial.print(-a32 * 1000.0f, 2);
            Serial.print(", ");
            Serial.print(a33 * 1000.0f, 2);
            Serial.println(" mg");
            Serial.print("Lin_ax, Lin_ay, Lin_az: ");
            Serial.print(lin_ax * 1000.0f, 2);
            Serial.print(", ");
            Serial.print(lin_ay * 1000.0f, 2);
            Serial.print(", ");
            Serial.print(lin_az * 1000.0f, 2);
            Serial.println(" mg");

        }

        return true;
    }

    return false;

}

void HWHeadTrackmpu9250::calibrate(JsonObject& jsonObject) {

}

