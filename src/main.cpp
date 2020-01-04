/*
  GY-521  NodeMCU
  MPU6050 devkit 1.0
  board   WemosD1       Description
  ======= ==========    ====================================================
  VCC     VU (5V USB)   Not available on all boards so use 3.3V if needed.
  GND     G             Ground
  SCL     D1 (GPIO05)   I2C clock
  SDA     D2 (GPIO04)   I2C data
  XDA     not connected
  XCL     not connected
  AD0     not connected
  INT     D8 (GPIO15)   Interrupt pin

*/

#include <FS.h>
#include <WiFiUdp.h>
#include <WiFiManager.h> // https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <stdio.h>

#include "udpStructures.h"

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps_V6_12.h"

#include "jscript.generated.h"
#include "protocolSelect_html.generated.h"

#define UPDATES_PER_SECOND            200
#define EFFECT_PERIOD_CALLBACK        (1000 / UPDATES_PER_SECOND)
#define TRACKER_CONFIG_FILENAME       "/tracker.json"
#define TRACKER_CONFIG_DOCUMENT_SIZE  512
#define PROTOCOL_FREEPIE              "freepie"
#define PROTOCOL_OPENTRACKUDP         "opentrackudp"
#define INTERRUPT_PIN                 15 // use pin 15 on ESP8266
#undef M_PI
#define M_PI 3.14159265358979323846f

#define JBONE_URI                      "/jscript.js"
#define TRACK_PEEK_URI                 "/peek"
#define STORE_CALIBRATION_URI          "/calibrate"

// routine to ensure we send timed message to the tracker
uint32_t effectPeriodStartMillis = 0;
uint32_t transitionCounter = 1;    // Will acount UPDATES_PER_SECOND per second

// Used in configurations
bool shouldSaveConfig = false;     // Indicate when configuration should be saved
uint32_t shouldRestart = 0;        // Indicate that a service requested an restart. Set to millies() of current time and it will restart 500ms later
bool shouldReloadAddress = false;  // Indicate when we should reload trackerIpAddress and trackerPort
bool hasTrackerLocation = false;   // Indicate when the tracker IP and port is known and good

// Used in mpu
volatile bool mpuInterrupt = false;         // indicates whether MPU interrupt pin has gone high
bool dmpReady = false;             // Indicate that the DMP is ready
uint16_t packetSize;                        // expected DMP packet size (default is 42 bytes)
uint8_t fifoBuffer[64];                     // FIFO storage buffer
uint16_t fifoCount;                         // count of all bytes currently in FIFO

struct LastMeasurements {
    float yaw;
    float pitch;
    float roll;
    float x;
    float y;
    float z;
} last_measurement;

// Used to send data over UDP
IPAddress trackerIpAddress;        // tracker IP address, should be reloaded on each config chance
uint16_t  trackerPort;             // tracker port number

// Configuration
StaticJsonDocument<TRACKER_CONFIG_DOCUMENT_SIZE> json;

// WiFI Manager
WiFiManager wm;

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high



WiFiManagerParameter custom_html("<p>Configure the location of OpenTrack or other receiver,</p>"); // only custom html
WiFiManagerParameter custom_track_server("track_server", "IP Address", nullptr, 40);
WiFiManagerParameter custom_track_port("track_port", "Port", nullptr, 6);

const char _customHtml_checkbox[] PROGMEM = "type=\"radio\"";

const char custoHeader[] = "<script src='/jscript.js'></script>";
WiFiManagerParameter custom_track_protocol("track_protocol", "", PROTOCOL_FREEPIE, 18, "type='hidden'");
WiFiManagerParameter custom_protocol_radio((char*)protocolSelect_html_nt);

void ICACHE_RAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}


void saveParamCallback() {
    Serial.println("[CALLBACK] saveParamCallback fired");
    json["tracker_server"] = custom_track_server.getValue();
    json["tracker_port"] = custom_track_port.getValue();
    json["tracker_protocol"] = custom_track_protocol.getValue();
    shouldSaveConfig = true;
    shouldReloadAddress = true;
}

/**
 * Load the tracker information into trackerPort and trackerIpAddress
 * return true if that condition is matched
 */
bool loadAddressInfoFromConfig() {
    if (json.containsKey("tracker_server")) {
        trackerPort = json["tracker_port"].as<int>();
        const char* tmp = json["tracker_server"].as<char*>();
        return trackerIpAddress.fromString(tmp);
    }

    return false;
}

void sendDatagramToTracker(const  uint8_t* buffer, size_t size) {
    Udp.beginPacket(trackerIpAddress, trackerPort);
    Udp.write(buffer, size);
    Udp.endPacket();
}

/**
 * Send tracker information using FreePie datastructure
 */
void sendFreePieDatagram() {
    FreePie freepie {
        0,
        2,
        last_measurement.yaw,
        last_measurement.pitch,
        last_measurement.roll,
        0, 0, 0,
        0, 0, 0, 0, 0, 0
    };
    sendDatagramToTracker((uint8_t*) &freepie, (size_t)sizeof(FreePie));
}

/**
 * Send a datagram using OpenTrack UDP data structure
 */
void sendOpentrackDatagram() {
    OpenTrack openTrack {
        0.0,
        0.0,
        0.0,
        last_measurement.yaw * 180 / M_PI,
        last_measurement.pitch * 180 / M_PI,
        last_measurement.roll * 180 / M_PI
    };
    sendDatagramToTracker((uint8_t*) &openTrack, (size_t)sizeof(OpenTrack));
}

/**
 * Send to tracker if hasTrackerLocation is set to true
 */
void sendTracker() {
    if (!hasTrackerLocation) {
        return;
    }

    if (strcmp(custom_track_protocol.getValue(), PROTOCOL_OPENTRACKUDP) == 0) {
        sendOpentrackDatagram();
    } else {
        sendFreePieDatagram();
    }
}

/**
 * Sendup the motion process
 */
void mpu_setup() {


    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    uint8_t devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {

        if (json.containsKey("mpu6050")) {
            JsonObject mpuConfig = json["mpu6050"].as<JsonObject>();
            mpu.setXGyroOffset(mpuConfig["XGyroOffset"].as<int>());
            mpu.setYGyroOffset(mpuConfig["YGyroOffset"].as<int>());
            mpu.setZGyroOffset(mpuConfig["ZGyroOffset"].as<int>());
            mpu.setXAccelOffset(mpuConfig["XAccelOffset"].as<int>());
            mpu.setYAccelOffset(mpuConfig["YAccelOffset"].as<int>());
            mpu.setZAccelOffset(mpuConfig["ZAccelOffset"].as<int>());
            Serial.println(F("Using saved calibration."));
        } else {
            // Set some defaults to might be okish....
            mpu.setXGyroOffset(51);
            mpu.setYGyroOffset(8);
            mpu.setZGyroOffset(21);
            mpu.setXAccelOffset(1150);
            mpu.setYAccelOffset(-50);
            mpu.setZAccelOffset(1060);
        }

        // Perform calibration
        //mpu.CalibrateAccel(6);
        //mpu.CalibrateGyro(6);

        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
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

void mpu_loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) {
        return;
    }

    // wait for MPU interrupt or extra packet(s) available
    if (!mpuInterrupt && fifoCount < packetSize) {
        return;
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    uint8_t mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) {
            fifoCount = mpu.getFIFOCount();
        }

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        float ypr[3];
        Quaternion q;           // [w, x, y, z]         quaternion container
        VectorFloat gravity;    // [x, y, z]            gravity vector

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        //float euler[3];         // [psi, theta, phi]    Euler angle container
        //mpu.dmpGetEuler(euler, &q);

        // Copy to last so we can use this in our webserver
        last_measurement.yaw = ypr[0] * 180 / M_PI;
        last_measurement.pitch = ypr[1] * 180 / M_PI;
        last_measurement.roll = ypr[2] * 180 / M_PI;
        last_measurement.x = 0.0f;
        last_measurement.y = 0.0f;
        last_measurement.z = 0.0f;

        sendTracker();

        /* Serial.print("ypr\t");
        Serial.print(ypr[0] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[1] * 180/M_PI);
        Serial.print("\t");
        Serial.print(ypr[2] * 180/M_PI);
        Serial.print("\t");
        Serial.print((size_t)sizeof(FreePie));
        Serial.println("");
        sendTracker(ypr); */
    }
}

/**
 * When server comes online setup other routes
 */
void serverOnlineCallback() {
    wm.server->on(JBONE_URI, []() {
        wm.server->sendHeader("Content-Encoding", "gzip");
        wm.server->setContentLength(jscript_js_gz_len);
        wm.server->send(200, "application/javascript", "");
        wm.server->sendContent_P((char*)jscript_js_gz, jscript_js_gz_len);
    });

    wm.server->on(TRACK_PEEK_URI, []() {
        char payloadBuffer[128];
        sprintf(payloadBuffer, F("{\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}"),
                last_measurement.yaw,
                last_measurement.pitch,
                last_measurement.roll,
                last_measurement.x,
                last_measurement.y,
                last_measurement.z
               );

        wm.server->setContentLength(strlen(payloadBuffer));
        wm.server->send(200, F("application/javascript"), payloadBuffer);
    });

    wm.server->on(STORE_CALIBRATION_URI, []() {
        if (dmpReady) {
            dmpReady = false;

            MPU6050 calMpu;
            calMpu.reset();
            delay(100);
            calMpu.initialize();

            // Set offset based on last measurement
            calMpu.setXGyroOffset(mpu.getXGyroOffset());
            calMpu.setXGyroOffset(mpu.getYGyroOffset());
            calMpu.setXGyroOffset(mpu.getZGyroOffset());
            calMpu.setXGyroOffset(mpu.getXAccelOffset());
            calMpu.setXGyroOffset(mpu.getYAccelOffset());
            calMpu.setXGyroOffset(mpu.getZAccelOffset());

            // Do calibration
            calMpu.CalibrateAccel(6);
            calMpu.CalibrateGyro(6);
            calMpu.CalibrateAccel(1);
            calMpu.CalibrateGyro(1);
            calMpu.CalibrateAccel(1);
            calMpu.CalibrateGyro(1);

            // Store config
            StaticJsonDocument<200> mpuConfig;
            mpuConfig["XGyroOffset"] = mpu.getYGyroOffset();
            mpuConfig["YGyroOffset"] = mpu.getYGyroOffset();
            mpuConfig["ZGyroOffset"] = mpu.getZGyroOffset();
            mpuConfig["XAccelOffset"] = mpu.getXAccelOffset();
            mpuConfig["YAccelOffset"] = mpu.getYAccelOffset();
            mpuConfig["ZAccelOffset"] = mpu.getZAccelOffset();
            json["mpu6050"] = mpuConfig;
            shouldSaveConfig = true;

            // Send result back
            wm.server->setContentLength(measureJson(mpuConfig));
            wm.server->send(200, F("application/javascript"), "");
            WiFiClient client = wm.server->client();
            serializeJson(mpuConfig, client);

            shouldRestart = millis();
        } else {
            // We would like to send a 503 but tje JS framework doesn´t give us the body
            wm.server->send(200, F("application/json"), F("{\"status\":\"error\", \"message\":\"MPU is not ready, please check hardware.\"}"));
        }
    });

}


/**
 * Load tracker configuration file and set the values in WifiManager Parameters
 */
bool loadTrackConfig() {
    if (SPIFFS.begin()) {
        Serial.println("mounted file system");

        if (SPIFFS.exists(TRACKER_CONFIG_FILENAME)) {
            //file exists, reading and loading
            Serial.println("reading config file");
            File configFile = SPIFFS.open(TRACKER_CONFIG_FILENAME, "r");

            if (configFile) {
                Serial.println("opened config file ");
                StaticJsonDocument<TRACKER_CONFIG_DOCUMENT_SIZE> jsonTmp;
                DeserializationError error = deserializeJson(jsonTmp, configFile);

                if (error) {
                    Serial.println(F("Failed to read file, using default configuration"));
                } else {
                    json = jsonTmp;
                    serializeJsonPretty(json, Serial);
                    return true;
                }
            }
        }
    }

    return false;
}


/**
 * Store custom oarameter configuration in SPIFFS
 */
bool saveConfigSPIFFS() {
    if (SPIFFS.begin()) {
        Serial.println("saving config");

        if (SPIFFS.begin()) {
            File configFile = SPIFFS.open(TRACKER_CONFIG_FILENAME, "w");

            if (!configFile) {
                Serial.println("failed to open config file for writing");
            } else {
                serializeJson(json, configFile);
                // serializeJsonPretty(json, Serial);
                configFile.close();
                return true;
            }
        }
    }

    return false;
}



void setup() {

    WiFi.mode(WIFI_STA);
    Serial.begin(115200);
    delay(250);

    Serial.println("\n Starting");
    WiFi.setSleepMode(WIFI_NONE_SLEEP); // disable sleep, can improve ap stability

    wm.setClass("invert");

    // custom_html.setValue("test",4);
    wm.setCustomHeadElement(custoHeader);

    //add all your parameters here
    wm.addParameter(&custom_html);
    wm.addParameter(&custom_track_server);
    wm.addParameter(&custom_track_port);
    wm.addParameter(&custom_track_protocol);
    wm.addParameter(&custom_protocol_radio);

    // callbacks
    wm.setSaveParamsCallback(saveParamCallback);

    std::vector<const char*> menu = {"wifi", "wifinoscan", "info", "param", "sep", "erase", "restart"};
    wm.setMenu(menu);

    // wm.setParamsPage(true); // move params to seperate page, not wifi, do not combine with setmenu!

    // set country
    wm.setCountry("US"); // setting wifi country seems to improve OSX soft ap connectivity, may help others as well

    wm.setWebServerCallback(serverOnlineCallback);

    // set configrportal timeout
    wm.setConfigPortalTimeout(120);

    if (!wm.autoConnect("WM_AutoConnectAP")) {
        Serial.println("failed to connect and hit timeout");
        wm.startConfigPortal();
    }

    //if you get here you have connected to the WiFi
    Serial.println("connected...yeey :)");
    wm.startWebPortal();

    shouldReloadAddress = loadTrackConfig();

    if (shouldReloadAddress == true) {
        // Configu page will get the info from here
        custom_track_server.setValue(json["tracker_server"], 40);
        custom_track_port.setValue(json["tracker_port"], 5);
        custom_track_protocol.setValue(json["tracker_protocol"], 12);
    } else {
        Serial.println(F("COnfiguration not loaded, is this first time started?"));
    }

    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
    effectPeriodStartMillis = millis();
}

#define NUMBER_OF_SLOTS 10
void loop() {

    const uint32_t currentMillis = millis();

    if (currentMillis - effectPeriodStartMillis >= EFFECT_PERIOD_CALLBACK) {
        effectPeriodStartMillis = currentMillis;
        transitionCounter++;
        int8_t slot = 0;

        if (dmpReady) {
            mpu_loop();
        }

        if (transitionCounter % NUMBER_OF_SLOTS == slot++) {
            wm.process();
        } else if (transitionCounter % NUMBER_OF_SLOTS == slot++) {
            if (!dmpReady && (transitionCounter % (UPDATES_PER_SECOND * 5) == slot - 1)) {
                mpu_setup();
            }
        } else if (transitionCounter % NUMBER_OF_SLOTS == slot++) {
            if (shouldSaveConfig) {
                shouldSaveConfig = false;
                saveConfigSPIFFS();
            } else if (shouldRestart != 0 && (currentMillis - shouldRestart >= 5000)) {
                shouldRestart = 0;
                ESP.restart();
            } else if (shouldReloadAddress) {
                shouldReloadAddress = false;
                hasTrackerLocation = loadAddressInfoFromConfig();
                Serial.println("has tracker location");
            }
        }
    }

}
