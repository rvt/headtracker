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

#include "jscript.generated.h"
#include "protocolSelect_html.generated.h"
#include "I2Cdev.h"
#include <hwheadtrack.h>
#include <hwheadtrackmpu6050.h>
#include <hwheadtrackmpu9250.h>


#define UPDATES_PER_SECOND            200
#define EFFECT_PERIOD_CALLBACK        (1000 / UPDATES_PER_SECOND)
#define TRACKER_CONFIG_FILENAME       "/tracker.json"
#define TRACKER_CONFIG_DOCUMENT_SIZE  512
#define PROTOCOL_FREEPIE              "freepie"
#define PROTOCOL_OPENTRACKUDP         "opentrackudp"
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

HWHeadTrack_Orientation last_measurement;
std::unique_ptr<HWHeadTrack> hwTrack(nullptr);

// Used to send data over UDP
IPAddress trackerIpAddress;        // tracker IP address, should be reloaded on each config chance
uint16_t  trackerPort;             // tracker port number

// Configuration
StaticJsonDocument<TRACKER_CONFIG_DOCUMENT_SIZE> json;

// WiFI Manager
WiFiManager wm;

// A UDP instance to let us send and receive packets over UDP
WiFiUDP Udp;


WiFiManagerParameter custom_html("<p>Configure the location of OpenTrack or other receiver,</p>"); // only custom html
WiFiManagerParameter custom_track_server("track_server", "IP Address", nullptr, 40);
WiFiManagerParameter custom_track_port("track_port", "Port", nullptr, 6);

const char _customHtml_checkbox[] PROGMEM = "type=\"radio\"";

const char custoHeader[] = "<script src='/jscript.js'></script>";
WiFiManagerParameter custom_track_protocol("track_protocol", "", PROTOCOL_FREEPIE, 18, "type='hidden'");
WiFiManagerParameter custom_protocol_radio((char*)protocolSelect_html_nt);



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
        sprintf(payloadBuffer, "{\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f,\"x\":%.2f,\"y\":%.2f,\"z\":%.2f}",
                last_measurement.yaw * 180 / M_PI,
                last_measurement.pitch * 180 / M_PI,
                last_measurement.roll * 180 / M_PI,
                last_measurement.x,
                last_measurement.y,
                last_measurement.z
               );

        wm.server->setContentLength(strlen(payloadBuffer));
        wm.server->send(200, F("application/javascript"), payloadBuffer);
    });

    wm.server->on(STORE_CALIBRATION_URI, []() {
        if (hwTrack->isReady()) {

            if (!json.containsKey(hwTrack->name())) {
                json.createNestedObject(hwTrack->name());
            }

            JsonObject config = json[hwTrack->name()].as<JsonObject>();
            hwTrack->calibrate(config);
            serializeJsonPretty(config, Serial);

            shouldSaveConfig = true;

            // Send result back
            wm.server->setContentLength(measureJson(config));
            wm.server->send(200, F("application/javascript"), "");
            WiFiClient client = wm.server->client();
            serializeJson(config, client);

            // Request restart
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
    //hwTrack.reset(new HWHeadTrackmpu6050());
    hwTrack.reset(new HWHeadTrackmpu9250());

    effectPeriodStartMillis = millis();
}

#define NUMBER_OF_SLOTS 10
void loop() {

    const uint32_t currentMillis = millis();

    if (currentMillis - effectPeriodStartMillis >= EFFECT_PERIOD_CALLBACK) {
        effectPeriodStartMillis = currentMillis;
        transitionCounter++;
        uint8_t slot = 0;

        if (hwTrack->loop()) {
            last_measurement = hwTrack->getOrientation();
        }
            
        sendTracker();

        if (transitionCounter % NUMBER_OF_SLOTS == slot++) {
            wm.process();
        } else if (transitionCounter % NUMBER_OF_SLOTS == slot++) {
            if (transitionCounter % UPDATES_PER_SECOND == slot - 1) {
                hwTrack->setup(json[hwTrack->name()]);
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
