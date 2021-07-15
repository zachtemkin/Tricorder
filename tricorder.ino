#include <ArduinoJson.h>

#include <Adafruit_GPS.h>
#include <WiFi.h>
#include <time.h>
#include <Wire.h>
#include <FirebaseESP32.h>
#include <FirebaseFS.h>
#include <ESP32Servo.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <Adafruit_VEML6070.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <WiFiManager.h>
#include <FS.h>
#include <SPIFFS.h>
#include <EEPROM.h>

#define EEPROM_SIZE 1

// WiFi Credentials
#define WIFI_SSID "batcave"
#define WIFI_PASSWORD "by-permit-only"

// Firebas credentials
#define API_KEY "AIzaSyDXPfLlPvMkDf8hdbHE7qwnh_Uq6FcJMFo"
#define DATABASE_URL "esp32-cb077-default-rtdb.firebaseio.com"

// Firebase objects
FirebaseData fbdo;
FirebaseJson parentNode_json;
FirebaseConfig config;
FirebaseAuth auth;

// Store device authentication status
bool isAuthenticated = false;

// Firebase database path
String databasePath = "";

// Firebase Unique Identifier
String fuid = "";

// NTP server to request epoch time
const char *ntpServer = "pool.ntp.org";

// Sensors
Adafruit_SGP30 sgp;
Adafruit_BME280 bme;
Adafruit_VEML6070 uv = Adafruit_VEML6070();

// Initialize sensor reading vars
int tvoc = 0;
int eco2 = 0;
int humidityReading = 0;
int temperatureReading = 0;
int pressureReading = 0;
int uvReading = 0;

//GPS
#define GPSSerial Serial1
#define GPSECHO false
Adafruit_GPS GPS(&GPSSerial);

uint32_t timer = millis();

float gpsLatReading = 0;
float gpsLonReading = 0;
float gpsSpeedReading = 0;
float gpsAngleReading = 0;
float gpsAltReading = 0;
int gpsSatsReading = 0;
int gpsFixReading = 0;
String gpsHour;
String gpsMin;
String gpsSec;
String gpsTime;
String gpsDay;
String gpsMonth;
String gpsYear;
String gpsDate;

// Stores the elapsed time from device start up
unsigned long dataReadTimer = 0;
unsigned long wifiCheckTimer = 0;
unsigned long wifiConnectTimer = 0;

// The frequency of sensor updates to firebase, set to 10seconds
unsigned long data_update_interval = 10000;
unsigned long wifi_connect_interval = 30000;
unsigned long wifi_reConnect_interval = 120000;

//upload state
bool dataHasBeenUploaded = false;

int fileCount = 0;

void setup()
{   
    Serial.begin(115200);
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    EEPROM.begin(EEPROM_SIZE);
    fileCount = EEPROM.read(0);

    listDir(SPIFFS, "/", 1);

    //connect to WiFi
    if (!wifiInit(WIFI_SSID, WIFI_PASSWORD)) {
        Serial.println("Saving offline");
    } 

    //WiFi Manager
    // WiFiManager wifiManager;
    // wifiManager.autoConnect(WIFI_SSID, WIFI_PASSWORD);

    //get time
    configTime((-5 * 3600), 3600, ntpServer);

    // set up sensors
    setupBME280();
    setupSGP30();
    setupVEML6070();
    setupGPS();

    if (WiFi.status() == WL_CONNECTED)
    {
        firebaseInit();
        uploadSensorData();
    }
}

void loop() {
    updateGPSreadings();
    
    if (WiFi.status() == WL_CONNECTED)
    {
        if (!dataHasBeenUploaded) {
            firebaseInit();
            uploadSensorData();
            dataHasBeenUploaded = true;
        }
    } else {
        saveDataOffline();
        dataHasBeenUploaded = false;

        if (millis() - wifiCheckTimer > wifi_reConnect_interval) {
            wifiCheckTimer = millis();

            wifiInit(WIFI_SSID, WIFI_PASSWORD);
        }
    }
}

bool wifiInit(const char * ssid, const char * password) {
    WiFi.begin(ssid, password);
    Serial.print("connecting to wifi");

    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);

        if (millis() - wifiConnectTimer > wifi_connect_interval) {
            wifiConnectTimer = millis();
            Serial.println("could not connect to wifi");
            return false;
        }
    }

    Serial.println();
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();
    return true;
}

void firebaseInit() {
    config.api_key = API_KEY;
    config.database_url = DATABASE_URL;
    Firebase.reconnectWiFi(true);

    Serial.println("------------------------------------");
    Serial.println("Sign up new user...");

    if (Firebase.signUp(&config, &auth, "", "")) {
        Serial.println("Success");
        isAuthenticated = true;
        databasePath = "/Tricorder";
        fuid = auth.token.uid.c_str();
    } else {
        Serial.printf("Failed, %s\n", config.signer.signupError.message.c_str());
        isAuthenticated = false;
    }

    config.token_status_callback = tokenStatusCallback;
    Firebase.begin(&config, &auth);
}

void setupSGP30()
{
    if (!sgp.begin())
    {
        Serial.println("Could not find a valid SGP30 sensor, check wiring!");
    }
}

void setupVEML6070()
{
    uv.begin(VEML6070_1_T);
}

void setupBME280()
{
    bool status;
    status = bme.begin();
    if (!status)
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
    }
}

void setupGPS()
{
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    GPS.sendCommand(PGCMD_ANTENNA);
}

void saveDataOffline() {
    if (millis() - dataReadTimer > data_update_interval) {
        dataReadTimer = millis();
                
        String logEntryFilePath = "/logEntryFile_";                                                                                         
        logEntryFilePath.concat(fileCount);
        logEntryFilePath.concat(".txt");
        fileCount ++;
        EEPROM.write(0, fileCount);
        EEPROM.commit();
        File LogEntryFile = SPIFFS.open(logEntryFilePath, FILE_WRITE);

        StaticJsonDocument<384> logEntryJson;
        
        logEntryJson["tvoc"] = tvoc;
        logEntryJson["eco2"] = eco2;
        logEntryJson["uv"] = uvReading;
        logEntryJson["temperature"] = temperatureReading;
        logEntryJson["humidity"] = humidityReading;
        logEntryJson["pressure"] = pressureReading;
        logEntryJson["latitude"] = gpsLatReading;
        logEntryJson["longitude"] = gpsLonReading;
        logEntryJson["speed"] = gpsSpeedReading;
        logEntryJson["angle"] = gpsAngleReading;
        logEntryJson["altitude"] = gpsAltReading;
        logEntryJson["satelites"] = gpsSatsReading;
        logEntryJson["gpsTime"] = gpsTime;
        logEntryJson["gpsDate"] = gpsDate;
        
        serializeJson(logEntryJson, LogEntryFile);
        LogEntryFile.close();
    }
}

void uploadSensorData() {
    if (isAuthenticated && Firebase.ready()) {
        String parent_node_path = databasePath + "/";

        File root = SPIFFS.open("/");
        File file = root.openNextFile();

        StaticJsonDocument<384> logEntryFile;

        while(file) {
            String logEntryString;
            deserializeJson(logEntryFile, file);
            serializeJson(logEntryFile, logEntryString);

            parentNode_json.setJsonData(logEntryString);

            if (Firebase.pushJSON(fbdo, parent_node_path, parentNode_json))
            {
                Serial.println("updloaded json");
                SPIFFS.remove(file.name());
            }
            else
            {
                Serial.println("FAILED");
                Serial.println("REASON: " + fbdo.errorReason());
                Serial.println("------------------------------------");
                Serial.println();
            }
            file = root.openNextFile();

            EEPROM.write(0, 0);
            EEPROM.commit();
        }
    }
}

void updateSesorReadings() {
    tvoc = sgp.TVOC;
    eco2 = sgp.eCO2;
    uvReading = uv.readUV();
    temperatureReading = bme.readTemperature() * 1.8 + 32;
    humidityReading = bme.readHumidity();
    pressureReading = bme.readPressure() / 100.0F;
}

void updateGPSreadings() {
    char c = GPS.read();
    if (GPSECHO) {
        if (c) {
            Serial.print(c);
        }
    }

    if (GPS.newNMEAreceived())
    {
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            return;                     // we can fail to parse a sentence in which case we should just wait for another
    }

    if (millis() - timer > 2000)
    {
        timer = millis(); // reset the timer

        if (GPS.hour < 10) {
            gpsHour = '0' + String(GPS.hour);
        } else {
            gpsHour = String(GPS.hour);
        }
        
        if (GPS.minute < 10) {
            gpsMin = '0' + String(GPS.minute);
        } else {
            gpsMin = String(GPS.minute);
        }

        if (GPS.seconds < 10) {
            gpsSec = '0' + String(GPS.seconds);
        } else {
            gpsSec = String(GPS.seconds);
        }

        gpsTime = gpsHour + ':' + gpsMin + ':' + gpsSec;

        gpsDay = String(GPS.day);
        gpsMonth = String(GPS.month);
        gpsYear = '20' + String(GPS.year);
        gpsDate = gpsMonth + '/' + gpsDay + '/' + gpsYear;

        // Serial.print("fix: "); Serial.print((int)GPS.fix);
        // Serial.print("\t fix quality: "); Serial.println((int)GPS.fixquality);
        // Serial.print("Satellites: ");
        // Serial.println((int)GPS.satellites);
        
        if (GPS.fix)
        {
            // Serial.print("Location: ");
            // Serial.print(GPS.latitude, 4);
            // Serial.print(GPS.lat);
            gpsLatReading = GPS.latitude;
            // Serial.print(", ");
            // Serial.print(GPS.longitude, 4);
            // Serial.println(GPS.lon);
            gpsLonReading = GPS.longitude;
            // Serial.print("Speed (knots): ");
            // Serial.println(GPS.speed);
            gpsSpeedReading = GPS.speed;
            // Serial.print("Angle: ");
            // Serial.println(GPS.angle);
            gpsAngleReading = GPS.angle;
            // Serial.print("Altitude: ");
            // Serial.println(GPS.altitude);
            gpsAltReading = GPS.altitude;
            gpsSatsReading = (int)GPS.satellites;
        }
        updateSesorReadings();
    }
}

void listDir(fs::FS &fs, const char *dirname, uint8_t levels)
{
    Serial.printf("Listing directory: %s\r\n", dirname);

    File root = fs.open(dirname);
    if (!root)
    {
        Serial.println("- failed to open directory");
        return;
    }
    if (!root.isDirectory())
    {
        Serial.println(" - not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file)
    {
        if (file.isDirectory())
        {
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if (levels)
            {
                listDir(fs, file.name(), levels - 1);
            }
        }
        else
        {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}