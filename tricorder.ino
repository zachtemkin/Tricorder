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

// Define the WiFi credentials 
#define WIFI_SSID "batcave"
#define WIFI_PASSWORD "by-permit-only"
bool wifiIsConnected = false;

// Define Firebas credentials
#define API_KEY "AIzaSyDXPfLlPvMkDf8hdbHE7qwnh_Uq6FcJMFo"
#define DATABASE_URL "esp32-cb077-default-rtdb.firebaseio.com"

// Firebase objects
FirebaseData fbdo;
FirebaseJson parentNode_json;
FirebaseJson metaData_json;
FirebaseJson logEntry_json;
FirebaseJson tvoc_json;
FirebaseJson eco2_json;
FirebaseJson uv_json;
FirebaseJson pressure_json;
FirebaseJson temperature_json;
FirebaseJson humidity_json;
FirebaseJson gps_json;
FirebaseJson gpsLat_json;
FirebaseJson gpsLon_json;
FirebaseJson gpsSpeed_json;
FirebaseJson gpsAngle_json;
FirebaseJson gpsAlt_json;
FirebaseJson gpsSats_json;
FirebaseJson gpsFix_json;
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

// Variable to save current epoch time
unsigned long t;

// Sensors
Adafruit_SGP30 sgp;
Adafruit_BME280 bme;
Adafruit_VEML6070 uv = Adafruit_VEML6070();

// sgp30 data
float tvoc = 0;
float eco2 = 0;

// BME280 Data
int humidityReading = 0;
int temperatureReading = 0;
int pressureReading = 0;

// VEML6070 Data
int uvReading = 0;

//GPS
// what's the name of the hardware serial port?
#define GPSSerial Serial1

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

float gpsLatReading = 0;
float gpsLonReading = 0;
float gpsSpeedReading = 0;
float gpsAngleReading = 0;
float gpsAltReading = 0;
int gpsSatsReading = 0;
int gpsFixReading = 0;

// Stores the elapsed time from device start up
unsigned long elapsedMillis = 0;

// The frequency of sensor updates to firebase, set to 10seconds
unsigned long update_interval = 10000;

int count = 0;

void setup()
{   
    Serial.begin(115200);
    if (!SPIFFS.begin(true))
    {
        Serial.println("An Error has occurred while mounting SPIFFS");
        return;
    }

    EEPROM.begin(EEPROM_SIZE);
    count = EEPROM.read(0);

    listDir(SPIFFS, "/", 1);

    //connect to WiFi
    if (!wifiInit(WIFI_SSID, WIFI_PASSWORD)) {
        Serial.println("can't connect to WiFi. Saving offline");
    } else {
        wifiIsConnected = true;
    }

    //WiFi Manager
    // WiFiManager wifiManager;
    // wifiManager.autoConnect(WIFI_SSID, WIFI_PASSWORD);

    //get time
    configTime((-5 * 3600), 3600, ntpServer);

    //Initialise firebase configuration and signup anonymously
    firebaseInit();
    setSensorMetaData();
    // uploadSensorData();

    // set up sensors
    setupBME280();
    setupSGP30();
    setupVEML6070();
    setupGPS();

    if (wifiIsConnected) {
        uploadSensorData();
    }
}

void loop() {
    if(!wifiIsConnected) {
        saveDataOffline();
    }
}

bool wifiInit(const char * ssid, const char * password) {
    WiFi.begin(ssid, password);
    Serial.print("connecting to wifi");
    while (WiFi.status() != WL_CONNECTED)
    {
        Serial.print(".");
        delay(300);
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

String getFormattedTime()
{
    char formatted_time[32];
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        return "Failed to obtain time";
    }
    strftime(formatted_time, 32, "%A, %B %d %Y %H:%M:%S", &timeinfo);
    return formatted_time;
}

void setSensorMetaData() {
    //time
    metaData_json.add("configTime", getFormattedTime());
    //temp
    metaData_json.add("temperatureDeviceId", "BME280");
    metaData_json.add("tempUnits", "F");
    //humidity
    metaData_json.add("humidityDeviceId", "BME280");
    metaData_json.add("humidityUnits", "%");
    //pressure
    metaData_json.add("pressureDeviceId", "BME280");
    metaData_json.add("units", "hPa");
    //air quality
    metaData_json.add("airQualityDeviceId", "SGP30_01");
    metaData_json.add("tvocUnits", "ppb");
    metaData_json.add("eCO2Units", "ppm");
    //uv
    metaData_json.add("uvDeviceId", "VEML6070");
    metaData_json.add("uvUnits", "uv index");

    if (Firebase.setJSON(fbdo, databasePath + "/metaData", metaData_json))
    {
        Serial.println("updated config json");
    }
    else
    {
        Serial.println("FAILED");
        Serial.println("REASON: " + fbdo.errorReason());
        Serial.println("------------------------------------");
        Serial.println();
    }
    return;
}

void saveDataOffline() {
    if (millis() - elapsedMillis > update_interval) {
        elapsedMillis = millis();
        
        Serial.println("saving data offline");
        updateSesorReadings();
        
        String logEntryFilePath = "/logEntryFile_";
        logEntryFilePath.concat(count);
        logEntryFilePath.concat(".txt");
        count ++;
        EEPROM.write(0, count);
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
            
            // Serial.println("**************************");
            // Serial.println("");
            // Serial.println(logEntryString);
            // Serial.println("");
            // Serial.println("**************************");

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
        }
    }
}

void setupSGP30() {
    if (!sgp.begin())
    {
        Serial.println("Could not find a valid SGP30 sensor, check wiring!");
        while (1);
    }
}

void setupVEML6070() {
    uv.begin(VEML6070_1_T);
}

void setupBME280()
{
    bool status;
    status = bme.begin();
    if (!status)
    {
        Serial.println("Could not find a valid BME280 sensor, check wiring!");
        while (1);
    }
}

void setupGPS() {
    GPS.begin(9600);
    GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); // 1 Hz update rate
    GPS.sendCommand(PGCMD_ANTENNA);
}

void updateSesorReadings() {
    tvoc = sgp.TVOC;
    eco2 = sgp.eCO2;
    uvReading = uv.readUV();
    temperatureReading = bme.readTemperature() * 1.8 + 32;
    humidityReading = bme.readHumidity();
    pressureReading = bme.readPressure() / 100.0F;
    
    //Read GPS
    if (GPS.newNMEAreceived())
    {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences!
        // so be very wary if using OUTPUT_ALLDATA and trying to print out data
        // Serial.print(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
        if (!GPS.parse(GPS.lastNMEA())) // this also sets the newNMEAreceived() flag to false
            return;                     // we can fail to parse a sentence in which case we should just wait for another
    }
    /*
    Serial.print("\nTime: ");
    if (GPS.hour < 10)
    {
        Serial.print('0');
    }
    Serial.print(GPS.hour, DEC);
    Serial.print(':');
    if (GPS.minute < 10)
    {
        Serial.print('0');
    }
    Serial.print(GPS.minute, DEC);
    Serial.print(':');
    if (GPS.seconds < 10)
    {
        Serial.print('0');
    }
    Serial.print(GPS.seconds, DEC);
    Serial.print('.');
    if (GPS.milliseconds < 10)
    {
        Serial.print("00");
    }
    else if (GPS.milliseconds > 9 && GPS.milliseconds < 100)
    {
        Serial.print("0");
    }
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.month, DEC);
    Serial.print("/");
    Serial.print(GPS.day, DEC);
    Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: ");
    Serial.print((int)GPS.fix);
    */
    gpsFixReading = (int)GPS.fix;
    // Serial.print(" quality: ");
    // Serial.println((int)GPS.fixquality);
    
    if (GPS.fix)
    {
        // Serial.print("Location: ");
        // Serial.print(GPS.latitude, 4);
        // Serial.print(GPS.lat);
        gpsLatReading = GPS.lat;
        
        // Serial.print(", ");
        // Serial.print(GPS.longitude, 4);
        // Serial.println(GPS.lon);
        gpsLonReading = GPS.lon;
        
        // Serial.print("Speed (knots): ");
        // Serial.println(GPS.speed);
        gpsSpeedReading = GPS.speed;
        
        // Serial.print("Angle: ");
        // Serial.println(GPS.angle);
        gpsAngleReading = GPS.angle;

        // Serial.print("Altitude: ");
        // Serial.println(GPS.altitude);
        gpsAltReading = GPS.altitude;

        // Serial.print("Satellites: ");
        // Serial.println((int)GPS.satellites);
        gpsSatsReading = (int)GPS.satellites;
    }


    // logEntry_json.add("dateTime", getFormattedTime());
    // logEntry_json.add("tvoc", tvoc);
    // logEntry_json.add("eco2", eco2);
    // logEntry_json.add("uv", uvReading);
    // logEntry_json.add("temperature", temperatureReading);
    // logEntry_json.add("humidity", humidityReading);
    // logEntry_json.add("pressure", pressureReading);
    // logEntry_json.add("lat", gpsLatReading);
    // logEntry_json.add("lon", gpsLonReading);
    // logEntry_json.add("speed", gpsSpeedReading);
    // logEntry_json.add("angle", gpsAngleReading);
    // logEntry_json.add("altitude", gpsAltReading);
    // logEntry_json.add("satelites", gpsSatsReading);
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
