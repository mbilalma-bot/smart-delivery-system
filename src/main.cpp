#define TINY_GSM_MODEM_SIM7600  // Modem is SIM7600
#define TINY_GSM_RX_BUFFER 1024  // Set RX buffer to 1Kb
#include <Arduino.h>
#include <TinyGsmClient.h>
#include <SSLClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RTClib.h>
#include <Preferences.h>

Preferences preferences; // Global declaration

#include "ca_cert.h"

// Define ESP32-C3 specific configurations
#define ESP32C3

// OLED Display Configuration
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing  0x3C atau 0x3D)

// Buzzer Configuration
#define BUZZER_PIN 3              // Pin GPIO untuk buzzer

// Button Configuration
#define START_BUTTON_PIN 6
#define STOP_BUTTON_PIN 7

// Variables for manual debouncing
long lastDebounceTimeStart = 0;
long lastDebounceTimeStop = 0;
long debounceDelay = 50; // Debounce interval in milliseconds

// Variables to store the current and previous button states
int buttonStateStart;
int lastButtonStateStart = HIGH; // Assuming pull-up, so HIGH when not pressed
int buttonStateStop;
int lastButtonStateStop = HIGH; // Assuming pull-up, so HIGH when not pressed

// Timing Variables
unsigned long startTime = 0;
unsigned long stopTime = 0;
bool timingInProgress = false;

// Estimated Time Variables
unsigned long estimatedDurationMs = 3600000; // Default to 1 hour (3600 * 1000 ms)
unsigned long lastTimeStatusSent = 0;
long estimatedDays = 0;
long estimatedHours = 0;
long estimatedMinutes = 0;
long estimatedSeconds = 0;

// Buzzer Variables
float tempSetpoint = 25.0;      // Setpoint suhu dalam Celsius
bool buzzerState = false;       // Status buzzer (on/off)Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
unsigned long lastBuzzerToggle = 0; // Waktu terakhir toggle buzzer
bool timeEstimationBuzzerActive = false; // Status buzzer untuk estimasi waktu
unsigned long lastTimeEstimationBuzzerToggle = 0; // Waktu terakhir toggle buzzer estimasi waktu
int timeEstimationBuzzerCount = 0; // Hitungan bunyi buzzer untuk estimasi waktu

// NTC Thermistor Configuration
const int SENSOR_PIN = 2; // Pin sensor Suhu
OneWire oneWire(SENSOR_PIN);
DallasTemperature sensors(&oneWire);

const double REFERENCE_RESISTANCE = 10000;  // Resistor seri 10k ohm
const double NOMINAL_RESISTANCE = 10000;    // Resistansi nominal NTC pada 25°C
const double NOMINAL_TEMPERATURE = 25;      // Temperatur nominal dalam Celsius
const double B_COEFFICIENT = 3950;          // Koefisien beta NTC
const int SAMPLES = 5;                      // Jumlah sampel untuk rata-rata

// ESP32 pins definition for SIM7600
#define MODEM_UART_BAUD 115200
#define MODEM_TX 21
#define MODEM_RX 20
#define MODEM_PWRKEY 10

// OLED pins (I2C)
#define OLED_SDA 4
#define OLED_SCL 5

// Set serial for debug console (to the Serial Monitor)
#define SerialMon Serial
// Set serial for AT commands (to the SIM7600 module)
#define SerialAT Serial1

// Your GPRS credentials (leave empty, if missing)
const char apn[] = "internet";       // Your APN
const char gprs_user[] = "wap"; // User
const char gprs_pass[] = "wap123"; // Password
const char simPIN[] = "";    // SIM card PIN code, if any

// MQTT Broker Details - HiveMQ Cloud
String device_id = "ESP32_SIM7600";
const char* mqtt_server = "67d560452e2d4534b5decfc22c4cb938.s1.eu.hivemq.cloud";
const int mqtt_port = 8883; // TCP-TLS Port
const char* mqtt_user = "7600Test";
const char* mqtt_password = "7600Test!";
const char* mqtt_clientId = "ESP32_SIM7600_Client";
const char* topic_publish = "esp32/data";
const char* topic_subscribe = "esp32/command";
const char* topic_setpoint = "esp32/setpoint"; // Topik untuk mengatur setpoint suhu
const char* topic_time_data = "esp32/time"; // MQTT Topic for Time Data
const char* topic_time_status = "esp32/time_notification"; // MQTT Topic for Time Status

// GPS Variables
bool gpsEnabled = false;
String gpsLat = "";
String gpsLon = "";
String gpsAlt = "";
String gpsDate = "";
String gpsTime = "";
bool gpsFixObtained = false;

// RTC Object
RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// OLED Display Object
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// GSM Clients
TinyGsm sim_modem(SerialAT);
TinyGsmClient gsm_transport_layer(sim_modem);

// SSL Secure Client using GSM client
SSLClient secure_presentation_layer(&gsm_transport_layer);

// Variables for sensor data
unsigned long lastSensorRead = 0;
const unsigned long normalPublishInterval = 30000; // 30 seconds
const unsigned long fastPublishInterval = 10000;    // 10 seconds
unsigned long currentPublishInterval = normalPublishInterval; // Dynamic interval
int messageCount = 0;

// MQTT Client
void callback(char* topic, byte* payload, unsigned int length);

// New Function declarations for time tracking
void handleButtonPresses();
void sendTimeStatus(bool exceededEstimatedTime);
void checkTimeEstimationBuzzer();

PubSubClient mqtt_client(mqtt_server, mqtt_port, callback, secure_presentation_layer);

// Function declarations
void setupModem();
void turnModemOn();
void turnModemOff();
void reconnect();
void sendSensorData();
void sendCurrentSetpoint();
void checkTimeExceeded();

// Sensor Function declarations
float readTemperature();
void checkBuzzer(float temperature);
void toggleBuzzer();

// OLED Display Function declarations
void displayText(const char* text, int size, int x, int y);
void displaySensorValue(const char* sensorName, float value, const char* unit, int y);
void drawProgressBar(int x, int y, int width, int height, int progress);
void displayLoading(int delayMs);
void updateDisplay();

// GPS Function declarations
String sendAT(const String &cmd, unsigned long timeout = 1500);
bool initGPS();
bool startGPS();
bool readGPSInfo();
float toDecimalDegrees(const String &ddmm, bool isLat);

// GPS Functions Implementation
String sendAT(const String &cmd, unsigned long timeout) {
  String resp;
  SerialAT.println(cmd);
  unsigned long start = millis();
  while (millis() - start < timeout) {
    while (SerialAT.available()) {
      resp += (char)SerialAT.read();
    }
    // Stop earlier if OK/ERROR received
    if (resp.indexOf("\r\nOK") != -1 || resp.indexOf("\r\nERROR") != -1) break;
    delay(10);
  }
  SerialMon.print("AT> "); SerialMon.println(cmd);
  SerialMon.print("< "); SerialMon.println(resp);
  return resp;
}

bool initGPS() {
  SerialMon.println("Initializing GPS...");
  
  // Enable verbose error reporting
  sendAT("AT+CMEE=2", 1000);
  
  // Disable sleep mode
  sendAT("AT+CSCLK=0", 1000);
  
  // Enable GNSS power/LNA (for LilyGO boards - will fail on other boards)
  SerialMon.println("Activating GNSS power...");
  String resp = sendAT("AT+SGPIO=0,4,1,1", 2000);
  if (resp.indexOf("OK") == -1) {
    SerialMon.println("WARNING: SGPIO failed (not a LilyGO board)");
  }
  
  // Check GPS mode support
  sendAT("AT+CGPS=?", 1000);
  
  SerialMon.println("GPS initialization complete");
  return true;
}

bool startGPS() {
  SerialMon.println("Starting GPS...");
  
  // Check current GPS status
  String status = sendAT("AT+CGPS?", 1000);
  if (status.indexOf("+CGPS: 1") != -1) {
    SerialMon.println("GPS already active");
    gpsEnabled = true;
    return true;
  }
  
  // Setup AGPS for faster fix (optional)
  SerialMon.println("Setting up AGPS...");
  sendAT("AT+CGPSURL=\"supl.google.com:7276\"", 2000);  // Google SUPL server
  sendAT("AT+CGPSSSL=0", 1000);  // Non-SSL for compatibility
  
  // Try different GPS modes for better compatibility
  SerialMon.println("Sending AT+CGPS=1...");
  String r = sendAT("AT+CGPS=1", 3000);
  
  if (r.indexOf("OK") == -1) {
    SerialMon.println("Trying AT+CGPS=1,2 (UE-based mode)...");
    r = sendAT("AT+CGPS=1,2", 3000);
    if (r.indexOf("OK") == -1) {
      SerialMon.println("Trying AT+CGPS=1,1 (standalone)...");
      r = sendAT("AT+CGPS=1,1", 3000);
      if (r.indexOf("OK") == -1) {
        SerialMon.println("ERROR: All GPS start attempts failed!");
        return false;
      }
    }
  }

  // Verify GPS is active
  delay(1000);
  status = sendAT("AT+CGPS?", 1000);
  if (status.indexOf("+CGPS: 1") == -1) {
    SerialMon.println("ERROR: GPS not active after start command");
    return false;
  }

  // Optional: set NMEA output and rate
  sendAT("AT+CGPSNMEA=7", 1000);        // GGA+RMC+GSV
  sendAT("AT+CGPSNMEARATE=1000", 1000); // 1 second

  gpsEnabled = true;
  SerialMon.println("GPS successfully activated!");
  SerialMon.println("Wait a few minutes to get GPS fix...");
  return true;
}

// Convert DDMM.MMMMM (lat: 2 digit degrees, lon: 3 digit degrees) to decimal degrees
float toDecimalDegrees(const String &ddmm, bool isLat) {
  if (ddmm.length() < 4) return NAN;
  int degDigits = isLat ? 2 : 3;
  float deg = ddmm.substring(0, degDigits).toFloat();
  float min = ddmm.substring(degDigits).toFloat();
  return deg + (min / 60.0f);
}

bool readGPSInfo() {
  if (!gpsEnabled) {
    SerialMon.println("GPS not enabled");
    return false;
  }

  // Ensure GPS is active
  String s = sendAT("AT+CGPS?", 1000);
  if (s.indexOf("+CGPS: 0") != -1) {
    SerialMon.println("GPS not active, trying to restart...");
    if (!startGPS()) {
      SerialMon.println("Failed to restart GPS");
      return false;
    }
    delay(1500);
  }

  // Get fix info
  String resp = sendAT("AT+CGPSINFO", 2000);
  int p = resp.indexOf("+CGPSINFO:");
  if (p == -1) {
    SerialMon.println("No CGPSINFO response");
    return false;
  }

  int lineStart = resp.indexOf(':', p) + 2; // skip ": "
  int lineEnd = resp.indexOf("\r", lineStart);
  if (lineEnd == -1) lineEnd = resp.length();
  String data = resp.substring(lineStart, lineEnd);
  
  SerialMon.println("Raw GPS data: " + data);
  

  if (data.indexOf(",,,,,,,,,") != -1 || data.length() < 10) {
    SerialMon.println("No GPS fix - try waiting in open area");
    SerialMon.println("Tip: Ensure GNSS antenna is connected and outdoors");
    gpsFixObtained = false;
    return false;
  }

  // Split CSV
  String v[9];
  int idx = 0, start = 0;
  for (int i = 0; i < data.length() && idx < 9; i++) {
    if (data.charAt(i) == ',' || i == data.length() - 1) {
      v[idx++] = data.substring(start, (i == data.length() - 1) ? i + 1 : i);
      start = i + 1;
    }
  }

  if (v[0].length() == 0 || v[2].length() == 0) {
    SerialMon.println("Incomplete GPS data");
    gpsFixObtained = false;
    return false;
  }

  float lat = toDecimalDegrees(v[0], true);
  if (v[1] == "S") lat = -lat;
  float lon = toDecimalDegrees(v[2], false);
  if (v[3] == "W") lon = -lon;

  gpsLat = String(lat, 6);
  gpsLon = String(lon, 6);
  gpsDate = v[4];      // DDMMYY
  gpsTime = v[5];      // HHMMSS.S
  gpsAlt = v[6];       // meter

  SerialMon.println("=== GPS Fix Success ===");
  SerialMon.println("  Lat: " + gpsLat);
  SerialMon.println("  Lon: " + gpsLon);
  SerialMon.println("  Alt: " + gpsAlt + " m");
  SerialMon.println("  UTC: " + gpsDate + " " + gpsTime);
  SerialMon.println("========================");
  
  gpsFixObtained = true;
  return true;
}

void setup() {
  #ifdef ESP32C3
    SerialMon.begin(115200);
  #else
    SerialMon.begin(115200);
  #endif
  delay(2000);

  preferences.begin("my-app", false); // Initialize Preferences

  // Baca tempSetpoint dari NVS
  tempSetpoint = preferences.getFloat("tempSetpoint", 25.0); // Default 25.0
  SerialMon.print("Setpoint dari NVS: ");
  SerialMon.println(tempSetpoint);

  // Simpan nilai default ke NVS jika belum ada (atau untuk memastikan ada)
  preferences.putFloat("tempSetpoint", tempSetpoint);

  // Baca estimated time components dari NVS
  estimatedDays = preferences.getLong("estDays", 0);
  estimatedHours = preferences.getLong("estHours", 0);
  estimatedMinutes = preferences.getLong("estMins", 0);
  estimatedSeconds = preferences.getLong("estSecs", 0);

  // Simpan nilai default ke NVS jika belum ada (atau untuk memastikan ada)
  preferences.putLong("estDays", estimatedDays);
  preferences.putLong("estHours", estimatedHours);
  preferences.putLong("estMins", estimatedMinutes);
  preferences.putLong("estSecs", estimatedSeconds);

  SerialMon.println("ESP32-C3 DeliveryBox starting...");

  // set up OLED di ESP32-C3
  Wire.begin(OLED_SDA, OLED_SCL);

  // Inisialisasi sensor DS18B20
  sensors.begin();
  
  // Inisialisasi RTC
  SerialMon.println("Initializing RTC...");
  if (!rtc.begin()) {
    SerialMon.println("Couldn't find RTC");
    while (1);
  }
  
  // Jika RTC kehilangan daya, atur waktu ke waktu kompilasi
  if (rtc.lostPower()) {
    SerialMon.println("RTC lost power, setting time to compile time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  
  // Setup buzzer pin
  pinMode(BUZZER_PIN, OUTPUT);

  // Setup button pins
  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(STOP_BUTTON_PIN, INPUT_PULLUP);
  
  // Initialize OLED Display
  SerialMon.println("Initializing OLED Display...");
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    SerialMon.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  
  // Clear the buffer and show startup animation
  display.clearDisplay();
  displayLoading(50);
  
  // Show startup message
  display.clearDisplay();
  displayText("SIM7600 ESP32", 1, 0, 0);
  displayText("Starting...", 1, 0, 16);
  display.display();
  delay(2000);
  
  // ADC Configuration for NTC Thermistor
  analogReadResolution(12);       // Set ADC resolution ke 12-bit
  analogSetAttenuation(ADC_11db); // Set attenuation untuk range 0-3.3V
  
  // Set SIM module baud rate and UART pins
  SerialAT.begin(MODEM_UART_BAUD, SERIAL_8N1, MODEM_RX, MODEM_TX);

  // Add CA Certificate
  secure_presentation_layer.setCACert(root_ca);

  // SIM modem initial setup
  setupModem();
  
  // Initialize GPS
  SerialMon.println("Initializing GPS system...");
  initGPS();
  
  // Start GPS with retry, 2 kali
  int gpsRetries = 2;
  bool gpsStarted = false;
  
  // Mencoba menghubungkan GPS 2 kali, setiap 3 detik (jika GPS tidak terhubung)
  for (int i = 0; i < gpsRetries && !gpsStarted; i++) {
    SerialMon.printf("GPS start attempt %d...\n", i + 1);
    gpsStarted = startGPS();
    if (!gpsStarted) {
      SerialMon.println("GPS start failed, waiting 3 seconds...");
      delay(3000);
    }
  }
  
  if (!gpsStarted) {
    SerialMon.println("WARNING: GPS failed to start after 3 attempts");
    SerialMon.println("Continuing without GPS functionality");
  } else {
    SerialMon.println("GPS system ready - searching for satellites...");
  }
}

// Variabel untuk status koneksi
bool isConnected = false;
unsigned long lastConnectionAttempt = 0;
const unsigned long connectionInterval = 60000; // Coba koneksi setiap 60 detik jika terputus

void loop() {
  // Variabel untuk timing
  static unsigned long lastDisplayUpdate = 0;
  static unsigned long lastGPSRead = 0;
  
  // ===== BAGIAN KOMUNIKASI (BERJALAN JIKA TERHUBUNG) =====
  // Coba koneksi jika belum terhubung dan jika sudah waktunya mencoba lagi
  if (!isConnected && (millis() - lastConnectionAttempt >= connectionInterval)) {
    SerialMon.println("Mencoba menghubungkan modem...");
    display.clearDisplay();
    displayText("Mencoba", 1, 0, 0);
    displayText("koneksi...", 1, 0, 16);
    display.display();
    
    lastConnectionAttempt = millis();
    
    // Inisialisasi modem
    if (!sim_modem.init()) {
      SerialMon.println("Inisialisasi modem gagal, mencoba restart...");
      setupModem();
    }
    
    // Informasi modem
    String name = sim_modem.getModemName();
    Serial.println("Modem Name: " + name);
    String modem_info = sim_modem.getModemInfo();
    Serial.println("Modem Info: " + modem_info);
    
    // Unlock SIM card jika diperlukan
    if (strlen(simPIN) && sim_modem.getSimStatus() != 3) {
      sim_modem.simUnlock(simPIN);
    }
    
    // Set mode jaringan
    sim_modem.setNetworkMode(2); // Automatic
    delay(2000);
    
    // Tunggu jaringan tersedia
    SerialMon.print("Menunggu jaringan...");
    if (!sim_modem.waitForNetwork(3000)) { // Timeout 30 detik
      SerialMon.println(" gagal");
      isConnected = false;
      return;
    }
    SerialMon.println(" OK");
    
    // Koneksi ke jaringan GPRS
    if (!sim_modem.isNetworkConnected()) {
      SerialMon.println("Koneksi jaringan gagal");
      isConnected = false;
      return;
    }
    
    // Koneksi ke APN
    SerialMon.print("Menghubungkan ke APN: ");
    SerialMon.print(apn);
    if (!sim_modem.gprsConnect(apn, gprs_user, gprs_pass)) {
      SerialMon.println(" gagal");
      isConnected = false;
      return;
    }
    SerialMon.println(" OK");
    
    // Informasi tambahan
    String ccid = sim_modem.getSimCCID();
    Serial.println("CCID: " + ccid);
    String imei = sim_modem.getIMEI();
    Serial.println("IMEI: " + imei);
    String cop = sim_modem.getOperator();
    Serial.println("Operator: " + cop);
    IPAddress local = sim_modem.localIP();
    Serial.println("Local IP: " + String(local));
    int csq = sim_modem.getSignalQuality();
    Serial.println("Signal quality: " + String(csq));
    
    isConnected = true;
  }

  // ===== BAGIAN SENSOR DAN BUZZER (SELALU BERJALAN) =====
  // Baca sensor suhu dan update display setiap 1 detik
  if (millis() - lastDisplayUpdate >= 1000) {
    // Baca suhu dan kontrol buzzer (fungsi readTemperature sudah memanggil checkBuzzer)
    float currentTemp = readTemperature();
    
    // Logika untuk mengubah interval pengiriman data MQTT
    if (currentTemp >= tempSetpoint) {
      currentPublishInterval = fastPublishInterval; // Kirim lebih cepat jika suhu di atas setpoint
    } else {
      currentPublishInterval = normalPublishInterval; // Kembali ke interval normal
    }
    
    // Update display dengan status terbaru
    updateDisplay();
    
    lastDisplayUpdate = millis();
  }

  // Periksa apakah estimasi waktu melebihi batas
  checkTimeExceeded();
  
  // Periksa dan kontrol buzzer untuk estimasi waktu
  checkTimeEstimationBuzzer();
  
  // Jika terhubung, jalankan fungsi komunikasi
  if (isConnected) {
    // Periksa apakah masih terhubung ke GPRS
    if (sim_modem.isGprsConnected()) {
      // Koneksi ke MQTT broker jika belum terhubung
      if (!mqtt_client.connected()) {
        reconnect();
      }
      
      // Proses event MQTT
      mqtt_client.loop();
      
      // Baca data GPS secara periodik (setiap 10 detik)
      if (gpsEnabled && (millis() - lastGPSRead >= 5000)) {
        lastGPSRead = millis();
        SerialMon.println("Membaca data GPS...");
        readGPSInfo();
      }
      
      // Kirim data sensor secara periodik
      if (millis() - lastSensorRead >= currentPublishInterval) {
        sendSensorData();
        lastSensorRead = millis();
      }
    } else {
      // GPRS terputus, tandai tidak terhubung
      SerialMon.println("Koneksi GPRS terputus");
      isConnected = false;
    }
  }
  
  // Delay kecil untuk mencegah CPU overload
  delay(10);

  // Handle button presses
  handleButtonPresses();
}

// Function to handle button presses

void turnModemOn() {

  pinMode(MODEM_PWRKEY, OUTPUT);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(1000); //Datasheet Ton mintues = 1S
  digitalWrite(MODEM_PWRKEY, HIGH);
}

void turnModemOff() {
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(1500); //Datasheet Ton mintues = 1.2S
  digitalWrite(MODEM_PWRKEY, HIGH);

}

void setupModem() {

  pinMode(MODEM_PWRKEY, OUTPUT);

  turnModemOff();
  delay(1000);
  turnModemOn();
  delay(2000);
}

// To connect to the broker
// Tambahkan variabel global untuk timer MQTT
unsigned long lastMqttAttempt = 0;
const unsigned long mqttReconnectInterval = 10000; // 10 detik

void reconnect() {
  // Hanya coba jika belum terhubung dan sudah waktunya mencoba lagi
  if (!mqtt_client.connected()) {
    if (millis() - lastMqttAttempt >= mqttReconnectInterval) {
      SerialMon.print("Attempting MQTT connection...");
      lastMqttAttempt = millis(); // Reset timer

      // Attempt to connect
      if (mqtt_client.connect(mqtt_clientId, mqtt_user, mqtt_password)) {
        SerialMon.println("connected");
        // Once connected, publish an announcement...
        mqtt_client.publish(topic_publish, "ESP32 SIM7600 connected");
        // ... and resubscribe
        mqtt_client.subscribe(topic_subscribe);
        mqtt_client.subscribe(topic_setpoint); // Subscribe ke topik setpoint
        mqtt_client.subscribe(topic_time_data); // Subscribe ke topik waktu estimasi
        
        // Kirim nilai setpoint saat ini
        sendCurrentSetpoint();
      } else {
        SerialMon.print("failed, rc=");
        SerialMon.print(mqtt_client.state());
        SerialMon.println("...retrying in 5 seconds");
        // Tidak ada delay() di sini
      }
    }
  }
}

// Fungsi untuk mengirim nilai setpoint saat ini
void sendCurrentSetpoint() {
  if (mqtt_client.connected()) {
    StaticJsonDocument<100> doc;
    doc["deviceid"] = device_id;
    doc["atur_setpoint"] = tempSetpoint;
    
    String jsonString;
    serializeJson(doc, jsonString);
    
    Serial.print("Publishing current setpoint: ");
    Serial.println(jsonString);
    
    mqtt_client.publish(topic_setpoint, jsonString.c_str());
  }
}

// For reading the MQTT events
void callback(char *topic, uint8_t *payload, unsigned int length) {
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("] ");
  
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  SerialMon.println(message);
  
  // Parse JSON command if needed
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, message);
  
  // Cek apakah pesan untuk topik setpoint
  if (strcmp(topic, topic_setpoint) == 0) {
    if (!error) {
      // Jika ada field "atur_setpoint" dalam JSON, gunakan sebagai setpoint baru
      if (doc.containsKey("atur_setpoint")) {
        float newSetpoint = doc["atur_setpoint"];
        tempSetpoint = newSetpoint;
        preferences.putFloat("tempSetpoint", tempSetpoint); // Simpan setpoint baru ke NVS
        SerialMon.print("Setpoint baru diterima: ");
        SerialMon.println(tempSetpoint);
        
        // Kirim konfirmasi perubahan setpoint
        StaticJsonDocument<100> responseDoc;
        responseDoc["status"] = "success";
        responseDoc["setpoint"] = tempSetpoint;
        
        String responseJson;
        serializeJson(responseDoc, responseJson);
        mqtt_client.publish(topic_publish, responseJson.c_str());
      }
    } else {
      SerialMon.print("Gagal parsing JSON: ");
      SerialMon.println(error.c_str());
    }
  } else if (strcmp(topic, topic_time_data) == 0) { // Cek apakah pesan untuk topik waktu estimasi
    if (!error) {
      if (doc.containsKey("days")) {
        estimatedDays = doc["days"];
        preferences.putLong("estDays", estimatedDays);
      } else {
        estimatedDays = 0;
        preferences.putLong("estDays", estimatedDays);
      }
      if (doc.containsKey("hours")) {
        estimatedHours = doc["hours"];
        preferences.putLong("estHours", estimatedHours);
      } else {
        estimatedHours = 0;
        preferences.putLong("estHours", estimatedHours);
      }
      if (doc.containsKey("minutes")) {
        estimatedMinutes = doc["minutes"];
        preferences.putLong("estMins", estimatedMinutes);
      } else {
        estimatedMinutes = 0;
        preferences.putLong("estMins", estimatedMinutes);
      }
      if (doc.containsKey("seconds")) {
        estimatedSeconds = doc["seconds"];
        preferences.putLong("estSecs", estimatedSeconds);
      } else {
        estimatedSeconds = 0;
        preferences.putLong("estSecs", estimatedSeconds);
      }

      estimatedDurationMs = (estimatedDays * 24 * 3600 + estimatedHours * 3600 + estimatedMinutes * 60 + estimatedSeconds) * 1000;
      SerialMon.print("Estimated time received: ");
      SerialMon.print(estimatedDays);
      SerialMon.print(" hari, ");
      SerialMon.print(estimatedHours);
      SerialMon.print(" jam, ");
      SerialMon.print(estimatedMinutes);
      SerialMon.print(" menit, ");
      SerialMon.print(estimatedSeconds);
      SerialMon.println(" detik.");
      SerialMon.print("Estimated Duration in ms: ");
      SerialMon.println(estimatedDurationMs);

      // Kirim konfirmasi perubahan estimasi waktu
      StaticJsonDocument<200> responseDoc;
      responseDoc["status"] = "success";
      responseDoc["estimated_time_set"] = true;
      responseDoc["days"] = estimatedDays;
      responseDoc["hours"] = estimatedHours;
      responseDoc["minutes"] = estimatedMinutes;
      responseDoc["seconds"] = estimatedSeconds;

      String responseJson;
      serializeJson(responseDoc, responseJson);
      mqtt_client.publish(topic_publish, responseJson.c_str());
    } else {
      SerialMon.print("Gagal parsing JSON untuk estimasi waktu: ");
      SerialMon.println(error.c_str());
    }
  }
}

// Temperature Sensor Functions Implementation
float readTemperature() {
  // Request suhu dari sensor
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);
  
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println("°C | Setpoint: " + String(tempSetpoint));
  // Serial.println("---------------");
  
  // Periksa status buzzer berdasarkan suhu
  checkBuzzer(temperature);
  
  return temperature;
  delay(500);
}

// Fungsi untuk memeriksa apakah buzzer perlu diaktifkan berdasarkan suhu
void checkBuzzer(float temperature) {
  // Jika suhu melebihi setpoint, aktifkan buzzer dengan pola on/off
  if (temperature > tempSetpoint) {
    // Toggle buzzer setiap 1 detik
    if (millis() - lastBuzzerToggle >= 1000) {
      toggleBuzzer();
      lastBuzzerToggle = millis();
    }
  } else {
    // Jika suhu di bawah setpoint, pastikan buzzer mati
    if (buzzerState) {
      noTone(BUZZER_PIN);
      buzzerState = false;
    }
  }
  
  // Tampilkan status setpoint dan buzzer di serial monitor
  static unsigned long lastStatusPrint = 0;
  if (millis() - lastStatusPrint >= 1000) { // Tampilkan setiap 2 detik
    Serial.println("=============");
    // Serial.print("Setpoint: ");
    // Serial.print(tempSetpoint);
    // Serial.println("°C");
    
    Serial.print("Buzzer: ");
    Serial.println(buzzerState ? "ON" : "OFF");
    Serial.println("=============");
    
    lastStatusPrint = millis();
  }
}

// Fungsi untuk toggle buzzer on/off
void toggleBuzzer() {
  if (buzzerState) {
    noTone(BUZZER_PIN);
    buzzerState = false;
  } else {
    tone(BUZZER_PIN, 40); // Frekuensi 300 Hz
    buzzerState = true;
  }
}

// Fungsi untuk mengontrol buzzer estimasi waktu
void checkTimeEstimationBuzzer() {
  // Jika buzzer estimasi waktu aktif
  if (timeEstimationBuzzerActive) {
    // Jika sudah waktunya untuk toggle buzzer (setiap 500ms)
    if (millis() - lastTimeEstimationBuzzerToggle >= 500) {
      // Jika buzzer sedang berbunyi, matikan
      if (buzzerState) {
        noTone(BUZZER_PIN);
        buzzerState = false;
        timeEstimationBuzzerCount++;
        
        // Jika sudah berbunyi 2 kali, tunggu 3 detik sebelum berbunyi lagi
        if (timeEstimationBuzzerCount >= 2) {
          timeEstimationBuzzerCount = 0;
          lastTimeEstimationBuzzerToggle = millis() - 2500; // Tunggu 3 detik total (500ms + 2500ms)
        }
      } else {
        // Jika buzzer mati dan belum mencapai 2 kali, nyalakan
        if (timeEstimationBuzzerCount < 2) {
          tone(BUZZER_PIN, 40); // Frekuensi 40 Hz untuk alarm
          buzzerState = true;
        }
      }
      lastTimeEstimationBuzzerToggle = millis();
    }
  } else {
    // Pastikan buzzer mati jika tidak aktif
    if (buzzerState) {
      noTone(BUZZER_PIN);
      buzzerState = false;
    }
    timeEstimationBuzzerCount = 0;
  }
}

void sendSensorData() {
  if (mqtt_client.connected()) {
    // Create simplified JSON payload
    StaticJsonDocument<200> doc;
    doc["deviceid"] = device_id;
    
    // Add GPS coordinates (lat, long)
    if (gpsEnabled && gpsFixObtained) {
      doc["lat"] = gpsLat.toFloat();
      doc["long"] = gpsLon.toFloat();
    } else {
      doc["lat"] = 0.0;
      doc["long"] = 0.0;
    }
    
    // Add sensor data (temp)
    doc["temp"] = readTemperature();
    
    // Add current setpoint
    doc["setpoint"] = tempSetpoint;
    
    String jsonString;
    serializeJson(doc, jsonString);
    
    Serial.print("Publishing sensor data: ");
    Serial.println(jsonString);
    
    if (mqtt_client.publish(topic_publish, jsonString.c_str())) {
      Serial.println("Data published successfully");
      if (gpsFixObtained) {
        Serial.println("GPS Location: https://maps.google.com/?q=" + gpsLat + "," + gpsLon);
      }
    } else {
      Serial.println("Failed to publish data");
    }
  } else {
    Serial.println("MQTT not connected, cannot send data");
  }
}

// Send AT command and get response
String sendATCommand(String command, unsigned long timeout) {
  String response = "";
  SerialAT.println(command);
  
  unsigned long startTime = millis();
  while (millis() - startTime < timeout) {
    if (SerialAT.available()) {
      response += SerialAT.readString();
      break;
    }
    delay(10);
  }
  
  Serial.print("AT Command: ");
  Serial.println(command);
  Serial.print("Response: ");
  Serial.println(response);
  
  return response;
}

// OLED Display Helper Functions
void displayText(const char* text, int size, int x, int y) {
  display.setTextSize(size);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x, y);
  display.println(text);
  display.display();
}

void checkTimeExceeded() {
  if (timingInProgress) {
    unsigned long currentElapsedTime = millis() - startTime;
    if (currentElapsedTime > estimatedDurationMs) {
      // Waktu telah melebihi estimasi
      if (!timeEstimationBuzzerActive) {
        timeEstimationBuzzerActive = true;
        lastTimeEstimationBuzzerToggle = millis();
        display.clearDisplay();
        displayText("Waktu melebihi", 1, 0, 0);
        displayText("batas! Tekan STOP!", 1, 0, 16);
        display.display();
        SerialMon.println("Peringatan: Waktu telah melebihi estimasi!");
        sendTimeStatus(true); // Kirim status true ke MQTT pertama kali
        lastTimeStatusSent = millis(); // Catat waktu pengiriman pertama
      }
      // Kirim pesan 'true' setiap 10 detik jika buzzer aktif
      else if (millis() - lastTimeStatusSent >= 10000) {
        sendTimeStatus(true);
        lastTimeStatusSent = millis();
      }
    }
  }
}

// Function to handle button presses
void handleButtonPresses() {
  // Read the state of the buttons
  int readingStart = digitalRead(START_BUTTON_PIN);
  int readingStop = digitalRead(STOP_BUTTON_PIN);

  // Debounce logic for START button
  if (readingStart != lastButtonStateStart) {
    lastDebounceTimeStart = millis();
  }
  if ((millis() - lastDebounceTimeStart) > debounceDelay) {
    if (readingStart != buttonStateStart) {
      buttonStateStart = readingStart;
      if (buttonStateStart == LOW) { // Button is pressed (LOW because of INPUT_PULLUP)
        if (!timingInProgress) {
          startTime = millis();
          timingInProgress = true;
          // Pastikan buzzer dimatikan saat memulai estimasi baru
          timeEstimationBuzzerActive = false;
          SerialMon.println("Estimasi dimulai...");
          display.clearDisplay();
          display.setTextSize(1);
          display.setTextColor(SSD1306_WHITE);
          display.setCursor(0, 0);
          display.println("Estimasi dimulai...");
          display.print("Batas: ");
          display.print(estimatedDays);
          display.print("-");
          display.print(estimatedHours);
          display.print("-");
          display.print(estimatedMinutes);
          display.print("-");
          display.println(estimatedSeconds);
          display.display();
          delay(1000);
        } else {
          SerialMon.println("Timer already running.");
        }
      }
    }
  }
  lastButtonStateStart = readingStart;

  // Debounce logic for STOP button
  if (readingStop != lastButtonStateStop) {
    lastDebounceTimeStop = millis();
  }
  if ((millis() - lastDebounceTimeStop) > debounceDelay) {
    if (readingStop != buttonStateStop) {
      buttonStateStop = readingStop;
      if (buttonStateStop == LOW) { // Button is pressed (LOW because of INPUT_PULLUP)
        if (timingInProgress) {
          stopTime = millis();
          timingInProgress = false;
          // Matikan buzzer saat tombol stop ditekan
          timeEstimationBuzzerActive = false;
          SerialMon.println("Estimasi waktu dihentikan ...");
          // Hapus tampilan "Waktu melebihi batas! Tekan STOP!" dari OLED
          display.clearDisplay();
          displayText("Estimasi berhenti", 1, 0, 0);
          display.display();
          delay(1000);
        } else {
          SerialMon.println("Timer not running.");
        }
      }
    }
  }
  lastButtonStateStop = readingStop;
}

// Function to publish time data to MQTT
void sendTimeStatus(bool exceededEstimatedTime) {
  if (mqtt_client.connected()) {
    if (exceededEstimatedTime) {
      StaticJsonDocument<200> doc;
      doc["message"] = true;
      String jsonMessage;
      serializeJson(doc, jsonMessage);

      // Kirim pesan JSON ke broker
      mqtt_client.publish(topic_time_status, jsonMessage.c_str());
      SerialMon.println("Published JSON " + jsonMessage + " to MQTT topic 'esp32/time_notification' due to exceeded time.");
    }
  } else {
    SerialMon.println("MQTT client not connected, cannot send time status.");
  }
}

void displaySensorValue(const char* sensorName, float value, const char* unit, int y) {
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, y);
  display.print(sensorName);
  display.print(": ");
  display.print(value);
  display.println(unit);
  display.display();
}

void drawProgressBar(int x, int y, int width, int height, int progress) {
  // Gambar border
  display.drawRect(x, y, width, height, SSD1306_WHITE);
  
  // Gambar progress (maksimum 100%)
  int progressWidth = (progress * (width - 4)) / 100;
  if (progressWidth > width - 4) progressWidth = width - 4;
  
  display.fillRect(x + 2, y + 2, progressWidth, height - 4, SSD1306_WHITE);
  display.display();
}

void displayLoading(int delayMs) {
  for (int i = 0; i <= 100; i += 10) {
    display.clearDisplay();
    displayText("Loading...", 1, 0, 0);
    drawProgressBar(0, 20, 128, 10, i);
    delay(delayMs);
  }
}

void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // BARIS 1 = Suhu dan Setpoint
  display.setCursor(0, 0);
  display.print(F("Temp:"));
  
  // Temperature
  float temperature = readTemperature();
  display.print(temperature, 1);
  display.print(F("C "));
  
  // Setpoint
  display.print(F("SP:"));
  display.print(tempSetpoint, 1);
  display.println(F("C"));
  
  // BARIS 2 =  Status koneksi atau koordinat GPS
  display.setCursor(0, 8);
  if (isConnected && sim_modem.isGprsConnected()) {
    if (gpsFixObtained) {
      display.print(F("Lat:"));
      display.println(gpsLat);
      display.print(F(" Lon:"));
      display.print(gpsLon);
    } else {
      display.print(F("Status: ONLINE"));
    }
  } else {
    display.print(F("Status: OFFLINE"));
  }
  
  // Baris 3-4: Tanggal dan Waktu dari RTC
  DateTime now = rtc.now();
  
  // BARIS 3 = Tanggal: YYYY/MM/DD
  display.setCursor(0, 24);
  display.print(now.year(), DEC);
  display.print(F("/"));
  if (now.month() < 10) display.print(F("0"));
  display.print(now.month(), DEC);
  display.print(F("/"));
  if (now.day() < 10) display.print(F("0"));
  display.print(now.day(), DEC);
  display.print(F(" | "));
  // display.print(daysOfTheWeek[now.dayOfTheWeek()]);
  
  // BARIS 4 = Waktu: HH:MM:SS
  // display.setCursor(0, 24);
  if (now.hour() < 10) display.print(F("0"));
  display.print(now.hour(), DEC);
  display.print(F(":"));
  if (now.minute() < 10) display.print(F("0"));
  display.print(now.minute(), DEC);
  display.print(F(":"));
  if (now.second() < 10) display.print(F("0"));
  display.print(now.second(), DEC);
  
  display.display();
}