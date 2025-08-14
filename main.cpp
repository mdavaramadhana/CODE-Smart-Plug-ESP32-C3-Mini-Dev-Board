#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <PZEM004Tv30.h>
#include <HardwareSerial.h>
#include "driver/timer.h"
#include <map>
#include <HTTPClient.h>
#include <vector>
#include <time.h>

esp_timer_handle_t timer;

HardwareSerial mySerial(0);

#define PZEM_RX 0
#define PZEM_TX 0

PZEM004Tv30 pzem(mySerial, PZEM_RX, PZEM_TX);

const long gmtOffset_sec = 7 * 3600; // 7 jam dalam detik
const int daylightOffset_sec = 0;   // Tidak ada daylight saving di Jakarta

// Konfigurasi WiFi
const char* ssid = "UNLMTD";
const char* password = "DAVAMAMA2";

// Konfigurasi WebSocket
const char* websocket_server = "api.penaku.site";  
const int websocket_port = 443;
const char* websocket_path = "/ws_esp/plug_meter_2";
const char* uid = "Q7gzuuSocRfME5PEmLAWD6H8bcM2";  

bool automation = false;
bool auto_dimming = false;
bool email_me = false;

// Konfigurasi sensor DHT11
#define DHTPIN 0
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Konfigurasi relay
#define relay_1 5
#define relay_2 4

// const int relayPin1 = 8;  // Relay untuk device_1
// const int relayPin2 = 7;  // Relay untuk device_2

WebSocketsClient webSocket;

volatile int i = 0;
volatile bool zero_cross = 0;  // Flag untuk mendeteksi zero-cross
int AC_pin = 11;               // Output ke Opto Triac (ubah ke pin ESP32 yang sesuai)
int dim = 0;                   // Level kecerahan (0-128)
int freqStep = 75;             // Delay per tingkat kecerahan (untuk 50Hz gunakan 75, untuk 60Hz gunakan 65)
int dimmerValue = 0;


// Deklarasi struct Measurement
struct Measurement {
    float voltage;
    float current;
    float power;
    float energy;
    float frequency;
    float pf;  // Power factor
};

// Fungsi untuk mengukur semua parameter dari PZEM-004T
Measurement measurement() {
    float voltage = pzem.voltage();
    float current = pzem.current();
    float power = pzem.power();
    float energy = pzem.energy();
    float frequency = pzem.frequency();
    float pf = pzem.pf(); // Power factor

    // Penanganan jika data tidak tersedia atau terjadi error
    if (isnan(voltage)) {
        voltage = 0;
    }
    if (isnan(current)) {
        current = 0;
    }
    if (isnan(power)) {
        power = 0;
    }
    if (isnan(energy)) {
        energy = 0;
    }
    if (isnan(frequency)) {
        frequency = 0;
    }
    if (isnan(pf)) {
        pf = 0;
    }

    // Mengembalikan semua parameter dalam bentuk struct
    return {voltage, current, power, energy, frequency, pf};
}

void IRAM_ATTR onTimer(void* arg) {
    dim_check();  // Memanggil fungsi dim_check setiap timer interrupt
}

void IRAM_ATTR zero_cross_detect() {
    zero_cross = true;
    i = 0;
    digitalWrite(AC_pin, LOW); // Matikan TRIAC (dan AC)
}

void setup() {
    Serial.begin(115200);

    mySerial.begin(9600, SERIAL_8N1, 0, 0);
    
    // Cek alasan reset terakhir
    Serial.println(esp_reset_reason());

    // Koneksi WiFi
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");

            // Inisialisasi waktu NTP
    configTime(gmtOffset_sec, daylightOffset_sec, "pool.ntp.org", "time.nist.gov");
    Serial.println("Time configured");
    
    // Inisialisasi sensor DHT
    dht.begin();
    
    // Konfigurasi WebSocket
    webSocket.beginSSL(websocket_server, websocket_port, websocket_path);
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);

    pinMode(AC_pin, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(3), zero_cross_detect, RISING); // Sesuaikan pin dengan pin ESP32 yang mendukung interrupt

    // Timer callback configuration
    const esp_timer_create_args_t timer_args = {
        .callback = &onTimer,  // Fungsi yang akan dipanggil setiap kali timer interrupt
        .name = "Dimmer Timer" // Nama timer (opsional)
    };

    // Membuat timer
    esp_timer_create(&timer_args, &timer);

    // Start timer periodik (misalnya setiap 1 detik atau sesuai dengan frekuensi Anda)
    esp_timer_start_periodic(timer, freqStep * 1000);  // Perhatikan unit mikrodetik

    pinMode(relay_1, OUTPUT);
    pinMode(relay_2, OUTPUT);
}

void dim_check() {
    if (zero_cross == true) {              
        if (i >= dim) {                    
            digitalWrite(AC_pin, HIGH);  // Nyalakan lampu       
            i = 0;                         
            zero_cross = false; // Reset zero cross detection
        } else {
            i++; // Inkrement counter                    
        }
    }
}

void dimming(){
    int aVal = dimmerValue; // Membaca potensiometer atau input sensor lainnya
    dim = map(aVal, 100, 0, 0, 128); // ESP32 menggunakan nilai ADC dari 0 hingga 4095
    delay(60);
}

void loop() {
    dimming();
    webSocket.loop();
    delay(2);

    static unsigned long lastCheck = 0;
    if (millis() - lastCheck > 1000) {
        checkSchedules();
        lastCheck = millis();
    }
    
    // Baca data sensor setiap 5 detik
    static unsigned long lastTime = 0;
    if (millis() - lastTime > 2000) {
        sendSensorData();
        lastTime = millis();
    }
}

void sendSensorData() {
    Measurement data = measurement(); // Get voltage and current
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    
    if (isnan(humidity) || isnan(temperature)) {
        Serial.println("Gagal membaca sensor DHT!");
        return;
    }
    // Mendapatkan waktu lokal
    struct tm timeinfo;
    char timeBuffer[16]; // HH:MM:SS.mmm
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Gagal mendapatkan waktu");
        strcpy(timeBuffer, "00:00:00.000");
    } else {
        // Ambil waktu milidetik sekarang
        uint16_t milli = millis() % 1000;
        snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d:%02d.%03d",
                 timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, milli);
    }
    
    // Buat JSON object
    // Struktur untuk menyimpan jadwal
    StaticJsonDocument<500> doc; // Pastikan ukuran dokumen cukup besar
    doc["uid"] = uid;
    doc["time_sent"] = timeBuffer;
    doc["voltage"] = data.voltage;
    doc["current"] = data.current;
    doc["power"] = data.power;
    doc["energy"] = data.energy;
    doc["frequency"] = data.frequency;
    doc["pf"] = data.pf; // Power factor
    doc["device_id"] = "plug_meter_2";  // Sesuaikan dengan ESP32-S3
    doc["temperature"] = temperature - 2; // Menyesuaikan nilai suhu
    doc["humidity"] = humidity - 2; // Menyesuaikan nilai kelembaban
        
    // Serialize JSON ke string
    String jsonString;
    serializeJson(doc, jsonString);
    
    // Kirim data
    webSocket.sendTXT(jsonString);
    Serial.println("Data terkirim: " + jsonString);
}


// Struktur untuk menyimpan jadwal
struct Schedule {
    int id;
    String user_id;
    int device_id;
    String turn_on;
    String turn_off;
    bool state;
    String device_name;
};

// List untuk menyimpan semua jadwal aktif
std::vector<Schedule> schedules;

// Mapping device_id ke GPIO pin untuk relay
std::map<String, int> deviceRelayPin = {
    {"solder", relay_1},  // Relay untuk device
    {"lamp 2", relay_2}   // Relay untuk device1
};

std::map<String, bool> deviceStates;

// Fungsi untuk membandingkan waktu
bool timeMatches(const String& scheduleTime, const String& currentTime) {
    // Memastikan format waktu adalah "HH:MM"
    if (scheduleTime.length() != 5 || currentTime.length() != 5) {
        return false;
    }
    
    // Membandingkan jam dan menit
    int scheduleHour = scheduleTime.substring(0, 2).toInt();
    int scheduleMinute = scheduleTime.substring(3, 5).toInt();
    int currentHour = currentTime.substring(0, 2).toInt();
    int currentMinute = currentTime.substring(3, 5).toInt();
    
    return (scheduleHour == currentHour && scheduleMinute == currentMinute);
}


String getCurrentTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        Serial.println("Failed to obtain time");
        return "00:00";
    }
    char timeStringBuff[6]; // Buffer untuk "HH:MM"
    strftime(timeStringBuff, sizeof(timeStringBuff), "%H:%M", &timeinfo);
    return String(timeStringBuff);
}

void checkSchedules() {
    String currentTime = getCurrentTime(); // Mendapatkan waktu saat ini dengan format "HH:MM"
    Serial.printf("Checking schedules at: %s\n", currentTime.c_str());

    // Debug: Menampilkan jumlah jadwal yang ada
    Serial.printf("Total schedules: %d\n", schedules.size());

    for (const auto& schedule : schedules) {
        // Debug: Menampilkan informasi setiap jadwal
        Serial.printf("Checking schedule - Device: %s, State: %d, Turn ON: %s, Turn OFF: %s\n",
                     schedule.device_name.c_str(),
                     schedule.state,
                     schedule.turn_on.c_str(),
                     schedule.turn_off.c_str());

        if (!schedule.state) {
            Serial.printf("Schedule for %s is inactive, skipping\n", schedule.device_name.c_str());
            continue;
        }

        // Cek waktu turn on
        if (timeMatches(schedule.turn_on, currentTime)) {
            Serial.printf("Turn ON time matches for device: %s\n", schedule.device_name.c_str());
            
            auto it = deviceRelayPin.find(schedule.device_name);
            if (it != deviceRelayPin.end()) {
                int relayPin = it->second;
                digitalWrite(relayPin, HIGH);
                deviceStates[schedule.device_name] = true;
                Serial.printf("Successfully turned ON relay for %s (Pin: %d)\n", 
                            schedule.device_name.c_str(), 
                            relayPin);
            } else {
                Serial.printf("ERROR: Device %s not found in relay mapping\n", 
                            schedule.device_name.c_str());
            }
        }
        
        // Cek waktu turn off
        if (timeMatches(schedule.turn_off, currentTime)) {
            Serial.printf("Turn OFF time matches for device: %s\n", schedule.device_name.c_str());
            
            auto it = deviceRelayPin.find(schedule.device_name);
            if (it != deviceRelayPin.end()) {
                int relayPin = it->second;
                digitalWrite(relayPin, LOW);
                deviceStates[schedule.device_name] = false;
                Serial.printf("Successfully turned OFF relay for %s (Pin: %d)\n", 
                            schedule.device_name.c_str(), 
                            relayPin);
            } else {
                Serial.printf("ERROR: Device %s not found in relay mapping\n", 
                            schedule.device_name.c_str());
            }
        }
    }
}



void webSocketEvent(WStype_t type, uint8_t * payload, size_t length) {
    switch(type) {
        case WStype_DISCONNECTED:
            Serial.println("WebSocket Disconnected!");
            break;
        case WStype_CONNECTED:
            Serial.println("WebSocket Connected!");
            break;
        case WStype_TEXT:
            Serial.printf("Received text: %s\n", payload);

            StaticJsonDocument<600> doc;  // Use a smaller document size
            DeserializationError error = deserializeJson(doc, payload);

            if (error) {
                Serial.print(F("JSON Parsing failed: "));
                Serial.println(error.f_str());
                return;
            }

            const char* type = doc["type"];
            if (type && strcmp(type, "dimmer_update") == 0) {
                dimmerValue = doc["value"];
                Serial.printf("Dimmer updated to: %d\n", dimmerValue);
            }

            if (type && strcmp(type, "preferences_update") == 0) {
                JsonObject data = doc["preference"];

                if (data.containsKey("automation")) {
                    automation = data["automation"];
                    Serial.printf("automation: %d\n", automation);
                }

                if (data.containsKey("auto_dimming")) {
                    auto_dimming = data["auto_dimming"];
                    Serial.printf("auto_dimming: %d\n", auto_dimming);
                }

                if (data.containsKey("email_me")) {
                    email_me = data["email_me"];
                    Serial.printf("email_me: %d\n", email_me);
                }
            }

            if (type && strcmp(type, "state") == 0) {
                // Periksa "data" yang diterima sebagai objek, bukan array
                JsonObject data = doc["data"];
                const char* device_name = data["device_name"];  // Ambil nama perangkat
                bool is_on = (data["is_on"] != 0);  // Ubah is_on menjadi boolean

                // Simpan status ke map dengan device_name sebagai key
                deviceStates[String(device_name)] = is_on;

                // Debug print untuk melihat status yang diperbarui
                Serial.printf("Device %s state updated: %d\n", device_name, is_on);

                // Kendalikan relay berdasarkan status perangkat
                if (deviceRelayPin.find(device_name) != deviceRelayPin.end()) {
                    int relayPin = deviceRelayPin[device_name];
                    digitalWrite(relayPin, is_on ? HIGH : LOW);
                    Serial.printf("Relay for %s is now %s\n", device_name, is_on ? "ON" : "OFF");
                }
            }

            // Periksa apakah tipe adalah "schedule"
            if (type && strcmp(type, "schedule") == 0) {
            // Bersihkan schedule yang ada terlebih dahulu
            schedules.clear();
            
            JsonArray schedulesArray = doc["data"].as<JsonArray>();
            for (JsonObject scheduleObj : schedulesArray) {
                Schedule newSchedule;
                newSchedule.device_name = scheduleObj["device_name"].as<String>();
                newSchedule.state = scheduleObj["state"].as<int>() == 1;
                newSchedule.turn_on = scheduleObj["turn_on"].as<String>();
                newSchedule.turn_off = scheduleObj["turn_off"].as<String>();
                
                // Debug print
                Serial.printf("Adding schedule - Device: %s, State: %d, On: %s, Off: %s\n",
                            newSchedule.device_name.c_str(),
                            newSchedule.state,
                            newSchedule.turn_on.c_str(),
                            newSchedule.turn_off.c_str());
                
                // Tambahkan schedule baru ke vector
                schedules.push_back(newSchedule);
            }
            
            // Debug print jumlah schedule yang tersimpan
            Serial.printf("Total schedules stored: %d\n", schedules.size());
        } 
          break;
        }
            
    }
