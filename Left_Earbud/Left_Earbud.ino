#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

// Mode configuration
bool isMaster = false;
bool masterFound = false;
uint8_t masterMAC[6] = {0};
uint8_t slaveMAC[6] = {0};
esp_now_peer_info_t peerInfo;

// Audio components
I2SStream i2s;
BluetoothA2DPSink a2dp_sink;

// ESP-NOW Data Packet Structure
typedef struct {
    uint16_t size;          // Size of valid audio data
    uint8_t audio_data[248]; // Audio data buffer
} DataPacket;

DataPacket dataPacket;

// Debugging function to print MAC address
void printMAC(const uint8_t *mac_addr) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
             mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    Serial.print(macStr);
}

// ESP-NOW Send Callback
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("ESP-NOW Send Status to ");
    printMAC(mac_addr);
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? " - Success" : " - Fail");
}

// ESP-NOW Receive Callback
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    Serial.print("Received packet from ");
    printMAC(info->src_addr);
    Serial.print(", Length: ");
    Serial.println(len);
    
    if (len < sizeof(uint16_t)) return;
    
    DataPacket* pkt = (DataPacket*)data;
    Serial.print("Audio data size: ");
    Serial.println(pkt->size);

    if (isMaster) {
        // Master receives "hello" from slave
        if (pkt->size == 0 && memcmp(pkt->audio_data, "hello", 5) == 0) {
            memcpy(slaveMAC, info->src_addr, 6);
            Serial.print("Registered slave MAC: ");
            printMAC(slaveMAC);
            Serial.println();
            
            if (!esp_now_is_peer_exist(slaveMAC)) {
                esp_now_peer_info_t peer = {};
                memcpy(peer.peer_addr, slaveMAC, 6);
                peer.channel = 1;
                peer.encrypt = false;
                if (esp_now_add_peer(&peer) == ESP_OK) {
                    Serial.println("Slave peer added successfully");
                }
            }
        }
    } else {
        // Slave plays received audio
        if (pkt->size > 0) {
            size_t bytes_written = i2s.write(pkt->audio_data, pkt->size);
            Serial.print("Slave wrote ");
            Serial.print(bytes_written);
            Serial.println(" bytes to I2S");
        }
    }
}

// Bluetooth audio callback
void read_data_stream(const uint8_t *data, uint32_t length) {
    static uint32_t totalSent = 0;
    uint32_t sent = 0;
    
    Serial.print("BT Audio received: ");
    Serial.print(length);
    Serial.println(" bytes");
    
    while (sent < length) {
        dataPacket.size = min((uint32_t)(length - sent), (uint32_t)sizeof(dataPacket.audio_data));
        memcpy(dataPacket.audio_data, data + sent, dataPacket.size);
        sent += dataPacket.size;
        
        // Play locally on master
        i2s.write(dataPacket.audio_data, dataPacket.size);

        // Send to slave if registered
        if (memcmp(slaveMAC, "\0\0\0\0\0\0", 6) != 0) {
            esp_err_t result = esp_now_send(slaveMAC, (uint8_t*)&dataPacket, sizeof(uint16_t) + dataPacket.size);
            if (result == ESP_OK) {
                totalSent += dataPacket.size;
                Serial.print("Sent to slave. Total: ");
                Serial.print(totalSent);
                Serial.println(" bytes");
            } else {
                Serial.print("Send failed. Error: ");
                Serial.println(result);
            }
        }
    }
}

void scanForMaster() {
    Serial.println("Scanning for master...");
    masterFound = false;
    WiFi.mode(WIFI_STA);
    int8_t n = WiFi.scanNetworks(false, true);
    if (n > 0) {
        for (int i = 0; i < n; i++) {
            if (WiFi.SSID(i) == "ESP32_TWS_Master") {
                WiFi.BSSID(i, masterMAC);
                masterFound = true;
                Serial.print("Found master: ");
                printMAC(masterMAC);
                Serial.println();
                break;
            }
        }
    }
    WiFi.scanDelete();
}

void registerMasterPeer() {
    if (!esp_now_is_peer_exist(masterMAC)) {
        memset(&peerInfo, 0, sizeof(peerInfo));
        memcpy(peerInfo.peer_addr, masterMAC, 6);
        peerInfo.channel = 1;
        peerInfo.encrypt = false;
        if (esp_now_add_peer(&peerInfo) == ESP_OK) {
            Serial.print("Registered master peer: ");
            printMAC(masterMAC);
            Serial.println();
        }
    }
}

void initESPNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        ESP.restart();
    }
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);
    Serial.println("ESP-NOW initialized");
}

void setup() {
    Serial.begin(115200);
    Serial.println("\nStarting TWS System...");

    // Configure I2S
    auto cfg = i2s.defaultConfig();
    cfg.sample_rate = 44100;
    cfg.bits_per_sample = 16;
    cfg.channels = 2;
    cfg.pin_bck = 27;
    cfg.pin_ws = 26;
    cfg.pin_data = 25;
    i2s.begin(cfg);
    Serial.println("I2S initialized");

    // Initialize WiFi and ESP-NOW
    WiFi.mode(WIFI_AP_STA);
    esp_wifi_set_ps(WIFI_PS_NONE);
    
    scanForMaster();
    isMaster = !masterFound;

    if (isMaster) {
        Serial.println("Running as MASTER");
        WiFi.softAP("ESP32_TWS_Master", nullptr, 1);
        Serial.println("Master AP started");
        
        a2dp_sink.set_stream_reader(read_data_stream, false);
        a2dp_sink.start("MySpeaker");
        Serial.println("A2DP Sink started");
    } else {
        Serial.println("Running as SLAVE");
        WiFi.begin("ESP32_TWS_Master", nullptr, 1);
        
        Serial.print("Connecting to master AP");
        for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
            delay(250);
            Serial.print(".");
        }
        Serial.println("\nConnected to master AP");
        
        registerMasterPeer();
        
        // Send hello to master
        DataPacket packet = {0};
        memcpy(packet.audio_data, "hello", 5);
        esp_now_send(masterMAC, (uint8_t*)&packet, sizeof(packet));
        Serial.println("Sent hello to master");
    }

    initESPNow();
    Serial.println("Setup complete");
}

void loop() {
    if (!isMaster && WiFi.status() != WL_CONNECTED) {
        Serial.println("Connection lost, restarting...");
        ESP.restart();
    }
    delay(100);
}