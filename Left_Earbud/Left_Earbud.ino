#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "AudioTools.h"
#include "BluetoothA2DPSink.h"

bool isMaster = false;
bool masterFound = false;
uint8_t masterMAC[6] = {0};
uint8_t slaveMAC[6] = {0};
esp_now_peer_info_t peerInfo;

I2SStream i2s;
BluetoothA2DPSink a2dp_sink;

// ESP-NOW Data Packet Structure
typedef struct {
    uint16_t size;          // Size of valid audio data in the packet
    uint8_t audio_data[248]; // Audio data buffer (max 248 bytes)
} DataPacket;

DataPacket dataPacket;

// Callback for receiving ESP-NOW data
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    DataPacket* pkt = (DataPacket*)data;
    if (isMaster) {
        // If master receives "hello" from slave, register slave's MAC
        if (pkt->size == 0 && memcmp(pkt->audio_data, "hello", 5) == 0) {
            memcpy(slaveMAC, info->src_addr, 6);
            if (!esp_now_is_peer_exist(slaveMAC)) {
                esp_now_peer_info_t peer;
                memset(&peer, 0, sizeof(peer));
                memcpy(peer.peer_addr, slaveMAC, 6);
                peer.channel = 1;
                peer.encrypt = false;
                esp_now_add_peer(&peer);
            }
        }
    } else {
        // Slave writes received audio data to I2S
        i2s.write(pkt->audio_data, pkt->size);
    }
}

// Bluetooth audio data callback
void read_data_stream(const uint8_t *data, uint32_t length) {
    uint32_t sent = 0;
    while (sent < length) {
        dataPacket.size = min((uint32_t)(length - sent), (uint32_t)sizeof(dataPacket.audio_data));
        memcpy(dataPacket.audio_data, data + sent, dataPacket.size);
        sent += dataPacket.size;
        if (memcmp(slaveMAC, "\0\0\0\0\0\0", 6) != 0) {
            esp_err_t result = esp_now_send(slaveMAC, (uint8_t*)&dataPacket, sizeof(dataPacket));
            if (result != ESP_OK) {
                // Handle transmission error
            }
        }
        // Play audio locally on master
        i2s.write(dataPacket.audio_data, dataPacket.size);
    }
}

void scanForMaster() {
    const int scanChannel = 1;
    masterFound = false;
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(scanChannel, WIFI_SECOND_CHAN_NONE);
    int8_t n = WiFi.scanNetworks(false, true, false, 100, scanChannel);
    if (n > 0) {
        for (int i = 0; i < n; i++) {
            if (WiFi.SSID(i) == "ESP32_TWS_Master") {
                WiFi.BSSID(i, masterMAC);
                masterFound = true;
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
        esp_now_add_peer(&peerInfo);
    }
}

void initESPNow() {
    if (esp_now_init() != ESP_OK) {
        ESP.restart();
    }
    esp_now_register_recv_cb(OnDataRecv);
}

void setup() {
    // Configure I2S for both devices
    auto cfg = i2s.defaultConfig();
    cfg.pin_bck = 27; // Adjust pins according to your setup
    cfg.pin_ws = 26;
    cfg.pin_data = 25;
    i2s.begin(cfg);

    // Initialize WiFi and ESP-NOW
    WiFi.mode(WIFI_AP_STA);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M);
    esp_wifi_config_espnow_rate(WIFI_IF_AP, WIFI_PHY_RATE_54M);
    
    scanForMaster();
    isMaster = !masterFound;

    if (isMaster) {
        WiFi.softAP("ESP32_TWS_Master", nullptr, 1);
        // Start Bluetooth A2DP sink
        a2dp_sink.set_stream_reader(read_data_stream, false);
        a2dp_sink.start("MySpeaker");
    } else {
        WiFi.begin("ESP32_TWS_Master", nullptr, 1);
        for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
            delay(250);
        }
        if (WiFi.status() != WL_CONNECTED) {
            ESP.restart();
        }
        // Send "hello" to master to register as peer
        registerMasterPeer();
        DataPacket packet;
        packet.size = 0;
        memcpy(packet.audio_data, "hello", 5);
        esp_now_send(masterMAC, (uint8_t*)&packet, sizeof(packet));
    }

    initESPNow();
}

void loop() {
    if (!isMaster && WiFi.status() != WL_CONNECTED) {
        ESP.restart();
    }
    delay(100); // Reduce CPU usage
}