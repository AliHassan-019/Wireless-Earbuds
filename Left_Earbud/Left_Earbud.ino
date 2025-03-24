#include <WiFi.h>
#include <esp_now.h>
#include <BluetoothSerial.h>
#include <esp_wifi.h>
#include <driver/i2s.h>

BluetoothSerial BTSerial;
bool isMaster = false;
bool masterFound = false;
uint8_t masterMAC[6] = {0};
uint8_t slaveMAC[6] = {0};
esp_now_peer_info_t peerInfo;

// I2S Configuration
#define I2S_MIC_SD 32
#define I2S_MIC_WS 25
#define I2S_MIC_SCK 26

#define I2S_SPK_SD 27
#define I2S_SPK_WS 14
#define I2S_SPK_SCK 15

#define SAMPLE_RATE 16000
#define BUFFER_SIZE 512

// Audio buffers
int16_t micBuffer[BUFFER_SIZE];
int16_t spkBuffer[BUFFER_SIZE];

#define BUFFER_SIZE_32 32
char btBuffer[BUFFER_SIZE_32];
int btIndex = 0;
char serialBuffer[BUFFER_SIZE_32];
int serialIndex = 0;

typedef struct {
    char type;  // 'M' for message, 'A' for audio
    char data[31];
    unsigned long sendTime;
} DataPacket;

DataPacket dataPacket;

void setupI2S() {
    // Configure I2S for microphone (input)
    i2s_config_t mic_i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,
        .dma_buf_len = 64,
        .use_apll = false
    };

    i2s_pin_config_t mic_pin_config = {
        .bck_io_num = I2S_MIC_SCK,
        .ws_io_num = I2S_MIC_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_MIC_SD
    };

    if (i2s_driver_install(I2S_NUM_0, &mic_i2s_config, 0, NULL) != ESP_OK) {
        Serial.println("Failed to install I2S microphone driver");
        return;
    }
    if (i2s_set_pin(I2S_NUM_0, &mic_pin_config) != ESP_OK) {
        Serial.println("Failed to set I2S microphone pins");
        return;
    }

    // Configure I2S for speaker (output)
    i2s_config_t spk_i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_STAND_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 4,
        .dma_buf_len = 64,
        .use_apll = false
    };

    i2s_pin_config_t spk_pin_config = {
        .bck_io_num = I2S_SPK_SCK,
        .ws_io_num = I2S_SPK_WS,
        .data_out_num = I2S_SPK_SD,
        .data_in_num = I2S_PIN_NO_CHANGE
    };

    if (i2s_driver_install(I2S_NUM_1, &spk_i2s_config, 0, NULL) != ESP_OK) {
        Serial.println("Failed to install I2S speaker driver");
        return;
    }
    if (i2s_set_pin(I2S_NUM_1, &spk_pin_config) != ESP_OK) {
        Serial.println("Failed to set I2S speaker pins");
        return;
    }

    Serial.println("I2S initialized successfully");
}

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    DataPacket* pkt = (DataPacket*)data;
    
    if (pkt->type == 'A') {  // Audio data
        // Play received audio on speaker
        size_t bytes_written;
        if (i2s_write(I2S_NUM_1, pkt->data, sizeof(pkt->data), &bytes_written, portMAX_DELAY) != ESP_OK) {
            Serial.println("Failed to write audio to speaker");
        }
        
        // Forward to Bluetooth if master
        if (isMaster) {
            BTSerial.write((uint8_t*)pkt, len);
        }
    } else {  // Regular message
        Serial.print("Received: ");
        Serial.println(pkt->data);
        BTSerial.println(pkt->data);

        if (isMaster) {
            if (memcmp(slaveMAC, info->src_addr, 6) != 0) {
                memcpy(slaveMAC, info->src_addr, 6);
                if (!esp_now_is_peer_exist(slaveMAC)) {
                    esp_now_peer_info_t peer;
                    memset(&peer, 0, sizeof(peer));
                    memcpy(peer.peer_addr, slaveMAC, 6);
                    peer.channel = 1;
                    peer.encrypt = false;

                    if (esp_now_add_peer(&peer) == ESP_OK) {
                        Serial.println("Slave peer added");
                    }
                }
            }
        }
    }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.printf("Data sent to: %02X:%02X:%02X:%02X:%02X:%02X, Status: %s\n",
                  mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
                  status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}
void scanForMaster() {
    const int maxRetries = 3;
    const int scanChannel = 1;
    masterFound = false;

    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(scanChannel, WIFI_SECOND_CHAN_NONE);

    for (int retry = 0; retry < maxRetries && !masterFound; retry++) {
        Serial.printf("Scanning (Attempt %d/%d)\n", retry+1, maxRetries);
        int8_t n = WiFi.scanNetworks(false, true, false, 100, scanChannel);

        if (n > 0) {
            for (int i = 0; i < n; i++) {
                if (WiFi.SSID(i) == "ESP32_TWS_Master") {
                    WiFi.BSSID(i, masterMAC);
                    masterFound = true;
                    Serial.printf("Master found on channel %d\n", scanChannel);
                    break;
                }
            }
        }
        WiFi.scanDelete();
    }
}
void registerMasterPeer() {
    if (!esp_now_is_peer_exist(masterMAC)) {
        memset(&peerInfo, 0, sizeof(peerInfo));
        memcpy(peerInfo.peer_addr, masterMAC, 6);
        peerInfo.channel = 1;
        peerInfo.encrypt = false;

        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("Failed to add master peer");
        } else {
            Serial.println("Master peer added successfully");
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
}
void sendAudioPacket() {
    static unsigned long lastMicRead = 0;
    if (micros() - lastMicRead < 1000000 / (SAMPLE_RATE / (BUFFER_SIZE_32 / 2))) return;

    size_t bytesRead = 0;
    if (i2s_read(I2S_NUM_0, micBuffer, BUFFER_SIZE_32, &bytesRead, 0) != ESP_OK) {
        Serial.println("Failed to read from microphone");
        return;
    }

    if (bytesRead > 0) {
        DataPacket audioPacket;
        audioPacket.type = 'A';
        memcpy(audioPacket.data, micBuffer, bytesRead);
        audioPacket.sendTime = micros();

        uint8_t* targetMAC = isMaster ? slaveMAC : masterMAC;
        if (memcmp(targetMAC, "\0\0\0\0\0\0", 6) != 0) {
            esp_now_send(targetMAC, (uint8_t*)&audioPacket, sizeof(audioPacket));
        }

        // If slave, also send to master via ESP-NOW
        if (!isMaster && masterFound) {
            esp_now_send(masterMAC, (uint8_t*)&audioPacket, sizeof(audioPacket));
        }
    }
    lastMicRead = micros();
}

void processBluetoothAudio() {
    static uint8_t btAudioBuffer[BUFFER_SIZE_32];
    static int btAudioIndex = 0;
    
    while (BTSerial.available()) {
        btAudioBuffer[btAudioIndex++] = BTSerial.read();
        
        if (btAudioIndex == sizeof(DataPacket)) {
            DataPacket* pkt = (DataPacket*)btAudioBuffer;
            if (pkt->type == 'A') {
                // Play received audio
                size_t bytes_written;
                if (i2s_write(I2S_NUM_1, pkt->data, sizeof(pkt->data), &bytes_written, portMAX_DELAY) != ESP_OK) {
                    Serial.println("Failed to write audio to speaker");
                }
                
                // Forward to slave if master
                if (isMaster) {
                    esp_now_send(slaveMAC, (uint8_t*)pkt, sizeof(DataPacket));
                }
            }
            btAudioIndex = 0;
        }
    }
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_AP_STA);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M);
    esp_wifi_config_espnow_rate(WIFI_IF_AP, WIFI_PHY_RATE_54M);

    scanForMaster();
    isMaster = !masterFound;
    setupI2S(); // Initialize I2S after Bluetooth

    if (isMaster) {
        WiFi.softAP("ESP32_TWS_Master", nullptr, 1);
        Serial.println("Mode: MASTER");
        BTSerial.begin("ESP32_Master");
    } else {
        WiFi.begin("ESP32_TWS_Master", nullptr, 1);
        Serial.print("Connecting");
        for (int i = 0; i < 20 && WiFi.status() != WL_CONNECTED; i++) {
            delay(250);
            Serial.print(".");
        }

        if (WiFi.status() == WL_CONNECTED) {
            BTSerial.begin("ESP32_Slave");
            Serial.println("\nConnected");
        } else {
            Serial.println("\nFailed to connect, restarting...");
            ESP.restart();
        }
    }

    initESPNow();
    if (!isMaster) {
        registerMasterPeer();
        delay(500);
    }

}

void loop() {
    // Existing loop code
    sendAudioPacket();
    processBluetoothAudio();
}