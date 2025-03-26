#include <WiFi.h>
#include <esp_now.h>
#include <BluetoothSerial.h>
#include <esp_wifi.h>

BluetoothSerial BTSerial;
bool isMaster = false;
bool masterFound = false;
uint8_t masterMAC[6] = {0};
uint8_t slaveMAC[6] = {0};
esp_now_peer_info_t peerInfo;

#define BUFFER_SIZE 32
char btBuffer[BUFFER_SIZE];
int btIndex = 0;
char serialBuffer[BUFFER_SIZE];
int serialIndex = 0;

typedef struct {
    char message[32];
    unsigned long sendTime;
    unsigned long syncTime;  // New: Synchronization timestamp
} DataPacket;

DataPacket dataPacket;
unsigned long timeOffset = 0;  // Offset correction for accurate latency

// Callback for receiving ESP-NOW data
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
    DataPacket* pkt = (DataPacket*)data;
    
    // Synchronize timestamps during startup handshake
    if (strcmp(pkt->message, "SYNC") == 0) {
        timeOffset = micros() - pkt->syncTime;
        Serial.println("Time synchronized!");
        return;
    }

    // Accurate latency calculation with synchronization offset
    unsigned long receivedTime = micros();
    unsigned long latency = (receivedTime - pkt->sendTime) - timeOffset;
    float latency_ms = latency / 1000.0; // Convert to milliseconds

    Serial.print("Received: ");
    Serial.print(pkt->message);
    Serial.print(" | Latency: ");
    Serial.print(latency_ms, 3); // Print with 3 decimal places
    Serial.println(" ms");

    BTSerial.println(pkt->message);  // Forward message to Bluetooth

    if (isMaster) {
        // Register slave MAC if received from a new device
        if (memcmp(slaveMAC, info->src_addr, 6) != 0) {
            memcpy(slaveMAC, info->src_addr, 6);

            if (!esp_now_is_peer_exist(slaveMAC)) {
                esp_now_peer_info_t peer;
                memset(&peer, 0, sizeof(peer));
                memcpy(peer.peer_addr, slaveMAC, 6);
                peer.channel = 1;
                peer.encrypt = false;

                if (esp_now_add_peer(&peer) == ESP_OK) {
                    Serial.println("Slave peer added successfully");
                } else {
                    Serial.println("Failed to add slave peer");
                }
            }
        }
    }
}

// Callback for sending ESP-NOW data
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.printf("Data sent to: %02X:%02X:%02X:%02X:%02X:%02X, Status: %s\n",
                  mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5],
                  status == ESP_NOW_SEND_SUCCESS ? "Success" : "Failed");
}

// Scan for master ESP32
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

// Register the master as a peer
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

// Initialize ESP-NOW communication
void initESPNow() {
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        ESP.restart();
    }
    esp_now_register_recv_cb(OnDataRecv);
    esp_now_register_send_cb(OnDataSent);
}

// Send data via ESP-NOW
void sendData(const char* msg) {
    strncpy(dataPacket.message, msg, sizeof(dataPacket.message) - 1);
    dataPacket.message[sizeof(dataPacket.message) - 1] = '\0';
    dataPacket.sendTime = micros() - timeOffset;

    uint8_t* targetMAC = isMaster ? slaveMAC : masterMAC;

    if (memcmp(targetMAC, "\0\0\0\0\0\0", 6) != 0) {
        esp_err_t result = esp_now_send(targetMAC, (uint8_t*)&dataPacket, sizeof(dataPacket));
        Serial.printf("Send %s\n", result == ESP_OK ? "OK" : "Fail");
        delay(10);
    } else {
        Serial.println("No valid peer to send data");
    }

    if (isMaster) {
        BTSerial.println(dataPacket.message);
    }
}

// Synchronization handshake
void syncTime() {
    strcpy(dataPacket.message, "SYNC");
    dataPacket.syncTime = micros();

    uint8_t* targetMAC = isMaster ? slaveMAC : masterMAC;
    esp_now_send(targetMAC, (uint8_t*)&dataPacket, sizeof(dataPacket));
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_AP_STA);
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_wifi_config_espnow_rate(WIFI_IF_STA, WIFI_PHY_RATE_54M);
    esp_wifi_config_espnow_rate(WIFI_IF_AP, WIFI_PHY_RATE_54M);

    scanForMaster();
    isMaster = !masterFound;

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
        syncTime();  // Synchronize time after connecting
    }
}

void loop() {
    while (BTSerial.available()) {
        char c = BTSerial.read();
        if (c == '\n' || btIndex >= BUFFER_SIZE - 1) {
            btBuffer[btIndex] = '\0';
            sendData(btBuffer);
            btIndex = 0;
        } else {
            btBuffer[btIndex++] = c;
        }
    }

    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || serialIndex >= BUFFER_SIZE - 1) {
            serialBuffer[serialIndex] = '\0';
            sendData(serialBuffer);
            serialIndex = 0;
        } else {
            serialBuffer[serialIndex++] = c;
        }
    }

    if (!isMaster && WiFi.status() != WL_CONNECTED) {
        Serial.println("Lost connection, restarting...");
        ESP.restart();
    }
}
