#include "BluetoothA2DPSource.h"
#include "driver/i2s.h"

BluetoothA2DPSource a2dp_source;

// I2S Configuration (Modify pins as needed)
#define I2S_WS   25  // Word Select (L/R clock)
#define I2S_BCLK 26  // Bit Clock
#define I2S_DIN  22  // Data Input from MIC

#define SAMPLE_RATE 16000  // 16kHz sampling rate
#define I2S_PORT I2S_NUM_0

void setupI2S() {
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, 
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 4,
        .dma_buf_len = 512,
        .use_apll = false
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = I2S_BCLK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_DIN
    };

    i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_PORT, &pin_config);
}

// Function to read I2S microphone audio
void audio_data_generator(int16_t *samples, int sample_count) {
    size_t bytes_read;
    i2s_read(I2S_PORT, samples, sample_count * sizeof(int16_t), &bytes_read, portMAX_DELAY);
}

void setup() {
    Serial.begin(115200);
    setupI2S();

    // Start Bluetooth Audio Source
    a2dp_source.start("ESP32_Audio_Source");
    a2dp_source.set_data_callback(audio_data_generator);  // ✅ Use set_data_callback() instead of set_stream_reader()
}

void loop() {
    // The A2DP library automatically handles audio streaming
}
