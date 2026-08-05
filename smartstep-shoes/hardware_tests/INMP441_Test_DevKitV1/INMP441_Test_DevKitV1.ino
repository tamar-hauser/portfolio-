#include <driver/i2s.h>

#define I2S_SCK  26
#define I2S_WS   25
#define I2S_SD   33
#define I2S_PORT I2S_NUM_0

void setup()
{
    Serial.begin(115200);
    delay(1500);

    i2s_config_t i2s_config =
    {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 16000,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
        .dma_buf_count = 8,
        .dma_buf_len = 128,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config =
    {
        .bck_io_num = I2S_SCK,
        .ws_io_num = I2S_WS,
        .data_out_num = I2S_PIN_NO_CHANGE,
        .data_in_num = I2S_SD
    };

    esp_err_t e1 = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
    esp_err_t e2 = i2s_set_pin(I2S_PORT, &pin_config);

    Serial.print("driver_install="); Serial.print(e1 == ESP_OK ? "OK" : String(e1));
    Serial.print("  set_pin="); Serial.println(e2 == ESP_OK ? "OK" : String(e2));
    Serial.println("INMP441 Ready (DevKit V1)");
}

void loop()
{
    int64_t sum = 0;

    for (int i = 0; i < 128; i++)
    {
        int32_t sample;
        size_t bytesRead;
        i2s_read(I2S_PORT, &sample, sizeof(sample), &bytesRead, portMAX_DELAY);
        sample >>= 8;
        sum += abs(sample);
    }

    int level = sum / 128;
    Serial.println(level);
    delay(200);
}
