#include "driver/i2s_std.h"
#include "driver/gpio.h"

#define I2S_SCK  2   // BCLK
#define I2S_WS   3   // LRCLK / WS
#define I2S_SD   4

i2s_chan_handle_t rx_handle;

void setup()
{
    Serial.begin(115200);
    delay(1500);

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    esp_err_t e1 = i2s_new_channel(&chan_cfg, NULL, &rx_handle);

    i2s_std_config_t std_cfg = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(16000),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = (gpio_num_t)I2S_SCK,
            .ws   = (gpio_num_t)I2S_WS,
            .dout = I2S_GPIO_UNUSED,
            .din  = (gpio_num_t)I2S_SD,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

    esp_err_t e2 = i2s_channel_init_std_mode(rx_handle, &std_cfg);
    esp_err_t e3 = i2s_channel_enable(rx_handle);

    Serial.printf("new_channel=%s  init_std=%s  enable=%s\n",
        e1 == ESP_OK ? "OK" : esp_err_to_name(e1),
        e2 == ESP_OK ? "OK" : esp_err_to_name(e2),
        e3 == ESP_OK ? "OK" : esp_err_to_name(e3));
    Serial.println("INMP441 Ready (new I2S driver)");
}

void loop()
{
    int32_t buf[128];
    size_t bytesRead = 0;

    esp_err_t r = i2s_channel_read(rx_handle, buf, sizeof(buf), &bytesRead, portMAX_DELAY);

    int64_t sum = 0;
    int samples = bytesRead / sizeof(int32_t);

    for (int i = 0; i < samples; i++)
    {
        int32_t sample = buf[i] >> 8;   // 32-bit -> 24-bit signed
        sum += abs(sample);
    }

    int level = samples > 0 ? (sum / samples) : 0;

    if (r != ESP_OK) {
        Serial.printf("read error: %s\n", esp_err_to_name(r));
    }
    Serial.println(level);

    delay(200);
}
