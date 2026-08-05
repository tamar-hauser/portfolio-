#include "driver/i2s_std.h"

#define I2S_PORT       I2S_NUM_0
#define I2S_PIN_BCLK   27
#define I2S_PIN_WS     25
#define I2S_PIN_DOUT   26

#define SAMPLE_RATE_HZ 16000
#define TONE_HZ        440

i2s_chan_handle_t txHandle = NULL;

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("בדיקת חומרה: מגדירה I2S ומתחילה לנגן צליל 440Hz ברציפות...");

  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
  esp_err_t e1 = i2s_new_channel(&chanCfg, &txHandle, NULL);

  i2s_std_config_t stdCfg = {
    .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ),
    .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
    .gpio_cfg = {
      .mclk = I2S_GPIO_UNUSED,
      .bclk = (gpio_num_t)I2S_PIN_BCLK,
      .ws   = (gpio_num_t)I2S_PIN_WS,
      .dout = (gpio_num_t)I2S_PIN_DOUT,
      .din  = I2S_GPIO_UNUSED,
      .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
    },
  };
  stdCfg.slot_cfg.slot_mask = I2S_STD_SLOT_LEFT;

  esp_err_t e2 = i2s_channel_init_std_mode(txHandle, &stdCfg);
  esp_err_t e3 = i2s_channel_enable(txHandle);

  Serial.printf("תוצאות הגדרת I2S: new_channel=%s init_std=%s enable=%s\n",
    esp_err_to_name(e1), esp_err_to_name(e2), esp_err_to_name(e3));
  if (e1 != ESP_OK || e2 != ESP_OK || e3 != ESP_OK) {
    Serial.println(">>> ההגדרה נכשלה! זו הסיבה שאין קול - בדקי התנגשות פינים או פינים לא נתמכים.");
  } else {
    Serial.println(">>> ההגדרה הצליחה. אם עדיין אין קול - הבעיה בוודאות בחיווט הפיזי (חיבור/מגבר/רמקול).");
  }
}

void loop() {
  static float phase = 0;
  int16_t buffer[256];
  for (int i = 0; i < 256; i++) {
    buffer[i] = (int16_t)(24000 * sinf(phase));
    phase += 2.0f * PI * TONE_HZ / SAMPLE_RATE_HZ;
    if (phase > 2.0f * PI) phase -= 2.0f * PI;
  }
  size_t written;
  i2s_channel_write(txHandle, buffer, sizeof(buffer), &written, portMAX_DELAY);
}
