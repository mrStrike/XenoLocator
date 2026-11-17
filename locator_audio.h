#pragma once
// ESP-IDF 5+ uses the channel API; older ESP32 cores use the legacy API.
#if __has_include(<driver/i2s_std.h>)
#include <driver/i2s_std.h>
#define LOCATOR_MODERN_I2S 1
static i2s_chan_handle_t locatorTx = nullptr;
#else
#include <driver/i2s.h>
#define LOCATOR_MODERN_I2S 0
#endif
#include <atomic>
static std::atomic<int> pingPeriod {0};
// Ten amplitude steps; zero mutes without changing the scan cadence.
static std::atomic<int> pingVolume {5};

#include <AudioFileSourcePROGMEM.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutput.h>
#include "locator_music_data.h"
static QueueHandle_t musicQueue;
// Precomputed once: no trigonometry or exponentials in the audio writer.
static int16_t sonarWave[12480];
static void prepareSonar() {
 for (int i=0; i<12480; ++i) {
  const float t=i/48000.f;
  const float attack=fminf(1.f,t/.004f);
  const float release=fminf(1.f,(.260f-t)/.025f);
  float direct=attack*expf(-t*23.f)*sinf(2.f*PI*1450.f*(i+1)/48000.f);
  float echo=0;
  if(t>=.085f) {
   float e=t-.085f;
   echo=.28f*fminf(1.f,e/.006f)*expf(-e*25.f)*sinf(2.f*PI*1450.f*e);
  }
  sonarWave[i]=int16_t(1800.f*release*(direct+echo));
 }
}
static std::atomic<int> musicRate {44100};
class MusicOutput : public AudioOutput {
public:
 bool begin() override {return true;}
 bool stop() override {return true;}
 bool SetRate(int hz) override {musicRate=hz; return AudioOutput::SetRate(hz);}
 bool ConsumeSample(int16_t s[2]) override {
  int16_t mono=(int32_t(s[0])+s[1])/2;
  return xQueueSend(musicQueue,&mono,pdMS_TO_TICKS(20))==pdTRUE;
 }
};
static void musicTask(void*) {
 AudioFileSourcePROGMEM source(locatorMusic,sizeof(locatorMusic));
 MusicOutput output;
 AudioGeneratorMP3 decoder;
 for (;;) {
  source.open(locatorMusic,sizeof(locatorMusic));
  if (!decoder.begin(&source,&output)) {vTaskDelete(nullptr); return;}
  while(decoder.isRunning()) {
   if(!decoder.loop()) decoder.stop();
   vTaskDelay(1);
  }
 }
}

static bool codecWrite(uint8_t r, uint8_t v) {
    Wire.beginTransmission(0x18);
    Wire.write(r);
    Wire.write(v);
    return Wire.endTransmission() == 0;
}
// Preserve the user's tested codec initialization tables exactly.

static bool es8311_init() {
    const uint8_t seq[][2] = {
        {0x01, 0x30},
        {0x02, 0},
        {0x03, 0x10},
        {0x16, 0x24},
        {0x04, 0x10},
        {0x05, 0},
        {0x0b, 0},
        {0x0c, 0},
        {0x10, 0x1f},
        {0x11, 0x7f},
        {0x00, 0x80}
    };
    for (auto &r : seq) if (!codecWrite(r[0], r[1])) return false;
    delay(20);
    const uint8_t run[][2] = {
        {0x00, 0},
        {0x01, 0x3f},
        {0x02, 0},
        {0x03, 0x10},
        {0x04, 0x10},
        {0x05, 0},
        {0x06, 3},
        {0x07, 0},
        {0x08, 0xff},
        {0x09, 0x0c},
        {0x0a, 0x4c},
        {0x13, 0x10},
        {0x1b, 0x0a},
        {0x1c, 0x6a},
        {0x0e, 2},
        {0x12, 0},
        {0x14, 0x10},
        {0x0d, 1},
        {0x15, 0},
        {0x37, 0x48},
        {0x45, 0},
        {0x31, 0},
        {0x32, 0xbf}
    };
    for (auto &r : run) if (!codecWrite(r[0], r[1])) return false;
    if (!codecWrite(0x00, 0x80)) return false;
    delay(20);
    return true;
}

static void pingTask(void*) {
    uint32_t sample = 0, start = 0;
    bool active = false;

    int16_t block[256 * 2];
    uint32_t musicPhase=48000;
    int16_t musicSample=0;
    // Start with a reserve rather than an empty music queue.
    uint32_t waitingSince=millis();
    while(uxQueueMessagesWaiting(musicQueue)<4096 && millis()-waitingSince<2000)
        vTaskDelay(1);
    for (;;) {
        int period = pingPeriod.load();
        const float gain = pingVolume.load() / 5.0f;
        for (int i = 0; i < 256; i++, sample++) {
            if (period > 0 && (!active || sample - start >= uint32_t(period * 48))) {
                start = sample;
                active = true;

            }
            uint32_t age = sample - start;
            float v = 0;
            if (period > 0 && active && age < 12480) {
                v = sonarWave[age];
            }
            if (period == 0) active = false;
            while(musicPhase>=48000) {
                if(xQueueReceive(musicQueue,&musicSample,0)!=pdTRUE) musicSample=0;
                musicPhase-=48000;
            }
            musicPhase+=musicRate.load();
            float mix=(v+musicSample*.06f)*gain;
            block[2*i]=block[2*i+1]=int16_t(fmaxf(-32768.f,fminf(32767.f,mix)));
        }
        size_t written = 0;
#if LOCATOR_MODERN_I2S
        esp_err_t result = i2s_channel_write(
            locatorTx, block, sizeof(block), &written, portMAX_DELAY);
#else
        esp_err_t result = i2s_write(
            I2S_NUM_0, block, sizeof(block), &written, portMAX_DELAY);
#endif
        if (result != ESP_OK || written != sizeof(block)) {
            digitalWrite(1, HIGH);
            vTaskDelete(nullptr);
        }
    }
}

static bool beginLocatorAudio() {
#if LOCATOR_MODERN_I2S
    i2s_chan_config_t channel = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    channel.dma_desc_num = 6;
    channel.dma_frame_num = 256;
    if (i2s_new_channel(&channel, &locatorTx, nullptr) != ESP_OK) return false;

    i2s_std_config_t standard = {};
    standard.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(48000);
    standard.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
        I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO);
    standard.gpio_cfg.mclk = GPIO_NUM_4;
    standard.gpio_cfg.bclk = GPIO_NUM_5;
    standard.gpio_cfg.ws = GPIO_NUM_7;
    standard.gpio_cfg.dout = GPIO_NUM_8;
    standard.gpio_cfg.din = I2S_GPIO_UNUSED;
    if (i2s_channel_init_std_mode(locatorTx, &standard) != ESP_OK) return false;
    if (i2s_channel_enable(locatorTx) != ESP_OK) return false;
#else
    i2s_config_t cfg = {
    };
    cfg.mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_TX);
    cfg.sample_rate = 48000;
    cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
    cfg.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
    cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
    cfg.dma_buf_count = 6;
    cfg.dma_buf_len = 256;
    cfg.use_apll = false;
    cfg.tx_desc_auto_clear = true;
    cfg.mclk_multiple = I2S_MCLK_MULTIPLE_256;
    if (i2s_driver_install(I2S_NUM_0, & cfg, 0, nullptr) != ESP_OK) return false;
    i2s_pin_config_t pins = {
    };
    pins.mck_io_num = 4;
    pins.bck_io_num = 5;
    pins.ws_io_num = 7;
    pins.data_out_num = 8;
    pins.data_in_num = I2S_PIN_NO_CHANGE;
    if (i2s_set_pin(I2S_NUM_0, & pins) != ESP_OK) return false;
    i2s_zero_dma_buffer(I2S_NUM_0);
#endif
    if (!es8311_init()) return false;
    prepareSonar();
    musicQueue=xQueueCreate(8192,sizeof(int16_t));
    if(!musicQueue) return false;
    if(xTaskCreatePinnedToCore(musicTask,"music",16384,nullptr,1,nullptr,0)!=pdPASS) return false;
    if (xTaskCreatePinnedToCore(pingTask, "locator_ping", 4096, nullptr, 2, nullptr, 0) != pdPASS) return false;
    digitalWrite(1, LOW);
    return true;
}
