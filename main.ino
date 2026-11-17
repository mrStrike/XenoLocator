#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <Adafruit_GFX.h>
#include <LovyanGFX.hpp>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "locator_audio.h"

bool audioReady = false;
class LGFX : public lgfx::LGFX_Device {
    lgfx::Bus_SPI bus;
    lgfx::Panel_ILI9341 panel;
    public:
    LGFX() {
        auto b = bus.config();
        b.spi_host = SPI2_HOST;
        b.freq_write = 40000000;
        b.freq_read = 16000000;
        b.spi_3wire = false;
        b.use_lock = true;
        b.dma_channel = SPI_DMA_CH_AUTO;
        b.pin_sclk = 12;
        b.pin_mosi = 11;
        b.pin_miso = 13;
        b.pin_dc = 46;
        bus.config(b);
        panel.setBus(&bus);
        auto p = panel.config();
        p.pin_cs = 10;
        p.pin_rst = -1;
        p.pin_busy = -1;
        p.memory_width = p.panel_width = 240;
        p.memory_height = p.panel_height = 320;
        p.invert = true;
        panel.config(p);
        setPanel(&panel);
    }
};
LGFX lcd;
GFXcanvas16 ui(240, 320);
struct AP {
    String name;
    uint8_t mac[6];
    int channel, rssi;
};
AP aps[30];
struct NearbyContact { uint8_t mac[6]; int rssi; };
NearbyContact nearby[30];
int nearbyCount = 0;
int count = 0, page = 0, state = 0, corner = 0;
uint8_t target[6];
int channel = 1;
bool targetSeen = false, havePrevious = false;
float previousRSSI = 0;
bool dirty = true, scanning = false, listScanning = false;
unsigned long nextScan = 0;
String selected = "", trend = "ОЖИДАНИЕ СИГНАЛА";
float meanRSSI = 0, delta = 0;
int good = 0;
int pulseFrame = -1;
const uint16_t BG = 0x0020, FG = 0x87f0, ACC = 0x07e0, DIM = 0x1244, AMBER = 0xfd60;
// Cyrillic capitals, top-to-bottom rows and left-to-right pixels.
// '#' is a lit pixel; '.' is empty. This is NOT a column-packed GFX font.
static const char ru[33][7][6] = {
    {".###.", "#...#", "#...#", "#####", "#...#", "#...#", "#...#"}, // А
    {"#####", "#....", "#....", "####.", "#...#", "#...#", "####."}, // Б
    {"####.", "#...#", "#...#", "####.", "#...#", "#...#", "####."}, // В
    {"#####", "#....", "#....", "#....", "#....", "#....", "#...."}, // Г
    {"..###", ".#..#", ".#..#", ".#..#", ".#..#", "#####", "#...#"}, // Д
    {"#####", "#....", "#....", "####.", "#....", "#....", "#####"}, // Е
    {"#.#.#", "#.#.#", "#.#.#", ".###.", "#.#.#", "#.#.#", "#.#.#"}, // Ж
    {".###.", "#...#", "....#", "..##.", "....#", "#...#", ".###."}, // З
    {"#...#", "#...#", "#..##", "#.#.#", "##..#", "#...#", "#...#"}, // И
    {".#.#.", "..#..", "#...#", "#..##", "#.#.#", "##..#", "#...#"}, // Й
    {"#...#", "#..#.", "#.#..", "##...", "#.#..", "#..#.", "#...#"}, // К
    {"..###", ".#..#", ".#..#", ".#..#", ".#..#", ".#..#", "#...#"}, // Л
    {"#...#", "##.##", "#.#.#", "#.#.#", "#...#", "#...#", "#...#"}, // М
    {"#...#", "#...#", "#...#", "#####", "#...#", "#...#", "#...#"}, // Н
    {".###.", "#...#", "#...#", "#...#", "#...#", "#...#", ".###."}, // О
    {"#####", "#...#", "#...#", "#...#", "#...#", "#...#", "#...#"}, // П
    {"####.", "#...#", "#...#", "####.", "#....", "#....", "#...."}, // Р
    {".###.", "#...#", "#....", "#....", "#....", "#...#", ".###."}, // С
    {"#####", "..#..", "..#..", "..#..", "..#..", "..#..", "..#.."}, // Т
    {"#...#", "#...#", "#...#", ".####", "....#", "#...#", ".###."}, // У
    {"..#..", ".###.", "#.#.#", "#.#.#", "#.#.#", ".###.", "..#.."}, // Ф
    {"#...#", "#...#", ".#.#.", "..#..", ".#.#.", "#...#", "#...#"}, // Х
    {"#..#.", "#..#.", "#..#.", "#..#.", "#..#.", "#####", "....#"}, // Ц
    {"#...#", "#...#", "#...#", ".####", "....#", "....#", "....#"}, // Ч
    {"#.#.#", "#.#.#", "#.#.#", "#.#.#", "#.#.#", "#.#.#", "#####"}, // Ш
    {"#.#.#", "#.#.#", "#.#.#", "#.#.#", "#.#.#", "#####", "....#"}, // Щ
    {"##...", ".#...", ".#...", ".###.", ".#..#", ".#..#", ".###."}, // Ъ
    {"#...#", "#...#", "#...#", "##..#", "#.#.#", "#.#.#", "##..#"}, // Ы
    {"#....", "#....", "#....", "####.", "#...#", "#...#", "####."}, // Ь
    {".###.", "#...#", "....#", "..###", "....#", "#...#", ".###."}, // Э
    {"#..#.", "#.#.#", "#.#.#", "###.#", "#.#.#", "#.#.#", "#..#."}, // Ю
    {".####", "#...#", "#...#", ".####", "..#.#", ".#..#", "#...#"}, // Я
    {".#.#.", ".....", "#####", "#....", "####.", "#....", "#####"}, // Ё
};

// Decode a complete UTF-8 character, including non-Cyrillic network names.
static uint32_t nextCodepoint(const String &s, unsigned &offset) {
    uint8_t first = static_cast<uint8_t>(s[offset++]);
    if (first < 0x80) return first;
    int remaining;
    uint32_t codepoint;
    if ((first & 0xE0) == 0xC0) {
        remaining = 1;
        codepoint = first & 0x1F;
    } else if ((first & 0xF0) == 0xE0) {
        remaining = 2;
        codepoint = first & 0x0F;
    } else if ((first & 0xF8) == 0xF0) {
        remaining = 3;
        codepoint = first & 0x07;
    } else {
        return '?';
    }
    while (remaining--) {
        if (offset >= s.length()) return '?';
        uint8_t continuation = static_cast<uint8_t>(s[offset]);
        if ((continuation & 0xC0) != 0x80) return '?';
        ++offset;
        codepoint = (codepoint << 6) | (continuation & 0x3F);
    }
    return codepoint;
}

void text(int x, int y, String s, int size = 1, uint16_t color = FG) {
    ui.setTextWrap(false);
    ui.setTextSize(size);
    ui.setTextColor(color);

    for (unsigned offset = 0; offset < s.length();) {
        uint32_t c = nextCodepoint(s, offset);
        // The locator uses uppercase lettering, including Cyrillic SSIDs.
        if (c >= 0x430 && c <= 0x44F) c -= 32;
        if (c == 0x451) c = 0x401;
        if (x + 5 * size > 234) break;

        int glyph = -1;
        if (c >= 0x410 && c <= 0x42F) glyph = c - 0x410;
        if (c == 0x401) glyph = 32;

        if (glyph >= 0) {
            for (int row = 0; row < 7; ++row) {
                for (int column = 0; column < 5; ++column) {
                    if (ru[glyph][row][column] == '#') {
                        ui.fillRect(x + column * size, y + row * size,
                                    size, size, color);
                    }
                }
            }
        } else {
            ui.setCursor(x, y);
            ui.write(c < 128 ? c : '?');
        }
        x += 6 * size;
    }
}

void button(String s) {
    ui.fillRect(8, 272, 224, 40, 0x0942);
    ui.drawRect(8, 272, 224, 40, ACC);
    text(20, 286, s, 1, FG);
}

void frame() {
    ui.fillScreen(BG);
    for (int y = 0; y < 268; y += 4) ui.drawFastHLine(0, y, 240, 0x0040);
    ui.drawRect(3, 3, 234, 264, DIM);
    text(12, 12, "КСЕНО-ЛОКАТОР", 2);
    text(12, 33, "USCSS / MOTION TRACKER / 2.4G", 1, ACC);
    ui.drawFastHLine(8, 46, 224, DIM);
}

void calculate(bool found, int rssi) {
    pingPeriod = 0;
    targetSeen = found;
    good = found ? 1 : 0;
    if (!found) {
        havePrevious = false;
        trend = "ЦЕЛЬ ПОТЕРЯНА";
        return;
    }
    meanRSSI = rssi;
    delta = havePrevious ? meanRSSI - previousRSSI : 0;
    trend = !havePrevious ? "ПЕРВЫЙ ЗАМЕР" :
            delta >= 3 ? "ПРИБЛИЖАЕТЕСЬ" : delta <= -3 ? "ОТДАЛЯЕТЕСЬ" : "БЕЗ ИЗМЕНЕНИЙ";
    previousRSSI = meanRSSI;
    havePrevious = true;
    if (audioReady)
        pingPeriod = 1800 - constrain(int((meanRSSI + 95) * 1500 / 60), 0, 1500);
}

void volumeControls() {
    const int x = 217, level = pingVolume.load();
    ui.fillRect(199, 63, 34, 88, BG);
    for (int y : {78, 137}) {
        ui.fillCircle(x, y, 14, 0x0942);
        ui.drawCircle(x, y, 14, ACC);
        ui.drawCircle(x, y, 11, DIM);
        ui.drawFastHLine(x -5, y, 11, audioReady ? FG : DIM);
    }
    ui.drawFastVLine(x, 73, 11, audioReady ? FG : DIM);
    // Ten horizontal segments between the two buttons.
    for (int i = 0; i < 10; i++) ui.fillRect(209, 120 - i * 3, 16, 2, audioReady && i < level ? ACC : DIM);
}

// Only this viewport changes during animation. Pack it for one SPI transfer.
constexpr int radarX = 40, radarY = 65, radarW = 158, radarH = 85;
static uint16_t radarPixels[radarW * radarH];

void drawRadar() {
    const uint32_t now = millis();
    pulseFrame = now / 33;
    ui.fillRect(radarX, radarY, radarW, radarH, BG);
    for (int y = 68; y < 150; y += 4)
        ui.drawFastHLine(radarX, y, radarW, 0x0040);

    // Clip every ring, including the fixed grid, away from labels and buttons.
    auto ring = [](float radius, uint16_t color) {
        for (int degree = 0; degree < 360; ++degree) {
            const float angle = degree * PI / 180.0f;
            const int x = lroundf(120 + cosf(angle) * radius);
            const int y = lroundf(108 + sinf(angle) * radius);
            if (x >= radarX && x < radarX + radarW &&
                y >= radarY && y < radarY + radarH)
                ui.drawPixel(x, y, color);
        }
    };
    for (int radius = 19; radius <= 76; radius += 19) ring(radius, DIM);
    ui.drawFastHLine(radarX, 108, radarW, DIM);
    ui.drawFastVLine(120, radarY, radarH, DIM);

    // Time-based motion stays at the same speed even if a frame is delayed.
    for (int wave = 0; wave < 3; ++wave) {
        const float progress = fmodf(now / 2400.0f + wave / 3.0f, 1.0f);
        const float radius = 2 + progress * 74;
        const int green = 12 + int((1 - progress) * 40);
        ring(radius - 2, uint16_t((green / 4) << 5));
        ring(radius - 1, uint16_t((green / 2) << 5));
        ring(radius, uint16_t(green << 5));
    }
    // BSSID gives each background contact a stable decorative bearing.
    // RSSI controls radius only: neither bearing nor metres are measured.
    for (int i = 0; i < nearbyCount; ++i) {
        if (memcmp(nearby[i].mac, target, 6) == 0) continue;
        uint32_t hash = 2166136261u;
        for (uint8_t b : nearby[i].mac) hash = (hash ^ b) * 16777619u;
        const float angle = (hash % 360) * PI / 180.f;
        const float radius = constrain(int((-nearby[i].rssi - 30) * .45f), 12, 32);
        const int x = lroundf(120 + cosf(angle) * radius);
        const int y = lroundf(108 + sinf(angle) * radius);
        ui.drawCircle(x, y, 3, 0x0200);
        ui.fillCircle(x, y, 1, 0x0320);
    }
    ui.drawCircle(120, 108, 4, ACC);
    ui.fillCircle(120, 108, 2, FG);
    const bool seen = targetSeen;
    if (good && seen) {
        // Above the centre is a decorative bearing, NOT a measured direction.
        const int radius = constrain(int((-meanRSSI - 30) * 0.45f), 12, 32);
        const int x = 120, y = 108 - radius;
        const float glow = (sinf(now * 0.004f) + 1) * 0.5f;
        ui.drawCircle(x, y, 8, uint16_t((12 + int(glow * 16)) << 5));
        ui.drawCircle(x, y, 5, ACC);
        ui.fillCircle(x, y, 2, FG);
        ui.drawFastHLine(x - 11, y, 3, ACC);
        ui.drawFastHLine(x + 9, y, 3, ACC);
    }
}

void pushRadar() {
    for (int row = 0; row < radarH; ++row)
        memcpy(radarPixels + row * radarW,
               ui.getBuffer() + (radarY + row) * 240 + radarX,
               radarW * sizeof(uint16_t));
    lcd.pushImage(radarX, radarY, radarW, radarH, radarPixels);
}

void render(bool animationOnly = false) {
    if (animationOnly) {
        drawRadar();
        pushRadar();
        return;
    }
    frame();
    if (state == 0) {
        text(12, 51, "ВЫБЕРИТЕ ЦЕЛЬ", 1, AMBER);
        for (int j = 0; j < 5; j++) {
            int i = page * 5 + j;
            if (i >= count) break;
            int y = 66 + j * 39;
            ui.drawRect(8, y -3, 224, 36, DIM);
            text(14, y, aps[i].name);
            text(14, y + 16, String(aps[i].rssi) + " dBm  КАНАЛ " + String(aps[i].channel), 1, ACC);
            int bars = constrain((aps[i].rssi + 100) / 12, 0, 5);
            for (int k = 0; k < 5; k++) ui.fillRect(200 + k * 5, y + 20 - k * 3, 3, 4 + k * 3, k < bars ? ACC : DIM);
        }
        if (!count) text(16, 112, "СЕТИ НЕ ОБНАРУЖЕНЫ", 1, AMBER);
        text(12, 258, "СТРАНИЦА " + String(page + 1), 1, DIM);
        button((page + 1) * 5 < count ? "СЛЕДУЮЩАЯ СТРАНИЦА >" : "ОБНОВИТЬ СПИСОК");
    } else {
        text(12, 52, selected, 1, FG);
        // Terminal markings stay outside the animated viewport.
        text(12, 76, "M314", 1, DIM);
        text(12, 91, "SYS", 1, DIM);
        text(12, 126, "01", 1, ACC);
        drawRadar();
        bool seen = targetSeen;
        volumeControls();
        text(12, 156, trend, trend == "ПРИБЛИЖАЕТЕСЬ" || trend == "ОТДАЛЯЕТЕСЬ" ? 2 : 1, trend == "ОТДАЛЯЕТЕСЬ" ? AMBER : FG);
        if (good && seen) {
            float distance = powf(10.0f,(-40.0f - meanRSSI) / 25.0f);
            text(12, 179, "ПРИМЕРНО: " + String(distance, 1) + " М", 1, FG);
        } else text(12, 179, "РАССТОЯНИЕ: НЕТ ДАННЫХ", 1, AMBER);
        text(185, 179, audioReady ? String(pingVolume.load() * 10) + "%" : "ТИШ.", 1, ACC);
        text(12, 194, "СИЛА КОНТАКТА / LIVE", 1, ACC);
        ui.drawRect(10, 208, 220, 34, DIM);
        int strength = targetSeen ? constrain(int(meanRSSI + 100) * 216 / 70, 0, 216) : 0;
        // Discrete phosphor bars, like an industrial tracking terminal.
        for (int segment = 0; segment < 22; ++segment) {
            const bool lit = (segment + 1) * 216 / 22 <= strength;
            ui.fillRect(13 + segment * 10, 212, 7, 26, lit ? ACC : 0x0942);
            if (lit) ui.drawFastHLine(13 + segment * 10, 212, 7, FG);
        }
        text(12, 247, good ? "СИГНАЛ " + String(meanRSSI, 1) + " dBm" : "ОЖИДАНИЕ СИГНАЛА", 1, ACC);
        text(12, 258, "NOSTROMO / SCIENCE DIVISION", 1, DIM);
        button("< ВЫБРАТЬ ДРУГУЮ СЕТЬ");
    }
    lcd.pushImage(0, 0, 240, 320, ui.getBuffer());
    dirty = false;
}

// Only the scan worker owns WiFi scan results. No String crosses cores.
struct ScanRequest {
    uint32_t generation;
    bool list;
    int channel;
    uint8_t mac[6];
};
struct ScanEntry {
    char name[129];
    uint8_t mac[6];
    int channel, rssi;
};
struct ScanResult {
    uint32_t generation;
    bool list, found, failed;
    int count, rssi;
    ScanEntry entries[30];
};
struct TouchEvent { int x, y; uint32_t at; };
QueueHandle_t scanRequests, scanResults, touchEvents;
uint32_t scanGeneration = 0;

void scanTask(void *) {
    ScanRequest request;
    // Large result lives on this task's stack, not the UI stack.
    ScanResult result;
    for (;;) {
        xQueueReceive(scanRequests, &request, portMAX_DELAY);
        result = {};
        result.generation = request.generation;
        result.list = request.list;
        int n = WiFi.scanNetworks(false, true, false, 180,
                                  0);
        result.failed = n < 0;
        for (int i = 0; i < n; ++i) {
            if (result.count < 30) {
                ScanEntry &entry = result.entries[result.count++];
                String name = WiFi.SSID(i);
                if (!name.length()) name = "[СКРЫТАЯ СЕТЬ]";
                name.toCharArray(entry.name, sizeof(entry.name));
                memcpy(entry.mac, WiFi.BSSID(i), 6);
                entry.channel = WiFi.channel(i);
                entry.rssi = WiFi.RSSI(i);
            }
            if (!request.list && memcmp(WiFi.BSSID(i), request.mac, 6) == 0) {
                result.found = true;
                result.rssi = WiFi.RSSI(i);
            }
        }
        WiFi.scanDelete();
        xQueueOverwrite(scanResults, &result);
        vTaskDelay(1);
    }
}

void requestScan(bool list) {
    ScanRequest request = {};
    request.generation = scanGeneration;
    request.list = list;
    request.channel = channel;
    memcpy(request.mac, target, 6);
    xQueueOverwrite(scanRequests, &request);
}

void listScan() {
    pingPeriod = 0;
    ++scanGeneration; // Ignore results from a previously selected target.
    scanning = false;
    listScanning = true;
    frame();
    text(20, 110, "ПОИСК СИГНАЛОВ...", 1, AMBER);
    lcd.pushImage(0, 0, 240, 320, ui.getBuffer());
    dirty = false;
    requestScan(true);
}

void tickScanResults() {
    static ScanResult result;
    if (xQueueReceive(scanResults, &result, 0) != pdTRUE) return;
    if (result.generation != scanGeneration) return;
    if (result.list) {
        listScanning = false;
        count = result.count;
        for (int i = 0; i < count; ++i) {
            aps[i].name = result.entries[i].name;
            memcpy(aps[i].mac, result.entries[i].mac, 6);
            aps[i].channel = result.entries[i].channel;
            aps[i].rssi = result.entries[i].rssi;
        }
        page = 0;
    } else {
        scanning = false;
        if (state != 1) return;
        if (result.failed) {
            trend = "ПРОПУСК ЗАМЕРА";
            dirty = true;
            return; // Keep last successful contacts; do not declare target lost.
        }
        nearbyCount = result.count;
        for (int i = 0; i < nearbyCount; ++i) {
            memcpy(nearby[i].mac, result.entries[i].mac, 6);
            nearby[i].rssi = result.entries[i].rssi;
        }
        calculate(result.found, result.rssi);
    }
    dirty = true;
}

void tickScan() {
    if (state != 1 || scanning || (int32_t)(millis() - nextScan) < 0) return;
    requestScan(false);
    scanning = true;
    nextScan = millis() + 3000;
}

int touched(int & x, int & y) {
    Wire.beginTransmission(0x38);
    Wire.write(0x02);
    if (Wire.endTransmission(false) != 0) return -1;
    if (Wire.requestFrom(0x38, 5) != 5) return -1;
    uint8_t b[5];
    for (int i = 0; i < 5; i++) b[i] = Wire.read();
    if (!(b[0] & 15) || (b[1] >> 6) == 1) return 0;
    x = ((b[1] & 15) << 8) | b[2];
    y = ((b[3] & 15) << 8) | b[4];
    if (x >= 240 || y >= 320) return -1;

    // With display rotation 0, this panel's touch axes run in the
    // opposite direction to the visible image (as in the working radio).
    // Convert once here, before sending coordinates to any UI button.
    x = 239 - x;
    y = 319 - y;
    return 1;
}

// Touch is polled independently of slow SPI display transfers.
// Codec setup finishes before this task starts: Wire has a single runtime owner.
void touchTask(void *) {
    bool down = false;
    uint32_t lastContact = 0;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        int x, y;
        int contact = touched(x, y);
        if (contact > 0) {
            lastContact = millis();
            if (!down) {
                TouchEvent event{x, y, millis()};
                xQueueSend(touchEvents, &event, 0);
                down = true;
            }
        } else if (contact == 0 && millis() - lastContact >= 20) {
            down = false;
        }
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(5));
    }
}

void uiTask(void *);

void setup() {
    pinMode(1, OUTPUT);
    digitalWrite(1, HIGH);
    pinMode(45, OUTPUT);
    digitalWrite(45, LOW);
    lcd.init();
    lcd.setRotation(0);
    lcd.setColorDepth(16);
    // GFXcanvas16 stores native RGB565; pushImage needs byte swapping.
    // This applies to both the full screen and the radar viewport.
    lcd.setSwapBytes(true);
    digitalWrite(45, HIGH);
    pinMode(18, OUTPUT);
    digitalWrite(18, LOW);
    delay(10);
    digitalWrite(18, HIGH);
    delay(300);
    Wire.begin(16, 15);
    Wire.setClock(100000);
    audioReady = beginLocatorAudio();
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    Wire.setTimeOut(10);
    scanRequests = xQueueCreate(1, sizeof(ScanRequest));
    scanResults = xQueueCreate(1, sizeof(ScanResult));
    touchEvents = xQueueCreate(8, sizeof(TouchEvent));
    if (!scanRequests || !scanResults || !touchEvents) {
        Serial.begin(115200);
        Serial.println("Queue allocation failed");
        while (true) delay(1000);
    }
    if (xTaskCreatePinnedToCore(scanTask, "wifi_scan", 12288, nullptr, 1, nullptr, 0) != pdPASS ||
        xTaskCreatePinnedToCore(touchTask, "touch", 3072, nullptr, 3, nullptr, 0) != pdPASS ||
        xTaskCreatePinnedToCore(uiTask, "locator_ui", 8192, nullptr, 1, nullptr, 1) != pdPASS) {
        Serial.begin(115200);
        Serial.println("Task creation failed");
        while (true) delay(1000);
    }
}

void uiStep() {
    TouchEvent event;
    if (xQueueReceive(touchEvents, &event, 0) == pdTRUE && millis() - event.at < 300 && !listScanning) {
        const int x = event.x, y = event.y;
        if (state == 0) {
            if (y >= 272) {
                if ((page + 1) * 5 < count) page++;
                else listScan();
                dirty = true;
            } else if (y >= 63 && y < 258) {
                int i = page * 5 +(y -63) / 39;
                if (i < count) {
                    memcpy(target, aps[i].mac, 6);
                    channel = aps[i].channel;
                    selected = aps[i].name;
                    good = 0;
                    targetSeen = havePrevious = false;
                    scanning = false;
                    pingPeriod = 0;
                    delta = 0;
                    trend = "ОЖИДАНИЕ СИГНАЛА";
                    ++scanGeneration;
                    state = 1;
                    nextScan = millis();
                    dirty = true;
                }
            }
        } else if (audioReady && ((x -217) *(x -217) +(y -78) *(y -78) <= 196)) {
            pingVolume.store(constrain(pingVolume.load() + 1, 0, 10));
            dirty = true;
        } else if (audioReady && ((x -217) *(x -217) +(y -137) *(y -137) <= 196)) {
            pingVolume.store(constrain(pingVolume.load() -1, 0, 10));
            dirty = true;
        } else if (y >= 272) {
            state = 0;
            pingPeriod = 0;
            listScan();
        }
    }
    tickScanResults();
    tickScan();
    if (dirty && !listScanning) render();
    else if (state == 1 && (int)(millis() / 33) != pulseFrame) {
        render(true);
    }
    delay(2);
}

// Scan worker runs in parallel; entry requires a fresh tap after loading.
// No scan state, touch calibration, audio controls or hardware setup changes.
void startupIntro() {
    const uint32_t start = millis();
    int lastFrame = -1;
    const uint32_t introDuration = 7600;
    while (millis() - start < introDuration) {
        // Ignore touches during loading, including held contacts.
        xQueueReset(touchEvents);
        const uint32_t elapsed = millis() - start;
        const int animationFrame = elapsed / 33;
        if (animationFrame == lastFrame) { delay(2); continue; }
        lastFrame = animationFrame;
        frame();
        text(12, 60, "ИНИЦИАЛИЗАЦИЯ M314", 1, ACC);
        text(12, 78, "USCSS / SYSTEM BOOT", 1, DIM);
        // Fade the locator grid in from black, then expand the phosphor rings.
        const int glow = constrain(int(elapsed * 40 / 1600), 0, 40);
        const uint16_t phosphor = uint16_t(glow << 5);
        const uint16_t grid = uint16_t((glow / 4) << 5);
        for (int radius = 20; radius <= 60; radius += 20)
            ui.drawCircle(120, 151, radius, grid);
        ui.drawFastHLine(56, 151, 129, grid);
        ui.drawFastVLine(120, 87, 129, grid);
        const int waveRadius = 4 + (elapsed % 1000) * 60 / 1000;
        ui.drawCircle(120, 151, waveRadius, phosphor);
        ui.drawCircle(120, 151, waveRadius + 1, uint16_t((glow / 3) << 5));
        ui.fillCircle(120, 151, 2, phosphor);
        if (elapsed >= 350) text(12, 219, "ДИСПЛЕЙ ........ ГОТОВ", 1, ACC);
        if (elapsed >= 850) text(12, 231, "СЕНСОР ......... ГОТОВ", 1, ACC);
        if (elapsed >= 1350)
            text(12, 243, audioReady ? "АУДИО .......... ГОТОВО" : "АУДИО .......... ТИШИНА",
                 1, audioReady ? ACC : AMBER);
        if (elapsed >= 1850) text(12, 255, "ПОИСК СИГНАЛОВ...", 1, FG);
        ui.drawRect(12, 274, 216, 12, DIM);
        const int progress = constrain(int(elapsed * 20 / introDuration), 0, 20);
        for (int i = 0; i < progress; ++i)
            ui.fillRect(15 + i * 10, 277, 7, 6, ACC);
        text(12, 299, "ЗАГРУЗКА СИСТЕМЫ...", 1, DIM);
        lcd.pushImage(0, 0, 240, 320, ui.getBuffer());
        delay(2);
    }
    // Keep the completed boot screen until a new touch arrives.
    ui.fillRect(12, 274, 216, 46, 0);
    ui.drawRect(12, 274, 216, 12, DIM);
    for (int i = 0; i < 20; ++i)
        ui.fillRect(15 + i * 10, 277, 7, 6, ACC);
    text(12, 299, "НАЖМИТЕ НА ЭКРАН", 1, ACC);
    lcd.pushImage(0, 0, 240, 320, ui.getBuffer());
    xQueueReset(touchEvents);
    const uint32_t readyAt = millis();
    TouchEvent event;
    for (;;) {
        if (xQueueReceive(touchEvents, &event, pdMS_TO_TICKS(20)) == pdTRUE &&
            int32_t(event.at - readyAt) > 0) break;
    }
    // The confirmation touch must not select a network.
    xQueueReset(touchEvents);
}

void uiTask(void *) {
    listScan();
    startupIntro();
    tickScanResults();
    if (listScanning) {
        frame();
        text(20, 110, "ПОИСК СИГНАЛОВ...", 1, AMBER);
        lcd.pushImage(0, 0, 240, 320, ui.getBuffer());
    } else dirty = true;
    for (;;) uiStep();
}

void loop() {
    // Arduino's loop task is unused; all application work is explicitly pinned.
    delay(1000);
}