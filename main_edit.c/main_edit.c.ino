/*
 * ==============================================================================
 * 프로젝트: 아마따 (AMATTA) - 스마트 트레이 Arduino UNO R3 메인 펌웨어 (v1.1)
 * ==============================================================================
 * [메모리 최적화]
 * - Flash(프로그램 저장 공간) & SRAM(동적 메모리) 대폭 다이어트 적용
 * - 무거운 snprintf / atof / float 연산 제거 -> 고정소수점(0.1g 단위, deci-grams) 연산
 * - 모든 상수 문자열 PROGMEM(F() / PSTR()) 배치로 SRAM 여유 공간 대폭 확보
 * ==============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <EEPROM.h>
#include <SoftwareSerial.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>

// ==============================================================================
// 1. 핀 매핑 및 하드웨어 상수 정의
// ==============================================================================
const byte NUM_TRAYS = 2;                      // 로드셀 트레이 2칸 (1번, 2번)
const byte HX_DT[NUM_TRAYS]    = {2, 4};       // CH1: D2, CH2: D4
const byte HX_CLOCK[NUM_TRAYS] = {3, 5};       // CH1: D3, CH2: D5

const byte NUM_SPACES = 3;                     // 1번 트레이, 2번 트레이, 3번 우산 공간
const byte STRIP_PIN[NUM_SPACES] = {8, 9, 10}; // D8: 1번칸, D9: 2번칸, D10: 3번(우산)칸
const byte LEDS_PER_STRIP = 8;                 // 스트립당 WS2812B 8구

const byte FAN_PIN   = 11;                     // 우산 건조 팬 제어 핀 (D11)
const byte BT_RX_PIN = 12;                     // HC-06 TXD -> UNO D12
const byte BT_TX_PIN = 13;                     // UNO D13 -> 분압 -> HC-06 RXD
const byte DOOR_PIN  = A0;                     // 현관문 리드스위치 (A0만 사용, A1/A2 제외)
const byte IR_PIN    = A3;                     // 3번 공간 우산 감지 IR 센서 (감지 시 LOW)

const byte DOOR_CLOSED_LEVEL = LOW;            // 자석 부착(닫힘) = LOW, 분리(열림) = HIGH
const byte IR_DETECTED_LEVEL = LOW;            // 장애물 감지 = LOW

const unsigned long DEBOUNCE_MS        = 40UL;
const unsigned long STATE_HEARTBEAT_MS = 1000UL;  // 1초 주기 상태 스냅샷 전송
const unsigned long NORMAL_BLINK_MS    = 750UL;   // 일반 백색 LED 0.75초 점멸 주기
const unsigned long NORMAL_MAX_MS      = 60000UL; // 문 열림 일반 점등 최대 유지 시간 (60초)
const unsigned long HX_TIMEOUT_MS      = 1500UL;  // 로드셀 응답 타임아웃

SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN);

// ==============================================================================
// 2. EEPROM 저장용 설정 및 보정 데이터 구조체 (고정소수점 최적화)
// ==============================================================================
const uint16_t EEPROM_MAGIC = 0xA412;

// 공간별 설정 비트 플래그
const uint8_t CFG_LED_EN        = 0x01;
const uint8_t CFG_LED_DOOR      = 0x02;
const uint8_t CFG_STERIL_EN     = 0x04;
const uint8_t CFG_STERIL_DETECT = 0x08;
const uint8_t CFG_STERIL_DOOR   = 0x10;

struct PersistentData {
  uint16_t magic;
  uint16_t bootCount;
  long tareOffset[NUM_TRAYS];       // 로드셀 영점 Raw 기준값
  long scaleX10[NUM_TRAYS];         // (1g당 Raw 변화량) * 10 (예: 25.0 -> 250)
  int16_t detectDg[NUM_TRAYS];      // 감지 문턱값 (0.1g 단위, 기본 150 = 15.0g)
  int16_t releaseDg[NUM_TRAYS];     // 해제 문턱값 (0.1g 단위, 기본 80  = 8.0g)
  int16_t deadzoneDg[NUM_TRAYS];    // 데드존 (0.1g 단위, 기본 30 = 3.0g)
  uint8_t tareValidMask;            // bit0: CH1, bit1: CH2
  uint8_t spaceFlags[NUM_SPACES];   // CFG_* 비트마스크
  uint8_t sterilSec[NUM_SPACES];    // 살균 시간 (5, 10, 20, 30초)
  uint8_t fanDurationSec;           // 우산 건조 팬 시간 (0, 15, 30, 60초)
  uint16_t crc;
};

PersistentData config;
uint32_t txSeq = 0;
bool stateDirty = false;
unsigned long lastStateSentMs = 0;

// ==============================================================================
// 3. 센서 및 출력 런타임 상태 구조체
// ==============================================================================
enum SensorHealth : uint8_t { SENSOR_WAITING = 0, SENSOR_OK = 1, SENSOR_ERROR = 2 };
enum LedLogicalMode : uint8_t { LED_OFF = 0, LED_NORMAL_BLINK = 1, LED_STERIL = 2 };

struct TrayRuntime {
  long rawFiltered;
  int16_t weightDg;                 // 0.1g 단위 (0 ~ 30000 -> 0.0g ~ 3000.0g)
  bool present;
  SensorHealth health;
  unsigned long lastSampleMs;
  unsigned long candidateSinceMs;
  bool candidatePresent;
};

struct DigitalSensorRuntime {
  byte rawLevel;
  byte stableLevel;
  unsigned long lastDebounceMs;
  bool active;                      // door: isOpen, umbrella: present
  SensorHealth health;
};

struct SpaceOutputRuntime {
  bool normalActive;
  unsigned long normalStartedMs;
  unsigned long normalUntilMs;      // 수동 테스트 시 일반 점멸 종료 시각
  bool sterilActive;
  unsigned long sterilUntilMs;
  bool manualOverride;              // 시리얼 수동 명령 시 센서 조건 무시 플래그
  char activeReqId[12];
  byte currentStateCode;            // 0:OFF, 1:WHITE, 2:PURPLE
};

struct FanRuntime {
  bool active;
  unsigned long untilMs;
  char activeReqId[12];
};

TrayRuntime trays[NUM_TRAYS];
DigitalSensorRuntime umbrella;
DigitalSensorRuntime door;
SpaceOutputRuntime outputs[NUM_SPACES];
FanRuntime fan;

uint8_t simMask = 0;                // bit0~2: 1~3번칸 가상거치, bit3: 현관문 가상고정
bool blinkPhaseOn = true;
unsigned long lastBlinkToggleMs = 0;
bool ledsNeedHardwareRefresh = true;
bool liveMonitorEnabled = false;
unsigned long lastMonitorPrintMs = 0;

// ==============================================================================
// 4. 1602 I2C LCD 및 2x2 날씨 도트 아이콘 (PROGMEM)
// ==============================================================================
byte lcdAddress = 0x27;
bool lcdAvailable = false;

enum WeatherIconType : uint8_t {
  W_ICON_WAITING = 0, // 기상 정보 대기중 아이콘
  W_ICON_SUNNY   = 1, // 해 (SUNNY)
  W_ICON_CLOUDY  = 2, // 구름 (CLOUDY)
  W_ICON_RAIN    = 3, // 비 (RAIN)
  W_ICON_SNOW    = 4, // 눈 (SNOW)
  W_ICON_NONE    = 5  // 수신 실패/만료 (NO DATA)
};

const byte WEATHER_ICONS[6][4][8] PROGMEM = {
  // 0: W_ICON_WAITING
  {
    {0b00111, 0b01000, 0b10000, 0b10001, 0b10001, 0b10001, 0b10001, 0b01000},
    {0b11100, 0b00010, 0b00001, 0b10001, 0b10001, 0b10001, 0b11101, 0b00010},
    {0b01000, 0b10000, 0b10000, 0b10000, 0b01000, 0b00111, 0b00000, 0b00000},
    {0b00010, 0b00001, 0b00001, 0b00001, 0b00010, 0b11100, 0b00000, 0b00000}
  },
  // 1: W_ICON_SUNNY
  {
    {0b00100, 0b10100, 0b01001, 0b00011, 0b11011, 0b00011, 0b01001, 0b10100},
    {0b00100, 0b00101, 0b10010, 0b11000, 0b11011, 0b11000, 0b10010, 0b00101},
    {0b10100, 0b01001, 0b00011, 0b11011, 0b00011, 0b01001, 0b10100, 0b00100},
    {0b00101, 0b10010, 0b11000, 0b11011, 0b11000, 0b10010, 0b00101, 0b00100}
  },
  // 2: W_ICON_CLOUDY
  {
    {0b00000, 0b00000, 0b00011, 0b00111, 0b01111, 0b11111, 0b11111, 0b01111},
    {0b00000, 0b00000, 0b11000, 0b11100, 0b11110, 0b11111, 0b11111, 0b11110},
    {0b01111, 0b00111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
    {0b11110, 0b11100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000}
  },
  // 3: W_ICON_RAIN
  {
    {0b00000, 0b00011, 0b00111, 0b01111, 0b11111, 0b11111, 0b01111, 0b00000},
    {0b00000, 0b11000, 0b11100, 0b11110, 0b11111, 0b11111, 0b11110, 0b00000},
    {0b00101, 0b01010, 0b10100, 0b00101, 0b01010, 0b10100, 0b00000, 0b00000},
    {0b01010, 0b10100, 0b01010, 0b01010, 0b10100, 0b01000, 0b00000, 0b00000}
  },
  // 4: W_ICON_SNOW
  {
    {0b00000, 0b00011, 0b00111, 0b01111, 0b11111, 0b11111, 0b01111, 0b00000},
    {0b00000, 0b11000, 0b11100, 0b11110, 0b11111, 0b11111, 0b11110, 0b00000},
    {0b01010, 0b00100, 0b01010, 0b00000, 0b01010, 0b00100, 0b01010, 0b00000},
    {0b01010, 0b00100, 0b01010, 0b00000, 0b01010, 0b00100, 0b01010, 0b00000}
  },
  // 5: W_ICON_NONE
  {
    {0b01111, 0b10000, 0b10000, 0b10001, 0b10001, 0b10001, 0b10001, 0b10000},
    {0b11110, 0b00001, 0b00001, 0b10001, 0b10001, 0b10001, 0b10001, 0b00001},
    {0b10000, 0b10001, 0b10001, 0b10000, 0b10000, 0b01111, 0b00000, 0b00000},
    {0b00001, 0b10001, 0b10001, 0b00001, 0b00001, 0b11110, 0b00000, 0b00000}
  }
};

struct WeatherDisplayState {
  bool receivedOnce;
  bool valid;
  WeatherIconType icon;
  WeatherIconType loadedIcon;
  char line1[14];
  char windows[2][14];              // 최대 2개 구간 순환 (SRAM 절약)
  uint8_t windowCount;
  uint8_t currentWindowIdx;
  unsigned long lastWindowCycleMs;
  unsigned long receivedAtMs;
  unsigned long ttlMs;
  bool dirty;
};

WeatherDisplayState weatherLcd;

// 수신 버퍼 및 통신 모드 (기본: MODE_BT 블루투스 모드, 시리얼 입력 시 MODE_SERIAL 독점 모드 전환)
enum CommMode : uint8_t { MODE_BT = 0, MODE_SERIAL = 1 };
CommMode commMode = MODE_BT;
bool btPeerActive = false;                  // 블루투스로부터 데이터가 1회 이상 수신되었는지 여부
const unsigned long SERIAL_MODE_TIMEOUT_MS = 180000UL; // 시리얼 무입력 180초 경과 시 자동 BT 모드 복귀

char btRxBuf[168];
byte btRxLen = 0;
bool btRxOverflow = false;
unsigned long lastBtRxMs = 0;

char serialRxBuf[80];
byte serialRxLen = 0;
unsigned long lastSerialRxMs = 0;

void setCommMode(CommMode nextMode);
void sendAckP(const char* reqId, PGM_P statusP, PGM_P reasonP = NULL);
void sendStateSnapshot(Stream &out);
void refreshLcdWeather(bool force = false);

// ==============================================================================
// 5. 유틸리티: 고정소수점(0.1 단위) 파싱 및 출력 (float/atof 완전 대체)
// ==============================================================================
// 문자열 "15.4" -> 154, "200" -> 2000 반환
long parseDeci(const char* s) {
  if (!s) return 0;
  while (*s == ' ' || *s == '\t') ++s;
  bool neg = (*s == '-');
  if (neg || *s == '+') ++s;
  long whole = 0;
  while (*s >= '0' && *s <= '9') {
    whole = whole * 10L + (*s - '0');
    ++s;
  }
  long frac = 0;
  if (*s == '.') {
    ++s;
    if (*s >= '0' && *s <= '9') frac = (*s - '0');
  }
  long res = whole * 10L + frac;
  return neg ? -res : res;
}

void printDeci(Stream &out, long valDg) {
  if (valDg < 0) {
    out.print('-');
    valDg = -valDg;
  }
  out.print(valDg / 10L);
  out.print('.');
  out.print(valDg % 10L);
}

void copyStrP(char* dst, PGM_P srcP, size_t maxLen) {
  strncpy_P(dst, srcP, maxLen - 1);
  dst[maxLen - 1] = '\0';
}

void toUpperStr(char* s) {
  if (!s) return;
  for (; *s; ++s) if (*s >= 'a' && *s <= 'z') *s -= 32;
}

// ==============================================================================
// 6. 순수 AVR 고속 WS2812B 드라이버 (16MHz 정밀 타이밍)
// ==============================================================================
void ws2812Send(byte pin, const byte *data, uint16_t len) {
  if (len == 0) return;
  volatile uint8_t *port = portOutputRegister(digitalPinToPort(pin));
  uint8_t pinMask = digitalPinToBitMask(pin);
  uint8_t hi = *port | pinMask;
  uint8_t lo = *port & ~pinMask;
  uint8_t sreg = SREG;
  cli();

  uint8_t curbyte, ctr;
  asm volatile(
    "1: \n\t"
    "ld %[curbyte], %a[dataptr]+ \n\t"
    "ldi %[ctr], 8 \n\t"
    "2: \n\t"
    "st %a[port], %[hi] \n\t"
    "sbrs %[curbyte], 7 \n\t"
    "st %a[port], %[lo] \n\t"
    "lsl %[curbyte] \n\t"
    "dec %[ctr] \n\t"
    "rjmp .+0 \n\t"
    "st %a[port], %[lo] \n\t"
    "rjmp .+0 \n\t"
    "brne 2b \n\t"
    "sbiw %[count], 1 \n\t"
    "brne 1b \n\t"
    : [dataptr] "+e" (data),
      [count] "+w" (len),
      [curbyte] "=&d" (curbyte),
      [ctr] "=&d" (ctr)
    : [port] "e" (port),
      [hi] "r" (hi),
      [lo] "r" (lo)
  );

  SREG = sreg;
  delayMicroseconds(80);
}

void writeStripHardware(byte spaceIdx, byte r, byte g, byte b) {
  if (spaceIdx >= NUM_SPACES) return;
  byte buf[LEDS_PER_STRIP * 3];
  for (byte i = 0; i < LEDS_PER_STRIP; ++i) {
    buf[i * 3 + 0] = g;
    buf[i * 3 + 1] = r;
    buf[i * 3 + 2] = b;
  }
  ws2812Send(STRIP_PIN[spaceIdx], buf, sizeof(buf));
  if (r == 0 && g == 0 && b == 0) {
    digitalWrite(STRIP_PIN[spaceIdx], LOW);
  }
}

// ==============================================================================
// 7. 1602 I2C LCD 드라이버 및 2x2 날씨 렌더링
// ==============================================================================
#define LCD_BACKLIGHT 0x08
#define LCD_ENABLE    0x04

void lcdWriteNibble(byte nibble, byte rs) {
  byte data = (nibble & 0xF0) | rs | LCD_BACKLIGHT;
  Wire.beginTransmission(lcdAddress);
  Wire.write(data | LCD_ENABLE);
  Wire.endTransmission();
  delayMicroseconds(2);
  Wire.beginTransmission(lcdAddress);
  Wire.write(data & ~LCD_ENABLE);
  Wire.endTransmission();
  delayMicroseconds(50);
}

void lcdSendByte(byte val, byte rs) {
  lcdWriteNibble(val & 0xF0, rs);
  lcdWriteNibble((val << 4) & 0xF0, rs);
}

void lcdCommand(byte cmd) {
  lcdSendByte(cmd, 0);
  if (cmd == 0x01 || cmd == 0x02) delay(2);
}

void lcdChar(char c) {
  lcdSendByte((byte)c, 1);
}

void lcdSetCursor(byte col, byte row) {
  lcdCommand(0x80 | (col + (row ? 0x40 : 0x00)));
}

void lcdLoadWeatherIcon(WeatherIconType iconType) {
  if (!lcdAvailable || iconType > W_ICON_NONE) return;
  for (byte slot = 0; slot < 4; ++slot) {
    lcdCommand(0x40 | (slot << 3));
    for (byte row = 0; row < 8; ++row) {
      lcdSendByte(pgm_read_byte(&WEATHER_ICONS[iconType][slot][row]), 1);
    }
  }
  lcdCommand(0x80);
  weatherLcd.loadedIcon = iconType;
}

bool initI2CLcd() {
  Wire.beginTransmission(0x27);
  if (Wire.endTransmission() == 0) {
    lcdAddress = 0x27;
  } else {
    Wire.beginTransmission(0x3F);
    if (Wire.endTransmission() == 0) lcdAddress = 0x3F;
    else return false;
  }
  delay(50);
  lcdWriteNibble(0x30, 0); delay(5);
  lcdWriteNibble(0x30, 0); delayMicroseconds(150);
  lcdWriteNibble(0x30, 0); delayMicroseconds(150);
  lcdWriteNibble(0x20, 0); delayMicroseconds(150);

  lcdCommand(0x28);
  lcdCommand(0x0C);
  lcdCommand(0x06);
  lcdCommand(0x01);
  delay(2);
  return true;
}

void lcdPrintPadded13(const char* text) {
  byte i = 0;
  while (text && text[i] && i < 13) {
    lcdChar(text[i]);
    ++i;
  }
  while (i < 13) {
    lcdChar(' ');
    ++i;
  }
}

void setWeatherPreset(WeatherIconType ic, PGM_P l1P, PGM_P l2P, bool isValid) {
  weatherLcd.valid = isValid;
  weatherLcd.icon = ic;
  copyStrP(weatherLcd.line1, l1P, sizeof(weatherLcd.line1));
  copyStrP(weatherLcd.windows[0], l2P, sizeof(weatherLcd.windows[0]));
  weatherLcd.windowCount = 1;
  weatherLcd.currentWindowIdx = 0;
  weatherLcd.lastWindowCycleMs = millis();
  weatherLcd.ttlMs = 0;
  weatherLcd.dirty = true;
}

void initWeatherDisplayWait() {
  weatherLcd.receivedOnce = false;
  weatherLcd.loadedIcon = (WeatherIconType)255;
  setWeatherPreset(W_ICON_WAITING, PSTR("WAITING..."), PSTR("WEATHER INFO"), false);
}

void setWeatherNoData() {
  setWeatherPreset(W_ICON_NONE, PSTR("NO DATA"), PSTR(""), false);
}

void refreshLcdWeather(bool force) {
  if (!lcdAvailable) return;
  unsigned long now = millis();

  if (weatherLcd.receivedOnce && weatherLcd.valid && weatherLcd.ttlMs > 0) {
    if (now - weatherLcd.receivedAtMs >= weatherLcd.ttlMs) {
      setWeatherNoData();
    }
  }

  if (weatherLcd.valid && weatherLcd.windowCount > 1) {
    if (now - weatherLcd.lastWindowCycleMs >= 3000UL) {
      weatherLcd.lastWindowCycleMs = now;
      weatherLcd.currentWindowIdx = (weatherLcd.currentWindowIdx + 1) % weatherLcd.windowCount;
      weatherLcd.dirty = true;
    }
  }

  if (!force && !weatherLcd.dirty) return;
  weatherLcd.dirty = false;

  if (weatherLcd.loadedIcon != weatherLcd.icon) {
    lcdLoadWeatherIcon(weatherLcd.icon);
  }

  lcdSetCursor(0, 0);
  lcdChar(0); lcdChar(1); lcdChar(' ');
  lcdPrintPadded13(weatherLcd.line1);

  lcdSetCursor(0, 1);
  lcdChar(2); lcdChar(3); lcdChar(' ');
  const char* line2 = (weatherLcd.windowCount > 0) ? weatherLcd.windows[weatherLcd.currentWindowIdx] : "";
  lcdPrintPadded13(line2);
}

// ==============================================================================
// 8. EEPROM 설정 관리 (CRC16 검증 포함)
// ==============================================================================
uint16_t calcConfigCrc(const PersistentData &d) {
  const byte *p = (const byte *)&d;
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < offsetof(PersistentData, crc); ++i) {
    crc ^= p[i];
    for (byte b = 0; b < 8; ++b) {
      crc = (crc >> 1) ^ ((crc & 1) ? 0xA001 : 0);
    }
  }
  return crc;
}

void saveConfigToEeprom() {
  config.crc = calcConfigCrc(config);
  EEPROM.put(0, config);
}

void resetDefaultConfig() {
  memset(&config, 0, sizeof(config));
  config.magic = EEPROM_MAGIC;
  config.bootCount = 1;

  for (byte i = 0; i < NUM_TRAYS; ++i) {
    config.tareOffset[i] = 0;
    config.scaleX10[i]   = 250; // 25.0 raw/g
    config.detectDg[i]   = 150; // 15.0g
    config.releaseDg[i]  = 80;  // 8.0g
    config.deadzoneDg[i] = 30;  // 3.0g
  }
  config.tareValidMask = 0;

  for (byte s = 0; s < NUM_SPACES; ++s) {
    config.spaceFlags[s] = CFG_LED_EN | CFG_LED_DOOR;
    config.sterilSec[s]  = 10;
  }
  config.fanDurationSec = 0;
  saveConfigToEeprom();
}

void loadConfigFromEeprom() {
  EEPROM.get(0, config);
  bool ok = (config.magic == EEPROM_MAGIC) && (config.crc == calcConfigCrc(config));
  for (byte i = 0; i < NUM_TRAYS; ++i) {
    if (labs(config.scaleX10[i]) < 1 || config.detectDg[i] < 10) ok = false;
  }
  if (!ok) {
    resetDefaultConfig();
  } else {
    config.bootCount++;
    saveConfigToEeprom();
  }
}

// ==============================================================================
// 9. HX711 로드셀 측정 및 보정
// ==============================================================================
bool readHX711Raw(byte ch, long &outRaw) {
  if (digitalRead(HX_DT[ch]) != LOW) return false;

  uint32_t bits = 0;
  for (byte i = 0; i < 25; ++i) {
    uint8_t oldSREG = SREG;
    cli();
    digitalWrite(HX_CLOCK[ch], HIGH);
    delayMicroseconds(1);
    byte b = (i < 24) ? digitalRead(HX_DT[ch]) : 0;
    digitalWrite(HX_CLOCK[ch], LOW);
    SREG = oldSREG;
    if (i < 24) bits = (bits << 1) | b;
    delayMicroseconds(1);
  }
  if (bits & 0x800000UL) bits |= 0xFF000000UL;
  outRaw = (long)bits;
  return true;
}

bool captureAverageRaw(byte ch, long &avgOut, unsigned long timeoutMs = 1500UL) {
  long sum = 0;
  byte count = 0;
  unsigned long start = millis();
  while (count < 6 && (millis() - start < timeoutMs)) {
    long val;
    if (readHX711Raw(ch, val)) {
      if (val != 8388607L && val != -8388608L && val != 0L && val != -1L) {
        sum += val;
        count++;
        delay(12);
      }
    }
  }
  if (count > 0) {
    avgOut = sum / (long)count;
    return true;
  }
  // 실시간 루프에서 이미 정상 수신 중이었다면 직전 필터값 활용
  if (trays[ch].health == SENSOR_OK && trays[ch].rawFiltered != 0) {
    avgOut = trays[ch].rawFiltered;
    return true;
  }
  return false;
}

bool performTare(byte ch, bool saveEeprom = false, unsigned long timeoutMs = 1500UL) {
  if (ch >= NUM_TRAYS) return false;
  long avg;
  if (!captureAverageRaw(ch, avg, timeoutMs)) {
    trays[ch].health = SENSOR_ERROR;
    return false;
  }
  config.tareOffset[ch] = avg;
  config.tareValidMask |= (1 << ch);
  trays[ch].rawFiltered = avg;
  trays[ch].weightDg    = 0;
  trays[ch].present     = false;
  trays[ch].candidatePresent = false;
  trays[ch].health      = SENSOR_OK;
  trays[ch].lastSampleMs = millis();
  if (saveEeprom) saveConfigToEeprom();
  return true;
}

// ==============================================================================
// 10. 출력 장치(공용 LED 3채널, 건조 팬) 제어 및 이벤트 트리거
// ==============================================================================
void startSterilization(byte spaceIdx, const char* reqId = NULL) {
  if (spaceIdx >= NUM_SPACES) return;
  uint8_t sec = config.sterilSec[spaceIdx];
  if (sec != 5 && sec != 10 && sec != 20 && sec != 30) sec = 10;

  outputs[spaceIdx].sterilActive   = true;
  outputs[spaceIdx].sterilUntilMs  = millis() + ((unsigned long)sec * 1000UL);
  outputs[spaceIdx].manualOverride = false;
  if (reqId && reqId[0]) {
    strncpy(outputs[spaceIdx].activeReqId, reqId, sizeof(outputs[spaceIdx].activeReqId) - 1);
    outputs[spaceIdx].activeReqId[11] = '\0';
  } else {
    outputs[spaceIdx].activeReqId[0] = '\0';
  }
  stateDirty = true;
}

void stopSterilization(byte spaceIdx, PGM_P reasonP = NULL, bool completed = false) {
  if (spaceIdx >= NUM_SPACES || !outputs[spaceIdx].sterilActive) return;

  outputs[spaceIdx].sterilActive   = false;
  outputs[spaceIdx].sterilUntilMs  = 0;
  outputs[spaceIdx].manualOverride = false;
  if (outputs[spaceIdx].activeReqId[0] && reasonP) {
    sendAckP(outputs[spaceIdx].activeReqId, completed ? PSTR("completed") : PSTR("stopped"), reasonP);
  }
  outputs[spaceIdx].activeReqId[0] = '\0';
  stateDirty = true;
}

void startFan(uint8_t seconds, const char* reqId = NULL) {
  if (seconds == 0) return;
  fan.active  = true;
  fan.untilMs = millis() + ((unsigned long)seconds * 1000UL);
  digitalWrite(FAN_PIN, HIGH);
  if (reqId && reqId[0]) {
    strncpy(fan.activeReqId, reqId, sizeof(fan.activeReqId) - 1);
    fan.activeReqId[11] = '\0';
  } else {
    fan.activeReqId[0] = '\0';
  }
  stateDirty = true;
}

void stopFan(PGM_P reasonP = NULL, bool completed = false) {
  if (!fan.active) {
    digitalWrite(FAN_PIN, LOW);
    return;
  }
  fan.active  = false;
  fan.untilMs = 0;
  digitalWrite(FAN_PIN, LOW);
  if (fan.activeReqId[0] && reasonP) {
    sendAckP(fan.activeReqId, completed ? PSTR("completed") : PSTR("stopped"), reasonP);
  }
  fan.activeReqId[0] = '\0';
  stateDirty = true;
}

bool isSpaceItemPresent(byte spaceIdx) {
  if (spaceIdx < NUM_TRAYS) return (trays[spaceIdx].health == SENSOR_OK) && trays[spaceIdx].present;
  if (spaceIdx == 2) return (umbrella.health == SENSOR_OK) && umbrella.active;
  return false;
}

bool isSpaceSensorOk(byte spaceIdx) {
  if (spaceIdx < NUM_TRAYS) return trays[spaceIdx].health == SENSOR_OK;
  if (spaceIdx == 2) return umbrella.health == SENSOR_OK;
  return false;
}

void onSpaceItemChanged(byte spaceIdx, bool nowPresent) {
  if (spaceIdx >= NUM_SPACES) return;
  uint8_t flags = config.spaceFlags[spaceIdx];

  if (nowPresent) {
    // 거치 감지 시 일반 LED 즉시 소등
    outputs[spaceIdx].normalActive   = false;
    outputs[spaceIdx].manualOverride = false;

    if ((flags & CFG_STERIL_EN) && (flags & CFG_STERIL_DETECT) && !outputs[spaceIdx].sterilActive) {
      startSterilization(spaceIdx, NULL);
    }
    if (spaceIdx == 2 && config.fanDurationSec > 0) {
      startFan(config.fanDurationSec, NULL);
    }
  } else {
    outputs[spaceIdx].normalActive   = false;
    outputs[spaceIdx].manualOverride = false;
    if (outputs[spaceIdx].sterilActive) stopSterilization(spaceIdx, PSTR("item_removed"), false);
    if (spaceIdx == 2 && fan.active) stopFan(PSTR("item_removed"), false);
  }
  stateDirty = true;
}

void onDoorStateChanged(bool nowOpen) {
  unsigned long now = millis();
  if (nowOpen) {
    for (byte s = 0; s < NUM_SPACES; ++s) {
      if (isSpaceItemPresent(s)) {
        uint8_t flags = config.spaceFlags[s];
        if ((flags & CFG_LED_EN) && (flags & CFG_LED_DOOR)) {
          outputs[s].normalActive    = true;
          outputs[s].normalStartedMs = now;
          outputs[s].normalUntilMs   = now + NORMAL_MAX_MS;
          outputs[s].manualOverride  = false;
        }
        if ((flags & CFG_STERIL_EN) && (flags & CFG_STERIL_DOOR) && !outputs[s].sterilActive) {
          startSterilization(s, NULL);
        }
      }
    }
    blinkPhaseOn = true;
    lastBlinkToggleMs = now;
  } else {
    for (byte s = 0; s < NUM_SPACES; ++s) {
      if (!outputs[s].manualOverride) outputs[s].normalActive = false;
    }
  }
  stateDirty = true;
}

LedLogicalMode getLogicalLedMode(byte spaceIdx) {
  if (outputs[spaceIdx].sterilActive) return LED_STERIL;
  if (outputs[spaceIdx].normalActive) return LED_NORMAL_BLINK;
  return LED_OFF;
}

void evaluateOutputs() {
  unsigned long now = millis();

  if (fan.active && (int32_t)(now - fan.untilMs) >= 0) {
    stopFan(PSTR("duration_elapsed"), true);
  }

  if (now - lastBlinkToggleMs >= NORMAL_BLINK_MS) {
    lastBlinkToggleMs = now;
    blinkPhaseOn = !blinkPhaseOn;
  }

  for (byte s = 0; s < NUM_SPACES; ++s) {
    uint8_t flags = config.spaceFlags[s];

    if (outputs[s].sterilActive) {
      if ((int32_t)(now - outputs[s].sterilUntilMs) >= 0) {
        stopSterilization(s, PSTR("duration_elapsed"), true);
      } else if (!outputs[s].manualOverride) {
        if (!(flags & CFG_STERIL_EN)) stopSterilization(s, PSTR("disabled"), false);
        else if (!isSpaceItemPresent(s)) stopSterilization(s, PSTR("item_removed"), false);
      }
    }

    if (outputs[s].normalActive) {
      if ((int32_t)(now - outputs[s].normalUntilMs) >= 0) {
        outputs[s].normalActive   = false;
        outputs[s].manualOverride = false;
        stateDirty = true;
      } else if (!outputs[s].manualOverride) {
        if (!(flags & CFG_LED_EN) || !(flags & CFG_LED_DOOR) ||
            !door.active || !isSpaceItemPresent(s)) {
          outputs[s].normalActive = false;
          stateDirty = true;
        }
      }
    }

    // 0:OFF, 1:WHITE(120,120,120), 2:PURPLE(150,0,255)
    byte targetCode = 0;
    if (outputs[s].sterilActive) targetCode = 2;
    else if (outputs[s].normalActive && blinkPhaseOn) targetCode = 1;

    // 시리얼 명령 수신 중에는 WS2812B cli() 호출을 잠시 보류하여 UART 오버런 방지
    if (Serial.available() == 0 && serialRxLen == 0) {
      if (ledsNeedHardwareRefresh || outputs[s].currentStateCode != targetCode) {
        outputs[s].currentStateCode = targetCode;
        if (targetCode == 2) writeStripHardware(s, 150, 0, 255);
        else if (targetCode == 1) writeStripHardware(s, 120, 120, 120);
        else writeStripHardware(s, 0, 0, 0);
      }
    }
  }
  if (Serial.available() == 0 && serialRxLen == 0) {
    ledsNeedHardwareRefresh = false;
  }
}

// ==============================================================================
// 11. 센서 폴링 (현관문 A0, 우산 IR A3, 로드셀 CH1/CH2)
// ==============================================================================
void pollSensors() {
  // 시리얼 데이터를 읽는 도중에는 센서 cli() 폴링을 건너뛰고 즉시 수신 우선 처리
  if (Serial.available() > 0 || serialRxLen > 0) return;

  unsigned long now = millis();

  // 1) 현관문 (A0) - 가상 시뮬레이션(bit3) 중이 아닐 때만 물리 핀 폴링
  if (!(simMask & 0x08)) {
    byte rawDoor = digitalRead(DOOR_PIN);
    if (rawDoor != door.rawLevel) {
      door.rawLevel = rawDoor;
      door.lastDebounceMs = now;
    }
    if (now - door.lastDebounceMs >= DEBOUNCE_MS && door.stableLevel != door.rawLevel) {
      door.stableLevel = door.rawLevel;
      bool nowOpen = (door.stableLevel != DOOR_CLOSED_LEVEL);
      if (door.active != nowOpen) {
        door.active = nowOpen;
        onDoorStateChanged(nowOpen);
      }
    }
  }

  // 2) 우산 IR (A3) - 가상 시뮬레이션(bit2) 중이 아닐 때만 물리 핀 폴링
  if (!(simMask & 0x04)) {
    byte rawIr = digitalRead(IR_PIN);
    if (rawIr != umbrella.rawLevel) {
      umbrella.rawLevel = rawIr;
      umbrella.lastDebounceMs = now;
    }
    if (now - umbrella.lastDebounceMs >= DEBOUNCE_MS && umbrella.stableLevel != umbrella.rawLevel) {
      umbrella.stableLevel = umbrella.rawLevel;
      bool nowPresent = (umbrella.stableLevel == IR_DETECTED_LEVEL);
      umbrella.health = SENSOR_OK;
      if (umbrella.active != nowPresent) {
        umbrella.active = nowPresent;
        onSpaceItemChanged(2, nowPresent);
      }
    }
  }

  // 3) 로드셀 1·2번 칸 - 가상 시뮬레이션(bit0, bit1) 중이 아닐 때만 물리 폴링
  for (byte i = 0; i < NUM_TRAYS; ++i) {
    if (simMask & (1 << i)) continue;
    if (Serial.available() > 0) return;
    long raw;
    if (readHX711Raw(i, raw)) {
      if (raw == 8388607L || raw == -8388608L) {
        if (trays[i].health != SENSOR_ERROR) {
          trays[i].health = SENSOR_ERROR;
          stateDirty = true;
        }
        continue;
      }

      trays[i].lastSampleMs = now;
      if (!(config.tareValidMask & (1 << i))) {
        config.tareOffset[i] = raw;
        config.tareValidMask |= (1 << i);
        trays[i].rawFiltered = raw;
      } else {
        trays[i].rawFiltered = (trays[i].rawFiltered * 6L + raw * 4L) / 10L;
      }

      long deltaRaw = trays[i].rawFiltered - config.tareOffset[i];
      long sc10 = (labs(config.scaleX10[i]) > 0) ? config.scaleX10[i] : 250L;
      // weightDg (0.1g 단위) = (deltaRaw * 100) / scaleX10
      long dg = (deltaRaw * 100L) / sc10;

      if (!trays[i].present && labs(dg) < (config.deadzoneDg[i] * 8 / 10)) {
        if (deltaRaw > 3) config.tareOffset[i]++;
        else if (deltaRaw < -3) config.tareOffset[i]--;
      }

      if (dg < config.deadzoneDg[i]) dg = 0;
      else if (dg > 30000L) dg = 30000L;

      if (trays[i].present && labs(dg - trays[i].weightDg) >= 20) {
        stateDirty = true;
      }
      trays[i].weightDg = (int16_t)dg;

      if (trays[i].health != SENSOR_OK) {
        trays[i].health = SENSOR_OK;
        stateDirty = true;
      }

      bool nextPresent = trays[i].present;
      if (dg >= config.detectDg[i]) nextPresent = true;
      else if (dg <= config.releaseDg[i]) nextPresent = false;

      if (nextPresent != trays[i].candidatePresent) {
        trays[i].candidatePresent = nextPresent;
        trays[i].candidateSinceMs = now;
      } else if (trays[i].present != nextPresent && (now - trays[i].candidateSinceMs >= 250UL)) {
        trays[i].present = nextPresent;
        onSpaceItemChanged(i, nextPresent);
      }
    } else {
      if (now - trays[i].lastSampleMs > HX_TIMEOUT_MS && trays[i].health != SENSOR_ERROR) {
        trays[i].health = SENSOR_ERROR;
        if (outputs[i].sterilActive) stopSterilization(i, PSTR("sensor_error"), false);
        outputs[i].normalActive = false;
        stateDirty = true;
      }
    }
  }
}

// ==============================================================================
// 12. JSON 직렬화 및 경량 파서
// ==============================================================================
void printHealthStr(Stream &out, SensorHealth h) {
  if (h == SENSOR_OK) out.print(F("ok"));
  else if (h == SENSOR_ERROR) out.print(F("error"));
  else out.print(F("waiting"));
}

void printLedModeStr(Stream &out, LedLogicalMode m) {
  if (m == LED_STERIL) out.print(F("steril"));
  else if (m == LED_NORMAL_BLINK) out.print(F("normal"));
  else out.print(F("off"));
}

unsigned long remainingMs(bool active, unsigned long untilMs) {
  if (!active) return 0UL;
  long diff = (long)(untilMs - millis());
  return (diff > 0) ? (unsigned long)diff : 0UL;
}

bool serialLineReady = false;

void setCommMode(CommMode nextMode) {
  if (commMode == nextMode) return;
  commMode = nextMode;
  if (commMode == MODE_SERIAL) {
    bluetooth.end(); // 시리얼 통신 중에는 블루투스(SoftwareSerial) 완전 비활성화
    btRxLen = 0;
    btRxOverflow = false;
    Serial.println(F("\n>> [MODE] SERIAL TEST MODE (BT OFF | Type 'BT ON' to resume BT)"));
  } else {
    liveMonitorEnabled = false;
    bluetooth.begin(9600);
    lastStateSentMs = millis();
    stateDirty = true;
    Serial.println(F("\n>> [MODE] BLUETOOTH MODE ACTIVE (9600bps)"));
  }
}

void pumpSerialRx() {
  unsigned long now = millis();
  while (Serial.available() && !serialLineReady) {
    char c = (char)Serial.read();
    lastSerialRxMs = now;
    if (c == '\r') continue;
    if (c == '\n') {
      if (serialRxLen > 0) {
        serialRxBuf[serialRxLen] = '\0';
        serialLineReady = true;
      }
    } else if (c >= 32 && c <= 126) {
      if (commMode != MODE_SERIAL) {
        setCommMode(MODE_SERIAL);
      }
      if (serialRxLen + 1 < sizeof(serialRxBuf)) {
        serialRxBuf[serialRxLen++] = c;
      }
    }
  }
}

void sendStateSnapshot(Stream &out) {
  ++txSeq;
  out.print(F("{\"type\":\"state\",\"v\":1,\"boot\":"));
  out.print(config.bootCount);
  out.print(F(",\"seq\":"));
  out.print(txSeq);
  pumpSerialRx();

  out.print(F(",\"door\":{\"state\":\""));
  out.print(door.active ? F("open") : F("closed"));
  out.print(F("\",\"sensor\":\""));
  printHealthStr(out, door.health);
  out.print(F("\"},\"trays\":["));
  pumpSerialRx();

  for (byte i = 0; i < NUM_TRAYS; ++i) {
    if (i > 0) out.print(',');
    out.print(F("{\"id\":"));
    out.print(i + 1);
    out.print(F(",\"present\":"));
    out.print((trays[i].health == SENSOR_OK && trays[i].present) ? F("true") : F("false"));
    out.print(F(",\"weight\":"));
    printDeci(out, (trays[i].health == SENSOR_OK) ? trays[i].weightDg : 0);
    out.print(F(",\"sensor\":\""));
    printHealthStr(out, trays[i].health);
    out.print(F("\",\"led\":\""));
    printLedModeStr(out, getLogicalLedMode(i));
    out.print(F("\",\"steril_ms\":"));
    out.print(remainingMs(outputs[i].sterilActive, outputs[i].sterilUntilMs));
    out.print('}');
    pumpSerialRx();
  }

  out.print(F("],\"umbrella\":{\"present\":"));
  out.print((umbrella.health == SENSOR_OK && umbrella.active) ? F("true") : F("false"));
  out.print(F(",\"sensor\":\""));
  printHealthStr(out, umbrella.health);
  out.print(F("\",\"led\":\""));
  printLedModeStr(out, getLogicalLedMode(2));
  out.print(F("\",\"steril_ms\":"));
  out.print(remainingMs(outputs[2].sterilActive, outputs[2].sterilUntilMs));
  pumpSerialRx();

  out.print(F("},\"fan\":{\"active\":"));
  out.print(fan.active ? F("true") : F("false"));
  out.print(F(",\"rem_ms\":"));
  out.print(remainingMs(fan.active, fan.untilMs));
  out.println(F("}}"));
  pumpSerialRx();
}

void sendAckP(const char* reqId, PGM_P statusP, PGM_P reasonP) {
  Stream &out = (commMode == MODE_SERIAL) ? (Stream&)Serial : (Stream&)bluetooth;
  out.print(F("{\"type\":\"ack\",\"req\":\""));
  if (reqId && reqId[0]) out.print(reqId);
  else out.print(F("none"));
  out.print(F("\",\"status\":\""));
  out.print((const __FlashStringHelper*)statusP);
  out.print('"');
  if (reasonP) {
    out.print(F(",\"reason\":\""));
    out.print((const __FlashStringHelper*)reasonP);
    out.print('"');
  }
  out.println('}');
}

// snprintf 없이 PROGMEM 키를 직접 매칭하여 Flash/SRAM 절약
const char* findJsonValueP(const char* json, PGM_P keyP) {
  size_t keyLen = strlen_P(keyP);
  const char* p = json;
  while ((p = strchr(p, '"')) != NULL) {
    ++p;
    if (strncmp_P(p, keyP, keyLen) == 0 && p[keyLen] == '"') {
      p += keyLen + 1;
      while (*p == ' ' || *p == '\t') ++p;
      if (*p == ':') {
        ++p;
        while (*p == ' ' || *p == '\t') ++p;
        return p;
      }
    }
  }
  return NULL;
}

bool extractJsonStringP(const char* json, PGM_P keyP, char* outBuf, size_t maxLen) {
  const char* p = findJsonValueP(json, keyP);
  if (!p || *p != '"') return false;
  ++p;
  size_t i = 0;
  while (*p && *p != '"' && i + 1 < maxLen) outBuf[i++] = *p++;
  outBuf[i] = '\0';
  return (*p == '"');
}

bool extractJsonIntP(const char* json, PGM_P keyP, long &outVal) {
  const char* p = findJsonValueP(json, keyP);
  if (!p) return false;
  char* endPtr;
  outVal = strtol(p, &endPtr, 10);
  return (endPtr != p);
}

bool extractJsonBoolP(const char* json, PGM_P keyP, bool &outVal) {
  const char* p = findJsonValueP(json, keyP);
  if (!p) return false;
  if (strncmp_P(p, PSTR("true"), 4) == 0 || *p == '1') { outVal = true; return true; }
  if (strncmp_P(p, PSTR("false"), 5) == 0 || *p == '0') { outVal = false; return true; }
  return false;
}

uint8_t extractJsonWindows(const char* json, char outArr[2][14]) {
  const char* p = findJsonValueP(json, PSTR("windows"));
  if (!p || *p != '[') return 0;
  ++p;
  uint8_t count = 0;
  while (*p && *p != ']' && count < 2) {
    while (*p && *p != '"' && *p != ']') ++p;
    if (*p != '"') break;
    ++p;
    size_t i = 0;
    while (*p && *p != '"' && i < 13) outArr[count][i++] = *p++;
    outArr[count][i] = '\0';
    while (*p && *p != '"' && *p != ']') ++p;
    if (*p == '"') ++p;
    ++count;
  }
  return count;
}

void setFlagBit(uint8_t &flags, uint8_t mask, bool on) {
  if (on) flags |= mask;
  else flags &= ~mask;
}

void handleJsonMessage(const char* line, Stream &replyStream) {
  char type[14] = {0};
  char reqId[12] = {0};
  if (!extractJsonStringP(line, PSTR("type"), type, sizeof(type))) {
    sendAckP(NULL, PSTR("rejected"), PSTR("invalid_json"));
    return;
  }
  if (!extractJsonStringP(line, PSTR("req"), reqId, sizeof(reqId))) {
    extractJsonStringP(line, PSTR("request_id"), reqId, sizeof(reqId));
  }

  // 1) get_state
  if (strcmp_P(type, PSTR("get_state")) == 0) {
    if (reqId[0]) sendAckP(reqId, PSTR("accepted"));
    sendStateSnapshot(replyStream);
    lastStateSentMs = millis();
    stateDirty = false;
    return;
  }

  // 2) set_config
  if (strcmp_P(type, PSTR("set_config")) == 0) {
    long space = 0;
    if (!extractJsonIntP(line, PSTR("space"), space)) extractJsonIntP(line, PSTR("space_id"), space);
    if (space < 1 || space > 3) {
      sendAckP(reqId, PSTR("rejected"), PSTR("invalid_space"));
      return;
    }
    byte idx = (byte)(space - 1);
    uint8_t &flags = config.spaceFlags[idx];

    bool bVal;
    long iVal;
    if (extractJsonBoolP(line, PSTR("led_en"), bVal)) setFlagBit(flags, CFG_LED_EN, bVal);
    if (extractJsonBoolP(line, PSTR("led_door"), bVal)) setFlagBit(flags, CFG_LED_DOOR, bVal);
    if (extractJsonBoolP(line, PSTR("steril_en"), bVal) || extractJsonBoolP(line, PSTR("enabled"), bVal))
      setFlagBit(flags, CFG_STERIL_EN, bVal);
    if (extractJsonBoolP(line, PSTR("steril_detect"), bVal) || extractJsonBoolP(line, PSTR("on_detect"), bVal))
      setFlagBit(flags, CFG_STERIL_DETECT, bVal);
    if (extractJsonBoolP(line, PSTR("steril_door"), bVal) || extractJsonBoolP(line, PSTR("on_door"), bVal))
      setFlagBit(flags, CFG_STERIL_DOOR, bVal);

    if (extractJsonIntP(line, PSTR("steril_sec"), iVal) || extractJsonIntP(line, PSTR("duration_s"), iVal)) {
      if (iVal == 5 || iVal == 10 || iVal == 20 || iVal == 30) {
        config.sterilSec[idx] = (uint8_t)iVal;
      } else {
        config.sterilSec[idx] = 10;
        setFlagBit(flags, CFG_STERIL_EN, false);
      }
    }

    if (idx == 2 && extractJsonIntP(line, PSTR("fan_sec"), iVal)) {
      if (iVal == 0 || iVal == 15 || iVal == 30 || iVal == 60) {
        config.fanDurationSec = (uint8_t)iVal;
        if (iVal == 0 && fan.active) stopFan(PSTR("disabled"), false);
      }
    }

    if (!(flags & CFG_STERIL_EN) && outputs[idx].sterilActive) stopSterilization(idx, PSTR("disabled"), false);
    if (!(flags & CFG_LED_EN)) outputs[idx].normalActive = false;

    saveConfigToEeprom();
    sendAckP(reqId, PSTR("accepted"));
    stateDirty = true;
    return;
  }

  // 3) cmd
  if (strcmp_P(type, PSTR("cmd")) == 0 || strcmp_P(type, PSTR("command")) == 0) {
    char action[22] = {0};
    long space = 0;
    extractJsonStringP(line, PSTR("action"), action, sizeof(action));
    if (!extractJsonIntP(line, PSTR("space"), space)) extractJsonIntP(line, PSTR("space_id"), space);

    if (strcmp_P(action, PSTR("start_steril")) == 0 || strcmp_P(action, PSTR("start_sterilization")) == 0) {
      if (space < 1 || space > 3) { sendAckP(reqId, PSTR("rejected"), PSTR("invalid_space")); return; }
      byte idx = (byte)(space - 1);
      if (!(config.spaceFlags[idx] & CFG_STERIL_EN)) { sendAckP(reqId, PSTR("rejected"), PSTR("steril_disabled")); return; }
      if (!isSpaceSensorOk(idx)) { sendAckP(reqId, PSTR("rejected"), PSTR("sensor_error")); return; }
      if (!isSpaceItemPresent(idx)) { sendAckP(reqId, PSTR("rejected"), PSTR("no_item")); return; }
      if (outputs[idx].sterilActive) { sendAckP(reqId, PSTR("rejected"), PSTR("already_running")); return; }

      sendAckP(reqId, PSTR("accepted"));
      startSterilization(idx, reqId);
      return;
    }
    if (strcmp_P(action, PSTR("stop_steril")) == 0 || strcmp_P(action, PSTR("stop_sterilization")) == 0) {
      if (space < 1 || space > 3) { sendAckP(reqId, PSTR("rejected"), PSTR("invalid_space")); return; }
      sendAckP(reqId, PSTR("accepted"));
      stopSterilization((byte)(space - 1), PSTR("manual_stop"), false);
      return;
    }
    if (strcmp_P(action, PSTR("start_fan")) == 0) {
      long sec = config.fanDurationSec;
      extractJsonIntP(line, PSTR("sec"), sec);
      if (sec != 15 && sec != 30 && sec != 60) sec = (config.fanDurationSec > 0) ? config.fanDurationSec : 30;
      if (!isSpaceSensorOk(2)) { sendAckP(reqId, PSTR("rejected"), PSTR("sensor_error")); return; }
      if (!isSpaceItemPresent(2)) { sendAckP(reqId, PSTR("rejected"), PSTR("no_item")); return; }

      sendAckP(reqId, PSTR("accepted"));
      startFan((uint8_t)sec, reqId);
      return;
    }
    if (strcmp_P(action, PSTR("stop_fan")) == 0) {
      sendAckP(reqId, PSTR("accepted"));
      stopFan(PSTR("manual_stop"), false);
      return;
    }
    if (strcmp_P(action, PSTR("tare")) == 0) {
      if (space < 1 || space > 2) { sendAckP(reqId, PSTR("rejected"), PSTR("invalid_space")); return; }
      if (performTare((byte)(space - 1), true)) {
        sendAckP(reqId, PSTR("accepted"));
        stateDirty = true;
      } else {
        sendAckP(reqId, PSTR("rejected"), PSTR("sensor_error"));
      }
      return;
    }
    sendAckP(reqId, PSTR("rejected"), PSTR("unknown_action"));
    return;
  }

  // 4) weather
  if (strcmp_P(type, PSTR("weather")) == 0) {
    weatherLcd.receivedOnce = true;
    bool valid = true;
    extractJsonBoolP(line, PSTR("valid"), valid);
    if (!valid) {
      setWeatherNoData();
      sendAckP(reqId, PSTR("accepted"));
      refreshLcdWeather(true);
      return;
    }

    char iconStr[14] = {0};
    if (!extractJsonStringP(line, PSTR("icon"), iconStr, sizeof(iconStr))) {
      extractJsonStringP(line, PSTR("display_condition"), iconStr, sizeof(iconStr));
    }

    if (strcmp_P(iconStr, PSTR("rain")) == 0 || strcmp_P(iconStr, PSTR("shower")) == 0 || strcmp_P(iconStr, PSTR("rain-snow")) == 0) {
      setWeatherPreset(W_ICON_RAIN, PSTR("RAIN"), PSTR("14:00-18:00"), true);
    } else if (strcmp_P(iconStr, PSTR("snow")) == 0) {
      setWeatherPreset(W_ICON_SNOW, PSTR("SNOW"), PSTR("NO RAIN"), true);
    } else if (strcmp_P(iconStr, PSTR("cloudy")) == 0 || strcmp_P(iconStr, PSTR("partly-cloudy")) == 0) {
      setWeatherPreset(W_ICON_CLOUDY, PSTR("CLOUDY"), PSTR("NO RAIN/SNOW"), true);
    } else if (strcmp_P(iconStr, PSTR("sunny")) == 0 || strcmp_P(iconStr, PSTR("clear")) == 0) {
      setWeatherPreset(W_ICON_SUNNY, PSTR("SUNNY"), PSTR("NO RAIN/SNOW"), true);
    } else {
      setWeatherNoData();
      sendAckP(reqId, PSTR("accepted"));
      refreshLcdWeather(true);
      return;
    }

    char customLine1[14] = {0};
    if (extractJsonStringP(line, PSTR("line1"), customLine1, sizeof(customLine1)) && customLine1[0]) {
      strncpy(weatherLcd.line1, customLine1, 13);
      weatherLcd.line1[13] = '\0';
    }

    uint8_t winCnt = extractJsonWindows(line, weatherLcd.windows);
    if (winCnt > 0) weatherLcd.windowCount = winCnt;

    long ttlSec = 3600;
    extractJsonIntP(line, PSTR("ttl_s"), ttlSec);
    weatherLcd.receivedAtMs = millis();
    weatherLcd.ttlMs = (ttlSec > 0) ? ((unsigned long)ttlSec * 1000UL) : 0UL;
    weatherLcd.dirty = true;

    sendAckP(reqId, PSTR("accepted"));
    refreshLcdWeather(true);
    return;
  }

  sendAckP(reqId, PSTR("rejected"), PSTR("unknown_type"));
}

// ==============================================================================
// 13. 시리얼 캘리브레이션 & 수동 테스트 콘솔 (PROGMEM 최적화)
// ==============================================================================
void printCalibrationHelp() {
  Serial.println(F("\n=== [ AMATTA SERIAL TEST & CALIBRATION ] ==="));
  Serial.print(F(" CURRENT MODE: "));
  Serial.println((commMode == MODE_SERIAL) ? F("SERIAL TEST (BT OFF)") : F("BLUETOOTH (9600bps)"));
  Serial.println(F(" BT ON (or EXIT) : Resume Bluetooth Mode"));
  Serial.println(F(" STATUS | MON (M) | SENSOR (D) | BT SEND"));
  Serial.println(F(" CAL SHOW | TARE [1|2] | CAL <1|2> <g>"));
  Serial.println(F(" SCALE <1|2> <val> | TH <1|2> <det> <rel> | DZ <1|2> <g>"));
  Serial.println(F(" SAVE | RESET"));
  Serial.println(F(" LED <1|2|3|ALL> <STERIL|BLINK|OFF> [sec]"));
  Serial.println(F(" FAN ON [sec] | FAN OFF"));
  Serial.println(F(" LCD <WAIT|SUNNY|CLOUDY|RAIN|SNOW|NODATA> [w1] [w2]"));
  Serial.println(F(" SIM DOOR <OPEN|CLOSE> | SIM ITEM <1|2|3> <1|0>"));
  Serial.println(F("============================================"));
}

void printCalibrationStatus() {
  for (byte i = 0; i < NUM_TRAYS; ++i) {
    long delta = trays[i].rawFiltered - config.tareOffset[i];
    Serial.print(F("T#")); Serial.print(i + 1);
    Serial.print(F(" W:")); printDeci(Serial, trays[i].weightDg); Serial.print(F("g"));
    Serial.print(F(" Raw:")); Serial.print(trays[i].rawFiltered);
    Serial.print(F("(d:"));
    if (delta >= 0) Serial.print('+');
    Serial.print(delta);
    Serial.print(F(") Tare:")); Serial.print(config.tareOffset[i]);
    Serial.print(F(" Sc:")); printDeci(Serial, config.scaleX10[i]);
    Serial.print(F(" P:")); Serial.print(trays[i].present);
    Serial.print(F(" TH:")); printDeci(Serial, config.detectDg[i]);
    Serial.print('/'); printDeci(Serial, config.releaseDg[i]);
    Serial.print(F(" DZ:")); printDeci(Serial, config.deadzoneDg[i]);
    Serial.println();
  }
}

void printLiveMonitorLine() {
  Serial.print(F("[MON] D:")); Serial.print(door.active ? F("OPN") : F("CLS"));
  for (byte i = 0; i < NUM_TRAYS; ++i) {
    long delta = trays[i].rawFiltered - config.tareOffset[i];
    Serial.print(F(" | T")); Serial.print(i + 1); Serial.print(':');
    printDeci(Serial, trays[i].weightDg);
    Serial.print(F("g Raw:")); Serial.print(trays[i].rawFiltered);
    Serial.print('(');
    if (delta >= 0) Serial.print('+');
    Serial.print(delta);
    Serial.print(F(") P:")); Serial.print(trays[i].present);
  }
  Serial.print(F(" | U:")); Serial.print(umbrella.active);
  Serial.print(F(" F:")); Serial.print(fan.active);
  Serial.println();
}

void handleSerialCommand(char* line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (*line == '\0') return;

  if (*line == '{') {
    handleJsonMessage(line, Serial);
    return;
  }

  char* cmd = strtok(line, " \t");
  if (!cmd) return;
  toUpperStr(cmd);

  if (strcmp_P(cmd, PSTR("HELP")) == 0 || strcmp_P(cmd, PSTR("H")) == 0 || strcmp_P(cmd, PSTR("?")) == 0) {
    printCalibrationHelp();
  }
  else if (strcmp_P(cmd, PSTR("STATUS")) == 0) {
    sendStateSnapshot(Serial);
  }
  else if (strcmp_P(cmd, PSTR("MON")) == 0 || strcmp_P(cmd, PSTR("M")) == 0) {
    liveMonitorEnabled = !liveMonitorEnabled;
    lastMonitorPrintMs = millis();
    Serial.print(F(">> MON=")); Serial.println(liveMonitorEnabled);
  }
  else if (strcmp_P(cmd, PSTR("SENSOR")) == 0 || strcmp_P(cmd, PSTR("D")) == 0) {
    printLiveMonitorLine();
    printCalibrationStatus();
  }
  else if (strcmp_P(cmd, PSTR("TARE")) == 0 || strcmp_P(cmd, PSTR("T")) == 0 ||
           strcmp_P(cmd, PSTR("TARE1")) == 0 || strcmp_P(cmd, PSTR("TARE2")) == 0 ||
           strcmp_P(cmd, PSTR("T1")) == 0 || strcmp_P(cmd, PSTR("T2")) == 0) {
    char* a1 = strtok(NULL, " \t");
    int targetCh = -1; // -1이면 전체 채널(1, 2) 영점 수행
    size_t cmdLen = strlen(cmd);
    if (cmd[cmdLen - 1] == '1') targetCh = 0;
    else if (cmd[cmdLen - 1] == '2') targetCh = 1;
    else if (a1) {
      for (const char* p = a1; *p; ++p) {
        if (*p == '1') { targetCh = 0; break; }
        if (*p == '2') { targetCh = 1; break; }
      }
      if (targetCh < 0 && (strcmp_P(a1, PSTR("ALL")) != 0 && strcmp_P(a1, PSTR("all")) != 0)) {
        Serial.println(F(">> USE: TARE [1|2]"));
        return;
      }
    }

    byte c0 = (targetCh < 0) ? 0 : (byte)targetCh;
    byte c1 = (targetCh < 0) ? (NUM_TRAYS - 1) : (byte)targetCh;
    for (byte ch = c0; ch <= c1; ++ch) {
      if (performTare(ch, false)) {
        Serial.print(F(">> TARE CH")); Serial.print(ch + 1);
        Serial.print(F(" OK Base=")); Serial.println(config.tareOffset[ch]);
      } else {
        Serial.print(F(">> TARE CH")); Serial.print(ch + 1);
        Serial.print(F(" ERR: HX711 NO SIGNAL (DT=D")); Serial.print(HX_DT[ch]);
        Serial.print(F(" LVL=")); Serial.print(digitalRead(HX_DT[ch])); Serial.println(')');
      }
    }
    stateDirty = true;
  }
  else if (strcmp_P(cmd, PSTR("CAL")) == 0 || strcmp_P(cmd, PSTR("CALSHOW")) == 0) {
    char* a1 = strtok(NULL, " \t");
    if (a1) toUpperStr(a1);
    if (!a1 || strcmp_P(a1, PSTR("SHOW")) == 0 || strcmp_P(cmd, PSTR("CALSHOW")) == 0) {
      printCalibrationStatus();
      return;
    }
    int ch = atoi(a1) - 1;
    long knownDg = parseDeci(strtok(NULL, " \t"));
    if (ch < 0 || ch >= NUM_TRAYS || knownDg < 50) {
      Serial.println(F(">> USE: CAL <1|2> <g>"));
      return;
    }
    long avgRaw;
    if (!captureAverageRaw((byte)ch, avgRaw)) { Serial.println(F(">> SAMPLE ERR")); return; }
    long delta = avgRaw - config.tareOffset[ch];
    if (labs(delta) < 50) { Serial.println(F(">> DELTA TOO SMALL")); return; }
    config.scaleX10[ch] = (delta * 100L) / knownDg;
    Serial.print(F(">> CAL OK Sc=")); printDeci(Serial, config.scaleX10[ch]); Serial.println();
    stateDirty = true;
  }
  else if (strcmp_P(cmd, PSTR("SCALE")) == 0) {
    char* a1 = strtok(NULL, " \t");
    char* a2 = strtok(NULL, " \t");
    int ch = a1 ? (atoi(a1) - 1) : -1;
    long sc10 = parseDeci(a2);
    if (ch >= 0 && ch < NUM_TRAYS && labs(sc10) > 0) {
      config.scaleX10[ch] = sc10;
      Serial.print(F(">> SCALE CH")); Serial.print(ch + 1);
      Serial.print('='); printDeci(Serial, sc10); Serial.println();
    } else {
      Serial.println(F(">> USE: SCALE <1|2> <val>"));
    }
  }
  else if (strcmp_P(cmd, PSTR("TH")) == 0) {
    char* a1 = strtok(NULL, " \t");
    char* a2 = strtok(NULL, " \t");
    char* a3 = strtok(NULL, " \t");
    int ch = a1 ? (atoi(a1) - 1) : -1;
    long det = parseDeci(a2);
    long rel = parseDeci(a3);
    if (ch >= 0 && ch < NUM_TRAYS && det >= 10) {
      config.detectDg[ch]  = (int16_t)det;
      config.releaseDg[ch] = (int16_t)((rel > 0) ? rel : (det / 2));
      Serial.print(F(">> TH CH")); Serial.print(ch + 1);
      Serial.print('='); printDeci(Serial, config.detectDg[ch]);
      Serial.print('/'); printDeci(Serial, config.releaseDg[ch]); Serial.println(F("g"));
    } else {
      Serial.println(F(">> USE: TH <1|2> <detect_g> [release_g]"));
    }
  }
  else if (strcmp_P(cmd, PSTR("DZ")) == 0) {
    char* a1 = strtok(NULL, " \t");
    char* a2 = strtok(NULL, " \t");
    int ch = a1 ? (atoi(a1) - 1) : -1;
    long dz = parseDeci(a2);
    if (a2 && ch >= 0 && ch < NUM_TRAYS && dz >= 0 && dz <= 500) {
      config.deadzoneDg[ch] = (int16_t)dz;
      Serial.print(F(">> DZ CH")); Serial.print(ch + 1);
      Serial.print('='); printDeci(Serial, dz); Serial.println(F("g"));
    } else {
      Serial.println(F(">> USE: DZ <1|2> <g>"));
    }
  }
  else if (strcmp_P(cmd, PSTR("LED")) == 0 || strcmp_P(cmd, PSTR("LEDON")) == 0 || strcmp_P(cmd, PSTR("LEDOFF")) == 0) {
    char* tok1 = strtok(NULL, " \t");
    char* tok2 = strtok(NULL, " \t");
    char* tok3 = strtok(NULL, " \t");
    if (tok1) toUpperStr(tok1);
    if (tok2) toUpperStr(tok2);

    byte s0 = 0, s1 = NUM_SPACES - 1;
    char modeBuf[8];
    char* modeStr = NULL;
    char* secStr  = NULL;

    if (strcmp_P(cmd, PSTR("LEDON")) == 0) {
      copyStrP(modeBuf, PSTR("BLINK"), sizeof(modeBuf));
      modeStr = modeBuf;
    } else if (strcmp_P(cmd, PSTR("LEDOFF")) == 0) {
      copyStrP(modeBuf, PSTR("OFF"), sizeof(modeBuf));
      modeStr = modeBuf;
    } else if (!tok1) {
      Serial.println(F(">> USE: LED <1|2|3|ALL> <STERIL|BLINK|OFF> [sec]"));
      return;
    } else if (strcmp_P(tok1, PSTR("1")) == 0 || strcmp_P(tok1, PSTR("2")) == 0 || strcmp_P(tok1, PSTR("3")) == 0) {
      s0 = s1 = (byte)(*tok1 - '1');
      modeStr = tok2;
      secStr  = tok3;
    } else if (strcmp_P(tok1, PSTR("ALL")) == 0) {
      modeStr = tok2;
      secStr  = tok3;
    } else {
      // 채널 생략 시(예: led steril, led blink, led on, led off, led purple, led white) 전체 채널(ALL)에 적용
      modeStr = tok1;
      secStr  = tok2;
    }

    if (!modeStr) {
      Serial.println(F(">> USE: LED <1|2|3|ALL> <STERIL|BLINK|OFF> [sec]"));
      return;
    }

    int sec = secStr ? atoi(secStr) : 0;
    unsigned long now = millis();

    for (byte s = s0; s <= s1; ++s) {
      if (strcmp_P(modeStr, PSTR("STERIL")) == 0 || strcmp_P(modeStr, PSTR("PURPLE")) == 0 || strcmp_P(modeStr, PSTR("UV")) == 0) {
        int dur = (sec >= 1 && sec <= 600) ? sec : 10;
        config.spaceFlags[s] |= CFG_STERIL_EN;
        outputs[s].normalActive   = false;
        outputs[s].sterilActive   = true;
        outputs[s].sterilUntilMs  = now + ((unsigned long)dur * 1000UL);
        outputs[s].manualOverride = true;
        outputs[s].activeReqId[0] = '\0';
        outputs[s].currentStateCode = 2;
        writeStripHardware(s, 150, 0, 255);
      } else if (strcmp_P(modeStr, PSTR("BLINK")) == 0 || strcmp_P(modeStr, PSTR("NORMAL")) == 0 ||
                 strcmp_P(modeStr, PSTR("WHITE")) == 0 || strcmp_P(modeStr, PSTR("ON")) == 0 || strcmp_P(modeStr, PSTR("1")) == 0) {
        int dur = (sec >= 1 && sec <= 600) ? sec : 60;
        config.spaceFlags[s] |= (CFG_LED_EN | CFG_LED_DOOR);
        outputs[s].sterilActive    = false;
        outputs[s].normalActive    = true;
        outputs[s].normalStartedMs = now;
        outputs[s].normalUntilMs   = now + ((unsigned long)dur * 1000UL);
        outputs[s].manualOverride  = true;
        blinkPhaseOn = true;
        lastBlinkToggleMs = now;
        outputs[s].currentStateCode = 1;
        writeStripHardware(s, 120, 120, 120);
      } else if (strcmp_P(modeStr, PSTR("OFF")) == 0 || strcmp_P(modeStr, PSTR("0")) == 0 || strcmp_P(modeStr, PSTR("BLACK")) == 0) {
        outputs[s].sterilActive   = false;
        outputs[s].normalActive   = false;
        outputs[s].manualOverride = false;
        outputs[s].currentStateCode = 0;
        writeStripHardware(s, 0, 0, 0);
      } else {
        Serial.println(F(">> USE: LED <1|2|3|ALL> <STERIL|BLINK|OFF> [sec]"));
        return;
      }
    }
    ledsNeedHardwareRefresh = true;
    stateDirty = true;
    Serial.print(F(">> LED [")); Serial.print(s0 + 1);
    if (s1 > s0) { Serial.print('-'); Serial.print(s1 + 1); }
    Serial.print(F("] -> ")); Serial.println(modeStr);
  }
  else if (strcmp_P(cmd, PSTR("FAN")) == 0 || strcmp_P(cmd, PSTR("F")) == 0) {
    char* sub = strtok(NULL, " \t");
    char* secStr = strtok(NULL, " \t");
    if (strcmp_P(cmd, PSTR("F")) == 0 && !sub) {
      if (fan.active) {
        stopFan(PSTR("manual_stop"), false);
        Serial.println(F(">> FAN OFF"));
      } else {
        startFan(30, NULL);
        Serial.println(F(">> FAN ON (30s)"));
      }
      return;
    }
    if (!sub) {
      Serial.println(F(">> USE: FAN ON [sec] | FAN OFF"));
      return;
    }
    toUpperStr(sub);
    if (strcmp_P(sub, PSTR("ON")) == 0 || strcmp_P(sub, PSTR("1")) == 0) {
      int sec = secStr ? atoi(secStr) : 30;
      if (sec < 1 || sec > 240) sec = 30;
      startFan((uint8_t)sec, NULL);
      Serial.print(F(">> FAN ON (")); Serial.print(sec); Serial.println(F("s)"));
    } else if (strcmp_P(sub, PSTR("OFF")) == 0 || strcmp_P(sub, PSTR("0")) == 0) {
      stopFan(PSTR("manual_stop"), false);
      Serial.println(F(">> FAN OFF"));
    } else if (*sub >= '1' && *sub <= '9') {
      int sec = atoi(sub);
      if (sec < 1 || sec > 240) sec = 30;
      startFan((uint8_t)sec, NULL);
      Serial.print(F(">> FAN ON (")); Serial.print(sec); Serial.println(F("s)"));
    } else {
      Serial.println(F(">> USE: FAN ON [sec] | FAN OFF"));
    }
  }
  else if (strcmp_P(cmd, PSTR("LCD")) == 0) {
    char* sub = strtok(NULL, " \t");
    if (!sub) {
      Serial.println(F(">> USE: LCD <WAIT|SUNNY|CLOUDY|RAIN|SNOW|NODATA> [w1] [w2]"));
      return;
    }
    toUpperStr(sub);
    weatherLcd.receivedOnce = true;
    if (strcmp_P(sub, PSTR("WAIT")) == 0) initWeatherDisplayWait();
    else if (strcmp_P(sub, PSTR("SUNNY")) == 0 || strcmp_P(sub, PSTR("CLEAR")) == 0) setWeatherPreset(W_ICON_SUNNY, PSTR("SUNNY"), PSTR("NO RAIN/SNOW"), true);
    else if (strcmp_P(sub, PSTR("CLOUDY")) == 0) setWeatherPreset(W_ICON_CLOUDY, PSTR("CLOUDY"), PSTR("NO RAIN/SNOW"), true);
    else if (strcmp_P(sub, PSTR("SNOW")) == 0) setWeatherPreset(W_ICON_SNOW, PSTR("SNOW"), PSTR("NO RAIN"), true);
    else if (strcmp_P(sub, PSTR("RAIN")) == 0) {
      setWeatherPreset(W_ICON_RAIN, PSTR("RAIN"), PSTR("14:00-18:00"), true);
      char* w1 = strtok(NULL, " \t");
      char* w2 = strtok(NULL, " \t");
      if (w1) {
        strncpy(weatherLcd.windows[0], w1, 13); weatherLcd.windows[0][13] = '\0';
        if (w2) {
          strncpy(weatherLcd.windows[1], w2, 13); weatherLcd.windows[1][13] = '\0';
          weatherLcd.windowCount = 2;
        }
      }
    } else setWeatherNoData();
    refreshLcdWeather(true);
    Serial.print(F(">> LCD OK: ")); Serial.println(sub);
  }
  else if (strcmp_P(cmd, PSTR("SIM")) == 0) {
    char* target = strtok(NULL, " \t");
    char* a1     = strtok(NULL, " \t");
    char* a2     = strtok(NULL, " \t");
    if (!target) {
      Serial.println(F(">> USE: SIM DOOR <OPEN|CLOSE> | SIM ITEM <1|2|3> <1|0> | SIM OFF"));
      return;
    }
    toUpperStr(target);
    if (strcmp_P(target, PSTR("OFF")) == 0 || strcmp_P(target, PSTR("RESET")) == 0) {
      simMask = 0;
      Serial.println(F(">> SIM OFF (Real Sensors Active)"));
      return;
    }
    if (!a1) {
      Serial.println(F(">> USE: SIM DOOR <OPEN|CLOSE> | SIM ITEM <1|2|3> <1|0> | SIM OFF"));
      return;
    }
    toUpperStr(a1);
    if (strcmp_P(target, PSTR("DOOR")) == 0) {
      bool opn = (strcmp_P(a1, PSTR("OPEN")) == 0 || *a1 == '1');
      simMask |= 0x08;
      door.active = opn;
      onDoorStateChanged(opn);
      Serial.print(F(">> SIM DOOR=")); Serial.println(opn ? F("OPEN") : F("CLOSED"));
    } else if (strcmp_P(target, PSTR("ITEM")) == 0 && a2) {
      int sp = atoi(a1);
      bool pres = (*a2 != '0');
      if (sp >= 1 && sp <= 3) {
        simMask |= (1 << (sp - 1));
        if (sp == 1 || sp == 2) {
          trays[sp - 1].health = SENSOR_OK;
          trays[sp - 1].present = pres;
          trays[sp - 1].candidatePresent = pres;
          trays[sp - 1].weightDg = pres ? 1200 : 0;
          onSpaceItemChanged((byte)(sp - 1), pres);
        } else {
          umbrella.health = SENSOR_OK;
          umbrella.active = pres;
          onSpaceItemChanged(2, pres);
        }
        Serial.print(F(">> SIM ITEM #")); Serial.print(sp);
        Serial.print('='); Serial.println(pres);
      } else {
        Serial.println(F(">> USE: SIM ITEM <1|2|3> <1|0>"));
      }
    } else {
      Serial.println(F(">> USE: SIM DOOR <OPEN|CLOSE> | SIM ITEM <1|2|3> <1|0> | SIM OFF"));
    }
  }
  else if (strcmp_P(cmd, PSTR("BT")) == 0 || strcmp_P(cmd, PSTR("MODE")) == 0) {
    char* sub = strtok(NULL, " \t");
    if (sub) toUpperStr(sub);
    if (sub && (strcmp_P(sub, PSTR("ON")) == 0 || strcmp_P(sub, PSTR("BT")) == 0 || strcmp_P(sub, PSTR("1")) == 0)) {
      btPeerActive = true;
      setCommMode(MODE_BT);
    } else if (sub && strcmp_P(sub, PSTR("SERIAL")) == 0) {
      setCommMode(MODE_SERIAL);
    } else {
      // 시리얼 모드에서 수동 1회 BT 패킷 전송 테스트 (BT 또는 BT SEND)
      if (commMode == MODE_SERIAL) bluetooth.begin(9600);
      sendStateSnapshot(bluetooth);
      if (commMode == MODE_SERIAL) bluetooth.end();
      sendStateSnapshot(Serial);
    }
  }
  else if (strcmp_P(cmd, PSTR("EXIT")) == 0 || strcmp_P(cmd, PSTR("BTON")) == 0) {
    btPeerActive = true;
    setCommMode(MODE_BT);
  }
  else if (strcmp_P(cmd, PSTR("SAVE")) == 0) {
    saveConfigToEeprom();
    Serial.println(F(">> SAVED"));
  }
  else if (strcmp_P(cmd, PSTR("RESET")) == 0) {
    resetDefaultConfig();
    Serial.println(F(">> RESET OK"));
  }
  else {
    Serial.print(F(">> UNKNOWN CMD: '"));
    Serial.print(cmd);
    Serial.println(F("' (TYPE 'HELP')"));
  }
}

// ==============================================================================
// 14. 비차단 스트림 수신 루프
// ==============================================================================
void pollStreams() {
  unsigned long now = millis();

  // 1) PC 시리얼(115200bps) 입력 최우선 확인 -> 입력 감지 시 자동으로 MODE_SERIAL 전환
  pumpSerialRx();
  if (serialLineReady || (serialRxLen > 0 && (now - lastSerialRxMs > 120UL))) {
    serialRxBuf[serialRxLen] = '\0';
    serialRxLen = 0;
    serialLineReady = false;
    handleSerialCommand(serialRxBuf);
    return;
  }

  // 2) 시리얼 무입력 타임아웃(180초) 경과 시 기본 블루투스 모드(MODE_BT)로 자동 복귀
  if (commMode == MODE_SERIAL && !liveMonitorEnabled && serialRxLen == 0 &&
      (now - lastSerialRxMs >= SERIAL_MODE_TIMEOUT_MS)) {
    setCommMode(MODE_BT);
  }

  // 3) 블루투스 모드(MODE_BT)일 때만 HC-06 수신 처리 (시리얼 모드에서는 완전 비활성화)
  if (commMode == MODE_BT) {
    while (bluetooth.available()) {
      char c = (char)bluetooth.read();
      lastBtRxMs = now;
      btPeerActive = true;
      if (c == '\r') continue;
      if (c == '\n') {
        if (!btRxOverflow && btRxLen > 0) {
          btRxBuf[btRxLen] = '\0';
          handleJsonMessage(btRxBuf, bluetooth);
        } else if (btRxOverflow) {
          sendAckP(NULL, PSTR("rejected"), PSTR("frame_too_long"));
        }
        btRxLen = 0;
        btRxOverflow = false;
      } else {
        if (btRxLen + 1 < sizeof(btRxBuf)) btRxBuf[btRxLen++] = c;
        else btRxOverflow = true;
      }
    }
    if (btRxLen > 0 && (now - lastBtRxMs > 1500UL)) {
      btRxLen = 0;
      btRxOverflow = false;
    }
  }
}

// ==============================================================================
// 15. 초기 설정 (setup) 및 메인 루프 (loop)
// ==============================================================================
void setup() {
  Serial.begin(115200);
  commMode = MODE_BT;
  bluetooth.begin(9600);

  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);

  for (byte s = 0; s < NUM_SPACES; ++s) {
    pinMode(STRIP_PIN[s], OUTPUT);
    digitalWrite(STRIP_PIN[s], LOW);
    writeStripHardware(s, 0, 0, 0);
  }

  pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(IR_PIN, INPUT_PULLUP);

  loadConfigFromEeprom();

  unsigned long now = millis();
  door.rawLevel = door.stableLevel = digitalRead(DOOR_PIN);
  door.lastDebounceMs = now;
  door.active = (door.stableLevel != DOOR_CLOSED_LEVEL);
  door.health = SENSOR_OK;

  umbrella.rawLevel = umbrella.stableLevel = digitalRead(IR_PIN);
  umbrella.lastDebounceMs = now;
  umbrella.active = (umbrella.stableLevel == IR_DETECTED_LEVEL);
  umbrella.health = SENSOR_OK;

  for (byte i = 0; i < NUM_TRAYS; ++i) {
    pinMode(HX_DT[i], INPUT);
    pinMode(HX_CLOCK[i], OUTPUT);
    digitalWrite(HX_CLOCK[i], LOW);
    trays[i].health = SENSOR_WAITING;
    trays[i].lastSampleMs = now;
    performTare(i, false, 250UL); // 부팅 시 빠른 초기 영점 확인 (최대 250ms)
  }

  Wire.begin();
  Wire.setWireTimeout(3000, true);
  lcdAvailable = initI2CLcd();
  initWeatherDisplayWait();
  refreshLcdWeather(true);

  Serial.println(F("AMATTA READY [DEFAULT: BT MODE | Type any cmd for SERIAL MODE]"));
  stateDirty = true;
}

void loop() {
  pollStreams();
  pollSensors();
  evaluateOutputs();
  refreshLcdWeather(false);

  unsigned long now = millis();
  // 기본 블루투스 모드(MODE_BT)일 때만 블루투스로 상태 전송 (시리얼 모드에서는 BT 전송 안 함)
  if (commMode == MODE_BT) {
    if (stateDirty || (btPeerActive && (now - lastStateSentMs >= STATE_HEARTBEAT_MS))) {
      stateDirty = false;
      lastStateSentMs = now;
      sendStateSnapshot(bluetooth);
    }
  } else {
    // 시리얼 모드에서는 자동 BT 송신을 하지 않음
    stateDirty = false;
  }

  if (liveMonitorEnabled && (now - lastMonitorPrintMs >= 500UL)) {
    lastMonitorPrintMs = now;
    printLiveMonitorLine();
  }
}
