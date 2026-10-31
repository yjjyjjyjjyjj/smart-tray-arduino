/*
 * ==============================================================================
 * 프로젝트: 아마따 (AMATTA) - 스마트 트레이 Arduino UNO R3 메인 펌웨어 (v1.0)
 * ==============================================================================
 *
 * [핵심 사양 및 확정 요구사항 반영]
 * 1. 하드웨어 핀 배치:
 *    - 1번 트레이 로드셀 (HX711): DT = D2, SCK = D3
 *    - 2번 트레이 로드셀 (HX711): DT = D4, SCK = D5
 *    - 3번 우산 공간 적외선 센서 (IR): OUT = A3 (감지 시 LOW)
 *    - 현관문 자석 도어센서 (리드스위치): A0 (INPUT_PULLUP, 닫힘 LOW / 열림 HIGH)
 *      (* 미사용 뚜껑 A1, 창문 A2 도어센서 제외)
 *    - 공간별 공용 WS2812B LED 바 (각 8구): 1번칸 = D8, 2번칸 = D9, 3번(우산)칸 = D10
 *      (외부 라이브러리 불필요, 순수 AVR 16MHz 고속 어셈블리 드라이버 내장)
 *    - 우산 건조 쿨링팬: D11 (HIGH = ON, LOW = OFF)
 *    - 블루투스 모듈 (HC-06): RX = D12(HC-06 TX), TX = D13(HC-06 RX, 분압) / 9600bps
 *    - 1602 I2C LCD: SDA = A4, SCL = A5 (순수 Wire.h 제어)
 *
 * 2. 동작 및 제어 정책:
 *    - [물건/우산 거치 감지]: 일반 LED 점등 동작 없이 즉시 소등 유지 + 앱으로 즉각 상태(state) 패킷 전송.
 *    - [공유 LED 우선순위]:
 *      1순위) 살균 점등("steril"): 보라색 RGB(150, 0, 255) 연속 점등 (5/10/20/30초 타이머)
 *      2순위) 일반 점등("normal"): 현관문 열림 시(led_en && led_door) 물건이 남아 있는 칸만
 *             백색 RGB(120, 120, 120) 1.5초 주기 점멸 (1500ms ON / 1500ms OFF).
 *             물건을 수거하거나, 새로 거치하거나, 현관문이 닫히면 즉시 소등.
 *    - [자동 실행 및 중지]:
 *      앱에서 내려보낸 설정값(EEPROM 저장)에 따라 거치 시/문 열림 시 살균 및 우산 건조 팬을
 *      아두이노가 자체 구동하며, 내부 상태 변경 즉시 앱으로 JSON 패킷을 전송.
 *      살균/팬 동작 중 물건이나 우산이 제거되면 즉시 소등/정지하고 앱에 통지.
 *    - [시리얼 캘리브레이션]:
 *      USB 시리얼 모니터(115200bps)로 실시간 영점(TARE), 분동 보정(CAL), 감도(TH),
 *      스케일(SCALE) 조정 및 EEPROM 영구 저장 지원.
 *    - [1602 LCD 날씨 표시]:
 *      좌측 2x2 커스텀 도트 아이콘(해/구름/비/눈/대기) + 우측 13칸 영문 날씨 및 비 예보 시간대.
 *      기상 정보 수신 전에는 대기 아이콘과 함께 "WAITING..." / "WEATHER INFO" 표시.
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
#include <math.h>

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

const unsigned long DEBOUNCE_MS       = 40UL;
const unsigned long STATE_HEARTBEAT_MS = 1000UL; // 1초 주기 상태 스냅샷 전송
const unsigned long NORMAL_BLINK_MS    = 1500UL; // 일반 백색 LED 1.5초 점멸 주기
const unsigned long NORMAL_MAX_MS      = 60000UL;// 문 열림 일반 점등 최대 유지 시간 (60초)
const unsigned long HX_TIMEOUT_MS      = 1500UL; // 로드셀 응답 타임아웃

SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN);

// ==============================================================================
// 2. EEPROM 저장용 설정 및 보정 데이터 구조체
// ==============================================================================
const uint16_t EEPROM_MAGIC = 0xA410;

struct SpaceConfig {
  bool ledEnabled;       // 일반 LED 활성화 여부
  bool ledOnDoor;        // 문 열림 시 미수거 알림 점멸 여부
  bool sterilEnabled;    // 보라색 살균 LED 활성화 여부
  bool sterilOnDetect;   // 물건/우산 거치 시 자동 살균 시작
  bool sterilOnDoor;     // 문 열림 시 자동 살균 시작
  uint8_t sterilSec;     // 살균 시간 (5, 10, 20, 30초)
};

struct PersistentData {
  uint16_t magic;
  uint16_t bootCount;
  long tareOffset[NUM_TRAYS];     // 로드셀 영점 Raw 기준값
  float scaleFactor[NUM_TRAYS];   // 1g당 Raw 변화량 (Scale factor)
  float detectGrams[NUM_TRAYS];   // 물건 감지 문턱값 (기본 15.0g)
  float releaseGrams[NUM_TRAYS];  // 물건 해제 문턱값 (히스테리시스, 기본 8.0g)
  float deadzoneGrams[NUM_TRAYS]; // 0.0g 고정 데드존 (기본 3.0g)
  bool tareValid[NUM_TRAYS];      // 영점 저장 여부
  SpaceConfig spaces[NUM_SPACES]; // 1, 2, 3번 공간 설정
  uint8_t fanDurationSec;         // 우산 건조 팬 동작 시간 (0, 15, 30, 60초)
  uint16_t crc;
};

PersistentData config;
uint32_t txSeq = 0;
bool stateDirty = false;          // 내부 값 변경 시 즉시 앱 전송 플래그
unsigned long lastStateSentMs = 0;

// ==============================================================================
// 3. 센서 및 출력 런타임 상태 구조체
// ==============================================================================
enum SensorHealth : uint8_t { SENSOR_WAITING = 0, SENSOR_OK = 1, SENSOR_ERROR = 2 };
enum LedLogicalMode : uint8_t { LED_OFF = 0, LED_NORMAL_BLINK = 1, LED_STERIL = 2 };

struct TrayRuntime {
  long rawFiltered;
  float weightGrams;
  bool present;
  SensorHealth health;
  unsigned long lastSampleMs;
  unsigned long candidateSinceMs;
  bool candidatePresent;
};

struct UmbrellaRuntime {
  byte rawLevel;
  byte stableLevel;
  unsigned long lastDebounceMs;
  bool present;
  SensorHealth health;
};

struct DoorRuntime {
  byte rawLevel;
  byte stableLevel;
  unsigned long lastDebounceMs;
  bool isOpen;
  SensorHealth health;
};

struct SpaceOutputRuntime {
  bool normalActive;               // 문 열림 미수거 백색 점멸 활성 여부
  unsigned long normalStartedMs;   // 일반 점멸 시작 시각
  bool sterilActive;               // 보라색 살균 활성 여부
  unsigned long sterilUntilMs;     // 살균 종료 예정 시각 (millis)
  char activeReqId[16];            // 수동 명령 요청 ID 보관
  byte currentR, currentG, currentB;
};

struct FanRuntime {
  bool active;
  unsigned long untilMs;
  char activeReqId[16];
};

TrayRuntime trays[NUM_TRAYS];
UmbrellaRuntime umbrella;
DoorRuntime door;
SpaceOutputRuntime outputs[NUM_SPACES];
FanRuntime fan;

// 일반 LED 백색 점멸 타이머 (1.5초 ON / 1.5초 OFF)
bool blinkPhaseOn = true;
unsigned long lastBlinkToggleMs = 0;
bool ledsNeedHardwareRefresh = true;

// ==============================================================================
// 4. 1602 I2C LCD 및 2x2 날씨 도트 아이콘 (PROGMEM)
// ==============================================================================
byte lcdAddress = 0x27;
bool lcdAvailable = false;

enum WeatherIconType : uint8_t {
  W_ICON_WAITING = 0, // 기상 정보 대기중 아이콘 (시계/전파 모양)
  W_ICON_SUNNY   = 1, // 해 (SUNNY)
  W_ICON_CLOUDY  = 2, // 구름 (CLOUDY)
  W_ICON_RAIN    = 3, // 비 (RAIN)
  W_ICON_SNOW    = 4, // 눈 (SNOW)
  W_ICON_NONE    = 5  // 수신 실패/만료 (NO DATA)
};

// 각 큰 아이콘은 좌상(0), 우상(1), 좌하(2), 우하(3) 4개의 5x8 도트 문자로 구성 (총 32바이트)
const byte WEATHER_ICONS[6][4][8] PROGMEM = {
  // 0: W_ICON_WAITING (시계/대기 아이콘)
  {
    {0b00111, 0b01000, 0b10000, 0b10001, 0b10001, 0b10001, 0b10001, 0b01000},
    {0b11100, 0b00010, 0b00001, 0b10001, 0b10001, 0b10001, 0b11101, 0b00010},
    {0b01000, 0b10000, 0b10000, 0b10000, 0b01000, 0b00111, 0b00000, 0b00000},
    {0b00010, 0b00001, 0b00001, 0b00001, 0b00010, 0b11100, 0b00000, 0b00000}
  },
  // 1: W_ICON_SUNNY (해 아이콘)
  {
    {0b00100, 0b10100, 0b01001, 0b00011, 0b11011, 0b00011, 0b01001, 0b10100},
    {0b00100, 0b00101, 0b10010, 0b11000, 0b11011, 0b11000, 0b10010, 0b00101},
    {0b10100, 0b01001, 0b00011, 0b11011, 0b00011, 0b01001, 0b10100, 0b00100},
    {0b00101, 0b10010, 0b11000, 0b11011, 0b11000, 0b10010, 0b00101, 0b00100}
  },
  // 2: W_ICON_CLOUDY (구름 아이콘)
  {
    {0b00000, 0b00000, 0b00011, 0b00111, 0b01111, 0b11111, 0b11111, 0b01111},
    {0b00000, 0b00000, 0b11000, 0b11100, 0b11110, 0b11111, 0b11111, 0b11110},
    {0b01111, 0b00111, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000},
    {0b11110, 0b11100, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000, 0b00000}
  },
  // 3: W_ICON_RAIN (비 아이콘)
  {
    {0b00000, 0b00011, 0b00111, 0b01111, 0b11111, 0b11111, 0b01111, 0b00000},
    {0b00000, 0b11000, 0b11100, 0b11110, 0b11111, 0b11111, 0b11110, 0b00000},
    {0b00101, 0b01010, 0b10100, 0b00101, 0b01010, 0b10100, 0b00000, 0b00000},
    {0b01010, 0b10100, 0b01010, 0b01010, 0b10100, 0b01000, 0b00000, 0b00000}
  },
  // 4: W_ICON_SNOW (눈 아이콘)
  {
    {0b00000, 0b00011, 0b00111, 0b01111, 0b11111, 0b11111, 0b01111, 0b00000},
    {0b00000, 0b11000, 0b11100, 0b11110, 0b11111, 0b11111, 0b11110, 0b00000},
    {0b01010, 0b00100, 0b01010, 0b00000, 0b01010, 0b00100, 0b01010, 0b00000},
    {0b01010, 0b00100, 0b01010, 0b00000, 0b01010, 0b00100, 0b01010, 0b00000}
  },
  // 5: W_ICON_NONE (정보 없음 / 느낌표 박스)
  {
    {0b01111, 0b10000, 0b10000, 0b10001, 0b10001, 0b10001, 0b10001, 0b10000},
    {0b11110, 0b00001, 0b00001, 0b10001, 0b10001, 0b10001, 0b10001, 0b00001},
    {0b10000, 0b10001, 0b10001, 0b10000, 0b10000, 0b01111, 0b00000, 0b00000},
    {0b00001, 0b10001, 0b10001, 0b00001, 0b00001, 0b11110, 0b00000, 0b00000}
  }
};

struct WeatherDisplayState {
  bool receivedOnce;               // 부팅 후 날씨 패킷을 한 번이라도 받았는지 여부
  bool valid;                      // 현재 날씨 데이터 유효 여부
  WeatherIconType icon;            // 현재 아이콘 종류
  WeatherIconType loadedIcon;      // LCD CGRAM에 로드된 아이콘 종류
  char line1[14];                  // 우측 1행 텍스트 (최대 13자)
  char windows[3][14];             // 우측 2행 순환 텍스트 (최대 3개, 각 13자)
  uint8_t windowCount;
  uint8_t currentWindowIdx;
  unsigned long lastWindowCycleMs;
  unsigned long receivedAtMs;
  unsigned long ttlMs;             // 만료 시간 (0이면 만료 없음)
  bool dirty;
};

WeatherDisplayState weatherLcd;

// 수신 버퍼
char btRxBuf[220];
byte btRxLen = 0;
bool btRxOverflow = false;
unsigned long lastBtRxMs = 0;

char serialRxBuf[96];
byte serialRxLen = 0;
unsigned long lastSerialRxMs = 0;

// 함수 프로토타입 선언
void sendAck(const char* reqId, const char* status, const char* reason = NULL);
void sendStateSnapshot(Stream &out);
void evaluateOutputs();
void refreshLcdWeather(bool force = false);

// ==============================================================================
// 5. 순수 AVR 고속 WS2812B 드라이버 (16MHz 정밀 타이밍)
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
    buf[i * 3 + 0] = g; // WS2812B는 GRB 순서
    buf[i * 3 + 1] = r;
    buf[i * 3 + 2] = b;
  }
  ws2812Send(STRIP_PIN[spaceIdx], buf, sizeof(buf));
  if (r == 0 && g == 0 && b == 0) {
    digitalWrite(STRIP_PIN[spaceIdx], LOW);
  }
}

// ==============================================================================
// 6. 1602 I2C LCD 드라이버 및 2x2 날씨 렌더링
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
  byte rowOffsets[] = {0x00, 0x40};
  lcdCommand(0x80 | (col + rowOffsets[row]));
}

void lcdLoadWeatherIcon(WeatherIconType iconType) {
  if (!lcdAvailable || iconType > W_ICON_NONE) return;
  for (byte slot = 0; slot < 4; ++slot) {
    lcdCommand(0x40 | (slot << 3));
    for (byte row = 0; row < 8; ++row) {
      byte pattern = pgm_read_byte(&WEATHER_ICONS[iconType][slot][row]);
      lcdSendByte(pattern, 1);
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
    if (Wire.endTransmission() == 0) {
      lcdAddress = 0x3F;
    } else {
      return false;
    }
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

void initWeatherDisplayWait() {
  weatherLcd.receivedOnce = false;
  weatherLcd.valid = false;
  weatherLcd.icon = W_ICON_WAITING;
  weatherLcd.loadedIcon = (WeatherIconType)255;
  strncpy(weatherLcd.line1, "WAITING...", sizeof(weatherLcd.line1) - 1);
  weatherLcd.line1[13] = '\0';
  strncpy(weatherLcd.windows[0], "WEATHER INFO", sizeof(weatherLcd.windows[0]) - 1);
  weatherLcd.windows[0][13] = '\0';
  weatherLcd.windowCount = 1;
  weatherLcd.currentWindowIdx = 0;
  weatherLcd.lastWindowCycleMs = millis();
  weatherLcd.ttlMs = 0;
  weatherLcd.dirty = true;
}

void setWeatherNoData() {
  weatherLcd.valid = false;
  weatherLcd.icon = W_ICON_NONE;
  strncpy(weatherLcd.line1, "NO DATA", sizeof(weatherLcd.line1) - 1);
  weatherLcd.line1[13] = '\0';
  weatherLcd.windows[0][0] = '\0';
  weatherLcd.windowCount = 1;
  weatherLcd.currentWindowIdx = 0;
  weatherLcd.ttlMs = 0;
  weatherLcd.dirty = true;
}

void refreshLcdWeather(bool force) {
  if (!lcdAvailable) return;
  unsigned long now = millis();

  // 1) 날씨 유효시간(TTL) 만료 검사
  if (weatherLcd.receivedOnce && weatherLcd.valid && weatherLcd.ttlMs > 0) {
    if (now - weatherLcd.receivedAtMs >= weatherLcd.ttlMs) {
      setWeatherNoData();
    }
  }

  // 2) 비 예보 시간대가 여러 개일 때 3초 간격 순환
  if (weatherLcd.valid && weatherLcd.windowCount > 1) {
    if (now - weatherLcd.lastWindowCycleMs >= 3000UL) {
      weatherLcd.lastWindowCycleMs = now;
      weatherLcd.currentWindowIdx = (weatherLcd.currentWindowIdx + 1) % weatherLcd.windowCount;
      weatherLcd.dirty = true;
    }
  }

  if (!force && !weatherLcd.dirty) return;
  weatherLcd.dirty = false;

  // 아이콘이 변경되었으면 CGRAM 0~3번 슬롯 교체
  if (weatherLcd.loadedIcon != weatherLcd.icon) {
    lcdLoadWeatherIcon(weatherLcd.icon);
  }

  // 1행: [좌상(0)][우상(1)][공백][line1 (13칸)] -> 정확히 16칸
  lcdSetCursor(0, 0);
  lcdChar(0);
  lcdChar(1);
  lcdChar(' ');
  lcdPrintPadded13(weatherLcd.line1);

  // 2행: [좌하(2)][우하(3)][공백][window (13칸)] -> 정확히 16칸
  lcdSetCursor(0, 1);
  lcdChar(2);
  lcdChar(3);
  lcdChar(' ');
  const char* line2 = (weatherLcd.windowCount > 0) ? weatherLcd.windows[weatherLcd.currentWindowIdx] : "";
  lcdPrintPadded13(line2);
}

// ==============================================================================
// 7. EEPROM 설정 관리 (CRC16 검증 포함)
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
    config.tareOffset[i]    = 0;
    config.scaleFactor[i]   = 25.0f; // 기본 스케일 (시리얼 CAL 명령으로 정밀 보정 가능)
    config.detectGrams[i]   = 15.0f; // 15g 이상 물건 있음
    config.releaseGrams[i]  = 8.0f;  // 8g 이하 물건 없음
    config.deadzoneGrams[i] = 3.0f;  // 3g 이내 흔들림 0.0g 고정
    config.tareValid[i]     = false;
  }

  for (byte s = 0; s < NUM_SPACES; ++s) {
    config.spaces[s].ledEnabled     = true;
    config.spaces[s].ledOnDoor      = true;
    config.spaces[s].sterilEnabled  = false;
    config.spaces[s].sterilOnDetect = false;
    config.spaces[s].sterilOnDoor   = false;
    config.spaces[s].sterilSec      = 10;
  }
  config.fanDurationSec = 0;
  saveConfigToEeprom();
}

void loadConfigFromEeprom() {
  EEPROM.get(0, config);
  bool ok = (config.magic == EEPROM_MAGIC) && (config.crc == calcConfigCrc(config));
  for (byte i = 0; i < NUM_TRAYS; ++i) {
    if (!isfinite(config.scaleFactor[i]) || fabs(config.scaleFactor[i]) < 0.01f) ok = false;
    if (!isfinite(config.detectGrams[i]) || config.detectGrams[i] < 1.0f) ok = false;
  }
  if (!ok) {
    resetDefaultConfig();
  } else {
    config.bootCount++;
    saveConfigToEeprom();
  }
}

// ==============================================================================
// 8. HX711 로드셀 측정 및 보정 (시리얼 캘리브레이션 지원)
// ==============================================================================
bool readHX711Raw(byte ch, long &outRaw) {
  if (digitalRead(HX_DT[ch]) != LOW) return false;

  uint32_t bits = 0;
  for (byte i = 0; i < 25; ++i) {
    uint8_t oldSREG = SREG;
    cli();
    digitalWrite(HX_CLOCK[ch], HIGH);
    delayMicroseconds(4);
    if (i < 24) bits = (bits << 1) | digitalRead(HX_DT[ch]);
    digitalWrite(HX_CLOCK[ch], LOW);
    SREG = oldSREG;
    delayMicroseconds(4);
  }
  if (bits & 0x800000UL) bits |= 0xFF000000UL;
  outRaw = (long)bits;
  return true;
}

// 16샘플 절사평균으로 정밀 영점/평균 Raw 측정
bool captureAverageRaw(byte ch, long &avgOut, unsigned long timeoutMs = 3000UL) {
  long samples[16];
  byte count = 0;
  unsigned long start = millis();
  while (count < 16 && (millis() - start < timeoutMs)) {
    long val;
    if (readHX711Raw(ch, val)) {
      if (val != 8388607L && val != -8388608L) {
        samples[count++] = val;
        delay(15);
      }
    }
  }
  if (count < 16) return false;

  for (byte i = 0; i < 15; ++i) {
    for (byte j = i + 1; j < 16; ++j) {
      if (samples[i] > samples[j]) {
        long tmp = samples[i];
        samples[i] = samples[j];
        samples[j] = tmp;
      }
    }
  }
  int64_t sum = 0;
  for (byte i = 2; i < 14; ++i) sum += samples[i];
  avgOut = (long)(sum / 12);
  return true;
}

bool performTare(byte ch, bool saveEeprom = false) {
  if (ch >= NUM_TRAYS) return false;
  long avg;
  if (!captureAverageRaw(ch, avg)) {
    trays[ch].health = SENSOR_ERROR;
    return false;
  }
  config.tareOffset[ch] = avg;
  config.tareValid[ch]  = true;
  trays[ch].rawFiltered = avg;
  trays[ch].weightGrams = 0.0f;
  trays[ch].present     = false;
  trays[ch].candidatePresent = false;
  trays[ch].health      = SENSOR_OK;
  trays[ch].lastSampleMs = millis();
  if (saveEeprom) saveConfigToEeprom();
  return true;
}

// ==============================================================================
// 9. 출력 장치(공용 LED 3채널, 건조 팬) 제어 및 이벤트 트리거
// ==============================================================================
void startSterilization(byte spaceIdx, const char* reqId = NULL) {
  if (spaceIdx >= NUM_SPACES) return;
  uint8_t sec = config.spaces[spaceIdx].sterilSec;
  if (sec != 5 && sec != 10 && sec != 20 && sec != 30) sec = 10;

  outputs[spaceIdx].sterilActive  = true;
  outputs[spaceIdx].sterilUntilMs = millis() + ((unsigned long)sec * 1000UL);
  if (reqId && reqId[0]) {
    strncpy(outputs[spaceIdx].activeReqId, reqId, sizeof(outputs[spaceIdx].activeReqId) - 1);
    outputs[spaceIdx].activeReqId[15] = '\0';
  } else {
    outputs[spaceIdx].activeReqId[0] = '\0';
  }
  stateDirty = true;
}

void stopSterilization(byte spaceIdx, const char* reason = NULL) {
  if (spaceIdx >= NUM_SPACES) return;
  if (!outputs[spaceIdx].sterilActive) return;

  outputs[spaceIdx].sterilActive  = false;
  outputs[spaceIdx].sterilUntilMs = 0;
  if (outputs[spaceIdx].activeReqId[0] && reason) {
    sendAck(outputs[spaceIdx].activeReqId,
            strcmp(reason, "duration_elapsed") == 0 ? "completed" : "stopped",
            reason);
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
    fan.activeReqId[15] = '\0';
  } else {
    fan.activeReqId[0] = '\0';
  }
  stateDirty = true;
}

void stopFan(const char* reason = NULL) {
  if (!fan.active) {
    digitalWrite(FAN_PIN, LOW);
    return;
  }
  fan.active  = false;
  fan.untilMs = 0;
  digitalWrite(FAN_PIN, LOW);
  if (fan.activeReqId[0] && reason) {
    sendAck(fan.activeReqId,
            strcmp(reason, "duration_elapsed") == 0 ? "completed" : "stopped",
            reason);
  }
  fan.activeReqId[0] = '\0';
  stateDirty = true;
}

bool isSpaceItemPresent(byte spaceIdx) {
  if (spaceIdx < NUM_TRAYS) {
    return (trays[spaceIdx].health == SENSOR_OK) && trays[spaceIdx].present;
  } else if (spaceIdx == 2) {
    return (umbrella.health == SENSOR_OK) && umbrella.present;
  }
  return false;
}

bool isSpaceSensorOk(byte spaceIdx) {
  if (spaceIdx < NUM_TRAYS) return trays[spaceIdx].health == SENSOR_OK;
  if (spaceIdx == 2) return umbrella.health == SENSOR_OK;
  return false;
}

// 물건/우산 상태 변화 시 호출 (거치 시 일반 LED 즉시 소등 + 앱에 데이터 전송)
void onSpaceItemChanged(byte spaceIdx, bool nowPresent) {
  if (spaceIdx >= NUM_SPACES) return;

  if (nowPresent) {
    // [확정 요구사항] 거치 감지 시 일반 백색 LED는 즉시 소등하고 앱으로 상태 전송
    outputs[spaceIdx].normalActive = false;

    // 거치 시 자동 살균 설정이 켜져 있으면 보라색 살균 시작
    if (config.spaces[spaceIdx].sterilEnabled && config.spaces[spaceIdx].sterilOnDetect) {
      if (!outputs[spaceIdx].sterilActive) {
        startSterilization(spaceIdx, NULL);
      }
    }

    // 3번 우산 공간에 우산이 거치되었고 팬 시간이 설정되어 있으면 팬 가동
    if (spaceIdx == 2 && config.fanDurationSec > 0) {
      startFan(config.fanDurationSec, NULL);
    }
  } else {
    // 물건/우산이 제거되면 일반 점멸 및 살균/팬 즉시 중지 (소등)
    outputs[spaceIdx].normalActive = false;
    if (outputs[spaceIdx].sterilActive) {
      stopSterilization(spaceIdx, "item_removed");
    }
    if (spaceIdx == 2 && fan.active) {
      stopFan("item_removed");
    }
  }

  stateDirty = true;
}

// 현관문 개폐 상태 변화 시 호출
void onDoorStateChanged(bool nowOpen) {
  unsigned long now = millis();
  if (nowOpen) {
    // 현관문이 닫힘 -> 열림으로 바뀔 때
    for (byte s = 0; s < NUM_SPACES; ++s) {
      if (isSpaceItemPresent(s)) {
        // 1) 일반 LED(미수거 알림): ledEnabled && ledOnDoor 이면 백색 1.5초 점멸 시작
        if (config.spaces[s].ledEnabled && config.spaces[s].ledOnDoor) {
          outputs[s].normalActive = true;
          outputs[s].normalStartedMs = now;
        }
        // 2) 문 열림 시 자동 살균: sterilEnabled && sterilOnDoor 이면 보라색 살균 시작
        if (config.spaces[s].sterilEnabled && config.spaces[s].sterilOnDoor) {
          if (!outputs[s].sterilActive) {
            startSterilization(s, NULL);
          }
        }
      }
    }
    // 점멸 시작 시 첫 페이즈를 ON으로 정렬
    blinkPhaseOn = true;
    lastBlinkToggleMs = now;
  } else {
    // 현관문이 닫히면 문 열림 미수거 알림 점멸은 모두 즉시 소등
    for (byte s = 0; s < NUM_SPACES; ++s) {
      outputs[s].normalActive = false;
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

  // 1) 팬 타이머 만료 확인
  if (fan.active && (int32_t)(now - fan.untilMs) >= 0) {
    stopFan("duration_elapsed");
  }

  // 2) 일반 점등 1.5초 점멸 토글 타이머
  if (now - lastBlinkToggleMs >= NORMAL_BLINK_MS) {
    lastBlinkToggleMs = now;
    blinkPhaseOn = !blinkPhaseOn;
  }

  // 3) 각 공간의 살균 타이머 및 일반 점멸 유효성 확인 + 실제 WS2812B 색상 반영
  for (byte s = 0; s < NUM_SPACES; ++s) {
    // 살균 만료 또는 센서/설정 해제 검사
    if (outputs[s].sterilActive) {
      if (!config.spaces[s].sterilEnabled) {
        stopSterilization(s, "disabled");
      } else if (!isSpaceItemPresent(s)) {
        stopSterilization(s, "item_removed");
      } else if ((int32_t)(now - outputs[s].sterilUntilMs) >= 0) {
        stopSterilization(s, "duration_elapsed");
      }
    }

    // 일반 점멸 유효성 검사 (문 닫힘, 물건 수거, 비활성화, 최대 60초 경과 시 소등)
    if (outputs[s].normalActive) {
      if (!config.spaces[s].ledEnabled || !config.spaces[s].ledOnDoor ||
          !door.isOpen || !isSpaceItemPresent(s) ||
          (now - outputs[s].normalStartedMs >= NORMAL_MAX_MS)) {
        outputs[s].normalActive = false;
        stateDirty = true;
      }
    }

    // 물리 RGB 색상 결정 (1순위: 보라색 살균 연속 점등, 2순위: 백색 1.5초 점멸)
    byte targetR = 0, targetG = 0, targetB = 0;
    if (outputs[s].sterilActive) {
      targetR = 150; targetG = 0; targetB = 255; // 보라색 살균 점등
    } else if (outputs[s].normalActive) {
      if (blinkPhaseOn) {
        targetR = 120; targetG = 120; targetB = 120; // 백색 점멸 (ON 구간)
      } else {
        targetR = 0; targetG = 0; targetB = 0;       // 백색 점멸 (OFF 구간)
      }
    }

    if (ledsNeedHardwareRefresh ||
        outputs[s].currentR != targetR ||
        outputs[s].currentG != targetG ||
        outputs[s].currentB != targetB) {
      outputs[s].currentR = targetR;
      outputs[s].currentG = targetG;
      outputs[s].currentB = targetB;
      writeStripHardware(s, targetR, targetG, targetB);
    }
  }
  ledsNeedHardwareRefresh = false;
}

// ==============================================================================
// 10. 센서 폴링 (현관문 A0, 우산 IR A3, 로드셀 CH1/CH2)
// ==============================================================================
void pollSensors() {
  unsigned long now = millis();

  // 1) 현관문 리드스위치 (A0) 디바운스
  byte rawDoor = digitalRead(DOOR_PIN);
  if (rawDoor != door.rawLevel) {
    door.rawLevel = rawDoor;
    door.lastDebounceMs = now;
  }
  if (now - door.lastDebounceMs >= DEBOUNCE_MS) {
    if (door.stableLevel != door.rawLevel) {
      door.stableLevel = door.rawLevel;
      bool nowOpen = (door.stableLevel != DOOR_CLOSED_LEVEL);
      if (door.isOpen != nowOpen) {
        door.isOpen = nowOpen;
        onDoorStateChanged(nowOpen);
      }
    }
  }

  // 2) 3번 우산 공간 IR 센서 (A3) 디바운스
  byte rawIr = digitalRead(IR_PIN);
  if (rawIr != umbrella.rawLevel) {
    umbrella.rawLevel = rawIr;
    umbrella.lastDebounceMs = now;
  }
  if (now - umbrella.lastDebounceMs >= DEBOUNCE_MS) {
    if (umbrella.stableLevel != umbrella.rawLevel) {
      umbrella.stableLevel = umbrella.rawLevel;
      bool nowPresent = (umbrella.stableLevel == IR_DETECTED_LEVEL);
      umbrella.health = SENSOR_OK;
      if (umbrella.present != nowPresent) {
        umbrella.present = nowPresent;
        onSpaceItemChanged(2, nowPresent);
      }
    }
  }

  // 3) 1·2번 트레이 로드셀 (HX711) 비차단 측정
  for (byte i = 0; i < NUM_TRAYS; ++i) {
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
      if (!config.tareValid[i]) {
        config.tareOffset[i] = raw;
        config.tareValid[i]  = true;
        trays[i].rawFiltered = raw;
      } else {
        // EMA 저역통과 필터 (기존 60% + 신규 40%)
        trays[i].rawFiltered = (trays[i].rawFiltered * 6L + raw * 4L) / 10L;
      }

      long deltaRaw = trays[i].rawFiltered - config.tareOffset[i];
      float scale = (fabs(config.scaleFactor[i]) > 0.01f) ? config.scaleFactor[i] : 25.0f;
      float grams = (float)deltaRaw / scale;

      // 빈 트레이 상태에서 미세한 온도/크리프 드리프트 자동 영점 추종
      if (!trays[i].present && fabs(grams) < (config.deadzoneGrams[i] * 0.8f)) {
        if (deltaRaw > 3) config.tareOffset[i]++;
        else if (deltaRaw < -3) config.tareOffset[i]--;
      }

      // 음수 오차 및 데드존 이내 흔들림은 0.0g으로 클램핑
      if (grams < config.deadzoneGrams[i]) {
        grams = 0.0f;
      } else if (grams > 3000.0f) {
        grams = 3000.0f;
      }

      // 무게가 2.0g 이상 유의미하게 변했을 때도 앱에 업데이트 전송
      if (trays[i].present && fabs(grams - trays[i].weightGrams) >= 2.0f) {
        stateDirty = true;
      }
      trays[i].weightGrams = grams;

      if (trays[i].health != SENSOR_OK) {
        trays[i].health = SENSOR_OK;
        stateDirty = true;
      }

      // 히스테리시스 + 250ms 안정화 필터로 물건 거치/수거 판정
      bool nextPresent = trays[i].present;
      if (grams >= config.detectGrams[i]) {
        nextPresent = true;
      } else if (grams <= config.releaseGrams[i]) {
        nextPresent = false;
      }

      if (nextPresent != trays[i].candidatePresent) {
        trays[i].candidatePresent = nextPresent;
        trays[i].candidateSinceMs = now;
      } else if (trays[i].present != nextPresent && (now - trays[i].candidateSinceMs >= 250UL)) {
        trays[i].present = nextPresent;
        onSpaceItemChanged(i, nextPresent);
      }
    } else {
      // HX711 일정 시간 무응답 시 센서 오류로 전환
      if (now - trays[i].lastSampleMs > HX_TIMEOUT_MS) {
        if (trays[i].health != SENSOR_ERROR) {
          trays[i].health = SENSOR_ERROR;
          if (outputs[i].sterilActive) stopSterilization(i, "sensor_error");
          outputs[i].normalActive = false;
          stateDirty = true;
        }
      }
    }
  }
}

// ==============================================================================
// 11. JSON 직렬화 및 경량 파서 (동적 메모리 할당 없음)
// ==============================================================================
const char* healthToStr(SensorHealth h) {
  if (h == SENSOR_OK) return "ok";
  if (h == SENSOR_ERROR) return "error";
  return "waiting";
}

const char* ledModeToStr(LedLogicalMode m) {
  if (m == LED_STERIL) return "steril";
  if (m == LED_NORMAL_BLINK) return "normal";
  return "off";
}

unsigned long remainingMs(bool active, unsigned long untilMs) {
  if (!active) return 0UL;
  long diff = (long)(untilMs - millis());
  return (diff > 0) ? (unsigned long)diff : 0UL;
}

void sendStateSnapshot(Stream &out) {
  ++txSeq;
  out.print(F("{\"type\":\"state\",\"v\":1,\"boot\":"));
  out.print(config.bootCount);
  out.print(F(",\"seq\":"));
  out.print(txSeq);

  out.print(F(",\"door\":{\"state\":\""));
  out.print(door.isOpen ? F("open") : F("closed"));
  out.print(F("\",\"sensor\":\""));
  out.print(healthToStr(door.health));
  out.print(F("\"},\"trays\":["));

  for (byte i = 0; i < NUM_TRAYS; ++i) {
    if (i > 0) out.print(',');
    out.print(F("{\"id\":"));
    out.print(i + 1);
    out.print(F(",\"present\":"));
    out.print((trays[i].health == SENSOR_OK && trays[i].present) ? F("true") : F("false"));
    out.print(F(",\"weight\":"));
    out.print((trays[i].health == SENSOR_OK) ? trays[i].weightGrams : 0.0f, 1);
    out.print(F(",\"sensor\":\""));
    out.print(healthToStr(trays[i].health));
    out.print(F("\",\"led\":\""));
    out.print(ledModeToStr(getLogicalLedMode(i)));
    out.print(F("\",\"steril_ms\":"));
    out.print(remainingMs(outputs[i].sterilActive, outputs[i].sterilUntilMs));
    out.print('}');
  }

  out.print(F("],\"umbrella\":{\"present\":"));
  out.print((umbrella.health == SENSOR_OK && umbrella.present) ? F("true") : F("false"));
  out.print(F(",\"sensor\":\""));
  out.print(healthToStr(umbrella.health));
  out.print(F("\",\"led\":\""));
  out.print(ledModeToStr(getLogicalLedMode(2)));
  out.print(F("\",\"steril_ms\":"));
  out.print(remainingMs(outputs[2].sterilActive, outputs[2].sterilUntilMs));

  out.print(F("},\"fan\":{\"active\":"));
  out.print(fan.active ? F("true") : F("false"));
  out.print(F(",\"rem_ms\":"));
  out.print(remainingMs(fan.active, fan.untilMs));
  out.println(F("}}"));
}

void sendAck(const char* reqId, const char* status, const char* reason) {
  bluetooth.print(F("{\"type\":\"ack\",\"req\":\""));
  bluetooth.print((reqId && reqId[0]) ? reqId : "none");
  bluetooth.print(F("\",\"status\":\""));
  bluetooth.print(status);
  bluetooth.print('"');
  if (reason && reason[0]) {
    bluetooth.print(F(",\"reason\":\""));
    bluetooth.print(reason);
    bluetooth.print('"');
  }
  bluetooth.println('}');
}

// JSON 문자열 내에서 특정 키("key":)의 값 시작 포인터 탐색
const char* findJsonValue(const char* json, const char* key) {
  char pattern[24];
  snprintf(pattern, sizeof(pattern), "\"%s\"", key);
  const char* p = strstr(json, pattern);
  if (!p) return NULL;
  p += strlen(pattern);
  while (*p == ' ' || *p == '\t') ++p;
  if (*p != ':') return NULL;
  ++p;
  while (*p == ' ' || *p == '\t') ++p;
  return p;
}

bool extractJsonString(const char* json, const char* key, char* outBuf, size_t maxLen) {
  const char* p = findJsonValue(json, key);
  if (!p || *p != '"') return false;
  ++p;
  size_t i = 0;
  while (*p && *p != '"' && i + 1 < maxLen) {
    outBuf[i++] = *p++;
  }
  outBuf[i] = '\0';
  return (*p == '"');
}

bool extractJsonInt(const char* json, const char* key, long &outVal) {
  const char* p = findJsonValue(json, key);
  if (!p) return false;
  char* endPtr;
  outVal = strtol(p, &endPtr, 10);
  return (endPtr != p);
}

bool extractJsonBool(const char* json, const char* key, bool &outVal) {
  const char* p = findJsonValue(json, key);
  if (!p) return false;
  if (strncmp(p, "true", 4) == 0 || *p == '1') { outVal = true; return true; }
  if (strncmp(p, "false", 5) == 0 || *p == '0') { outVal = false; return true; }
  return false;
}

// "windows":["14:00-18:00","20:00-22:00"] 배열 추출
uint8_t extractJsonStringArray(const char* json, const char* key, char outArr[3][14]) {
  const char* p = findJsonValue(json, key);
  if (!p || *p != '[') return 0;
  ++p;
  uint8_t count = 0;
  while (*p && *p != ']' && count < 3) {
    while (*p && *p != '"' && *p != ']') ++p;
    if (*p != '"') break;
    ++p;
    size_t i = 0;
    while (*p && *p != '"' && i < 13) {
      outArr[count][i++] = *p++;
    }
    outArr[count][i] = '\0';
    while (*p && *p != '"' && *p != ']') ++p;
    if (*p == '"') ++p;
    ++count;
  }
  return count;
}

void handleJsonMessage(const char* line, Stream &replyStream) {
  char type[16] = {0};
  char reqId[16] = {0};
  if (!extractJsonString(line, "type", type, sizeof(type))) {
    sendAck("none", "rejected", "invalid_json");
    return;
  }
  extractJsonString(line, "req", reqId, sizeof(reqId));
  if (!reqId[0]) extractJsonString(line, "request_id", reqId, sizeof(reqId));

  // 1) 즉시 상태 조회: {"type":"get_state","req":"..."}
  if (strcmp(type, "get_state") == 0) {
    if (reqId[0]) sendAck(reqId, "accepted");
    sendStateSnapshot(replyStream);
    lastStateSentMs = millis();
    stateDirty = false;
    return;
  }

  // 2) 공간별 설정 동기화: {"type":"set_config","req":"...","space":1,...}
  if (strcmp(type, "set_config") == 0) {
    long space = 0;
    if (!extractJsonInt(line, "space", space)) {
      extractJsonInt(line, "space_id", space);
    }
    if (space < 1 || space > 3) {
      sendAck(reqId, "rejected", "invalid_space");
      return;
    }
    byte idx = (byte)(space - 1);
    SpaceConfig &sc = config.spaces[idx];

    bool bVal;
    long iVal;
    if (extractJsonBool(line, "led_en", bVal)) sc.ledEnabled = bVal;
    if (extractJsonBool(line, "led_door", bVal)) sc.ledOnDoor = bVal;
    if (extractJsonBool(line, "steril_en", bVal) || extractJsonBool(line, "enabled", bVal)) sc.sterilEnabled = bVal;
    if (extractJsonBool(line, "steril_detect", bVal) || extractJsonBool(line, "on_detect", bVal)) sc.sterilOnDetect = bVal;
    if (extractJsonBool(line, "steril_door", bVal) || extractJsonBool(line, "on_door", bVal)) sc.sterilOnDoor = bVal;

    if (extractJsonInt(line, "steril_sec", iVal) || extractJsonInt(line, "duration_s", iVal)) {
      if (iVal == 5 || iVal == 10 || iVal == 20 || iVal == 30) {
        sc.sterilSec = (uint8_t)iVal;
      } else {
        sc.sterilSec = 10;
        sc.sterilEnabled = false;
      }
    }

    if (idx == 2 && (extractJsonInt(line, "fan_sec", iVal))) {
      if (iVal == 0 || iVal == 15 || iVal == 30 || iVal == 60) {
        config.fanDurationSec = (uint8_t)iVal;
        if (config.fanDurationSec == 0 && fan.active) {
          stopFan("disabled");
        }
      }
    }

    if (!sc.sterilEnabled && outputs[idx].sterilActive) {
      stopSterilization(idx, "disabled");
    }
    if (!sc.ledEnabled && outputs[idx].normalActive) {
      outputs[idx].normalActive = false;
    }

    saveConfigToEeprom();
    sendAck(reqId, "accepted");
    stateDirty = true;
    return;
  }

  // 3) 수동 제어 명령: {"type":"cmd","req":"...","space":1,"action":"start_steril"}
  if (strcmp(type, "cmd") == 0 || strcmp(type, "command") == 0) {
    char action[24] = {0};
    long space = 0;
    extractJsonString(line, "action", action, sizeof(action));
    if (!extractJsonInt(line, "space", space)) extractJsonInt(line, "space_id", space);

    if (strcmp(action, "start_steril") == 0 || strcmp(action, "start_sterilization") == 0) {
      if (space < 1 || space > 3) { sendAck(reqId, "rejected", "invalid_space"); return; }
      byte idx = (byte)(space - 1);
      if (!config.spaces[idx].sterilEnabled) { sendAck(reqId, "rejected", "steril_disabled"); return; }
      if (!isSpaceSensorOk(idx)) { sendAck(reqId, "rejected", "sensor_error"); return; }
      if (!isSpaceItemPresent(idx)) { sendAck(reqId, "rejected", "no_item"); return; }
      if (outputs[idx].sterilActive) { sendAck(reqId, "rejected", "already_running"); return; }

      sendAck(reqId, "accepted");
      startSterilization(idx, reqId);
      return;
    }
    if (strcmp(action, "stop_steril") == 0 || strcmp(action, "stop_sterilization") == 0) {
      if (space < 1 || space > 3) { sendAck(reqId, "rejected", "invalid_space"); return; }
      sendAck(reqId, "accepted");
      stopSterilization((byte)(space - 1), "manual_stop");
      return;
    }
    if (strcmp(action, "start_fan") == 0) {
      long sec = config.fanDurationSec;
      extractJsonInt(line, "sec", sec);
      if (sec != 15 && sec != 30 && sec != 60) sec = (config.fanDurationSec > 0) ? config.fanDurationSec : 30;
      if (!isSpaceSensorOk(2)) { sendAck(reqId, "rejected", "sensor_error"); return; }
      if (!isSpaceItemPresent(2)) { sendAck(reqId, "rejected", "no_item"); return; }

      sendAck(reqId, "accepted");
      startFan((uint8_t)sec, reqId);
      return;
    }
    if (strcmp(action, "stop_fan") == 0) {
      sendAck(reqId, "accepted");
      stopFan("manual_stop");
      return;
    }
    if (strcmp(action, "tare") == 0) {
      if (space < 1 || space > 2) { sendAck(reqId, "rejected", "invalid_space"); return; }
      if (performTare((byte)(space - 1), true)) {
        sendAck(reqId, "accepted");
        stateDirty = true;
      } else {
        sendAck(reqId, "rejected", "sensor_error");
      }
      return;
    }

    sendAck(reqId, "rejected", "unknown_action");
    return;
  }

  // 4) 1602 LCD 날씨 업데이트: {"type":"weather","req":"...","valid":true,"icon":"rain","line1":"RAIN","windows":["14:00-18:00"],"ttl_s":3600}
  if (strcmp(type, "weather") == 0) {
    weatherLcd.receivedOnce = true;
    bool valid = true;
    if (!extractJsonBool(line, "valid", valid)) {
      char statusStr[12] = {0};
      if (extractJsonString(line, "status", statusStr, sizeof(statusStr))) {
        valid = (strcmp(statusStr, "valid") == 0 || strcmp(statusStr, "ready") == 0);
      }
    }

    if (!valid) {
      setWeatherNoData();
      sendAck(reqId, "accepted");
      refreshLcdWeather(true);
      return;
    }

    char iconStr[16] = {0};
    if (!extractJsonString(line, "icon", iconStr, sizeof(iconStr))) {
      extractJsonString(line, "display_condition", iconStr, sizeof(iconStr));
    }

    if (strcmp(iconStr, "rain") == 0 || strcmp(iconStr, "shower") == 0 || strcmp(iconStr, "rain-snow") == 0) {
      weatherLcd.icon = W_ICON_RAIN;
      strncpy(weatherLcd.line1, "RAIN", sizeof(weatherLcd.line1) - 1);
    } else if (strcmp(iconStr, "snow") == 0) {
      weatherLcd.icon = W_ICON_SNOW;
      strncpy(weatherLcd.line1, "SNOW", sizeof(weatherLcd.line1) - 1);
    } else if (strcmp(iconStr, "cloudy") == 0 || strcmp(iconStr, "partly-cloudy") == 0) {
      weatherLcd.icon = W_ICON_CLOUDY;
      strncpy(weatherLcd.line1, "CLOUDY", sizeof(weatherLcd.line1) - 1);
    } else if (strcmp(iconStr, "sunny") == 0 || strcmp(iconStr, "clear") == 0) {
      weatherLcd.icon = W_ICON_SUNNY;
      strncpy(weatherLcd.line1, "SUNNY", sizeof(weatherLcd.line1) - 1);
    } else {
      setWeatherNoData();
      sendAck(reqId, "accepted");
      refreshLcdWeather(true);
      return;
    }

    char customLine1[14] = {0};
    if (extractJsonString(line, "line1", customLine1, sizeof(customLine1)) && customLine1[0]) {
      strncpy(weatherLcd.line1, customLine1, sizeof(weatherLcd.line1) - 1);
      weatherLcd.line1[13] = '\0';
    }

    uint8_t winCnt = extractJsonStringArray(line, "windows", weatherLcd.windows);
    if (winCnt == 0) {
      char line2Str[14] = {0};
      if (extractJsonString(line, "line2", line2Str, sizeof(line2Str)) && line2Str[0]) {
        strncpy(weatherLcd.windows[0], line2Str, 13);
        weatherLcd.windows[0][13] = '\0';
        winCnt = 1;
      } else {
        bool hasRain = false, hasSnow = false;
        extractJsonBool(line, "has_rain_forecast", hasRain);
        extractJsonBool(line, "has_snow_forecast", hasSnow);
        if (weatherLcd.icon == W_ICON_SNOW || hasSnow) {
          strncpy(weatherLcd.windows[0], "NO RAIN", 13);
        } else {
          strncpy(weatherLcd.windows[0], "NO RAIN/SNOW", 13);
        }
        winCnt = 1;
      }
    }

    long ttlSec = 3600;
    extractJsonInt(line, "ttl_s", ttlSec);
    weatherLcd.valid = true;
    weatherLcd.windowCount = winCnt;
    weatherLcd.currentWindowIdx = 0;
    weatherLcd.lastWindowCycleMs = millis();
    weatherLcd.receivedAtMs = millis();
    weatherLcd.ttlMs = (ttlSec > 0) ? ((unsigned long)ttlSec * 1000UL) : 0UL;
    weatherLcd.dirty = true;

    sendAck(reqId, "accepted");
    refreshLcdWeather(true);
    return;
  }

  sendAck(reqId, "rejected", "unknown_type");
}

// ==============================================================================
// 12. 시리얼 캘리브레이션 및 진단 콘솔 (USB Serial 115200bps)
// ==============================================================================
void printCalibrationHelp() {
  Serial.println(F("\n================ [ 아마따 시리얼 캘리브레이션 & 진단 안내 ] ================"));
  Serial.println(F("  STATUS             : 현재 전체 JSON 상태 스냅샷 출력"));
  Serial.println(F("  CAL SHOW           : 1·2번 로드셀 현재 보정값(Tare/Scale/임계값) 조회"));
  Serial.println(F("  TARE 1  (또는 2)   : 해당 트레이 빈 상태에서 영점(0.0g) 기준 측정"));
  Serial.println(F("  CAL 1 200          : 1번 트레이에 200g 분동(또는 물건) 올린 후 스케일 자동 계산"));
  Serial.println(F("  SCALE 1 25.4       : 1번 트레이 Scale Factor 수동 입력"));
  Serial.println(F("  TH 1 15.0 8.0      : 1번 트레이 감지 문턱값(15g) / 해제 문턱값(8g) 설정"));
  Serial.println(F("  DZ 1 3.0           : 1번 트레이 데드존(3.0g 이하 0g 고정) 설정"));
  Serial.println(F("  SAVE               : 현재 보정값 및 설정을 EEPROM에 영구 저장"));
  Serial.println(F("  RESET              : EEPROM 설정을 공장 초기값으로 리셋"));
  Serial.println(F("  LCD WAIT           : LCD를 '기상 정보 대기중' 화면으로 전환 테스트"));
  Serial.println(F("===========================================================================\n"));
}

void printCalibrationStatus() {
  Serial.println(F("\n--- [ 로드셀 캘리브레이션 상태 ] ---"));
  for (byte i = 0; i < NUM_TRAYS; ++i) {
    Serial.print(F("Tray #")); Serial.print(i + 1);
    Serial.print(F(" | Health: ")); Serial.print(healthToStr(trays[i].health));
    Serial.print(F(" | Raw: ")); Serial.print(trays[i].rawFiltered);
    Serial.print(F(" | Tare: ")); Serial.print(config.tareOffset[i]);
    Serial.print(F(" | Scale: ")); Serial.print(config.scaleFactor[i], 2);
    Serial.print(F(" | Weight: ")); Serial.print(trays[i].weightGrams, 1); Serial.print(F("g"));
    Serial.print(F(" | Present: ")); Serial.print(trays[i].present ? F("YES") : F("NO"));
    Serial.print(F(" (TH:")); Serial.print(config.detectGrams[i], 1);
    Serial.print(F("g / REL:")); Serial.print(config.releaseGrams[i], 1);
    Serial.print(F("g / DZ:")); Serial.print(config.deadzoneGrams[i], 1); Serial.println(F("g)"));
  }
  Serial.println(F("------------------------------------\n"));
}

void handleSerialCommand(char* line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (*line == '\0') return;

  // JSON 형식이면 블루투스와 동일하게 JSON 파서로 처리 (시리얼 테스트 용도)
  if (*line == '{') {
    handleJsonMessage(line, Serial);
    return;
  }

  char* cmd = strtok(line, " \t");
  if (!cmd) return;
  for (char* p = cmd; *p; ++p) if (*p >= 'a' && *p <= 'z') *p -= 32;

  if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "H") == 0 || strcmp(cmd, "?") == 0) {
    printCalibrationHelp();
  }
  else if (strcmp(cmd, "STATUS") == 0) {
    sendStateSnapshot(Serial);
  }
  else if (strcmp(cmd, "TARE") == 0) {
    char* arg1 = strtok(NULL, " \t");
    if (!arg1) {
      for (byte i = 0; i < NUM_TRAYS; ++i) performTare(i, false);
      Serial.println(F(">>> [TARE] 1·2번 트레이 모두 영점 보정 완료 (영구 저장은 SAVE 입력)"));
    } else {
      int ch = atoi(arg1) - 1;
      if (ch >= 0 && ch < NUM_TRAYS && performTare((byte)ch, false)) {
        Serial.print(F(">>> [TARE] ")); Serial.print(ch + 1);
        Serial.print(F("번 트레이 영점 완료 (Tare Raw = "));
        Serial.print(config.tareOffset[ch]); Serial.println(F(")"));
      } else {
        Serial.println(F(">>> [ERR] TARE 실패 (트레이 번호 1~2 또는 센서 연결 확인)"));
      }
    }
    stateDirty = true;
  }
  else if (strcmp(cmd, "CAL") == 0) {
    char* arg1 = strtok(NULL, " \t");
    if (!arg1 || strcasecmp(arg1, "SHOW") == 0) {
      printCalibrationStatus();
      return;
    }
    int ch = atoi(arg1) - 1;
    char* arg2 = strtok(NULL, " \t");
    float knownGrams = arg2 ? atof(arg2) : 0.0f;
    if (ch < 0 || ch >= NUM_TRAYS || knownGrams < 5.0f) {
      Serial.println(F(">>> [사용법] CAL <트레이 1|2> <기준무게g> (예: CAL 1 200)"));
      return;
    }
    long avgRaw;
    Serial.println(F(">>> [CAL] 정밀 샘플링 중... 물건을 움직이지 마세요."));
    if (!captureAverageRaw((byte)ch, avgRaw)) {
      Serial.println(F(">>> [ERR] 로드셀 샘플링 실패"));
      return;
    }
    long delta = avgRaw - config.tareOffset[ch];
    if (labs(delta) < 50) {
      Serial.println(F(">>> [ERR] 변화량이 너무 작습니다. 먼저 빈 상태에서 TARE를 수행하세요."));
      return;
    }
    config.scaleFactor[ch] = (float)delta / knownGrams;
    Serial.print(F(">>> [CAL 완료] Tray #")); Serial.print(ch + 1);
    Serial.print(F(" 새 Scale Factor = ")); Serial.print(config.scaleFactor[ch], 2);
    Serial.println(F(" (SAVE 명령으로 EEPROM에 저장하세요)"));
    stateDirty = true;
  }
  else if (strcmp(cmd, "SCALE") == 0) {
    char* arg1 = strtok(NULL, " \t");
    char* arg2 = strtok(NULL, " \t");
    int ch = arg1 ? (atoi(arg1) - 1) : -1;
    float sc = arg2 ? atof(arg2) : 0.0f;
    if (ch >= 0 && ch < NUM_TRAYS && fabs(sc) > 0.01f) {
      config.scaleFactor[ch] = sc;
      Serial.print(F(">>> [SCALE] Tray #")); Serial.print(ch + 1);
      Serial.print(F(" Scale = ")); Serial.println(sc, 2);
    }
  }
  else if (strcmp(cmd, "TH") == 0) {
    char* arg1 = strtok(NULL, " \t");
    char* arg2 = strtok(NULL, " \t");
    char* arg3 = strtok(NULL, " \t");
    int ch = arg1 ? (atoi(arg1) - 1) : -1;
    float det = arg2 ? atof(arg2) : 0.0f;
    float rel = arg3 ? atof(arg3) : (det * 0.55f);
    if (ch >= 0 && ch < NUM_TRAYS && det >= 2.0f) {
      config.detectGrams[ch]  = det;
      config.releaseGrams[ch] = rel;
      Serial.print(F(">>> [TH] Tray #")); Serial.print(ch + 1);
      Serial.print(F(" 감지=")); Serial.print(det, 1);
      Serial.print(F("g / 해제=")); Serial.print(rel, 1); Serial.println(F("g"));
    }
  }
  else if (strcmp(cmd, "DZ") == 0) {
    char* arg1 = strtok(NULL, " \t");
    char* arg2 = strtok(NULL, " \t");
    int ch = arg1 ? (atoi(arg1) - 1) : -1;
    float dz = arg2 ? atof(arg2) : -1.0f;
    if (ch >= 0 && ch < NUM_TRAYS && dz >= 0.0f && dz <= 30.0f) {
      config.deadzoneGrams[ch] = dz;
      Serial.print(F(">>> [DZ] Tray #")); Serial.print(ch + 1);
      Serial.print(F(" 데드존=")); Serial.print(dz, 1); Serial.println(F("g"));
    }
  }
  else if (strcmp(cmd, "SAVE") == 0) {
    saveConfigToEeprom();
    Serial.println(F(">>> [SAVE] 보정값 및 설정이 EEPROM에 저장되었습니다."));
  }
  else if (strcmp(cmd, "RESET") == 0) {
    resetDefaultConfig();
    Serial.println(F(">>> [RESET] EEPROM 설정이 초기값으로 리셋되었습니다."));
  }
  else if (strcmp(cmd, "LCD") == 0) {
    initWeatherDisplayWait();
    refreshLcdWeather(true);
    Serial.println(F(">>> [LCD] 기상 정보 대기중 화면으로 전환되었습니다."));
  }
  else {
    Serial.println(F(">>> 알 수 없는 명령어입니다. 'HELP'를 입력하세요."));
  }
}

// ==============================================================================
// 13. 비차단 스트림 수신 루프
// ==============================================================================
void pollStreams() {
  unsigned long now = millis();

  // 1) HC-06 블루투스 수신
  while (bluetooth.available()) {
    char c = (char)bluetooth.read();
    lastBtRxMs = now;
    if (c == '\r') continue;
    if (c == '\n') {
      if (!btRxOverflow && btRxLen > 0) {
        btRxBuf[btRxLen] = '\0';
        handleJsonMessage(btRxBuf, bluetooth);
      } else if (btRxOverflow) {
        sendAck("none", "rejected", "frame_too_long");
      }
      btRxLen = 0;
      btRxOverflow = false;
    } else {
      if (btRxLen + 1 < sizeof(btRxBuf)) {
        btRxBuf[btRxLen++] = c;
      } else {
        btRxOverflow = true;
      }
    }
  }
  if (btRxLen > 0 && (now - lastBtRxMs > 1500UL)) {
    btRxLen = 0;
    btRxOverflow = false;
  }

  // 2) USB 시리얼 수신 (캘리브레이션 및 디버그용)
  while (Serial.available()) {
    char c = (char)Serial.read();
    lastSerialRxMs = now;
    if (c == '\r') continue;
    if (c == '\n') {
      if (serialRxLen > 0) {
        serialRxBuf[serialRxLen] = '\0';
        handleSerialCommand(serialRxBuf);
        serialRxLen = 0;
      }
    } else if (serialRxLen + 1 < sizeof(serialRxBuf)) {
      serialRxBuf[serialRxLen++] = c;
    }
  }
  if (serialRxLen > 0 && (now - lastSerialRxMs > 150UL)) {
    serialRxBuf[serialRxLen] = '\0';
    handleSerialCommand(serialRxBuf);
    serialRxLen = 0;
  }
}

// ==============================================================================
// 14. 초기 설정 (setup) 및 메인 루프 (loop)
// ==============================================================================
void setup() {
  Serial.begin(115200);
  bluetooth.begin(9600);

  // 1) 출력 핀 초기화
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);

  for (byte s = 0; s < NUM_SPACES; ++s) {
    pinMode(STRIP_PIN[s], OUTPUT);
    digitalWrite(STRIP_PIN[s], LOW);
    writeStripHardware(s, 0, 0, 0);
  }

  // 2) 센서 핀 초기화 (현관문 A0, 우산 IR A3)
  pinMode(DOOR_PIN, INPUT_PULLUP);
  pinMode(IR_PIN, INPUT_PULLUP);

  // 3) EEPROM 설정 및 보정값 로드
  loadConfigFromEeprom();

  // 4) 현관문 및 우산 IR 초기 상태 설정
  unsigned long now = millis();
  door.rawLevel = door.stableLevel = digitalRead(DOOR_PIN);
  door.lastDebounceMs = now;
  door.isOpen = (door.stableLevel != DOOR_CLOSED_LEVEL);
  door.health = SENSOR_OK;

  umbrella.rawLevel = umbrella.stableLevel = digitalRead(IR_PIN);
  umbrella.lastDebounceMs = now;
  umbrella.present = (umbrella.stableLevel == IR_DETECTED_LEVEL);
  umbrella.health = SENSOR_OK;

  // 5) 로드셀 핀 초기화 및 부팅 시 초기 영점(Baseline) 확인
  for (byte i = 0; i < NUM_TRAYS; ++i) {
    pinMode(HX_DT[i], INPUT);
    pinMode(HX_CLOCK[i], OUTPUT);
    digitalWrite(HX_CLOCK[i], LOW);
    trays[i].health = SENSOR_WAITING;
    trays[i].present = false;
    trays[i].candidatePresent = false;
    trays[i].weightGrams = 0.0f;
    trays[i].lastSampleMs = now;

    // 부팅 시 센서가 연결되어 있으면 초기 영점 측정
    performTare(i, false);
  }

  // 6) 1602 I2C LCD 초기화 -> "기상 정보 대기중" 화면 출력
  Wire.begin();
  Wire.setWireTimeout(3000, true);
  lcdAvailable = initI2CLcd();
  initWeatherDisplayWait();
  refreshLcdWeather(true);

  Serial.println(F("\n======================================================="));
  Serial.println(F(" 아마따 (AMATTA) 스마트 트레이 펌웨어 v1.0 준비 완료"));
  Serial.println(F(" - 캘리브레이션 도움말: 시리얼 모니터에 'HELP' 입력"));
  Serial.println(F("======================================================="));

  stateDirty = true;
}

void loop() {
  // 1) 블루투스(HC-06) 및 USB 시리얼 명령 수신 처리
  pollStreams();

  // 2) 현관문(A0), 우산 IR(A3), 로드셀(CH1/CH2) 상태 측정 및 변화 감지
  pollSensors();

  // 3) 공유 LED(보라색 살균 / 백색 1.5초 점멸) 및 건조 팬 타이머 갱신
  evaluateOutputs();

  // 4) 1602 LCD 날씨 화면 갱신 및 비 예보 구간 순환
  refreshLcdWeather(false);

  // 5) 내부 상태가 변경되었거나(stateDirty) 1초 주기가 도래하면 즉시 앱으로 전송
  unsigned long now = millis();
  if (stateDirty || (now - lastStateSentMs >= STATE_HEARTBEAT_MS)) {
    stateDirty = false;
    lastStateSentMs = now;
    sendStateSnapshot(bluetooth);
  }
}
