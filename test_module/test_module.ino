/*
 * ==============================================================================
 * 프로젝트: 까먹는 물건 보관함 - 통합 하드웨어 정밀 진단 펌웨어 (v2.3)
 * ==============================================================================
 * 
 * [통합 진단 및 테스트 기능]
 *  1. 1602 LCD 5x8 커스텀 아이콘 그래픽 (순수 Wire.h 제어):
 *     - [🔑] 열쇠/차 키 (0번)
 *     - [📦] 상자/지갑   (1번)
 *     - [🌀] 쿨링팬     (2번)
 *     - [☂️] 우산/IR     (3번)
 *     - [🚪] 도어       (4번)
 *     - [✔️] 물건 있음   (5번)
 *     - [✕]  비어 있음   (6번)
 *     - [💡] LED 바     (7번)
 *  2. WS2812B 3채널 LED 바 (8구 x 3 = 24구) 드라이버 및 테스트 기능:
 *     - D8(1번칸), D9(2번칸), D10(3번칸)
 *     - 외장 라이브러리 설치 필요 없는 순수 AVR 고속 비트뱅 드라이버 탑재!
 *     - 'led white', 'led red', 'led green', 'led blue', 'led purple', 'led off'
 *     - 'led test' (빨강->초록->파랑->보라->백색 자동 순환 테스트)
 *     - 'led auto' (뚜껑 열림 시 백색 조명 자동 ON, 닫힘 시 OFF 연동 모드)
 *  3. 리드스위치 (자석 도어센서 SASO-DS30A) 3채널 테스트:
 *     - A0: 현관, A1: 뚜껑, A2: 창문 (INPUT_PULLUP)
 *     - 40ms 디바운스, 단축키 'd'로 실시간 리포트
 *  4. 쿨링팬 (D11) 신호 제어 및 테스트:
 *     - '1' (켜기), '0' (끄기), 'f' (토글), 'test' (2초 점멸 테스트)
 *     - 'a' (우산 감지 시 팬 자동 연동 ON/OFF)
 *  5. 로드셀 2채널 고정밀 무게 변화 감지:
 *     - CH1 (D2, D3), CH2 (D4, D5)
 *     - EMA 필터 + 제로 데드존(250) + 자동 영점 추종으로 흔들림 없는 (+0) 유지!
 *     - 't' (영점 리셋), 'th 700' (감도 조절), 'dz 250' (데드존 조절)
 *  6. IR 장애물 센서: A3 (우산 감지 LOW)
 * ==============================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <avr/pgmspace.h>

// ------------------------------------------------------------------------------
// 핀 매핑 (회로 요약서 기준)
// ------------------------------------------------------------------------------
const byte NUM_CH = 2;
const byte HX_DT[NUM_CH]    = {2, 4};
const byte HX_CLOCK[NUM_CH] = {3, 5};

const byte NUM_STRIPS = 3;
const byte STRIP_PIN[NUM_STRIPS] = {8, 9, 10}; // D8: 1번칸, D9: 2번칸, D10: 3번칸
const byte LEDS_PER_STRIP = 8;                 // 각 8구 WS2812B LED

const byte FAN_PIN   = 11; // 쿨링팬 제어 신호 출력 핀 (D11)
const byte BT_RX_PIN = 12; // HC-06 TX
const byte BT_TX_PIN = 13; // HC-06 RX (분압 회로)
const byte IR_PIN    = A3; // IR 장애물 센서 OUT (감지 시 LOW)
const byte IR_DETECTED_LEVEL = LOW;

const byte NUM_DOOR = 3;
const byte DOOR_PIN[NUM_DOOR] = {A0, A1, A2}; // A0: 현관, A1: 뚜껑, A2: 창문
const char* const DOOR_NAME[NUM_DOOR] = {"현관(A0)", "뚜껑(A1)", "창문(A2)"};

long threshold = 700; // 로드셀 감지 문턱값
long deadzone  = 250; // 제로 데드존 (진동 제거)

SoftwareSerial bluetooth(BT_RX_PIN, BT_TX_PIN);

byte lcdAddress = 0x27;
bool lcdAvailable = false;

// ------------------------------------------------------------------------------
// 1602 LCD 5x8 커스텀 아이콘 비트맵 정의 (PROGMEM 저장)
// ------------------------------------------------------------------------------
const byte ICON_KEY[8] PROGMEM = { // 0번: 🔑 열쇠/차 키
  0b01110,
  0b10001,
  0b01110,
  0b00100,
  0b00110,
  0b00100,
  0b00110,
  0b00000
};

const byte ICON_BOX[8] PROGMEM = { // 1번: 📦 상자/지갑
  0b11111,
  0b10001,
  0b10101,
  0b10101,
  0b10001,
  0b11111,
  0b00000,
  0b00000
};

const byte ICON_FAN[8] PROGMEM = { // 2번: 🌀 쿨링팬
  0b00100,
  0b10101,
  0b01110,
  0b11111,
  0b01110,
  0b10101,
  0b00100,
  0b00000
};

const byte ICON_UMB[8] PROGMEM = { // 3번: ☂️ 우산
  0b00100,
  0b01110,
  0b11111,
  0b10101,
  0b00100,
  0b00100,
  0b10100,
  0b01100
};

const byte ICON_DOOR[8] PROGMEM = { // 4번: 🚪 도어
  0b11111,
  0b10001,
  0b10101,
  0b10001,
  0b10001,
  0b10001,
  0b11111,
  0b00000
};

const byte ICON_CHECK[8] PROGMEM = { // 5번: ✔️ 물건 있음
  0b00000,
  0b00001,
  0b00010,
  0b10100,
  0b01000,
  0b00000,
  0b00000,
  0b00000
};

const byte ICON_EMPTY[8] PROGMEM = { // 6번: ✕ 비어 있음
  0b00000,
  0b10001,
  0b01010,
  0b00100,
  0b01010,
  0b10001,
  0b00000,
  0b00000
};

const byte ICON_BULB[8] PROGMEM = { // 7번: 💡 LED 바
  0b01110,
  0b10001,
  0b10101,
  0b10101,
  0b01110,
  0b01110,
  0b00100,
  0b00000
};

// ------------------------------------------------------------------------------
// LED 바 제어 상태 변수
// ------------------------------------------------------------------------------
enum LedMode { LED_MODE_MANUAL, LED_MODE_AUTO, LED_MODE_TEST };
LedMode ledMode = LED_MODE_MANUAL;
byte ledColorR = 0, ledColorG = 0, ledColorB = 0;
const char* ledColorName = "OFF";
bool ledCycleRunning = false;
unsigned long lastLedCycleTime = 0;
byte ledCycleStep = 0;

// ------------------------------------------------------------------------------
// 팬 제어 상태 변수
// ------------------------------------------------------------------------------
bool fanState = false;
bool fanTestMode = false;
bool fanAutoMode = false;
unsigned long fanUntilTime = 0;
unsigned long lastFanToggleTime = 0;

// ------------------------------------------------------------------------------
// IR 센서 상태 변수
// ------------------------------------------------------------------------------
bool irDetected = false;
bool lastIrDetected = false;

// ------------------------------------------------------------------------------
// 리드스위치 (자석 도어 센서) 상태 구조체 및 변수
// ------------------------------------------------------------------------------
struct DoorSensor {
  byte raw;
  byte stable;
  unsigned long lastDebounceTime;
};
DoorSensor doors[NUM_DOOR];
byte lastLcdDoorState[NUM_DOOR] = {255, 255, 255};

// ------------------------------------------------------------------------------
// 로드셀 모니터 구조체
// ------------------------------------------------------------------------------
struct ChannelMonitor {
  long baseline;
  long currentRaw;
  long delta;
  bool hasItem;
  bool isConnected;
  unsigned long lastReady;
};
ChannelMonitor chs[NUM_CH];
bool lastItemState[NUM_CH] = {false, false};
bool lastFanState = false;
const char* lastLedName = "";

unsigned long lastDisplayTime = 0;

// 시리얼 버퍼 관리
String serialBuffer = "";
unsigned long lastSerialCharTime = 0;
String btBuffer = "";
unsigned long lastBtCharTime = 0;

// ==============================================================================
// 순수 AVR 고속 WS2812B 드라이버 (외장 라이브러리 불필요, 16MHz 나노초 정밀 타이밍)
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
    "st %a[port], %[hi] \n\t"        // [0] HIGH 시작
    "sbrs %[curbyte], 7 \n\t"        // [2] 비트7이 1이면 다음 줄 건너뜀
    "st %a[port], %[lo] \n\t"        // [3-4] 0비트인 경우: 4사이클(250ns)만에 LOW!
    "lsl %[curbyte] \n\t"            // [5] 다음 비트 준비
    "dec %[ctr] \n\t"                // [6] 8비트 카운터 감산
    "rjmp .+0 \n\t"                  // [7-8] 타이밍 정밀 딜레이
    "st %a[port], %[lo] \n\t"        // [9-10] 1비트인 경우: 10사이클(625ns)에 LOW!
    "rjmp .+0 \n\t"                  // [11-12] 딜레이
    "brne 2b \n\t"                   // [13-14] 바이트 내 비트 반복
    "sbiw %[count], 1 \n\t"          // [15-16] 바이트 수 감산
    "brne 1b \n\t"                   // [17-18] 전체 바이트 반복 (정확히 800kHz 주기)
    : [dataptr] "+e" (data),
      [count] "+w" (len),
      [curbyte] "=&d" (curbyte),
      [ctr] "=&d" (ctr)
    : [port] "e" (port),
      [hi] "r" (hi),
      [lo] "r" (lo)
  );

  SREG = sreg;
  delayMicroseconds(80); // 80us 래치 딜레이 (데이터 적용 확정)
}

void setStripColor(byte stripIdx, byte r, byte g, byte b) {
  if (stripIdx >= NUM_STRIPS) return;
  byte buffer[LEDS_PER_STRIP * 3];
  for (byte i = 0; i < LEDS_PER_STRIP; ++i) {
    buffer[i * 3 + 0] = g; // WS2812B는 GRB 순서
    buffer[i * 3 + 1] = r;
    buffer[i * 3 + 2] = b;
  }
  ws2812Send(STRIP_PIN[stripIdx], buffer, sizeof(buffer));
}

void setAllStripsColor(byte r, byte g, byte b, const char* name = NULL) {
  ledColorR = r;
  ledColorG = g;
  ledColorB = b;
  if (name) ledColorName = name;
  for (byte i = 0; i < NUM_STRIPS; ++i) {
    setStripColor(i, r, g, b);
  }
}

// ==============================================================================
// 1602 I2C LCD 초경량 드라이버 (Wire.h 순수 제어)
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

void lcdPrintStr(const char* str) {
  while (*str) lcdChar(*str++);
}

void lcdSetCursor(byte col, byte row) {
  byte rowOffsets[] = {0x00, 0x40};
  lcdCommand(0x80 | (col + rowOffsets[row]));
}

void lcdCreateChar(byte loc, const byte charmap[]) {
  loc &= 0x07;
  lcdCommand(0x40 | (loc << 3));
  for (byte i = 0; i < 8; ++i) {
    lcdSendByte(pgm_read_byte(&charmap[i]), 1);
  }
  lcdCommand(0x80);
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

  // 8개 커스텀 아이콘 CGRAM 등록
  lcdCreateChar(0, ICON_KEY);   // 🔑
  lcdCreateChar(1, ICON_BOX);   // 📦
  lcdCreateChar(2, ICON_FAN);   // 🌀
  lcdCreateChar(3, ICON_UMB);   // ☂️
  lcdCreateChar(4, ICON_DOOR);  // 🚪
  lcdCreateChar(5, ICON_CHECK); // ✔️
  lcdCreateChar(6, ICON_EMPTY); // ✕
  lcdCreateChar(7, ICON_BULB);  // 💡

  return true;
}

void updateLcdScreen(bool force = false) {
  if (!lcdAvailable) return;

  bool doorChanged = false;
  for (byte i = 0; i < NUM_DOOR; ++i) {
    if (doors[i].stable != lastLcdDoorState[i]) {
      doorChanged = true;
      lastLcdDoorState[i] = doors[i].stable;
    }
  }

  if (!force && !doorChanged && 
      (chs[0].hasItem == lastItemState[0]) && 
      (chs[1].hasItem == lastItemState[1]) &&
      (fanState == lastFanState) &&
      (irDetected == lastIrDetected) &&
      (ledColorName == lastLedName)) {
    return;
  }

  lastItemState[0] = chs[0].hasItem;
  lastItemState[1] = chs[1].hasItem;
  lastFanState = fanState;
  lastIrDetected = irDetected;
  lastLedName = ledColorName;

  // 1행: [🔑]:[✔/✕]  [📦]:[✔/✕]  [🌀]:ON/OFF -> 정확히 16자
  // 예: [0]:[5] [1]:[6] [2]:ON
  lcdSetCursor(0, 0);
  lcdChar(0); // KEY 아이콘
  lcdChar(':');
  if (!chs[0].isConnected)      lcdPrintStr("ER ");
  else if (chs[0].hasItem)      { lcdChar(5); lcdChar(' '); } // ✔️
  else                          { lcdChar(6); lcdChar(' '); } // ✕

  lcdChar(' ');
  lcdChar(1); // BOX 아이콘
  lcdChar(':');
  if (!chs[1].isConnected)      lcdPrintStr("ER ");
  else if (chs[1].hasItem)      { lcdChar(5); lcdChar(' '); } // ✔️
  else                          { lcdChar(6); lcdChar(' '); } // ✕

  lcdChar(' ');
  lcdChar(2); // FAN 아이콘
  lcdPrintStr(fanState ? ":ON " : ":OFF");

  // 2행: [☂️]:[✔/✕] [🚪]:CLS/OPN [💡]:WHT/OFF -> 정확히 16자
  lcdSetCursor(0, 1);
  lcdChar(3); // UMB 아이콘
  lcdChar(':');
  if (irDetected)               { lcdChar(5); lcdChar(' '); } // ✔️
  else                          { lcdChar(6); lcdChar(' '); } // ✕

  lcdChar(' ');
  lcdChar(4); // DOOR 아이콘
  lcdPrintStr(doors[0].stable == LOW ? ":CLS " : ":OPN ");

  lcdChar(7); // BULB 아이콘
  lcdChar(':');
  if (strcmp(ledColorName, "WHITE") == 0)       lcdPrintStr("WHT");
  else if (strcmp(ledColorName, "RED") == 0)    lcdPrintStr("RED");
  else if (strcmp(ledColorName, "GREEN") == 0)  lcdPrintStr("GRN");
  else if (strcmp(ledColorName, "BLUE") == 0)   lcdPrintStr("BLU");
  else if (strcmp(ledColorName, "PURPLE") == 0) lcdPrintStr("PUR");
  else                                          lcdPrintStr("OFF");
}

// ==============================================================================
// HX711 24비트 정밀 읽기
// ==============================================================================
bool readHX711(byte ch, long &outputValue) {
  if (digitalRead(HX_DT[ch]) != LOW) return false;

  uint32_t rawBits = 0;
  for (byte i = 0; i < 25; ++i) {
    uint8_t oldSREG = SREG;
    cli();
    digitalWrite(HX_CLOCK[ch], HIGH);
    delayMicroseconds(4);
    if (i < 24) rawBits = (rawBits << 1) | digitalRead(HX_DT[ch]);
    digitalWrite(HX_CLOCK[ch], LOW);
    SREG = oldSREG;
    delayMicroseconds(4);
  }

  if (rawBits & 0x800000UL) rawBits |= 0xFF000000UL;
  outputValue = (long)rawBits;
  return true;
}

bool setBaseline(byte ch) {
  int count = 0;
  long samples[16];
  unsigned long start = millis();

  while (count < 16 && (millis() - start < 3500)) {
    long val;
    if (readHX711(ch, val)) {
      samples[count++] = val;
      delay(20);
    }
  }

  if (count == 16) {
    for (int i = 0; i < 15; ++i) {
      for (int j = i + 1; j < 16; ++j) {
        if (samples[i] > samples[j]) {
          long temp = samples[i];
          samples[i] = samples[j];
          samples[j] = temp;
        }
      }
    }
    int64_t sum = 0;
    for (int i = 2; i < 14; ++i) {
      sum += samples[i];
    }
    chs[ch].baseline = (long)(sum / 12);
    chs[ch].currentRaw = chs[ch].baseline;
    chs[ch].delta = 0;
    chs[ch].hasItem = false;
    return true;
  }
  return false;
}

// ==============================================================================
// 쿨링팬 제어 함수 (실제 D11 출력 및 검증)
// ==============================================================================
void setFan(bool turnOn, const char* reason = NULL) {
  fanState = turnOn;
  digitalWrite(FAN_PIN, fanState ? HIGH : LOW);

  int pinActual = digitalRead(FAN_PIN);

  Serial.println(F("\n======================================================="));
  Serial.print(F(" [팬 제어 신호] "));
  if (reason) {
    Serial.print(F("(")); Serial.print(reason); Serial.print(F(") "));
  }
  Serial.print(F("D11 핀 상태 -> "));
  if (pinActual == HIGH) {
    Serial.println(F("【 HIGH (5.0V 출력 중) 】"));
    Serial.println(F(" -> 아두이노 D11 핀에서 5.0V 신호가 확실하게 출력되고 있습니다!"));
  } else {
    Serial.println(F("【 LOW (0.0V 차단 상태) 】"));
    Serial.println(F(" -> 아두이노 D11 핀이 0.0V(접지)로 차단되었습니다."));
  }
  Serial.println(F("=======================================================\n"));

  updateLcdScreen(true);
}

// ==============================================================================
// 리드스위치 실시간 상태 상세 출력 함수
// ==============================================================================
void printDoorStatus() {
  Serial.println(F("\n======================================================="));
  Serial.println(F("       [ 리드스위치 (자석 도어센서) 실시간 상태 ]       "));
  Serial.println(F("======================================================="));
  for (byte i = 0; i < NUM_DOOR; ++i) {
    Serial.print(F("  - "));
    Serial.print(DOOR_NAME[i]);
    Serial.print(F(" : "));
    if (doors[i].stable == LOW) {
      Serial.println(F("【 닫힘 (CLOSED) 】- 자석 부착 감지 (접점 연결 / LOW)"));
    } else {
      Serial.println(F("【 열림 (OPEN)   】- 자석 분리 상태 (접점 분리 / HIGH)"));
    }
  }
  Serial.println(F("-------------------------------------------------------"));
  Serial.println(F("  * 닫힘: LOW (자석 밀착) / 열림: HIGH (자석 분리)"));
  Serial.println(F("=======================================================\n"));
}

// ==============================================================================
// 명령어 파싱 및 실행 루틴
// ==============================================================================
void processCommand(String cmd, bool fromBluetooth = false) {
  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.print(fromBluetooth ? F("[BT 수신] '") : F("[시리얼 수신] '"));
  Serial.print(cmd);
  Serial.println(F("'"));

  String upperCmd = cmd;
  upperCmd.toUpperCase();

  // ----------------------------------------------------------------------------
  // LED 바 제어 명령어
  // ----------------------------------------------------------------------------
  if (upperCmd == "LED ON" || upperCmd == "LEDON" || upperCmd == "LED 1" || upperCmd == "LED WHITE" || upperCmd == "WHITE") {
    ledMode = LED_MODE_MANUAL;
    ledCycleRunning = false;
    setAllStripsColor(120, 120, 120, "WHITE"); // 눈부심 없는 밝은 백색
    Serial.println(F(">>> [LED 바] 3채널 전체 백색 조명 ON (밝기 120/255)"));
    updateLcdScreen(true);
  }
  else if (upperCmd == "LED OFF" || upperCmd == "LEDOFF" || upperCmd == "LED 0" || upperCmd == "BLACK") {
    ledMode = LED_MODE_MANUAL;
    ledCycleRunning = false;
    setAllStripsColor(0, 0, 0, "OFF");
    for (byte i = 0; i < NUM_STRIPS; ++i) {
      digitalWrite(STRIP_PIN[i], LOW);
    }
    Serial.println(F(">>> [LED 바] 3채널 전체 소등 (OFF) - 0값 데이터 전송 완료"));
    updateLcdScreen(true);
  }
  else if (upperCmd == "LED RED" || upperCmd == "RED") {
    ledMode = LED_MODE_MANUAL;
    ledCycleRunning = false;
    setAllStripsColor(200, 0, 0, "RED");
    Serial.println(F(">>> [LED 바] 빨간색 (경고 모드) 점등"));
    updateLcdScreen(true);
  }
  else if (upperCmd == "LED GREEN" || upperCmd == "GREEN") {
    ledMode = LED_MODE_MANUAL;
    ledCycleRunning = false;
    setAllStripsColor(0, 200, 0, "GREEN");
    Serial.println(F(">>> [LED 바] 초록색 (정상 완료 모드) 점등"));
    updateLcdScreen(true);
  }
  else if (upperCmd == "LED BLUE" || upperCmd == "BLUE") {
    ledMode = LED_MODE_MANUAL;
    ledCycleRunning = false;
    setAllStripsColor(0, 0, 200, "BLUE");
    Serial.println(F(">>> [LED 바] 파란색 점등"));
    updateLcdScreen(true);
  }
  else if (upperCmd == "LED PURPLE" || upperCmd == "LED UV" || upperCmd == "PURPLE") {
    ledMode = LED_MODE_MANUAL;
    ledCycleRunning = false;
    setAllStripsColor(150, 0, 255, "PURPLE");
    Serial.println(F(">>> [LED 바] 보라색 (살균 연출 모드) 점등"));
    updateLcdScreen(true);
  }
  else if (upperCmd == "LED TEST" || upperCmd == "LED CYCLE") {
    ledMode = LED_MODE_TEST;
    ledCycleRunning = true;
    ledCycleStep = 0;
    lastLedCycleTime = millis();
    Serial.println(F(">>> [LED 바 자동 순환 테스트 시작] 빨강 -> 초록 -> 파랑 -> 보라 -> 백색 자동 순환"));
    setAllStripsColor(200, 0, 0, "RED");
    updateLcdScreen(true);
  }
  else if (upperCmd == "LED AUTO") {
    ledMode = LED_MODE_AUTO;
    ledCycleRunning = false;
    Serial.println(F(">>> [LED 바 자동 연동 모드 활성화] 뚜껑(A1) 열리면 백색 ON, 닫히면 소등"));
    updateLcdScreen(true);
  }

  // ----------------------------------------------------------------------------
  // 팬 제어 명령어
  // ----------------------------------------------------------------------------
  else if (cmd.equalsIgnoreCase("fan on") || cmd.equalsIgnoreCase("on") || 
           cmd.equalsIgnoreCase("fan 1") || cmd.equalsIgnoreCase("1")) {
    fanTestMode = false;
    fanAutoMode = false;
    fanUntilTime = 0;
    setFan(true, "수동 명령: 켜기");
  }
  else if (cmd.equalsIgnoreCase("fan off") || cmd.equalsIgnoreCase("off") || 
           cmd.equalsIgnoreCase("fan 0") || cmd.equalsIgnoreCase("0") || 
           cmd.equalsIgnoreCase("stop")) {
    fanTestMode = false;
    fanAutoMode = false;
    fanUntilTime = 0;
    setFan(false, "수동 명령: 끄기/정지");
  }
  else if (cmd.equalsIgnoreCase("f")) {
    fanTestMode = false;
    fanAutoMode = false;
    fanUntilTime = 0;
    setFan(!fanState, "단축키: 토글");
  }
  else if (cmd.equalsIgnoreCase("fan auto") || cmd.equalsIgnoreCase("auto") || cmd.equalsIgnoreCase("a")) {
    fanTestMode = false;
    fanAutoMode = true;
    fanUntilTime = 0;
    Serial.println(F("\n>>> [팬 자동 연동 모드 (IR 센서 연동)] 우산 감지 시 자동 ON, 제거 시 OFF"));
    setFan(irDetected, irDetected ? "자동: 우산 감지" : "자동: 우산 없음");
  }
  else if (upperCmd.startsWith("FAN ")) {
    fanTestMode = false;
    fanAutoMode = false;
    int seconds = cmd.substring(4).toInt();
    if (seconds > 0) {
      fanUntilTime = millis() + ((unsigned long)seconds * 1000UL);
      Serial.print(F(">>> [팬 타이머 동작] ")); Serial.print(seconds); Serial.println(F("초 동안 가동"));
      setFan(true, "타이머 시작");
    } else {
      fanUntilTime = 0;
      setFan(false, "타이머 정지");
    }
  }
  else if (cmd.equalsIgnoreCase("fan test") || cmd.equalsIgnoreCase("test")) {
    fanTestMode = true;
    fanAutoMode = false;
    fanUntilTime = 0;
    lastFanToggleTime = millis();
    Serial.println(F("\n>>> [팬 점멸 테스트 모드] 2초마다 5V <-> 0V 자동 반복"));
    setFan(true, "점멸 테스트 시작");
  }

  // ----------------------------------------------------------------------------
  // 센서 진단 및 보정 명령어
  // ----------------------------------------------------------------------------
  else if (cmd.equalsIgnoreCase("d") || cmd.equalsIgnoreCase("door") || cmd.equalsIgnoreCase("reed")) {
    printDoorStatus();
  }
  else if (cmd.equalsIgnoreCase("t") || cmd.equalsIgnoreCase("tare")) {
    Serial.println(F("\n>>> [영점 보정] 손을 뗀 상태에서 기준값(Baseline)을 다시 잡습니다..."));
    for (byte i = 0; i < NUM_CH; ++i) {
      if (setBaseline(i)) {
        Serial.print(F("CH")); Serial.print(i + 1); Serial.print(F(" 영점 완료 (Base: "));
        Serial.print(chs[i].baseline); Serial.println(F(")"));
      } else {
        Serial.print(F("CH")); Serial.print(i + 1); Serial.println(F(" 영점 실패"));
      }
    }
    updateLcdScreen(true);
  }
  else if (upperCmd.startsWith("TH ") || upperCmd.startsWith("SENS ")) {
    int spaceIdx = cmd.indexOf(' ');
    long newTh = cmd.substring(spaceIdx + 1).toInt();
    if (newTh >= 100 && newTh <= 20000) {
      threshold = newTh;
      Serial.print(F("\n>>> [감도 설정] 로드셀 감지 문턱값이 ")); Serial.print(threshold); Serial.println(F(" 으로 변경되었습니다.\n"));
    }
  }
  else if (upperCmd.startsWith("DZ ") || upperCmd.startsWith("DEADZONE ")) {
    int spaceIdx = cmd.indexOf(' ');
    long newDz = cmd.substring(spaceIdx + 1).toInt();
    if (newDz >= 0 && newDz <= 1000) {
      deadzone = newDz;
      Serial.print(F("\n>>> [데드존 설정] 진동 0고정 범위(Deadzone)가 ")); Serial.print(deadzone); Serial.println(F(" 으로 변경되었습니다.\n"));
    }
  }
  else if (cmd.equalsIgnoreCase("h") || cmd.equalsIgnoreCase("help") || cmd.equalsIgnoreCase("?")) {
    Serial.println(F("\n================== [ 사용 가능한 명령어 전체 안내 ] =================="));
    Serial.println(F("  [LED 바 테스트]"));
    Serial.println(F("    led white (또는 on) : 3채널 LED 바 백색 조명 ON"));
    Serial.println(F("    led off (또는 black): LED 바 소등 (OFF)"));
    Serial.println(F("    led red / green / blue / purple : 각 색상 점등 테스트"));
    Serial.println(F("    led test            : 빨->초->파->보->백 자동 순환 점멸"));
    Serial.println(F("    led auto            : 뚜껑(A1) 열림 시 백색 ON, 닫힘 시 OFF"));
    Serial.println(F("  [쿨링팬 제어]"));
    Serial.println(F("    1 / 0 (또는 on/off) : 팬 켜기 (5V) / 끄기 (0V)"));
    Serial.println(F("    f                   : 팬 상태 반전 (토글)"));
    Serial.println(F("    a (또는 auto)       : IR 센서 우산 감지 시 팬 자동 가동"));
    Serial.println(F("    test                : 팬 2초 자동 점멸 테스트"));
    Serial.println(F("    FAN 60              : 60초간 팬 가동 후 자동 정지"));
    Serial.println(F("  [센서 진단]"));
    Serial.println(F("    d                   : 리드스위치 (A0, A1, A2) 실시간 접점 상태"));
    Serial.println(F("    t                   : 로드셀 현재 무게를 (+0)으로 영점 맞춤"));
    Serial.println(F("    th 700              : 로드셀 감도 변경 (기본값: 700)"));
    Serial.println(F("    dz 250              : 진동 제거 제로 데드존 변경 (기본값: 250)"));
    Serial.println(F("======================================================================\n"));
  }
  else {
    Serial.print(F("알 수 없는 명령어: '")); Serial.print(cmd);
    Serial.println(F("' (도움말은 'h' 또는 '?' 입력)"));
  }
}

// ==============================================================================
// 시리얼 및 블루투스 입력 비차단 감지
// ==============================================================================
void handleInputs() {
  unsigned long now = millis();

  while (Serial.available()) {
    char c = Serial.read();
    lastSerialCharTime = now;
    if (c == '\r') continue;
    if (c == '\n') {
      if (serialBuffer.length() > 0) {
        processCommand(serialBuffer, false);
        serialBuffer = "";
      }
    } else {
      serialBuffer += c;
    }
  }

  if (serialBuffer.length() > 0 && (now - lastSerialCharTime >= 60)) {
    processCommand(serialBuffer, false);
    serialBuffer = "";
  }

  while (bluetooth.available()) {
    char c = (char)bluetooth.read();
    lastBtCharTime = now;
    if (c == '\r') continue;
    if (c == '\n') {
      if (btBuffer.length() > 0) {
        processCommand(btBuffer, true);
        btBuffer = "";
      }
    } else {
      btBuffer += c;
    }
  }

  if (btBuffer.length() > 0 && (now - lastBtCharTime >= 60)) {
    processCommand(btBuffer, true);
    btBuffer = "";
  }
}

// ==============================================================================
// 초기 설정 (setup)
// ==============================================================================
void setup() {
  Serial.begin(115200);
  bluetooth.begin(9600);

  // 1. 팬 핀 초기화 (D11 OUTPUT)
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, LOW);

  // 2. LED 바 핀 초기화 (D8, D9, D10 OUTPUT)
  for (byte i = 0; i < NUM_STRIPS; ++i) {
    pinMode(STRIP_PIN[i], OUTPUT);
    digitalWrite(STRIP_PIN[i], LOW);
  }
  setAllStripsColor(0, 0, 0, "OFF"); // 시작 시 소등

  // 3. IR 센서 핀 초기화 (A3 INPUT_PULLUP)
  pinMode(IR_PIN, INPUT_PULLUP);

  // 4. 리드스위치 (A0, A1, A2 INPUT_PULLUP)
  for (byte i = 0; i < NUM_DOOR; ++i) {
    pinMode(DOOR_PIN[i], INPUT_PULLUP);
    doors[i].raw = digitalRead(DOOR_PIN[i]);
    doors[i].stable = doors[i].raw;
    doors[i].lastDebounceTime = millis();
  }

  Wire.begin();
  Wire.setWireTimeout(3000, true);

  lcdAvailable = initI2CLcd();
  if (lcdAvailable) {
    lcdSetCursor(0, 0);
    lcdPrintStr("SMART VALET TRAY");
    lcdSetCursor(0, 1);
    lcdPrintStr("ICONS & LED BAR ");
    delay(700);
  }

  Serial.println(F("\n======================================================="));
  Serial.println(F(" 까먹는 물건 보관함 · 하드웨어 통합 진단 프로그램 (v2.3) "));
  Serial.println(F("======================================================="));
  Serial.println(F(" [LCD] 8대 커스텀 아이콘 활성화: 🔑, 📦, 🌀, ☂️, 🚪, ✔️, ✕, 💡"));
  Serial.println(F(" [LED] WS2812B 3채널 LED 바 (D8, D9, D10) 지원 완료!"));
  Serial.println(F("  - 'led on'          : 3채널 백색 조명 켜기"));
  Serial.println(F("  - 'led test'        : 색상 자동 순환 점멸 테스트"));
  Serial.println(F("  - 'led auto'        : 뚜껑(A1) 열림 시 백색 조명 자동 연동"));
  Serial.println(F("  - '1' / '0'         : 쿨링팬 ON / OFF"));
  Serial.println(F("  - 'd'               : 리드스위치 실시간 점검\n"));

  printDoorStatus();

  for (byte i = 0; i < NUM_CH; ++i) {
    pinMode(HX_DT[i], INPUT);
    pinMode(HX_CLOCK[i], OUTPUT);
    digitalWrite(HX_CLOCK[i], LOW);
    chs[i].baseline = 0;
    chs[i].currentRaw = 0;
    chs[i].delta = 0;
    chs[i].hasItem = false;
    chs[i].isConnected = false;
    chs[i].lastReady = 0;
  }

  for (byte i = 0; i < NUM_CH; ++i) {
    if (setBaseline(i)) chs[i].isConnected = true;
  }

  updateLcdScreen(true);
}

// ==============================================================================
// 메인 루프 (loop)
// ==============================================================================
void loop() {
  unsigned long currentMillis = millis();

  // 1. 모든 입력(시리얼 모니터, 블루투스) 신호 감지
  handleInputs();

  // 2. 리드스위치 3개 핀 비차단 디바운스 및 상태 변경 감지
  for (byte i = 0; i < NUM_DOOR; ++i) {
    byte currentPinVal = digitalRead(DOOR_PIN[i]);
    if (currentPinVal != doors[i].raw) {
      doors[i].raw = currentPinVal;
      doors[i].lastDebounceTime = currentMillis;
    }

    if ((currentMillis - doors[i].lastDebounceTime) >= 40) {
      if (doors[i].stable != doors[i].raw) {
        doors[i].stable = doors[i].raw;

        Serial.print(F(">>> [리드스위치] "));
        Serial.print(DOOR_NAME[i]);
        if (doors[i].stable == LOW) {
          Serial.println(F(" : 자석 부착 -> 【 닫힘 (CLOSED) 】"));
        } else {
          Serial.println(F(" : 자석 분리 -> 【 열림 (OPEN) 】"));
        }

        // LED AUTO 모드: 뚜껑(A1) 열림 시 백색 조명 자동 점등, 닫힘 시 소등
        if (ledMode == LED_MODE_AUTO && i == 1) {
          if (doors[1].stable != LOW) { // 뚜껑 열림
            setAllStripsColor(120, 120, 120, "WHITE");
            Serial.println(F(">>> [LED AUTO] 뚜껑 열림 감지 -> 백색 조명 자동 ON"));
          } else { // 뚜껑 닫힘
            setAllStripsColor(0, 0, 0, "OFF");
            Serial.println(F(">>> [LED AUTO] 뚜껑 닫힘 감지 -> 조명 소등"));
          }
        }

        updateLcdScreen();
      }
    }
  }

  // 3. IR 센서 읽기 및 상태 변화 처리
  bool currentIr = (digitalRead(IR_PIN) == IR_DETECTED_LEVEL);
  if (currentIr != irDetected) {
    irDetected = currentIr;
    Serial.println(irDetected ? F(">>> [IR 센서] 물건/우산 감지됨!") : F(">>> [IR 센서] 물건/우산 감지 해제!"));

    if (fanAutoMode) {
      setFan(irDetected, irDetected ? "자동: 우산 감지" : "자동: 우산 제거");
    }

    updateLcdScreen();
  }

  // 4. 팬 타이머 및 점멸 테스트 관리
  if (fanUntilTime > 0 && currentMillis >= fanUntilTime) {
    fanUntilTime = 0;
    setFan(false, "설정 시간 만료");
  }

  if (fanTestMode && (currentMillis - lastFanToggleTime >= 2000)) {
    lastFanToggleTime = currentMillis;
    setFan(!fanState, "점멸 테스트 전환");
  }

  // 5. LED 바 테스트 모드 (1초 간격 자동 순환)
  if (ledCycleRunning && (currentMillis - lastLedCycleTime >= 1000)) {
    lastLedCycleTime = currentMillis;
    ledCycleStep = (ledCycleStep + 1) % 5;
    switch (ledCycleStep) {
      case 0: setAllStripsColor(200, 0, 0, "RED"); break;
      case 1: setAllStripsColor(0, 200, 0, "GREEN"); break;
      case 2: setAllStripsColor(0, 0, 200, "BLUE"); break;
      case 3: setAllStripsColor(150, 0, 255, "PURPLE"); break;
      case 4: setAllStripsColor(120, 120, 120, "WHITE"); break;
    }
    updateLcdScreen();
  }

  // 6. 로드셀 2채널 실시간 읽기
  for (byte i = 0; i < NUM_CH; ++i) {
    long raw;
    if (readHX711(i, raw)) {
      chs[i].isConnected = true;
      chs[i].lastReady = currentMillis;

      if (chs[i].currentRaw == 0) {
        chs[i].currentRaw = raw;
      } else {
        chs[i].currentRaw = (chs[i].currentRaw * 6 + raw * 4) / 10;
      }

      chs[i].delta = chs[i].currentRaw - chs[i].baseline;

      if (!chs[i].hasItem && abs(chs[i].delta) < 180) {
        if (chs[i].delta > 5) chs[i].baseline++;
        else if (chs[i].delta < -5) chs[i].baseline--;
      }

      long absDelta = abs(chs[i].delta);
      if (absDelta >= threshold) {
        chs[i].hasItem = true;
      } else if (absDelta < (threshold * 6 / 10)) {
        chs[i].hasItem = false;
      }
    } else {
      if (currentMillis - chs[i].lastReady > 1500) {
        chs[i].isConnected = false;
      }
    }
  }

  updateLcdScreen();

  // 7. 실시간 통합 텔레메트리 출력 (350ms 주기)
  if (currentMillis - lastDisplayTime >= 350) {
    lastDisplayTime = currentMillis;

    // (1) 도어센서 3개 요약
    Serial.print(F("[도어]"));
    Serial.print(doors[0].stable == LOW ? F("현:닫 ") : F("현:열 "));
    Serial.print(doors[1].stable == LOW ? F("뚜:닫 ") : F("뚜:열 "));
    Serial.print(doors[2].stable == LOW ? F("창:닫 ") : F("창:열 "));
    Serial.print(F("| "));

    // (2) 로드셀 CH1 & CH2 상태
    for (byte i = 0; i < NUM_CH; ++i) {
      Serial.print(F("[CH")); Serial.print(i + 1); Serial.print(F("] "));

      if (!chs[i].isConnected) {
        Serial.print(F("미연결        "));
      } else {
        long displayDelta = chs[i].delta;
        if (abs(displayDelta) < deadzone) {
          displayDelta = 0;
        }

        Serial.print(chs[i].currentRaw);
        Serial.print(F("("));
        if (displayDelta >= 0) Serial.print(F("+"));
        Serial.print(displayDelta);
        Serial.print(F(")"));

        if (chs[i].hasItem) {
          Serial.print(F("★[있음]"));
        } else {
          Serial.print(F("  [비움]"));
        }
      }

      if (i == 0) Serial.print(F(" | "));
    }

    // (3) 팬 D11 신호
    Serial.print(F(" | [팬:"));
    Serial.print(fanState ? F("5V ON]") : F("0V OFF]"));

    // (4) LED 바 상태
    Serial.print(F(" [LED:"));
    Serial.print(ledColorName);
    Serial.print(F("]"));

    // (5) IR 센서
    Serial.print(F(" [IR:"));
    Serial.print(irDetected ? F("감지]") : F("---]"));

    Serial.println();
  }
}
