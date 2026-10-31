# 아마따 (AMATTA) · Smart Tray Arduino Firmware

**아마따(AMATTA)**는 현관의 소지품 트레이 2칸과 접이식 우산 보관 공간 1칸을 Android 앱([`smart-tray-app`](https://github.com/yjjyjjyjjyjj/smart-tray-app))과 Bluetooth Classic(HC-06)으로 연동하여, 외출할 때 소지품이나 우산을 두고 나가지 않도록 돕는 스마트 보관함의 **Arduino UNO R3 펌웨어 저장소**입니다.

---

## 📁 저장소 구조

```text
smart-tray-arduino/
├── README.md                      # 펌웨어 기능 명세, 핀 배치, JSON 통신 프로토콜 및 캘리브레이션 가이드
├── main_edit.c/
│   └── main_edit.c.ino            # Android 앱 연동 및 하드웨어 자율 제어 메인 프로덕션 펌웨어 (v1.0)
└── test_module/
    └── test_module.ino            # 하드웨어 단품/통합 정밀 진단 및 테스트용 펌웨어 (v2.3)
```

* **외부 라이브러리 무의존성 설계:**  
  `main_edit.c.ino`는 아두이노 기본 내장 라이브러리(`Arduino.h`, `Wire.h`, `EEPROM.h`, `SoftwareSerial.h`)만 사용합니다. 별도의 `Adafruit_NeoPixel`, `LiquidCrystal_I2C`, `HX711` 라이브러리를 설치하지 않아도 **즉시 컴파일 및 업로드**할 수 있도록 순수 AVR 16MHz 어셈블리 WS2812B 드라이버와 경량 I²C LCD/HX711 드라이버를 내장하고 있습니다.

---

## 🔌 하드웨어 구성 및 핀 배치 (Arduino UNO R3)

| 공간 / 역할 | 부품명 | UNO 핀 번호 | 입출력 / 비고 |
| :--- | :--- | :--- | :--- |
| **1번 트레이 무게 감지** | 로드셀 + HX711 (#1) | `D2` (DT), `D3` (SCK) | 24-bit ADC, 용기 무게 차감 및 `g` 환산 |
| **2번 트레이 무게 감지** | 로드셀 + HX711 (#2) | `D4` (DT), `D5` (SCK) | 24-bit ADC, 1번 트레이와 독립 동작 |
| **3번 우산 공간 감지** | IR 적외선 장애물 센서 | `A3` (OUT) | `INPUT_PULLUP`, 우산 감지 시 `LOW` |
| **현관문 개폐 감지** | 자석 도어센서 (리드스위치) | `A0` | `INPUT_PULLUP`, 닫힘(자석 부착) `LOW` / 열림 `HIGH` |
| **1번 트레이 공용 LED** | WS2812B 스트립 (8구) | `D8` (DIN, 330Ω 직렬) | 일반 점멸(백색) 및 살균(보라색) 공유 출력 |
| **2번 트레이 공용 LED** | WS2812B 스트립 (8구) | `D9` (DIN, 330Ω 직렬) | 일반 점멸(백색) 및 살균(보라색) 공유 출력 |
| **3번 우산 공간 공용 LED**| WS2812B 스트립 (8구) | `D10` (DIN, 330Ω 직렬)| 일반 점멸(백색) 및 살균(보라색) 공유 출력 |
| **우산 건조 쿨링팬** | 12V 팬 (MOSFET 스위치 구동) | `D11` | `HIGH`(5V) = 팬 ON / `LOW`(0V) = 팬 OFF |
| **Bluetooth 통신** | HC-06 (Classic SPP) | `D12` (RX), `D13` (TX) | `9600 bps, 8N1` (`D13` → 1kΩ/2kΩ 분압 → HC-06 RXD) |
| **날씨 디스플레이** | 1602 I²C LCD (16×2) | `A4` (SDA), `A5` (SCL) | I²C 주소 `0x27` (또는 `0x3F` 자동 탐색) |

> **⚠️ 미사용 핀 제외 안내:** 초기 회로도의 뚜껑(`A1`) 및 창문(`A2`) 도어센서는 실제 구현에서 사용하지 않으므로 제외되었습니다.

---

## ⚙️ 주요 하드웨어 동작 정책

### 1. 설정 기반 자율 액션 & 실시간 이벤트 동기화
* **앱 → 아두이노:** 공간별 설정값(`set_config`), 수동 제어 명령(`cmd`), 기상청 날씨 정보(`weather`)를 전송합니다. 수신된 설정값은 아두이노 내장 **`EEPROM`에 CRC16과 함께 영구 저장**되어 재부팅 후에도 유지됩니다.
* **아두이노 자율 실행:** 저장된 설정값에 따라 센서 상태가 변하면 아두이노가 자체적으로 LED·살균·팬 타이머를 즉시 구동합니다.
* **즉시 상태 보고:** 센서 값(물건/우산 거치·수거, 현관문 개폐, 2.0g 이상의 무게 변화)이나 출력 상태(LED/살균/팬 시작·종료)가 바뀌면 **즉시(`stateDirty`) 앱으로 `state` JSON 패킷을 전송**합니다. 상태 변화가 없더라도 연결 유지 및 신선도 보장을 위해 **1초(`1000ms`) 주기**로 스냅샷을 전송합니다.

### 2. 물건 / 우산 거치 및 수거 동작
* **거치 감지 시 (`present: false → true`):**
  * **일반 백색 LED는 켜지 않고 즉시 소등(`OFF`)** 상태를 유지하며, 곧바로 앱에 업데이트된 상태 데이터를 전송합니다.
  * 해당 공간의 **거치 시 자동 살균(`steril_en && steril_detect`)**이 켜져 있으면 보라색 살균 타이머(`5/10/20/30초`)를 시작합니다.
  * 3번 우산 공간에 우산이 거치되었고 **우산 건조 팬 시간(`fan_sec`: `15/30/60초`)**이 설정되어 있으면 즉시 쿨링팬(`D11`) 타이머를 가동합니다.
* **물건 / 우산 제거 시 (`present: true → false`):**
  * 동작 중이던 일반 점멸, 보라색 살균 타이머, 우산 건조 팬을 **즉시 중단(소등/정지)**하고 앱으로 상태 변경 및 중단 알림(`ack: "stopped", reason: "item_removed"`)을 전송합니다.

### 3. 공간별 공용 LED (WS2812B) 제어 규칙
각 공간(1·2·3번)의 LED 스트립은 **살균 표시**와 **일반 미수거 알림** 두 가지 역할을 공유하며, 우선순위는 다음과 같습니다.

| 우선순위 | 모드 (`led`) | 색상 및 동작 방식 | 시작 및 종료 조건 |
| :---: | :--- | :--- | :--- |
| **1순위** | **살균 점등**<br>(`"steril"`) | **보라색 연속 점등**<br>`RGB(150, 0, 255)` | • **시작:** 거치 시 자동(`steril_detect`), 문 열림 시 자동(`steril_door`), 또는 앱 수동 명령(`start_steril`)<br>• **종료:** 설정 시간(`5/10/20/30초`) 만료, 물건 제거, 수동 중지(`stop_steril`) |
| **2순위** | **일반 점등**<br>(`"normal"`) | **백색 1.5초 주기 점멸**<br>`RGB(120, 120, 120)`<br>(`1.5초 ON ↔ 1.5초 OFF`) | • **시작:** 현관문이 열릴 때(`closed → open`), 해당 칸에 물건/우산이 남아 있고 `led_en && led_door`가 켜져 있을 때<br>• **종료:** 물건을 집어 들거나(수거), 물건을 새로 거치하거나, 현관문이 닫히거나, 최대 60초 경과 시 즉시 소등 |
| **기본** | **소등**<br>(`"off"`) | `RGB(0, 0, 0)` | 위 조건에 해당하지 않을 때 |

> **💡 점멸 중 상태 값 유지:** 일반 점등이 `1.5초 ON ↔ 1.5초 OFF`로 깜빡이는 동안 물리 LED가 잠시 꺼진 구간(OFF 페이즈)에 있더라도, 앱에 전송하는 JSON의 `led` 필드는 `"normal"`로 일정하게 유지됩니다. 또한 살균(`"steril"`)이 끝났을 때 현관문이 아직 열려 있고 미수거 점멸 조건이 유효하면 자동으로 일반 백색 점멸로 복귀합니다.

### 4. 1602 I²C LCD 날씨 디스플레이
* **부팅 직후 (앱에서 기상 정보를 받기 전):**
  * 좌측 `2×2` 칸에 시계/대기 커스텀 도트 아이콘을 표시하고, 우측에 **`WAITING...` / `WEATHER INFO` (기상 정보 대기중)** 문구를 출력합니다.
* **기상 정보 수신 후:**
  * 좌측 `2×2` 칸(CGRAM 0~3번 슬롯 동적 교체)에 **해(`SUNNY`), 구름(`CLOUDY`), 비(`RAIN`), 눈(`SNOW`)** 큰 도트 아이콘을 출력하고, 우측 1행에 날씨 영문명, 2행에 비 예보 시간대(`14:00-18:00` 등) 또는 안내 문구(`NO RAIN`, `NO RAIN/SNOW`)를 표시합니다.
  * 비 예보 시간대(`windows`)가 2~3개 전달된 경우 **3초 간격으로 자동 순환 표시**합니다.
  * 유효 시간(`ttl_s`)이 만료되거나 `valid: false` 수신 시 **`NO DATA`** 화면으로 전환됩니다.

---

## 📡 Arduino ↔ Android JSON 통신 프로토콜 (v1.0)

### 1. 프레이밍 및 전송 규칙
* **통신 방식:** Bluetooth Classic SPP (RFCOMM), UART `9600 bps, 8N1`
* **메시지 구분 (NDJSON):** 모든 패킷은 줄바꿈 없는 **한 줄 UTF-8 JSON 문자열 + `\n` (LF)**으로 송수신합니다. (`\r`은 무시)
* **채널 분리:** 시리얼 모니터 디버그/캘리브레이션 로그는 USB `Serial`(`115200 bps`)로만 출력되며, Bluetooth(`HC-06`) 채널에는 순수 JSON 패킷만 송수신됩니다.

---

### 2. Arduino → Android 송신 메시지 (2종)

#### 2-1. 전체 상태 스냅샷 (`type: "state"`)
내부 상태 변화 시 즉시, 그리고 평상시 1초 주기로 전송됩니다.

```json
{"type":"state","v":1,"boot":3,"seq":42,"door":{"state":"open","sensor":"ok"},"trays":[{"id":1,"present":true,"weight":128.5,"sensor":"ok","led":"normal","steril_ms":0},{"id":2,"present":false,"weight":0.0,"sensor":"ok","led":"off","steril_ms":0}],"umbrella":{"present":true,"sensor":"ok","led":"steril","steril_ms":8000},"fan":{"active":true,"rem_ms":15000}}
```

| 필드 경로 | 타입 | 허용 값 / 단위 | 설명 |
| :--- | :--- | :--- | :--- |
| `v` | Int | `1` | 프로토콜 버전 |
| `boot` | Int | `1 ~ 65535` | 부팅 카운트 (`EEPROM` 저장, 재부팅 감지용) |
| `seq` | Long | `1 ~` | 송신 패킷 시퀀스 번호 |
| `door.state` | String | `"open"` \| `"closed"` | 현관문(`A0`) 열림/닫힘 상태 |
| `door.sensor` | String | `"ok"` \| `"error"` | 현관문 센서 상태 |
| `trays[].id` | Int | `1`, `2` | 물리 트레이 번호 (1번 칸, 2번 칸) |
| `trays[].present` | Boolean | `true` \| `false` | 물건 유무 (임계값·히스테리시스·250ms 안정화 적용) |
| `trays[].weight` | Double | `0.0 ~ 3000.0` | 용기 차감 후 순수 물건 무게 (`g`, 소수점 1자리) |
| `trays[].sensor` | String | `"ok"` \| `"waiting"` \| `"error"` | 로드셀(HX711) 상태 (`1.5초` 이상 응답 없을 시 `"error"`) |
| `trays[].led` | String | `"off"` \| `"normal"` \| `"steril"` | 해당 트레이 공용 LED 논리 동작 상태 |
| `trays[].steril_ms`| Long | `0 ~ 30000` | 살균 남은 시간 (`ms`) |
| `umbrella.present` | Boolean | `true` \| `false` | 3번 우산 공간 우산 감지 유무 (`A3` IR 센서) |
| `umbrella.sensor` | String | `"ok"` \| `"waiting"` \| `"error"` | IR 센서 상태 |
| `umbrella.led` | String | `"off"` \| `"normal"` \| `"steril"` | 3번 우산 공간 공용 LED 논리 동작 상태 |
| `umbrella.steril_ms`| Long | `0 ~ 30000` | 3번 우산 공간 살균 남은 시간 (`ms`) |
| `fan.active` | Boolean | `true` \| `false` | 우산 건조 팬(`D11`) 실제 구동 여부 |
| `fan.rem_ms` | Long | `0 ~ 60000` | 우산 건조 팬 남은 동작 시간 (`ms`) |

#### 2-2. 명령 응답 및 완료 통지 (`type: "ack"`)
앱이 보낸 명령의 접수/거절 결과 및 수동 동작의 완료/중단 사유를 알립니다.

```json
{"type":"ack","req":"app-21","status":"accepted"}
{"type":"ack","req":"app-21","status":"rejected","reason":"no_item"}
{"type":"ack","req":"app-21","status":"completed","reason":"duration_elapsed"}
{"type":"ack","req":"app-21","status":"stopped","reason":"item_removed"}
```
* `status`: `"accepted"` \| `"rejected"` \| `"completed"` \| `"stopped"`
* `reason`: `"invalid_json"`, `"invalid_space"`, `"steril_disabled"`, `"sensor_error"`, `"no_item"`, `"already_running"`, `"duration_elapsed"`, `"item_removed"`, `"manual_stop"`, `"disabled"`

---

### 3. Android → Arduino 수신 메시지 (4종)

#### 3-1. 즉시 상태 조회 (`type: "get_state"`)
```json
{"type":"get_state","req":"app-01"}
```

#### 3-2. 공간별 설정 동기화 (`type: "set_config"`)
* **1·2번 트레이 설정 예시:**
```json
{"type":"set_config","req":"app-10","space":1,"led_en":true,"led_door":true,"steril_en":false,"steril_detect":false,"steril_door":false,"steril_sec":10}
```
* **3번 우산 공간 설정 예시 (`fan_sec` 포함):**
```json
{"type":"set_config","req":"app-11","space":3,"led_en":true,"led_door":true,"steril_en":true,"steril_detect":true,"steril_door":false,"steril_sec":10,"fan_sec":30}
```
* `space`: `1`(1번 트레이), `2`(2번 트레이), `3`(3번 우산 공간)
* `steril_sec`: `5`, `10`, `20`, `30` (그 외 값은 `10`초 및 `steril_en: false`로 안전 초기화)
* `fan_sec`: `0`(사용 안 함), `15`, `30`, `60` (`space: 3` 전용)

#### 3-3. 수동 제어 명령 (`type: "cmd"`)
```json
{"type":"cmd","req":"app-21","space":1,"action":"start_steril"}
{"type":"cmd","req":"app-22","space":1,"action":"stop_steril"}
{"type":"cmd","req":"app-23","space":3,"action":"start_fan","sec":30}
{"type":"cmd","req":"app-24","space":3,"action":"stop_fan"}
{"type":"cmd","req":"app-25","space":1,"action":"tare"}
```

#### 3-4. 1602 LCD 날씨 표시 업데이트 (`type: "weather"`)
```json
{"type":"weather","req":"app-33","valid":true,"icon":"rain","line1":"RAIN","windows":["14:00-18:00","20:00-22:00"],"ttl_s":3600}
{"type":"weather","req":"app-34","valid":true,"icon":"snow","line1":"SNOW","windows":["NO RAIN"],"ttl_s":3600}
{"type":"weather","req":"app-35","valid":true,"icon":"sunny","line1":"SUNNY","windows":["NO RAIN/SNOW"],"ttl_s":3600}
{"type":"weather","req":"app-36","valid":false,"icon":"none","line1":"NO DATA","windows":[""],"ttl_s":0}
```
* `icon` (또는 `display_condition`): `"sunny"`(`"clear"`), `"cloudy"`(`"partly-cloudy"`), `"rain"`(`"shower"`, `"rain-snow"`), `"snow"`, `"none"`

---

## ⚖️ 시리얼 모니터 캘리브레이션 및 하드웨어 수동 테스트 명령어

하드웨어 조립 상태에 맞춰 로드셀을 보정하고, 앱 연결 없이도 모든 센서·출력·LCD·이벤트를 즉시 점검할 수 있도록 **USB 시리얼 모니터(`115200 bps`, 줄바꿈 `LF` 또는 `Both NL & CR`)** 전용 명령어를 지원합니다.

### 1. 상태 조회 및 실시간 모니터
| 시리얼 명령어 | 동작 설명 |
| :--- | :--- |
| `HELP` (또는 `H`, `?`) | 사용 가능한 시리얼 캘리브레이션 및 수동 테스트 명령어 전체 안내 출력 |
| `STATUS` | 현재 전체 상태 JSON(`type: "state"`)을 시리얼 모니터로 1회 출력 |
| `MON` (또는 `M`) | **실시간 텔레메트리 모니터 ON/OFF 토글** (500ms 주기로 문·트레이 무게·우산·LED·팬 상태 연속 출력) |
| `SENSOR` (또는 `D`) | 현관문(`A0`), 우산 IR(`A3`), 건조 팬(`D11`), 공간별 LED/살균 설정 및 로드셀 상세 리포트 출력 |

### 2. 로드셀 캘리브레이션 및 EEPROM 관리
| 시리얼 명령어 | 동작 설명 |
| :--- | :--- |
| `CAL SHOW` | 1·2번 로드셀의 현재 Raw값, Tare 기준값, Scale Factor, 측정 무게(`g`), 감지/해제 문턱값 조회 |
| `TARE 1` / `TARE 2` | 해당 트레이가 비어 있는 상태에서 현재 무게를 `0.0g` 영점으로 설정 (`TARE`만 입력 시 1·2번 동시 영점) |
| `CAL 1 200` | `TARE 1` 수행 후, 1번 트레이에 `200g` 기준 분동(또는 물건)을 올리고 입력하면 Scale Factor 자동 계산 |
| `SCALE 1 25.4` | 1번 트레이의 Scale Factor(`1g`당 Raw 변화량)를 수동으로 직접 지정 |
| `TH 1 15.0 8.0` | 1번 트레이의 물건 감지 문턱값을 `15.0g`, 해제 문턱값을 `8.0g`으로 변경 |
| `DZ 1 3.0` | 1번 트레이의 진동 제거 제로 데드존을 `3.0g`으로 설정 (`3.0g` 미만은 `0.0g`으로 고정) |
| `SAVE` / `RESET` | 현재 보정값 및 설정을 **`EEPROM`에 영구 저장** / 공장 초기값으로 리셋 |

### 3. 공유 LED (WS2812B 3채널) & 건조 팬 수동 제어 테스트
| 시리얼 명령어 | 동작 설명 |
| :--- | :--- |
| `LED 1 STERIL 10` | 1번 칸(`1`, `2`, `3` 또는 `ALL` 지정 가능) **보라색 살균 연속 점등**을 `10`초간 강제 시작 (시간 생략 시 10초) |
| `LED 1 BLINK` | 1번 칸(`1`, `2`, `3` 또는 `ALL`) **백색 1.5초 주기 일반 점멸** 강제 시작 |
| `LED ALL OFF` | 지정 칸(`1`, `2`, `3` 또는 `ALL`)의 살균 및 일반 점멸 즉시 소등 (`OFF`) |
| `FAN ON 15` | 우산 건조 팬(`D11`)을 `15`초간 수동 가동 (시간 생략 시 30초 가동) |
| `FAN OFF` | 우산 건조 팬(`D11`) 즉시 정지 (`0V` 차단) |

### 4. 1602 LCD 날씨 프리셋 수동 테스트
| 시리얼 명령어 | 동작 설명 |
| :--- | :--- |
| `LCD WAIT` | `WAITING... / WEATHER INFO` (**기상 정보 대기중** 시계 아이콘 표시) |
| `LCD SUNNY` | `SUNNY / NO RAIN/SNOW` (해 `2×2` 도트 아이콘 표시) |
| `LCD CLOUDY` | `CLOUDY / NO RAIN/SNOW` (구름 `2×2` 도트 아이콘 표시) |
| `LCD RAIN 14:00-18:00 20:00-22:00` | `RAIN` (비 `2×2` 아이콘 + 비 예보 시간대 표시, 시간대 2개 입력 시 3초 순환 테스트) |
| `LCD SNOW` | `SNOW / NO RAIN` (눈 `2×2` 도트 아이콘 표시) |
| `LCD NODATA` | `NO DATA` (기상 정보 수신 실패/만료 아이콘 표시) |

### 5. 이벤트 시뮬레이션 & 블루투스 강제 송신 테스트
| 시리얼 명령어 | 동작 설명 |
| :--- | :--- |
| `SIM DOOR OPEN` / `CLOSE` | 물리 자석을 움직이지 않고도 **현관문 열림/닫힘 이벤트**를 강제 발생시켜 미수거 LED 점멸 및 앱 알림 연동 테스트 |
| `SIM ITEM 1 1` / `SIM ITEM 1 0` | 1번 트레이(`1`, `2`) 또는 3번 우산(`3`)의 **거치(`1`)/수거(`0`) 이벤트**를 강제 발생시켜 자동 살균·팬 구동 및 앱 전송 테스트 |
| `BT SEND` | 현재 `state` JSON 패킷을 HC-06 블루투스 채널로 즉시 강제 전송 |

> **💡 팁:** 시리얼 모니터 입력창에 `{"type":"weather","valid":true,"icon":"rain","line1":"RAIN","windows":["14:00-18:00"]}` 처럼 JSON 문자열을 직접 입력해도 Bluetooth 수신과 동일하게 파싱 및 테스트할 수 있습니다.
