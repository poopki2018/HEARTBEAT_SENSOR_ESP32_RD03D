# ESP32 and RD-03D Sensor based Heartbeat Sensor 

스마트폰 브라우저에서 실제 레이더로 사람을 확인하는 프로젝트.

ESP32가 Wi-Fi 서버로 동작하므로 추가 앱 설치 불필요. 

## 주요 기능
- 최대 3명 동시 추적 (다중 모드) / 1명만 추적 (단일 모드)
- 표적별 거리·방위·속도, 접근/이탈 판정 표시
- 60fps 스캔 빔 애니메이션, 잔상 트레일, CRT 스캔라인 효과
- 가장 가까운 표적에 LOCK 마커 표시
- 탐지 반경 2 / 4 / 6 / 8 m 전환
- 스윕이 표적을 지날 때 거리에 따라 음높이가 바뀌는 핑 사운드 (기본 OFF)
- 가로모드 전용 레이아웃: HUD·표적목록·조작버튼이 오른쪽 세로열로 이동
- 전체화면 지원

## 준비물

| 부품 | 비고 |
|---|---|
| RD-03D 레이더 모듈 | Ai-Thinker 24GHz, UART 256000 bps |
| ESP32 개발보드 | ESP32 Dev Module 계열 |
| 5V 전원 | 레이더 모듈에 최소 200mA 필요 |

## 배선

| RD-03D | ESP32 |
|---|---|
| VCC | 5V (Vin) |
| GND | GND |
| TXD | GPIO16 (RX2) |
| RXD | GPIO17 (TX2) |

## 사용법

1. 스마트폰을 이 Wi-Fi에 접속 (SSID: `RD03D-RADAR` / PW: `12345678`, 인터넷이 없다는 경고는 "항상 연결" 터치)
2. 브라우저에서 **http://192.168.4.1** 로 접속
3. 위치는 0.2초마다 갱신.

### 화면 조작

| 컨트롤 | 설명 |
|---|---|
| RANGE | 표시 반경 선택 (2 / 4 / 6 / 8 m) |
| MODE | 다중(최대 3명) / 단일(1명) 추적 전환 |
| ♪ | 표적 감지 핑 사운드 ON/OFF |
| ⛶ | 전체화면 진입/해제 (레이더 화면 두 번 탭해도 토글) |

## 설정 변경

```cpp
#define RADAR_RX_PIN   16
#define RADAR_TX_PIN   17
#define RADAR_BAUD     256000      // RD-03D 고정값

#define USE_AP         true        // false로 바꾸면 집 공유기에 접속
const char* AP_SSID  = "RD03D-RADAR";
const char* AP_PASS  = "12345678"; // 8자 이상
const char* STA_SSID = "your-ssid";
const char* STA_PASS = "your-password";

#define TARGET_TIMEOUT_MS 700      // 갱신 없으면 표적 소멸까지의 시간
```

* `USE_AP`를 `false`로 두면 공유기 접속 모드.
* 접속 주소는 시리얼 모니터(115200bps)에 출력.
* 시리얼 모니터에서 `IP`를 입력하면 언제든 현재 IP·RSSI·MAC 정보 확인 가능.

## HTTP 엔드포인트

| 경로 | 설명 |
|---|---|
| `GET /` | 스코프 화면 (HTML) |
| `GET /data` | 표적 데이터 (JSON) |
| `GET /mode?m=multi\|single` | 추적 모드 전환 |

`/data` 응답 예시:

```json
{
  "ms": 154820,
  "mode": "multi",
  "frames": 7412,
  "link": true,
  "targets": [
    { "i": 0, "x": -0.31, "y": 0.65, "d": 0.72, "a": -25.5, "s": 0.14 }
  ]
}
```

`x`는 좌(-)/우(+) 오프셋, `y`는 정면 거리, `d`는 직선거리(m), `a`는 방위각(도, 0=정면), `s`는 속도(m/s)

## 프레임 포맷

다중 표적 모드의 한 프레임은 30바이트.

```
AA FF 03 00 | 표적1 (8B) | 표적2 (8B) | 표적3 (8B) | 55 CC
```

표적 8바이트는 `X(2) Y(2) SPEED(2) RES(2)` 순서, 각 값의 **최상위 비트가 부호** (1이면 양수, 0이면 음수). 나머지 15비트가 크기값. 일반적인 2의 보수가 아님.

```cpp
uint16_t mag = ((hi & 0x7F) << 8) | lo;
float v = (hi & 0x80) ? (float)mag : -(float)mag;
```

## 라이선스

MIT
