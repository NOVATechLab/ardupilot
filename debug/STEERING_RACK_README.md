# Steering Rack — Closed-Loop Control via Relays + Potentiometer

Інтеграція лінійного рульового актуатора (рейки) в ArduRover без Arduino.
Замість окремого мікроконтролера — пряме керування через GPIO реле JarvisH743 з
зворотним зв'язком по потенціометру через вбудований ADC STM32H743.

---

## Архітектура

```
ArduPilot (SRV_Ch1 steering PWM)
        │
        ▼
  steering_rack.lua          ← Lua скрипт, 20 Hz
  ┌─────────────────────┐
  │ PWM → target voltage │
  │ pot_v = ADC read     │
  │ bang-bang control    │
  └──────┬──────────────┘
         │
    ┌────┴────┐
    ▼         ▼
RELAY1(PE8)  RELAY2(PE7)     ← GPIO outputs (3.3V logic)
    │             │
    ▼             ▼
  LEFT          RIGHT         ← Motor driver / relay module
    └──────┬───────┘
           ▼
     Steering rack motor

PB0 (ADC1_INP9) ←──── Potentiometer (0–3.3V)
```

---

## Зміни в hwdef.dat

### Увімкнення ADC

```ini
# До:
define HAL_USE_ADC FALSE
define STM32_ADC_USE_ADC1 FALSE
define HAL_DISABLE_ADC_DRIVER TRUE

# Після:
define STM32_ADC_USE_ADC1 TRUE
PB0 ADC1 ADC1
```

### Переконфігурація реле (PB0 звільнено під ADC)

```ini
# До:
PB0  PINIO1 OUTPUT GPIO(71) LOW   ← тепер ADC
PB1  PINIO2 OUTPUT GPIO(72) LOW
PB2  PINIO3 OUTPUT GPIO(73) LOW
PE7  PINIO4 OUTPUT GPIO(74) LOW
PE8  PINIO5 OUTPUT GPIO(75) LOW

define RELAY1_PIN_DEFAULT 75
define RELAY2_PIN_DEFAULT 74
define RELAY3_PIN_DEFAULT 73
define RELAY4_PIN_DEFAULT 72
define RELAY5_PIN_DEFAULT 71

# Після:
PB1  PINIO1 OUTPUT GPIO(71) LOW
PB2  PINIO2 OUTPUT GPIO(72) LOW
PE7  PINIO3 OUTPUT GPIO(73) LOW
PE8  PINIO4 OUTPUT GPIO(74) LOW

define RELAY1_PIN_DEFAULT 74  # PE8 — LEFT
define RELAY2_PIN_DEFAULT 73  # PE7 — RIGHT
define RELAY3_PIN_DEFAULT 72  # PB2
define RELAY4_PIN_DEFAULT 71  # PB1
```

> **RELAY5 (PB0) більше не існує** — пін перероблено на ADC вхід.

---

## Підключення

### Потенціометр → FC

| Потенціометр | JarvisH743 |
|---|---|
| + живлення | 3.3V |
| – живлення | GND |
| Wiper (середній вивід) | **PB0** (ADC1_INP9) |

> Потенціометр має бути розрахований на напругу 3.3V. Якщо живиться від 5V —
> потрібен дільник напруги або рейка буде пошкоджена!

### Реле / H-bridge → FC

| Сигнал | Пін FC | GPIO | Реле AP |
|---|---|---|---|
| LEFT direction | **PE8** | 74 | RELAY1 |
| RIGHT direction | **PE7** | 73 | RELAY2 |

> Виходи PE8/PE7 — **3.3V логіка**, не призначені для прямого керування реле.
> Обов'язково потрібен драйвер: relay module (з оптронами) або H-bridge (L298N, BTS7960 тощо).

---

## Lua скрипт

**Файл:** `libraries/AP_HAL_ChibiOS/hwdef/JarvisH743/scripts/steering_rack.lua`

### Логіка роботи

1. Читає `SRV_Channels` output ch1 (GroundSteering, `SERVO1_FUNCTION=26`) — бажаний кут
2. Маппить PWM (1100–1900 мкс) → цільова напруга потенціометра
3. Читає поточну напругу потенціометра (PB0, ADC1_INP9)
4. Bang-bang: вмикає LEFT або RIGHT реле поки похибка > deadband
5. Зупиняє реле при досягненні цілі або апаратних меж рейки

### Захист

- Перевірка pot_v < 0.1V або > 3.3V → зупинка + повідомлення в GCS (fault)
- Апаратні межі: зупинка при pot_v ≤ V_MIN+50мВ або ≥ V_MAX-50мВ

---

## Параметри GCS (вкладка JRVS_)

| Параметр | За замовч. | Опис |
|---|---|---|
| `JRVS_RACK_V_MIN` | 2.00 | Напруга потенціометра на лівому упорі (V) |
| `JRVS_RACK_V_CTR` | 2.62 | Напруга в центральному положенні (V) |
| `JRVS_RACK_V_MAX` | 3.26 | Напруга на правому упорі (V) |
| `JRVS_RACK_DB_C`  | 0.07 | Deadband коли рейка в центрі (V) |
| `JRVS_RACK_DB_M`  | 0.04 | Deadband під час руху до цілі (V) |

> Зміна параметрів через GCS **не потребує перезбірки** firmware.

---

## Калібрування (після першої прошивки)

1. Переконатись що `SERVO1_FUNCTION = 26` (GroundSteering)
2. Переконатись що `RELAY_PIN1 = 74`, `RELAY_PIN2 = 73`
3. Відкрити MAVLink Inspector або Serial Monitor
4. Встановити рейку вручну в крайнє **ліве** положення → записати pot_v → вписати в `JRVS_RACK_V_MIN`
5. Встановити рейку в **центр** → записати pot_v → `JRVS_RACK_V_CTR`
6. Встановити рейку в крайнє **праве** положення → записати pot_v → `JRVS_RACK_V_MAX`
7. Зберегти параметри (Write) в GCS
8. Перевірити напрямок: якщо LEFT і RIGHT переплутані — поміняти `RELAY_LEFT` і `RELAY_RIGHT` в скрипті або переключити проводку

---

## Порівняння з Arduino-рішенням

| | Arduino (було) | JarvisH743 Lua (є) |
|---|---|---|
| Зчитування PWM | Pin-change ISR | ArduPilot SRV_Channels |
| Режими MANUAL/AUTO | CH_MODE PWM | ArduPilot flight mode |
| Потенціометр | analogRead A0 (10-bit) | ADC1_INP9 (16-bit) |
| Реле керування | digitalWrite D6/D10 | relay:on/off (RELAY1/2) |
| Параметри | Хардкод у коді | GCS параметри JRVS_* |
| Пристроїв | 2 (FC + Arduino) | 1 (тільки FC) |

---

## Файли

```
libraries/AP_HAL_ChibiOS/hwdef/JarvisH743/
├── hwdef.dat                        ← зміни ADC + relay
└── scripts/
    └── steering_rack.lua            ← скрипт керування

debug/
├── Arduino-Steering.txt             ← оригінальний Arduino скетч клієнта
└── STEERING_RACK_README.md          ← цей файл
```
