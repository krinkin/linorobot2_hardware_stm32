# План порта прошивки linorobot2_hardware на STM32

## 1. Итог и рекомендация

**Рекомендуемый подход по умолчанию: minimal-diff порт на `platform = ststm32` + `framework = arduino` (STM32duino)**, повторяющий уже существующий compile-time-паттерн ESP32/Pico. Верификация подтверждает, что `micro_ros_platformio` официально поддерживает STM32 **только** через `framework=arduino` (две протестированных платы: `olimex_e407` и `portenta_h7_m7`); `framework=stm32cube` этой библиотекой **не** поддерживается. Нативный путь (CubeMX+HAL+FreeRTOS) — это 3–6 инженер-недель против ~1–2 дней; его берём только при специальных требованиях (см. §6).

**Целевая плата.** По данным верификации (footprint micro-ROS: реалистично ~100–200+ KB Flash и ~25–50 KB рабочей RAM для этого набора entity — 1 node + 3 publishers + 1 subscriber + 1 timer, или 2 pub при `USE_FAKE_MAG`):

- **Основная: Nucleo-F446RE** (`board = nucleo_f446re`, 128 KB RAM / 512 KB Flash, Cortex-M4F). Лучший баланс запаса RAM, FPU для float-математики PID/kinematics и цены. 4 независимых таймера для энкодеров доступны (TIM1/2/3/4/5/8).
- **Запас по памяти / для WiFi/Ethernet в будущем: Nucleo-F767ZI** (`nucleo_f767zi`, 512 KB RAM / 2 MB Flash, M7).
- **Low-cost: WeAct BlackPill F411CE** (`blackpill_f411ce`, 128 KB RAM / 512 KB Flash) — работает по serial, но запас тоньше (TIM8 отсутствует на F411).
- **Самый безопасный «known-good»: Olimex STM32-E407** (`olimex_e407`, F407, 192 KB RAM) — это официально CI-тестируемая micro-ROS-плата. **Рекомендуется начать именно с неё для де-рискинга**, затем перенести на F446RE.
- **Исключено: STM32F103 «BluePill»** (20 KB RAM) — документировано переполнение SRAM; и любые ≤32 KB-RAM части (F0/F1/G0/L0).

> ВАЖНО (подтверждено верификацией): на STM32 **НЕ** использовать `board_microros_user_meta = atomic.meta`. Этот файл задаёт `RCUTILS_NO_64_ATOMIC=OFF` специально для Pico (у pico-sdk свои atomics) и **повторно введёт** ошибку линковки `undefined reference to __sync_synchronize` на STM32. Дефолтная `common.meta` уже имеет `NO_64_ATOMIC=ON` — это то, что нужно STM32. Ключ `board_microros_user_meta` для STM32 **опускаем полностью**.

---

## 2. Что переносится без изменений vs что требует платформенного кода

| Компонент | Файл(ы) | Статус | Почему |
|---|---|---|---|
| Кинематика | `firmware/lib/kinematics/*` | ✅ Без изменений | Чистый C++, только `Arduino.h` для типов |
| PID | `firmware/lib/pid/*` | ✅ Без изменений | Чистый C++ |
| Одометрия | `firmware/lib/odometry/*` | ✅ Без изменений | Чистый C++ |
| Главное приложение | `firmware/src/firmware.ino` | ✅ Без изменений | Уже arch-agnostic; instantiates `Encoder(int,int,int,bool)` и `Motor(freq,bits,inv,pwm,ina,inb)` — новые ветки удовлетворяют эти сигнатуры (подтверждено: строки 105–113) |
| Диспетчер мотора | `firmware/lib/motor/motor.h` | ✅ Без изменений | Выбирает класс по типу драйвера, не по arch |
| IMU/MAG | `firmware/lib/imu/*`, `firmware/lib/mag/*` | ✅ Без изменений | I2Cdevlib + Wire по I2C-пинам из config; есть `FakeIMU` fallback. **Проверить на железе** работу Wire/I2Cdevlib на выбранной плате |
| Servo include | `default_motor.h` (стр. 19–23) | ✅ Без изменений | STM32 проходит через существующую ветку `#else #include <Servo.h>`; STM32duino поставляет `Servo` с `attach(pin)`/`writeMicroseconds(int)` |
| **Энкодер** | `firmware/lib/encoder/encoder.h` | ⚠️ **ТРЕБУЕТСЯ** новая ветка | PJRC `#else` опирается на `utility/direct_pin_read.h`, где **нет** ветки STM32 → `IO_REG_TYPE`/`PIN_TO_BASEREG`/`DIRECT_PIN_READ` не определены → не компилируется |
| **PWM-шим мотора** | `firmware/lib/motor/default_motor.h` (стр. 25–39) | ⚠️ **ТРЕБУЕТСЯ** ветка `#elif defined(STM32)` | STM32duino даёт только 1-арг глобальный `analogWriteFrequency(uint32_t)`; классы зовут 2-арг `analogWriteFrequency(pin,freq)` — этой перегрузки нет → не компилируется |
| Селектор config | `config/config.h` | ⚠️ +1 include-блок | Добавить `USE_STM32_CONFIG` |
| Новый board config | `config/custom/stm32_config.h` | ⚠️ **НОВЫЙ файл** | Геометрия/PID/пины/I2C/`BOARD_INIT` |
| Сборка | `firmware/platformio.ini` | ⚠️ +1 env-блок | Новый `[env:...]` |
| CI | `.github/parse_platformio.py` | ✅ Без изменений | Авто-подхватывает любой не-`teensy` env |
| Fallback config | `config/lino_base_config.h` | ✅ Без изменений (inert) | `stm32_config.h` `#define LINO_BASE` → fallback пропускается |

---

## 3. Рекомендуемый подход (minimal-diff), пошагово по файлам

### 3.1 `firmware/platformio.ini` — новый env-блок

Добавить **выше** барьера `; add maintainer configurations above this line` (стр. 300), по образцу блока `[env:pico]`. Для де-рискинга начинаем с Olimex E407 (официально CI-тестируется), затем добавляем F446RE.

```ini
[env:olimex_e407]
platform = ststm32
board = olimex_e407
framework = arduino
; board_microros_transport и board_microros_distro наследуются из [env] (serial / ${sysenv.ROS_DISTRO})
; ВАЖНО: board_microros_user_meta НЕ задаём — atomic.meta сломает линковку на STM32
upload_protocol = stlink           ; зависит от платы (jlink/blackmagic)
monitor_speed = 115200
lib_deps =
    ${env.lib_deps}
build_flags =
    -I ../config
    -D STM32                       ; arch-макрос для шимов в encoder.h / default_motor.h
    -D USE_STM32_CONFIG            ; селектор board-конфига
```

Для F446RE — тот же блок с `board = nucleo_f446re`.

**Замечания (подтверждено):**
- `${env.lib_deps}` тянет дефолтные SPI/I2Cdevlib — на STM32 пригодны.
- `board_microros_transport = serial` и `board_microros_distro = ${sysenv.ROS_DISTRO}` уже в `[env]` (стр. 15–16) — не дублируем.
- `ARDUINO_ARCH_STM32` авто-определяется ядром ststm32; но **наш собственный** макрос `-D STM32` нужен, потому что шимы в коде завязаны именно на него (см. ниже).
- Этот env авто-подхватится CI (`parse_platformio.py`).

### 3.2 `config/config.h` — селектор

Вставить рядом с другими селекторами, **выше** барьера (после блока `USE_PICO_CONFIG`, стр. 53–55):

```c
#ifdef USE_STM32_CONFIG
    #include "custom/stm32_config.h"
#endif
```

### 3.3 `config/custom/stm32_config.h` — НОВЫЙ файл

Скопировать `pico_config.h` дословно и адаптировать:

- Уникальный guard `STM32_CONFIG_H`.
- `LED_PIN` — `LED_BUILTIN` на Nucleo (на E407 — соответствующий GPIO; на BluePill PC13 active-low → мигание инвертировано, косметика).
- `#define LINO_BASE ...` (копируется из Pico) — **обязательно**, иначе сработает fallback на `lino_base_config.h`.
- `USE_BTS7960_MOTOR_DRIVER` (как в sibling-конфигах) или нужный драйвер; `USE_*_IMU` / `USE_FAKE_IMU`.
- `PWM_BITS 10` + `PWM_FREQUENCY 20000` — STM32duino поддерживает (`analogWriteResolution` зажат до 16 бит, 10 в пределах).
- **Пины энкодеров и моторов** — критично: см. §3.4 о выборе стратегии. Если выбран hardware-timer-энкодер, пары A/B каждого колеса **обязаны** ложиться на CH1/CH2 одного TIMx (нельзя разнести по двум таймерам). Если interrupt-based — пины с **различными номерами** (EXTI шарит линию по номеру пина между портами: PA0/PB0 конфликтуют).
- I2C: по образцу Pico —
  ```c
  #define SDA_PIN PB7   // проверить по datasheet выбранной платы
  #define SCL_PIN PB6
  #define BOARD_INIT { \
      Wire.setSDA(SDA_PIN); \
      Wire.setSCL(SCL_PIN); \
      Wire.begin(); \
      Wire.setClock(400000); \
  }
  ```
  (Pico-стиль `setSDA/setSCL`, **не** ESP32-стиль `Wire.begin(sda,scl)`.) **Внимание:** PB6/PB7 = I2C1 по умолчанию И TIM4_CH1/CH2 — если TIM4 используется под энкодер, перенести I2C или энкодер.
- `BATTERY_PIN`/`BATTERY_ADJUST`, `BAUDRATE 921600`, `NODE_NAME`/`DEVICE_HOSTNAME "stm32"`.

> Точный pin-map (моторы PWM/IN_A/IN_B, энкодеры, I2C) **необходимо** сверить с alternate-function таблицей конкретного корпуса платы. Конфликты CH1/CH2 с PWM/I2C/SPI/USART — реальное ограничение, **не** количество таймеров. Помечаю как **«уточнить по datasheet + проверить на железе»**.

### 3.4 `firmware/lib/encoder/encoder.h` — ветка `#elif defined(STM32)`

Вставить **между** концом блока `#elif defined(PICO)` (стр. 124) и `#else` (стр. 125) — чтобы STM32 **никогда** не попал на PJRC/`direct_pin_read.h`.

Реализовать класс с обязательным API: `Encoder(int pin1,int pin2,int counts_per_rev,bool invert)`, `float getRPM()`, `int32_t read()`, `void write(int32_t)`. Тело `getRPM()` копируется из ветки ESP32/Pico (delta-ticks / counts_per_rev / dtm) — оно идентично.

**Две стратегии (в порядке приоритета по верификации):**

**ROUTE A (рекомендуется — hardware-timer encoder mode, zero ISR-load).** Для 4-колёсной базы на 50 Hz это стандартный STM32-подход: аппаратное x4-декодирование, нулевая нагрузка CPU, без потери счёта на высоких RPM, автоопределение направления. **Важно:** официальное ядро STM32duino `HardwareTimer` **не** имеет публичного encoder-API (PR #576, Issue #628); `setMode(ch, TIMER_ENCODER)` — это старое libmaple-ядро Roger Clark, **не** ststm32. Поэтому:

- *Вариант A1 (минимум своего кода):* подключить через GitHub-URL `lib_deps` библиотеку `juraganled/STM32_QuadEncoder` (MIT; конструктор от **пинов**, выводит таймер через `pinmap_peripheral`, ставит `TIM_ENCODERMODE_TI12`=x4 на официальном ядре). В обёртке: транслировать `int` pin → `PinName`, передать `WITH_PULLUP`. **Критично:** библиотека ставит `ARR = PPR`-аргумент и `getCount()` возвращает `unsigned long` → передавать **`0xFFFF`** как PPR (чтобы счётчик свободно бежал), а физический CPR держать только в `COUNTS_PER_REVn` для формулы RPM. `read()` = sign-extend `(int16_t)enc_.getCount()`; `write(p)` = `enc_.setCount((uint16_t)p)`.
- *Вариант A2 (без внешней либы, ~60 строк, полный контроль overflow):* по `pinmap_peripheral(digitalPinToPinName(pin1), PinMap_PWM)` получить `TIM_TypeDef*`, включить такт таймера, настроить `CCMR1` (TI1/TI2 + input filter), `SMCR.SMS`=encoder-mode-3 (TI12, x4), `ARR=0xFFFF`, `CR1|=CEN`. `read()` = `(int16_t)TIMx->CNT` для 16-бит таймеров или 32-бит CNT для TIM2/TIM5. `write(p)` = `TIMx->CNT = p`.

Skeleton ветки (по образцу ESP32/Pico, тело getRPM копируется):

```c
#elif defined(STM32)
#include "STM32_QuadEncoder.h"   // Route A1; или register-уровень для A2
class Encoder {
    int counts_per_rev_ = -1;
    unsigned long prev_update_time_;
    int32_t prev_encoder_ticks_;
    STM32_QuadEncoder enc_;
public:
    Encoder(int p1,int p2,int cpr,bool invert=false)
      : enc_(invert?p2:p1, invert?p1:p2, WITH_PULLUP, 0xFFFF, DIRECTION_NORMAL) {
        if (p1<0||p2<0) return;     // unused encoder, как в др. ветках
        counts_per_rev_ = cpr;
    }
    int32_t read() { if(counts_per_rev_<0) return 0; return (int16_t)enc_.getCount(); }
    void write(int32_t p) { if(counts_per_rev_<0) return; enc_.setCount((uint16_t)p); }
    float getRPM() { /* идентично ветке ESP32/Pico: delta / cpr / dtm */ }
};
```

> Замечание по 16-бит таймерам: `int32_t read()` корректен через wrap только потому, что `getRPM`/odometry используют **дельту** между опросами, а 50 Hz опрашивает быстрее одного полного wrap. Если когда-либо нужна абсолютная multi-wrap позиция — брать 32-бит TIM2/TIM5 или software-аккумуляцию по UPDATE-прерыванию. **Проверить на железе** знак направления и counts-per-rev (через `MOTOR*_ENCODER_INV` / аргумент invert).

**ROUTE B (fallback / минимальный риск по pin-map — interrupt-based).** Самодостаточная quadrature-обёртка на `attachInterrupt(digitalPinToInterrupt(pin), isr, CHANGE)` + `digitalRead`, `volatile int32_t position`, без внешних `lib_deps`, работает на **любом** GPIO. Минус: дороже по CPU, может терять счёт на высоких RPM на медленных частях; и ограничение EXTI на различные **номера** пинов. Это альтернатива, не дефолт.

### 3.5 `firmware/lib/motor/default_motor.h` — PWM-шим

Добавить ветку `#elif defined(STM32)` в существующий блок шимов (стр. 25–39, после ветки PICO, перед `#include "motor_interface.h"`). STM32duino уже даёт нативные `analogWriteResolution(int)`, `analogWrite(pin,val)`, глобальный `analogWriteFrequency(uint32_t)` и `Servo` — не хватает **только** 2-арг перегрузки `(pin,freq)`, которую зовут `Generic2/Generic1/BTS7960`:

```c
#elif defined(STM32)
// STM32duino: нативный глобальный analogWriteFrequency(uint32_t) — частота per-timer/global.
// Pin намеренно игнорируется; даём 2-арг перегрузку, которую зовут классы моторов.
inline void analogWriteFrequency(uint8_t pin, double frequency)
{
  (void)pin;
  analogWriteFrequency((uint32_t)frequency);
}
```

**Servo include не трогать** — STM32 проходит через существующую `#else #include <Servo.h>`.

**Замечания (подтверждено):** частота PWM на STM32 — **per-timer/global**, не per-pin; вызов на каждый пин безвреден, но избыточен — пины одного таймера разделят последнюю заданную частоту. Для BTS7960 оба IN_A и IN_B должны быть PWM-таймерными пинами. **Проверить на железе** реально достигнутую частоту при 20 kHz/10-бит (ограничения prescaler/ARR на высокой частоте + битности).

### 3.6 micro-ROS transport

Дефолт `serial` @ 921600 (`set_microros_serial_transports(Serial)`, firmware.ino стр. 183) — как у Pico/Teensy, без изменений. WiFi-пути для generic STM32 нет. USB-CDC vs UART зависит от платы (на ST-Link/UART — USART; на платах с native USB — Serial/USB). `micro_ros_platformio` должна предоставить prebuilt rmw-static-lib для ststm32+board+`${ROS_DISTRO}` — **это главный внешний риск**, проверить до мержа (см. §4).

---

## 4. Открытые риски и что проверить на железе

1. **(ВНЕШНИЙ, главный риск)** `micro_ros_platformio` может не иметь prebuilt rmw-static-lib для выбранного ststm32-board + ROS-distro → отказ линковки в CI и локально. **Митигировать:** начать с `olimex_e407` (официально поддержан/CI-тестируется) и **верифицировать сборку до мержа** для остальных плат.
2. **Pin-map (уточнить по datasheet + проверить на железе).** Точные 4 пары timer/CH1/CH2 для энкодеров и пины моторов сверить с alternate-function таблицей конкретного корпуса. Частые коллизии: PB6/PB7 = TIM4_CH1/CH2 **И** I2C1; CH-пины с PWM моторов/SPI/USART. Неверный map → тихо нет PWM или сломан канал энкодера.
3. **Достигнутая частота PWM (проверить на железе).** Семантика `analogWriteFrequency`/`analogWriteResolution` различается между версиями ядра STM32duino; 20 kHz/10-бит достижимо на типовых тактовых, но проверить фактическую частоту осциллографом (особенно для BTS7960). Запинить версию `platform = ststm32`.
4. **16-бит wrap энкодера (проверить на железе).** `int32_t read()` корректен только из-за дельта-расчёта при 50 Hz. На высоких RPM и медленном опросе — риск дрейфа; предпочесть 32-бит TIM2/TIM5 для критичных колёс.
5. **STM32_QuadEncoder caveats (проверить против исходника либы).** `ARR=PPR`, `getCount()`→`unsigned long`: передача физического CPR сломает дельта-математику — передавать `0xFFFF`. Ни одна из кандидатных либ не в реестре PlatformIO → GitHub-URL `lib_dep`; **запинить конкретный commit** для воспроизводимости CI.
6. **Знак направления и CPR (проверить на железе).** Откалибровать через `MOTOR*_ENCODER_INV`/аргумент invert.
7. **Flash/RAM headroom.** Для F411 может потребоваться снизить RMW history/MTU через свой `colcon.meta` (аналогично тому, как Pico нужен `atomic.meta`, но **не** тот же файл). Для F446RE/F767ZI/E407 запас комфортный. Открытые отчёты о HardFault на `rclc_support_init` (F4) обычно из-за недоразмеренных stack/heap — щедро задать linker heap/stack (bare-metal loop, без per-task стека).
8. **Pull-ups энкодеров.** `WITH_PULLUP`/`INPUT_PULLUP` — безопасный дефолт для push-pull; open-collector/длинные кабели могут требовать внешних pull-up и RC-фильтра.
9. **EXTI line-sharing (только Route B).** Два пина с одинаковым **номером** на разных портах тихо ломают один канал — валидировать pin-map.
10. **Servo на ESC-пути.** Если STM32duino `Servo` не собирается для конкретной платы — компиляция сломается даже если ESC не выбран; при необходимости загейтить include. (Дефолт sibling-конфигов — BTS7960, не ESC.)
11. **Wire/I2Cdevlib для IMU/MAG (проверить на железе).** Подтвердить работу I2C-драйверов сенсоров на выбранной плате; `FakeIMU`/`USE_FAKE_MAG` как fallback на ранних фазах.

---

## 5. Фазы и чеклист (milestones)

**Фаза 0 — Скелет/пустая сборка.**
- Добавить `[env:olimex_e407]`, селектор в `config.h`, минимальный `stm32_config.h` (`USE_FAKE_IMU`+`USE_FAKE_MAG`, моторы как `-1`-placeholder, энкодеры `-1`), PWM-шим в `default_motor.h`, ветку энкодера в `encoder.h`.
- **Критерий готовности:** `pio run -e olimex_e407` компилируется и линкуется (включая micro-ROS rmw) **без** `__sync_synchronize`-ошибки; CI-job для нового env зелёный.

**Фаза 1 — Calibration / минимальный bring-up на железе.**
- Прошить, проверить serial-вывод, мигание LED (heartbeat `flashLED`).
- **Критерий:** плата стартует, не уходит в HardFault на `rclc_support_init`; serial читается.

**Фаза 2 — test_motors / PWM.**
- Включить реальный драйвер (BTS7960) с проверенными timer-пинами; прогнать `test_motor` (env уже есть в jazzy).
- **Критерий:** все 4 мотора вращаются вперёд/назад; осциллографом подтверждена частота ~20 kHz и разрешение 10-бит; направления корректны.

**Фаза 3 — Энкодеры.**
- Реальные пины энкодеров (Route A timer-mode). Проверить `read()`/`getRPM()`, знак, counts-per-rev; откалибровать `MOTOR*_ENCODER_INV`.
- **Критерий:** ручное прокручивание колёс даёт корректный знак и масштаб тиков; на максимальных RPM нет потери счёта/дрейфа.

**Фаза 4 — IMU/MAG по I2C.**
- Включить реальный `USE_*_IMU` через `BOARD_INIT` (`Wire.setSDA/setSCL`); проверить I2Cdevlib.
- **Критерий:** `imu/data_raw` (и `imu/mag`, если не `USE_FAKE_MAG`) публикуют валидные данные; `calibrateGyro` отрабатывает.

**Фаза 5 — Полный контур с micro-ROS агентом по serial.**
- Запустить `micro_ros_agent serial` @ 921600; полный цикл cmd_vel → kinematics → PID → motor.spin; публикация odom/imu на 50 Hz.
- **Критерий:** агент стабильно коннектится (без интермиттент-дисконнектов), `cmd_vel` двигает базу, `odom/unfiltered` отражает движение, control-loop держит 50 Hz без джиттера. После этого продублировать env для `nucleo_f446re` и повторить Фазы 0–5.

---

## 6. Когда выбирать нативный HAL/FreeRTOS-подход

Нативный путь (hand-written HAL + FreeRTOS + libmicroros, **GUI-free / без CubeMX**, через `micro_ros_setup`/colcon, отдельный `firmware_stm32/` или submodule, портативный C++ из `firmware/lib/{kinematics,pid,odometry}` переиспользуется как single source of truth, **не** форкается) оправдан **только** при одном из условий:

- Нужна **RTOS-детерминированность** (50 Hz control tick изолирован от executor/publish-джиттера) — за пределами кооперативного `loop()`.
- Нужен **DMA-grade UART** (`ReceiveToIdle_DMA` circular RX + DMA TX) для loss-free serial на высоких скоростях — главный технический выигрыш нативного пути.
- Целевой STM32 **без поддерживаемого Arduino-ядра** (некоторые H7/G4/L4-конфигурации) или нужен прямой доступ к clock tree / low-power / watchdog (всё это пишется руками на HAL/CMSIS — без GUI).
- Требования **safety/сертификации**.

Цена: 3–6 инженер-недель, второй build-system (CMake/Make + arm-none-eabi + colcon libmicroros), который **не** покрывается `parse_platformio.py` (нужен отдельный CI-workflow); IMU/MAG-драйверы (jrowberg I2Cdev) придётся **переписывать** на HAL I2C (не тонкий reuse); custom DMA-transport — самый рискованный компонент (framing/timeouts/ring-overrun). Самый большой риск нативного — **drift** между `firmware.ino` и нативным `app_main.cpp`, если общую логику копировать, а не референсить.

**Для задачи «просто запустить эту прошивку на STM32» нативный путь избыточен — берём minimal-diff (§3).**

---

## Источники

- micro_ros_platformio README (Supported boards: olimex_e407, portenta_h7_m7; transports; `board_microros_distro`/`transport`; clock_gettime requirement) — https://github.com/micro-ROS/micro_ros_platformio
- micro_ros_platformio `metas/common.meta` (`RCUTILS_NO_64_ATOMIC=ON` default) — https://github.com/micro-ROS/micro_ros_platformio/blob/main/metas/common.meta
- micro_ros_platformio `ci/platformio.ini` (реальные `[env:olimex_e407]` / `[env:portenta_h7_m7]`) — https://github.com/micro-ROS/micro_ros_platformio/blob/main/ci/platformio.ini
- PlatformIO Community: STM32 + micro-ROS `undefined reference to __sync_synchronize` и фикс `-DRCUTILS_NO_64_ATOMIC=ON` — https://community.platformio.org/t/stm32-framework-with-micro-ros-stm32cubemx-utils-undefined-references-to-sync-synchronize/42180
- micro_ros_setup issue #529: STM32F407 HardFault на `rclc_support_init` (RAM/stack sizing) — https://github.com/micro-ROS/micro_ros_setup/issues/529
- PlatformIO board: Olimex STM32-E407 (F407, 192 KB RAM) — https://docs.platformio.org/en/latest/boards/ststm32/olimex_e407.html
- ST Nucleo F446RE (PlatformIO) — https://docs.platformio.org/en/latest/boards/ststm32/nucleo_f446re.html
- ST Nucleo F767ZI (PlatformIO) — https://docs.platformio.org/en/latest/boards/ststm32/nucleo_f767zi.html
- WeAct BlackPill F411CE (PlatformIO) — https://docs.platformio.org/en/latest/boards/ststm32/blackpill_f411ce.html
- Arduino_Core_STM32 `wiring_analog.c` (analogWriteFrequency/Resolution/analogWrite) — https://github.com/stm32duino/Arduino_Core_STM32/blob/main/cores/arduino/wiring_analog.c
- Arduino_Core_STM32 `wiring.h` (`extern void analogWriteFrequency(uint32_t)`) — https://github.com/stm32duino/Arduino_Core_STM32/blob/main/cores/arduino/wiring.h
- Arduino_Core_STM32 Wiki API (частота common для всех каналов таймера) — https://github.com/stm32duino/Arduino_Core_STM32/wiki/API
- Arduino_Core_STM32 `libraries/Servo/src/Servo.h` — https://github.com/stm32duino/Arduino_Core_STM32/blob/main/libraries/Servo/src/Servo.h
- Arduino_Core_STM32 PR #576 / Issue #628 (нет encoder-API у HardwareTimer) — https://github.com/stm32duino/Arduino_Core_STM32/pull/576 , https://github.com/stm32duino/Arduino_Core_STM32/issues/628
- juraganled/STM32_QuadEncoder (pin-based ctor, `TIM_ENCODERMODE_TI12`, NO_PULLUP/WITH_PULLUP) — https://github.com/juraganled/STM32_QuadEncoder
- gianni-carbone/STM32encoder (timer-based ctor, managed int32, speed/filters) — https://github.com/gianni-carbone/STM32encoder
- ST AN4013 — Introduction to timers for STM32 MCUs (encoder-mode, 16- vs 32-бит) — https://www.st.com/resource/en/application_note/an4013-introduction-to-timers-for-stm32-mcus-stmicroelectronics.pdf
- DeepBlueEmbedded — STM32 Timer Encoder Mode (x2 vs x4 TI12, ARR/overflow) — https://deepbluembedded.com/stm32-timer-encoder-mode-stm32-rotary-encoder-interfacing/
- ControllersTech — STM32 Incremental Encoder (`TIM_ENCODERMODE_TI12`, CH1/CH2 одного таймера) — https://controllerstech.com/incremental-encoder-with-stm32/
- MicromouseOnline — Quadrature Encoders with the STM32F4 (pin maps, 16-бит overflow через вычитание) — https://micromouseonline.com/2013/02/16/quadrature-encoders-with-the-stm32f4/
- Memory profiling | micro-ROS (per-entity, MTU×history) — https://micro.ros.org/docs/concepts/benchmarking/memo_prof/
- micro-ROS client Memory Profiling | eProsima (~400 B/pub, ~500 B/sub) — https://www.eprosima.com/developer-resources/performance/micro-ros-client-memory-profiling
- Memory Profiling of Micro XRCE-DDS | micro-ROS (<75 KB Flash / ~3 KB RAM, 512 B msgs) — https://micro.ros.org/blog/2020/09/03/memo-prof-xrcedds/
- Supported Hardware | micro-ROS (32-бит, Olimex E407 reference, Nucleo F446 community) — https://micro.ros.org/docs/overview/hardware/
- micro_ros_stm32cubemx_utils issue #35 (F103 20 KB SRAM overflow, ~10 KB task stack) — https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/issues/35
- Micro-ROS on STM32 How-To (min 32 KB RAM / 256 KB Flash, 64 KB+ рекомендуется) — https://thinkrobotics.com/blogs/learn/micro-ros-on-stm32-complete-how-to-guide-for-ros-2-on-mcus

---

**Затронутые файлы (абсолютные пути):**
- `/home/claude/Projects/linorobot2_hardware/firmware/platformio.ini` (новый `[env:...]` выше барьера стр. 300)
- `/home/claude/Projects/linorobot2_hardware/config/config.h` (новый `#ifdef USE_STM32_CONFIG` блок после стр. 55, выше барьера стр. 57)
- `/home/claude/Projects/linorobot2_hardware/config/custom/stm32_config.h` (НОВЫЙ, по образцу `config/custom/pico_config.h`)
- `/home/claude/Projects/linorobot2_hardware/firmware/lib/encoder/encoder.h` (ветка `#elif defined(STM32)` между стр. 124 и 125)
- `/home/claude/Projects/linorobot2_hardware/firmware/lib/motor/default_motor.h` (ветка `#elif defined(STM32)` в блоке шимов стр. 25–39)

**Без изменений:** `firmware/src/firmware.ino`, `firmware/lib/motor/motor.h`, `firmware/lib/kinematics/*`, `firmware/lib/pid/*`, `firmware/lib/odometry/*`, `firmware/lib/imu/*`, `firmware/lib/encoder/utility/direct_pin_read.h` (STM32 его не касается), `config/lino_base_config.h`, `.github/parse_platformio.py`.