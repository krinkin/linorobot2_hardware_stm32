# План порта linorobot2_hardware на нативный STM32Cube (HAL) — надёжный план

> Это **отдельный** план для нативного пути STM32Cube (ST HAL/LL + CMSIS, без Arduino-прослойки).
> Arduino/STM32duino-вариант (minimal-diff) описан в соседнем `docs/STM32_PORTING_PLAN.md` и здесь
> **не** заменяется. Этот документ — design-spec; детальный план реализации генерируется отдельно
> (writing-plans) на его основе.

## 0. Резюме и зафиксированные решения

**Цель:** перенести прошивку низкоуровневого контроллера linorobot2 на нативный STM32Cube так, чтобы
код был **универсален по любому STM32** (с поправкой: практический пол — чип с FPU и ≥ ~64 КБ RAM;
F0/L0/G0/F1-BluePill исключены — micro-ROS туда не влезает).

**Развилка интеграции micro-ROS решена в пользу официального нативного пути (Маршрут B):**
проект: **нативный HAL/FreeRTOS + hand-written Makefile (GUI-free, без CubeMX)** + официальный **`micro_ros_stm32cubemx_utils`** (ветка `jazzy`),
`libmicroros.a` собирается Docker-образом `microros/micro_ros_static_library_builder:jazzy`.

| Решение | Выбор |
|---|---|
| Интеграция micro-ROS | Маршрут B (нативный HAL + hand-written Makefile, **GUI-free**, + `micro_ros_stm32cubemx_utils`, ветка jazzy) |
| Сборка libmicroros | Docker `microros/micro_ros_static_library_builder:jazzy` (ABI-match по построению) |
| Модель выполнения | FreeRTOS / CMSIS-OS v2; rclc-executor в задаче со стеком ≥ 24 КБ |
| Транспорт | UART, compile-time переключатель **DMA** (железо/Renode) ↔ **IT/polling** (QEMU) |
| IMU/MAG | MPU6050 / MPU9250 через I2Cdevlib (QMI8658/QMC5883L вне области — не выбраны) |
| Разработка | **Emulation-first** (нет железа): Renode (основной) + host-юнит-тесты + QEMU (опц.) |
| Первая мишень | **NUCLEO-F446RE** (M4F, 128 КБ RAM / 512 КБ Flash; community-validated, лучшее покрытие в эмуляторах) |
| Стоимость | Все обязательные инструменты — бесплатны (см. §11) |

**Bottom line по «без железа»:** ~70 % работы делается и проверяется без платы (математика — host-тестами,
MCU-интеграция и транспорт — в Renode); «аналого-временнóй хвост» (реальная PWM-частота, физика энкодера,
bring-up MPU, framing@921600, джиттер) валидируется только на чипе позднее (§10).

---

## 1. Архитектура: 3 слоя как граница тестируемости

Главный принцип порта — **жёстко разделить портируемое и платформенное**, чтобы максимум кода проверялся
без железа.

```
┌──────────────────────────────────────────────────────────────┐
│ Слой 3: приложение micro-ROS + FreeRTOS                        │
│  - порт 4-state reconnect-машины в задачу (по образцу          │
│    sample_main.c), rclc executor, таймер 50 Гц, deadman 200 мс │
│  - проверяется: Renode / QEMU                                  │
├──────────────────────────────────────────────────────────────┤
│ Слой 2: тонкий HAL-слой периферии (НОВЫЙ нативный код)         │
│  - encoder (TIM encoder mode), motor (TIM PWM + GPIO dir),      │
│    I2Cdev-бэкенд (HAL_I2C_Mem_*), micro-ROS UART transport,     │
│    таймбаза millis/micros, минимальные Serial/Wire-шимы         │
│  - проверяется: Renode (частично) → железо (полностью)         │
├──────────────────────────────────────────────────────────────┤
│ Слой 1: портируемое ядро (переиспользуется как есть)           │
│  - kinematics, pid, odometry(*) — только типы Arduino + таймбаза│
│  - проверяется: HOST-юнит-тесты на ПК (без эмулятора)          │
└──────────────────────────────────────────────────────────────┘
(*) odometry требует точечного рефактора — см. §5.6
```

Compile-time HW-абстракция репозитория (`USE_*`-макросы, `#define Motor/Encoder/IMU`, хуки
`BOARD_INIT/BOARD_INIT_LATE/BOARD_LOOP`) **сохраняется** и расширяется эмуляционными/Fake-таргетами
(scripted-encoder, FakeIMU) — в духе существующего `USE_FAKE_IMU`.

---

## 2. Маршрут интеграции micro-ROS (B) и почему не A

Общий жёсткий факт: библиотека, которую репозиторий использует сейчас (`micro_ros_platformio`),
**структурно только под `framework=arduino`** (в ней есть только каталог `platform_code/arduino/`).
Значит авто-кросс-компиляции `libmicroros` «как на ESP32/Pico» на нативном Cube **не будет**;
`libmicroros.a` собирается отдельно.

**Маршрут A — PlatformIO `framework=stm32cube` + вручную слинкованный `libmicroros.a`. ОТКЛОНЁН.**
Единственный публичный прецедент (Nucleo-F446RE) прошёл 64-бит-атомики, но **намертво встал на
`libmicroros.a uses VFP register arguments` — рассогласование hard-float ABI** между Docker-сборкой и
тулчейном PlatformIO. Воспроизводимой сборки нет.

**Маршрут B — нативный HAL/FreeRTOS + hand-written Makefile (GUI-free) + `micro_ros_stm32cubemx_utils`. ВЫБРАН.**
- Живая ветка `jazzy` (совпадает с моделью «дистрибутив = ветка» этого репо).
- **ABI совпадает по построению:** Docker-сборщик берёт реальные CFLAGS приложения через
  `make print_cflags` и прокидывает их в colcon → нет VFP-стены Маршрута A.
- Стабильный контракт транспорта: `rmw_uros_set_custom_transport(true, &huartX, open, close, write, read)`.
- `colcon.meta` уже задаёт нужные STM32-ключи: `RMW_UXRCE_MAX_NODES=1`, `MAX_PUBLISHERS=10`,
  `MAX_SUBSCRIPTIONS=5`, `RMW_UXRCE_TRANSPORT=custom`, `RCUTILS_NO_64_ATOMIC=ON` — но он покрывает
  **только `rcutils`**; `rcl` (time/timer/client) использует 64-бит атомики безусловно, поэтому на
  Cortex-M дополнительно нужен **свой `__atomic_*_8`-шим** (эмпирически подтверждено — см. Plan 2 Task 5a).
- Цена: предполагается **FreeRTOS** (все примеры и транспортные `.c` тянут `cmsis_os.h`) и **отдельная CI**
  с Docker-сборкой libmicroros (PlatformIO-шный `parse_platformio.py` её не видит).

---

## 3. Структура репозитория

- Новый каталог **`firmware_stm32/`** (hand-written HAL/FreeRTOS проект + Makefile, **без CubeMX/GUI**) **рядом** с `firmware/`, не вместо.
- Портируемые либы (`firmware/lib/{kinematics,pid,odometry}`) подключаются **по include-ссылке**, не
  форкаются — single source of truth между Arduino- и Cube-сборками.
- `micro_ros_stm32cubemx_utils` вендорится (submodule или копия с pin'ом коммита).
- `libmicroros.a` + `microros_include/` коммитятся как **артефакт-локфайл**; Docker-образ пиннится по
  digest; `extra_packages`/`colcon.meta` фиксируются для воспроизводимости.
- **`I2Cdevlib-Core` форкается/вендорится** (его файлы тянутся через lib_deps и редактировать in-repo
  нельзя) — в форк добавляется HAL-бэкенд (§5.3). Классы сенсоров (MPU6050/MPU9250) при этом
  **не меняются**.

---

## 4. Объём работ по компонентам

| Компонент | Файл(ы) | Статус | Что делаем |
|---|---|---|---|
| Кинематика | `firmware/lib/kinematics/*` | ✅ as-is | только таймбаза-шим |
| PID | `firmware/lib/pid/*` | ✅ as-is | только таймбаза-шим |
| Одометрия | `firmware/lib/odometry/*` | ✏️ рефактор | вынести чистый интегратор без `nav_msgs`-зависимости (§5.6) |
| Классы IMU (MPU6050/9250) | `firmware/lib/imu/default_imu.h` | ✅ as-is | работают поверх нового I2Cdev-бэкенда |
| I2Cdev примитивы | форк `I2Cdevlib-Core` | 🔁 rewrite | ~10 методов → `HAL_I2C_Mem_Read/Write` (§5.3) |
| Энкодер | `firmware/lib/encoder/encoder.h` | 🔁 rewrite | TIM encoder mode (§5.1) |
| Мотор/PWM | `firmware/lib/motor/default_motor.h` | 🔁 rewrite | TIM PWM + GPIO-направление (§5.2) |
| Транспорт | `firmware/src/firmware.ino:183` | 🔁 rewrite | custom transport DMA/IT (§5.4) |
| Платформенный клей | `firmware/src/firmware.ino` (setup/loop, Serial, millis…) | 🪡 shim | → HAL/FreeRTOS-задача (§5.5) |
| Board config | `firmware_stm32/.../f446re_config.h` | 🆕 new | pin-map → конкретные `TIMx_CHy` + AF |
| Сборка | `firmware_stm32/Makefile` (hand-written, GUI-free) | 🆕 new | Makefile-flow micro_ros_stm32cubemx_utils |
| CI | `.github/workflows/stm32cube.yml` | 🆕 new | отдельный workflow (§7) |

**Вне области (не выбрано пользователем):** `QMI8658` и `default_mag.h`/QMC5883L (прямой Wire) — отдельные
Wire→HAL переписывания; не делаем, пока IMU = MPU6050/9250.

---

## 5. Конкретика нативных переписываний

### 5.1 Энкодер — TIM encoder mode
- `TIM_ENCODERMODE_TI12` (x4-декодирование), фазы A/B = **CH1/CH2 одного TIMx** (нельзя разнести по
  двум таймерам). Чтение — `TIMx->CNT` напрямую; направление — флаг `DIR`.
- 32-бит таймеры (**TIM2/TIM5**) предпочтительны там, где важна абсолютная позиция; 16-бит — корректны
  только через **дельта-расчёт** между опросами (на 50 Гц wrap не успевает), что и делает текущая
  формула `getRPM()`. ⚠️ Текущий код хранит сырые монотонные тики — при наивном порте на 16-бит CNT
  нужно обрабатывать wrap явно (вычитание по модулю).
- API сохраняется: `Encoder(int p1,int p2,int cpr,bool invert)`, `float getRPM()`, `int32_t read()`,
  `void write(int32_t)`.

### 5.2 Мотор/PWM — TIM PWM mode
- `HAL_TIM_PWM_Start(&htimX, TIM_CHANNEL_y)` + `__HAL_TIM_SET_COMPARE(...)`. 20 кГц @ 10-бит →
  `ARR = 1023` + подобранный `PSC` от тактовой таймера.
- ⚠️ Частота PWM — **per-timer (общий ARR), не per-pin**: для BTS7960 оба PWM-пина должны быть двумя
  каналами одного таймера. Направление — GPIO через `HAL_GPIO_WritePin`.
- ESC-путь (`Servo`) → канал TIM на 50 Гц с импульсом 1000–2000 мкс.

### 5.3 I2Cdev-бэкенд
- Портировать **только ~10 статических примитивов** `I2Cdev::readByte/writeByte/readBytes/readBit(s)/
  writeBit(s)` поверх `HAL_I2C_Mem_Read/Write`. Все 8 классов устройств (MPU6050/9250, ADXL345, ITG3200,
  HMC5883L, AK89xx) дёргают только этот статический интерфейс и остаются **байт-в-байт** прежними.
- `I2Cdev.cpp` использует `millis()` для таймаутов чтения → нужен таймбаза-шим (§5.5).
- ⚠️ Делается в **форке** I2Cdevlib-Core (см. §3).

### 5.4 Транспорт micro-ROS — compile-time переключатель
- Заменить `set_microros_serial_transports(Serial)` на
  `rmw_uros_set_custom_transport(true, &huartX, cubemx_transport_open/close/write/read)`.
- Готовые файлы из `micro_ros_stm32cubemx_utils`: `dma_transport.c` (UART/DMA, рекоменд.),
  `it_transport.c` (UART/IT), `usb_cdc_transport.c`, `udp_transport.c`.
- **Ключевой выбор для эмуляции:** DMA — на железе и в Renode; IT/polling — в QEMU-сборке (DMA в QEMU не
  смоделирован). Одна и та же логика во всех трёх местах через build-флаг.
- ⚠️ **USB-CDC не выбираем** — тянет проприетарный `STM32_USB_Device_Library` (SLA0044); UART остаётся
  полностью BSD-3/Apache-2.0 (см. §11).

### 5.5 Платформенный клей (шим)
- `millis()` → `HAL_GetTick()`; `micros()` → DWT cycle counter; `delay()` → `HAL_Delay()`;
  `pinMode/digitalWrite/digitalRead` → `HAL_GPIO_*`; `Serial` (логи) → `HAL_UART` или ITM/semihosting.
- `setup()/loop()` → инициализация в `main()` + rclc-executor в FreeRTOS-задаче (стек ≥ 24 КБ).

### 5.6 Рефактор odometry (для host-тестируемости)
- Вынести чистый интегратор позиции
  (`x += (vx·cosθ − vy·sinθ)·dt; y += (vx·sinθ + vy·cosθ)·dt; θ += wz·dt`)
  в функцию **без** зависимости от `nav_msgs__msg__Odometry` / `micro_ros_string_utilities`, чтобы
  host-тест не линковал rosidl-рантайм. Публикационная обёртка остаётся в Слое 3.

---

## 6. Эмуляция без железа (3 яруса) + матрица «что-что-доказывает»

- **Ярус A — host-юнит-тесты (быстро, каждый коммит):** `kinematics`, `pid`, вынесенный интегратор
  `odometry` под PlatformIO-`native`/GoogleTest с `Arduino.h`-шимом (`constrain/fabs/PI`). Самый быстрый
  и надёжный гейт математики; эмулятор не нужен.
- **Ярус B — Renode (основной интеграционный гейт):**
  - `.repl` для F446RE моделирует всю boot-периферию (RCC/FLASH/NVIC/SysTick/USART/DMA/TIM/I2C) — иначе
    `HAL_Init` зависнет/HardFault (верхний эмуляционный риск).
  - Агент: либо in-sim hub (как в `antmicro/renode-microros-demo`), либо
    `CreateUartPtyTerminal` → host `micro_ros_agent serial --dev /tmp/uart` (этот путь — спайк: публичного
    end-to-end примера host-agent-over-Renode-pty нет).
  - Headless-asserts: Robot Framework `Wait For Line On Uart  <regex>  treatAsRegex=true`; через
    `antmicro/renode-test-action`, с pin'ом `renode-revision`.
  - Энкодер — скриптинг фронтов CH1/CH2; IMU — кастомный I2C-slave (готовой модели MPU в Renode нет)
    **или** FakeIMU.
- **Ярус C — QEMU (опционально):** `qemu-system-arm -M netduinoplus2 -semihosting` boot+session-smoke с
  FakeIMU и IT-транспортом. ⚠️ Машина `netduinoplus2` — **F405-класс, не F446RE точь-в-точь**: это smoke
  на уровне ядра Cortex-M4, а не board-accurate (отдельная QEMU-сборка под доступную машину). ⚠️
  `docker/setup-qemu-action` **не** ставит `qemu-system-arm` (только user-mode binfmt) — нужен явный
  `apt-get install qemu-system-arm`.

**Матрица «что-что-доказывает»:**

| Кусок контура | Host-тест | QEMU | Renode | Только железо |
|---|---|---|---|---|
| `kinematics.getRPM` микс/сатурация | ✅ primary | — | — | — |
| `PID.compute` сатурация/anti-windup | ✅ primary | — | — | — |
| `odometry` интегратор | ✅ (после рефактора) | — | — | — |
| 50 Гц rcl-таймер (логика) | — | ✅ | ✅ | реальный джиттер |
| reconnect-машина + сессия + round-trip | — | ✅ (IT) | ✅ (incl. DMA) | framing@921600 |
| FreeRTOS стек + `rclc_support_init` | — | ✅ (medium) | ✅ (medium) | worst-case high-water-mark |
| Энкодер: счёт/направление/масштаб | — | ❌ | ✅ (скрипт фронтов) | CPR, знак на max RPM, 16-бит wrap, динамика |
| IMU I2C-плумбинг + публикация | — | ❌ | ✅ (нужен I2C-slave) | bring-up MPU, `calibrateGyro` |
| PWM duty/канал + GPIO-направление | — | ❌ | ⚠️ регистры, не волна | 20 кГц/10-бит на осц., знак |
| UART/DMA надёжность (RX-to-idle) | — | ❌ | ✅ функц. | framing под нагрузкой |

⚠️ Оба эмулятора **не точны по таймингу/бодрейту** (Renode UART — байт-очередь; QEMU USART не блюдёт
битовый тайминг): баг-рейт-mismatch, framing-под-нагрузкой и interrupt-latency **не воспроизводятся
ни там, ни там**.

---

## 7. CI

Отдельный `.github/workflows/stm32cube.yml` (не трогает `parse_platformio.py`, существующая матрица не
ломается):
1. `docker run microros/micro_ros_static_library_builder:jazzy` → `libmicroros.a` (неинтерактивно: либо
   pre-seed stdin, либо IDE/Make-вариант; upstream-CI там нет — пишем сами).
2. `make` сборка `firmware_stm32` → `.elf`.
3. Renode robot-тесты (`antmicro/renode-test-action`, pin revision) → основной гейт.
4. (опц.) QEMU boot/session-smoke.
- **Пиннинг:** `antmicro/renode:1.16.1`, `microros/micro-ros-agent:jazzy` (digest re-resolve),
  QEMU из apt Ubuntu 24.04, Docker-образ сборщика по sha256.
- **Docker Hub лимиты:** `docker/login-action` или зеркало GHCR (троттлинг, не цена — §11).

---

## 8. Фазы (milestones)

- **Ф0 — скелет:** проект F446RE (hand-written HAL/FreeRTOS, GUI-free) компилируется, `libmicroros.a` линкуется (**ABI-smoke** —
  **ABI-smoke = ДВА гейта**: VFP **и** 64-бит атомики/POSIX. Эмпирически: VFP снят (все 2014 членов hard-float); атомики требуют `__atomic_*_8`-шима + `usleep` (Plan 2 Task 5a)).
- **Ф1 — host-тесты:** `kinematics/pid/odometry`-интегратор зелёные на ПК.
- **Ф2 — Renode boot:** FreeRTOS стартует, `rclc_support_init` без HardFault (стек 24–32 КБ).
- **Ф3 — Renode транспорт:** micro-ROS-сессия к агенту, round-trip.
- **Ф4 — Renode энкодер/PWM:** логика энкодера (скриптинг фронтов) + регистровые asserts PWM.
- **Ф5 — IMU:** FakeIMU → кастомный Renode I2C-slave → `/imu/data_raw` публикуется.
- **Ф6 — Renode полный контур:** `cmd_vel`→kinematics→PID→motor, `/odom`+`/imu` на 50 Гц.
- **Ф7 — железо (когда появится плата):** 6 residual-пунктов (§10); затем дублирование env под другие
  STM32-семейства (универсальность).

---

## 9. Риски (ранжированы) и митигиции

1. **Renode `.repl` авторинг** для F446RE — верхний эмуляционный риск. ↦ взять готовый прецедент `.repl`
   F446RE (prdktntwcklr/renode-example), наращивать периферию инкрементально.
2. **Float-ABI/FPU** — снят Маршрутом B (Docker `make print_cflags`); проверяется ABI-smoke в Ф0. Эмпирически: VFP снят (2014/2014 hard-float). **НО** на Cortex-M остаётся реальный блокер: `rcl` тянет `__atomic_*_8`, которых нет в arm-none-eabi (baremetal); `RCUTILS_NO_64_ATOMIC=ON` покрывает только `rcutils`. ↦ PRIMASK `__atomic_*_8`-шим + `usleep` (Plan 2 Task 5a; micro_ros_stm32cubemx_utils#112).
3. **FreeRTOS стек / HardFault на `rclc_support_init`** (sample-стек 12 КБ мал). ↦ 24–32 КБ, high-water-mark
   + статический анализ; аккуратно с custom allocator (может конфликтовать со стеком).
4. **Нет модели MPU в Renode** — кастомный I2C-slave (WHO_AM_I + регистры) или FakeIMU до железа.
5. **Эмуляторы не точны по таймингу/бодрейту** — framing@921600, джиттер 50 Гц, реальная PWM-частота →
   только железо (Ф7).
6. **Универсальность за пределами F4** (G4/H7/L4) — может требовать авторинга Renode `.repl`/моделей;
   H7 ещё и MPU-кэш для UDP. ↦ «универсально» = цель после зелёного F4-пути.
7. **Воспроизводимость libmicroros** — плавающие branch-HEAD/Docker-теги. ↦ pin образа по digest, коммит
   `.a` как локфайла, фиксация `extra_packages`/`colcon.meta`.

---

## 10. Residual — валидируется только на железе (Ф7)

1. **PWM** — реальная частота/разрешение (20 кГц @ 10-бит) на осциллографе.
2. **Энкодер** — реальные `COUNTS_PER_REV`, знак направления, поведение на `MOTOR_MAX_RPM`, 16-бит wrap;
   замкнутая динамика PID/одометрии (в эмуляции нет физической модели).
3. **IMU** — bring-up MPU6050/9250 (WHO_AM_I, у MPU9250 — AK8963 passthrough), `calibrateGyro` против
   реального шума. ⚠️ `imu.init()` фатален в `setup()` — это реальный гейт.
4. **UART framing @ 921600** под DMA-нагрузкой (эмуляторы не bod/timing-accurate).
5. **Джиттер control-loop** — 50 Гц таймер и 200 мс deadman под реальной шинной/DMA-нагрузкой.
6. **RAM-headroom** worst-case на конкретном чипе (чистый прогон в эмуляции снижает, но не снимает риск
   F4-HardFault).

---

## 11. Стоимость и лицензии — всё бесплатно (проверено)

**Итог:** все обязательные инструменты бесплатны при трёх условиях, которые план и так выполняет:
**публичный репозиторий**, **CI на Linux + Docker Engine** (не Desktop), **UART-транспорт** (не USB-CDC).

| Инструмент | Лицензия | Free? |
|---|---|---|
| STM32Cube HAL/LL + BSP | BSD-3-Clause | ✅ |
| CMSIS Core + Device | Apache-2.0 | ✅ |
| arm-none-eabi-gcc / GNU Make | GPL (+GCC-exception) | ✅ |
| STM32CubeMX/IDE/Programmer | SLA0048 (проприет. freeware) | ⛔ **НЕ используется** (порт GUI-free) |
| micro_ros_stm32cubemx_utils, rcl/rclc/rmw, Micro XRCE-DDS, ROS 2 Jazzy | Apache-2.0 | ✅ |
| Renode / renode-test-action / Robot Framework | MIT / Apache-2.0 | ✅ |
| QEMU | GPL-2.0 | ✅ |
| GoogleTest / PlatformIO Core / socat | BSD / Apache-2.0 / GPL-2.0 | ✅ |
| FreeRTOS kernel / CMSIS-RTOS v2 | MIT / Apache-2.0 | ✅ |
| jrowberg I2Cdevlib | MIT | ✅ |
| Docker Engine (Linux) | Apache-2.0 | ✅ |
| Docker Desktop | DSSA (проприет.) | ⚠️ платно 250+ сотр./≥$10M — **не используем** |
| GitHub Actions | SaaS | ⚠️ публичный репо без лимита; приват = 2000 мин/мес |
| STM32_USB_Device_Library (если USB-CDC) | SLA0044 (проприет.) | ⚠️ **избегаем — берём UART** |

**Watch-outs (как остаться в бесплатной полосе):**
1. В CI — **Docker Engine** на Linux-runner'ах, не Docker Desktop.
2. Репозиторий **публичный** → Actions бесплатны без лимита (иначе 2000 мин/мес или self-hosted).
3. Docker Hub pull-лимиты — `docker/login-action`/зеркало GHCR (троттлинг, не цена).
4. **ST GUI-тулзы (CubeMX/IDE) не используются вообще** — проект hand-written из **BSD-3/Apache-2.0** исходников
   HAL/CMSIS + FreeRTOS (MIT) + gcc; никакого CubeMX/`.ioc`/GUI. Это и даёт полную CI-воспроизводимость.
5. **UART-транспорт** обходит проприетарный SLA0044 (`STM32_USB_Device_Library`) — лишний довод за UART.

Замена инструментов **не требуется** — нужны лишь эти конфигурационные выборы (все уже в плане).

---

## 12. Открытые вопросы / что подтвердить по ходу

- Точный pin-map F446RE: 4 пары `TIMx_CH1/CH2` под энкодеры + PWM-каналы моторов + I2C(IMU) + UART
  (micro-ROS), без коллизий по alternate-function. Сверить по datasheet.
- Привод (2 vs 4 колеса) — задаёт бюджет таймеров; F446RE имеет TIM1/2/3/4/5/8 — достаточно для 4 колёс,
  но pin-map требует валидации.
- Спайк host-agent-over-Renode-pty (нет публичного end-to-end примера) — или сразу in-sim hub.

---

## Источники (проверено разведкой)

- micro_ros_stm32cubemx_utils (ветки humble/jazzy/…; Docker-сборщик; транспорты; colcon.meta) —
  https://github.com/micro-ROS/micro_ros_stm32cubemx_utils
- jazzy colcon.meta —
  https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/blob/jazzy/microros_static_library/library_generation/colcon.meta
- jazzy sample_main.c (контракт custom transport) —
  https://raw.githubusercontent.com/micro-ROS/micro_ros_stm32cubemx_utils/jazzy/sample_main.c
- dma_transport.c / it_transport.c —
  https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/tree/humble/extra_sources/microros_transports
- micro_ros_platformio (только framework=arduino) — https://github.com/micro-ROS/micro_ros_platformio
- PlatformIO Community: VFP/float-ABI стена Маршрута A —
  https://community.platformio.org/t/stm32-framework-with-micro-ros-stm32cubemx-utils-undefined-references-to-sync-synchronize/42180
- micro_ros_stm32cubemx_utils #110 (стек 24–32 КБ, HardFault на init) —
  https://github.com/micro-ROS/micro_ros_stm32cubemx_utils/issues/110
- Renode encoder modes (STM32_Timer.cs) —
  https://github.com/renode/renode-infrastructure/blob/master/src/Emulator/Peripherals/Peripherals/Timers/STM32_Timer.cs
- Renode I2C модели (STM32F1_I2C.cs / STM32F7_I2C.cs) —
  https://github.com/renode/renode-infrastructure/tree/master/src/Emulator/Peripherals/Peripherals/I2C
- antmicro/renode-microros-demo + блог —
  https://github.com/antmicro/renode-microros-demo ,
  https://renode.io/news/fully-deterministic-linux-zephyr-micro-ros-testing-in-renode/
- renode-test-action / pinning —
  https://renode.io/news/renode-github-action-for-automated-testing-in-simulation/
- QEMU STM32 (моделируемое/нет) — https://www.qemu.org/docs/master/system/arm/stm32.html
- setup-qemu-action не ставит qemu-system-arm — https://github.com/docker/setup-qemu-action
- STM32 HAL = BSD-3-Clause; CMSIS = Apache-2.0; ST SLA0048 (freeware); SLA0044 (USB middleware);
  FreeRTOS = MIT; Renode = MIT; Docker Engine = Apache-2.0 vs Docker Desktop DSSA.
- ST AN4013 (timers/encoder mode) —
  https://www.st.com/resource/en/application_note/an4013-introduction-to-timers-for-stm32-mcus-stmicroelectronics.pdf

---

## Затронутые файлы (абсолютные пути)

**Новые:**
- `/home/claude/Projects/linorobot2_hardware/firmware_stm32/` — hand-written HAL/FreeRTOS проект + `Makefile` (GUI-free, без `.ioc`)
- `/home/claude/Projects/linorobot2_hardware/firmware_stm32/.../f446re_config.h` — board config (pin-map)
- `/home/claude/Projects/linorobot2_hardware/.github/workflows/stm32cube.yml` — отдельный CI
- форк/вендор `I2Cdevlib-Core` с HAL-бэкендом
- host-тесты (PlatformIO-`native`/GoogleTest) для kinematics/pid/odometry-интегратора
- Renode `.repl` + Robot-тесты

**Изменяются:**
- `firmware/lib/odometry/odometry.{h,cpp}` — вынос чистого интегратора (рефактор, обратносовместимо)

**Переиспользуются как есть:**
- `firmware/lib/kinematics/*`, `firmware/lib/pid/*`, `firmware/lib/imu/default_imu.h` (классы MPU)

**Не трогаем:** Arduino/ESP32/Pico/Teensy-ветки `encoder.h`/`default_motor.h`, существующий
`platformio.ini` и `parse_platformio.py`, `docs/STM32_PORTING_PLAN.md`.
