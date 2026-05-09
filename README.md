# :helicopter: MiniFly-FC

<div align="center">
<p>
<a href="LICENSE">
  <img src="https://img.shields.io/badge/License-MIT-blue.svg" alt="License: MIT">
</a>
<a href="https://en.wikipedia.org/wiki/C11_(C_standard_revision)">
  <img src="https://img.shields.io/badge/Language-C11-blue.svg" alt="Language: C11">
</a>
<a href="https://www.st.com/en/microcontrollers-microprocessors/stm32f411ce.html">
  <img src="https://img.shields.io/badge/MCU-STM32F411CEUx-orange.svg" alt="MCU: STM32F411">
</a>
<a href="https://www.freertos.org/">
  <img src="https://img.shields.io/badge/RTOS-FreeRTOS-green.svg" alt="RTOS: FreeRTOS">
</a>
<a href="https://cmake.org/">
  <img src="https://img.shields.io/badge/Build-CMake-yellow.svg" alt="Build: CMake">
</a>
</p>
</div>

> A modernized flight controller firmware for the **Alientek MiniFly** mini quadcopter, rewritten from legacy SPL + Keil to STM32CubeMX + HAL + CMake + FreeRTOS (CMSIS-RTOS v2).

---

## :dart: What Is This

A complete quadcopter flight controller running on the **STM32F411CEUx** (Cortex-M4F, 96 MHz), with cascaded PID control, sensor fusion, radio/USB telemetry, and a hot-pluggable expansion module system. Originally based on the [Alientek (正点原子) MiniFly](https://github.com/alientek-minifly) open-source project (2017), fully restructured for modern tooling and maintainability.

```
           M1 (Front-Left)        M2 (Front-Right)
              TIM4_CH2               TIM4_CH1
                 \                    /
                  \    +--------+    /
                   \   |  MPU   |   /
                    \  | 6500   |  /
                     \ |________| /
                      +----++----+
                      |  STM32  |
                      | F411CE  |
                      +----++----+
                     / |________| \
                    /  | BMP280 |  \
                   /   | SPL06  |   \
                  /    +--------+    \
                 /                    \
           M4 (Rear-Left)         M3 (Rear-Right)
              TIM2_CH1               TIM2_CH3
```

---

## :star2: Features

### :joystick: Flight Control

- :white_check_mark: Cascaded PID (angle + rate loops)
- :white_check_mark: Position & velocity PID with auto hover-thrust
- :white_check_mark: Mahony attitude estimator (quaternion)
- :white_check_mark: INAV-style position estimator
- :white_check_mark: Flip maneuver support
- :white_check_mark: Carefree mode (body-to-world frame)

### :thermometer: Sensors

- :white_check_mark: MPU6500 IMU (accel + gyro, 1 kHz)
- :white_check_mark: AK8963 magnetometer (100 Hz, via MPU bypass)
- :white_check_mark: BMP280 / SPL06 barometer (auto-detected)
- :white_check_mark: Gyro bias calibration (1024-sample, variance-based)
- :white_check_mark: 2nd-order low-pass filters (80 Hz gyro, 30 Hz accel)
- :white_check_mark: Board-frame coordinate remapping


### :satellite: Communication
- :white_check_mark: nRF24L01 radio (USART2, 1 Mbaud, DMA TX)
- :white_check_mark: USB CDC (virtual COM port)
- :white_check_mark: ATKP binary protocol (telemetry + commands)
- :white_check_mark: Dual-source RC input (radio + WiFi)
- :white_check_mark: Hardware flow control (EXTI on PA0)
- :white_check_mark: Commander with link-loss watchdog (auto-land)

### :electric_plug: Expansion Modules

- :white_check_mark: Hot-plug detection via ADC resistor dividers
- :white_check_mark: WS2812 12-LED RGB ring (TIM3 + DMA)
- :white_check_mark: PMW3901 optical flow sensor (SPI2)
- :white_check_mark: VL53L0X/VL53L1X laser rangefinder
- :white_check_mark: WiFi camera module
- :white_check_mark: Shared I2C3 bus for expansion

### :shield: Safety

- :white_check_mark: IWDG watchdog (auto-kick)
- :white_check_mark: Anomaly detection (crash/flip guard)
- :white_check_mark: Flight lock / unlock
- :white_check_mark: Auto-land on link loss (500 ms stabilize, 1000 ms shutdown)
- :white_check_mark: Emergency motor stop
- :white_check_mark: Low-battery LED warnings

### :wrench: System

- :white_check_mark: FreeRTOS (CMSIS-RTOS v2, 10 tasks)
- :white_check_mark: STM32CubeMX + HAL (modern driver layer)
- :white_check_mark: CMake + Ninja build system
- :white_check_mark: Flash-backed config persistence (bootloader-protected)
- :white_check_mark: BSP abstraction layer (clean hw/sw boundary)
- :white_check_mark: Layered architecture (App / Control / Services / Platform / BSP)

---

## :computer: Hardware at a Glance

### MCU

| Spec | Value |
|------|-------|
| **Chip** | STM32F411CEUx |
| **Core** | ARM Cortex-M4F (FPU) |
| **Clock** | 96 MHz (HSE 8 MHz, PLL x12) |
| **Flash** | 512 KB (8 sectors) |
| **SRAM** | 128 KB |
| **Package** | UFQFPN48 |

### Sensors

| Chip | Interface | Function |
|------|-----------|----------|
| MPU6500 | I2C1 (0x68/0x69) | 6-axis IMU (accel +/-16g, gyro +/-2000 dps) |
| AK8963 | I2C1 (0x0C, via MPU bypass) | 3-axis magnetometer |
| BMP280 | I2C1 (0x76) | Barometer (auto-detected) |
| SPL06 | I2C1 (0x76) | Barometer (fallback) |

### Communication

| Interface | Pins | Baud / Speed | Purpose |
|-----------|------|-------------|---------|
| USART2 | PA2 TX, PA3 RX | 1,000,000 | nRF24L01 radio link (DMA TX) |
| USB OTG FS | PA11 DM, PA12 DP | CDC | Ground station (virtual COM) |
| USART1 | PA15 TX, PB3 RX | 19,200 | WiFi module / debug |
| SPI2 | PB13 SCK, PB14 MISO, PB15 MOSI | 1.5 Mbit/s | Optical flow sensor |

### Motor Layout (X-Quad)

```
        FRONT
    M1 ------- M2
     \         /
      \       /
       +-----+
       | FC  |
       +-----+
      /       \
     /         \
    M4 ------- M3
        REAR
```

| Motor | Position | Pin | Timer |
|-------|----------|-----|-------|
| M1 | Front-Left | PB7 | TIM4_CH2 |
| M2 | Front-Right | PB6 | TIM4_CH1 |
| M3 | Rear-Right | PB10 | TIM2_CH3 |
| M4 | Rear-Left | PA5 | TIM2_CH1 |

### Status LEDs

| LED | Pin | Polarity | Function |
|-----|-----|----------|----------|
| BLUE_L | PB12 | Active High | System status |
| GREEN_L | PA6 | Active Low | RX indicator |
| RED_L | PA7 | Active Low | TX indicator |
| GREEN_R | PC13 | Active Low | Calibration status |
| RED_R | PC14 | Active Low | Low battery warning |

### Flash Memory Map

```
 0x08000000 +----------------------------+
            |       Bootloader           |
            |         (16 KB)            |  Sector 0
 0x08004000 +----------------------------+
            |     Config Parameters      |
            |         (16 KB)            |  Sector 1
 0x08008000 +----------------------------+
            |                            |
            |       Application          |
            |        (480 KB)            |  Sectors 2-7
            |                            |
 0x08080000 +----------------------------+
```

---

## :building_construction: Software Architecture

### Layer Diagram

```
+---------------------------------------------------------------+
|                        Application                            |
|   app_boot.c  |  app_tasks.c  |  app_hooks.c                  |
+---------------------------------------------------------------+
|     Control     |     Services     |     Communication        |
| stabilizer      | sensors (1kHz)   | radiolink (nRF24L01)     |
| attitude_pid    | config_service   | usblink (USB CDC)        |
| position_pid    | pm_service       | atkp (binary protocol)   |
| attitude_est.   | ledseq           | commander (RC input)     |
| position_est.   | console_service  |                          |
| power_control   |                  |                          |
| anomaly_detect  |                  |                          |
| flip            |                  |                          |
+---------------------------------------------------------------+
|                     Modules (Expansion)                       |
|   module_manager  |  ledring_module  |  optical_flow_module   |
|   wifi_module     |                  |                        |
+---------------------------------------------------------------+
|                       Platform                                |
|   filter.c  |  maths.c  |  platform_init  |  platform_irq     |
|   platform_fault  |  timebase  |  axis.h                      |
+---------------------------------------------------------------+
|                   BSP (Board Support)                         |
|   bsp_sensors  |  bsp_motors  |  bsp_led  |  bsp_ws2812       |
|   bsp_flash    |  bsp_watchdog |  bsp_module                  |
+---------------------------------------------------------------+
|         STM32 HAL / CMSIS / FreeRTOS / USB Device Library     |
+---------------------------------------------------------------+
```

### FreeRTOS Task Map

| Task | Stack | Priority | Rate | Description |
|------|-------|----------|------|-------------|
| `stabilizer` | 1024 B | High | 500 Hz | Main flight control loop |
| `sensors` | 1024 B | AboveNormal | 1 kHz | IMU / barometer sampling |
| `radiolink` | 512 B | High | IRQ-driven | nRF24L01 radio receive |
| `atkpRx` | 768 B | High | Event | ATKP protocol RX decode |
| `atkpTx` | 512 B | Normal | Event | ATKP protocol TX encode |
| `usblinkRx` | 512 B | AboveNormal | Event | USB CDC receive |
| `usblinkTx` | 512 B | Normal | Event | USB CDC transmit |
| `pmSvc` | 512 B | BelowNormal | Periodic | Power management |
| `configSvc` | 512 B | Low | Deferred | Flash config persistence |
| `moduleMgr` | 512 B | Low | Periodic | Expansion module hot-plug |

### Flight Control Pipeline

```
 +----------+     +-------------+     +-------------+     +----------+
 |  Sensor   |---->|  Attitude   |---->|  Position   |---->| Commander|
 |  Acquire  |     |  Estimator  |     |  Estimator  |     | Setpoint |
 |  (500Hz)  |     |  (250 Hz)   |     |  (250 Hz)   |     | (100 Hz) |
 +----------+     +-------------+     +-------------+     +----------+
       |                                      |                 |
       v                                      v                 v
 +----------+                          +-------------+    +----------+
 | Anomaly  |                          |  Position   |    |   Flip   |
 | Detect   |                          |  Adjust     |    |  Check   |
 | (500 Hz) |                          |  (250 Hz)   |    | (500 Hz) |
 +----------+                          +-------------+    +----------+
       |                                      |                 |
       +------------------+-------------------+-----------------+
                          |
                          v
                  +---------------+
                  |   PID Control |
                  |  (attitude +  |
                  |   position)   |
                  +-------+-------+
                          |
                          v
                  +---------------+
                  |  Motor Output |
                  |    (500 Hz)   |
                  +---------------+
```

---

## :arrows_counterclockwise: Legacy vs Modern

| Aspect | Legacy (Alientek Original) | Modern (minifly-fc) |
|--------|---------------------------|---------------------|
| **Driver Library** | SPL (register-level) | STM32CubeMX + HAL |
| **Build System** | Keil uVision (.uvprojx) | CMake + Ninja + GCC |
| **RTOS** | FreeRTOS v9.0 (Keil port) | FreeRTOS (CMSIS-RTOS v2) |
| **Directory Layout** | Flat (FLIGHT/, HARDWARE/, COMMUNICATE/) | Layered (App / Control / Services / Platform / BSP) |
| **Naming** | Mixed (camelCase, inconsistent) | Prefixed (bsp_\*, platform_\*, \*_service) |
| **BSP** | Mixed into HARDWARE/ | Clean Drivers/BSP/ abstraction |
| **USB Stack** | Old ST USB library | CubeMX-managed USB Device Library |
| **Config Storage** | SPL flash (no sector protection) | HAL flash with bootloader protection |
| **Comments** | GB2312 Chinese, @author tags | Doxygen-clean, SPDX headers |
| **Platform Init** | Monolithic systemInit() | Decomposed (irq, timebase, fault) |

---

## :open_file_folder: Project Structure

```
minifly-fc/
├── Core/                          # Application source
│   ├── Inc/                       #   Headers
│   │   ├── app/                   #     Boot, task table, hooks
│   │   ├── comm/                  #     ATKP, radiolink, usblink, commander
│   │   ├── control/               #     Stabilizer, PID, estimators, power
│   │   ├── modules/               #     Expansion module drivers
│   │   ├── platform/              #     Filter, maths, fault, init, irq
│   │   └── services/              #     Sensors, config, PM, console, LED
│   └── Src/                       #   Implementations (mirrors Inc/)
├── Drivers/
│   ├── BSP/                       # Board Support Package
│   │   ├── Inc/                   #   bsp_sensors, bsp_motors, bsp_led, ...
│   │   └── Src/                   #   Implementations
│   ├── CMSIS/                     # ARM CMSIS (core, DSP, RTOS)
│   └── STM32F4xx_HAL_Driver/      # ST HAL/LL drivers
├── Middlewares/
│   ├── ST/STM32_USB_Device_Library/  # USB CDC class
│   └── Third_Party/FreeRTOS/         # FreeRTOS kernel
├── USB_DEVICE/                    # USB CDC application layer
├── Legacy/                        # Original Alientek firmware (reference only)
├── Docs/                          # Architecture docs, hardware analysis
├── Test/                          # BSP API compile tests
├── cmake/                         # Toolchain files + CubeMX sub-build
├── minifly-fc.ioc                 # STM32CubeMX project
├── STM32F411XX_FLASH.ld           # Linker script (app at 0x08008000)
├── CMakeLists.txt                 # Top-level build
└── CMakePresets.json              # Debug / Release presets
```

---

## :hammer_and_wrench: Build

### Prerequisites

- **arm-none-eabi-gcc** (ARM GNU Toolchain)
- **CMake** >= 3.22
- **Ninja** build system
- **STM32CubeMX** (only if regenerating peripheral init code)

### Clone & Build

```bash
git clone https://github.com/bfmhno3/minifly-fc.git
cd minifly-fc

# Configure (Debug preset)
cmake --preset Debug

# Build
cmake --build build/Debug

# Output: build/Debug/minifly-fc.elf
```

### Flash

Using **STM32CubeProgrammer** (SWD):

```bash
STM32_Programmer_CLI -c port=SWD -w build/Debug/minifly-fc.elf -v -rst
```

Or using **OpenOCD**:

```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program build/Debug/minifly-fc.elf verify reset exit"
```

> **Note**: The application starts at `0x08008000` (32 KB offset). The bootloader occupies the first 16 KB (Sector 0). You must flash the Alientek bootloader or other bootloader separately if starting from a blank chip.

---

## :electric_plug: Expansion Module System

Modules are detected at boot via ADC voltage dividers on PB1 (ADC1_IN9):

```
    Module Connector
    +------------------+
    |  PB1 (ADC)       |-----> Voltage Divider
    |  I2C3 (PA8/PB4)  |-----> Data Bus
    |  SPI2 (PB13-15)  |-----> Data Bus
    |  PB0 (Power EN)  |-----> Module Power Rail
    +------------------+
```

| Module | ADC Value | Interface | Description |
|--------|-----------|-----------|-------------|
| LED Ring | ~2048 | TIM3_CH1 (PB4) + DMA | WS2812 12-LED RGB ring |
| WiFi Camera | ~4095 | USART1 (PB3) | WiFi video module |
| Optical Flow | ~2815 | SPI2 (PB13-15) | PMW3901 motion sensor |
| Reserved | ~1280 | -- | Future expansion |
| None | ~0 | -- | No module connected |

---

## :books: Documentation

| Document | Description |
|----------|-------------|
| [Docs/hardware.md](Docs/hardware.md) | Complete hardware pinout, peripheral config, DMA/NVIC matrix |
| [Docs/software_architecture_plan.md](Docs/software_architecture_plan.md) | Full rewrite architecture, module specs, implementation phases |
| [Docs/board_memory_analysis.md](Docs/board_memory_analysis.md) | Flash layout, bootloader analysis, config persistence |
| [Docs/comment_style.md](Docs/comment_style.md) | Doxygen commenting standards |

---

## :handshake: Credits

- **Original firmware**: [Alientek (正点原子)](http://www.alientek.com/) MiniFly V1.0-V1.3 (2017-2018)
- **Modernization rewrite**: [bfmhno3](https://github.com/bfmhno3)
- **Flight algorithms**: Partially ported from [iNAV](https://github.com/iNavFlight/inav) (sensor fusion)

---

## :page_facing_up: License

This project is licensed under the [MIT License](LICENSE).

The original Alientek firmware is Copyright (c) Guangzhou Alientek Electronic Technology Co., Ltd. The `Legacy/` directory is preserved for reference only and is not part of the active build.
