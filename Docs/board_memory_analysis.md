# board_memory 模块分析：Flash 布局、持久化参数与 Bootloader

本文档整理了对 Legacy 代码中 Flash 存储架构的分析，回答三个核心问题：为什么要设计 `board_memory` 模块、为什么需要持久化参数、以及 Bootloader 的来源。

---

## 1. 背景：为什么要设计 board_memory 模块

### 1.1 问题

Legacy 代码中，Flash 布局相关的常量分散在多个文件中，缺乏单一真源：

| 文件 | 定义 |
|------|------|
| `Legacy/CONFIG/interface/config.h` | `BOOTLOADER_SIZE`, `CONFIG_PARAM_SIZE`, `CONFIG_PARAM_ADDR`, `FIRMWARE_START_ADDR` |
| `Drivers/BSP/Src/bsp_flash.c` | `BSP_FLASH_BOOT_RESERVED_SIZE_BYTES`, `BSP_FLASH_CONFIG_RESERVED_SIZE_BYTES` |
| `Core/Src/services/config_service.c` | `CONFIG_PARAM_ADDR = 0x08004000UL`（硬编码，与 config.h 重复） |
| `STM32F411XX_FLASH.ld` | `FLASH: ORIGIN = 0x8000000, LENGTH = 512K`（未预留 boot/config 区域） |

同一个物理事实（"bootloader 占 16KB，config 占 16KB，app 从 0x08008000 开始"）被多处声明。一旦修改，需要同步 4 个文件，否则 linker 脚本与运行时代码产生偏移——这是架构文档中标记为最高优先级的风险。

### 1.2 解决方案

`board_memory.h` 是该架构的重构产物，定位为**板级存储常量的单一真源**。所有与硬件存储相关的定义收敛到一个文件：

```c
// board_memory.h —— 板级存储常量单一真源

#define BOARD_FLASH_BASE         0x08000000UL
#define BOARD_FLASH_SIZE         0x00080000UL   // 512KB (STM32F411CE)

#define BOARD_FLASH_BOOT_SIZE    0x00004000UL   //  16KB, Sector 0
#define BOARD_FLASH_CONFIG_SIZE  0x00004000UL   //  16KB, Sector 1
#define BOARD_FLASH_APP_START    (BOARD_FLASH_BASE + BOARD_FLASH_BOOT_SIZE + BOARD_FLASH_CONFIG_SIZE)
                                                // 0x08008000, Sectors 2-7

#define BOARD_SYSCLK_HZ          96000000UL
#define BOARD_MOTOR_COUNT        4
#define BOARD_HAS_USB_LINK       1
#define BOARD_HAS_OPTICAL_FLOW   1
```

Linker 脚本、BSP flash 驱动、config service 均引用此文件，保证一致性。

---

## 2. Flash 布局分区

### 2.1 分区方案

```
0x08000000 +-----------------------------+
           |  Bootloader    16KB         |  Sector 0 (16KB)
0x08004000 +-----------------------------+
           |  Config Params 16KB         |  Sector 1 (16KB)
0x08008000 +-----------------------------+
           |                             |  Sector 2 (16KB)
           |                             |  Sector 3 (16KB)
           |  Application                |  Sector 4 (64KB)
           |  Firmware                   |  Sector 5 (128KB)
           |                             |  Sector 6 (128KB)
           |                             |  Sector 7 (128KB)
0x08080000 +-----------------------------+
```

### 2.2 为什么 Config 区是 16KB

不是因为实际存储需要 16KB。实际 config 结构体只有约 200 字节。

原因是 **STM32F411 Flash 硬件约束**：Sector 0-3 每个固定 16KB，这是最小的擦除单元。Flash 写入前必须先擦除整个 Sector。为了擦除 config 区时不破坏 bootloader，必须把 bootloader 和 config 放在不同的 Sector。

所以：

- Sector 0 → Bootloader（16KB）
- Sector 1 → Config（16KB，最小擦除单元）
- Sector 2-7 → Application（480KB）

"16KB 给 config"不是需求决定的，是硬件决定的。

### 2.3 STM32F411 Flash Sector 布局（ST 定义，不可改）

| Sector | 地址范围 | 大小 |
|--------|---------|------|
| 0 | 0x08000000 - 0x08003FFF | 16 KB |
| 1 | 0x08004000 - 0x08007FFF | 16 KB |
| 2 | 0x08008000 - 0x0800BFFF | 16 KB |
| 3 | 0x0800C000 - 0x0800FFFF | 16 KB |
| 4 | 0x08010000 - 0x0801FFFF | 64 KB |
| 5 | 0x08020000 - 0x0803FFFF | 128 KB |
| 6 | 0x08040000 - 0x0805FFFF | 128 KB |
| 7 | 0x08060000 - 0x0807FFFF | 128 KB |

### 2.4 分区是谁定义的

**ALIENTEK（正点原子）的旧代码开发者定义的，不是 STM32 官方规范。**

ST 官方只定义了 Flash Sector 的物理大小，不规定用户如何使用。ALIENTEK 选择了让 Bootloader 和 Config 各占一个最小 Sector 的方案。

证据：

- `Legacy/CONFIG/interface/config.h:19-23` — ALIENTEK 代码中的地址常量
- 注释行：`//16K bootloader+ 16 模拟eeprom`

---

## 3. 为什么需要持久化参数

### 3.1 存储内容

四旋翼飞行控制器需要在 Flash 中持久化的参数（来自 `config_param_t`，约 200 字节）：

**PID 参数（核心，约 120 字节）**

- `pidAngle`：角度环 PID（roll/pitch/yaw）
- `pidRate`：角速度环 PID（roll/pitch/yaw）
- `pidPos`：位置环 PID（vx/vy/vz/x/y/z）

**Trim 补偿**

- `trimP`：pitch 方向补偿
- `trimR`：roll 方向补偿

**基础参数**

- `thrustBase`：基础油门值

**完整性**

- `version`：配置版本号（当前 V1.3）
- `checksum`：加法和校验

### 3.2 必要性

| 场景 | 说明 |
|------|------|
| **PID 调参是迭代过程** | 每台四旋翼的物理特性（重量分布、电机一致性、桨叶差异）不同。PID 参数需要多次飞行测试迭代。如果每次调参都要重新编译、烧录固件，调参效率极低。 |
| **地面站无线调参** | 通过 ATKP 无线协议接收地面站的 PID 参数，飞行中实时调整，落地后保存。参数必须掉电不丢失。 |
| **每台机器不同** | Trim 补偿和 thrustBase 因机器而异。同一固件支持多台机器时，参数必须独立于固件存储。 |
| **固件升级不丢参数** | Bootloader + Config + App 分离架构的核心价值：升级 Application 固件时，Config 区的 PID 参数不受影响。 |

### 3.3 参数写入流程

```
地面站 --(ATKP 无线协议)--> atkp.c 接收 PID 更新
    --> 修改内存中的 config_param_t
    --> configParamGiveSemaphore() 通知 FreeRTOS task
    --> configParamTask() 延迟写入 Flash（批量合并多次修改）
    --> stmflash.c / bsp_flash.c 擦除 Sector 1 + 编程
```

**延迟写入机制**：参数更新后不立即写 Flash，而是标记 dirty，通过信号量/队列通知后台任务。这避免了频繁擦写 Flash（延长寿命），并将多次修改合并为一次写入。

**自动保存触发点**：`Legacy/FLIGHT/src/state_control.c:117` — 解锁（disarm）后 1500 个控制周期自动触发保存。

### 3.4 重置机制

`Legacy/COMMUNICATE/src/atkp.c:496`：地面站发送 `D_ACK_RESET_PARAM` 命令 → `resetConfigParamPID()` 将所有 PID 恢复为默认值 → 写入 Flash。

默认值定义在 `configParamDefault` 结构体中，是 ALIENTEK 针对 MiniFly 硬件调校的出厂 PID，作为恢复基线。

### 3.5 为什么不用外挂 EEPROM

MiniFly 硬件上**没有外挂 EEPROM 芯片**。STM32F411CE 内部 Flash 模拟 EEPROM 是唯一的非易失存储手段。这也是为什么必须预留一个完整的 Flash Sector——擦除只能以 Sector 为单位。

---

## 4. Bootloader 分析

### 4.1 是原厂的吗？不是。

**这是 ALIENTEK（正点原子）自定义的 Bootloader，不是 STM32 原厂系统 Bootloader。**

三条证据：

| 证据 | 说明 |
|------|------|
| **物理位置不同** | 原厂系统 bootloader 固化在 ROM `0x1FFF0000`（ST 出厂烧录，不可擦除），通过 BOOT0 引脚拉高启动。本项目的 bootloader 位于用户 Flash `0x08000000` Sector 0，用户 Flash 的内容只能由开发者烧录。 |
| **向量表重定位** | `Legacy/HARDWARE/src/nvic.c:32`：`NVIC_SetVectorTable(FIRMWARE_START_ADDR, 0)` 将向量表从 `0x08000000` 搬到 `0x08008000`。如果用的是原厂 ROM bootloader，向量表本来就在 `0x08000000`，无需重定位。 |
| **源码不在本仓库** | 整个 Legacy/ 搜索不到 bootloader 源码。这是 ALIENTEK 的典型发布方式：bootloader 作为独立项目单独维护，预编译成二进制分发。用户通过 ST-Link 烧录一次 bootloader 后，后续固件更新通过 bootloader 的 USB/串口完成。 |

### 4.2 启动流程

```
上电 / 复位
    |
    v
Bootloader (0x08000000, Sector 0)
    |
    +--> 收到升级指令 --> 通过 USB/串口接收新固件 --> 写入 Application 区 --> 跳转到 0x08008000
    |
    +--> 未收到升级指令 --> 直接跳转到 Application (0x08008000)
                                |
                                v
                         nvicInit() --> NVIC_SetVectorTable(0x08008000, 0)
                                |
                                v
                         进入 main() --> 正常运行
```

### 4.3 STM32 原厂系统 Bootloader vs 自定义 Bootloader

| 特性 | STM32 原厂系统 Bootloader | ALIENTEK 自定义 Bootloader |
|------|--------------------------|---------------------------|
| 位置 | ROM `0x1FFF0000` | 用户 Flash `0x08000000` |
| 来源 | ST 出厂烧录 | 开发者烧录 |
| 启动方式 | BOOT0 引脚拉高 | 上电自动运行 |
| 协议 | ST 内置协议（USART/I2C/SPI/USB DFU） | ALIENTEK 自定义协议 |
| 是否可改 | 不可擦除修改 | 完全可控 |
| 源码 | 不开源（ST 闭源） | 独立仓库，未包含在本 repo |

---

## 5. 旧代码与新代码对比

| 层次 | Legacy（SPL） | 新架构（HAL） |
|------|--------------|--------------|
| 底层驱动 | `Legacy/HARDWARE/src/stmflash.c`（SPL API） | `Drivers/BSP/Src/bsp_flash.c`（HAL API） |
| 配置管理层 | `Legacy/CONFIG/src/config_param.c` | `Core/Src/services/config_service.c` |
| 常量定义 | `Legacy/CONFIG/interface/config.h`（分散在 4 个文件） | `board_memory.h`（计划中，单一真源） |
| Flash API 设计 | `STMFLASH_Read` / `STMFLASH_Write`（合并擦除+写入） | `bsp_flash_init` / `bsp_flash_read` / `bsp_flash_erase` / `bsp_flash_write`（职责分离） |
| 数据完整性 | VERSION + 加法 checksum | VERSION + `__attribute__((packed))` + 加法 checksum |
| 并发保护 | FreeRTOS 二值信号量 | FreeRTOS mutex + queue（支持 ISR 安全写入标记） |
| 启动区保护 | 无 | `bsp_flash.c` 主动保护 Bootloader + Config 区 |

### 5.1 Legacy Flash 写入存在的风险

旧 `stmflash.c` 的 `STMFLASH_Write` 在写入前检测目标地址是否非 0xFF，若是则自动擦除：

- 没有显式的擦除/写入分离
- 没有扇区保护——理论上应用代码可以擦除 Bootloader 区
- 擦除期间阻塞总线，不响应中断

新 `bsp_flash.c` 解决了这些问题：主动拒绝对保护区的操作，API 职责清晰分离。

---

## 6. CubeMX 配置指南

CubeMX 本身不管理 Flash 分区，但以下配置与 Flash 存储密切相关：

### 6.1 在 CubeMX 中需要配置的项目

| 配置项 | 位置 | 值 | 说明 |
|--------|------|-----|------|
| MCU 型号 | Project Manager → MCU | STM32F411CEUx | 决定 Flash 参数和 Sector 布局 |
| RCC HSE | Pinout → RCC | Bypass Clock Source（8MHz） | 外部晶振，PLL 输入源 |
| RCC PLL | Clock Configuration | PLL_M=8, PLL_N=192, PLL_P=2, PLL_Q=4 | 产生 96MHz SYSCLK，48MHz USB |
| Flash Latency | 自动 | 2 WS（96MHz 下自动设定） | HAL 库自动配置 |
| SYSCLK | Clock Configuration | 96 MHz | 目标系统时钟 |

### 6.2 CubeMX 无法自动处理的部分

1. **Linker Script**：需要手动修改 `STM32F411XX_FLASH.ld`，将 `FLASH` 的 `ORIGIN` 从 `0x08000000` 改为 `0x08008000`，`LENGTH` 从 `512K` 改为 `480K`
2. **board_memory.h**：手动创建，定义 Flash 布局宏
3. **Bootloader**：独立项目，不在本仓库维护

---

## 7. 总结

| 问题 | 答案 |
|------|------|
| board_memory.h 是什么 | 待创建的板级存储常量单一真源文件，收敛分散在 4 个文件中的 Flash 布局定义 |
| 为什么需要持久化参数 | PID 调参需要迭代、trim 每台机器不同、固件升级不丢参数、无外挂 EEPROM 只能用内部 Flash |
| 分区谁定义的 | ALIENTEK 自定义，16KB 对齐受 STM32F411 Flash Sector 硬件约束 |
| Bootloader 是原厂的吗 | 不是，是 ALIENTEK 的自定义 Bootloader，源码在独立仓库，未包含在本 repo |
| CubeMX 能做什么 | 配置 MCU 型号（决定 Flash 参数）和时钟树（决定 Flash Latency），分区和 linker 需手动配置 |
