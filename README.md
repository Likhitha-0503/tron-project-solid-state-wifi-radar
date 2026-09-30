# Solid-State Switched-Beam WiFi Radar

## 1. Project Overview
This project is a high-accuracy indoor localization system designed to leverage edge-based artificial intelligence. By steering Wi-Fi signals through an RF switching matrix rather than relying on mechanical parts, the system rapidly captures spatial telemetry data. The raw RF data is processed locally to determine target positioning with low latency. The software architecture runs on a highly deterministic real-time operating system (RTOS) and utilizes optimized AI models for spatial inference mapping.

## 2. Key Features
* **Solid-State Beam Steering:** Implements a Butler Matrix paired with an SP4T RF switch for rapid, mechanical-free directional signal control.
* **Edge AI Processing:** Executes neural network inference locally to minimize latency, eliminate cloud dependency, and maintain privacy-compliant indoor localization.
* **Real-Time OS Integration:** Built on µT-Kernel 3.0 BSP2 to ensure hard real-time execution for state switching and sensor polling.
* **High-Speed Telemetry:** Utilizes a dedicated communications layer for low-latency data transfer between the RF front-end and the AI processing back-end.

## 3. Hardware Architecture
The hardware design is modular, splitting the AI inference backend and the RF telemetry frontend across two dedicated processing units.
* **Main Processing Node:** STM32N6570-DK Discovery Kit featuring an ARM Cortex-M55 core at 800 MHz and an ST Neural-ART Accelerator.
* **Wi-Fi / RF Node:** ESP32U-WROOM-32UE microcontroller board.
* **Core RF Components:** Uses a Butler Matrix for passive RF beamforming and an SP4T Switch for sequential state-switching between the beamforming paths.

## 4. Software Architecture
The system operates on a dual-node topology:
* **ESP32 Frontend:** Executes bare-metal radio firmware developed using Visual Studio Code and the ESP-IDF extension. It utilizes a 4-deep circular DMA queue of 8-byte packets to ensure a zero-gap SPI telemetry pipeline without garbage data.
* **STM32N6 Backend:** Runs the IEEE-compliant µT-Kernel 3.0 RTOS developed entirely within STM32CubeIDE. It processes a 10-class spatial classifier neural network compiled via ST's AtoNN compiler, completing inference in < 1 ms. Spatial output is smoothed by a 1D Kalman filter and visualized using a Gaussian Heatmap Engine rendered to an LCD.

## 5. Project Structure
```text
tron-project-solid-state-wifi-radar
├── Docs
│   ├── ESP-IDF_User_manual.pdf                       // Step-by-step setup for the ESP32
│   ├── TRON_CONTEST_Solid_State_Switched_WiFi_Radar.pdf // Main system architecture whitepaper
│   └── polar_generate_report.txt                     // NPU memory and latency metrics from AtoNN
├── ESP32_Firmware
│   ├── CMakeLists.txt                                // Root CMake configuration for ESP-IDF
│   ├── sdkconfig                                     // Saved ESP-IDF system configurations
│   └── main
│       ├── CMakeLists.txt                            // Registers the source files for compilation
│       ├── esp_arechfirmware.c                       // Promiscuous sniffer, beam steering, and SPI DMA logic
│       └── telemetry_contract.h                      // Shared 8-byte struct definition for SPI payloads
└── STM32_Firmware
    ├── FSBL                                          // First Stage Boot Loader (Runs in internal SRAM)
    │   ├── Core
    │   │   ├── Inc
    │   │   │   ├── extmem_manager.h
    │   │   │   ├── gpdma.h                           // General Purpose DMA init for early boot
    │   │   │   ├── gpio.h                            // FSBL pin configurations
    │   │   │   ├── icache.h                          // Instruction cache enable/disable logic
    │   │   │   ├── main.h
    │   │   │   ├── stm32_extmem_conf.h
    │   │   │   ├── stm32n6xx_hal_conf.h
    │   │   │   ├── stm32n6xx_it.h
    │   │   │   ├── xspi.h                            // Octo-SPI peripheral initialization
    │   │   │   └── xspim.h
    │   │   └── Src
    │   │       ├── extmem_manager.c                  // Manages booting Appli from external flash
    │   │       ├── gpdma.c
    │   │       ├── gpio.c
    │   │       ├── icache.c
    │   │       ├── main.c                            // FSBL entry point
    │   │       ├── stm32n6xx_hal_msp.c               // Hardware Support Package for FSBL peripherals
    │   │       ├── stm32n6xx_it.c                    // Interrupt handlers for the bootloader
    │   │       ├── sysmem.c                          // System memory management stubs
    │   │       ├── system_stm32n6xx_fsbl.c           // Low-level system clock and bus setup
    │   │       ├── xspi.c
    │   │       └── xspim.c                           // Octo-SPI memory manager routing
    │   ├── Secure_nsclib                             // TrustZone secure domain library
    │   └── STM32N657X0HXQ_AXISRAM2_fsbl.ld           // Linker script targeting internal AXISRAM2
    └── Appli                                         // Main Application (Executes in-place from XSPI)
        ├── Application
        ├── Core
        │   ├── Inc
        │   │   ├── heatmap.h                         // Gaussian math and dirty-cell render logic
        │   │   ├── lcd_port.h                        // RK050HR18 LTDC configuration and layer blending
        │   │   ├── main.h
        │   │   ├── spi.h
        │   │   └── telemetry_contract.h              // Must match the ESP32 version exactly
        │   └── Src
        │       ├── heatmap.c
        │       ├── lcd_port.c
        │       ├── main.c                            // Peripheral init before launching the RTOS
        │       ├── spi.c                             // SPI5 and GPDMA1 configuration for telemetry
        │       └── usermain.c                        // µT-Kernel 3.0 tasks (Capture, Tracking, Display)
        ├── Npu
        │   └── ll_aton                               // Low-level API for the Neural-ART Accelerator
        │       └── ll_aton_NN_interface.h
        ├── X-CUBE-AI
        │   └── App
        │       ├── app_x-cube-ai.c                   // Initializes the generated neural network
        │       ├── app_x-cube-ai.h
        │       ├── constants_ai.h
        │       ├── polar.c                           // Auto-generated 10-class spatial DNN topology
        │       └── polar.h                           // Buffer alignment and size macros for polar.c
        ├── mtk3_bsp2
        │   ├── config
        │   └── include                               // Complete µT-Kernel 3.0 source tree
        ├── mtkernel
        ├── Drivers                                   // ST HAL and Low-Level drivers
        ├── Middlewares                               // ExtMem, X-CUBE-AI libraries
        └── STM32N657X0HXQ_LRUN.ld                    // Linker script mapping code to external XSPI flash