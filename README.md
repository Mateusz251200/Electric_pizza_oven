# Electric pizza oven project

**The goal of this project is to design and build a homemade electric oven for pizza and bread, covering the electronics, control system, and mechanical structure.**

**Status:** Early work in progress.

*Disclaimer:* This is a personal project, and I'm not yet an expert, so some solutions may not be optimal.

## Planned features

- Temperature range: up to 500°C
- PID temperature control, possibly with auto-tuning
- Independent top and bottom heating elements, each with its own temperature sensor
- Multiple heating modes for experimental purposes:
  - single temperature setpoint with coupled heaters (one control loop)
  - single setpoint with variable power ratio between heaters (one control loop)
  - independent top and bottom temperature control (two control loops)
  - hysteresis control (one control loop)
  - manual (open-loop) control
- User interface including an LCD (HD44780) with an interactive menu
- Overheating protection
- USB connection for diagnostic data
- System based on an STM32 MCU (specific model not yet selected; the NUCLEO-F303RE is used for prototyping)

## Progress list

- 🔄 Control panel
  - ✅ HD44780 LCD driver (I2C+DMA)
  - 🔄 Interactive menu
- ⏳ Temperature measurement circuit
- ⏳ PID algorithm
- ⏳ Heating modes
- ⏳ Overheating protection
- ⏳ PCB
- ⏳ Heating elements and solid-state relays
- ⏳ Power supply
- ⏳ Mechanical hardware (baking chamber, insulation, door, casing, etc.)

Legend: ✅ — works; 🔧 — needs improvement; 🔄 — in progress; ⏳ — not yet started

*Note:* The firmware is developed using STM32CubeMX, CMake, and VS Code with the STM32Cube extension.

## Licensing

See [LICENSE.md](./LICENSE.md).
