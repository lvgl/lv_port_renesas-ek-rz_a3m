# LVGL ported to Renesas EK-RZ/A3M

**:rocket: This repository is available in the [LVGL Project Creator](https://lvgl.io/tools/project-creator), making it easy to create and customize a new project in just a few clicks.**

## Overview

The Renesas EK-RZ/A3M board is an evaluation kit for the RZ/A3M MPU.
It is an MPU but it has a focus on baremetal and RTOS applications.
It boasts a mighty 1 GHz Cortex-A55 CPU and 128 MB of DDR3L-SDRAM.
It is easy to use the low-level IO, of which it provides a lot.
The board offers connectivity to many
ecosystems like Arduino, Qwiic, Grove, mikroBUS, and Pmod.
A 1280x720 display is included in the kit. It mounts to the board
via a board-to-board connector.

## Buy

The Renesas EK-RZ/A3M can be purchased from https://www.renesas.com/en/products/microcontrollers-microprocessors/rz-mpus/ek-rza3m-evaluation-kit-rza3m-mpu.

## Benchmark

<a href="https://www.youtube.com/watch?v=IEjBGgrR_mk">
    <img src="https://github.com/user-attachments/assets/10766145-1a92-4bec-a952-68ce376974fa" width="75%">
</a>

| Name                      | Avg. CPU | Avg. FPS | Avg. time | render time | flush time |
| :------------------------ | -------: | -------: | --------: | ----------: | ---------: |
| Empty screen              | 50%      | 30       | 23        | 1           | 22         |
| Moving wallpaper          | 56%      | 30       | 29        | 4           | 25         |
| Single rectangle          |  8%      | 30       | 15        | 0           | 15         |
| Multiple rectangles       | 23%      | 30       | 20        | 1           | 19         |
| Multiple RGB images       | 56%      | 30       | 24        | 3           | 21         |
| Multiple ARGB images      | 68%      | 30       | 27        | 6           | 21         |
| Rotated ARGB images       | 44%      | 29       | 27        | 12          | 15         |
| Multiple labels           | 70%      | 30       | 31        | 7           | 24         |
| Screen sized text         | 80%      | 20       | 48        | 24          | 24         |
| Multiple arcs             | 72%      | 30       | 30        | 7           | 23         |
| Containers                | 18%      | 30       | 14        | 4           | 10         |
| Containers with overlay   | 90%      | 30       | 31        | 16          | 15         |
| Containers with opa       | 25%      | 30       | 17        | 5           | 12         |
| Containers with opa_layer | 37%      | 29       | 21        | 11          | 10         |
| Containers with scrolling | 92%      | 29       | 31        | 15          | 16         |
| Widgets demo              | 70%      | 25       | 35        | 16          | 19         |
| All scenes avg.           | 53%      | 28       | 26        | 8           | 18         |

## Specification

### CPU and Memory
- **MPU:** 1 GHz ARM Cortex-A55
- **RAM:** 128 MB internal DDR3L-SDRAM and 128 KB more internal SRAM,
- **Flash:** 128 MB external QSPI NAND flash and 32 MB more external QSPI NOR flash
- **GPU:** None

### Display and Touch
- **Resolution:** 1280x720
- **Display Size:** 4.3"
- **Interface:** MIPI
- **Color Depth:** 16-bit
- **Technology:** TFT
- **DPI:** 342 px/inch
- **Touch Pad:** Capacitive

### Connectivity
- Audio codec with i2s and ADC
- Micro-SD Card slot
- USB host or device
- Two 40 pin native pin headers
- MIPI display connector (connects to included display board)
- Ecosystem connectors (Arduino, Qwiic, Grove, mikroBUS, and Pmod)

## Getting started

### Hardware setup
- Mount the graphics expansion board to the mainboard.
  Two of the standoffs on the mainboard should be replaced with shorter ones and then
  the graphics board can be mounted underneath the mainboard.
  The shorter standoffs can be secured with the included screws.
- Set all the DIP switches to OFF.
- Connect a USB C cable to DEBUG1 (J10) and your PC.

### Software setup
- [Install e2 studio for your OS](https://www.renesas.com/en/software-tool/e2studio-information-rz-family).
  - When prompted, choose "Custom Install".
  - Ensure "RZ" is included in your selection of "Device Families" to install.
  - Ensure "Renesas FSP Smart Configurator Core" and "Renesas FSP Smart Configurator ARM"
    are included in your selection of "Customize Features".
  - Ensure "GCC ToolChains & Utilities" is selected and "GCC ARM A-Profile (AArch64 bare-metal) 13.2 rel1"
    under that tab is also selected in "Additional Software".
- Install FSP Packs. Please install FSP v3.4.0 for this project.
  It's okay if you have multiple versions installed.
  - On Windows, simply download RZA_FSP_Packs_v3.4.0.exe
    [from here](https://github.com/renesas/rza-fsp/releases) under "Assets".
    Run the installer and follow the prompts.
  - On Linux, download RZA_FSP_Packs_v3.4.0.zip
    [from here](https://github.com/renesas/rza-fsp/releases/tag/v3.4.0) under "Assets".
    Locate the e2 studio install location. If it is `~/.local/share/renesas/e2_studio`, unzip the file
    with the following commands:
    ```shell
    cd ~/.local/share/renesas/e2_studio
    unzip ~/Downloads/RZA_FSP_Packs_v3.4.0.zip
    ```
    The directory structure in the ZIP overlaps with the
    e2_studio install loction. This is expected. The `unzip` process will
    update the directory structure with the new FSP files from the ZIP and preserve any existing FSP packs.
    [Refer to "2.2.3 Installation of FSP Packs using Package Zip file" in this guide](https://www.renesas.com/en/document/apn/rza-getting-started-flexible-software-package)
    for more details.

### Run the project
- Clone the repository
  ```shell
  git clone --recurse-submodules https://github.com/lvgl/lv_port_renesas-ek-rz_a3m.git
  ```
- Open e2 studio and go to **File > Open Projects from File System...**. Click "Directory"
  and navigate to the cloned project and then click "Finish".
- Click the hammer to build the project. If it is greyed-out, first single-click the project
  "lv_ek_rza3m" in the left sidebar and the hammer should become clickable.
- To upload and run the project, click the bug (debug) icon. The debugger will break (stop execution)
  at the beginning. Click the "Resume" button to continue execution.

### Debugging
- In the previous section, the project was run using the debugger.
  Simply continue using the interactive debugger in e2 studio to debug your program.
  Set breakpoints, continue, step, etc. as with any other Eclipse-based IDE.

## Notes

## Contribution and Support

If you find any issues with the development board feel free to open an Issue in this repository. For LVGL related issues (features, bugs, etc) please use the main [lvgl repository](https://github.com/lvgl/lvgl).

If you found a bug and found a solution too please send a Pull request. If you are new to Pull requests refer to [Our Guide](https://docs.lvgl.io/master/CONTRIBUTING.html#pull-request) to learn the basics.
