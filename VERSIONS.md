# Spectral Micro versions

Which firmware your Spectral Micro runs, and which release, bootloader and manual go with it.

## Releases

| Release | Date | What it is | Bootloader | Files |
|---|---|---|---|---|
| **V110** | 2026-11-13 | gripper moves end cleanly and start softly; positions 0 and 255 clear the stops and 255 closes fully; the gripper keeps its own current limit; CAN parameters 90-93; `#Estop` over UART; brake or coast saved; no watchdog hold while idle | required | [download](https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller/releases/tag/V110) |
| V109 | 2026-11-10 | phase B's enable pin fixed (full torque, calibrations that pass); the driver kept awake in every mode; parameters by number, captures, the watchdog's actions, sound; bench-tuned current-loop defaults; no 108, it shares its numbers with STEPFOC from here | required | [download](https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller/releases/tag/V109) |
| V107 | 2026-08-26 | PD mode over CAN uses the commanded speed; idle can brake or coast (`#Brake`); a `#Cal` fix for some motor units | required | [download](https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller/releases/tag/V107) |
| V106 | 2026-07-06 | no dropped CAN messages in bursts; a 100 ms watchdog; CRC checks on the saved configuration; `#Help` | required | [download](https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller/releases/tag/V106) |
| V105 | 2026-07-05 | `#Cal` detects reversed phase wiring and finds the angle offset itself; `#Info` reports the true version | required | [download](https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller/releases/tag/V105) |
| V104 | 2026-06-19 | the first release for the bootloader, from `Bootloader_main`; the Kt request, the CAN voltage limit | required | [download](https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller/releases/tag/V104) |
| v101 | 2025-09-29 | the last release from `main`, flashed with a programming adapter: the angle offset calibration, gripper mode fixes; run `#Default` after updating | none | [tag](https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller/releases/tag/v101) |
| v100 | 2024-03-27 | the first tagged release | none | [tag](https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller/releases/tag/v100) |

The release notes of each are on its release page and in the manual's
[Firmware release notes](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/appendix/firmware-release-notes/).

## Which version do I have?

| | What differs | How to tell |
|---|---|---|
| Release 105 or newer | runs above the bootloader and updates over UART or CAN, with [Motor GUI](https://motorgui.com/) or the bootloader's tool | `#Info` prints `Software version: <number>`; Motor GUI shows it in the driver's identity |
| A driver that reports 101 | release 101 without the bootloader, or release 104 (or an earlier build of `Bootloader_main`) above it: before 105 the number came from the EEPROM, and 101 and 104 both write 101 | power the driver up while Motor GUI's *Scan bootloaders* or `bl_flash.py list` is polling: a driver with the bootloader answers in its first two seconds |
| Bought before release 105 | shipped with firmware 101 or 103 | as for a driver that reports 101; without the bootloader, [install it once](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/07-firmware/bootloader/) with a programming adapter, then [update as usual](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/07-firmware/update/) |
| Board v2.0 | the production board; it runs firmware 101 and newer | `#Info` prints it as `Hardware version`; the silkscreen on the bottom reads "Spectral motor controller micro v2.0, 10.1.2024" |

## What works together

The firmware, the bootloader, the Python library, the PAR6 runtime, the STEPFOC equivalent and the board for each
release: [Versions and compatibility](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/appendix/compatibility/) in the manual.
