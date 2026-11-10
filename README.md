<p align="center">
  <img src="Photos/bldc_grouped-Photoroom.png" alt="Spectral Micro BLDC controller" width="100%">
</p>

<h1 align="center">Spectral Micro</h1>

<p align="center">
  <b>Compact FOC controller for brushless (BLDC) motors, with CAN, UART and open-source firmware.</b><br>
  It mounts behind the motor, measures the motor in one calibration run, and controls its position, speed, torque or impedance.
</p>

<p align="center">
  <a href="https://source-robotics.com/products/spectral-micro-bldc-controller"><b>Buy</b></a>&nbsp;&nbsp;·&nbsp;&nbsp;
  <a href="https://docs.source-robotics.com/Spectral-BLDC-docs/"><b>Documentation</b></a>&nbsp;&nbsp;·&nbsp;&nbsp;
  <a href="https://docs.source-robotics.com/Spectral-BLDC-docs/latest/quick-start/"><b>Quick start</b></a>&nbsp;&nbsp;·&nbsp;&nbsp;
  <a href="https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller/releases/latest"><b>Download</b></a>&nbsp;&nbsp;·&nbsp;&nbsp;
  <a href="https://discord.com/invite/prjUvjmGpZ"><b>Discord</b></a>
</p>

<p align="center">
  <a href="https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller/releases/latest"><img src="https://img.shields.io/github/v/release/Source-Robotics/Spectral-Micro-BLDC-controller?style=flat-square&color=143952&label=release" alt="Release"></a>
  <a href="LICENSE"><img src="https://img.shields.io/badge/license-GPL--3.0-143952?style=flat-square" alt="License"></a>
  <a href="https://discord.com/invite/prjUvjmGpZ"><img src="https://img.shields.io/discord/1072498136284667955?style=flat-square&logo=discord&logoColor=white&label=discord&color=292B37" alt="Discord"></a>
  <a href="https://doi.org/10.5281/zenodo.24113621"><img src="https://img.shields.io/badge/DOI-10.5281%2Fzenodo.24113621-143952?style=flat-square" alt="DOI"></a>
</p>

<div align="center">

| **12 to 29 V** | **1.5 A** | **14-bit** | **5 kHz** | **39 × 39 mm** |
|:-:|:-:|:-:|:-:|:-:|
| supply, 24 V typical | motor current, 2.8 A max | encoder | control loop | NEMA 17 size, 8 g |

</div>

## Project overview

<table>
  <tr>
    <td width="55%" valign="middle">
      Spectral Micro is a brushless motor controller designed to offer affordable and capable hardware, complemented
      by open-source software. It supports multiple firmware options, integrates a 14-bit magnetic encoder, inline
      current sensing and a CAN and UART data interface, and provides position, velocity, torque and impedance
      control. Everything you need for developing high-performance robotic solutions is included. It is ideal for
      gimbal motors, quadrupeds, robotic arms and grippers: see the
      <a href="https://docs.source-robotics.com/Spectral-BLDC-docs/latest/03-technical-data/tested-motors/">tested motors</a>.
      <br><br>
      <a href="https://youtu.be/FRhTW86H9zU"><b>&#9654;&#xFE0E;&nbsp; Watch the Spectral Micro introduction</b></a>
    </td>
    <td width="45%" valign="middle">
      <a href="https://youtu.be/FRhTW86H9zU"><img src=".github/readme/video.jpg" alt="Spectral Micro - BLDC motor controller, on YouTube" width="100%"></a>
    </td>
  </tr>
</table>

## Start here

| I want to | Go to |
|---|---|
| **Buy a driver or a starter kit** | [source-robotics.com](https://source-robotics.com/products/spectral-micro-bldc-controller) · [starter kit](https://source-robotics.com/products/spectral-micro-bldc-starter-kit) |
| **Choose a motor, mount and wire it** | [tested motors](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/03-technical-data/tested-motors/) · [what you need](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/01-overview/what-you-need/) · [mounting and wiring](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/04-mounting/) · [STEP files](STEP%20files/) |
| **Set it up and make it move** | [Quick start](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/quick-start/) · [Motor GUI](https://motorgui.com/) in the browser ([guide](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/05-setup/motor-gui/)) · [calibration](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/05-setup/calibration/) |
| **Control it** | [over CAN](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/06-operation/can/) · [over UART](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/06-operation/uart/) · [Python library](https://github.com/Source-Robotics/Spectral-BLDC-Python) and [Source Robotics toolbox](https://github.com/Source-Robotics/Source-Robotics-Toolbox) · [Arduino](https://github.com/Source-Robotics/SpectralMicroArduino) (early stage) · [ROS&nbsp;2](https://github.com/PCrnjak/SpectralMicroROS2) (early stage) · [API reference](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/reference/) |
| **Run SimpleFOC on it** | [SimpleFOC port and examples](https://github.com/Source-Robotics/Spectral-Micro-Simple-FOC) · [SimpleFOC guide](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/guides/simplefoc/) |
| **Update the firmware** | [latest release](https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller/releases/latest) · [updating through the bootloader](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/07-firmware/update/) · [a board without the bootloader](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/07-firmware/bootloader/) |
| **Use it in a gripper or a robot** | [SSG-48 gripper](https://github.com/Source-Robotics/SSG-48-adaptive-electric-gripper) · [gripper mode](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/06-operation/gripper-mode/) · [guides](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/guides/): mobile robot, quadruped leg, gravity compensation, teleoperation |

<details>
<summary><b>What is in this repository</b></summary>

| Folder | What is in it |
|---|---|
| [`Spectral BLDC Firmware`](Spectral%20BLDC%20Firmware/) | the firmware (PlatformIO, STM32F103CB): on `main` the one for boards without the bootloader (release 101 and earlier, no longer updated); releases from 104 on are built from the [`Bootloader_main`](https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller/tree/Bootloader_main) branch |
| [`Binaries`](Binaries/) | `firmware.bin`, the build of this branch, for a board without the bootloader |
| [`STEP files`](STEP%20files/) | the board and an example mounting bracket |
| [`Test scripts`](Test%20scripts/) | `PI_current_loop_calib.py`, the current loop's PI gains from a motor's resistance and inductance |
| [`Photos`](Photos/) | product photos |

The [latest release](https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller/releases/latest) has the
firmware image (`spectral-bldc-firmware-V<number>.bin`) and its manifest (`firmware.json`);
[Motor GUI](https://motorgui.com/) flashes it from there. [`en.stsw-link009.zip`](en.stsw-link009.zip) is the ST-LINK USB
driver (STSW-LINK009), for the programming adapter on Windows.

</details>

## Community

Ask in [Discussions](https://github.com/orgs/Source-Robotics/discussions), chat on
[Discord](https://discord.com/invite/prjUvjmGpZ), report a bug in
[Issues](https://github.com/Source-Robotics/Spectral-Micro-BLDC-controller/issues), and follow Source Robotics:

[![Discord](https://img.shields.io/discord/1072498136284667955?style=flat-square&logo=discord&logoColor=white&label=discord&color=143952)](https://discord.com/invite/prjUvjmGpZ)
[![Discussions](https://img.shields.io/badge/discussions-ask_a_question-143952?style=flat-square&logo=github&logoColor=white)](https://github.com/orgs/Source-Robotics/discussions)
[![YouTube](https://img.shields.io/badge/youtube-%40source--robotics-143952?style=flat-square&logo=youtube&logoColor=white)](https://www.youtube.com/@source-robotics)
[![Instagram](https://img.shields.io/badge/instagram-%40source__robotics-143952?style=flat-square&logo=instagram&logoColor=white)](https://www.instagram.com/source_robotics/)
[![X](https://img.shields.io/badge/x-%40SourceRobotics-143952?style=flat-square&logo=x&logoColor=white)](https://x.com/SourceRobotics)
[![LinkedIn](https://img.shields.io/badge/linkedin-Source_Robotics-143952?style=flat-square&logo=data%3Aimage%2Fsvg%2Bxml%3Bbase64%2CPHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHZpZXdCb3g9IjAgMCAyNCAyNCI%2BPHBhdGggZmlsbD0id2hpdGUiIGQ9Ik0yMC40NDcgMjAuNDUyaC0zLjU1NHYtNS41NjljMC0xLjMyOC0uMDI3LTMuMDM3LTEuODUyLTMuMDM3LTEuODUzIDAtMi4xMzYgMS40NDUtMi4xMzYgMi45Mzl2NS42NjdIOS4zNTFWOWgzLjQxNHYxLjU2MWguMDQ2Yy40NzctLjkgMS42MzctMS44NSAzLjM3LTEuODUgMy42MDEgMCA0LjI2NyAyLjM3IDQuMjY3IDUuNDU1djYuMjg2ek01LjMzNyA3LjQzM2MtMS4xNDQgMC0yLjA2My0uOTI2LTIuMDYzLTIuMDY1IDAtMS4xMzguOTItMi4wNjMgMi4wNjMtMi4wNjMgMS4xNCAwIDIuMDY0LjkyNSAyLjA2NCAyLjA2MyAwIDEuMTM5LS45MjUgMi4wNjUtMi4wNjQgMi4wNjV6bTEuNzgyIDEzLjAxOUgzLjU1NVY5aDMuNTY0djExLjQ1MnpNMjIuMjI1IDBIMS43NzFDLjc5MiAwIDAgLjc3NCAwIDEuNzI5djIwLjU0MkMwIDIzLjIyNy43OTIgMjQgMS43NzEgMjRoMjAuNDUxQzIzLjIgMjQgMjQgMjMuMjI3IDI0IDIyLjI3MVYxLjcyOUMyNCAuNzc0IDIzLjIgMCAyMi4yMjIgMGguMDAzeiIvPjwvc3ZnPg%3D%3D)](https://www.linkedin.com/company/source-robotics/)
[![Blog](https://img.shields.io/badge/blog-source--robotics.com-143952?style=flat-square&logo=rss&logoColor=white)](https://source-robotics.com/blogs/blog)

## Safety, liability and disclaimer

1. The software and hardware are still in development and may contain bugs, errors, or incomplete features.
2. Users are encouraged to use this software and hardware responsibly and at their own risk.

Read the manual's [safety chapter](https://docs.source-robotics.com/Spectral-BLDC-docs/latest/02-safety/) before you wire and power the driver.

## Licensing

Spectral firmware is under GPLv3 Licence

## Versions

Which firmware release your driver runs, and what works with it: [VERSIONS.md](VERSIONS.md).
