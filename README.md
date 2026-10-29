## Overview

This project is a comprehensive robot control system for cable-driven parallel robots (CDPRs) with integrated gripper and rail control. It supports calibration, manual operation, automated brick placement, trajectory execution, and demo modes. The system is designed for use with a Windows environment and leverages Visual Studio Code for development.

## Features

- **Calibration Mode:** Set cable torques, update robot position/configuration, control individual motors, and clear alerts.
- **Operation Mode:** Control robot position, run brick placement routines, execute point-to-point and trajectory files, save/load positions, and run demo sequences.
- **Gripper & Rail Control:** Full menu-driven control for gripper and rail motors.
- **Logging:** All operations are logged for traceability.
- **File-based Configuration:** Robot configuration and last position are loaded from JSON and TXT files.
- **Network Integration:** Can request position data from a Raspberry Pi via UDP.

### Prerequisites

- Windows OS
- Visual Studio 2019 or newer with the **Desktop development with C++**
  workload (the standalone Build Tools edition is also supported)
- Visual Studio Code
- Teknic ClearPath Driver: https://teknic.com/downloads/
- Required hardware connected and powered

### Build

The build script discovers Visual Studio through `vswhere`, selects the x86
toolchain required by the bundled 32-bit vendor libraries, and derives all
source, dependency, and output paths from the repository location. A Visual
Studio Developer Command Prompt is not required.

From a regular PowerShell terminal:

```powershell
.\tools\build.ps1 -Target Main
```

The controller executable is written to `bin\main.exe`. Other available build
targets are `GUI`, `DeltaServer`, `LegacyTest`, and `OfflineTests`. The VS Code
build tasks call the same script.

For a Visual Studio installation that `vswhere` cannot find, pass the developer
command script explicitly:

```powershell
.\tools\build.ps1 -Target Main -VsDevCmd 'D:\VisualStudio\Common7\Tools\VsDevCmd.bat'
```

Alternatively, set the `CU_BRICK_VSDEVCMD` environment variable to that path.

### Hardware-independent tests

The offline test target covers serial-controller lifecycle and failure
propagation, brake rollback, trajectory generation, and cable kinematics. It
uses injected fake serial ports and does not open COM ports, TwinCAT ADS,
Teknic nodes, or network sockets.

Run it from PowerShell (this delegates to the shared portable build script):

```powershell
.\test\run_offline_tests.ps1
```

Alternatively, run the VS Code task `Test: offline unit tests`. Build products
are written under the system temporary directory rather than the repository.

### Run

1. Execute the program:
   ```sh
   ./bin/Main.exe
   ```

## Usage

On startup, the program loads configuration from `RobotConfig.json` and attempts to recover the last position from `lastPos.txt`. You will be presented with a menu to select Calibration or Operation mode.

---
   
### Calibration Mode

For setup, diagnostics, and maintenance. Menu options:

1. **Set Cables Torque**  
   Adjust the tension of all cables for safe operation.
2. **Request current cable motor torque readings**  
   Display the measured torque for each cable motor.
3. **Control Cable Motor**  
   Manually operate individual cable motors for diagnostics or setup.
4. **Update Robot Pos**  
   Load robot position from a file and update the system state.
5. **Reset EE Rotation to Zero**  
   Set the end effector’s rotation angles to zero.
6. **Control Linear Rail Motor**  
   Manually operate rail motors for calibration or troubleshooting.
7. **Control Gripper**  
   Open, close, rotate, release, or calibrate the gripper.
8. **Update Robot Config**  
   Load a new robot configuration from a JSON file.
9. **Print Robot Status**  
   Show current end effector position, rail offsets, and cable lengths.
10. **Clear Exception**  
    Reset any error or alert states in the cable motors.
11. **Robot Control Mode**  
    Enter the main robot control menu for position and gripper operations.
12. **Request Current Position from RPi**  
    Query the robot’s position from a Raspberry Pi over UDP.
q. **Finish Calibration**  
   Exit calibration mode and return to the main menu.

---

### Operation Mode

For running robot tasks, brick placement, and demos. Menu options:

1. **Robot Control**  
   Access robot position control, move to home/pre-pickup, gripper and rail control.
2. **Read Brick Positions File**  
   Load a CSV file with brick positions and run automated pick-and-place routines.
3. **Read Point-to-Point Path File**  
   Execute a sequence of robot movements from a CSV file of points.
4. **Read Trajectory File**  
   Run a pre-defined trajectory from a CSV file.
5. **Request current cable motor torque readings**  
   Display the measured torque for each cable motor.
6. **Save Current EE Pos to File**  
   Save the current end effector position to a file for later recall.
7. **Print Robot Status**  
   Show current end effector position, rail offsets, and cable lengths.
8. **Demo Mode**  
   Run a demonstration sequence using a sample brick position file.
9. **Auto Aim Mode**
   Read cameraOffset in RobotConfig.json,
   Move to the nearest brick, pick up the brick, and move to the other nearest brick.
   Place a brick at the right/left/top of the brick.
   

---

Follow on-screen prompts to interact with the robot.

## File Structure

- `ver_2/src/Main.cpp`: Main program logic and menu system.
- `ver_2/include/`: Header files for robot, logger, trajectory generator, and utilities.
- `ver_2/tools/json.hpp`: JSON parsing library.
- `ver_2/bricks.csv`, `ver_2/demoab1.csv`, `ver_2/traj.csv`: Example data files for robot tasks.
- `ver_2/log/robot.log`: Operation log file.
- `ver_2/lastPos.txt`: Stores last known robot position.
- `ver_2/RobotConfig.json`: Robot configuration file.
- `ver_2/dxfextract.py`: Python calibration tool for Leica laser measurements.
- `arduino/`: Contains all code needed for the end effector, including ArUco marker calibration and gripper control.
- **Note:** `RobotConfig.json` and example CSV files for brick positions are located inside the `ver_2` folder.

## Dependencies

- [Eigen](ver_2/Dependencies/eigen-3.3.7/): Linear algebra library.
- [DynamixelSDK](ver_2/Dependencies/DynamixelSDK-3.7.31/): Motor control SDK.
- [sFoundation20](ver_2/Dependencies/sFoundation20/): Additional hardware support.
- [TcAdsDll](ver_2/Dependencies/TcAdsDll/): TwinCAT ADS communication.

## Notes

- Ensure all hardware is connected before running.
- Log files and position files are automatically managed.
- Demo mode uses `demoab1.csv` for brick placement demonstration.

## License

See individual dependency folders for license information.

## Contact

For support, please contact the project maintainer.
