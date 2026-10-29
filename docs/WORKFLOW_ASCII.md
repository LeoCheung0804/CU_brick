# CU_brick function workflows and exit paths

Source review: 8 October 2026. Baseline: f9fb380, including the controller
changes in f839c1d. The diagrams describe the current source, not a verified
copy of an executable deployed on another machine.

This document covers every function defined in src/*.cpp, with overloads
grouped, followed by the first-party Python, Arduino, build, and test helpers.
Third-party library internals and the archived PLC project in bin/ are outside
this map. There are 219 function definitions in src/, including constructors,
destructors, and DeltaServer's inline methods. Named local lambdas are also
shown where they affect cleanup.

EE means end effector. Pose order in Robot is:

```text
[x, y, z, roll, pitch, yaw]
 |  |  |     |      |    |
 +--+--+     +------+------+-- radians
    |
    +-- meters

-->          next call or step
REJECT       command rejected before motion; session can remain ready
FAULT        session made unavailable; menus unwind toward shutdown
EXIT         process finishes through an explicit return/exit
CRASH RISK   exception or invalid memory access can bypass normal cleanup
WAIT         operation can block or wait for user input
```

The connection flags record software state. They are not continuous hardware
health checks. A successful serial write means eight bytes were written; the
application does not read a physical completion acknowledgement from the
gripper or brakes. Motor command success is also different from measured
arrival at the requested position.

## 1. Application startup and exit

Source: src/Main.cpp:1996, :2020.

```text
start.cmd --> .\bin\Main.exe
                 |
                 v
              main()
                 |
                 +--> logger.OpenFile("log\\robot.log")
                 |       |
                 |       +-- failure --> exit(-1)                     [EXIT]
                 |
                 +--> RobotConfig.json exists in working directory?
                 |       +-- yes --> robot.UpdateModelFromFile(...)
                 |
                 +--> while !robot.IsValid()
                 |       prompt for config --> UpdateModelFromFile()
                 |       +-- invalid --> repeat                       [WAIT]
                 |
                 +--> robot.Connect()
                 |       |
                 |       +-- failure / not fully connected
                 |             --> robot.Disconnect()
                 |             --> retry on failure, with pause       [WAIT]
                 |             --> logger.CloseFile() --> return 1    [EXIT]
                 |
                 +--> lastPos.txt exists?
                 |       +-- yes --> UpdatePosFromFile(file, true)
                 |                       |
                 |                       +-- failure
                 |                             --> ShutdownRobotUntilSafe()
                 |                             --> CloseFile()
                 |                             --> return 1            [EXIT]
                 |
                 +--> pause --> MainMenu()
                 |                  |
                 |                  +--> CalibrationMode()
                 |                  +--> OperationMode()
                 |                  +--> confirmed q --> return
                 |                  +--> !IsConnected() --> return    [FAULT]
                 |
                 +--> still robot.IsConnected()?
                 |       +-- yes --> print pose/rails
                 |       |           prompt for save path
                 |       |           --> SavePosToFile(), unless "no"
                 |       +-- no  --> skip saving recovery file
                 |
                 +--> while !robot.Disconnect(): pause, retry          [WAIT]
                 +--> logger.LogInfo("Robot Disconnected.")
                 +--> logger.CloseFile() --> print Bye --> return 0    [EXIT]
```

Paths are relative to the process working directory, not the executable's
directory. start.cmd does not change that directory. The VS Code launch
configuration uses the repository root. Starting inside bin/ selects different
configuration and recovery files. In this checkout root/log/ exists and
bin/log/ does not; a launch with bin/ as the working directory would fail the
first log-file open unless that directory is created there.

If lastPos.txt is absent, startup does not copy homePos into endEffectorPos.
The default software pose is all zeros. Connect() alone does not establish the
EE's measured physical pose.

The runtime FAULT route can ultimately return 0 from main(). Startup failures
return 1, and the first logger failure uses exit(-1). An exit code of zero
therefore does not prove that an operational session was fault-free.

## 2. Complete console menu tree

Each menu loop checks robot.IsConnected() before its next iteration. q in a
submenu normally returns to its caller. A FAULT makes the enclosing menus
return in turn until main() reaches shutdown.

```text
MainMenu() --> PrintMainMenu() --> read selection
 |
 +-- 1 --> CalibrationMode() --> PrintCalibrationMenu()
 |           |
 |           +-- 1  torque prompt --> cable.SetCableTrq()
 |           +-- 2  cable.GetMotorTorqueMeasured(), for every cable
 |           +-- 3  CableMotorControlMode()
 |           +-- 4  UpdatePosFromFile() --> LogPos() --> CheckLimits()
 |           +-- 5  set rotations to 0 --> MoveToParaBlend(3500 ms)
 |           +-- 6  OpenAllRailBrake() --> RailMotorControlMode()
 |           |                             --> CloseAllRailBrake()
 |           +-- 7  GripperControlMode()
 |           +-- 8  UpdateModelFromFile(path, reconnect=true)
 |           +-- 9  PrintEEPos() + PrintRailOffset() + PrintCableLength()
 |           +-- 10 cable.ClearAlert()
 |           +-- 11 RobotControlMode()
 |           +-- 12 UDP get_position --> optional UpdatePos()
 |           +-- q  return to MainMenu()
 |
 +-- 2 --> OperationMode() --> PrintOperationMenu()
 |           |
 |           +-- 1  RobotControlMode()
 |           +-- 2  ReadBrickPosFile() --> brick placement loop
 |           +-- 3  ReadPointFile() --> MoveToParaBlend() per point
 |           +-- 4  ReadTrajFile() --> move to first pose --> RunCableTraj()
 |           +-- 5  cable.GetMotorTorqueMeasured(), for every cable
 |           +-- 6  SavePosToFile()
 |           +-- 7  PrintEEPos() + PrintRailOffset() + PrintCableLength()
 |           +-- 8  ReadBrickPosFile() --> demo placement/return loop
 |           +-- 9  UDP get_brick_position --> auto aiming loop
 |           +-- 10 drone mode --> change z --> two moves to same goal
 |           |                    --> return to MainMenu()
 |           +-- q  return to MainMenu()
 |
 +-- q --> confirm y/q --> return to main()
```

```text
RobotControlMode() --> PrintRobotControlMenu()
 |
 +-- 1 --> RobotPosControlMode() --> PrintRobotPosControlMenu()
 |           |
 |           +-- A/D --> x +/- 0.01 m
 |           +-- W/S --> y +/- 0.01 m
 |           +-- R/F --> z +/- 0.01 m
 |           |           --> MoveToLinear(100 ms, show=false, eBrake=false)
 |           |           --> PrintEEPos() --> Sleep(120 ms) --> repeat
 |           +-- Q --> ClearConsoleInputBuffer() --> return
 |
 +-- 2 --> PrintHomePos() --> MoveToParaBlend(homePos)
 +-- 3 --> pickup pose + 0.21 m z, yaw=0.0141 --> MoveToParaBlend()
 +-- 4 --> GripperControlMode()
 +-- 5 --> OpenAllRailBrake() --> RailControlMode()
 |                                --> CloseAllRailBrake()
 +-- q --> return

GripperControlMode() --> PrintGripperControlMenu()
 +-- 1 Close()    +-- 2 Open()    +-- 3 angle prompt --> Rotate()
 |                                      +-- x SetCalibrateStartAngle()
 |                                      +-- y SetCalibrateEndAngle()
 +-- 4 Release()  +-- 5 Calibrate()  +-- 6 Reset()  +-- 7 Reconnect()
 +-- q --> return

CableMotorControlMode() --> PrintCableMotorControlMenu()
 +-- 1 select configured cable index
 +-- 2 SetCableTrqByIndex()
 +-- 3 relative counts --> MoveSingleMotorCmd(..., absolute=false)
 +-- 4 relative length --> CableMotorLengthToCmd() --> MoveSingleMotorCmd()
 +-- 5 same relative counts on all --> MoveAllMotorCmd(counts, false)
 +-- 6 length converted per cable --> MoveAllMotorCmd(vector, false)
 +-- q --> return
     Options 7-14 are commented out; they are not executable menu actions.

RailMotorControlMode() --> PrintRailMotorControlMenu()
 +-- 1 SelectWorkingMotor()
 +-- 2 selected absolute counts --> MoveSelectedMotorCmdAbsulote()
 +-- 3 selected absolute meters --> RailMotorLengthToCmd() --> same move
 +-- 4 all absolute counts --> MoveMotorCmdAbsuloteById(), for each rail
 +-- 5 all absolute meters --> convert/move each --> update railOffset[]
 +-- 6/7 OpenRailBrakeByIndex() / CloseRailBrakeByIndex()
 +-- 8/9 OpenAllRailBrake() / CloseAllRailBrake()
 +-- q --> return

RailControlMode() --> PrintRailControlMenu()
 +-- 1 SelectWorkingMotor()
 +-- 2 RaiseRailWithCableByLengthAbsulote(selectedRail, selectedRail, meters)
 +-- 3 same coordinated move for each rail, sequentially
 +-- q --> return
```

Manual cable moves do not update the EE pose. Raw rail motor moves are different
from coordinated rail/cable moves. The latter update cable commands to account
for a moving rail attachment.

## 3. Connection and resource ownership

Source: src/Robot.cpp:22, :240; controller Connect() methods.

```text
Robot::Connect()
 |
 +--> existing connection / retained resources / incomplete shutdown?
 |       --> Disconnect() first
 |       +-- failure --> false; keep ownership for another attempt
 |
 +--> online? --> posLogger.OpenFile("log\\pos.log")
 +--> useGripper? --> construct GripperController --> Connect(COM port)
 +--> useRailBraker or useCableBraker?
 |       --> construct BrakeController
 |       --> UseCableBrake(4), if enabled
 |       --> UseRailBrake(railMotorNum), if enabled
 |       --> Connect(railBrakeCommPort)
 +--> useRailMotor?
 |       --> construct RailController --> Connect(851, railMotorNum)
 |       --> GetMotorPosMeasured(out) --> verify result count
 +--> useCableMotor?
 |       --> construct CableController
 |       --> OpenAllCableBrake()
 |       --> cable.Connect(cableMotorNum)
 |       --> if failed with live resources: StopAllMotor()
 |       |      +-- stop failed --> FAULT; retain resources/brake state
 |       --> CloseAllCableBrake()
 |
 +--> mark isConnected=true --> IsConnected() --> ready

Any connection-stage failure
 +--> usually Disconnect() rollback --> return false
 +--> rollback failure --> ownership retained; main() retries shutdown
```

```text
Hardware enabled + isOnline=true  --> real device/API access
Hardware enabled + isOnline=false --> controller can report connected offline
Hardware disabled                --> many commands return true without I/O

Robot::IsConnected()
 --> robot isConnected && !shutdownIncomplete
 --> every enabled gripper/brake/cable/rail controller reports connected
 --> true only if all those software conditions hold

Controller::HasActiveResources()
 --> connected OR cleanupPending
 --> permits another cleanup attempt even after operational readiness is lost
```

The root configuration currently has every use* hardware flag set to false.
The bin/ configuration has useCableMotor=true and the other hardware flags
false. Both have isOnline=true. Consequently, the same executable can run
different hardware workflows depending on its working directory.

## 4. Shutdown and menu unwinding

Source: src/Main.cpp:20, :33; src/Robot.cpp:180, :960.

```text
ShutdownRobotUntilSafe(reason)
 --> LogError(reason) + print reason
 --> robot.Disconnect()
      +-- false --> print CRITICAL --> pause --> retry Disconnect()
      +-- true  --> return to calling function
                     --> caller returns / menu detects !IsConnected()
                     --> enclosing menus return --> main() finishes

RequireAutomatedGripperCommand(result, description)
 +-- true  --> continue automated sequence
 +-- false --> ShutdownRobotUntilSafe(reason) --> return false
```

```text
Robot::Disconnect()
 |
 +--> disconnect gripper, if resources exist
 +--> cable resources exist? --> StopAllMotor()
 |       +-- failure --> mark unavailable + shutdownIncomplete
 |                      retain motor/brake connections --> false
 +--> brake controller connected?
 |       --> CloseAllRailBrake() + CloseAllCableBrake()
 |       +-- failure --> retain motor connections --> false
 +--> cable.Disconnect()
 +--> rail.Disconnect()
 +--> cable AND rail disconnected? --> brake.Disconnect()
 +--> compute shutdownIncomplete from retained resources/results
 +--> mark robot isConnected=false
 +--> cleanup complete? --> posLogger.CloseFile(), if online --> true
 +--> otherwise --> print CRITICAL --> false
```

The brake and stop decisions are based on successful API requests. This code
does not independently confirm a stationary mechanism or a physically engaged
brake before closing device connections.

```text
READY
 +-- invalid input / rejected trajectory --> READY (often return to menu)
 +-- buffered key during trajectory ------> UNAVAILABLE
 +-- pose-motion/calibration fault -------> UNAVAILABLE
 +-- caller forces shutdown -------------> UNAVAILABLE
                                            |
                                            v
                                      Disconnect()
                                       /       \
                                  succeeds     fails
                                     |           |
                                DISCONNECTED   RETAIN RESOURCES
                                     |         pause + retry
                                     v
                                application exit
```

There is no reconnect-and-resume branch in MainMenu() after a FAULT.

## 5. Model and position calibration

Source: src/Robot.cpp:318, :443, :484, :607.

```text
UpdateModelFromFile(path, reconnect)
 +-- reconnect=true --> Disconnect() before reading the file
 |                      +-- failure --> false
 +-- reconnect=false + active controllers --> refuse change --> false
 --> read JSON
 --> validate cable count 1..8 and rail count 1..4
 --> load settings, limits, geometry, hardware flags, scales, network fields
 --> cable.SetTrqLmt(absTrqLmt)
 --> mark model valid
 +-- reconnect=true --> Connect()
 --> success true
 +-- std::exception --> print error --> mark model invalid --> false

UpdatePos(pose)
 --> require a ready robot if online
 --> replace software pose
 --> GetCableLength() --> convert each length to counts
 --> online: cable.CalibrationMotor(), for each cable
     +-- failure --> mark FAULT --> false
 --> print pose and cable counts --> true

UpdatePosFromFile(path, calibration=true)
 --> ready-state check when online and calibrating
 --> read first 6 numeric tokens into endEffectorPos[]
 --> read next 4 into railOffset[] AND startRailOffset[]
 --> GetCableLength()
 --> online + calibration?
 |     --> calibrate cable counters --> calibrate rail counters
 |     +-- failure --> FAULT --> false
 --> print pose, cable counts; verify rail reads if rails enabled
 --> online: posLogger.LogPos() --> true
 +-- missing file / std::exception --> false

UpdatePosFromLog(path)
 --> similar numeric-token reading and motor calibration
 --> leaves startRailOffset[] unchanged
 --> despite its name, does NOT parse timestamped Logger::LogPos() records

CheckLimits()
 --> compare current x/y/z with the six configured min/max entries
 --> true/false only; does not stop a move or validate rotations
```

Calibration changes the motors' interpretation of their position. It is not a
request to move to the pose read from the file. These readers write directly
into live software state while parsing, and do not require exactly ten valid
pose/rail values before calibration. A short file can therefore retain older
values; a parse failure can leave a partially updated software pose.

CalibrationMode() forces shutdown for any false result from its position-file
action, including a missing file. Configuration reload also disconnects first;
a missing replacement file can end the current session even if the previous
model remains valid.

## 6. Pose motion, kinematics, and cable command workers

Source: src/Robot.cpp:762, :930, :1002, :1012, :1148;
src/TrajectoryGenerator.cpp:8, :41; src/CableKinematics.cpp:8;
src/CableController.cpp:202, :242.

```text
MoveToLinear(destination, time, display, eBrake)
 --> optionally disable key-based eBrake for this call
 --> GenLinearTrajForCableMotor(currentPose, destination, time)
 --> RunCableTraj(frames, display)
 --> restore useEBrake=true --> return result

MoveToParaBlend(destination, time, display)
 --> GenParaBlendTrajForCableMotor(currentPose, destination, time)
 --> RunCableTraj(frames, display) --> return result

MoveToParaBlend(destination, display)
 --> duration = XYZ distance / endEffVelLmt * 1000
 --> clamp duration to minimum 1000 ms
 --> GenParaBlendTrajForCableMotor() --> RunCableTraj()
```

```text
GenLinearTrajForCableMotor()
 --> step per millisecond = (end - start) / requested time
 --> sample all 6 pose values at t=0,50,100,... <= time
 --> return frames

GenParaBlendTrajForCableMotor()
 --> time <= 200 ms? --> empty result                      [REJECT]
 --> solve each axis' acceleration/cruise/deceleration coefficients
 --> velocity or acceleration too high? --> empty result  [REJECT]
 --> sample all 6 pose values every 50 ms --> return frames
```

Both generators use a fixed 50 ms interval. Robot's JSON controlFreq setting is
used as an execution interval in milliseconds. Changing it changes playback
timing without changing the generators' sampling interval. For a duration that
is not a multiple of 50 ms, the generated last frame can precede the exact
requested endpoint. Automatic duration uses XYZ distance, even for a move with
substantial rotation.

```text
PoseTrajToCmdTraj(poseFrames)
 --> for every pose: EEPoseToCmd(pose)
                      |
                      v
              EEPoseToCableLength(pose, railOffsets)
                      |
                      v
              ComputeCableLengths(config, pose, rails)
               --> R = Rx(roll) * Ry(pitch) * Rz(yaw)
               --> transform each EE attachment into frame coordinates
               --> compute pulley center and cable tangent/outlet
               --> wrapped arc length + straight cable length
               --> first 4: compensate changed rail offset from start
                      |
                      v
              CableMotorLengthToCmd(cableId, length)
               --> length * individual scale --> int32_t count
```

```text
RunCableTraj(poseFrames)
 |
 +--> require robot.IsConnected()
 +--> reject empty input / frames with fewer than 6 pose values
 +--> PoseTrajToCmdTraj() --> check command frame widths
 +--> compute current software-pose cable counts
 +--> precheck ALL consecutive cable-length changes
 |       +-- any change outside +/-0.5 m --> false          [REJECT]
 |
 +--> for every command frame:
 |       |
 |       +--> eBrake(cableBrake=true, railBrake=false)
 |       |       +-- false --> return false                [FAULT]
 |       |
 |       +--> cable.MoveAllMotorCmd(frame)
 |       |       |
 |       |       +--> validate command count
 |       |       +--> create one std::thread per cable
 |       |       |       --> MoveSingleMotorCmd(index, count, absolute)
 |       |       |            --> TeknicNode::MoveToPosn()
 |       |       |                 --> vendor MovePosnStart()
 |       |       |            --> NodeIsInAlert()?
 |       |       |            --> log alert/mnErr to log.txt; false
 |       |       |            --> catch worker exceptions; record failure
 |       |       +--> join every worker that was created
 |       |       +--> any worker/creation failure?
 |       |                --> StopAllMotor() --> false
 |       |
 |       +-- command failure --> mark robot FAULT --> false
 |       +-- success --> assign frame's pose to endEffectorPos[]
 |       +--> online: posLogger.LogPos()
 |       +--> optional PrintEEPos() and command display
 |       +--> busy-wait for remainder of frame interval
 |
 +--> return true
```

The software pose advances after command calls succeed. There is no per-frame
motor-arrival handshake. Command workers are joined; exceptions inside those
workers and partial thread-creation failures already have explicit handling.
The main menu thread does not have equivalent top-level exception handling.

```text
eBrake(cableBrake, railBrake)
 +-- useEBrake=false --> true
 +-- kbhit()==false  --> true
 +-- ANY buffered key available
       --> cable.StopAllMotor()
       +-- failure --> FAULT --> false; retain brake state
       --> close requested cable/rail brakes
       --> print "Trajectory Stopped!!" --> pause
       --> ALWAYS mark robot unavailable + shutdownIncomplete
       --> false --> enclosing menus unwind --> process exits
```

This is a whole-session termination path, even when the stop requests succeed.
It does not distinguish q, Escape, or other keys. Jogging explicitly disables
this check during its short linear moves.

## 7. Coordinated rail and cable motion

Source: src/Robot.cpp:1170; src/RailController.cpp:12, :79, :121.

```text
RaiseRailWithCableByLengthAbsulote(railId, cableId, targetMeters)
 --> ready state + index checks + finite target
 --> outside 0..2.55 m? --> warning --> CheckContinue()
 --> OpenCableBrakeByIndex(cableId) + OpenRailBrakeByIndex(railId)
     +-- failed open --> close successfully opened brakes
                         markUnsafeShutdownState() --> false       [FAULT]
 --> duration = abs(target-currentRailOffset) / railVelLmt * 1000
     +-- nonfinite / <=0 --> secureBrakes() --> false
     +-- <=200 or >10000 ms --> prompt CheckContinue()
 --> solve cubic rail trajectory
 --> for every sampled rail position:
       --> EEPoseToCmd(same EE pose, temporary rail offsets)[cableId]
       --> RailMotorLengthToCmd(railId, railPosition)
 --> for each rail/cable frame pair:
       --> eBrake(true, true)
       --> rail.MoveMotorCmdAbsuloteById(railId, railCount)
       --> cable.MoveSingleMotorCmd(cableId, cableCount, true)
       +-- either failed --> StopAllMotor()
                             --> secureBrakes() if stop succeeded
                             --> markUnsafeShutdownState() --> false
       --> update software railOffset[railId] --> LogPos()
       --> sleep remaining frame time
 --> Sleep(50 ms) --> secureBrakes() --> return result
```

markUnsafeShutdownState() is a local lambda that makes the session unavailable.
secureBrakes() is a local lambda that closes this cable/rail pair and makes the
session unavailable if either close fails.

```text
RailController::MoveMotorCmdAbsuloteById(logicalId, count)
 --> physicalId = ToPhysicalRailIndex(logicalId) = (logicalId + 2) % 4
 --> ADS WriteReq("MAIN.Axis_GoalPos", count)
 --> ADS WriteReq("MAIN.startMove[physicalId+1]", true)
 --> true/false

MoveSelectedMotorCmdAbsulote(count)
 --> same writes, using the previously mapped workingMotor

CalibrationMotor(index, count)
 --> write Axis_GoalPos
 --> write MAIN.bHomeSwitch[index+1] = true
 --> return; no homeBusy/done handshake

GetMotorPosMeasured(out) / GetMotorPosMeasuredById(index)
 --> read MAIN.actPos[index+1], without logical-to-physical remapping
```

Review the different index mapping used by movement versus calibration/readback.
The coordinated trajectory's temporary rail-offset array starts with all zeros;
only the selected rail's value is filled. Other rails' current offsets are not
copied into this calculation. Also, the raw selected-rail count action stores
counts directly in railOffset[], whose other uses expect meters.

## 8. Cable torque adjustment

Source: src/CableController.cpp:77, :112.

```text
SetCableTrqByIndex(index, target, tolerance)
 --> disabled/offline? --> success without hardware adjustment
 --> check connection and index
 --> SetAccLmt(index, 200) --> GetTrqMeasured()
 --> if current torque > target:
       --> SetVel(index, -10)
       --> read repeatedly while torque > target-tolerance
       +-- buffered key --> StopNode(index) --> break
 --> StopAllMotor() --> SetAccLmt(index, 40000)
 --> return stop result

SetCableTrq(target, tolerance)
 --> SetAccLmt(200) + read torque for every cable
 --> start -10 velocity on cables above target-tolerance
 --> roughly every 50 ms:
       --> GetTrqMeasured() for every cable
       --> below target-tolerance / already stopped? --> StopNode()
       --> LogInfo(torque list)
       --> all stopped? --> finish
       +-- buffered key --> finish
 --> StopAllMotor() --> restore all acceleration limits to 40000
 --> return stop result
```

The menus pass tolerance=4. A configured target of -3.5 therefore uses -7.5 as
the loop's stopping comparison. Neither adjustment has an elapsed-time timeout.
Their direct Teknic reads/writes can throw outside a local exception handler.

SetTrqLmt() only stores a field; the current movement code does not compare
torque against that field. TeknicNode::Connect() separately sets the vendor
global torque limit to 30. The configured absTrqLmt is not applied there.

## 9. Brick placement, demo, point files, and drone mode

Source: src/Main.cpp:1235, :1451, :1465, :1512, :1936;
src/TrajectoryGenerator.cpp:131, :222, :250.

```text
ReadBrickPosFile(path, rotationalAngleOffset, rotationalDistanceOffset)
 --> read line --> split whitespace and comma-containing tokens
 --> stod() numeric tokens
 --> type heading "holes" / "NO" / "half"? --> remember type, skip line
 --> 4 fields: convert [x,y,z,angle] --> [x,y,z,yaw=0,angle]
 --> 5 fields: [x,y,z,yaw,angle]; adjust negative angle by +180
 --> apply rotation-alignment XY offset --> append type
 --> print each result as 6 fields --> return rows

OperationMode option 2
 --> ReadBrickPosFile() --> empty? return to menu
 --> prompt start brick index --> OpenAllCableBrake()
 --> for each brick:
       Rotate(90) --> Open() --> move to pre-pickup (+0.21 m)
       --> descend to pickup --> Close() --> lift (+0.14 m)
       --> move to hard-coded safePt [8.24,6.51,-0.6,0,0,-0.0237]
       --> Rotate(brickAngle+92.2 - yaw*180/pi)
       --> optionally raise to placement clearance --> CheckContinue()
       --> move above placement at z + groundOffset + 0.3 m
       --> descend 0.3 m --> Open() --> lift 0.3 m
       --> return horizontally above safePt if needed, then to safePt
       --> CheckContinue() --> next brick
 --> pause --> return to operation menu if still ready

Each automated gripper command
 --> RequireAutomatedGripperCommand() --> failure forces shutdown
Each pose move
 --> MoveToParaBlend() --> false breaks sequence
     +-- session unavailable --> enclosing menus unwind toward shutdown
```

The fixed sleeps between commands are timing assumptions, not sensor checks.
The close-brakes call at the end of this sequence is commented out.

```text
OperationMode option 8: demo
 --> ReadBrickPosFile() --> start index --> OpenAllCableBrake()
 --> Rotate(90) + Open()
 --> pre-pickup (+0.2 m) --> pickup --> Close() --> lift (+0.14 m)
 --> rotate for placement
 --> above brick (+0.2 m) --> descend --> Open() --> lift
 --> descend again --> Close() --> lift
 --> return to pre-pickup --> descend --> Open() --> lift
 --> MoveToParaBlend(homePos, safeT*2) --> next brick

Demo differences from option 2:
 --> no safePt transit or ground offset addition
 --> placement yaw assignment is commented out
 --> brickType display remains 0 instead of being loaded from the row

ReadPointFile() / ReadTrajFile()
 --> read every line --> stod() for each whitespace-separated token
 --> append row --> return rows (missing file returns empty vector)

OperationMode option 3: points
 --> ReadPointFile() --> CheckContinue()
 --> for each row: MoveToParaBlend(&row[0])

OperationMode option 4: sampled trajectory
 --> ReadTrajFile() --> CheckContinue()
 --> MoveToParaBlend(&rows[0][0])
 --> RunCableTraj(rows)

OperationMode option 10: drone
 --> parse relative length; enforce +/-0.5 m comparison
 --> goal = current pose with z += length
 --> MoveToParaBlend(goal, 3000 ms)
 --> Sleep(800 ms) --> MoveToParaBlend(same goal)
 --> return to MainMenu()
```

Point/trajectory readers are whitespace parsers. A token such as "7.3," can be
partially accepted by stod(), so the sample files with comma-space separators
can appear to work. A compact CSV row with no spaces is not split into six
coordinates. The point file's extra time column is not used by option 3.
In the four-field brick branch, the XY alignment correction uses the inserted
zero yaw; in the five-field branch it uses the gripper angle at column 4.

The point and trajectory callers access row data before RunCableTraj() can
apply its six-value checks. The brick reader prints six fields even when a
malformed row contains fewer. These are CRASH RISK paths, detailed below.

## 10. Position requests and auto aiming

Source: src/Main.cpp:1084, :1722; src/Robot.cpp:832.

```text
CalibrationMode option 12
 --> WSAStartup() --> create UDP socket
 --> server = configured RPiIP:RPiPort
 --> send "get_position" --> recvfrom()                         [WAIT]
 --> cleanupRPiSocketUntilReleased()
 |     --> cleanupRPiSocket(): closesocket() then WSACleanup()
 |     --> failed cleanup: pause, retry
 --> parse JSON --> display x/y/z/roll/pitch/yaw
 --> user selects y? --> extract 6 doubles --> robot.UpdatePos()
     +-- false --> ShutdownRobotUntilSafe()
```

This socket has no receive timeout. Invalid JSON syntax is caught, but numeric
type/missing-value failures during the later pose extraction can escape.

```text
OperationMode option 9
 --> GetCameraOffset() --> WSAStartup() --> UDP socket
 --> ACTUAL destination = 127.0.0.1:5000
     (console prints configured RPiIP:RPiPort instead)
 --> copy current software pose to goal
 --> repeat:
       --> 3 times:
             GetMoveToBrickPos(socket, address, offsets, angle)
             --> set receive timeout to 5000 ms
             --> send "get_brick_position" --> receive JSON
             --> require x/y/z/angle and extract numeric values
             --> offset[0]=JSON y, offset[1]=JSON x,
                 offset[2]=0.5-JSON z
             --> goal += offsets --> MoveToParaBlend(goal)
             --> Sleep(500 ms)
       --> prompt y/n/g/p
           +-- y: another group of three corrections
           +-- n / invalid input: end auto aiming
           +-- g: Open() --> add camera XY offset, z-=0.30
           |      --> move --> Rotate(-angle) --> z-=0.185 --> move
           |      --> Close() --> move home --> end auto aiming
           +-- p: Rotate(-angle)
                  --> choose right/left/top relative to detected brick
                  --> add 0.28 m side offset, or +0.055 m z for top
                  --> add camera XY offset, z-=0.30 --> move
                  --> z-=0.185 --> move --> Open()
                  --> retreat 0.185 m --> move home --> end auto aiming
 --> cleanupAutoAimSocketUntilReleased()
       --> cleanupAutoAimSocket(): closesocket() then WSACleanup()
       --> failed cleanup: pause, retry
```

Every failed pose move or gripper command in this auto-aim branch forces
ShutdownRobotUntilSafe(), then network cleanup, then return. This includes
motion rejected before any motor command, unlike several other menus.
A failed offset request ends auto aiming while leaving the robot session ready.

The model loader reads literal keys "cameraOffset.x" and "cameraOffset.y".
bin/RobotConfig.json supplies a nested cameraOffset object, so those nested
values are not loaded by the current code; defaults are used instead.

## 11. Serial gripper and brake transport

Source: src/COMPortNode.cpp; src/GripperController.cpp;
src/BrakeController.cpp.

```text
GripperController / BrakeController constructor
 --> create COMPortNode(isOnline), or accept injected ISerialPort
 --> null injected port? --> throw invalid_argument

COMPortNode::Connect(port)
 --> release previous handle first
 --> offline? --> mark connected --> true
 --> CreateFile(COM port)
 --> SetSerialParams(): 57600 baud, 8 data bits, no parity, 1 stop bit
                       read/write timeouts + receive mask
 --> Sleep(2000 ms) --> mark connected --> true
 +-- setup failure --> try CloseHandle(); retain failed-to-close handle
                      --> false

COMPortNode::Send(command)
 --> require connected and command.size()==8
 --> offline? --> true
 --> WriteFile(8 bytes) --> require exactly 8 bytes written
 --> true/false; no response read or device completion check

COMPortNode::Read()
 --> up to ~1000 ms deadline; accumulate available bytes
 --> at most 1024 bytes --> return string (error can return partial string)

COMPortNode::Disconnect()
 --> offline / no handle? --> mark disconnected --> true
 --> if connected: Send("(z,    )")
     +-- failed --> retain open port --> false
 --> CloseHandle()
     +-- failed --> retain open port --> false
 --> reset handle and connection flag --> true
```

```text
Gripper::Open()    --> "(o,    )" --+
Gripper::Close()   --> "(c,    )" --+--> Send() three times
Gripper::Release() --> "(r,    )" --+    ALL three writes must succeed
Gripper::Rotate(angle)             |
 --> map logical angle through startAngle/endAngle
 --> format "(d,<number><padding>)"+--> failure: invalidate controller
                                       --> Robot::IsConnected() becomes false

SetCalibrateStartAngle() --> set local temporary start angle
SetCalibrateEndAngle()   --> set local temporary end angle
Calibrate()             --> copy temporary angles to active mapping
                           x/y/w serial sends are commented out
Reset()                 --> Send("(h,  0 )") once
                           failure prints error; does not invalidate controller
Reconnect()             --> Disconnect() --> wait 1 s online
                           --> reconnect same saved COM port

Brake rail index 0..3  --> command index 0..3
Brake cable index 0..3 --> command index 4..7
Open by index          --> "(<index>:0)   " --> Send()
Close by index         --> "(<index>:1)   " --> Send()
Close all rail         --> "(8:1)   " --> Send()
Close all cable        --> "(9:1)   " --> Send()
Open all rail/cable    --> open each configured brake in turn
                          --> any failure: attempt close-all rollback
                          --> return original failure
```

The supplied Arduino gripper firmware handles o/c/r/d/f/x/y/w/p. It has no h or
z case. Thus this source's Reset()/serial disconnect packets do not correspond
to a reset or shutdown action in that firmware. The physical installation may
use a different firmware; this is a source-level protocol mismatch to verify.

## 12. What can make the program quit

These are confirmed code paths and static risks, not a reproduced diagnosis of
the user's latest failure. No robot process or hardware command was run during
this review.

```text
Trigger                         Route                         Outcome
------------------------------  ----------------------------  -----------------
log\\robot.log cannot open       main() --> exit(-1)           immediate EXIT
connection/recovery fails       rollback --> return 1         startup EXIT
any key during trajectory       eBrake() --> FAULT            session ends
one cable pose command fails    MoveAll... --> RunCableTraj    FAULT --> EXIT
rail/brake/gripper failure      caller --> ShutdownRobot...   FAULT --> EXIT
bad calibration file path       false --> ShutdownRobot...    session ends
auto-aim planner rejects move   false --> ShutdownRobot...    session ends
trajectory rejected elsewhere  false; readiness retained     usually menu
shutdown cannot finish          Disconnect() false            pause/retry WAIT
RPi position server silent      recvfrom(), no timeout        potentially WAIT
invalid numeric file token      uncaught stod() exception     CRASH RISK
empty/short trajectory rows     unchecked vector/pointer      CRASH RISK
direct Teknic API exception     no main-thread catch boundary CRASH RISK
```

Review these locations first:

1. **Empty trajectory before validation** -- Main.cpp:1474 uses rows[0][0]
   after ReadTrajFile(), including its empty result on a missing file. Choosing
   Continue reaches the access before RunCableTraj() validates anything.
2. **Short or blank rows before validation** -- Main.cpp:1461 passes &point[0]
   into a generator that reads six values. TrajectoryGenerator.cpp:213 prints
   six brick values regardless of actual row size. Blank/invalid brick rows can
   reach this print. Vector operator[] does not enforce these bounds.
3. **Uncaught numeric conversion** -- ReadPointFile()/ReadTrajFile() do not
   catch stod() failures. ReadBrickPosFile() catches invalid_argument but not
   out_of_range. main() has no catch boundary around these calls.
4. **Uncaught direct Teknic access** -- torque menus, torque adjustment,
   ClearAlert(), and SavePosToFile() call vendor methods outside the command
   workers' catch blocks. A vendor exception here can terminate the process
   without the normal Robot::Disconnect() sequence.
5. **Temporary-string end pointer** -- Main.cpp:1249-1250 and :1527-1528 pass
   (userInput != "" ? userInput : "0").c_str() to strtol(), retain its end
   pointer, then dereference that pointer in the following statement. The
   temporary string has already been destroyed; the dereference is invalid.
6. **Any-key stop ends the whole session** -- Robot.cpp:960-997. A successful
   emergency stop still sets isConnected=false and shutdownIncomplete=true.
   The enclosing menus then return; there is no in-menu resume action.
7. **Input error can force full shutdown** -- Main.cpp:1010 and :1793. The
   calibration file and auto-aim callers do not distinguish a preflight input
   rejection from a fault after motion started.
8. **Working directory changes startup** -- Main.cpp:2021, :2028, :2051.
   Root/bin configurations have different geometry and enabled hardware.
   bin/log/ is absent in this checkout. The bin configuration's RPiIP value
   is also "19268.101.30", which is not a valid IPv4 address.

Further workflow issues relevant to reviewing errors:

```text
Current software pose --> command generation --> physical motion
          ^                                         |
          |                                         |
          +------- measured-pose feedback ---------+   not implemented here
```

- Raw cable commands leave the software pose unchanged. Raw selected-rail count
  moves store counts where other code expects meters. Later trajectories use
  that software state as their starting position.
- CheckLimits() is only called after one calibration action. RunCableTraj()
  does not enforce the configured workspace box, nor a finite-pose check.
  Kinematics can divide by zero or produce invalid acos/sqrt inputs in singular
  geometry; conversion to integer motor counts has no finite/range guard.
- Cable brakes are closed during Connect(). Most pose/torque menu paths have
  their open/close calls commented out, while brick/demo explicitly open them.
  Review this sequencing if cable brakes are enabled in the deployed config.
- SavePosToFile() has no checked write result and reads hardware after opening
  the output file. A read failure can leave a partially rewritten recovery
  file. Position loading is also incremental rather than transactional.
- Logger::Log() writes tm_year without adding 1900, so 2026 appears as 126.
  Logger's pose labels exchange roll and yaw relative to Robot. LogPos() writes
  EE coordinates again as rail/cable offsets instead of using railPos[].
  These defects reduce the logs' value for diagnosing a fault.
- The existing log.txt contains repeated MovePosnStart failures with
  0x8004010A. The bundled vendor header defines this as MN_ERR_CMD_MV_SHUTDOWN,
  "Motion attempt / drive in shutdown" (Dependencies/sFoundation20/inc/
  mnErrors.h:150). This identifies the rejected drive state, not what first
  caused that shutdown.
- The inspected robot/position logs were last modified in July 2026, and
  log.txt records only times for those motor failures. They cannot establish
  that the recent failure frequency has increased or that the deployed binary
  matches this source.

The f839c1d controller change explicitly propagates more failed commands and
ends sessions after uncertain motion/calibration. That can explain more
visible controlled exits after an update. It does not prove that the hardware
is producing more faults.

## 13. Function-by-function index for all src/ definitions

The workflows above explain the multi-step paths. This index includes small
wrappers, getters, display functions, overloads, and companion executable
functions so that every source function has a review entry. Numbers identify
the function's first source line. Names retain the source's spelling.

```text
src/Main.cpp -- 23 definitions
  main [2020] ----------------------------> startup, menus, save, shutdown
  MainMenu [1996] -------------------------> mode dispatch/readiness gate
  PrintMainMenu [1987] --------------------> clear console and print options
  CalibrationMode [956] -------------------> 12 calibration/diagnostic actions
  PrintCalibrationMenu [937] --------------> print those options
  OperationMode [1226] --------------------> 10 operation actions
  PrintOperationMenu [1195] ---------------> print those options
  RobotControlMode [885] ------------------> jogging/home/pickup/gripper/rails
  PrintRobotControlMenu [873] -------------> print those options
  RobotPosControlMode [688] ---------------> keyboard 0.01 m jogging loop
  PrintRobotPosControlMenu [674] ----------> print keys and warning
  CableMotorControlMode [196] -------------> raw cable selection/torque/moves
  PrintCableMotorControlMenu [173] --------> print selected cable/options
  RailMotorControlMode [488] --------------> raw rail moves and brake actions
  PrintRailMotorControlMenu [470] ---------> print selected rail/options
  RailControlMode [767] -------------------> coordinated cable/rail moves
  PrintRailControlMenu [756] --------------> print selected rail/options
  GripperControlMode [96] -----------------> gripper actions/calibration/reset
  PrintGripperControlMenu [82] ------------> print those options
  DrawBricksASCII [45] --------------------> draw holes/normal/half brick
  ClearConsoleInputBuffer [69] ------------> ReadConsoleInput(), free buffer
  ShutdownRobotUntilSafe [20] -------------> log error, retry Disconnect()
  RequireAutomatedGripperCommand [33] -----> continue or force shutdown

src/Robot.cpp -- 52 definitions
  Robot() [14] ----------------------------> defaults/member construction
  Robot(configPath) [17] ------------------> UpdateModelFromFile()
  Connect [22] ----------------------------> staged enabled-controller setup
  Disconnect [180] ------------------------> stop/close/disable/release/retry
  IsConnected [240] -----------------------> robot and enabled-controller flags
  IsValid [250] ---------------------------> model-valid flag
  CheckLimits [252] -----------------------> current XYZ box comparison
  UpdateModelFromFile [318] ---------------> JSON model load; optional reconnect
  UpdatePos [443] -------------------------> replace pose, calibrate cables
  UpdatePosFromFile [484] -----------------> load pose/rail numbers, calibrate
  UpdatePosFromLog [607] ------------------> numeric-file variant, not log parser
  SavePosToFile [1341] --------------------> online pose/rail/count file output
  PrintEEPos [264] ------------------------> display six pose coordinates
  PrintBrickPickUpPos [273] ---------------> display pickup; repeats element 0
  PrintRailOffset [282] -------------------> display software rail offsets
  PrintCableLength [291] ------------------> calculate/display cable lengths
  PrintHomePos [308] ----------------------> display home coordinates
  PrintRobotConfig [726] ------------------> display settings and geometry poses
  EEPoseToCableLength(array,rails) [762] ---> build config --> ComputeCableLengths()
  EEPoseToCableLength(array) [790] ---------> use current railOffset[]
  EEPoseToCableLength(vector) [795] --------> use current railOffset[]
  EEPoseToCableLength(vector,rails) [800] --> get vector data --> array overload
  GetEEToGroundOffset [806] ---------------> stored vertical ground offset
  GetCameraOffset [808] -------------------> pointer to stored XY camera offsets
  GetWorkingTrq [810] ---------------------> stored target torque
  GetAbsTrqLmt [812] ----------------------> stored absolute torque limit
  GetCableMotorNum [814] ------------------> stored cable count
  GetRailMotorNum [816] -------------------> stored rail count
  GetGripperCommPort [818] ----------------> stored gripper COM port
  GetRailBrakeCommPort [820] --------------> stored brake COM port
  GetCableBrakeCommPort [822] -------------> field not assigned by model loader
  GetCableMotorScale(id) [824] ------------> stored individual scale; no ID check
  GetRailMotorScale [826] -----------------> stored rail scale
  GetEffVelLmt [828] ----------------------> stored EE velocity limit
  GetCableMotorBrakeNum [830] -------------> stored cable brake count (default 4)
  GetMoveToBrickPos [832] -----------------> UDP request/parse/coordinate offsets
  CableMotorLengthToCmd [905] -------------> meters * cable scale --> int32_t
  CableMotorCmdToLength [910] -------------> counts / cable scale --> meters
  RailMotorLengthToCmd [915] --------------> meters * rail scale --> int32_t
  RailMotorCmdToLength [920] --------------> counts / rail scale --> meters
  GetCableLength [925] --------------------> EEPoseToCableLength(current pose)
  EEPoseToCmd(vector) [930] ---------------> use current railOffset[]
  EEPoseToCmd(vector,rails) [935] ---------> vector data --> array overload
  EEPoseToCmd(array) [941] ----------------> use current railOffset[]
  EEPoseToCmd(array,rails) [946] ----------> lengths --> counts per cable
  eBrake [960] ---------------------------> any-key stop/brakes; invalidate session
  PoseTrajToCmdTraj [1002] ----------------> EEPoseToCmd() for each frame
  RunCableTraj [1012] ---------------------> validate/convert/stream/update/log
  MoveToLinear [1148] ---------------------> linear generator --> RunCableTraj()
  MoveToParaBlend(dest,time,show) [1157] ---> blend generator --> RunCableTraj()
  MoveToParaBlend(dest,show) [1162] --------> distance-based duration --> same
  RaiseRailWithCableByLengthAbsulote [1170] > cubic coordinated rail/cable move

src/CableKinematics.cpp -- 1 definition
  cu_brick::ComputeCableLengths [8] -------> pose/geometry/rails --> cable lengths
                                             cable count >8 throws out_of_range

src/TrajectoryGenerator.cpp -- 5 definitions
  GenLinearTrajForCableMotor [8] ----------> six-axis linear samples at 50 ms
  GenParaBlendTrajForCableMotor [41] ------> blend coefficients/limits --> samples
  ReadBrickPosFile [131] ------------------> parse brick rows/type/alignment
  ReadTrajFile [222] ----------------------> whitespace numeric rows
  ReadPointFile [250] ---------------------> whitespace numeric rows

src/CableController.cpp -- 17 definitions
  CableController [10] -------------------> configure flags and Teknic node
  IsValidCableIndex [13] ------------------> index in [0,cableNumber)
  Connect [17] ----------------------------> count check/log/node setup
  Disconnect [55] ------------------------> node cleanup/log close; retain on fail
  IsConnected [71] -----------------------> readiness flag
  HasActiveResources [73] -----------------> readiness OR cleanupPending
  SetCableTrqByIndex [77] -----------------> one cable tension loop --> stop all
  SetCableTrq [112] -----------------------> all cable tension loop --> stop all
  SetTrqLmt [179] -------------------------> store limit only
  ClearAlert [183] ------------------------> TeknicNode::ClearAlert()
  StopAllMotor [189] ----------------------> TeknicNode::StopAll(); fail invalidates
  MoveSingleMotorCmd [202] ----------------> move/check alert; catch/log mnErr
  MoveAllMotorCmd(vector,bool) [242] ------> worker per cable; join/stop on fail
  MoveAllMotorCmd(count,bool) [295] -------> repeat count --> vector overload
  CalibrationMotor [301] ------------------> SetCurrentPosn(); catch mnErr
  GetMotorPosMeasured [321] ---------------> GetCurrentPosn(), or 0 if unavailable
  GetMotorTorqueMeasured [330] ------------> GetTrqMeasured(), or 0 if unavailable

src/TeknicNode.cpp -- 18 definitions
  TeknicNode [6] -------------------------> store online flag
  IsValidNodeIndex [8] --------------------> check against actual nodeList size
  CleanupConnection [12] -----------------> stop + disable nodes --> PortsClose()
                                             retain connection state on failure
  Connect [71] ----------------------------> discover hubs --> open ports
                                             --> verify exact motor count
                                             --> clear stops/alerts, enable
                                             --> set limits; wait <=2 s/node
  Disconnect [184] -----------------------> CleanupConnection()
  IsConnected [188] ----------------------> connection/resource flag
  GetNodeList [192] -----------------------> copy acquired node pointers
  GetNode [196] ---------------------------> indexed pointer, or nullptr
  SetAccLmt [202] -------------------------> vendor Motion.AccLimit
  GetTrqMeasured [211] --------------------> vendor Motion.TrqMeasured.Value()
  SetVel [220] ----------------------------> vendor Motion.MoveVelStart()
  StopNode [229] --------------------------> vendor abrupt NodeStop()
  StopAll [238] ---------------------------> stop each open port's nodes
                                             or retained individual nodes
                                             --> catch mnErr; aggregate success
  ClearAlert [296] ------------------------> EnableReq(true), AlertsClear(),
                                             NodeStopClear() per acquired node
  MoveToPosn [305] ------------------------> vendor MovePosnStart()
  NodeIsInAlert [314] ---------------------> vendor alert register; invalid ID=true
  SetCurrentPosn [323] --------------------> Refresh() --> AddToPosition(delta)
  GetCurrentPosn [333] ---------------------> measured position Value()

src/RailController.cpp -- 14 definitions
  RailController [5] ----------------------> configure flags and ADS node
  IsValidRailIndex [8] --------------------> index in [0,railNumber)
  ToPhysicalRailIndex [12] ----------------> (logicalIndex+2)%4
  Connect [17] ----------------------------> count check --> ADS Connect()
  Disconnect [46] -------------------------> ADS Disconnect(); retain on failure
  IsConnected [60] ------------------------> readiness flag
  HasActiveResources [64] -----------------> readiness OR cleanupPending
  SelectWorkingMotor [68] -----------------> validate and map selected index
  MoveMotorCmdAbsuloteById [79] -----------> finite check; goal/start ADS writes
  MoveSelectedMotorCmdAbsulote [103] ------> goal/start writes using workingMotor
  CalibrationMotor [121] ------------------> goal/home-switch writes; no handshake
  GetMotorPosMeasured(out) [146] ----------> read every actPos; aggregate success
  GetMotorPosMeasured() [168] -------------> out overload; discard bool result
  GetMotorPosMeasuredById [174] -----------> one actPos, or 0 on failure

src/TwincatADSNode.cpp -- 11 definitions
  TwincatADSNode [4] ----------------------> store online flag
  TryGetHandle [6] -----------------------> find resolved symbol; false if missing
  Connect [16] ----------------------------> open ADS client port/get address
                                             --> target PLC port
                                             --> resolve 17 symbols
                                             --> resolve/write MAIN.power=true
                                             --> release power symbol handle
                                             --> rollback on failed stage
  Disconnect [147] -----------------------> resolve/write MAIN.power=false
                                             --> release handles --> AdsPortClose()
                                             --> retain connection if unsafe
  IsConnected [248] ----------------------> connection/resource flag
  WriteReq(name,double) [252] -------------> require handle --> AdsSyncWriteReq()
  WriteReq(name,bool) [272] ---------------> same, bool-sized request
  ReadReq(name,bool&) [292] ---------------> handle --> AdsSyncReadReq()
  ReadReq(name,double&) [312] -------------> same, double-sized request
  ReadReq(name,int&) [332] ----------------> same, int-sized request
  ReadReq(name,float&) [352] --------------> same, float-sized request
     All ReadReq/WriteReq methods return bool; offline calls skip actual I/O.

src/GripperController.cpp -- 15 definitions
  GripperController(flags) [7] ------------> create COMPortNode --> other ctor
  GripperController(flags,port) [14] ------> own ISerialPort; reject nullptr
  Connect [25] ----------------------------> release old resources; serial Connect()
  Disconnect [47] -------------------------> serial Disconnect(); retain on fail
  IsConnected [61] ------------------------> readiness flag
  HasActiveResources [63] -----------------> readiness OR cleanupPending
  Open [67] -------------------------------> 3 open writes; failure invalidates
  Close [85] ------------------------------> 3 close writes; failure invalidates
  Release [102] ---------------------------> 3 release writes; failure invalidates
  Rotate [120] ----------------------------> angle mapping/format; 3 rotate writes
  SetCalibrateStartAngle [146] ------------> store temporary start mapping angle
  SetCalibrateEndAngle [163] --------------> store temporary end mapping angle
  Calibrate [181] -------------------------> activate local angle mapping
  Reset [196] -----------------------------> one reset packet; void result
  Reconnect [206] -------------------------> disconnect/wait/reopen saved port

src/BrakeController.cpp -- 18 definitions
  BrakeController(online) [10] ------------> create COMPortNode --> other ctor
  BrakeController(online,port) [16] -------> own ISerialPort; reject nullptr
  IsValidRailBrakeIndex [24] --------------> configured rail brake bounds
  IsValidCableBrakeIndex [28] -------------> configured cable brake bounds
  Connect [32] ----------------------------> serial connect if any brakes enabled
  Disconnect [54] -------------------------> serial disconnect; retain on fail
  UseCableBrake [69] ----------------------> validate 0..4; set count/enabled flag
  UseRailBrake [81] -----------------------> validate 0..4; set count/enabled flag
  IsConnected [93] ------------------------> readiness flag
  HasActiveResources [95] -----------------> readiness OR cleanupPending
  OpenRailBrakeByIndex [99] ---------------> validate --> format/send rail open
  CloseRailBrakeByIndex [118] -------------> validate --> format/send rail close
  OpenAllRailBrake [137] ------------------> open each; close-all rollback on fail
  CloseAllRailBrake [159] -----------------> group-8 close packet
  OpenCableBrakeByIndex [174] -------------> validate --> format/send cable open
  CloseCableBrakeByIndex [193] ------------> validate --> format/send cable close
  OpenAllCableBrake [212] -----------------> open each; close-all rollback on fail
  CloseAllCableBrake [234] ----------------> group-9 close packet

src/COMPortNode.cpp -- 8 definitions
  COMPortNode [6] -------------------------> store online flag; invalid handle
  SetSerialParams [8] ---------------------> baud/framing/timeouts/receive mask
  Connect [50] ----------------------------> CreateFile/setup/wait; rollback
  Disconnect [92] -------------------------> send z packet/CloseHandle; retryable
  IsConnected [118] -----------------------> readiness flag
  HasActiveResources [122] ----------------> ready OR live serial handle
  Send [127] ------------------------------> exactly 8 bytes --> checked WriteFile()
  Read [157] ------------------------------> deadline/length-bounded serial read

include/SerialPort.h -- transport interface
  ~ISerialPort() --------------------------> default virtual destructor
  Connect/Disconnect/IsConnected/HasActiveResources/Send/Read
    --> pure virtual contract; implemented by COMPortNode or test fake

src/Logger.cpp -- 9 definitions
  Logger [3] ------------------------------> default stream construction
  OpenFile [5] ----------------------------> append-mode stream; return good()
  NewFile [17] ----------------------------> overwrite-mode stream; return good()
  CloseFile [31] --------------------------> close stream; print filename
  Log [36] --------------------------------> timestamp + message + newline
  LogError [43] ---------------------------> prefix [Error] --> Log()
  LogWarning [47] -------------------------> prefix [Warning] --> Log()
  LogInfo [51] ----------------------------> prefix [Info] --> Log()
  LogPos [55] -----------------------------> format pose/offset text --> Log()

src/utils.cpp -- 1 definition
  CheckContinue [20] ----------------------> getline loop: empty/y/Y=true, n/N=false
                                             other text reprompts
```

### Companion executables and the unused Delta client

Main links DeltaClient.cpp, but Robot owns RailController and no Main/Robot
function constructs or calls DeltaClient. GUI and DeltaServer are separate
build targets.

```text
src/GUI.cpp -- 11 definitions
  MyApp::OnInit [28] ----------------------> new MyFrame() --> Show(true)
  MyFrame::MyFrame [34] -------------------> build tabs/status/log widgets
                                             --> bind close and button events
  MyFrame::OnClose [181] ------------------> Destroy(); immediate return
                                             confirmation code is unreachable
  OnSetCablesTorque [200] -----------------> new empty "Cable Torque Control" frame
  OnControlCableMotor [208] ---------------> new empty "Cable Motor Control" frame
  OnControlLinearRailMotor [216] ----------> new empty rail control frame
  OnControlRobot [224] --------------------> new empty robot control frame
  OnRequestCurrentPositionFromRPi [232] ---> new empty position request frame
  OnUpdateRobotPosFromFile [240] ----------> new empty position-file frame
  OnUpdateRobotConfigFromFile [248] -------> new empty configuration frame
  OnClearException [256] ------------------> new empty clear-exception frame
     These handlers have no robot controller calls and do not Show() the frames.
     GUI status values are constants; they are not live robot telemetry.

src/DeltaClient.cpp -- 11 definitions
  DeltaClient [4] -------------------------> store flags; server=192.168.0.1:12345
  ~DeltaClient [12] -----------------------> Disconnect(); ignores return result
  EnsureSocket [16] -----------------------> WSAStartup/socket/1000 ms recv timeout
  ReleaseSocketResources [51] -------------> closesocket() --> WSACleanup()
  Connect [69] ----------------------------> validate count; send plain "connect"
                                             --> wait for any datagram --> ready
  Disconnect [115] -----------------------> ReleaseSocketResources(); unavailable
  IsConnected [121] -----------------------> cached flag
  SelectWorkingMotor [123] ----------------> bounds check; store selected index
  MoveSelectedMotorCmd [132] --------------> require absolute; send plain "move"
                                             no index/count payload or reply wait
  CalibrationMotor [153] ------------------> print "not implemented"
  GetMotorPosMeasured [157] ---------------> print "not implemented"; empty vector

src/DeltaServer.cpp -- 5 definitions
  main [120] ------------------------------> construct server(12345) --> start()
                                             catch std::exception --> return 1
  DeltaServer::DeltaServer [22] -----------> WSAStartup/socket/bind; throw on fail
  DeltaServer::~DeltaServer [49] ----------> closesocket() + WSACleanup()
  DeltaServer::start [54] ------------------> RailController(false,true).Connect(851,1)
                                             then UDP receive/parse loop
                                             rail setup failure --> exit(-1)
  DeltaServer::handle_json_message [95] ---> read index/command
                                             command=0: read position, rail move
                                             catch extraction std::exception

DeltaClient plain "connect"/"move"
                --> DeltaServer expects JSON {index,command,position}
                --> parse error; server sends no acknowledgement
                --> client's Connect() times out
```

DeltaServer constructs its rail controller with isOnline=false, so its current
rail actions do not reach hardware. It also does not call rail.Disconnect() in
its destructor. This companion path is not used by the console robot workflow.

## 14. Python camera helpers and Arduino firmware

These are separate programs. main.exe neither launches nor supervises them.
The two vision servers both default to UDP port 5000; on the same host/interface
they cannot normally bind that port simultaneously.

```text
arduino/Arucomarker/ArUcoDetector.py (position service)
  startup --> UDP daemon thread + load camera/home data + capture loop
  load_camera_data() ------> np.load() --> camera matrix/distortion coefficients
  load_home_position() ----> read saved home pose/rotation JSON
  detect_aruco_markers() --> detect board --> estimatePoseBoard()
                            --> invert camera transform
                            --> apply_low_pass_filter() to position/rotation
                            --> SVD rotation correction
                            --> relative home transform --> real pose globals
  apply_low_pass_filter() -> append history --> mean of last 10 samples
  save_home_position() ----> write reference position/rotation JSON on h key
  get_current_position() --> globals --> x/y/z/roll/pitch/yaw/distance object
  run_udp_server() --------> "get_position" --> get_current_position()
                            --> JSON reply, or "No position data available"
  capture loop ------------> display; h save, l load, q exit --> release camera
     Published data has no age check; lost tracking can leave the previous pose.

camera/ArUcoDetector.py (display-only variant)
  load_camera_data() ------> load calibration NPZ
  apply_low_pass_filter() -> history mean
  detect_aruco_markers() --> board pose --> filtered camera/home-relative display
  save_home_position() ----> write JSON
  load_home_position() ----> read JSON; missing file returns false
  top-level loop ----------> camera index 1; detect/display; h/l/q keys
     This version has no get_current_position()/run_udp_server() functions.

camera/yoloe-test.py (brick service)
  startup --> load OpenVINO model; fallback load/export YOLO model
          --> WebcamVideoStream().start() + UDP daemon thread
  main loop --> camera.read() --> model.predict() --> each mask
                --> get_simple_mask()/get_center()/get_dist()/get_orientation()
                --> choose brick nearest image center
                --> saved={x,y,z,angle} --> draw/display --> q ends loop
  get_current_position() -> return saved detection or None
  run_udp_server() -------> "get_brick_position" --> JSON detection reply
     saved is not cleared when detections disappear, so old data can be served.

camera/camera.py -- WebcamVideoStream methods
  __init__() -------------> open camera; configure MJPG/resolution; fail raises
  start() ----------------> daemon thread running update(); return self
  update() ---------------> repeatedly read camera --> replace latest frame
  read(raw=false) --------> wait for first frame --> optional horizontal flip
                            --> raw frame, or square crop/resize to 640x640
  set_flip_horizontal() --> set flip flag
  stop() -----------------> set stopped flag --> release capture device
     read() has no timeout while waiting for the first successful frame.

camera/utils.py
  draw_contour() ---------> outline mask on image
  draw_point() -----------> small circle on image
  draw_bounding_box() ----> rectangle on image
  draw_text() ------------> text on image
  get_center() -----------> moments --> centroid, or None
  get_orientation() -----> minimum-area rectangle --> adjusted angle
  get_simple_mask() -----> contour perimeter --> polygon approximation
  get_longest_edge() ----> largest adjacent contour-point distance
  get_area() -------------> contour area
  get_dist() -------------> sqrt(referenceArea / detectedArea)
```

The following three scripts are byte-identical between camera/ and
arduino/Arucomarker/ in this checkout. Every defined function is listed here.

```text
CameraCalibration.py
  calibrate_camera_online()
    --> capture chessboard --> detect/refine corners
    --> a key accepts a frame (collect up to 10); q ends collection
    --> calibrateCamera() --> save camera_matrix/dist_coeffs NPZ
    --> return those arrays
  top-level example --> invoke calibration and print result

OnScreenArUco.py
  generate_aruco_marker()
    --> physical monitor sizing --> marker pixels --> PDF + onscreen display
  generate_aruco_board()
    --> physical sizing --> GridBoard --> board image --> PDF + display
  top-level example --> generate_aruco_board(3,4,...)

OnScreenChessboard.py
  draw_chessboard() ------> physical monitor sizing --> alternating square image
  save_to_pdf() ---------> convert colors --> physical page size --> PDF
  main() ----------------> draw_chessboard() --> save_to_pdf() --> display

dxfExtract.py
  top-level file picker --> require DXF --> extract measured POINT entities
    --> plane fit/SVD --> EE center offset --> calculate orientation --> print
  midpoint3D() ----------> average XYZ point coordinates
  projectPlane() --------> subtract vector component along plane normal
  invalid extension/read/structure --> explicit sys.exit()

drawBricks.py
  top-level CSV load --> blank image --> draw_bricks() --> display
  draw_bricks() ---------> scale each XY --> rotated rectangle contour

brickDrawer.py
  main() ----------------> CSV via pandas --> ranges/canvas
                          --> scale rows --> draw_brick() --> display
  draw_brick() ----------> rotate rectangle --> fill/border/index label

arduino/Gripper/Gripper.ino
  setup()
    --> initialize pins/Bluetooth UART/Dynamixel protocol
    --> ping motors --> set current/extended-position operating modes
    --> torque on --> read angle mapping values from EEPROM
  loop()
    --> wait for serial --> collect packet --> parse command/angle
    +-- o/c/r --> goal current for open/close/release
    +-- d ----> map angle --> rotation motor goal position
    +-- f ----> rotate to finish position --> wait 10 s --> torque off
    +-- x/y --> store temporary degree/current-position mapping endpoint
    +-- w ----> activate mapping --> write EEPROM values
    +-- p ----> print rotation position
    --> wait for another serial arrival --> clear packet strings --> repeat
```

## 15. Build helpers and tests

```text
tools/build.ps1
  Resolve-VsDevCmd()
    --> explicit -VsDevCmd, environment override, VSINSTALLDIR, then vswhere
    --> Assert-FileExists() --> developer command path, or throw
  Get-FullPath() ----------> root relative paths or normalize absolute paths
  Assert-FileExists() -----> missing leaf file --> throw
  Quote-CommandArgument() -> reject quote-containing value --> quoted argument
  top-level workflow
    --> choose Main / GUI / DeltaServer / LegacyTest / OfflineTests
    --> resolve/check source/library/include paths
    --> create output/object directories
    --> VsDevCmd.bat -arch=x86 --> cl.exe /EHsc /std:c++14
    --> build failed? exit compiler code
    --> -Run? only permitted for OfflineTests --> execute test binary

test/run_offline_tests.ps1
  top-level --> build.ps1 -Target OfflineTests
            --> add -Run unless -BuildOnly --> propagate exit code

test/offline/OfflineTests.cpp
  main() --> TestRunner::Run() for each of the eight test functions below
          --> zero failures: return 0; otherwise return 1
  TestRunner::Expect() -----> boolean assertion; record failure
  TestRunner::ExpectEqual() > equality assertion; record failure
  TestRunner::ExpectNear() -> tolerance assertion; record failure
  TestRunner::Failures() --> failure count
  TestRunner::Run() -------> capture output --> call test --> catch exceptions
                            --> restore streams --> print PASS/FAIL
  FakeSerialPort::NextResult() --> consume queued result, default true
  FakeSerialPort::Connect() --> track port/calls and fake owned-resource state
  FakeSerialPort::Disconnect() --> consume result; release state on success
  FakeSerialPort::IsConnected() --> fake ready flag
  FakeSerialPort::HasActiveResources() --> fake ownership flag
  FakeSerialPort::Read() --> configured string
  FakeSerialPort::Send() --> record packet; consume configured success/failure
  MakeFakeSerial() -------> construct owned fake and expose pointer to test
  TestOfflineCOMPortLifecycle() ----------> offline COM connect/send/disconnect
  TestGripperCommandEncoding() -----------> packet/angle mapping assertions
  TestGripperFailureAndCleanupRetry() ----> command failure + shutdown retry
  TestFailedConnectRetainsCleanupState() -> failure retains resources for retry
  TestBrakeBoundsAndPartialOpenRollback() > index checks + rollback packets
  TestLinearTrajectory() -----------------> sample counts/coordinates
  TestParabolicTrajectory() -------------> blending/endpoints/rejected timing
  TestCableKinematics() -----------------> geometry/rail effects/count bounds
     No COM, ADS, Teknic, or network hardware is opened by this test target.
     It does not test the main menu's file parsing and pre-validation accesses.

test/test.cpp (LegacyTest target; accesses hardware)
  main()
    --> logger.NewFile() --> legacy motorNode.Connect(1) --> GetNode(0)
    --> velocity +40 until key --> stop
    --> velocity -20 until torque threshold/key --> stop
    --> monitor torque until key --> Disconnect() --> logger.CloseFile()

test/TeknicNode.cpp -- separate legacy implementations
  TeknicNode() -----------> store flag
  Connect() -------------> discover/open/enumerate/enable/set limits
                            --> ready wait; catch mnErr
  Disconnect() ----------> disable acquired nodes --> close ports; catch mnErr
  GetNodeList() ---------> copy node pointers
  GetNode() -------------> checked indexed node pointer or nullptr

test/Logger.cpp -- duplicate legacy implementations
  Logger/OpenFile/NewFile/CloseFile/Log/LogError/LogWarning/LogInfo/LogPos
    --> same workflows as src/Logger.cpp above
```

Suggested review order: sections 1, 2, 4, and 6 for the main control flow;
section 12 for unexpected exits; then use the function index to inspect the
specific controller or helper behind the action that failed.
