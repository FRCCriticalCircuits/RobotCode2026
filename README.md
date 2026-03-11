# Robot Code 2026

[Changelog](./changelog.md)

# Clone
```bash
git clone https://github.com/FRCCriticalCircuits/RobotCode2026.git
```
# vscode settings
disable `detect indentation`
enable `git: auto fetch`

# Controls
Create Joystick Axis Config with `./AxisConfigTool/AxisProfile.html`
and save files to `./src/main/deploy/axis_configs`

# Pathplanner Autos
The Dot(Heading) is the intake side
Intake against the hub

# CAN IDs
Cancoder
3, 6, 9, 12
Drive
1, 4, 7, 10
Turn
2, 5, 8, 11
FL, FR, RL, RR
Pigeon: 20

Intake:
    Arm: 30
    Secondary Arm: 31
    Roller: 32
Shooter:
    Hood: 40
    Flywheel #1: 41
    Flywheel #2: 42
Hopper: 50
Climber: 51
pdh 60

# 2026/2/19 - 2/21 drive tests
"// TODO drivetest", changes will be revert later

# Units / SysID
all units on software level should be in radians,
so divide them with 2PI when using SysID
all units ctre control request get or encoder returns should be in rotations.
climber is the only REV product we got and we dont care its position/velocity.