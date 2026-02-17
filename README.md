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
Dot is the intake side
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
Climber: 60