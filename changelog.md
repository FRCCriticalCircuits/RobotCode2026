# Format
date - post `commit ID (short version)`

# Circumstance
Verified Telop Swerve/Simulation Swerve
SysID Untested

# Todo
- Implement Pathplanner
- Later will set the default option of `autoChooser` to regular auto

## 2026/1/22 - post `365f8a1`
- FF constants (temporary removed)
- deadzone for a buggy Xbox controller  
(Will write code to have speed mapping later)

## 2026/1/22 - post `b61a4ca`
`GlobalConstants.SYS_ID` to enable/disable SysID and  
CTRE logging for SysID
Publish autoChoosers to NT4

## 2026/1/22 - post `69002c4`
PID Tunning

## 2026/1/18 - post `49291d6`
Telemetry with AdvantageKit

## 2026/1/18 - post `74418fc`
Removed Transcope Notation in comments

## 2026/1/18 - post `ace5ace`
- Change MaxSpeed to actual MaxSpeed
- Removed `Point` Request Stuff  
(Making Modules facing the direction of joystick inputs) 
- Remove `Brake` Request & Keybinding
- Telemetry  
    &emsp;removed:  
    &emsp;&emsp;Mechanism2d Swerve Outputs  
    &emsp;&emsp;raw ChassisSpeeds and ModulePositions   
    &emsp;keeping:  
        &emsp;&emsp;Field Pose2D  
        &emsp;&emsp;Module States(Current/Target)  
        &emsp;&emsp;Odometry Period
- Removed template auto
- Moved SysID routines into an AutoChooser
Rotation by default for safety
- marked `m_sysIdRoutineSteer` as unused 
we only need chassis Rotation/Translation Ka/Kv for MOI estimate
- Removed Replay (Using REV & AKit is better & Replay didnt work offseason)
- Signal Logger log stuff to the usb memory

