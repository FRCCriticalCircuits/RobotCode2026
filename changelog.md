# Format
date - post `commit ID (short version)`

# Symbols
(?) -> TBD  
(!) -> Untested Code  
(+) -> Tested  
(+ SIM) -> Tested in simulation

# Todo
- Single Drive Command File for robot-rel support 
with trigger for more percise velocity control

## 2026/2/02 - post `39002e2`
ShooterIOKraken (!)

## 2026/1/30 - post `80238d7`
Subsystem Draft

## 2026/1/29 - post `1616386`(main) at `d8cde9f`
- Simulation Implements(Intake/Hopper/Shooter)

## 2026/1/27 - post `8d3faf3`
ShooterIO & CAD visualization (!) (+ SIM)
will modify later for actual robot

## 2026/1/25 - post `7412eb0`
Separate Drive Command WIP (!)

## 2026/1/24 - post `74848db`
- Test Auto (+ SIM) 
- Add Orientation stuff back

## 2026/1/23 - post `dad2cb9`
- Implement Pathplanner (+ SIM)

## 2026/1/23 - post `708f62f`
- Refine AutoDrive codes

## 2026/1/23 - post `b346901`
- Singleton for `CommandSwerveDriveTrain`

## 2026/1/23 - post `3b3162e`
- Testing Auto Drive with in Simulated Environment

## 2026/1/23 - post `af10dff`
- refine changelog writing style
- Auto Drive (+ SIM)

## 2026/1/22 - post `365f8a1`
- FF constants (temporary removed) (+)
- deadzone for a buggy Xbox controller  
(Will write code to have speed mapping later) (+)

## 2026/1/22 - post `b61a4ca`
- `GlobalConstants.SYS_ID` to enable/disable SysID (+)  
CTRE logging for SysID
Publish autoChoosers to NT4 (+)

## 2026/1/22 - post `69002c4`
PID Tunning (+)

## 2026/1/18 - post `49291d6`
Telemetry with AdvantageKit (+)

## 2026/1/18 - post `74418fc`
Removed Transcope Notation in comments (+)

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

