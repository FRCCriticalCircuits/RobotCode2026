# Format
date - post `commit ID (short version)`

# Todo
- Rewrite Telemetry with AdvantageKit
- Implement Pathplanner
- Tune Swerve PID to solve noise issues
- if `SignalLogger.start();` is needed

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
Later will set the default option of `autoChooser` to regular auto
- marked `m_sysIdRoutineSteer` as unused 
we only need chassis Rotation/Translation Ka/Kv for MOI estimate
- Removed Replay (Using REV & AKit is better & Replay didnt work offseason)
- Signal Logger log stuff to the usb memory

