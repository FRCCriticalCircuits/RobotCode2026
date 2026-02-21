# Format
date - post `commit ID (short version)`

# Symbols
(?) -> TBD  
(!) -> Untested Code  
(+) -> Tested  
(+ SIM) -> Tested in simulation

# Todo
MotionMagic FOC Control for Hood and maybe Intake arm also

IF THE FUEL EVER GET STUCK WE DO A Cascade PID FOR THE arm/HOPPER(SEQUENCER)

## post `97d14c3`
- a KidsMode Axis Config for swerve
- catch BufferUnderFlow for Logging
- Fix Rotation Feedforward for AutoAim
- Some changes that will be revert after drivetest
- Different Constants for AutoAim Rotation PID
- rough tune for AutoAim Rotation PID Controller   

## post `d1a0688`
- Moved Climber Voltage to the Constant File
- format comments