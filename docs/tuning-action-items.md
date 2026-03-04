# 2026 Robot Tuning Action Items

## Shooter
- [ ] Verify `HOOD_GEARING` against measured mechanism reduction in `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java`.
- [ ] Verify `SHOOTER_GEARING` against measured mechanism reduction in `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java`.
- [ ] Tune hood MotionMagic gains `HOOD_PID_P`, `HOOD_PID_D`, and `HOOD_VEL_FF` in `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java`.
- [ ] Tune hood profile limits `HOOD_MAX_VEL` and `HOOD_MAX_ACCEL` (mechanism rotations units) in `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java`.
- [ ] Tune shooter velocity loop `SHOOTER_PID_P`, `SHOOTER_PID_I`, `SHOOTER_PID_D` and optional `SHOOTER_VEL_FF` in `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java`.
- [ ] Re-tune readiness tolerances `HOOD_STABLE_TOLERANCE_RAD` and `SHOOTER_STABLE_TOLERANCE_RAD_PER_SEC` in `src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java`.

## Intake
- [ ] Keep the arm gearing expression format and verify value on robot for `ARM_GEARING` in `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java`.
- [ ] Tune arm MotionMagic gains `ARM_PID_P`, `ARM_PID_D`, `ARM_VEL_FF`, plus optional gravity `ARM_GRAVITY_FF` in `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java`.
- [ ] Tune arm profile limits `ARM_MAX_VEL` and `ARM_MAX_ACCEL` in `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java`.
- [ ] Update `ROLLER_GEARING` to measured mechanism ratio in `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java`.
- [ ] Tune roller `ROLLER_PID_P`, `ROLLER_PID_I`, `ROLLER_PID_D`, and optional `ROLLER_VEL_FF` in `src/main/java/frc/robot/subsystems/intake/IntakeConstants.java`.

## Hopper
- [x] Hopper PID constants removed from `src/main/java/frc/robot/subsystems/hopper/HopperConstants.java`.
- [x] Hopper closed-loop PID config removed from `src/main/java/frc/robot/subsystems/hopper/HopperIOSpark.java`.
- [ ] Keep validating direct-voltage behavior and current limits `HOPPER_STALL_LIMIT` / `HOPPER_FREE_LIMIT` in `src/main/java/frc/robot/subsystems/hopper/HopperConstants.java`.

## Vision
- [ ] Tune AprilTag vision covariance `VISION_STD_DEVS` in `src/main/java/frc/robot/subsystems/vision/VisionConstants.java`.
- [x] Vision measurement std-devs now applied in `src/main/java/frc/robot/subsystems/vision/Vision.java`.

## Superstructure
- [ ] Retune `INTAKE_ARM_POS` after final intake mounting in `src/main/java/frc/robot/subsystems/SuperStructureConstants.java`.
- [ ] Retune `INTAKE_ROLLER_VEL` in `src/main/java/frc/robot/subsystems/SuperStructureConstants.java`.
- [ ] Retune `SHOOT_FLYWHEEL_VEL` (currently based on an estimate near 8V) in `src/main/java/frc/robot/subsystems/SuperStructureConstants.java`.
- [x] Keep `SHOOT_SEQUENCER_VOLTS` baseline as-is in `src/main/java/frc/robot/subsystems/SuperStructureConstants.java`.
- [ ] Retune `CLIMBER_FORWARD` and `CLIMBER_REVERSE` in `src/main/java/frc/robot/subsystems/SuperStructureConstants.java`.

## Drive
- [ ] Tune `SIMULATION` and `REAL` translation/rotation PID values in `src/main/java/frc/robot/subsystems/drive/ChassisConstants.java`.
- [ ] Tune auto-aim PID values in `AUTOAIM_ROTATION.SIMULATION` and `AUTOAIM_ROTATION.REAL` in `src/main/java/frc/robot/subsystems/drive/ChassisConstants.java`.
- [ ] Tune `TOLORANCE_AUTO_DRIVE` in `src/main/java/frc/robot/subsystems/drive/ChassisConstants.java`.
- [ ] Tune `AUTO_DRIVE_MAX_VEL` and `AUTO_DRIVE_MAX_ACCEL` in `src/main/java/frc/robot/subsystems/drive/ChassisConstants.java`.

## Global
- [ ] Select final driver axis mappings (`LEFT_AXIS_CONFIG`, `RIGHT_AXIS_CONFIG`) with driver testing in `src/main/java/frc/robot/GlobalConstants.java`.

## Aim
- [ ] Replace placeholder hood interpolation table points in `src/main/java/frc/robot/utils/AimCalc.java`.
