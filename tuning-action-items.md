# 2026 Robot Tuning Action Items

## Hopper
- [ ] Keep validating direct-voltage behavior and current limits `HOPPER_STALL_LIMIT` / `HOPPER_FREE_LIMIT` in `src/main/java/frc/robot/subsystems/hopper/HopperConstants.java`.

## Vision
- [ ] Tune AprilTag vision covariances in `src/main/java/frc/robot/subsystems/vision/VisionConstants.java`.

## Drive
- [ ] Tune `SIMULATION` and `REAL` translation/rotation PID values in `src/main/java/frc/robot/subsystems/drive/ChassisConstants.java`.
- [x] Tune auto-aim PID values in `AUTOAIM_ROTATION.SIMULATION` and `AUTOAIM_ROTATION.REAL` in `src/main/java/frc/robot/subsystems/drive/ChassisConstants.java`.
- [ ] Tune `TOLORANCE_AUTO_DRIVE` in `src/main/java/frc/robot/subsystems/drive/ChassisConstants.java`.
- [ ] Tune `AUTO_DRIVE_MAX_VEL` and `AUTO_DRIVE_MAX_ACCEL` in `src/main/java/frc/robot/subsystems/drive/ChassisConstants.java`.

## Global
- [ ] driver axis mappings

## Aim
- [ ] Replace placeholder hood interpolation table points in `src/main/java/frc/robot/utils/AimCalc.java`.
