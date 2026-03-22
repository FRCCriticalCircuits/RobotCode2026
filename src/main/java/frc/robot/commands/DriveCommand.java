package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.ChassisConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.calc.AimCalc.ShootingParams;

public class DriveCommand extends Command{
    private final Drive drive;

    private final double maxTeleopSpeed =
        TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * ChassisConstants.TELEOP_MAX_SPEED_SCALE;
    private final double maxTeleopAngularRate =
        RotationsPerSecond.of(ChassisConstants.TELEOP_MAX_ANGULAR_RATE_ROTATIONS_PER_SECOND)
            .in(RadiansPerSecond);
    
    private final SwerveRequest.FieldCentric manualDrive = new SwerveRequest.FieldCentric()
        .withDeadband(maxTeleopSpeed * ChassisConstants.TELEOP_TRANSLATION_DEADBAND_RATIO)
        .withRotationalDeadband(maxTeleopAngularRate * ChassisConstants.TELEOP_ROTATION_DEADBAND_RATIO)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.Position)
        .withDesaturateWheelSpeeds(true);
    private final SwerveRequest.FieldCentric aimDrive = new SwerveRequest.FieldCentric()
        .withDeadband(maxTeleopSpeed * ChassisConstants.TELEOP_TRANSLATION_DEADBAND_RATIO)
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.Position)
        .withDesaturateWheelSpeeds(true);

    private final Supplier<Double> velocityX;
    private final Supplier<Double> velocityY;
    private final Supplier<Double> rotationalRate;
    private final Supplier<Boolean> aiming;
    private final Supplier<ShootingParams> yawSupplier;

    private final PIDController rotationController;

    private SwerveDriveState state;

    public DriveCommand(
        Drive drive,
        Supplier<Double> velocityX,
        Supplier<Double> velocityY,
        Supplier<Double> rotationalRate,
        Supplier<Boolean> aiming,
        Supplier<ShootingParams> yawSupplier
    ){
        this.drive = drive;

        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.rotationalRate = rotationalRate;

        this.aiming = aiming;

        this.yawSupplier = yawSupplier;

        rotationController = new PIDController(
            ChassisConstants.AUTOAIM_ROTATION_PID_P,
            0,
            ChassisConstants.AUTOAIM_ROTATION_PID_D
        );

        rotationController.enableContinuousInput(0, Math.PI * 2);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        state = drive.getState();
        rotationController.reset();
    }

    @Override
    public void execute() {
        state = drive.getState();
        double requestedVelocityX = velocityX.get() * maxTeleopSpeed;
        double requestedVelocityY = velocityY.get() * maxTeleopSpeed;

        if(aiming.get()){
            ShootingParams aimParams = yawSupplier.get();
            double rotationSpeed = aimParams.yaw_ff + rotationController.calculate(
                state.Pose.getRotation().getRadians(),
                aimParams.yaw
            );

            drive.setControl(
                aimDrive
                    .withVelocityX(requestedVelocityX)
                    .withVelocityY(requestedVelocityY)
                    .withRotationalRate(rotationSpeed)
            );
        }else{
            drive.setControl(
                manualDrive
                    .withVelocityX(requestedVelocityX)
                    .withVelocityY(requestedVelocityY)
                    .withRotationalRate(rotationalRate.get() * maxTeleopAngularRate)
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
