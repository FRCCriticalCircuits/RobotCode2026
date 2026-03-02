package frc.robot.commands;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.ChassisConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.AimCalc.ShootingParams;

public class PPDriveCommand extends Command{
    private final Drive drive;

    private final Supplier<ShootingParams> yawSupplier;

    private final PIDController rotationController;

    private SwerveDriveState state;
    private double rotationSpeed;

    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
        .withDesaturateWheelSpeeds(true);

    public PPDriveCommand(
        Drive drive,
        Supplier<ShootingParams> yawSupplier
    ){
        this.drive = drive;

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
    }

    @Override
    public void execute() {
        rotationSpeed = yawSupplier.get().yaw_ff + rotationController.calculate(
            state.Pose.getRotation().getRadians(),
            yawSupplier.get().yaw
        );

        drive.setControl(
            fieldCentric.withRotationalRate(rotationSpeed)
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
