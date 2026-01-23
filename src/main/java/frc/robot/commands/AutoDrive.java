package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;

public class AutoDrive extends Command {
    private final CommandSwerveDrivetrain drive;
    private final SwerveRequest.ApplyFieldSpeeds controlRequest;
    private final Pose2d targetPose2d; 
    private final HolonomicDriveController swerveController;

    public AutoDrive(CommandSwerveDrivetrain drive, Pose2d targetPose2d){
        this.drive = drive;
        this.controlRequest = new SwerveRequest.ApplyFieldSpeeds()
            .withDesaturateWheelSpeeds(true);
        this.targetPose2d = targetPose2d;

        swerveController = new HolonomicDriveController(
            new PIDController(
                0,
                0,
                0
            ),
            new PIDController(
                0,
                0,
                0
            ),
            new ProfiledPIDController(
                0,
                0,
                0,
                new Constraints(6, 3)
            )
        );
    }

    @Override
    public void initialize() {
        swerveController.setTolerance(
            new Pose2d(
                new Translation2d(0.05, 0.05),
                Rotation2d.fromDegrees(1)
            )
        );

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.applyRequestUnsafe(
            () -> controlRequest.withSpeeds(
                swerveController.calculate(
                    drive.getStateCopy().Pose,
                    targetPose2d,
                    0,
                    targetPose2d.getRotation()
                )
            )
        );
    }

    @Override
    public boolean isFinished() {
        return swerveController.atReference();
    }

    @Override
    public void end(boolean interrupted) {}
}
