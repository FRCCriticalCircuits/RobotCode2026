package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.ChassisConstants;
import frc.robot.subsystems.drive.Drive;

public class AutoDrive extends Command {
    private final Drive drive;
    private final SwerveRequest.ApplyFieldSpeeds controlRequest;

    private Pose2d curPose2d = new Pose2d();

    private final Pose2d targetPose2d, tolorance;
    private final ChassisSpeeds speeds;

    private final ProfiledPIDController omegaController, xController, yController;
    private final State targetRotation = new State();
    private final State targetX = new State();
    private final State targetY = new State();

    public AutoDrive(Drive drive, Pose2d targetPose2d){
        this.drive = drive;
        this.controlRequest = new SwerveRequest.ApplyFieldSpeeds()
            .withDesaturateWheelSpeeds(true);
        this.targetPose2d = targetPose2d;
        this.speeds = new ChassisSpeeds();

        this.tolorance = ChassisConstants.TOLORANCE_AUTO_DRIVE;

        omegaController = new ProfiledPIDController(
            ChassisConstants.ROTATION_PID_P,
            0,
            ChassisConstants.ROTATION_PID_D,
            new Constraints(2, 0.1)
        );

        xController = new ProfiledPIDController(
            ChassisConstants.TRANSLATION_PID_P,
            0,
            ChassisConstants.TRANSLATION_PID_D,
            new Constraints(3, 1)
        );

        yController = new ProfiledPIDController(
            ChassisConstants.TRANSLATION_PID_P,
            0,
            ChassisConstants.TRANSLATION_PID_D,
            new Constraints(3, 1)
        );
    }

    @Override
    public void initialize() {
        this.curPose2d = drive.getStateCopy().Pose;
        ChassisSpeeds curChassisSpeeds = drive.getStateCopy().Speeds;

        omegaController.reset(curPose2d.getRotation().getRadians(), curChassisSpeeds.omegaRadiansPerSecond);
        xController.reset(curPose2d.getX(), curChassisSpeeds.vxMetersPerSecond);
        yController.reset(curPose2d.getY(), curChassisSpeeds.vyMetersPerSecond);

        this.targetRotation.velocity = 0;
        this.targetX.velocity = 0;
        this.targetY.velocity = 0;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        // getState() will not have GC issue
        // but it changes
        this.curPose2d = drive.getStateCopy().Pose;
    
        this.targetRotation.position = targetPose2d.getRotation().getRadians();
        this.targetX.position = targetPose2d.getX();
        this.targetY.position = targetPose2d.getY();

        speeds.vxMetersPerSecond = xController.calculate(
            curPose2d.getX(),
            targetX
        );

        speeds.vyMetersPerSecond = yController.calculate(
            curPose2d.getY(),
            targetY
        );

        speeds.omegaRadiansPerSecond = omegaController.calculate(
            curPose2d.getRotation().getRadians(),
            targetRotation
        );

        drive.applyRequestUnsafe(
            () -> controlRequest.withSpeeds(
                this.speeds
            )
        );
    }

    @Override
    public boolean isFinished() {
        return Math.abs(curPose2d.getX() - targetPose2d.getX()) < tolorance.getX()
        && Math.abs(curPose2d.getY() - targetPose2d.getY()) < tolorance.getY()
        && Math.abs(curPose2d.getRotation().getRadians()) < tolorance.getRotation().getRadians();
    }

    @Override
    public void end(boolean interrupted) {}
}
