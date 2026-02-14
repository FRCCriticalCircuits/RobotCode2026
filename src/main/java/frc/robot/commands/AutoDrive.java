package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

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
    private final SwerveRequest.ApplyFieldSpeeds controlRequest = new SwerveRequest.ApplyFieldSpeeds()
        .withDesaturateWheelSpeeds(true);

    private Pose2d targetPose2d;

    private final Pose2d tolorance = ChassisConstants.TOLORANCE_AUTO_DRIVE;
    private final ChassisSpeeds speeds = new ChassisSpeeds();

    private final ProfiledPIDController rotationController, xController, yController;
    private final State targetRotation = new State();
    private final State targetX = new State();
    private final State targetY = new State();

    private final SwerveDriveState state;

    public AutoDrive(Drive drive){
        this.drive = drive;
        this.state = drive.getState();

        rotationController = new ProfiledPIDController(
            ChassisConstants.ROTATION_PID_P,
            0,
            ChassisConstants.ROTATION_PID_D,
            new Constraints(Math.PI * 2, Math.PI * 4)
        );

        xController = new ProfiledPIDController(
            ChassisConstants.TRANSLATION_PID_P,
            0,
            ChassisConstants.TRANSLATION_PID_D,
            new Constraints(2, 3)
        );

        yController = new ProfiledPIDController(
            ChassisConstants.TRANSLATION_PID_P,
            0,
            ChassisConstants.TRANSLATION_PID_D,
            new Constraints(2, 3)
        );

        rotationController.enableContinuousInput(0, Math.PI * 2);

        this.targetRotation.velocity = 0;
        this.targetX.velocity = 0;
        this.targetY.velocity = 0;

        addRequirements(drive);
    }

    public AutoDrive withTarget(Pose2d targetPose2d){
        this.targetPose2d = targetPose2d;
        return this;
    }

    @Override
    public void initialize() {
        if(this.targetPose2d == null) this.cancel();

        rotationController.reset(state.Pose.getRotation().getRadians(), state.Speeds.omegaRadiansPerSecond);
        xController.reset(state.Pose.getX(), state.Speeds.vxMetersPerSecond);
        yController.reset(state.Pose.getY(), state.Speeds.vyMetersPerSecond);
    }

    @Override
    public void execute() {
        this.targetRotation.position = targetPose2d.getRotation().getRadians();
        this.targetX.position = targetPose2d.getX();
        this.targetY.position = targetPose2d.getY();

        speeds.vxMetersPerSecond = xController.calculate(
            state.Pose.getX(),
            targetX
        );

        speeds.vyMetersPerSecond = yController.calculate(
            state.Pose.getY(),
            targetY
        );

        speeds.omegaRadiansPerSecond = rotationController.calculate(
            state.Pose.getRotation().getRadians(),
            targetRotation
        );

        drive.setControl(
            controlRequest.withSpeeds(this.speeds)
        );
    }

    @Override
    public boolean isFinished() {
        return Math.abs(state.Pose.getX() - targetPose2d.getX()) < tolorance.getX()
        && Math.abs(state.Pose.getY() - targetPose2d.getY()) < tolorance.getY()
        && Math.abs(state.Pose.getRotation().getRadians()) < tolorance.getRotation().getRadians();
    }

    @Override
    public void end(boolean interrupted) {}
}
