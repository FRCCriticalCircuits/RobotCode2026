package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.GlobalConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;

public class DriveCommand extends Command{
    private final Drive drive;

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond);
    
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * GlobalConstants.VEL_CONTROL_BASE * GlobalConstants.DEADBAND_TRANSLATION)
        .withRotationalDeadband(MaxAngularRate * GlobalConstants.VEL_CONTROL_BASE * GlobalConstants.DEADBAND_ROTATION)
        .withDesaturateWheelSpeeds(true);

    private final SwerveRequest.ApplyFieldSpeeds customFieldCentric = new SwerveRequest.ApplyFieldSpeeds()
            .withDesaturateWheelSpeeds(true);
    private final ChassisSpeeds speeds = new ChassisSpeeds();

    private final Supplier<Double> velocityX;
    private final Supplier<Double> velocityY;
    private final Supplier<Double> rotationalRate;
    private final Supplier<Boolean> aiming;

    public DriveCommand(
        Drive drive,
        Supplier<Double> velocityX,
        Supplier<Double> velocityY,
        Supplier<Double> rotationalRate,
        Supplier<Boolean> aiming
    ){
        this.drive = drive;

        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.rotationalRate = rotationalRate;

        this.aiming = aiming;

        addRequirements(this.drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if(aiming.get() == true){
            drive.setControl(
                customFieldCentric.withSpeeds(this.speeds)
            );
        }else{
            drive.setControl(
                fieldCentric.withVelocityX(velocityX.get())
                    .withVelocityY(velocityY.get())
                    .withRotationalRate(rotationalRate.get())
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
