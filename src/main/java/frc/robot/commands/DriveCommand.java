package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;

public class DriveCommand extends Command{
    private final Drive drive;

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond);
    
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed)
        .withRotationalDeadband(MaxAngularRate)
        .withDesaturateWheelSpeeds(true);

    private final SwerveRequest.ApplyFieldSpeeds customFieldCentric = new SwerveRequest.ApplyFieldSpeeds()
            .withDesaturateWheelSpeeds(true);
    private final ChassisSpeeds speeds = new ChassisSpeeds();

    private final Supplier<Double> velocityX;
    private final Supplier<Double> velocityY;
    private final Supplier<Double> rotationalRate;
    private final Supplier<Boolean> aiming;
    private final InterpolatingDoubleTreeMap leftAxisTable, rightAxisTable;

    public DriveCommand(
        Drive drive,
        Supplier<Double> velocityX,
        Supplier<Double> velocityY,
        Supplier<Double> rotationalRate,
        Supplier<Boolean> aiming,
        InterpolatingDoubleTreeMap leftAxisTable,
        InterpolatingDoubleTreeMap rightAxisTable
    ){
        this.drive = drive;

        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.rotationalRate = rotationalRate;

        this.aiming = aiming;

        this.leftAxisTable = leftAxisTable;
        this.rightAxisTable = rightAxisTable;

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
                fieldCentric
                    .withVelocityX(
                        leftAxisTable.get(velocityX.get() * MaxSpeed)
                    ).withVelocityY(
                        leftAxisTable.get(velocityY.get() * MaxSpeed)
                    ).withRotationalRate(
                        rightAxisTable.get(rotationalRate.get() * MaxAngularRate)
                    )
            );
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
