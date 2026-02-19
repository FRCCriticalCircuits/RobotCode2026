package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.ChassisConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.AutoAim.ShootingParams;
import frc.robot.utils.AxisMappingTable;

public class DriveCommand extends Command{
    private final Drive drive;

    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);
    
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
        .withDesaturateWheelSpeeds(true);

    private final Supplier<Double> velocityX;
    private final Supplier<Double> velocityY;
    private final Supplier<Double> rotationalRate;
    private final Supplier<Boolean> aiming;
    private final AxisMappingTable leftAxisTable, rightAxisTable;
    private final Supplier<ShootingParams> yawSupplier;

    private final PIDController rotationController;

    private SwerveDriveState state;
    private double rotationSpeed;

    public DriveCommand(
        Drive drive,
        Supplier<Double> velocityX,
        Supplier<Double> velocityY,
        Supplier<Double> rotationalRate,
        Supplier<Boolean> aiming,
        AxisMappingTable leftAxisTable,
        AxisMappingTable rightAxisTable,
        Supplier<ShootingParams> yawSupplier
    ){
        this.drive = drive;

        this.velocityX = velocityX;
        this.velocityY = velocityY;
        this.rotationalRate = rotationalRate;

        this.aiming = aiming;

        this.leftAxisTable = leftAxisTable;
        this.rightAxisTable = rightAxisTable;

        this.yawSupplier = yawSupplier;

        rotationController = new PIDController(
            ChassisConstants.ROTATION_PID_P,
            0,
            ChassisConstants.ROTATION_PID_D
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
        if(aiming.get()){
            rotationSpeed = yawSupplier.get().yaw_ff + rotationController.calculate(
                state.Pose.getRotation().getRadians(),
                yawSupplier.get().yaw
            );
        }else{
            rotationSpeed = rightAxisTable.get(rotationalRate.get()) * MaxAngularRate;
        }
        drive.setControl(
            fieldCentric
                .withVelocityX(
                    leftAxisTable.get(velocityX.get()) * MaxSpeed
                ).withVelocityY(
                    leftAxisTable.get(velocityY.get()) * MaxSpeed
                ).withRotationalRate(
                    rotationSpeed
                )
        );
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
