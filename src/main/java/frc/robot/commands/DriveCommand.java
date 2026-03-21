package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.ChassisConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.utils.calc.AimCalc.ShootingParams;
import frc.robot.utils.axis.AxisMappingTable;

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
    private final SlewRateLimiter xLimiter = new SlewRateLimiter(ChassisConstants.TELEOP_TRANSLATION_SLEW_RATE);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(ChassisConstants.TELEOP_TRANSLATION_SLEW_RATE);
    private final SlewRateLimiter rotationLimiter = new SlewRateLimiter(ChassisConstants.TELEOP_ROTATION_SLEW_RATE);

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
        xLimiter.reset(0.0);
        yLimiter.reset(0.0);
        rotationLimiter.reset(0.0);
    }

    @Override
    public void execute() {
        double requestedVelocityX = leftAxisTable.get(velocityX.get()) * MaxSpeed;
        double requestedVelocityY = leftAxisTable.get(velocityY.get()) * MaxSpeed;

        if(aiming.get()){
            rotationSpeed = yawSupplier.get().yaw_ff + rotationController.calculate(
                state.Pose.getRotation().getRadians(),
                yawSupplier.get().yaw
            );
        }else{
            rotationSpeed = rotationLimiter.calculate(
                rightAxisTable.get(rotationalRate.get()) * MaxAngularRate
            );
        }

        drive.setControl(
            fieldCentric
                .withVelocityX(
                    xLimiter.calculate(requestedVelocityX)
                ).withVelocityY(
                    yLimiter.calculate(requestedVelocityY)
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
