package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.ShooterConstants.TUNING;

public class ShooterIOSim implements ShooterIO {
    private final DCMotorSim hood, shooter;

    private final PIDController hoodController = new PIDController(10, 0, 0);
    private final PIDController shooterController = new PIDController(0.5, 0, 0);

    private double appliedVoltsHood, appliedVoltsShooter;
    private double hoodPosition = 0, shooterVelocity = 0;
    private boolean shooterStopped = false;

    public ShooterIOSim() {
        hood = new DCMotorSim(
            ShooterConstants.HOOD_STATE_SPACE,
            ShooterConstants.HOOD_GEARBOX
        );

        shooter = new DCMotorSim(
            ShooterConstants.SHOOTER_STATE_SPACE,
            ShooterConstants.SHOOTER_GEARBOX
        );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        hood.update(0.02);
        shooter.update(0.02);
        
        inputs.hoodPosition = hood.getAngularPositionRad();
        inputs.shooterVelocity = shooter.getAngularVelocityRadPerSec();

        inputs.appliedVoltsHood = this.appliedVoltsHood;
        inputs.supplyCurrentHood = hood.getCurrentDrawAmps();

        inputs.appliedVoltsShooter = this.appliedVoltsShooter;
        inputs.supplyCurrentShooter = shooter.getCurrentDrawAmps();

        inputs.hoodConnected = true;
        inputs.shooterConnected = true;

        // update closeloop
        hood.setInputVoltage(
            MathUtil.clamp(
                hoodController.calculate(
                    hood.getAngularPositionRad(),
                    this.hoodPosition
                ),
                -12.0,
                12.0
            )
        );

        if(!shooterStopped){
            shooter.setInputVoltage(
                MathUtil.clamp(
                    shooterController.calculate(
                        shooter.getAngularVelocityRadPerSec(),
                        this.shooterVelocity
                    ),
                    -12.0,
                    12.0
                )
            );
        } 
    }

    @Override
    public Command runHood(DoubleSupplier positionRad) {
        return Commands.run(
            () -> {
                this.hoodPosition = positionRad.getAsDouble();
            }
        ).withName("Shooter.runHoodPosition");
    }

    @Override
    public Command runShooter(double velocity) {
        return Commands.run(
            () -> {
                this.shooterStopped = false;
                this.shooterVelocity = velocity;
            }
        ).withName("Shooter.runShooterVelocity"); 
    }

    @Override
    public void runShooterVoltage(double voltage) {
        // will ignore `shooterStopped` variable
        shooter.setInputVoltage(voltage);
    }

    @Override
    public Boolean isStable() {
        // Match Kraken readiness behavior in sim so sequencing logic is testable.
        double hoodErrorRad = Math.abs(hood.getAngularPositionRad() - hoodPosition);
        double shooterErrorRadPerSec = Math.abs(shooter.getAngularVelocityRadPerSec() - shooterVelocity);
        return hoodErrorRad <= TUNING.HOOD_STABLE_TOLERANCE_RAD
            && shooterErrorRadPerSec <= TUNING.SHOOTER_STABLE_TOLERANCE_RAD_PER_SEC;
    }

    @Override
    public void stopMotors() {
        this.shooterStopped = true;
        this.hoodPosition = 0.0;
        shooter.setInputVoltage(0);
    }
}
