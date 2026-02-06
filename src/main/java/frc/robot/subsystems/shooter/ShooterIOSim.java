package frc.robot.subsystems.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterIOSim implements ShooterIO {
    private final DCMotorSim hood, shooter;

    private final PIDController hoodController = new PIDController(10, 0, 0);
    private final PIDController shooterController = new PIDController(0.5, 0, 0);

    private double appliedVoltsHood, appliedVoltsShooter;
    private double hoodPosition = 0, shooterVelocity = 0;
    private boolean motorStopped = false;

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

        // always need closeloop
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

        if(!motorStopped){
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
        return Commands.runOnce(
            () -> {
                this.motorStopped = false;
                this.hoodPosition = positionRad.getAsDouble();
            }
        ).withName("Shooter.runHoodPosition");
    }

    @Override
    public Command runShooter(double velocity) {
        return Commands.runOnce(
            () -> {
                this.motorStopped = false;
                this.shooterVelocity = velocity;
            }
        ).withName("Shooter.runShooterVelocity"); 
    }

    @Override
    public void stopMotors() {
        this.motorStopped = true;
        this.hoodPosition = 0.0;
        shooter.setInputVoltage(0);
    }
}
