package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ShooterIOSim implements ShooterIO {
    private final DCMotorSim hood, shooter;

    private final PIDController hoodController = new PIDController(10, 0, 0);
    private final PIDController shooterController = new PIDController(1, 0, 0);

    private double appliedVoltsHood, appliedVoltsShooter;

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
    }

    @Override
    public Command runHood(double positionRad) {
        return Commands.runOnce(
            () -> {
                hood.setInputVoltage(
                    MathUtil.clamp(
                        hoodController.calculate(
                            hood.getAngularPositionRad(),
                            positionRad
                        ),
                        -12.0,
                        12.0
                    )
                );
            }
        ).withName("Shooter.runHoodPosition");
    }

    @Override
    public Command runShooter(double velocity) {
        return Commands.runOnce(
            () -> {
                shooter.setInputVoltage(
                    MathUtil.clamp(
                        shooterController.calculate(
                            shooter.getAngularVelocityRadPerSec(),
                            velocity
                        ),
                        -12.0,
                        12.0
                    )
                );
            }
        ).withName("Shooter.runShooterVelocity"); 
    }

    @Override
    public void stopMotors() {
        hood.setInputVoltage(0.0);
        shooter.setInputVoltage(0.0);
    }
}
