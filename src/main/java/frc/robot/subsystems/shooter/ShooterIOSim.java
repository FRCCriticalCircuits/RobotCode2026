package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.shooter.ShooterConstants.HAL;

public class ShooterIOSim implements ShooterIO {
    private final DCMotorSim hood, shooter;

    private final PIDController hoodController = new PIDController(10, 0, 0);
    private final PIDController shooterController = new PIDController(1, 0, 0);

    public ShooterIOSim() {
        hood = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ShooterConstants.HOOD_GEARBOX,
                ShooterConstants.HOOD_MOI,
                HAL.HOOD_GEARING
            ),
            ShooterConstants.HOOD_GEARBOX
        );

        shooter = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                ShooterConstants.SHOOTER_GEARBOX,
                ShooterConstants.SHOOTER_MOI,
                HAL.SHOOTER_GEARING
            ),
            ShooterConstants.SHOOTER_GEARBOX
        );
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        hood.update(0.02);
        shooter.update(0.02);
        
        inputs.hoodPosition = hood.getAngularPositionRad();
        inputs.shooterVelocity = shooter.getAngularVelocityRadPerSec();

        inputs.hoodConnected = true;
        inputs.shooterConnected = true;
    }

    @Override
    public void runShooter(double velocity) {
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

    @Override
    public void runHood(double positionRad) {
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
}
