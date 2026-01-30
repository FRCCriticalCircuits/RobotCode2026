package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim pivot, roller;

    private final PIDController pivotController = new PIDController(10, 0, 0);
    private final PIDController rollerController = new PIDController(1, 0, 0);

    private double appliedVoltsPivot, appliedVoltsRoller;

    public IntakeIOSim() {
        pivot = new DCMotorSim(
            IntakeConstants.PIVOT_STATE_SPACE,
            IntakeConstants.PIVOT_GEARBOX
        );

        roller = new DCMotorSim(
            IntakeConstants.ROLLER_STATE_SPACE,
            IntakeConstants.ROLLER_GEARBOX
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        pivot.update(0.02);
        roller.update(0.02);
        
        inputs.pivotPosition = pivot.getAngularPositionRad();
        inputs.rollerVelocity = roller.getAngularVelocityRadPerSec();

        inputs.appliedVoltsPivot = this.appliedVoltsPivot;
        inputs.supplyCurrentPivot = pivot.getCurrentDrawAmps();

        inputs.appliedVoltsRoller = this.appliedVoltsRoller;
        inputs.supplyCurrentRoller = roller.getCurrentDrawAmps();

        inputs.pivotConnected = true;
        inputs.rollerConnected = true;
    }

    @Override
    public void runRoller(double velocity) {
        roller.setInputVoltage(
            MathUtil.clamp(
                rollerController.calculate(
                    roller.getAngularVelocityRadPerSec(),
                    velocity
                ),
                -12.0,
                12.0
            )
        );
    }

    @Override
    public void runPivot(double positionRad) {
        pivot.setInputVoltage(
            MathUtil.clamp(
                pivotController.calculate(
                    pivot.getAngularPositionRad(),
                    positionRad
                ),
                -12.0,
                12.0
            )
        );
    }
}

