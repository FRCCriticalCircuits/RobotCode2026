package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim arm, roller;

    private final PIDController armController = new PIDController(10, 0, 0);
    private final PIDController rollerController = new PIDController(0.5, 0, 0);

    // Cached Desired States
    private double desiredArmPositionRad = 0.0;
    private double desiredRollerVelocityRadPerSec = 0.0;
    private boolean rollerClosedLoopEnabled = false;

    // Cached Voltage for Logging
    private double appliedVoltsArm = 0.0;
    private double appliedVoltsRoller = 0.0;

    public IntakeIOSim() {
        arm = new DCMotorSim(
            IntakeConstants.ARM_STATE_SPACE,
            IntakeConstants.ARM_GEARBOX
        );

        roller = new DCMotorSim(
            IntakeConstants.ROLLER_STATE_SPACE,
            IntakeConstants.ROLLER_GEARBOX
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.armPosition = arm.getAngularPositionRad();
        inputs.rollerPosition = roller.getAngularPositionRad();
        inputs.rollerVelocity = roller.getAngularVelocityRadPerSec();

        inputs.appliedVoltsArm = appliedVoltsArm;
        inputs.supplyCurrentArm = arm.getCurrentDrawAmps();

        inputs.appliedVoltsRoller = appliedVoltsRoller;
        inputs.supplyCurrentRoller = roller.getCurrentDrawAmps();

        inputs.armConnected = true;
        inputs.rollerConnected = true;
    }

    @Override
    public void applyOutputs(){
        arm.update(0.02);
        roller.update(0.02);

        appliedVoltsArm = MathUtil.clamp(
            armController.calculate(
                arm.getAngularPositionRad(),
                desiredArmPositionRad
            ),
            -12.0,
            12.0
        );

        arm.setInputVoltage(appliedVoltsArm);

        if (rollerClosedLoopEnabled) {
            appliedVoltsRoller = MathUtil.clamp(
                rollerController.calculate(
                    roller.getAngularVelocityRadPerSec(),
                    desiredRollerVelocityRadPerSec
                ),
                -12.0,
                12.0
            );
        } else {
            appliedVoltsRoller = 0.0;
        }

        roller.setInputVoltage(appliedVoltsRoller);
    }

    @Override
    public void setArmPosition(double positionRad) {
        desiredArmPositionRad = positionRad;
    }

    @Override
    public void setRollerVelocity(double velocityRadPerSec) {
        desiredRollerVelocityRadPerSec = velocityRadPerSec;
        rollerClosedLoopEnabled = true;
    }

    @Override
    public void runRollerVoltage(double voltage) {
        this.appliedVoltsRoller = voltage;
        // will ignore `rollerStopped` variable
        roller.setInputVoltage(voltage);
    }

    @Override
    public void stopIntake() {
        desiredArmPositionRad = 0.0;
        rollerClosedLoopEnabled = false;
    }
}

