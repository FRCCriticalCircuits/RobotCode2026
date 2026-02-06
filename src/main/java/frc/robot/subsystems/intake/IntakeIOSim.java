package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim arm, roller;

    private final PIDController armController = new PIDController(10, 0, 0);
    private final PIDController rollerController = new PIDController(0.5, 0, 0);

    private double appliedVoltsArm, appliedVoltsRoller;
    private double armPosition = 0, rollerVelocity = 0;
    private Boolean motorStopped = false;

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
        arm.update(0.02);
        roller.update(0.02);
        
        inputs.armPosition = arm.getAngularPositionRad();
        inputs.rollerVelocity = roller.getAngularVelocityRadPerSec();

        inputs.appliedVoltsArm = this.appliedVoltsArm;
        inputs.supplyCurrentArm = arm.getCurrentDrawAmps();

        inputs.appliedVoltsRoller = this.appliedVoltsRoller;
        inputs.supplyCurrentRoller = roller.getCurrentDrawAmps();

        inputs.armConnected = true;
        inputs.rollerConnected = true;

        // always need closeloop 
        arm.setInputVoltage(
            MathUtil.clamp(
                armController.calculate(
                    arm.getAngularPositionRad(),
                    this.armPosition
                ),
                -12.0,
                12.0
            )
        );

        if(!motorStopped){    
            roller.setInputVoltage(
                MathUtil.clamp(
                    rollerController.calculate(
                        roller.getAngularVelocityRadPerSec(),
                        this.rollerVelocity
                    ),
                    -12.0,
                    12.0
                )
            );
        }
    }

    @Override
    public Command runArm(double positionRad) {
        return Commands.runOnce(
            () -> {
                this.motorStopped = false;
                this.armPosition = positionRad;
            }
        ).withName("Intake.runArmPosition");
    }

    @Override
    public Command runRoller(double velocity) {
        return Commands.runOnce(
            () -> {
                this.motorStopped = false;
                this.rollerVelocity = velocity;
            }
        ).withName("Intake.runRollerVelocity");
    }

    @Override
    public void stopMotors() {
        roller.setInputVoltage(0.0);
        this.motorStopped = true;
        this.armPosition = 0;
    }
}

