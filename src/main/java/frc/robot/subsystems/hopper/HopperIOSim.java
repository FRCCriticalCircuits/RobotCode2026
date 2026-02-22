package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HopperIOSim implements HopperIO{
    private final DCMotorSim hopper;

    private final PIDController hopperController = new PIDController(10, 0, 0);

    private double appliedVoltsHopper = 0;
    private double hopperVelocity = 0;
    private boolean hopperStopped = true;

    public HopperIOSim(){
        hopper = new DCMotorSim(
            HopperConstants.HOPPER_STATE_SPACE,
            HopperConstants.HOPPER_GEARBOX
        );
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        hopper.update(0.02);
        
        inputs.hopperPosition = hopper.getAngularPositionRad();
        inputs.hopperVelocity = hopper.getAngularVelocityRadPerSec();

        inputs.appliedVoltsHopper = this.appliedVoltsHopper;
        inputs.supplyCurrentHopper = hopper.getCurrentDrawAmps();

        inputs.hopperConnected = true;

        if(!hopperStopped){
            this.appliedVoltsHopper = MathUtil.clamp(
                hopperController.calculate(
                    hopper.getAngularVelocityRadPerSec(),
                    this.hopperVelocity
                ),
                -12.0,
                12.0
            );

            hopper.setInputVoltage(appliedVoltsHopper);
        }
    }

    @Override
    public Command runHopper(double velocity) {
        return Commands.run(
            () -> {
                this.hopperStopped = false;
                this.hopperVelocity = velocity;
            }
        ).withName("hopper.runRollerVelocity");
    }

    @Override
    public void runHopperVoltage(double voltage) {
        this.appliedVoltsHopper = voltage;
        // will ignore `hopperStopped` variable
        hopper.setInputVoltage(voltage);
    }

    @Override
    public void stopMotors() {
        this.hopperStopped = true;

        hopper.setInputVoltage(0.0);
    }
}
