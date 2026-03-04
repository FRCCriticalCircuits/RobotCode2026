package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class HopperIOSim implements HopperIO{
    private final DCMotorSim hopper;

    private double appliedVoltsHopper = 0;

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
    }

    /* Feed voltage into simulation state-spate, considering KS */
    private void feedHopperVoltage(double voltage){
        if(Math.abs(voltage) < HopperConstants.HOPPER_KS){
            voltage = 0.0;
        }
        hopper.setInputVoltage(voltage - Math.signum(voltage) * HopperConstants.HOPPER_KS);
    }

    @Override
    public Command runHopper(double voltage) {
        return Commands.run(
            () -> {
                feedHopperVoltage(voltage);
            }
        ).withName("hopper.runRollerVoltage");
    }

    @Override
    public void runHopperVoltage(double voltage) {
        this.appliedVoltsHopper = voltage;
        // will ignore `hopperStopped` variable
        feedHopperVoltage(voltage);
    }

    @Override
    public void stopMotors() {
        hopper.setInputVoltage(0.0);
    }
}
