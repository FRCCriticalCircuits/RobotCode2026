package frc.robot.subsystems.hopper;

import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.robot.subsystems.hopper.HopperConstants.TUNING;

public class HopperIOSim implements HopperIO{
    private final DCMotorSim hopper;

    private double appliedVoltsHopper = 0;

    // Cached Desired States
    private double desiredHopperVelocityRps = 0.0;   // rotations per second
    private boolean hopperClosedLoopEnabled = false;

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

    @Override
    public void applyOutputs() {
        if (hopperClosedLoopEnabled) {
            // no time to write ks for this implement
            this.appliedVoltsHopper = desiredHopperVelocityRps * TUNING.HOPPER_KV;
            feedHopperVoltage(appliedVoltsHopper);
        } else {
            feedHopperVoltage(0);
        }
    }

    /* Feed voltage into simulation state-spate, considering KS */
    private void feedHopperVoltage(double voltage){
        if(Math.abs(voltage) < HopperConstants.HOPPER_KS_SIM){
            voltage = 0.0;
        }
        hopper.setInputVoltage(voltage - Math.signum(voltage) * HopperConstants.HOPPER_KS_SIM);
    }

    @Override
    public void runHopper(double velocityRadPerSec) {
        this.desiredHopperVelocityRps = velocityRadPerSec;
        this.hopperClosedLoopEnabled = true;
    }

    @Override
    public void runHopperVoltage(double voltage) {
        this.appliedVoltsHopper = voltage;
        // will ignore `hopperStopped` variable
        feedHopperVoltage(voltage);
    }

    @Override
    public void stopMotors() {
        this.hopperClosedLoopEnabled = false;
    }
}
