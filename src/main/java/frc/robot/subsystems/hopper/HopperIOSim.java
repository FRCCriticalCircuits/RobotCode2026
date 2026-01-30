package frc.robot.subsystems.hopper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class HopperIOSim implements HopperIO{
    private final DCMotorSim hopper;

    private final PIDController hopperController = new PIDController(10, 0, 0);

    private double appliedVoltsHopper;

    public HopperIOSim(){
        hopper = new DCMotorSim(
            HopperConstants.HOPPER_STATE_SPACE,
            HopperConstants.HOPPER_GEARBOX
        );
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        hopper.update(0.02);
        
        inputs.hopperVelocity = hopper.getAngularVelocityRadPerSec();

        inputs.appliedVoltsHopper = this.appliedVoltsHopper;
        inputs.supplyCurrentHopper = hopper.getCurrentDrawAmps();

        inputs.hopperConnected = true;
    }

    @Override
    public void runHopper(double velocity) {
        hopper.setInputVoltage(
            MathUtil.clamp(
                hopperController.calculate(
                    hopper.getAngularVelocityRadPerSec(),
                    velocity
                ),
                -12.0,
                12.0
            )
        );
    }
}
