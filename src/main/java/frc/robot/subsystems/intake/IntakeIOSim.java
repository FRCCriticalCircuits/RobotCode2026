package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeIOSim implements IntakeIO {
    private final DCMotorSim slider, roller;
    private final DCMotor sliderGearing = DCMotor.getKrakenX60(1);
    private final DCMotor rollerGearing = DCMotor.getKrakenX60(1);
    private double sliderVoltage = 0.0;
    private double rollerVoltage = 0.0;

    private final PIDController sliderController = new PIDController(20, 0, 0);

    public IntakeIOSim() {
        slider = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(6, 0.05),
            sliderGearing
        );

        roller = new DCMotorSim(
            LinearSystemId.createDCMotorSystem(0.03, 0.001),
            rollerGearing
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        slider.update(0.02);
        roller.update(0.02);
        
        inputs.positionSlider = slider.getAngularPositionRad();
        inputs.velocityRoller = roller.getAngularVelocityRadPerSec();
        
        inputs.sliderVoltage = sliderVoltage;
        inputs.rollerVoltage = rollerVoltage;
        
        inputs.supplyCurrentSlider = slider.getCurrentDrawAmps();
        inputs.torqueCurrentSlider = sliderGearing.getCurrent(slider.getAngularVelocityRadPerSec(), sliderVoltage);  
        inputs.supplyCurrentRoller = roller.getCurrentDrawAmps();
        inputs.torqueCurrentRoller = rollerGearing.getCurrent(slider.getAngularVelocityRadPerSec(), rollerVoltage);
        
        inputs.connected = true;
    }

    public void runVoltageSlider(double voltage) {
        sliderVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        slider.setInputVoltage(sliderVoltage);
    }

    @Override
    public void runRoller(double voltage) {
        rollerVoltage = MathUtil.clamp(voltage, -12.0, 12.0);
        roller.setInputVoltage(rollerVoltage);
    }

    @Override
    public void setSlider(double position) {
        runVoltageSlider(
            sliderController.calculate(slider.getAngularPositionRad(), position)
        );
    }
}
