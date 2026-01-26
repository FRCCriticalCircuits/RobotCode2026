package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public double positionSlider;
        public double velocityRoller;

        public double sliderVoltage = 0.0;
        public double rollerVoltage = 0.0;

        public double supplyCurrentSlider;
        public double torqueCurrentSlider;
        public double supplyCurrentRoller;
        public double torqueCurrentRoller;
        
        public boolean connected;
    }

    default void updateInputs(IntakeIOInputs inputs) {}
    default void runRoller(double voltage) {}
    default void setSlider(double position) {}
}
