package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public double pivotPosition = 0;
        public double rollerVelocity = 0;

        public double appliedVoltsPivot = 0;
        public double supplyCurrentPivot = 0;
        public double torqueCurrentPivot = 0;

        public double appliedVoltsRoller = 0;
        public double supplyCurrentRoller = 0;
        public double torqueCurrentRoller = 0;

        public double tempPivot = 0;
        public double tempRoller = 0;

        public boolean pivotConnected = false;
        public boolean rollerConnected = false;
    }

    default void updateInputs(IntakeIOInputs inputs) {}
    
    default void runPivot(double positionRad) {}
    default void runRoller(double velocity) {}
}
