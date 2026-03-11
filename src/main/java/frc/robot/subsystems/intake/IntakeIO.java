package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public double armPosition = 0;
        public double rollerPosition = 0;
        public double rollerVelocity = 0;

        public double appliedVoltsArm = 0;
        public double supplyCurrentArm = 0;
        public double torqueCurrentArm = 0;

        public double appliedVoltsRoller = 0;
        public double supplyCurrentRoller = 0;
        public double torqueCurrentRoller = 0;

        public double tempArm = 0;
        public double tempSecondaryArm = 0;
        public double tempRoller = 0;

        public boolean armConnected = false;
        public boolean rollerConnected = false;
    }

    default void updateInputs(IntakeIOInputs inputs) {}
    default void applyOutputs() {}
    
    default void setArmPosition(double positionRad) {}
    default void setRollerVelocity(double velocityRadPerSec) {}

    default void runArmVoltage(double voltage) {}
    default void runRollerVoltage(double voltage) {}

    default void stopRoller() {}
}
