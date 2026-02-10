package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface IntakeIO {
    @AutoLog
    public class IntakeIOInputs {
        public double armPosition = 0;
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
    
    default Command runArm(double positionRad) {return Commands.none();}
    default Command runRoller(double velocity) {return Commands.none();}

    default void stopMotors() {}
}
