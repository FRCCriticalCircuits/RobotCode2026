package frc.robot.subsystems.hopper;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface HopperIO {
    @AutoLog
    public class HopperIOInputs {
        public double hopperVelocity = 0;

        public double appliedVoltsHopper = 0;
        public double supplyCurrentHopper = 0;
        public double torqueCurrentHopper = 0;

        public double tempHopper = 0;

        public boolean hopperConnected = false;
    }

    default void updateInputs(HopperIOInputs inputs) {}
    
    default Command runHopper(double velocity) {return Commands.none();}
    
    default void stopMotors() {}
}
