package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public interface ClimberIO {
    @AutoLog
    public class ClimberIOInputs{
        public double climberPosition = 0;
        
        public double appliedVoltsClimber = 0;
        public double torqueCurrentClimber = 0;

        public double tempClimber = 0;

        public boolean climberConnected = false;
    }
    
    default void updateInputs(ClimberIOInputs inputs) {}
    
    default Command runClimber(double voltage) {return Commands.none();}

    default void stopMotors() {}
}
