package frc.robot.subsystems.climber;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimberIOSim implements ClimberIO{
    private final DCMotorSim climber;

    //private final PIDController climberController = new PIDController(10, 0, 0);

    private double appliedVoltsClimber = 0;
    private boolean climberStopped = true;

    public ClimberIOSim(){
        climber = new DCMotorSim(
            ClimberConstants.CLIMBER_STATE_SPACE,
            ClimberConstants.CLIMBER_GEARBOX
        );
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        climber.update(0.02);
        inputs.climberConnected = true;
        
        if(!climberStopped){
            this.appliedVoltsClimber = climber.getInputVoltage();
            climber.setInputVoltage(appliedVoltsClimber);
        }   
    }
    @Override
    public Command runClimber(double voltage) {
        return Commands.run(
            () -> {
                this.climberStopped = false;
                this.appliedVoltsClimber = voltage;
            }
        ).withName("climber.runClimberVoltage");
    }

    @Override
    public void stopMotors() {
        this.climberStopped = true;
        this.appliedVoltsClimber = 0;

        climber.setInputVoltage(0);
    }    
}