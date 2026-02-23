package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimberIOSim implements ClimberIO{
    private final DCMotorSim climber;

    private final PIDController climberController = new PIDController(10, 0, 0);

    private double appliedVoltsClimber = 0;
    private double climberVelocity = 0;
    private boolean climberStopped = true;

    /*
     * DO NOT RUN IT
     * 1. runClimber Function is suppose to take voltage as input
     * 2. We dont need velocity closeloop, we need either position closeloop or voltage openloop
     */
    @Deprecated()
    public ClimberIOSim(){
        climber = new DCMotorSim(
            ClimberConstants.CLIMBER_STATE_SPACE,
            ClimberConstants.CLIMBER_GEARBOX
        );
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        climber.update(0.02);
        inputs.climberPosition = climber.getAngularPositionRad();

        inputs.climberConnected = true;

        if(!climberStopped){
            this.appliedVoltsClimber = climberController.calculate(
                climber.getAngularVelocityRadPerSec(),
                this.climberVelocity
            );

            climber.setInputVoltage(appliedVoltsClimber);
        }
    }
    @Override
    public Command runClimber(double velocity) {
        return Commands.run(
            () -> {
                this.climberStopped = false;
                this.climberVelocity = velocity;
            }
        ).withName("climber.runClimberVelocity");
    }

    @Override
    public void stopMotors() {
        this.climberStopped = true;
        this.appliedVoltsClimber = 0;

        climber.setInputVoltage(0);
    }    
}