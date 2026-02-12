package frc.robot.subsystems.climber;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.climber.ClimberConstants.HAL;

public class ClimberIONeoVortex implements ClimberIO{
    private final SparkFlex climberNeoMotor;
    private final RelativeEncoder climberEncoder;

    @SuppressWarnings("unused")
    private double appliedVoltsClimber = 0;

    @SuppressWarnings("unused")
    private double tempClimber = 0;

    // Configuration
    private final SparkFlexConfig climberConfig;
    @SuppressWarnings("unused")
    private PIDController climberPID;
    
    // Control Requests
    public ClimberIONeoVortex(){
        climberNeoMotor = new SparkFlex(63, MotorType.kBrushless);        
        climberEncoder = climberNeoMotor.getEncoder();

        // Configuration
        climberConfig = new SparkFlexConfig();

        climberConfig.voltageCompensation(HAL.NOMINAL_VOLTAGE);
        climberConfig.smartCurrentLimit(HAL.CLIMBER_CURRENT_LIMIT);

        climberConfig.encoder.positionConversionFactor((1/HAL.CLIMBER_GEARING) * Math.PI * 2);
        climberConfig.encoder.velocityConversionFactor((60.0/HAL.CLIMBER_GEARING) * Math.PI * 2);

        climberConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000.0 / HAL.FREQUENCY))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20
        );
        
        climberConfig.idleMode(IdleMode.kBrake);
        climberConfig.inverted(HAL.CLIMBER_INVERT);

        this.tempClimber = climberNeoMotor.getMotorTemperature();
        this.appliedVoltsClimber = climberNeoMotor.getAppliedOutput();

        // PID
        climberPID = new PIDController(
            HAL.CLIMBER_PID_P,
            HAL.CLIMBER_PID_I,
            HAL.CLIMBER_PID_D
        );
        
        // Encoder Position
        climberEncoder.setPosition(HAL.ENCODER_POSTION);

        climberNeoMotor.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        climberEncoder.getPosition();
        climberEncoder.getVelocity();
        climberNeoMotor.getOutputCurrent();

        inputs.tempClimber = climberNeoMotor.getMotorTemperature();
        inputs.appliedVoltsClimber = climberNeoMotor.getAppliedOutput();
    }

    @Override
    public Command runClimber(double voltage) {
        return Commands.runOnce(
            () -> {
                climberNeoMotor.setVoltage(voltage);
            }
        );
    }
    
    @Override
    public void stopMotors() {
        climberNeoMotor.setVoltage(0);
    }
}
