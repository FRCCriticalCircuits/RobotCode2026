package frc.robot.subsystems.climber;

import static frc.robot.utils.SparkUtil.*;
import frc.robot.subsystems.climber.ClimberConstants.*;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class ClimberIOVortex implements ClimberIO{
    private final SparkFlex climberSparkFlex;
    private final RelativeEncoder climberEncoder;
    @SuppressWarnings("unused")
    private final SparkClosedLoopController climberController;
    private final SparkFlexConfig climberConfig;
    
    // Control Requests
    public ClimberIOVortex(){
        this.climberSparkFlex = new SparkFlex(63, MotorType.kBrushless);        
        this.climberEncoder = climberSparkFlex.getEncoder();
        this.climberController = climberSparkFlex.getClosedLoopController();
        this.climberConfig = new SparkFlexConfig();

        climberConfig
            .idleMode(IdleMode.kCoast)
            .inverted(HAL.CLIMBER_INVERT);

        climberConfig
            .smartCurrentLimit(TUNING.CLIMBER_STALL_LIMIT, TUNING.CLIMBER_FREE_LIMIT)
            .voltageCompensation(12.0);
        
        climberConfig.encoder
            .positionConversionFactor((1 / HAL.CLIMBER_GEARING) * Math.PI * 2)
            .velocityConversionFactor((1 / HAL.CLIMBER_GEARING) * Math.PI * 2 / 60)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(4);
        
        climberConfig.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(20)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(100) // dont need this to be 20ms
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20); // might need to detect jam

        // To reduce CAN utilization
        climberConfig.signals
            .limitsPeriodMs(300)
            .setSetpointAlwaysOn(false)
            .isAtSetpointAlwaysOn(false)
            .selectedSlotAlwaysOn(false)
            .analogVoltageAlwaysOn(false)
            .iAccumulationAlwaysOn(false)
            .analogPositionAlwaysOn(false)
            .analogVelocityAlwaysOn(false)
            .absoluteEncoderPositionAlwaysOn(false)
            .absoluteEncoderVelocityAlwaysOn(false)
            .maxMotionSetpointPositionAlwaysOn(false)
            .maxMotionSetpointVelocityAlwaysOn(false);

        climberConfig.closedLoop
            .p(TUNING.CLIMBER_PID_P)
            .i(TUNING.CLIMBER_PID_I)
            .d(TUNING.CLIMBER_PID_D)
            .outputRange(-1.0, 1.0);

        tryUntilOk(
            climberSparkFlex,
            5,
            () -> climberSparkFlex.configure(
                climberConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
            )
        );
        tryUntilOk(climberSparkFlex, 5, () -> climberEncoder.setPosition(0.0));
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberPosition = climberEncoder.getPosition();
        inputs.appliedVoltsClimber = climberSparkFlex.getBusVoltage() * climberSparkFlex.getAppliedOutput();
        inputs.supplyCurrentClimber = 0.0;
        inputs.tempClimber = climberSparkFlex.getMotorTemperature();
        inputs.climberConnected = (climberSparkFlex.getLastError() == REVLibError.kOk);
    }

    @Override
    public Command runClimber(double voltage) {
        return Commands.run(
            () -> climberSparkFlex.setVoltage(voltage)
        );
    }
    
    @Override
    public void stopMotors() {
        climberSparkFlex.stopMotor();
    }
}
