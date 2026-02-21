package frc.robot.subsystems.hopper;

import static frc.robot.utils.SparkUtil.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.hopper.HopperConstants.HAL;
import frc.robot.subsystems.hopper.HopperConstants.TUNING;

public class HopperIOSpark implements HopperIO{
    private final SparkMax hopperSparkMax;
    private final RelativeEncoder hopperEncoder;
    private final SparkClosedLoopController hopperController;
    private final SparkBaseConfig hopperConfig;
    
    public HopperIOSpark(){
        this.hopperSparkMax = new SparkMax(50, MotorType.kBrushless);
        this.hopperEncoder = hopperSparkMax.getEncoder();
        this.hopperController = hopperSparkMax.getClosedLoopController();
        this.hopperConfig = new SparkMaxConfig();
        
        hopperConfig
            .idleMode(IdleMode.kCoast)
            .inverted(HAL.HOPPER_INVERT);

        hopperConfig
            .smartCurrentLimit(TUNING.CLIMBER_STALL_LIMIT, TUNING.CLIMBER_FREE_LIMIT)
            .voltageCompensation(12.0);
        
        hopperConfig.encoder
            .positionConversionFactor((1/HAL.HOPPER_GEARING) * Math.PI * 2)
            .velocityConversionFactor((1/HAL.HOPPER_GEARING) * Math.PI * 2 / 60)
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(4);
        
        hopperConfig.signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(100) // dont need this to be 20ms
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20); // might need to detect jam

        // To reduce CAN utilization
        hopperConfig.signals
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

        hopperConfig.closedLoop
            .p(TUNING.HOPPER_PID_P)
            .i(TUNING.HOPPER_PID_I)
            .d(TUNING.HOPPER_PID_D)
            .outputRange(-1.0, 1.0);

        tryUntilOk(
            hopperSparkMax,
            5,
            () -> hopperSparkMax.configure(
                hopperConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
            )
        );
        tryUntilOk(hopperSparkMax, 5, () -> hopperEncoder.setPosition(0.0));
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperVelocity = hopperEncoder.getVelocity();
        inputs.appliedVoltsHopper = hopperSparkMax.getBusVoltage() * hopperSparkMax.getAppliedOutput();
        inputs.supplyCurrentHopper = 0.0;
        inputs.tempHopper = hopperSparkMax.getMotorTemperature();
        inputs.hopperConnected = (hopperSparkMax.getLastError() == REVLibError.kOk);
    }

    @Override
    public Command runHopper(double velocity) {
        return Commands.run(
            () -> hopperController.setSetpoint(velocity, ControlType.kVelocity)
        ).withName("Hopper.runHopperVelocity");
    }

    @Override
    public void stopMotors() {
        hopperSparkMax.stopMotor();
    }
}
