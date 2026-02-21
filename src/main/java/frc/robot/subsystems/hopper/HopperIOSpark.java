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

public class HopperIOSpark implements HopperIO{
    private final SparkMax hopperSparkMax;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController controller;
    private final SparkBaseConfig config;

    public HopperIOSpark(){
        this.hopperSparkMax = new SparkMax(50, MotorType.kBrushless);
        this.encoder = hopperSparkMax.getEncoder();
        this.controller = hopperSparkMax.getClosedLoopController();
        this.config = new SparkMaxConfig();

        config.idleMode(IdleMode.kCoast);
            
        config
            .smartCurrentLimit(30, 20)
            .voltageCompensation(12.0);
        
        config.encoder
            .uvwMeasurementPeriod(10)
            .uvwAverageDepth(4);
        
        config
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs(100)
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20); // might need to detect jam

        config.closedLoop
            .p(HAL.HOPPER_PID_P)
            .i(HAL.HOPPER_PID_I)
            .d(HAL.HOPPER_PID_D)
            .outputRange(-1.0, 1.0);

        tryUntilOk(
            hopperSparkMax,
            5,
            () -> hopperSparkMax.configure(
                config,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters
            )
        );
        tryUntilOk(hopperSparkMax, 5, () -> encoder.setPosition(0.0));
    }

    @Override
    public void updateInputs(HopperIOInputs inputs) {
        inputs.hopperVelocity = encoder.getVelocity();
        inputs.appliedVoltsHopper = hopperSparkMax.getBusVoltage() * hopperSparkMax.getAppliedOutput();
        inputs.supplyCurrentHopper = 0.0;
        inputs.tempHopper = hopperSparkMax.getMotorTemperature();
        inputs.hopperConnected = (hopperSparkMax.getLastError() == REVLibError.kOk);
    }

    @Override
    public Command runHopper(double velocity) {
        return Commands.run(
            () -> controller.setSetpoint(velocity, ControlType.kVelocity)
        );
    }

    @Override
    public void stopMotors() {
        hopperSparkMax.stopMotor();
    }
}
