package frc.robot.subsystems.hopper;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.hopper.HopperConstants.HAL;

public class HopperIOKraken implements HopperIO{
    private final TalonFX hopperMotor;

    // Status Signals
    private final StatusSignal<AngularVelocity> hopperVelocity;
    private final StatusSignal<Voltage> appliedVoltsHopper;
    private final StatusSignal<Current> supplyCurrentHopper;
    private final StatusSignal<Current> torqueCurrentHopper;
    private final StatusSignal<Temperature> tempHopper;
        
    // Configuration
    private final TalonFXConfiguration hopperConfig;

    // Control Requests
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0)
        .withUpdateFreqHz(0.0);

    public HopperIOKraken(){
        this.hopperMotor = new TalonFX(60, GlobalConstants.CARNIVORE); //Need to set a new ID for hopper

        //Configuration
        this.hopperConfig = new TalonFXConfiguration();

        this.hopperConfig.Slot0.kP = HAL.HOPPER_PID_P;
        this.hopperConfig.Slot0.kI = HAL.HOPPER_PID_I;
        this.hopperConfig.Slot0.kD = HAL.HOPPER_PID_D;
        this.hopperConfig.MotorOutput.Inverted = HAL.HOPPER_INVERT
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

        this.hopperConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        this.hopperConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        this.hopperConfig.Feedback.RotorToSensorRatio = HAL.HOPPER_GEARING;

        // Status Signals
        this.hopperVelocity = hopperMotor.getVelocity();
        this.appliedVoltsHopper = hopperMotor.getMotorVoltage();

        this.supplyCurrentHopper = hopperMotor.getSupplyCurrent();
        this.torqueCurrentHopper = hopperMotor.getTorqueCurrent();

        this.tempHopper = hopperMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0, 
            this.hopperVelocity,
            this.appliedVoltsHopper,
            this.supplyCurrentHopper,
            this.torqueCurrentHopper,
            this.tempHopper
        );

        hopperMotor.optimizeBusUtilization(1.0);
        
        hopperMotor.getConfigurator().apply(hopperConfig);
    }

    @Override
    public void updateInputs(HopperIOInputs inputs){
        inputs.hopperConnected = BaseStatusSignal.refreshAll(
            this.hopperVelocity,
            this.appliedVoltsHopper,
            this.supplyCurrentHopper,
            this.torqueCurrentHopper,
            this.tempHopper
        ).isOK();

        inputs.hopperVelocity = this.hopperVelocity.getValueAsDouble() * Math.PI * 2;
        inputs.appliedVoltsHopper = this.appliedVoltsHopper.getValueAsDouble();
        inputs.supplyCurrentHopper = this.supplyCurrentHopper.getValueAsDouble();
        inputs.torqueCurrentHopper = this.torqueCurrentHopper.getValueAsDouble();
        inputs.tempHopper = this.tempHopper.getValueAsDouble();
    }

    @Override
    public Command runHopper(double velocity){
        return Commands.runOnce(
            () -> {
                hopperMotor.setControl(
                    velocityVoltage.withVelocity(velocity)
                );
            }
        ).withName("Hopper.runHopperVelocity");
    }

    @Override
    public void stopMotors(){
        hopperMotor.setControl(new NeutralOut());
    }
}
