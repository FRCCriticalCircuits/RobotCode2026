package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.intake.IntakeConstants.HAL;

public class IntakeIOKraken implements IntakeIO{
    private final TalonFX armMotor, rollerMotor, secondaryArmMotor;
    
    // Status Signals
    private final StatusSignal<Angle> armPosition; 
    private final StatusSignal<Voltage> appliedVoltsArm;
    private final StatusSignal<Current> supplyCurrentArm;
    private final StatusSignal<Current> torqueCurrentArm;
    private final StatusSignal<Temperature> tempArm;
    private final StatusSignal<Temperature> tempSecondaryArm;

    // Roller
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> appliedVoltsRoller;
    private final StatusSignal<Current> supplyCurrentRoller;
    private final StatusSignal<Current> torqueCurrentRoller;
    private final StatusSignal<Temperature> tempRoller;

    // Configuration
     private final TalonFXConfiguration armConfig, rollerConfig;

    // Control Requests
    private final PositionTorqueCurrentFOC armPostionFOC = new PositionTorqueCurrentFOC(0)
        .withUpdateFreqHz(0.0);
    private final VelocityVoltage rollerVelocityVoltage = new VelocityVoltage(0.0)
        .withEnableFOC(true)
        .withUpdateFreqHz(0.0);

    public IntakeIOKraken(){
        this.armMotor = new TalonFX(61, GlobalConstants.CARNIVORE); // Temp ID //TODO
        this.rollerMotor = new TalonFX(62, GlobalConstants.CARNIVORE); // Temp ID //TODO
        this.secondaryArmMotor = new TalonFX(64, GlobalConstants.CARNIVORE);

        // Configuration
        this.armConfig = new TalonFXConfiguration();
        this.rollerConfig = new TalonFXConfiguration();
    
        this.armConfig.Slot0.kP = HAL.ARM_PID_P;
        this.armConfig.Slot0.kI = HAL.ARM_PID_I;
        this.armConfig.Slot0.kD = HAL.ARM_PID_D;

        this.armConfig.MotorOutput.Inverted = 
            HAL.ARM_INVERT
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        this.armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        this.armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        this.armConfig.Feedback.RotorToSensorRatio = HAL.ARM_GEARING;

        this.rollerConfig.Slot0.kP = HAL.ROLLER_PID_P;
        this.rollerConfig.Slot0.kI = HAL.ROLLER_PID_I;
        this.rollerConfig.Slot0.kD = HAL.ROLLER_PID_D;
        this.rollerConfig.MotorOutput.Inverted =
            HAL.ROLLER_INVERT
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        this.rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        this.rollerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        this.rollerConfig.Feedback.RotorToSensorRatio = HAL.ROLLER_GEARING;

        /* Default Supply Limit */
        this.armConfig.TorqueCurrent.PeakForwardTorqueCurrent = 30.0;
        this.armConfig.TorqueCurrent.PeakReverseTorqueCurrent = -30.0;

        // Status Signals
        this.armPosition = armMotor.getPosition();
        this.appliedVoltsArm = armMotor.getMotorVoltage();
        this.supplyCurrentArm = armMotor.getSupplyCurrent();
        this.torqueCurrentArm = armMotor.getTorqueCurrent();
        this.tempArm = armMotor.getDeviceTemp();
        this.tempSecondaryArm = secondaryArmMotor.getDeviceTemp();

        this.rollerVelocity = rollerMotor.getVelocity();
        this.appliedVoltsRoller = rollerMotor.getMotorVoltage();
        this.supplyCurrentRoller = rollerMotor.getSupplyCurrent();
        this.torqueCurrentRoller = rollerMotor.getSupplyCurrent();
        this.tempRoller = rollerMotor.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0, 
            this.armPosition,
            this.appliedVoltsArm,
            this.supplyCurrentArm,
            this.torqueCurrentArm,
            this.tempArm,
            this.rollerVelocity,
            this.appliedVoltsRoller,
            this.supplyCurrentRoller,
            this.torqueCurrentRoller,
            this.tempRoller,
            this.tempSecondaryArm
        );
        
        rollerMotor.optimizeBusUtilization(1.0);
        armMotor.optimizeBusUtilization(1.0);
        secondaryArmMotor.optimizeBusUtilization(1.0);

        rollerMotor.getConfigurator().apply(rollerConfig);
        armMotor.getConfigurator().apply(armConfig);
        secondaryArmMotor.getConfigurator().apply(armConfig);

        secondaryArmMotor.setControl(
            new Follower(armMotor.getDeviceID(), MotorAlignmentValue.Aligned) //TODO
        );
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs){
        inputs.rollerConnected = BaseStatusSignal.refreshAll(
            this.rollerVelocity,
            this.appliedVoltsRoller,
            this.supplyCurrentRoller,
            this.torqueCurrentRoller,
            this.tempRoller
        ).isOK();

        inputs.armConnected = BaseStatusSignal.refreshAll(
            this.armPosition,
            this.appliedVoltsArm,
            this.supplyCurrentArm,
            this.torqueCurrentArm,
            this.tempArm,
            this.tempSecondaryArm
        ).isOK();

        inputs.rollerVelocity = this.rollerVelocity.getValueAsDouble() * Math.PI * 2;
        inputs.armPosition = this.armPosition.getValueAsDouble() * Math.PI * 2;

        inputs.appliedVoltsRoller = this.appliedVoltsRoller.getValueAsDouble();
        inputs.supplyCurrentRoller = this.supplyCurrentRoller.getValueAsDouble();
        inputs.torqueCurrentRoller = this.torqueCurrentRoller.getValueAsDouble();
        inputs.tempRoller = this.tempRoller.getValueAsDouble();

        inputs.appliedVoltsArm = this.appliedVoltsArm.getValueAsDouble();
        inputs.supplyCurrentArm = this.supplyCurrentArm.getValueAsDouble();
        inputs.torqueCurrentArm = this.torqueCurrentArm.getValueAsDouble();
        inputs.tempArm = this.tempArm.getValueAsDouble();
        inputs.tempSecondaryArm = this.tempSecondaryArm.getValueAsDouble();
    }

    @Override
    public Command runArm(double positionRad){
        return Commands.runOnce(
            () -> {
                armMotor.setControl(
                    armPostionFOC.withPosition(positionRad)
                );
            }
        ).withName("Intake.runArm");
    }

    @Override
    public Command runRoller(double velocity) {
        return Commands.runOnce(
            () -> {
                rollerMotor.setControl(
                    rollerVelocityVoltage.withVelocity(velocity)  
                );
            }
        ).withName("Intake.runRoller");
    }

    @Override
    public void stopMotors() {
        rollerMotor.setControl(new NeutralOut());
        armMotor.setControl(new NeutralOut());
    }
}