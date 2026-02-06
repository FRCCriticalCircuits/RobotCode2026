package frc.robot.subsystems.intake;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.GlobalConstants;
import frc.robot.subsystems.intake.IntakeConstants.HAL;

public class IntakeIOKraken implements IntakeIO{
    private final TalonFX armMotor, rollerMotor;
    private final TalonFXConfiguration armConfig, rollerConfig;
    // Status Signals

    //Arm
    private final StatusSignal<Angle> armPosition; 
    private final StatusSignal<Voltage> appliedVoltsArm;
    private final StatusSignal<Current> supplyCurrentArm;
    private final StatusSignal<TorqueCurrentFOC> torqueCurrentArm;
    private final StatusSignal<Temperature> tempArm;


    //Roller
    private final StatusSignal<AngularVelocity> rollerVelocity;
    private final StatusSignal<Voltage> appliedVoltsRoller;
    private final StatusSignal<Current> supplyCurrentRoller;
    private final StatusSignal<TorqueCurrentFOC> torqueCurrentRoller;
    private final StatusSignal<Temperature> tempRoller;

    public IntakeIOKraken(){
        this.armMotor = new TalonFX(61, GlobalConstants.CARNIVORE); // Temp ID
        this.rollerMotor = new TalonFX(62, GlobalConstants.CARNIVORE); // Temp ID

        this.armConfig = new TalonFXConfiguration();
        this.rollerConfig = new TalonFXConfiguration();

        this.armConfig.Slot0.kP = HAL.ARM_PID_P;
        this.armConfig.Slot0.kI = HAL.ARM_PID_I;
        this.armConfig.Slot0.kD = HAL.ARM_PID_D;

        this.rollerConfig.Slot0.kP = HAL.ROLLER_PID_P;
        this.rollerConfig.Slot0.kI = HAL.ROLLER_PID_I;
        this.rollerConfig.Slot0.kD = HAL.ROLLER_PID_D;

        this.rollerConfig.
    }
}
