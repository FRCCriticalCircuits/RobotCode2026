package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOInputsAutoLogged;
import frc.robot.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOInputsAutoLogged;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOInputsAutoLogged;

public class SuperStructure extends SubsystemBase{
    public final Notifier notifier = new Notifier(this::run);

    public final ShooterIO shooterIO; 
    private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();
    private final Debouncer hoodConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer shooterConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert hoodDisconnected, shooterDisconnected;

    public final IntakeIO intakeIO;
    private final IntakeIOInputsAutoLogged intakeInputs = new IntakeIOInputsAutoLogged();
    private final Debouncer armConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Debouncer rollerConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert armDisconnected, rollerDisconnected;

    public final HopperIO hopperIO;
    private final HopperIOInputsAutoLogged hopperInputs = new HopperIOInputsAutoLogged();
    private final Debouncer hopperConnectedDebouncer = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
    private final Alert hopperDisconnected;

    public SuperStructure(ShooterIO shooterIO, IntakeIO intakeIO, HopperIO hopperIO){
        this.shooterIO = shooterIO;
        this.intakeIO = intakeIO;
        this.hopperIO = hopperIO;

        hoodDisconnected = new Alert("hood motor disconnected!", Alert.AlertType.kWarning);
        shooterDisconnected = new Alert("shooter motor disconnected!", Alert.AlertType.kWarning);
        armDisconnected = new Alert("arm motor disconnected!", Alert.AlertType.kWarning);
        rollerDisconnected = new Alert("roller motor disconnected!", Alert.AlertType.kWarning);
        hopperDisconnected = new Alert("hopper motor disconnected!", Alert.AlertType.kWarning);

        notifier.startPeriodic(100);
        notifier.setName("SuperStructure");
    }

    public void run() {
        shooterIO.updateInputs(shooterInputs);
        intakeIO.updateInputs(intakeInputs);
        hopperIO.updateInputs(hopperInputs);

        Logger.processInputs("ShooterIO", shooterInputs);
        Logger.processInputs("IntakeIO", intakeInputs);
        Logger.processInputs("HopperIO", hopperInputs);
        
        hoodDisconnected.set(!hoodConnectedDebouncer.calculate(shooterInputs.hoodConnected));
        shooterDisconnected.set(!shooterConnectedDebouncer.calculate(shooterInputs.shooterConnected));
        armDisconnected.set(!armConnectedDebouncer.calculate(intakeInputs.armConnected));
        rollerDisconnected.set(!rollerConnectedDebouncer.calculate(intakeInputs.rollerConnected));
        hopperDisconnected.set(!hopperConnectedDebouncer.calculate(hopperInputs.hopperConnected));

        visualize();
    }

    public Command runIntake(){
        return Commands.parallel(
            intakeIO.runArm(SuperStructureConstants.INTAKE_ARM_POS),
            intakeIO.runRoller(SuperStructureConstants.INTAKE_ROLLER_VEL),
            Commands.waitSeconds(999) // dont want this to end until interrupted (button unpressed)
        ).finallyDo(
            (interrupted) -> intakeIO.stopMotors()
        ).withName("SuperStructure.runIntake");
    }

    public Command runShooter(DoubleSupplier hoodPosition){
        return new ParallelCommandGroup(
            shooterIO.runShooter(SuperStructureConstants.SHOOT_FLYWHEEL_VEL),
            new RepeatCommand(shooterIO.runHood(hoodPosition)),
            Commands.waitUntil(shooterIO::isStable).andThen(hopperIO.runHopper(SuperStructureConstants.SHOOT_SEQUENCER_VEL))
        ).finallyDo(
            (interrupted) -> {
                shooterIO.stopMotors();
                hopperIO.stopMotors();
            }
        ).withName("SuperStructure.runShooter");
    }

    private void visualize(){
        Logger.recordOutput("Visualization/Hood", 
            new Pose3d(
                ShooterConstants.CAD.ORIGIN_OFFSET_X,
                ShooterConstants.CAD.ORIGIN_OFFSET_Y, 
                ShooterConstants.CAD.ORIGIN_OFFSET_Z,
                new Rotation3d(
                    0,
                    shooterInputs.hoodPosition + ShooterConstants.CAD.PITCH_OFFSET,
                    0
                )
            )
        );
        
        Logger.recordOutput("Visualization/Intake", 
            new Pose3d(
                IntakeConstants.CAD.ORIGIN_OFFSET_X, 
                IntakeConstants.CAD.ORIGIN_OFFSET_Y, 
                IntakeConstants.CAD.ORIGIN_OFFSET_Z,
                new Rotation3d(
                    0,
                    IntakeConstants.CAD.PITCH_OFFSET + intakeInputs.armPosition,
                    0
                )
            )
        );
    }
}
