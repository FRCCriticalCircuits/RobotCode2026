// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.SwerveTelemetry;
import frc.robot.generated.TunerConstants;

import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionLimelight;

import frc.robot.commands.*;
import frc.robot.utils.calc.AimCalc;
import frc.robot.utils.calc.ClimbCalc;
import frc.robot.utils.axis.AxisConfigLoader;

import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.climber.*;
import frc.robot.subsystems.intake.*;
import frc.robot.subsystems.hopper.*;
import frc.robot.subsystems.shooter.*;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Boolean> rotationSysID = new SendableChooser<>();

    // Swerve
    private final Drive drivetrain = TunerConstants.createDrivetrain();
    private final SwerveTelemetry swerveLogger = new SwerveTelemetry();     
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    // AutoAim
    private final Trigger autoAimTrigger = driverController.rightTrigger(0.15).debounce(0.02);
    
    private final AimCalc autoAimCalc = new AimCalc(drivetrain);
    private final ClimbCalc climbCalc = new ClimbCalc(drivetrain);

    private final DriveCommand teleDrive = new DriveCommand(
        drivetrain,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> autoAimTrigger.getAsBoolean(),
        AxisConfigLoader.loadTable(GlobalConstants.LEFT_AXIS_CONFIG),
        AxisConfigLoader.loadTable(GlobalConstants.RIGHT_AXIS_CONFIG),
        () -> autoAimCalc.getAimParams()
    );

    // need this separate command because
    // teleDrive cant work with pathplanner (NamedCommands)
    // it's the default command of drive subsystem
    private final PPDriveCommand pathplannerDrive = new PPDriveCommand(
        drivetrain,
        () -> autoAimCalc.getAimParams()
    );

    // Vision
    private final VisionLimelight ll3 = new VisionLimelight("limelight");
    @SuppressWarnings("unused")
    private final Vision visionSubsystem = new Vision(drivetrain, ll3);

    // SuperStructure    
    private final ShooterIO shooterIO = Utils.isSimulation() ? new ShooterIOSim() : new ShooterIOKraken();
    private final IntakeIO intakeIO = Utils.isSimulation() ? new IntakeIOSim() : new IntakeIOKraken();
    private final HopperIO hopperIO = Utils.isSimulation() ? new HopperIO() {} : new HopperIOKraken();
    private final ClimberIO climberIO = Utils.isSimulation() ? new ClimberIO(){} : new ClimberIOVortex();
    private final SuperStructure upperParts = new SuperStructure(shooterIO, intakeIO, hopperIO, climberIO);

    // SysID Routine for flywheels(shooter/hopper/intake)
    private final SysIdRoutine flywheelRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Units.Volts.of(0.05).per(Units.Second),
            Voltage.ofRelativeUnits(5, Units.Volts),
            null,
            (state) -> Logger.recordOutput("flywheelRoutine", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            // Change this line to characterize other motors
            (voltage) -> shooterIO.runShooterVoltage(voltage.in(Volts)), 
            null, // No log consumer, since data is recorded by AdvantageKit
            upperParts
        )
    );

    // SysID Routine for arm, ramp rate and voltage unverified
    private final SysIdRoutine armRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Units.Volts.of(0.08).per(Units.Second),
            Voltage.ofRelativeUnits(0.4, Units.Volts),
            null,
            (state) -> Logger.recordOutput("armRoutine", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            // Change this line to characterize other motors
            (voltage) -> intakeIO.runArmVoltage(voltage.in(Volts)), 
            null, // No log consumer, since data is recorded by AdvantageKit
            upperParts
        )
    );

    // SysID Routine for hood, ramp rate and voltage verified
    private final SysIdRoutine hoodRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Units.Volts.of(0.1).per(Units.Second),
            Voltage.ofRelativeUnits(0.3, Units.Volts),
            null,
            (state) -> Logger.recordOutput("hoodRoutine", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            // Change this line to characterize other motors
            (voltage) -> shooterIO.runHoodVoltage(voltage.in(Volts)), 
            null, // No log consumer, since data is recorded by AdvantageKit
            upperParts
        )
    );

    // Commands
    private final AutoDrive autoDrive = new AutoDrive(drivetrain);
    private final Command autoIntakeCommand = upperParts.runIntake();
    private final Command autoShooterCommand = Commands.parallel(
        pathplannerDrive,
        upperParts.runShooter(() -> autoAimCalc.getAimParams())
    );

    public RobotContainer() {
        drivetrain.registerTelemetry(swerveLogger::telemeterize);

        NamedCommands.registerCommand("runIntake", autoIntakeCommand);
        
        NamedCommands.registerCommand(
            "stopIntake",
            Commands.runOnce(
                () -> CommandScheduler.getInstance().cancel(autoIntakeCommand)
            )
        );

        NamedCommands.registerCommand("runShooter", autoShooterCommand);

        NamedCommands.registerCommand(
            "stopShooter",
            Commands.runOnce(
                () -> {
                    CommandScheduler.getInstance().cancel(autoShooterCommand);
                }
            )
        );

        // NamedCommands.registerCommand(
        //     "drive27",
        //     new AutoDrive(drivetrain).withTarget(new Pose2d(2, 7, Rotation2d.fromDegrees(0)))
        // );

        // This will use Commands.none() as the default option.
        // Displayed as "None"
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
            (stream) -> GlobalConstants.COMP
                ? stream.filter(auto -> auto.getName().startsWith("comp_"))
                : stream
        );

        //#region SysID
        if(GlobalConstants.SYS_ID_SWERVE && !GlobalConstants.COMP){
            rotationSysID.setDefaultOption("Rotation", true);
            rotationSysID.addOption("Translation", false);
            SmartDashboard.putData("SysID Option", rotationSysID);

            autoChooser.addOption(
                "SysID | Dynamic Forward", 
                drivetrain.sysIdDynamic(Direction.kForward, rotationSysID.getSelected())
            );

            autoChooser.addOption(
                "SysID | Dynamic Reverse", 
                drivetrain.sysIdDynamic(Direction.kReverse, rotationSysID.getSelected())
            );

            autoChooser.addOption(
                "SysID | Quasistatic Forward", 
                drivetrain.sysIdQuasistatic(Direction.kForward, rotationSysID.getSelected())
            );

            autoChooser.addOption(
                "SysID | Quasistatic Reverse", 
                drivetrain.sysIdQuasistatic(Direction.kReverse, rotationSysID.getSelected())
            );
        }

        if(GlobalConstants.SYS_ID_FLYWHEELS && !GlobalConstants.COMP){
            autoChooser.addOption("Flywheel SysID | DF", flywheelRoutine.dynamic(Direction.kForward));
            autoChooser.addOption("Flywheel SysID | DR", flywheelRoutine.dynamic(Direction.kReverse));
            autoChooser.addOption("Flywheel SysID | QF", flywheelRoutine.quasistatic(Direction.kForward));
            autoChooser.addOption("Flywheel SysID | QR", flywheelRoutine.quasistatic(Direction.kReverse));

            autoChooser.addOption("Arm SysID | DF", armRoutine.dynamic(Direction.kForward));
            autoChooser.addOption("Arm SysID | DR", armRoutine.dynamic(Direction.kReverse));
            autoChooser.addOption("Arm SysID | QF", armRoutine.quasistatic(Direction.kForward));
            autoChooser.addOption("Arm SysID | QR", armRoutine.quasistatic(Direction.kReverse));

            autoChooser.addOption("Hood SysID | DF", hoodRoutine.dynamic(Direction.kForward));
            autoChooser.addOption("Hood SysID | DR", hoodRoutine.dynamic(Direction.kReverse));
            autoChooser.addOption("Hood SysID | QF", hoodRoutine.quasistatic(Direction.kForward));
            autoChooser.addOption("Hood SysID | QR", hoodRoutine.quasistatic(Direction.kReverse));
        }
        //#endregion

        drivetrain.setDefaultCommand(teleDrive);

        SmartDashboard.putData("Auto to Run", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        //#region Bindings
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Reset the field-centric heading on left bumper press.
        driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        driverController.a().debounce(0.02).onTrue(
            new InstantCommand(
                () -> visionSubsystem.setVisionEnabled()
            )
        );

        driverController.leftTrigger(0.15).debounce(0.02).whileTrue(
            upperParts.runIntake()
        );

        autoAimTrigger.whileTrue(
            upperParts.runShooter(() -> autoAimCalc.getAimParams())
        );

        driverController.povUp().whileTrue(
            upperParts.openClimber() 
        );

        driverController.povDown().whileTrue(
            upperParts.closeClimber()
        ); 

        // TODO integrate climb calc
        driverController.x().whileTrue(
            autoDrive.withTarget(() -> climbCalc.nearestClimbPos()).finallyDo(
                () -> {
                    CommandScheduler.getInstance().schedule(
                        autoDrive.withTarget(
                            () -> drivetrain.getState().Pose.plus(
                                new Transform2d(0.05, 0, Rotation2d.kZero)
                            )
                        ).withTimeout(1.0)
                    );
                }
            )
        );
        //#endregion
    }

    /**
     * execute the command, then re-enable vision
     * @return
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected().andThen(() -> visionSubsystem.setVisionEnabled());
    }
}
