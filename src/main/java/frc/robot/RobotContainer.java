// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.AutoDrive;
import frc.robot.subsystems.SuperStructure;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.drive.Drive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.SwerveTelemetry;
import frc.robot.subsystems.hopper.HopperIO;
import frc.robot.subsystems.hopper.HopperIOSim;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.utils.AutoAim;
import frc.robot.utils.AxisConfigLoader;

public class RobotContainer {
    private final CommandXboxController driverController = new CommandXboxController(0);
    private final SendableChooser<Command> autoChooser;
    private final SendableChooser<Boolean> rotationSysID = new SendableChooser<>();

    // Swerve
    private final Drive drivetrain = TunerConstants.createDrivetrain();
    private final SwerveTelemetry swerveLogger = new SwerveTelemetry();     
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();

    private final Trigger autoAim = driverController.rightBumper().debounce(0.02);
    private final AutoAim calculationUtil = new AutoAim(drivetrain);

    private final DriveCommand teleDrive = new DriveCommand(
        drivetrain,
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightX(),
        () -> autoAim.getAsBoolean(),
        AxisConfigLoader.loadTable(GlobalConstants.LEFT_AXIS_CONFIG),
        AxisConfigLoader.loadTable(GlobalConstants.RIGHT_AXIS_CONFIG),
        () -> calculationUtil.getAimParams()
    );

    // SuperStructure    
    private final ShooterIO shooterIO = Utils.isSimulation() ? new ShooterIOSim() : new ShooterIOSim();
    private final IntakeIO intakeIO = Utils.isSimulation() ? new IntakeIOSim() : new IntakeIOSim();
    private final HopperIO hopperIO = Utils.isSimulation() ? new HopperIOSim() : new HopperIOSim();
    private final ClimberIO climberIO = Utils.isSimulation() ? new ClimberIOSim() : new ClimberIOSim();
    private final SuperStructure upperParts = new SuperStructure(shooterIO, intakeIO, hopperIO, climberIO);

    private final SysIdRoutine shooterRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            null, null, null, // Use default config
            (state) -> Logger.recordOutput("shooterRoutine", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (voltage) -> shooterIO.runShooterVoltage(voltage.in(Volts)),
            null, // No log consumer, since data is recorded by AdvantageKit
            upperParts
        )
    );

    // Auto Aim Calculation
    

    public RobotContainer() {
        drivetrain.registerTelemetry(swerveLogger::telemeterize);

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
            autoChooser.addOption("Shooter SysID | DF", shooterRoutine.dynamic(Direction.kForward));
            autoChooser.addOption("Shooter SysID | DR", shooterRoutine.dynamic(Direction.kReverse));
            autoChooser.addOption("Shooter SysID | QF", shooterRoutine.quasistatic(Direction.kForward));
            autoChooser.addOption("Shooter SysID | QR", shooterRoutine.quasistatic(Direction.kReverse));
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

        /*/
        driverController.leftTrigger(0.15).whileTrue(
            upperParts.runIntake()
        );

        driverController.b().whileTrue(
            upperParts.openClimber() 
        );

        driverController.a().whileTrue(
            upperParts.closeClimber()
        ); 

        autoAim.whileTrue(
            upperParts.runShooter(
                () -> calculationUtil.getAimParams().pitch
            )
        );
        */

        // TODO drivetest
        driverController.a().onTrue(new AutoDrive(drivetrain).withTarget(new Pose2d(5, 5, Rotation2d.fromDegrees(0))));
        driverController.x().onTrue(
            drivetrain.runOnce(
                () -> drivetrain.resetPose(new Pose2d(2.5, 4.5, Rotation2d.kZero)) // BLUE
            )
        );
        driverController.b().onTrue(
            drivetrain.runOnce(
                () -> drivetrain.resetPose(new Pose2d(14, 4.5, Rotation2d.k180deg)) // RED
            )
        );
        //#endregion
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
