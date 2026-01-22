// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.SwerveTelemetry;

public class RobotContainer {
    //#region swerve
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Declare variables so GC will not be used for them */
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveTelemetry swerveLogger = new SwerveTelemetry();
    private final SendableChooser<Boolean> rotationSysID = new SendableChooser<>();
    //#endregion

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        autoChooser.setDefaultOption("null", Commands.print("default command"));

        //#region sysID
        if(GlobalConstants.SYS_ID){
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
        //#endregion

        SmartDashboard.putData("Auto to Run", autoChooser);
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(swerveLogger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
