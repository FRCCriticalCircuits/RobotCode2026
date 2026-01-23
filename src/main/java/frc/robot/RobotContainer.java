// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoDrive;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.CommandSwerveDrivetrain;
import frc.robot.subsystems.drive.SwerveTelemetry;

public class RobotContainer {
    //#region Swerve Constants
    private double MaxSpeed = 0.2 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = 0.2 * RotationsPerSecond.of(0.5).in(RadiansPerSecond);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.2); // Add a 10% deadband

    private final SwerveTelemetry swerveLogger = new SwerveTelemetry();
    private final SendableChooser<Boolean> rotationSysID = new SendableChooser<>();
    //#endregion

    private final CommandXboxController driverController = new CommandXboxController(0);
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    public RobotContainer() {
        // swerve
        drivetrain.registerTelemetry(swerveLogger::telemeterize);
        
        autoChooser.setDefaultOption("null", Commands.print("default command"));

        //#region SysID Autos
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
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-driverController.getLeftX() * MaxSpeed)
                    .withRotationalRate(-driverController.getRightX() * MaxAngularRate)
            )
        );

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverController.a().debounce(0.02).onTrue(
            new AutoDrive(
                drivetrain, 
                new Pose2d(
                    5,
                    5,
                    Rotation2d.kZero
                )
            )
        );

        // Reset the field-centric heading on left bumper press.
        driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
