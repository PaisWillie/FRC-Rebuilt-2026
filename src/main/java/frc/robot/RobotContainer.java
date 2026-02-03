// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
    final CommandPS5Controller m_driverController = new CommandPS5Controller(Constants.DRIVER_CONTROLLER_PORT);

    private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> m_driverController.getLeftY() * -1,
            () -> m_driverController.getLeftX() * -1)
            .withControllerRotationAxis(m_driverController::getRightX)
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);

    /**
     * Clone's the angular velocity input stream and converts it to a fieldRelative
     * input stream.
     */
    SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
            .withControllerHeadingAxis(m_driverController::getRightX,
                    m_driverController::getRightY)
            .headingWhile(true);

    /**
     * Clone's the angular velocity input stream and converts it to a robotRelative
     * input stream.
     */
    SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
            .allianceRelativeControl(false);

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
            () -> -m_driverController.getLeftY(),
            () -> -m_driverController.getLeftX())
            .withControllerRotationAxis(() -> m_driverController.getRawAxis(
                    2))
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(0.8)
            .allianceRelativeControl(true);
    // Derive the heading axis with math!
    SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
            .withControllerHeadingAxis(() -> Math.sin(
                    m_driverController.getRawAxis(
                            2) *
                            Math.PI)
                    *
                    (Math.PI *
                            2),
                    () -> Math.cos(
                            m_driverController.getRawAxis(
                                    2) *
                                    Math.PI)
                            *
                            (Math.PI *
                                    2))
            .headingWhile(true)
            .translationHeadingOffset(true)
            .translationHeadingOffset(Rotation2d.fromDegrees(
                    0));

    Pose2d blueHub = new Pose2d(4.0218614 + Units.inchesToMeters(47.0) / 2.0, 8.069 / 2, null);

    private DoubleSupplier headingXSupplier() {
        return () -> {
            Pose2d robotPose = drivebase.getSwerveDrive().getPose();
            Translation2d delta = blueHub.getTranslation().minus(robotPose.getTranslation());
            Rotation2d heading = delta.getAngle().minus(Rotation2d.fromDegrees(90));
            return heading.getCos();
        };
    }

    private DoubleSupplier headingYSupplier() {
        return () -> {
            Pose2d robotPose = drivebase.getSwerveDrive().getPose();
            Translation2d delta = blueHub.getTranslation().minus(robotPose.getTranslation());
            Rotation2d heading = delta.getAngle().minus(Rotation2d.fromDegrees(90));
            return -heading.getSin();
        };
    }

    SwerveInputStream driveAutoAim = driveAngularVelocity.copy()
            .withControllerHeadingAxis(headingXSupplier(), headingYSupplier())
            .headingWhile(true);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
        Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAutoAim = drivebase.driveFieldOriented(driveAutoAim);

        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        m_driverController.R1().whileTrue(driveFieldOrientedAutoAim);

        if (Robot.isSimulation()) {
            Pose2d target = new Pose2d(new Translation2d(1, 4),
                    Rotation2d.fromDegrees(90));
            // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
            driveDirectAngleKeyboard.driveToPose(() -> target,
                    new ProfiledPIDController(5,
                            0,
                            0,
                            new Constraints(5, 2)),
                    new ProfiledPIDController(5,
                            0,
                            0,
                            new Constraints(Units.degreesToRadians(360),
                                    Units.degreesToRadians(180))));
            m_driverController.options()
                    .onTrue(Commands.runOnce(() -> drivebase
                            .resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
            m_driverController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
            m_driverController.button(2)
                    .whileTrue(Commands.runEnd(
                            () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                            () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

        }
        if (DriverStation.isTest()) {
            drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command
                                                                             // above!

            m_driverController.square().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
            m_driverController.options().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            m_driverController.create().whileTrue(drivebase.centerModulesCommand());
            m_driverController.L1().onTrue(Commands.none());
        } else {
            m_driverController.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
            m_driverController.options().whileTrue(Commands.none());
            m_driverController.create().whileTrue(Commands.none());
            m_driverController.L1()
                    .whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
        }
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
