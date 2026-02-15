// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import java.io.File;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
    final CommandPS5Controller m_driverController = new CommandPS5Controller(Constants.DRIVER_CONTROLLER_PORT);

    private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
    private final HopperSubsystem m_hopperSubsystem = new HopperSubsystem();
    private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
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

    SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
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

    private DoubleSupplier headingXSupplier() {
        return () -> {
            Translation2d robotTranslation = m_swerveSubsystem.getSwerveDrive().getPose().getTranslation();
            Translation2d delta = FieldConstants.BLUE_HUB_CENTER.minus(robotTranslation);
            Rotation2d heading = delta.getAngle().minus(Rotation2d.fromDegrees(90));
            return heading.getCos();
        };
    }

    private DoubleSupplier headingYSupplier() {
        return () -> {
            Translation2d robotTranslation = m_swerveSubsystem.getSwerveDrive().getPose().getTranslation();
            Translation2d delta = FieldConstants.BLUE_HUB_CENTER.minus(robotTranslation);
            Rotation2d heading = delta.getAngle().minus(Rotation2d.fromDegrees(90));
            return -heading.getSin();
        };
    }

    SwerveInputStream driveAutoAim = driveAngularVelocity.copy()
            .withControllerHeadingAxis(headingXSupplier(), headingYSupplier())
            .headingWhile(true);

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        configureBindings();
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
        Command driveFieldOrientedDirectAngleKeyboard = m_swerveSubsystem
                .driveFieldOriented(driveDirectAngleKeyboard);
        Command driveFieldOrientedAutoAim = m_swerveSubsystem.driveFieldOriented(driveAutoAim);

        m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        // if (Robot.isSimulation()) {
        // Pose2d target = new Pose2d(new Translation2d(1, 4),
        // Rotation2d.fromDegrees(90));
        // // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
        // driveDirectAngleKeyboard.driveToPose(() -> target,
        // new ProfiledPIDController(5,
        // 0,
        // 0,
        // new Constraints(5, 2)),
        // new ProfiledPIDController(5,
        // 0,
        // 0,
        // new Constraints(Units.degreesToRadians(360),
        // Units.degreesToRadians(180))));
        // m_driverController.start()
        // .onTrue(Commands.runOnce(() -> m_swerveSubsystem
        // .resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
        // m_driverController.button(1).whileTrue(m_swerveSubsystem.sysIdDriveMotorCommand());
        // m_driverController.button(2)
        // .whileTrue(Commands.runEnd(
        // () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
        // () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

        // }
        // if (DriverStation.isTest()) {
        // m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity); //
        // Overrides drive
        // // command
        // // above!

        // m_driverController.square()
        // .whileTrue(Commands.runOnce(m_swerveSubsystem::lock, m_swerveSubsystem)
        // .repeatedly());
        // m_driverController.options().onTrue((Commands.runOnce(m_swerveSubsystem::zeroGyro)));
        // m_driverController.create().whileTrue(m_swerveSubsystem.centerModulesCommand());
        // m_driverController.L1().onTrue(Commands.none());
        // } else {

        // m_driverController.L1()
        // .whileTrue(Commands.runOnce(m_swerveSubsystem::lock, m_swerveSubsystem)
        // .repeatedly());

        m_driverController.povUp().whileTrue(m_swerveSubsystem.driveForward());
        m_driverController.povDown().whileTrue(m_swerveSubsystem.driveBackward());
        m_driverController.povLeft().whileTrue(m_swerveSubsystem.driveLeft());
        m_driverController.povRight().whileTrue(m_swerveSubsystem.driveRight());

        m_driverController.options().onTrue((Commands.runOnce(m_swerveSubsystem::zeroGyro)));

        m_driverController.L1().whileTrue(driveFieldOrientedAutoAim);
        // m_driverController.R1().whileTrue(
        // m_shooterSubsystem.aimAndShoot(m_swerveSubsystem::getDistanceFromHub));

        // m_driverController.R1().whileTrue(
        // m_shooterSubsystem.m_hoodSubsystem.setAngle(
        // Degrees.of(m_shooterSubsystem.m_distanceToHoodAngleMap
        // .get(m_swerveSubsystem::getDistanceFromHub.get().in(Meters)))));

        // m_driverController.R1().whileTrue(
        // m_shooterSubsystem.m_hoodSubsystem.setAngle(
        // () -> {
        // return Degrees.of(m_shooterSubsystem.m_distanceToHoodAngleMap
        // .get(m_swerveSubsystem.getDistanceFromHub().in(Meters)));
        // }));

        m_driverController.R1().whileTrue(m_shooterSubsystem.aimAndShoot(m_swerveSubsystem::getDistanceToHub))
                .onFalse(m_shooterSubsystem.stopShooting());

        m_driverController.cross().whileTrue(m_shooterSubsystem.runFlywheel());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
