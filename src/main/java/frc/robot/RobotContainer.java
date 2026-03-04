// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.io.File;
import java.util.Map;
import java.util.function.DoubleSupplier;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SimSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.Zone;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.LinearIntakeSubsystem;
import frc.robot.subsystems.intake.LinearIntakeSubsystem.LinearIntakePosition;
import swervelib.SwerveInputStream;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

public class RobotContainer {
    final CommandPS5Controller m_driverController = new CommandPS5Controller(Constants.DRIVER_CONTROLLER_PORT);

    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));

    // Choreo
    private final AutoFactory autoFactory = new AutoFactory(
            m_swerveSubsystem::getPose, // A function that returns the current robot pose
            m_swerveSubsystem::resetOdometry, // A function that resets the current robot pose to
                                              // the provided
                                              // Pose2d
            m_swerveSubsystem::followTrajectory, // The drive subsystem trajectory follower

            true, // If alliance flipping should be enabled

            m_swerveSubsystem // The drive subsystem
    );
    private final AutoChooser autoChooser;

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
            () -> m_driverController.getLeftY() * -1,
            () -> m_driverController.getLeftX() * -1)
            .withControllerRotationAxis(() -> m_driverController.getRightX() * -1) // TODO: Check if * -1 is
                                                                                   // needed IRL
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

    private DoubleSupplier autoAimHeadingX() {
        return () -> m_swerveSubsystem.getAutoAimHeading().getCos();
    }

    private DoubleSupplier autoAimHeadingY() {
        return () -> -m_swerveSubsystem.getAutoAimHeading().getSin();
    }

    SwerveInputStream driveAutoAim = driveAngularVelocity.copy()
            .withControllerHeadingAxis(autoAimHeadingX(), autoAimHeadingY())
            .headingWhile(true)
            .scaleTranslation(SwerveConstants.AUTO_AIM_SCALE_TRANSLATION);

    public RobotContainer() {

        autoChooser = new AutoChooser();
        // autoChooser.addRoutine("routine1", this::routine1);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

        configureBindings();
    }

    private final Command selectRedLeftTrenchTraversal = new SelectCommand<>(
            Map.ofEntries(
                    Map.entry(Zone.RED_ALLIANCE_LEFT,
                            autoFactory.trajectoryCmd("TrenchLeftFromAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_RED_LEFT,
                            autoFactory.trajectoryCmd("TrenchLeftToAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_BLUE_RIGHT,
                            autoFactory.trajectoryCmd("TrenchLeftToOpponent")),
                    Map.entry(Zone.BLUE_ALLIANCE_RIGHT,
                            autoFactory.trajectoryCmd("TrenchLeftFromOpponent"))),
            m_swerveSubsystem::getCurrentZone);

    private final Command selectRedRightTrenchTraversal = new SelectCommand<>(
            Map.ofEntries(
                    Map.entry(Zone.RED_ALLIANCE_RIGHT,
                            autoFactory.trajectoryCmd("TrenchRightFromAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_RED_RIGHT,
                            autoFactory.trajectoryCmd("TrenchRightToAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_BLUE_LEFT,
                            autoFactory.trajectoryCmd("TrenchRightToOpponent")),
                    Map.entry(Zone.BLUE_ALLIANCE_LEFT,
                            autoFactory.trajectoryCmd("TrenchRightFromOpponent"))),
            m_swerveSubsystem::getCurrentZone);

    private final Command selectBlueLeftTrenchTraversal = new SelectCommand<>(
            Map.ofEntries(
                    Map.entry(Zone.BLUE_ALLIANCE_LEFT,
                            autoFactory.trajectoryCmd("TrenchLeftFromAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_BLUE_LEFT,
                            autoFactory.trajectoryCmd("TrenchLeftToAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_RED_RIGHT,
                            autoFactory.trajectoryCmd("TrenchLeftToOpponent")),
                    Map.entry(Zone.RED_ALLIANCE_RIGHT,
                            autoFactory.trajectoryCmd("TrenchLeftFromOpponent"))),
            m_swerveSubsystem::getCurrentZone);

    private final Command selectBlueRightTrenchTraversal = new SelectCommand<>(
            Map.ofEntries(
                    Map.entry(Zone.BLUE_ALLIANCE_RIGHT,
                            autoFactory.trajectoryCmd("TrenchRightFromAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_BLUE_RIGHT,
                            autoFactory.trajectoryCmd("TrenchRightToAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_RED_LEFT,
                            autoFactory.trajectoryCmd("TrenchRightToOpponent")),
                    Map.entry(Zone.RED_ALLIANCE_LEFT,
                            autoFactory.trajectoryCmd("TrenchRightFromOpponent"))),
            m_swerveSubsystem::getCurrentZone);

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
        Command driveFieldOrientedAutoAim = m_swerveSubsystem.driveFieldOriented(driveAutoAim);

        m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        m_driverController.options().onTrue((Commands.runOnce(m_swerveSubsystem::zeroGyroWithAlliance)));
        m_driverController.create().whileTrue(m_swerveSubsystem.centerModulesCommand());

        // Auto-aim (swerve heading with calculated hood angle) and shoot
        m_driverController.R2().whileTrue(driveFieldOrientedAutoAim);

        // PID-tuned auto-align for climbing start position
        driveAngularVelocity.driveToPose(m_swerveSubsystem::getSelectedClimbPose,
                new ProfiledPIDController(
                        SwerveConstants.DRIVE_TO_POSE_TRANSLATION_kP,
                        SwerveConstants.DRIVE_TO_POSE_TRANSLATION_kI,
                        SwerveConstants.DRIVE_TO_POSE_TRANSLATION_kD,
                        new TrapezoidProfile.Constraints(
                                SwerveConstants.DRIVE_TO_POSE_TRANSLATION_MAX_VELOCITY,
                                SwerveConstants.DRIVE_TO_POSE_TRANSLATION_MAX_ACCELERATION)),
                new ProfiledPIDController(
                        SwerveConstants.DRIVE_TO_POSE_ROTATION_kP,
                        SwerveConstants.DRIVE_TO_POSE_ROTATION_kI,
                        SwerveConstants.DRIVE_TO_POSE_ROTATION_kD,
                        new TrapezoidProfile.Constraints(
                                SwerveConstants.DRIVE_TO_POSE_ROTATION_MAX_VELOCITY_RAD,
                                SwerveConstants.DRIVE_TO_POSE_ROTATION_MAX_ACCELERATION_RAD)));

        // Auto-align to left side tower for climbing
        m_driverController.povLeft().whileTrue(
                Commands.sequence(
                        new InstantCommand(
                                () -> m_swerveSubsystem.setSelectedClimbPose(true)),
                        Commands.runEnd(
                                () -> driveAngularVelocity.driveToPoseEnabled(true),
                                () -> driveAngularVelocity.driveToPoseEnabled(false))));

        // Auto-align to right side tower for climbing
        m_driverController.povRight().whileTrue(
                Commands.sequence(
                        new InstantCommand(
                                () -> m_swerveSubsystem.setSelectedClimbPose(false)),
                        Commands.runEnd(
                                () -> driveAngularVelocity.driveToPoseEnabled(true),
                                () -> driveAngularVelocity.driveToPoseEnabled(false))));

        // Auto-traverse the trench through left side
        m_driverController.L3().whileTrue(
                new ConditionalCommand(
                        selectRedLeftTrenchTraversal,
                        selectBlueLeftTrenchTraversal,
                        m_swerveSubsystem::isRedAlliance));

        // Auto-traverse the trench through right side
        m_driverController.R3().whileTrue(
                new ConditionalCommand(
                        selectRedRightTrenchTraversal,
                        selectBlueRightTrenchTraversal,
                        m_swerveSubsystem::isRedAlliance));
    }
}
