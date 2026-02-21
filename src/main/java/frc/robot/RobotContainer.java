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
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FuelSimSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.Zone;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.LinearIntakeSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
    final CommandPS5Controller m_driverController = new CommandPS5Controller(Constants.DRIVER_CONTROLLER_PORT);

    private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
    private final HopperSubsystem m_hopperSubsystem = new HopperSubsystem();
    private final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
    private final IntakeRollerSubsystem m_intakeRollerSubsystem = new IntakeRollerSubsystem();
    private final LinearIntakeSubsystem m_linearIntakeSubsystem = new LinearIntakeSubsystem();
    private final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));

    private final FuelSimSubsystem m_fuelSimSubsystem = new FuelSimSubsystem();

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
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        autoChooser = new AutoChooser();
        // autoChooser.addRoutine("routine1", this::routine1);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

        configureBindings();
    }

    private final Command selectRedLeftTrenchTraversal = new SelectCommand<>(
            Map.ofEntries(
                    Map.entry(Zone.RED_ALLIANCE_LEFT, autoFactory.trajectoryCmd("TrenchLeftFromAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_RED_LEFT, autoFactory.trajectoryCmd("TrenchLeftToAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_BLUE_RIGHT, autoFactory.trajectoryCmd("TrenchLeftToOpponent")),
                    Map.entry(Zone.BLUE_ALLIANCE_RIGHT, autoFactory.trajectoryCmd("TrenchLeftFromOpponent"))),
            m_swerveSubsystem::getCurrentZone);

    private final Command selectRedRightTrenchTraversal = new SelectCommand<>(
            Map.ofEntries(
                    Map.entry(Zone.RED_ALLIANCE_RIGHT, autoFactory.trajectoryCmd("TrenchRightFromAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_RED_RIGHT, autoFactory.trajectoryCmd("TrenchRightToAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_BLUE_LEFT, autoFactory.trajectoryCmd("TrenchRightToOpponent")),
                    Map.entry(Zone.BLUE_ALLIANCE_LEFT, autoFactory.trajectoryCmd("TrenchRightFromOpponent"))),
            m_swerveSubsystem::getCurrentZone);

    private final Command selectBlueLeftTrenchTraversal = new SelectCommand<>(
            Map.ofEntries(
                    Map.entry(Zone.BLUE_ALLIANCE_LEFT, autoFactory.trajectoryCmd("TrenchLeftFromAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_BLUE_LEFT, autoFactory.trajectoryCmd("TrenchLeftToAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_RED_RIGHT, autoFactory.trajectoryCmd("TrenchLeftToOpponent")),
                    Map.entry(Zone.RED_ALLIANCE_RIGHT, autoFactory.trajectoryCmd("TrenchLeftFromOpponent"))),
            m_swerveSubsystem::getCurrentZone);

    private final Command selectBlueRightTrenchTraversal = new SelectCommand<>(
            Map.ofEntries(
                    Map.entry(Zone.BLUE_ALLIANCE_RIGHT, autoFactory.trajectoryCmd("TrenchRightFromAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_BLUE_RIGHT, autoFactory.trajectoryCmd("TrenchRightToAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_RED_LEFT, autoFactory.trajectoryCmd("TrenchRightToOpponent")),
                    Map.entry(Zone.RED_ALLIANCE_LEFT, autoFactory.trajectoryCmd("TrenchRightFromOpponent"))),
            m_swerveSubsystem::getCurrentZone);

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
        Command driveFieldOrientedAutoAim = m_swerveSubsystem.driveFieldOriented(driveAutoAim);

        m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAnglularVelocity);

        m_driverController.options().onTrue((Commands.runOnce(m_swerveSubsystem::zeroGyro)));
        m_driverController.create().whileTrue(m_swerveSubsystem.centerModulesCommand());

        // Auto-aim (swerve heading with calculated hood angle) and shoot
        m_driverController.R2().whileTrue(driveFieldOrientedAutoAim);
        m_driverController.R2()
                .onTrue(m_shooterSubsystem.aimAndShoot(m_swerveSubsystem::getDistanceToTarget,
                        m_swerveSubsystem::isAutoAimOnTarget))
                .onFalse(new ConditionalCommand(
                        Commands.sequence(
                                m_shooterSubsystem.stopShooting(),
                                m_shooterSubsystem.storeFuel()),
                        m_shooterSubsystem.stopShooting(),
                        m_driverController.L2()::getAsBoolean));
        m_driverController.R2()
                .onTrue(m_indexerSubsystem.run())
                .onFalse(m_indexerSubsystem.stop()
                        .unless(m_driverController.L2()::getAsBoolean));

        if (Robot.isSimulation()) {
            m_driverController.R2()
                    .whileTrue(Commands.sequence(
                            FuelSimSubsystem.shootFuel(
                                    m_shooterSubsystem::getFlywheelLinearVelocity,
                                    m_swerveSubsystem::getPose,
                                    m_swerveSubsystem::getFieldVelocity,
                                    m_swerveSubsystem::getHeading,
                                    m_shooterSubsystem::getHoodAngle),
                            Commands.waitTime(Seconds.of(0.25)))
                            .onlyIf(() -> m_shooterSubsystem.isShooterReady() && m_swerveSubsystem.isAutoAimOnTarget())
                            .repeatedly());
        }

        // Shoot without auto-aiming, defaulting to a preset hood angle for shooting
        // from directly in front of the hub
        m_driverController.R1()
                .and(m_driverController.R2().negate())
                .onTrue(m_shooterSubsystem.shootNoAutoAim())
                .onFalse(new ConditionalCommand(
                        Commands.sequence(
                                m_shooterSubsystem.stopShooting(),
                                m_shooterSubsystem.storeFuel()),
                        m_shooterSubsystem.stopShooting(),
                        m_driverController.L2()::getAsBoolean));
        m_driverController.R1()
                .and(m_driverController.R2().negate())
                .onTrue(m_indexerSubsystem.run())
                .onFalse(m_indexerSubsystem.stop()
                        .unless(m_driverController.L2()::getAsBoolean));

        // Extend intake, expand hopper, and run intake rollers
        m_driverController.L2()
                .onTrue(m_linearIntakeSubsystem.extend())
                .onFalse(m_linearIntakeSubsystem.retract());
        m_driverController.L2()
                .onTrue(m_hopperSubsystem.expand());
        m_driverController.L2()
                .onTrue(m_intakeRollerSubsystem.intake())
                .onFalse(m_intakeRollerSubsystem.stop());
        m_driverController.L2()
                .whileTrue(
                        Commands.parallel(
                                m_indexerSubsystem.run(),
                                m_shooterSubsystem.storeFuel()))
                .onFalse(
                        m_indexerSubsystem.stop()
                                .unless(m_driverController.R2()::getAsBoolean));

        m_driverController.L1()
                .and(m_driverController.R2().negate()) // Not shooting
                .and(m_driverController.L2().negate()) // Not intaking

                // Extend intake, reverse indexer and intake rollers at the same time
                .onTrue(Commands.sequence( // TODO: Check if sequence is needed, or if parallel alone is
                                           // fine
                        m_linearIntakeSubsystem.extend(),
                        Commands.parallel(
                                m_indexerSubsystem.reverse(),
                                m_intakeRollerSubsystem.outtake())))

                // Retract intake, then stop indexer and intake rollers
                .onFalse(
                        Commands.sequence(
                                m_linearIntakeSubsystem.retract(),
                                Commands.parallel(
                                        m_indexerSubsystem.stop(),
                                        m_intakeRollerSubsystem.stop())));

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
        m_driverController.triangle().whileTrue(
                new ConditionalCommand(
                        selectRedLeftTrenchTraversal,
                        selectBlueLeftTrenchTraversal,
                        m_swerveSubsystem::isRedAlliance));
        // Stop shooting to prevent hood from hitting trench
        m_driverController.triangle().onTrue(
                new ConditionalCommand(
                        Commands.sequence( // If intake is active, continue storing fuel
                                m_shooterSubsystem.stopShooting(),
                                m_shooterSubsystem.storeFuel()),
                        m_shooterSubsystem.stopShooting(),
                        m_driverController.L2()::getAsBoolean));

        // Auto-traverse the trench through right side
        m_driverController.square().whileTrue(
                new ConditionalCommand(
                        selectRedRightTrenchTraversal,
                        selectBlueRightTrenchTraversal,
                        m_swerveSubsystem::isRedAlliance));
        // Stop shooting to prevent hood from hitting trench
        m_driverController.square().onTrue(
                new ConditionalCommand(
                        Commands.sequence( // If intake is active, continue storing fuel
                                m_shooterSubsystem.stopShooting(),
                                m_shooterSubsystem.storeFuel()),
                        m_shooterSubsystem.stopShooting(),
                        m_driverController.L2()::getAsBoolean));
    }

    /**
     * Starts the flywheel spinning at the default RPM, the speed at which it should
     * spin when the shooter is not actively shooting.
     * 
     * This command is intended to be scheduled when teleop starts (in
     * Robot.teleopInit()).
     * 
     * @return a Command that starts the flywheel at the default RPM when executed
     */
    public Command startFlywheelDefaultRPM() {
        return m_shooterSubsystem.startFlywheelDefaultRPM();
    }
}
