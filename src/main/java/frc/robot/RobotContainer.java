// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Seconds;

import java.io.File;
import java.util.Map;
import java.util.function.BooleanSupplier;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimbConstants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.DashboardSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SimSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.SwerveSubsystem.Zone;
import frc.robot.subsystems.climb.ElevatorSubsystem;
import frc.robot.subsystems.intake.IntakeRollerSubsystem;
import frc.robot.subsystems.intake.LinearIntakeSubsystem;
import frc.robot.subsystems.intake.LinearIntakeSubsystem.LinearIntakePosition;
import frc.robot.utils.LimelightWrapper;
import limelight.networktables.LimelightSettings.ImuMode;
import swervelib.SwerveInputStream;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

public class RobotContainer {
    final CommandPS5Controller m_driverController = new CommandPS5Controller(Constants.DRIVER_CONTROLLER_PORT);

    // private final ClimbSubsystem m_climbSubsystem = new ClimbSubsystem();
    public final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

    private final DashboardSubsystem m_dashboardSubsystem = new DashboardSubsystem();
    public final HopperSubsystem m_hopperSubsystem = new HopperSubsystem();
    public final IndexerSubsystem m_indexerSubsystem = new IndexerSubsystem();
    public final IntakeRollerSubsystem m_intakeRollerSubsystem = new IntakeRollerSubsystem();
    public final LinearIntakeSubsystem m_linearIntakeSubsystem = new LinearIntakeSubsystem();
    public final ShooterSubsystem m_shooterSubsystem = new ShooterSubsystem();
    public final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));

    private final SimSubsystem m_simSubsystem;

    private final LimelightWrapper m_limelightA;
    private final LimelightWrapper m_limelightB;

    // Choreo
    public final AutoFactory m_autoFactory = new AutoFactory(
            m_swerveSubsystem::getPose, // A function that returns the current robot pose
            m_swerveSubsystem::resetOdometry, // A function that resets the current robot pose to
                                              // the provided
                                              // Pose2d
            m_swerveSubsystem::followTrajectory, // The drive subsystem trajectory follower

            true, // If alliance flipping should be enabled

            m_swerveSubsystem // The drive subsystem
    );

    private final AutoChooser autoChooser;
    private final Choreo m_choreo = new Choreo(this);

    /**
     * Converts driver input into a field-relative ChassisSpeeds that is controlled
     * by angular velocity.
     */
    public SwerveInputStream driveAngularVelocity = SwerveInputStream.of(m_swerveSubsystem.getSwerveDrive(),
            () -> m_driverController.getLeftY() * -1,
            () -> m_driverController.getLeftX() * -1)
            .withControllerRotationAxis(() -> m_driverController.getRightX() * -1) // TODO: Check if * -1 is
                                                                                   // needed IRL
            .deadband(OperatorConstants.DEADBAND)
            .scaleTranslation(1.0)
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

    public DoubleSupplier autoAimHeadingX() {
        return () -> m_swerveSubsystem.getAutoAimHeading().getCos();
    }

    public DoubleSupplier autoAimHeadingY() {
        return () -> -m_swerveSubsystem.getAutoAimHeading().getSin();
    }

    public SwerveInputStream driveAutoAim = driveAngularVelocity.copy()
            .withControllerHeadingAxis(autoAimHeadingX(), autoAimHeadingY())
            .headingWhile(true)
            .scaleTranslation(SwerveConstants.AUTO_AIM_SCALE_TRANSLATION);

    Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngle = m_swerveSubsystem.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAutoAim = m_swerveSubsystem.driveFieldOriented(driveAutoAim);

    public RobotContainer() {
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
            m_simSubsystem = new SimSubsystem(
                    m_swerveSubsystem.getSwerveDrive().getMapleSimDrive().get());
        } else {
            m_simSubsystem = null;
        }

        autoChooser = new AutoChooser();

        autoChooser.addCmd("Anywhere - Shoot pre-load", m_choreo::shootPreloadAuto);
        autoChooser.addCmd("Right - Sweep once", m_choreo::rightNeutralAutoSweepOnce);
        autoChooser.addCmd("Right - Sweep twice", m_choreo::rightNeutralAutoSweepTwice);
        autoChooser.addCmd("Right - Sweep then climb", m_choreo::rightNeutralAutoThenClimb);
        autoChooser.addCmd("Left - Sweep once", m_choreo::leftNeutralAutoSweepOnce);
        autoChooser.addCmd("Left - Sweep twice", m_choreo::leftNeutralAutoSweepTwice);
        autoChooser.addCmd("Left - Sweep then climb", m_choreo::leftNeutralAutoThenClimb);
        autoChooser.addCmd("Left - Sweep then depot", m_choreo::leftNeutralAutoThenDepot);
        autoChooser.addCmd("Center - Shoot and climb left", m_choreo::shootPreloadAndClimbLeft);
        autoChooser.addCmd("Center - Shoot and climb right", m_choreo::shootPreloadAndClimbRight);

        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

        m_limelightA = new LimelightWrapper("limelight-a", true);
        m_limelightB = new LimelightWrapper("limelight-b", true);

        // Only do this for LL4, so we use heading readings from MT1 from 3G?
        m_limelightA.getSettings().withImuMode(ImuMode.ExternalImu).save();
        m_limelightB.getSettings().withImuMode(ImuMode.ExternalImu).save();

        // PID-tuned auto-align for climbing start position
        driveAngularVelocity.driveToPose(m_swerveSubsystem::getDriveToWaypoint,
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

        configureBindings();
    }

    private final Command selectRedLeftTrenchTraversal = new SelectCommand<>(
            Map.ofEntries(
                    Map.entry(Zone.RED_ALLIANCE_LEFT,
                            m_autoFactory.trajectoryCmd("TrenchLeftFromAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_RED_LEFT,
                            m_autoFactory.trajectoryCmd("TrenchLeftToAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_BLUE_RIGHT,
                            m_autoFactory.trajectoryCmd("TrenchLeftToOpponent")),
                    Map.entry(Zone.BLUE_ALLIANCE_RIGHT,
                            m_autoFactory.trajectoryCmd("TrenchLeftFromOpponent"))),
            m_swerveSubsystem::getCurrentZone);

    private final Command selectRedRightTrenchTraversal = new SelectCommand<>(
            Map.ofEntries(
                    Map.entry(Zone.RED_ALLIANCE_RIGHT,
                            m_autoFactory.trajectoryCmd("TrenchRightFromAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_RED_RIGHT,
                            m_autoFactory.trajectoryCmd("TrenchRightToAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_BLUE_LEFT,
                            m_autoFactory.trajectoryCmd("TrenchRightToOpponent")),
                    Map.entry(Zone.BLUE_ALLIANCE_LEFT,
                            m_autoFactory.trajectoryCmd("TrenchRightFromOpponent"))),
            m_swerveSubsystem::getCurrentZone);

    private final Command selectBlueLeftTrenchTraversal = new SelectCommand<>(
            Map.ofEntries(
                    Map.entry(Zone.BLUE_ALLIANCE_LEFT,
                            m_autoFactory.trajectoryCmd("TrenchLeftFromAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_BLUE_LEFT,
                            m_autoFactory.trajectoryCmd("TrenchLeftToAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_RED_RIGHT,
                            m_autoFactory.trajectoryCmd("TrenchLeftToOpponent")),
                    Map.entry(Zone.RED_ALLIANCE_RIGHT,
                            m_autoFactory.trajectoryCmd("TrenchLeftFromOpponent"))),
            m_swerveSubsystem::getCurrentZone);

    private final Command selectBlueRightTrenchTraversal = new SelectCommand<>(
            Map.ofEntries(
                    Map.entry(Zone.BLUE_ALLIANCE_RIGHT,
                            m_autoFactory.trajectoryCmd("TrenchRightFromAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_BLUE_RIGHT,
                            m_autoFactory.trajectoryCmd("TrenchRightToAlliance")),
                    Map.entry(Zone.NEUTRAL_ZONE_RED_LEFT,
                            m_autoFactory.trajectoryCmd("TrenchRightToOpponent")),
                    Map.entry(Zone.RED_ALLIANCE_LEFT,
                            m_autoFactory.trajectoryCmd("TrenchRightFromOpponent"))),
            m_swerveSubsystem::getCurrentZone);

    private void configureBindings() {
        m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);

        BooleanSupplier isIdle = () -> Math.abs(m_driverController.getLeftX()) < OperatorConstants.DEADBAND &&
                Math.abs(m_driverController.getLeftY()) < OperatorConstants.DEADBAND &&
                Math.abs(m_driverController.getRightX()) < OperatorConstants.DEADBAND &&
                !DriverStation.isAutonomous();

        Trigger isIdleTrigger = new Trigger(isIdle);

        m_driverController.options().onTrue((Commands.runOnce(m_swerveSubsystem::zeroGyroWithAlliance)));
        m_driverController.create().whileTrue(m_swerveSubsystem.centerModulesCommand());

        // Trigger for if driver is controlling the robot
        Trigger isControllingDriveTrigger = new Trigger(() -> Math
                .abs(m_driverController.getLeftX()) > OperatorConstants.DEADBAND
                || Math.abs(m_driverController.getLeftY()) > OperatorConstants.DEADBAND);

        Trigger isClimberUp = new Trigger(m_elevatorSubsystem::isClimberUp);

        // TODO: Check if aiming
        isClimberUp.and(
                m_driverController.R2().negate()).onTrue(
                        new InstantCommand(() -> {
                            driveAngularVelocity.scaleTranslation(
                                    SwerveConstants.AUTO_AIM_SCALE_TRANSLATION)
                                    .scaleRotation(SwerveConstants.AUTO_AIM_SCALE_TRANSLATION);
                        }))
                .onFalse(
                        new InstantCommand(() -> driveAngularVelocity.scaleTranslation(
                                1.0).scaleRotation(1.0)));

        Trigger autoAimOnTarget = new Trigger(m_swerveSubsystem::isAutoAimOnTarget);

        // Auto-aim (swerve heading with calculated hood angle) and shoot
        m_driverController.R2()
                .and(isIdleTrigger.negate())
                .and(autoAimOnTarget)
                .whileTrue(driveFieldOrientedAutoAim);
        m_driverController.R2()
                .and(isIdleTrigger)
                .and(autoAimOnTarget.negate())
                .whileTrue(driveFieldOrientedAutoAim);
        m_driverController.R2()
                .and(isIdleTrigger.negate())
                .and(autoAimOnTarget.negate())
                .whileTrue(driveFieldOrientedAutoAim);
        m_driverController.R2()
                .and(isIdleTrigger)
                .and(autoAimOnTarget)
                .onTrue(
                        Commands.sequence(
                                Commands.waitSeconds(0.01),
                                new InstantCommand(m_swerveSubsystem::lock,
                                        m_swerveSubsystem)));
        m_driverController.R2()
                .and(isControllingDriveTrigger)
                .onTrue(m_shooterSubsystem.aimAndShoot(
                        () -> m_swerveSubsystem.getDistanceToTarget(true),
                        m_swerveSubsystem::isAutoAimOnTarget, false, m_swerveSubsystem::isInAllianceZone)
                        .beforeStarting(m_shooterSubsystem.stopFeeder()));
        m_driverController.R2()
                .and(isControllingDriveTrigger.negate())
                .onTrue(m_shooterSubsystem.aimAndShoot(
                        () -> m_swerveSubsystem.getDistanceToTarget(true),
                        m_swerveSubsystem::isAutoAimOnTarget, true, m_swerveSubsystem::isInAllianceZone)
                        .beforeStarting(m_shooterSubsystem.stopFeeder()));
        // Stop shooter subsystem
        m_driverController.R2()
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
        m_driverController.R2()
                .onTrue(m_intakeRollerSubsystem.intake())
                .onFalse(m_intakeRollerSubsystem.stop()
                        .unless(m_driverController.L2()::getAsBoolean));
        m_driverController.R2()
                .onTrue(m_linearIntakeSubsystem.shuffle()
                        .unless(m_driverController.L2()::getAsBoolean))
                .onFalse(m_linearIntakeSubsystem.midpoint()
                        .unless(m_driverController.L2()::getAsBoolean));

        if (Robot.isSimulation()) {
            SimulatedArena.getInstance().addGamePiece(new RebuiltFuelOnField(new Translation2d(3, 3)));
            m_driverController.R2()
                    .whileTrue(
                            Commands.sequence(
                                    m_simSubsystem.shootFuel(
                                            m_shooterSubsystem::getFlywheelLinearVelocity,
                                            m_swerveSubsystem::getPose,
                                            m_swerveSubsystem::getFieldVelocity,
                                            m_swerveSubsystem::getHeading,
                                            m_shooterSubsystem::getHoodSetpointAngle),
                                    Commands.waitTime(Seconds.of(0.1)))
                                    .onlyIf(() -> m_shooterSubsystem
                                            .isShooterReady()
                                            && m_swerveSubsystem
                                                    .isAutoAimOnTarget())
                                    .repeatedly());

            m_driverController.L2()
                    .onTrue(
                            new ConditionalCommand(m_simSubsystem.startIntake(),
                                    m_simSubsystem.stopIntake(),
                                    () -> m_linearIntakeSubsystem
                                            .getCurrentPositionEnum() == LinearIntakePosition.EXTENDED)
                                    .repeatedly())
                    .onFalse(m_simSubsystem.stopIntake());

            m_driverController.L3().onTrue(m_swerveSubsystem.simulationLocalize());
            m_driverController.R3().onTrue(m_swerveSubsystem.simulationLocalize());
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
                .whileTrue(
                        Commands.sequence(
                                Commands.waitSeconds(0.01),
                                new InstantCommand(m_swerveSubsystem::lock,
                                        m_swerveSubsystem)));
        m_driverController.R1()
                .and(m_driverController.R2().negate())
                .onTrue(m_indexerSubsystem.run())
                .onFalse(m_indexerSubsystem.stop()
                        .unless(m_driverController.L2()::getAsBoolean));
        m_driverController.R1()
                .onTrue(m_linearIntakeSubsystem.shuffle()
                        .unless(m_driverController.L2()::getAsBoolean))
                .onFalse(m_linearIntakeSubsystem.midpoint()
                        .unless(m_driverController.L2()::getAsBoolean));

        // Extend intake, expand hopper, and run intake rollers
        m_driverController.L2()
                .onTrue(Commands.parallel(
                        m_linearIntakeSubsystem.extend(),
                        m_intakeRollerSubsystem.intake()))
                .onFalse(m_linearIntakeSubsystem.midpoint().andThen(m_intakeRollerSubsystem.stop()));
        m_driverController.L2()
                .onTrue(m_hopperSubsystem.expand());
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
                                m_linearIntakeSubsystem.midpoint(),
                                Commands.parallel(
                                        m_indexerSubsystem.stop(),
                                        m_intakeRollerSubsystem.stop())));

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

        m_driverController.povUp().onTrue(m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_UPPER_LIMIT));
        m_driverController.povUp().onTrue(m_linearIntakeSubsystem.retract());
        m_driverController.povDown().onTrue(m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_LOWER_LIMIT));

        // Auto-traverse the trench through left side
        m_driverController.L3().whileTrue(
                new ConditionalCommand(
                        selectRedLeftTrenchTraversal,
                        selectBlueLeftTrenchTraversal,
                        m_swerveSubsystem::isRedAlliance));
        // Stop shooting to prevent hood from hitting trench
        m_driverController.L3().onTrue(
                new ConditionalCommand(
                        Commands.sequence( // If intake is active, continue storing fuel
                                m_shooterSubsystem.stopShooting(),
                                m_shooterSubsystem.storeFuel()),
                        m_shooterSubsystem.stopShooting(),
                        m_driverController.L2()::getAsBoolean));

        // Auto-traverse the trench through right side
        m_driverController.R3().whileTrue(
                new ConditionalCommand(
                        selectRedRightTrenchTraversal,
                        selectBlueRightTrenchTraversal,
                        m_swerveSubsystem::isRedAlliance));
        // Stop shooting to prevent hood from hitting trench
        m_driverController.R3().onTrue(
                new ConditionalCommand(
                        Commands.sequence( // If intake is active, continue storing fuel
                                m_shooterSubsystem.stopShooting(),
                                m_shooterSubsystem.storeFuel()),
                        m_shooterSubsystem.stopShooting(),
                        m_driverController.L2()::getAsBoolean));
    }

    public void calibrateLinearIntakePosition() {
        if (m_linearIntakeSubsystem.getExtendedLimitSwitch()) {
            CommandScheduler.getInstance().schedule(m_linearIntakeSubsystem.setEncoderPositionExtended());
        } else if (m_linearIntakeSubsystem.getRetractedLimitSwitch()) {
            CommandScheduler.getInstance().schedule(m_linearIntakeSubsystem.setEncoderPositionRetracted());
        }
    }

    public void updateLocalization() {
        // TODO: Prioritize LL4 over LL3G
        for (LimelightWrapper limelight : new LimelightWrapper[] { m_limelightB, m_limelightA }) {
            if (limelight.updateLocalization(m_swerveSubsystem.getSwerveDrive())) {
                break; // Stop once a limelight successfully localizes
            }
        }
    }

    public Command stopAllSubsystems() {
        return Commands.parallel(
                m_swerveSubsystem.stop(),
                m_shooterSubsystem.stopShooting(true, true),
                m_indexerSubsystem.stop(),
                m_hopperSubsystem.retract(),
                m_intakeRollerSubsystem.stop(),
                m_linearIntakeSubsystem.midpoint());
    }

    public void zeroGyroWithAlliance() {
        m_swerveSubsystem.zeroGyroWithAlliance();
    }

    public DashboardSubsystem getDashboardSubsystem() {
        return m_dashboardSubsystem;
    }
}
// Zach worked on this line therefore is now a part of prog subdivision