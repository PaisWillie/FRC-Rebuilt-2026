// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;
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
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.HopperSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
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

    // Choreo
    private final AutoFactory autoFactory;
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

    private DoubleSupplier headingXSupplier() {
        return () -> m_swerveSubsystem.getAutoAimHeading().getCos();
    }

    private DoubleSupplier headingYSupplier() {
        return () -> -m_swerveSubsystem.getAutoAimHeading().getSin();
    }

    SwerveInputStream driveAutoAim = driveAngularVelocity.copy()
            .withControllerHeadingAxis(headingXSupplier(), headingYSupplier())
            .headingWhile(true)
            .scaleTranslation(SwerveConstants.AUTO_AIM_SCALE_TRANSLATION);

    public RobotContainer() {
        if (Robot.isSimulation()) {
            DriverStation.silenceJoystickConnectionWarning(true);
        }

        autoFactory = new AutoFactory(
                m_swerveSubsystem::getPose, // A function that returns the current robot pose
                m_swerveSubsystem::resetOdometry, // A function that resets the current robot pose to
                                                  // the provided
                                                  // Pose2d
                m_swerveSubsystem::followTrajectory, // The drive subsystem trajectory follower

                true, // If alliance flipping should be enabled

                m_swerveSubsystem // The drive subsystem
        );
        autoChooser = new AutoChooser();
        // autoChooser.addRoutine("routine1", this::routine1);
        SmartDashboard.putData("Auto Chooser", autoChooser);
        RobotModeTriggers.autonomous().whileTrue(autoChooser.selectedCommandScheduler());

        configureBindings();
    }

    private void configureBindings() {
        Command driveFieldOrientedAnglularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);
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

        // Cardinal directions for PID tuning
        // m_driverController.povUp().whileTrue(m_swerveSubsystem.driveForward());
        // m_driverController.povDown().whileTrue(m_swerveSubsystem.driveBackward());
        // m_driverController.povLeft().whileTrue(m_swerveSubsystem.driveLeft());
        // m_driverController.povRight().whileTrue(m_swerveSubsystem.driveRight());

        m_driverController.options().onTrue((Commands.runOnce(m_swerveSubsystem::zeroGyro)));

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

        m_driverController.R1()
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

        m_driverController.povLeft().whileTrue(
                Commands.sequence(
                        new InstantCommand(
                                () -> m_swerveSubsystem.setSelectedClimbPose(true)),
                        Commands.runEnd(
                                () -> driveAngularVelocity.driveToPoseEnabled(true),
                                () -> driveAngularVelocity.driveToPoseEnabled(false))));

        m_driverController.povRight().whileTrue(
                Commands.sequence(
                        new InstantCommand(
                                () -> m_swerveSubsystem.setSelectedClimbPose(false)),
                        Commands.runEnd(
                                () -> driveAngularVelocity.driveToPoseEnabled(true),
                                () -> driveAngularVelocity.driveToPoseEnabled(false))));

        // m_linearIntakeSubsystem.setDefaultCommand(m_linearIntakeSubsystem.set(0));

        // m_driverController.cross().whileTrue(m_linearIntakeSubsystem.sysId());
        // m_driverController.cross().whileTrue(m_swerveSubsystem.sysIdDriveMotorCommand());
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
