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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.ClimbConstants.ElevatorConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.climb.ElevatorSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
    final CommandPS5Controller m_driverController = new CommandPS5Controller(Constants.DRIVER_CONTROLLER_PORT);

    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
    private final SwerveSubsystem m_swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
            "swerve"));

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
            .scaleTranslation(1.0)
            .allianceRelativeControl(true);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        Command driveFieldOrientedAngularVelocity = m_swerveSubsystem.driveFieldOriented(driveAngularVelocity);

        m_swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
        m_driverController.options().onTrue((Commands.runOnce(m_swerveSubsystem::zeroGyro)));

        m_driverController.povDown().onTrue(m_elevatorSubsystem.setHeight(ElevatorConstants.STARTING_HEIGHT));
        m_driverController.povLeft().onTrue(m_elevatorSubsystem.setHeight(ElevatorConstants.FIRST_LEVEL_HEIGHT));
        m_driverController.povRight().onTrue(m_elevatorSubsystem.setHeight(ElevatorConstants.SECOND_LEVEL_HEIGHT));
        m_driverController.povUp().onTrue(m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_UPPER_LIMIT));

        m_driverController.cross().onTrue(m_elevatorSubsystem.elevCmd(-0.5)).onFalse(m_elevatorSubsystem.elevCmd(0.0));
        m_driverController.triangle().onTrue(m_elevatorSubsystem.elevCmd(0.5))
                .onFalse(m_elevatorSubsystem.elevCmd(0.0));
    }
}
