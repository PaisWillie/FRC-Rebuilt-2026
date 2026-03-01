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
import frc.robot.subsystems.climb.ElevatorSubsystem;

public class RobotContainer {
    final CommandPS5Controller m_driverController = new CommandPS5Controller(Constants.DRIVER_CONTROLLER_PORT);

    private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        m_driverController.cross().whileTrue(m_elevatorSubsystem.sysId());
        m_driverController.povDown().onTrue(m_elevatorSubsystem.setHeight(ElevatorConstants.STARTING_HEIGHT));
        m_driverController.povLeft().onTrue(m_elevatorSubsystem.setHeight(ElevatorConstants.FIRST_LEVEL_HEIGHT));
        m_driverController.povRight().onTrue(m_elevatorSubsystem.setHeight(ElevatorConstants.SECOND_LEVEL_HEIGHT));
        m_driverController.povUp().onTrue(m_elevatorSubsystem.setHeight(ElevatorConstants.SOFT_UPPER_LIMIT));
    }
}
