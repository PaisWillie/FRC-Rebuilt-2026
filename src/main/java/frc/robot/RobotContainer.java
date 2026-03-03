// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.intake.LinearIntakeSubsystem;

public class RobotContainer {
    final CommandPS5Controller m_driverController = new CommandPS5Controller(Constants.DRIVER_CONTROLLER_PORT);

    private final LinearIntakeSubsystem m_linearIntakeSubsystem = new LinearIntakeSubsystem();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        m_driverController.circle().onTrue(m_linearIntakeSubsystem.extend());
        m_driverController.triangle().onTrue(m_linearIntakeSubsystem.retract());
        m_driverController.cross().onTrue(m_linearIntakeSubsystem.fullyRetract());

        m_driverController.options().whileTrue(m_linearIntakeSubsystem.sysId());
    }
}
