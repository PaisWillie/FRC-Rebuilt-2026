// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.shooter.FeederSubsystem;

public class RobotContainer {
    final CommandPS5Controller m_driverController = new CommandPS5Controller(Constants.DRIVER_CONTROLLER_PORT);

    private final FeederSubsystem m_feederSubsystem = new FeederSubsystem();

    public RobotContainer() {
        m_driverController.cross().whileTrue(m_feederSubsystem.sysId());
    }
}
