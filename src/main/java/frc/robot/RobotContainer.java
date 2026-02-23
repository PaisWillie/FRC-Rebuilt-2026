// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants.HoodConstants;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class RobotContainer {
    final CommandPS5Controller m_driverController = new CommandPS5Controller(Constants.DRIVER_CONTROLLER_PORT);

    private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();

    public RobotContainer() {
        m_driverController.cross().onTrue(m_hoodSubsystem.setAngle(HoodConstants.SOFT_LIMIT_MIN));
        m_driverController.triangle().onTrue(m_hoodSubsystem.setAngle(HoodConstants.SOFT_LIMIT_MAX));
        m_driverController.circle().onTrue(m_hoodSubsystem.setAngle(HoodConstants.DEFAULT_ANGLE));

        m_driverController.options().whileTrue(m_hoodSubsystem.sysId());
    }

}
