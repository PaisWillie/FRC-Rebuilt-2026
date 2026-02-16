// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ElevatorSubsystem;
import frc.robot.subsystems.climb.TongueSubsystem;

public class ClimbSubsystem extends SubsystemBase {

    private final ElevatorSubsystem m_elevatorSubsystem;
    private final TongueSubsystem m_tongueSubsystem;

    public ClimbSubsystem() {
        m_elevatorSubsystem = new ElevatorSubsystem();
        m_tongueSubsystem = new TongueSubsystem();
    }

    @Override
    public void periodic() {

    }
}
