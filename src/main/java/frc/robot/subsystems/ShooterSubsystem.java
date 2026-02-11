// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class ShooterSubsystem extends SubsystemBase {

    FeederSubsystem m_feederSubsystem;
    FlywheelSubsystem m_flywheelSubsystem;
    HoodSubsystem m_hoodSubsystem;

    public ShooterSubsystem() {
        m_feederSubsystem = new FeederSubsystem();
        m_flywheelSubsystem = new FlywheelSubsystem();
        m_hoodSubsystem = new HoodSubsystem();

        // TODO: Hood to constantly correct itself to the target angle using vision
        // feedback

        // TODO: Set flywheel velocity to default speed when not shooting, and only
        // increase to target speed when shooting command is active
    }

    // public Command shoot() {
    // return new SequentialCommandGroup();
    // }

    @Override
    public void periodic() {
    }
}
