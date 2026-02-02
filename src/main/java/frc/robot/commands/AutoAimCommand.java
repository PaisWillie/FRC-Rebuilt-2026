// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class AutoAimCommand extends Command {

    SwerveSubsystem m_swerveSubsystem;

    PIDController pidController = new PIDController(0.1, 0, 0);
    Pose2d pose;

    Pose2d blueHub = new Pose2d(4.0218614 + Units.inchesToMeters(47.0) / 2.0, 0.00, null);

    public AutoAimCommand(SwerveSubsystem swerveSubsystem) {
        m_swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        pose = m_swerveSubsystem.getSwerveDrive().swerveDrivePoseEstimator.getEstimatedPosition();
    }

    @Override
    public void execute() {
        pose.getTranslation().minus(blueHub.getTranslation());
        double angleToHub = Math.atan2(blueHub.getY() - pose.getY(), blueHub.getX() - pose.getX());
        double currentAngle = pose.getRotation().getRadians();
        double angleError = angleToHub - currentAngle;
        
        // Use SwerveInputStream with YAGSL to direct angle to angleToHub
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
