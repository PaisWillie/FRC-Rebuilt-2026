// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_robotContainer.updateLocalization();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        // Zero gyro (shooter must face away from driver, towards opponent wall)
        m_robotContainer.zeroGyroWithAlliance();

        CommandScheduler.getInstance().schedule(m_robotContainer.stopAllSubsystems());

        // Check the linear intake position and set the encoder position accordingly
        m_robotContainer.calibrateLinearIntakePosition();

        // Start the flywheel at the default RPM when teleop starts
        CommandScheduler.getInstance().schedule(m_robotContainer.m_shooterSubsystem.startFlywheelDefaultRPM());

        // Extend the intake to lower the hopper enough to go underneath the trench
        // NOTE: Extend intake fully in auto, due to bug not allowing you to move it
        // during auto without interrupting the auto
        CommandScheduler.getInstance().schedule(m_robotContainer.m_linearIntakeSubsystem.extend());
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
        m_robotContainer.driveAngularVelocity.driveToPoseEnabled(false);
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        // Check the linear intake position and set the encoder position accordingly
        m_robotContainer.calibrateLinearIntakePosition();

        CommandScheduler.getInstance().schedule(m_robotContainer.stopAllSubsystems());
        m_robotContainer.driveAngularVelocity.driveToPoseEnabled(false);

        // Start the flywheel at the default RPM when teleop starts
        CommandScheduler.getInstance().schedule(m_robotContainer.m_shooterSubsystem.startFlywheelDefaultRPM());

        // Extend the intake to lower the hopper enough to go underneath the trench
        CommandScheduler.getInstance().schedule(m_robotContainer.m_linearIntakeSubsystem.midpoint());
        try {
            m_robotContainer.getDashboardSubsystem().setInactiveFirst(DriverStation.getGameSpecificMessage().charAt(0));
        } catch (IndexOutOfBoundsException e) {
            SmartDashboard.putString("Inactive first", "Blue");
            String inactiveFirst = SmartDashboard.getString("Inactive first", "None");
            if (inactiveFirst.equalsIgnoreCase("Red")) {
                m_robotContainer.getDashboardSubsystem().setInactiveFirst('R');
            } else if (inactiveFirst.equalsIgnoreCase("Blue")) {
                m_robotContainer.getDashboardSubsystem().setInactiveFirst('B');
            }
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }
}
