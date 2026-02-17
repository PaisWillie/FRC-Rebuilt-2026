// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import java.util.function.Supplier;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class ShooterSubsystem extends SubsystemBase {

    private FeederSubsystem m_feederSubsystem;
    private FlywheelSubsystem m_flywheelSubsystem;
    private HoodSubsystem m_hoodSubsystem;

    private InterpolatingDoubleTreeMap m_distanceToHoodAngleMap;

    public ShooterSubsystem() {
        m_feederSubsystem = new FeederSubsystem();
        m_flywheelSubsystem = new FlywheelSubsystem();
        m_hoodSubsystem = new HoodSubsystem();

        m_distanceToHoodAngleMap = new InterpolatingDoubleTreeMap();
        ShooterConstants.SHOOTER_DISTANCE_TO_HOOD_ANGLE.forEach(m_distanceToHoodAngleMap::put);

        // Set default command for flywheel to maintain a default RPM when not shooting
        m_flywheelSubsystem.setDefaultCommand(m_flywheelSubsystem.setDefaultRPM()); // TODO: Check if this is still
                                                                                    // active after running
                                                                                    // flywheel.stop()
    }

    /**
     * Checks if the shooter is ready to shoot by verifying that the hood is at the
     * correct angle and the flywheel is at the target RPM.
     * 
     * @return true if the shooter is ready to shoot, false otherwise
     */
    public boolean isShooterReady() {
        return m_hoodSubsystem.isAtTargetAngle() && m_flywheelSubsystem.isAtTargetRPM();
    }

    /**
     * Aims the shooter by adjusting the hood angle based on the distance to the
     * target,
     * and then shoots by spinning up the flywheel and feeding balls into it.
     *
     * @param distanceToTarget the distance from the robot to the target, used to
     *                         determine the appropriate hood angle
     * @param isAutoAimReady   a supplier that indicates whether the chassis'
     *                         auto-aiming
     *                         is ready
     * 
     * @return a Command that performs the aiming and shooting sequence when
     *         executed
     */
    public Command aimAndShoot(Supplier<Distance> getDistanceToTarget, Supplier<Boolean> isAutoAimReady) {
        return Commands.parallel(
                m_hoodSubsystem.setAngle(
                        () -> {
                            return Degrees.of(m_distanceToHoodAngleMap
                                    .get(getDistanceToTarget.get().in(Meters)));
                        }),
                m_flywheelSubsystem.shoot(),
                new ConditionalCommand(m_feederSubsystem.feed(), m_feederSubsystem.stop(),
                        () -> isAutoAimReady.get() && isShooterReady()).repeatedly()) // TODO: Find a way to remove the
                                                                                      // repeatedly(), and instead only
                                                                                      // switch between feed and stop
                                                                                      // once the shooter is ready/not
                                                                                      // ready, instead of constantly
                                                                                      // running feed/stop commands
                .withName("SHTR - Aim and Shoot");
    }

    public Command stopShooting() {
        return Commands.parallel(
                m_hoodSubsystem.lowerHood(),
                m_flywheelSubsystem.setDefaultRPM(),
                m_feederSubsystem.stop())
                .withName("SHTR - Stop Shooting");
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putBoolean("isHoodReady", m_hoodSubsystem.isAtTargetAngle());
        SmartDashboard.putBoolean("isFlywheelReady", m_flywheelSubsystem.isAtTargetRPM());
    }
}
