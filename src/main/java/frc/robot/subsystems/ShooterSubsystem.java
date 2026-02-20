// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.FeederSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;

public class ShooterSubsystem extends SubsystemBase {

    private final FeederSubsystem m_feederSubsystem;
    private final FlywheelSubsystem m_flywheelSubsystem;
    private final HoodSubsystem m_hoodSubsystem;

    public ShooterSubsystem() {
        m_feederSubsystem = new FeederSubsystem();
        m_flywheelSubsystem = new FlywheelSubsystem();
        m_hoodSubsystem = new HoodSubsystem();
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
                            return m_hoodSubsystem.getAngleToTarget(getDistanceToTarget.get());
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
    // TODO: What if we get pushed while we're auto-aiming? This may 'cause
    // isAutoAimReady to never be true. Maybe lock swerve pose?

    public Command shootNoAutoAim() {
        return Commands.parallel(
                m_hoodSubsystem.setDefaultAngle(),
                m_flywheelSubsystem.shoot(),
                new ConditionalCommand(m_feederSubsystem.feed(), m_feederSubsystem.stop(), this::isShooterReady)
                        .repeatedly());
    }

    /**
     * Stops the shooting process by lowering the hood, setting the flywheel to its
     * default RPM, and stopping the feeder.
     * 
     * @return a Command that stops the shooting process when executed
     */
    public Command stopShooting() {
        return Commands.parallel(
                m_hoodSubsystem.lowerHood(),
                m_flywheelSubsystem.setDefaultRPM(),
                m_feederSubsystem.stop())
                .withName("SHTR - Stop Shooting");
    }

    /**
     * Stops the shooting process by lowering the hood, setting the flywheel to its
     * default RPM, and optionally stopping the feeder.
     * 
     * @param stopFeeder whether to stop the feeder
     * @return a Command that stops the shooting process when executed
     */
    public Command stopShooting(boolean stopFeeder) {
        return Commands.parallel(
                m_hoodSubsystem.lowerHood(),
                m_flywheelSubsystem.setDefaultRPM(),
                stopFeeder ? m_feederSubsystem.stop() : Commands.none())
                .withName("SHTR - Stop Shooting");
    }

    /**
     * Feeds fuel into the shooter until fuel is detected by the beam break sensor,
     * then reverses the feeder until the fuel is no longer detected, effectively
     * positioning fuel correctly in the feeder for shooting.
     * 
     * @return a Command that performs the fuel storing sequence when executed
     */
    public Command storeFuel() {
        return m_feederSubsystem.feed()
                .until(m_feederSubsystem::isBeamBroken)
                .finallyDo(interrupted -> m_feederSubsystem.reverse() // TODO: Check behavious if interrupted while
                                                                      // feeding
                        .until(() -> !m_feederSubsystem.isBeamBroken()));
    }

    /**
     * Starts the flywheel spinning at its default RPM, the speed at which it should
     * spin when the shooter is not actively shooting
     * 
     * @return a Command that starts the flywheel at its default RPM when executed
     */
    public Command startFlywheelDefaultRPM() {
        return m_flywheelSubsystem.setDefaultRPM();
    }

    public Command flywheelSysId() {
        return m_flywheelSubsystem.sysId();
    }

    public Command hoodSysId() {
        return m_hoodSubsystem.sysId();
    }

    // TODO: isShooterAlmostEmpty()

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putBoolean("isHoodReady", m_hoodSubsystem.isAtTargetAngle());
        SmartDashboard.putBoolean("isFlywheelReady", m_flywheelSubsystem.isAtTargetRPM());
    }
}
