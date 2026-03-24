// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.ShooterZone;
import frc.robot.Robot;
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
        // TODO: Remove this one hood mech simulation is fixed
        if (Robot.isSimulation())
            return m_flywheelSubsystem.isAtTargetRPM();

        return m_hoodSubsystem.isAtTargetAngle() &&
                m_flywheelSubsystem.isAtTargetRPM();
    }

    public boolean isShooterReady(boolean isFeeding) {
        // TODO: Remove this one hood mech simulation is fixed
        if (Robot.isSimulation())
            return m_flywheelSubsystem.isAtTargetRPM(isFeeding);

        return m_hoodSubsystem.isAtTargetAngle(isFeeding) &&
                m_flywheelSubsystem.isAtTargetRPM(isFeeding);
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
    // public Command aimAndShoot(Supplier<Distance> getDistanceToTarget,
    // Supplier<Boolean> isAutoAimReady) {
    // return Commands.parallel(
    // m_hoodSubsystem.setAngle(() -> {
    // Distance distance = getDistanceToTarget.get();
    // ShooterZone zone = m_hoodSubsystem.getSpeedZone(distance);
    // return m_hoodSubsystem.getAngleToTarget(distance, zone);
    // }),
    // m_flywheelSubsystem.setSpeed(() ->
    // m_flywheelSubsystem.getTargetVelocity(getDistanceToTarget.get())),
    // new ConditionalCommand(
    // m_feederSubsystem.feed(),
    // m_feederSubsystem.stop(),
    // () -> isAutoAimReady.get() && isShooterReady()).repeatedly())
    // .withName("SHTR - Aim and Shoot");
    // }

    public Command aimAndShoot(Supplier<Distance> getDistanceToTarget, Supplier<Boolean> isAutoAimReady,
            boolean stationaryShooting, Supplier<Boolean> isFeeding) {
        return Commands.parallel(
                m_hoodSubsystem.setAngle(() -> {
                    Distance distance = getDistanceToTarget.get();
                    ShooterZone zone = m_hoodSubsystem.getSpeedZone(distance);
                    return m_hoodSubsystem.getAngleToTarget(distance, zone);
                }),
                m_flywheelSubsystem.setSpeed(() -> m_flywheelSubsystem.getTargetVelocity(getDistanceToTarget.get())),
                stationaryShooting ? Commands.sequence(
                        Commands.waitSeconds(0.25),
                        Commands.waitUntil(() -> isAutoAimReady.get() && isShooterReady(isFeeding.get()))
                                .andThen(m_feederSubsystem.feed()))
                        : new ConditionalCommand(
                                m_feederSubsystem.feed(),
                                m_feederSubsystem.stop(),
                                () -> isAutoAimReady.get() && isShooterReady(isFeeding.get())).repeatedly())
                .withName("SHTR - Aim and Shoot Stationary");
    }

    public Command aimAndShootIgnoreCheck(Supplier<Distance> getDistanceToTarget, Time delayBeforeShooting) {
        return Commands.parallel(
                m_hoodSubsystem.setAngle(() -> {
                    Distance distance = getDistanceToTarget.get();
                    ShooterZone zone = m_hoodSubsystem.getSpeedZone(distance);
                    return m_hoodSubsystem.getAngleToTarget(distance, zone);
                }),
                m_flywheelSubsystem.setSpeed(() -> m_flywheelSubsystem.getTargetVelocity(getDistanceToTarget.get())),
                new WaitCommand(delayBeforeShooting).andThen(
                        m_feederSubsystem.feed()))
                .withName("SHTR - Aim and Shoot");
    }

    public Command aimAndShootIgnoreCheck(Supplier<Distance> getDistanceToTarget) {
        return aimAndShootIgnoreCheck(getDistanceToTarget, Seconds.of(1));
    }

    // TODO: What if we get pushed while we're auto-aiming? This may 'cause
    // isAutoAimReady to never be true. Maybe lock swerve pose?

    public Command shootNoAutoAim() {
        return Commands.parallel(
                m_hoodSubsystem.setDefaultAngle(),
                m_flywheelSubsystem.setDefaultRPM(),
                Commands.waitSeconds(1).andThen(m_feederSubsystem.feed()));
    }

    public Command shootWith(Angle angle, AngularVelocity RPM) {
        return Commands.parallel(
                m_hoodSubsystem.setAngle(angle),
                m_flywheelSubsystem.setSpeed(RPM),
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
        return Commands.sequence(
                m_feederSubsystem.stop(),
                Commands.deadline(
                        Commands.waitSeconds(0.5),
                        m_feederSubsystem.reverse()),
                new ParallelCommandGroup(
                        m_hoodSubsystem.lowerHood(),
                        m_flywheelSubsystem.setDefaultRPM(),
                        m_feederSubsystem.stop()).withName("SHTR - Stop Shooting"));
    }

    /**
     * Stops the shooting process by lowering the hood, setting the flywheel to its
     * default RPM, and optionally stopping the feeder.
     * 
     * @param stopFeeder whether to stop the feeder
     * @return a Command that stops the shooting process when executed
     */
    public Command stopShooting(boolean stopFeeder, boolean stopFlywheel) {
        return Commands.parallel(
                m_hoodSubsystem.lowerHood(),
                stopFeeder ? m_feederSubsystem.stop() : Commands.none(),
                stopFlywheel ? m_flywheelSubsystem.stop() : m_flywheelSubsystem.setDefaultRPM())
                .withName("SHTR - Stop Shooting");
    }

    public Command stopFeeder() {
        return m_feederSubsystem.stop();
    }

    /**
     * Feeds fuel into the shooter until fuel is detected by the beam break sensor,
     * then reverses the feeder until the fuel is no longer detected, effectively
     * positioning fuel correctly in the feeder for shooting.
     * 
     * @return a Command that performs the fuel storing sequence when executed
     */
    public Command storeFuel() {

        // TODO: Remove this when beam break sensor is working
        return Commands.none();

        // return m_feederSubsystem.feed()
        // .until(m_feederSubsystem::isBeamBroken)
        // .finallyDo(interrupted -> m_feederSubsystem.reverse() // TODO: Check
        // behavious if interrupted while
        // // feeding
        // .until(() -> !m_feederSubsystem.isBeamBroken()));
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

    public Command feederSysId() {
        return m_feederSubsystem.sysId();
    }

    // TODO: isShooterAlmostEmpty()

    // Simulation
    public LinearVelocity getFlywheelLinearVelocity() {
        return m_flywheelSubsystem.getLinearVelocity();
    }

    public Angle getHoodAngle() {
        return m_hoodSubsystem.getAngle();
    }

    public Angle getHoodSetpointAngle() {
        // TODO: Change `orElse()` statement default
        return m_hoodSubsystem.getAngleSetpoint().orElse(Degrees.of(0));
    }

    @Override
    public void periodic() {
        if (Constants.TELEMETRY && !DriverStation.isFMSAttached()) {
            SmartDashboard.putBoolean("isHoodReady", m_hoodSubsystem.isAtTargetAngle());
            SmartDashboard.putBoolean("isFlywheelReady", m_flywheelSubsystem.isAtTargetRPM());
            SmartDashboard.putBoolean("isHoodReady (feeding)", m_hoodSubsystem.isAtTargetAngle(true));
            SmartDashboard.putBoolean("isFlywheelReady (feeding)", m_flywheelSubsystem.isAtTargetRPM(true));
        }
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putBoolean("isHoodReady", m_hoodSubsystem.isAtTargetAngle());
        SmartDashboard.putBoolean("isFlywheelReady", m_flywheelSubsystem.isAtTargetRPM());
    }
}
