// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

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
        m_flywheelSubsystem.setDefaultCommand(m_flywheelSubsystem.setDefaultRPM());
    }

    /**
     * Checks if the shooter is ready to shoot by verifying that the hood is at the
     * correct angle and the flywheel is at the target RPM.
     * 
     * @return true if the shooter is ready to shoot, false otherwise
     */
    public boolean isShooterReady() {
        return m_hoodSubsystem.isAtAngle() && m_flywheelSubsystem.isAtTargetRPM();
    }

    /**
     * Aims the shooter by adjusting the hood angle based on the distance to the
     * target,
     * and then shoots by spinning up the flywheel and feeding balls into it.
     *
     * @param distanceToTarget the distance from the robot to the target, used to
     *                         determine the appropriate hood angle
     * @return a Command that performs the aiming and shooting sequence when
     *         executed
     */
    public Command aimAndShoot(Supplier<Distance> getDistanceToTarget) {
        // return Commands.parallel(

        // // Spin up flywheel and adjust hood angle in parallel, then feed balls when
        // // ready
        // m_flywheelSubsystem.shoot(),
        // m_hoodSubsystem.setAngle(Degrees.of(m_distanceToHoodAngleMap.get(getDistanceToTarget.get().in(Meters))))
        // .repeatedly(),
        // new ConditionalCommand(m_feederSubsystem.feed(), m_feederSubsystem.stop(),
        // this::isShooterReady)
        // .repeatedly()

        // // Slow flywheel, lower hood, and stop feeder when we stop shooting
        // ).finallyDo((interrupted) -> {
        // m_flywheelSubsystem.setDefaultRPM();
        // m_hoodSubsystem.lowerHood();
        // m_feederSubsystem.stop();
        // }) // TODO: Check if this requires .schedule(), or a Commands.parallel(), or
        // // if it will run automatically after the parallel command finishes
        // .withName("Aim and Shoot");

        return m_hoodSubsystem.setAngle(
                () -> {
                    return Degrees.of(m_distanceToHoodAngleMap
                            .get(getDistanceToTarget.get().in(Meters)));
                });

    }

    public Command test() {
        return m_flywheelSubsystem.test();
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putNumber("m_distanceToHoodAngleMap", m_distanceToHoodAngleMap.get(4.0));
    }
}
