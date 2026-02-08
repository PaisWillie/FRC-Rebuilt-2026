// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * Subsystem for controlling the intake motor.
 */
public class IntakeSubsystem extends SubsystemBase {

    /** PWM motor controller for the intake. */
    private final PWMTalonFX m_rollerMotor;

    /**
     * Constructs the intake subsystem and initializes the motor controller.
     */
    public IntakeSubsystem() {
        // Initialize motor controller using configured PWM ID.
        m_rollerMotor = new PWMTalonFX(IntakeConstants.MOTOR_PWM_ID);
    }

    /**
     * Sets the intake motor speed.
     *
     * @param speed the desired motor output (-1.0 to 1.0)
     */
    public void setSpeed(double speed) {
        m_rollerMotor.set(speed);
    }

    /**
     * Stops the intake motor.
     */
    public void stop() {
        m_rollerMotor.stopMotor();
    }

    @Override
    public void periodic() {

    }
}
